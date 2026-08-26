#include <strikeengine/kernel/systems/SeekerSystem.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace StrikeEngine::Kernel {

    // Helper for quaternion rotation
    glm::dvec3 rotateVector(const glm::dquat& q, const glm::dvec3& v) {
        return q * v;
    }

    void SeekerSystem::reset()
    {
        measurementHistory.clear();
        historyEntityCount = 0;
    }

    void SeekerSystem::update(
        const PhysicsBlock& physics,
        const EntityStatusBlock& status,
        SeekerBlock& seeker,
        double dt)
    {
        const std::size_t n = std::min({seeker.size, physics.size, status.size});
        const double stepDt = std::max(0.0, dt);
        if (historyEntityCount != seeker.size) {
            measurementHistory.clear();
            measurementHistory.resize(n);
            historyEntityCount = seeker.size;
        } else if (measurementHistory.size() < n) {
            measurementHistory.resize(n);
        }

        for (std::size_t i = 0; i < n; ++i) {
            if (!physics.active[i] || !status.isAlive[i] || seeker.type[i] == SeekerType::None) {
                measurementHistory[i].clear();
                seeker.isLocked[i] = false;
                seeker.lockLostTimeSec[i] = 0.0;
                seeker.hasPreviousLos[i] = false;
                seeker.targetAzimuthRate[i] = 0.0;
                seeker.targetElevationRate[i] = 0.0;
                continue;
            }

            glm::dvec3 seekerPos(physics.px[i], physics.py[i], physics.pz[i]);
            glm::dvec3 seekerVel(physics.vx[i], physics.vy[i], physics.vz[i]);
            for (auto& measurement : measurementHistory[i]) measurement.age += stepDt;

            auto publishAvailable = [&]() {
                auto& history = measurementHistory[i];
                const double latency = std::max(0.0, seeker.measurementLatencySec[i]);
                while (!history.empty() && history.front().age + 1e-12 >= latency) {
                    const DelayedMeasurement measurement = history.front();
                    history.pop_front();
                    seeker.targetRange[i] = measurement.range;
                    seeker.targetRangeRate[i] = measurement.rangeRate;
                    seeker.targetAzimuth[i] = measurement.azimuth;
                    seeker.targetElevation[i] = measurement.elevation;
                    seeker.targetAzimuthRate[i] = measurement.azimuthRate;
                    seeker.targetElevationRate[i] = measurement.elevationRate;
                }
            };

            struct Candidate {
                bool geometryValid = false;
                bool acquisitionValid = false;
                bool maintenanceValid = false;
                double range = 0.0;
                double rangeRate = 0.0;
                double azimuth = 0.0;
                double elevation = 0.0;
            };

            auto evaluateTarget = [&](std::size_t target) {
                Candidate candidate;
                glm::dvec3 targetPos(physics.px[target], physics.py[target], physics.pz[target]);
                glm::dvec3 targetVel(physics.vx[target], physics.vy[target], physics.vz[target]);
                glm::dvec3 rangeVec = targetPos - seekerPos;
                candidate.range = glm::length(rangeVec);
                if (candidate.range <= 1e-9) return candidate;

                const glm::dvec3 losWorld = rangeVec / candidate.range;
                glm::dvec3 relVel = targetVel - seekerVel;
                candidate.rangeRate = glm::dot(losWorld, relVel);

                glm::dquat seekerQ(physics.qw[i], physics.qx[i], physics.qy[i], physics.qz[i]);
                const glm::dvec3 losInSeekerFrame =
                    rotateVector(glm::inverse(seekerQ), losWorld);
                candidate.azimuth = std::atan2(losInSeekerFrame.y, losInSeekerFrame.x);
                candidate.elevation = std::asin(std::clamp(-losInSeekerFrame.z, -1.0, 1.0));

                const double fov = std::max(0.0, seeker.fieldOfViewHalfAngleRad[i]);
                const double gimbalAz = std::max(0.0, seeker.gimbalAzimuthLimitRad[i]);
                const double gimbalEl = std::max(0.0, seeker.gimbalElevationLimitRad[i]);
                const double offBoresight = std::acos(
                    std::clamp(losInSeekerFrame.x, -1.0, 1.0));
                candidate.geometryValid =
                    offBoresight <= fov &&
                    std::abs(candidate.azimuth) <= gimbalAz &&
                    std::abs(candidate.elevation) <= gimbalEl;
                if (!candidate.geometryValid) return candidate;

                // Target aspect is measured in the target frame for signature
                // lookup; seeker geometry above is measured in the seeker frame.
                glm::dquat targetQ(physics.qw[target], physics.qx[target],
                                   physics.qy[target], physics.qz[target]);
                const glm::dvec3 losInTargetFrame =
                    rotateVector(glm::inverse(targetQ), losWorld);
                const double targetAzimuth = std::atan2(
                    losInTargetFrame.y, losInTargetFrame.x);
                const double targetElevation = std::asin(
                    std::clamp(-losInTargetFrame.z, -1.0, 1.0));
                const double hysteresisDb = std::max(0.0, seeker.lockHysteresisDb[i]);

                if (seeker.type[i] == SeekerType::RF) {
                    const std::string& profileId = status.rcsProfileId[target];
                    if (profileId.empty()) return candidate;

                    if (!rcsCache.contains(profileId)) {
                        auto db = std::make_unique<Models::RCSDatabase>();
                        if (db->loadProfile(profileId)) rcsCache[profileId] = std::move(db);
                        else return candidate;
                    }

                    const double rcsM2 = rcsCache.at(profileId)->getRCS(
                        targetAzimuth, targetElevation);
                    const double pt = std::max(0.0, seeker.transmitterPowerW[i]);
                    const double gain = std::pow(10.0, seeker.antennaGainDb[i] / 10.0);
                    const double lambda = std::max(0.0, seeker.wavelengthM[i]);
                    const double noise = std::max(
                        seeker.noiseFloorW[i], std::numeric_limits<double>::min());
                    const double receivedPower = (pt * gain * gain * lambda * lambda * rcsM2) /
                        (std::pow(4.0 * std::numbers::pi, 3.0) *
                         std::pow(candidate.range, 4.0));
                    const double snrDb = 10.0 * std::log10(
                        std::max(receivedPower, std::numeric_limits<double>::min()) / noise);
                    candidate.acquisitionValid = snrDb > seeker.snrThresholdDb[i];
                    candidate.maintenanceValid = snrDb >
                        seeker.snrThresholdDb[i] - hysteresisDb;
                } else if (seeker.type[i] == SeekerType::IR) {
                    const std::string& profileId = status.irProfileId[target];
                    if (profileId.empty()) return candidate;

                    if (!irCache.contains(profileId)) {
                        auto db = std::make_unique<Models::IRSignatureDatabase>();
                        if (db->loadProfile(profileId)) irCache[profileId] = std::move(db);
                        else return candidate;
                    }

                    const double radiantIntensity = irCache.at(profileId)->getRadiantIntensity(
                        targetAzimuth, targetElevation);
                    const double irradiance = radiantIntensity /
                        (candidate.range * candidate.range);
                    const double transmissivity = std::exp(-0.1 * (candidate.range / 1000.0));
                    const double finalPower = irradiance * transmissivity;
                    const double sensitivity = std::max(
                        seeker.sensitivityW[i], std::numeric_limits<double>::min());
                    const double maintenanceSensitivity = sensitivity /
                        std::pow(10.0, hysteresisDb / 10.0);
                    candidate.acquisitionValid = finalPower > sensitivity;
                    candidate.maintenanceValid = finalPower > maintenanceSensitivity;
                }

                return candidate;
            };

            auto commitTrack = [&](std::size_t target, const Candidate& candidate) {
                const bool sameTrack = seeker.isLocked[i] &&
                    seeker.lockedTargetId[i] == target && seeker.hasPreviousLos[i];
                if (sameTrack && stepDt > 0.0) {
                    constexpr double filterTau = 0.05;
                    const double rawAzRate = std::remainder(
                        candidate.azimuth - seeker.previousAzimuth[i],
                        2.0 * std::numbers::pi) / stepDt;
                    const double rawElRate =
                        (candidate.elevation - seeker.previousElevation[i]) / stepDt;
                    const double blend = 1.0 - std::exp(-stepDt / filterTau);
                    seeker.targetAzimuthRate[i] += blend *
                        (rawAzRate - seeker.targetAzimuthRate[i]);
                    seeker.targetElevationRate[i] += blend *
                        (rawElRate - seeker.targetElevationRate[i]);
                } else {
                    seeker.targetAzimuthRate[i] = 0.0;
                    seeker.targetElevationRate[i] = 0.0;
                }
                seeker.isLocked[i] = true;
                seeker.lockedTargetId[i] = target;
                seeker.previousAzimuth[i] = candidate.azimuth;
                seeker.previousElevation[i] = candidate.elevation;
                seeker.hasPreviousLos[i] = true;
                seeker.lockLostTimeSec[i] = 0.0;
                measurementHistory[i].push_back({
                    target, 0.0, candidate.range, candidate.rangeRate,
                    candidate.azimuth, candidate.elevation,
                    seeker.targetAzimuthRate[i], seeker.targetElevationRate[i]});
            };

            const bool hadLock = seeker.isLocked[i];
            if (hadLock && seeker.lockedTargetId[i] < physics.size) {
                const std::size_t target = seeker.lockedTargetId[i];
                if (target != i && physics.active[target] && status.isAlive[target] &&
                    status.allegiance[i] != status.allegiance[target]) {
                    const Candidate candidate = evaluateTarget(target);
                    if (candidate.geometryValid && candidate.maintenanceValid) {
                        commitTrack(target, candidate);
                        publishAvailable();
                        continue;
                    }
                    if (candidate.geometryValid) {
                        seeker.lockLostTimeSec[i] += stepDt;
                        const double dropout = std::max(0.0, seeker.lockDropoutTimeSec[i]);
                        if (seeker.lockLostTimeSec[i] <= dropout) {
                            publishAvailable();
                            continue;
                        }
                    }
                }
            }

            seeker.isLocked[i] = false;
            measurementHistory[i].clear();
            seeker.lockLostTimeSec[i] = 0.0;
            seeker.hasPreviousLos[i] = false;
            seeker.targetAzimuthRate[i] = 0.0;
            seeker.targetElevationRate[i] = 0.0;

            // Iterate through all potential targets
            for (std::size_t t = 0; t < physics.size; ++t) {
                if (t == i || !physics.active[t] || !status.isAlive[t]) continue;
                if (status.allegiance[i] == status.allegiance[t]) continue; // Don't lock onto friendlies
                const Candidate candidate = evaluateTarget(t);
                if (candidate.acquisitionValid) {
                    commitTrack(t, candidate);
                    break;
                }
            }
            publishAvailable();
        }
    }

} // namespace StrikeEngine::Kernel
