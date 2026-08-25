#include <strikeengine/kernel/systems/SeekerSystem.hpp>
#include <cmath>
#include <numbers>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace StrikeEngine::Kernel {

    // Helper for quaternion rotation
    glm::dvec3 rotateVector(const glm::dquat& q, const glm::dvec3& v) {
        return q * v;
    }

    void SeekerSystem::update(
        const PhysicsBlock& physics,
        const EntityStatusBlock& status,
        SeekerBlock& seeker,
        double dt)
    {
        std::size_t n = seeker.size;
        for (std::size_t i = 0; i < n; ++i) {
            if (!physics.active[i] || !status.isAlive[i] || seeker.type[i] == SeekerType::None) {
                seeker.isLocked[i] = false;
                continue;
            }

            bool lockMaintained = false;

            glm::dvec3 seekerPos(physics.px[i], physics.py[i], physics.pz[i]);
            glm::dvec3 seekerVel(physics.vx[i], physics.vy[i], physics.vz[i]);

            // Iterate through all potential targets
            for (std::size_t t = 0; t < physics.size; ++t) {
                if (t == i || !physics.active[t] || !status.isAlive[t]) continue;
                if (status.allegiance[i] == status.allegiance[t]) continue; // Don't lock onto friendlies

                glm::dvec3 targetPos(physics.px[t], physics.py[t], physics.pz[t]);
                glm::dvec3 targetVel(physics.vx[t], physics.vy[t], physics.vz[t]);

                glm::dvec3 rangeVec = targetPos - seekerPos;
                double range = glm::length(rangeVec);

                // Target relative velocity
                glm::dvec3 relVel = targetVel - seekerVel;
                double rangeRate = glm::dot(glm::normalize(rangeVec), relVel);

                // Transform LOS into target's local frame
                glm::dquat targetQ(physics.qw[t], physics.qx[t], physics.qy[t], physics.qz[t]);
                glm::dvec3 losInTargetFrame = rotateVector(glm::inverse(targetQ), glm::normalize(rangeVec));

                double azimuthRad = std::atan2(losInTargetFrame.y, losInTargetFrame.x);
                double elevationRad = std::asin(-losInTargetFrame.z);

                if (seeker.type[i] == SeekerType::RF) {
                    const std::string& profileId = status.rcsProfileId[t];
                    if (profileId.empty()) continue; // Skip targets with no signature

                    if (!rcsCache.contains(profileId)) {
                        auto db = std::make_unique<Models::RCSDatabase>();
                        if (db->loadProfile(profileId)) rcsCache[profileId] = std::move(db);
                        else continue;
                    }

                    double rcs_m2 = rcsCache.at(profileId)->getRCS(azimuthRad, elevationRad);

                    double pt = seeker.transmitterPowerW[i];
                    double gain = std::pow(10.0, seeker.antennaGainDb[i] / 10.0);
                    double lambda = seeker.wavelengthM[i];

                    double receivedPower = (pt * gain * gain * lambda * lambda * rcs_m2) /
                                           (std::pow(4.0 * std::numbers::pi, 3.0) * std::pow(range, 4.0));

                    double snrDb = 10.0 * std::log10(receivedPower / seeker.noiseFloorW[i]);

                    if (snrDb > seeker.snrThresholdDb[i]) {
                        seeker.isLocked[i] = true;
                        seeker.lockedTargetId[i] = t;
                        seeker.targetRange[i] = range;
                        seeker.targetRangeRate[i] = rangeRate;
                        
                        // Body-frame LOS angles (for guidance)
                        glm::dquat seekerQ(physics.qw[i], physics.qx[i], physics.qy[i], physics.qz[i]);
                        glm::dvec3 losInSeekerFrame = rotateVector(glm::inverse(seekerQ), glm::normalize(rangeVec));
                        seeker.targetAzimuth[i] = std::atan2(losInSeekerFrame.y, losInSeekerFrame.x);
                        seeker.targetElevation[i] = std::asin(-losInSeekerFrame.z);

                        lockMaintained = true;
                        break;
                    }
                } 
                else if (seeker.type[i] == SeekerType::IR) {
                    const std::string& profileId = status.irProfileId[t];
                    if (profileId.empty()) continue;

                    if (!irCache.contains(profileId)) {
                        auto db = std::make_unique<Models::IRSignatureDatabase>();
                        if (db->loadProfile(profileId)) irCache[profileId] = std::move(db);
                        else continue;
                    }

                    double radiantIntensity = irCache.at(profileId)->getRadiantIntensity(azimuthRad, elevationRad);
                    double irradiance = radiantIntensity / (range * range);

                    // Basic Transmissivity Placeholder
                    double extinctionCoefficient = 0.1; // km^-1
                    double transmissivity = std::exp(-extinctionCoefficient * (range / 1000.0));

                    double finalPowerW = irradiance * transmissivity;

                    if (finalPowerW > seeker.sensitivityW[i]) {
                        seeker.isLocked[i] = true;
                        seeker.lockedTargetId[i] = t;
                        seeker.targetRange[i] = range;
                        seeker.targetRangeRate[i] = rangeRate;

                        glm::dquat seekerQ(physics.qw[i], physics.qx[i], physics.qy[i], physics.qz[i]);
                        glm::dvec3 losInSeekerFrame = rotateVector(glm::inverse(seekerQ), glm::normalize(rangeVec));
                        seeker.targetAzimuth[i] = std::atan2(losInSeekerFrame.y, losInSeekerFrame.x);
                        seeker.targetElevation[i] = std::asin(-losInSeekerFrame.z);

                        lockMaintained = true;
                        break;
                    }
                }
            }

            if (!lockMaintained) {
                seeker.isLocked[i] = false;
            }
        }
    }

} // namespace StrikeEngine::Kernel
