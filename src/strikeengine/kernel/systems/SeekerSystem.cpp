#include <strikeengine/kernel/systems/SeekerSystem.hpp>
#include <strikeengine/models/physics/earth/EarthFrames.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace StrikeEngine::Kernel {

    // Helper for quaternion rotation
    glm::dvec3 rotateVector(const glm::dquat& q, const glm::dvec3& v) {
        return q * v;
    }

namespace {

// Defensive SeekerBlock reads: direct unit tests hand-build blocks without the
// newer config/state arrays, so every post-legacy read defaults instead of
// running off the end. Kernel-driven runs always wire full arrays.
bool flagAt(const std::vector<bool>& v, std::size_t i)
{
    return i < v.size() && v[i];
}

double valAt(const std::vector<double>& v, std::size_t i, double def)
{
    return i < v.size() ? v[i] : def;
}

int intAt(const std::vector<int>& v, std::size_t i, int def)
{
    return i < v.size() ? v[i] : def;
}

// Grow only the newly-added state arrays so hand-built blocks stay safe.
void ensureState(SeekerBlock& s, std::size_t n)
{
    auto rbool = [&](std::vector<bool>& v, bool d) { if (v.size() < n) v.resize(n, d); };
    auto rdouble = [&](std::vector<double>& v, double d) { if (v.size() < n) v.resize(n, d); };
    auto rint = [&](std::vector<int>& v, int d) { if (v.size() < n) v.resize(n, d); };
    rbool(s.lockActive, false);
    rbool(s.hasPublishedMeasurement, false);
    rdouble(s.measurementAgeSec, 0.0);
    rdouble(s.gimbalAzimuthRad, 0.0);
    rdouble(s.gimbalElevationRad, 0.0);
    rdouble(s.lastSignalStrength, 0.0);
    rint(s.lockRejectReason, 0);
    rdouble(s.glintAzM, 0.0);
    rdouble(s.glintElM, 0.0);
}

double wrapPi(double angle)
{
    return std::remainder(angle, 2.0 * std::numbers::pi);
}

// Move `current` toward `desired` by at most `maxStep` radians.
double slewToward(double current, double desired, double maxStep)
{
    const double delta = wrapPi(desired - current);
    if (std::abs(delta) <= maxStep) return desired;
    return current + (delta > 0.0 ? maxStep : -maxStep);
}

// Coarse sampled line-of-sight terrain check. Returns true only when a sample
// point is decisively below terrain; unknown/no-data samples never block.
bool terrainBlocks(const glm::dvec3& from, const glm::dvec3& to,
                   const EnvironmentConfig& environment)
{
    if (!environment.globalTerrain && !environment.terrainElevation) return false;
    if (!environment.globalTerrain) {
        // terrainElevation is only meaningful in local mode.
        if (environment.earth.useEcefTruth) return false;
    }
    constexpr int kSamples = 12;
    for (int k = 1; k < kSamples; ++k) {
        const double t = static_cast<double>(k) / kSamples;
        const double x = from.x + (to.x - from.x) * t;
        const double y = from.y + (to.y - from.y) * t;
        const double z = from.z + (to.z - from.z) * t;
        if (environment.earth.useEcefTruth) {
            const auto geo = Models::ecefToGeodetic({x, y, z});
            double elevation = 0.0;
            bool known = false;
            if (environment.globalTerrain) {
                const auto sample = environment.globalTerrain->sample(
                    geo.latitudeRad, geo.longitudeRad);
                if (sample.hasElevation()) { elevation = sample.elevationM; known = true; }
            } else {
                elevation = environment.terrainElevation(x, y);
                known = true;
            }
            if (known && geo.altitudeM < elevation) return true;
        } else {
            double elevation = 0.0;
            bool known = false;
            if (environment.globalTerrain) {
                const Models::GeodeticCoordinate reference{
                    environment.earth.referenceLatitudeRad,
                    environment.earth.referenceLongitudeRad, 0.0};
                const auto geo = Models::EarthFrames::enuToGeodetic({x, y, z}, reference);
                const auto sample = environment.globalTerrain->sample(
                    geo.latitudeRad, geo.longitudeRad);
                if (sample.hasElevation()) { elevation = sample.elevationM; known = true; }
            } else {
                elevation = environment.terrainElevation(x, y);
                known = true;
            }
            if (known && z < elevation) return true;
        }
    }
    return false;
}

} // namespace

    void SeekerSystem::setSeed(std::uint32_t seed) {
        // Measurement-noise / Swerling / emitter-duty stream. Disjoint from
        // sensor, nav-alignment and warhead streams.
        rng.seed(seed ^ 0x5EE7E9u);
    }

    void SeekerSystem::reset()
    {
        measurementHistory.clear();
        historyEntityCount = 0;
        // Retry failed profile loads on the next scenario (a missing file may
        // have been restored); successful cache entries persist by design.
        rcsLoadFailed.clear();
    }

    void SeekerSystem::update(
        const PhysicsBlock& physics,
        const EntityStatusBlock& status,
        SeekerBlock& seeker,
        double dt,
        const EnvironmentConfig& environment)
    {
        const std::size_t n = std::min({seeker.size, physics.size, status.size});
        const double stepDt = std::max(0.0, dt);
        ensureState(seeker, n);
        if (historyEntityCount != seeker.size) {
            measurementHistory.clear();
            measurementHistory.resize(n);
            historyEntityCount = seeker.size;
        } else if (measurementHistory.size() < n) {
            measurementHistory.resize(n);
        }

        std::normal_distribution<double> stdNorm(0.0, 1.0);
        std::uniform_real_distribution<double> stdUniform(0.0, 1.0);

        for (std::size_t i = 0; i < n; ++i) {
            const auto setReason = [&](SeekerRejectReason reason) {
                seeker.lockRejectReason[i] = static_cast<int>(reason);
            };
            const auto refreshLockState = [&]() {
                seeker.isLocked[i] =
                    seeker.lockActive[i] && seeker.hasPublishedMeasurement[i];
            };
            const auto clearTrack = [&]() {
                measurementHistory[i].clear();
                seeker.isLocked[i] = false;
                seeker.lockActive[i] = false;
                seeker.hasPublishedMeasurement[i] = false;
                seeker.measurementAgeSec[i] = 0.0;
                seeker.lockLostTimeSec[i] = 0.0;
                seeker.hasPreviousLos[i] = false;
                seeker.targetAzimuthRate[i] = 0.0;
                seeker.targetElevationRate[i] = 0.0;
                seeker.glintAzM[i] = 0.0;
                seeker.glintElM[i] = 0.0;
            };

            if (!physics.active[i] || !status.isAlive[i] ||
                seeker.type[i] == SeekerType::None) {
                clearTrack();
                setReason(SeekerRejectReason::NoSeeker);
                continue;
            }

            glm::dvec3 seekerPos(physics.px[i], physics.py[i], physics.pz[i]);
            glm::dvec3 seekerVel(physics.vx[i], physics.vy[i], physics.vz[i]);
            seeker.measurementAgeSec[i] += stepDt;
            for (auto& measurement : measurementHistory[i]) measurement.age += stepDt;

            auto publishAvailable = [&]() {
                auto& history = measurementHistory[i];
                const double latency = std::max(
                    0.0, valAt(seeker.measurementLatencySec, i, 0.0));
                while (!history.empty() && history.front().age + 1e-12 >= latency) {
                    const DelayedMeasurement measurement = history.front();
                    history.pop_front();
                    seeker.targetRange[i] = measurement.range;
                    seeker.targetRangeRate[i] = measurement.rangeRate;
                    seeker.targetAzimuth[i] = measurement.azimuth;
                    seeker.targetElevation[i] = measurement.elevation;
                    seeker.targetAzimuthRate[i] = measurement.azimuthRate;
                    seeker.targetElevationRate[i] = measurement.elevationRate;
                    seeker.hasPublishedMeasurement[i] = true;
                    seeker.measurementAgeSec[i] = 0.0;
                }
            };

            struct Candidate {
                bool geometryValid = false;
                bool rangeGateValid = true;
                bool terrainClear = true;
                bool acquisitionValid = false;
                bool maintenanceValid = false;
                double signalStrength = 0.0;  // SNR dB (RF family) or W (IR)
                double range = 0.0;
                double rangeRate = 0.0;
                double azimuth = 0.0;         // desired (true) LOS angle
                double elevation = 0.0;
                SeekerRejectReason reject = SeekerRejectReason::None;
            };

            // True LOS angles in the seeker body frame, shared by the geometry
            // gate and the gimbal servo's slew target.
            auto losAngles = [&](std::size_t target, double& range,
                                 double& rangeRate, double& azimuth,
                                 double& elevation, glm::dvec3& losWorld) {
                glm::dvec3 targetPos(physics.px[target], physics.py[target], physics.pz[target]);
                glm::dvec3 targetVel(physics.vx[target], physics.vy[target], physics.vz[target]);
                const glm::dvec3 rangeVec = targetPos - seekerPos;
                range = glm::length(rangeVec);
                if (range <= 1e-9) { rangeRate = 0.0; azimuth = 0.0; elevation = 0.0;
                                     losWorld = glm::dvec3(1, 0, 0); return; }
                losWorld = rangeVec / range;
                rangeRate = glm::dot(losWorld, targetVel - seekerVel);
                const glm::dquat seekerQ(physics.qw[i], physics.qx[i], physics.qy[i], physics.qz[i]);
                const glm::dvec3 losBody = rotateVector(glm::inverse(seekerQ), losWorld);
                azimuth = std::atan2(losBody.y, losBody.x);
                elevation = std::asin(std::clamp(-losBody.z, -1.0, 1.0));
            };

            auto evaluateTarget = [&](std::size_t target, bool useServo,
                                      double servoAz, double servoEl) {
                Candidate candidate;
                glm::dvec3 losWorld;
                losAngles(target, candidate.range, candidate.rangeRate,
                          candidate.azimuth, candidate.elevation, losWorld);
                if (candidate.range <= 1e-9) {
                    candidate.reject = SeekerRejectReason::NoTarget;
                    return candidate;
                }

                const double fov = std::max(0.0, valAt(seeker.fieldOfViewHalfAngleRad, i, 0.0));
                const double gimbalAz = std::max(0.0, valAt(seeker.gimbalAzimuthLimitRad, i, 0.0));
                const double gimbalEl = std::max(0.0, valAt(seeker.gimbalElevationLimitRad, i, 0.0));
                if (useServo) {
                    const double errAz = wrapPi(candidate.azimuth - servoAz);
                    const double errEl = candidate.elevation - servoEl;
                    const double off = std::sqrt(errAz * errAz + errEl * errEl);
                    candidate.geometryValid =
                        off <= fov && std::abs(servoAz) <= gimbalAz &&
                        std::abs(servoEl) <= gimbalEl;
                } else {
                    const glm::dquat seekerQ(physics.qw[i], physics.qx[i], physics.qy[i], physics.qz[i]);
                    const glm::dvec3 losBody = rotateVector(glm::inverse(seekerQ), losWorld);
                    const double offBoresight = std::acos(std::clamp(losBody.x, -1.0, 1.0));
                    candidate.geometryValid =
                        offBoresight <= fov && std::abs(candidate.azimuth) <= gimbalAz &&
                        std::abs(candidate.elevation) <= gimbalEl;
                }
                if (!candidate.geometryValid) {
                    candidate.reject = SeekerRejectReason::Geometry;
                    return candidate;
                }

                const double minRange = valAt(seeker.minRangeGateM, i, 0.0);
                const double maxRange = valAt(seeker.maxRangeGateM, i, 0.0);
                if ((minRange > 0.0 && candidate.range < minRange) ||
                    (maxRange > 0.0 && candidate.range > maxRange)) {
                    candidate.rangeGateValid = false;
                    candidate.reject = SeekerRejectReason::RangeGate;
                    return candidate;
                }

                const double minClosing = valAt(seeker.minClosingRateMps, i, 0.0);
                if (minClosing > 0.0 && candidate.rangeRate > -minClosing) {
                    candidate.reject = SeekerRejectReason::VelocityGate;
                    return candidate;
                }

                if (flagAt(seeker.terrainMaskingEnabled, i)) {
                    glm::dvec3 targetPos(physics.px[target], physics.py[target], physics.pz[target]);
                    if (terrainBlocks(seekerPos, targetPos, environment)) {
                        candidate.terrainClear = false;
                        candidate.reject = SeekerRejectReason::Terrain;
                        return candidate;
                    }
                }

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
                const double hysteresisDb = std::max(
                    0.0, valAt(seeker.lockHysteresisDb, i, 0.0));

                // Countermeasure hook: Chaff/Flare decoys carry the same
                // signature metadata as real targets, but a configured
                // rejection cuts their apparent signal (default 0 = no cut).
                const bool isDecoy = target < status.type.size() &&
                    (status.type[target] == EntityType::Chaff ||
                     status.type[target] == EntityType::Flare) &&
                    valAt(seeker.decoyRejectionDb, i, 0.0) > 0.0;
                const double decoyCutDb = isDecoy
                    ? valAt(seeker.decoyRejectionDb, i, 0.0) : 0.0;

                // RF jammer (assumed co-located with the target): degrades
                // the SINR by S/(1+J/N). No jammer EIRP = unchanged.
                auto applyJammer = [&](double snrDb, double range, double noise) {
                    const double jammerEirp = target < status.jammerEirpW.size()
                        ? status.jammerEirpW[target] : 0.0;
                    if (!(jammerEirp > 0.0) || range <= 1e-9) return snrDb;
                    const double gr = std::pow(
                        10.0, valAt(seeker.antennaGainDb, i, 0.0) / 10.0);
                    const double lambda = std::max(0.0, valAt(seeker.wavelengthM, i, 0.0));
                    const double jPower = jammerEirp * gr * lambda * lambda /
                        (16.0 * std::numbers::pi * std::numbers::pi * range * range);
                    const double jnr = jPower /
                        std::max(noise, std::numeric_limits<double>::min());
                    const double snr = std::pow(10.0, snrDb / 10.0);
                    return 10.0 * std::log10(std::max(
                        snr / (1.0 + jnr), std::numeric_limits<double>::min()));
                };

                if (seeker.type[i] == SeekerType::RF ||
                    seeker.type[i] == SeekerType::SARH) {
                    double rcsM2 = 0.0;
                    // Signature profile load (shared by RF and SARH paths).
                    const std::string& profileId = status.rcsProfileId[target];
                    if (profileId.empty()) {
                        candidate.reject = SeekerRejectReason::NoProfile;
                        return candidate;
                    }
                    if (!rcsCache.contains(profileId)) {
                        auto db = std::make_unique<Models::RCSDatabase>();
                        if (db->loadProfile(profileId)) rcsCache[profileId] = std::move(db);
                        else {
                            if (rcsLoadFailed.insert(profileId).second) {
                                std::fprintf(stderr, "[seek] RF seeker: RCS profile '%s' failed to load; "
                                             "entity %zu will not be acquired (check working directory)\n",
                                             profileId.c_str(), target);
                            }
                            candidate.reject = SeekerRejectReason::NoProfile;
                            return candidate;
                        }
                    }
                    rcsM2 = rcsCache.at(profileId)->getRCS(targetAzimuth, targetElevation);
                    if (flagAt(seeker.swerlingEnabled, i)) {
                        // Swerling-1 (scan-to-scan) power fluctuation: unit-mean
                        // exponential, drawn only when enabled.
                        const double u = std::min(stdUniform(rng), 1.0 - 1e-12);
                        rcsM2 *= -std::log(1.0 - u);
                    }

                    const double noise = std::max(
                        valAt(seeker.noiseFloorW, i, 0.0), std::numeric_limits<double>::min());
                    double receivedPower = 0.0;
                    if (seeker.type[i] == SeekerType::RF) {
                        const double pt = std::max(0.0, valAt(seeker.transmitterPowerW, i, 0.0));
                        const double gain = std::pow(10.0, valAt(seeker.antennaGainDb, i, 0.0) / 10.0);
                        const double lambda = std::max(0.0, valAt(seeker.wavelengthM, i, 0.0));
                        receivedPower = (pt * gain * gain * lambda * lambda * rcsM2) /
                            (std::pow(4.0 * std::numbers::pi, 3.0) *
                             std::pow(candidate.range, 4.0));
                    } else {
                        // SARH bistatic; illuminator is either the static
                        // configured point or a live entity (opt-in).
                        glm::dvec3 illuminatorPos(
                            valAt(seeker.illuminatorPx, i, 0.0),
                            valAt(seeker.illuminatorPy, i, 0.0),
                            valAt(seeker.illuminatorPz, i, 0.0));
                        const int illuminatorId = intAt(seeker.illuminatorEntityId, i, -1);
                        if (illuminatorId >= 0 &&
                            static_cast<std::size_t>(illuminatorId) < physics.size &&
                            physics.active[illuminatorId]) {
                            const auto li = static_cast<std::size_t>(illuminatorId);
                            illuminatorPos = glm::dvec3(physics.px[li], physics.py[li], physics.pz[li]);
                        }
                        glm::dvec3 targetPos(physics.px[target], physics.py[target], physics.pz[target]);
                        const double rt = glm::length(targetPos - illuminatorPos);
                        const double rr = candidate.range;
                        if (rt <= 1e-9 || rr <= 1e-9) {
                            candidate.reject = SeekerRejectReason::Geometry;
                            return candidate;
                        }
                        const double pt = std::max(0.0, valAt(seeker.illuminatorPowerW, i, 0.0));
                        const double gt = std::pow(10.0, valAt(seeker.illuminatorGainDb, i, 0.0) / 10.0);
                        const double gr = std::pow(10.0, valAt(seeker.antennaGainDb, i, 0.0) / 10.0);
                        const double lambda = std::max(0.0, valAt(seeker.illuminatorWavelengthM, i, 0.0));
                        receivedPower = (pt * gt * gr * lambda * lambda * rcsM2) /
                            (std::pow(4.0 * std::numbers::pi, 3.0) * rt * rt * rr * rr);
                    }
                    double snrDb = 10.0 * std::log10(
                        std::max(receivedPower, std::numeric_limits<double>::min()) / noise);
                    if (decoyCutDb > 0.0) snrDb -= decoyCutDb;
                    snrDb = applyJammer(snrDb, candidate.range, noise);
                    candidate.signalStrength = snrDb;
                    const double threshold = valAt(seeker.snrThresholdDb, i, 0.0);
                    candidate.acquisitionValid = snrDb > threshold;
                    candidate.maintenanceValid = snrDb > threshold - hysteresisDb;
                } else if (seeker.type[i] == SeekerType::PassiveRF) {
                    const double eirp = target < status.emitterEirpW.size()
                        ? status.emitterEirpW[target] : 0.0;
                    const double duty = std::clamp(
                        valAt(seeker.passiveRfDutyCycle, i, 1.0), 0.0, 1.0);
                    bool emitterOn = eirp > 0.0;
                    if (emitterOn && duty < 1.0) {
                        // Emitter duty cycle: on for a fraction of the time,
                        // drawn only when a duty cycle is configured.
                        emitterOn = stdUniform(rng) <= duty;
                    }
                    if (!emitterOn) {
                        candidate.reject = SeekerRejectReason::Signal;
                        return candidate;
                    }
                    const double gr = std::pow(10.0, valAt(seeker.antennaGainDb, i, 0.0) / 10.0);
                    const double lambda = std::max(0.0, valAt(seeker.wavelengthM, i, 0.0));
                    const double noise = std::max(
                        valAt(seeker.noiseFloorW, i, 0.0), std::numeric_limits<double>::min());
                    double receivedPower = (eirp * gr * lambda * lambda) /
                        (16.0 * std::numbers::pi * std::numbers::pi *
                         candidate.range * candidate.range);
                    double snrDb = 10.0 * std::log10(
                        std::max(receivedPower, std::numeric_limits<double>::min()) / noise);
                    if (decoyCutDb > 0.0) snrDb -= decoyCutDb;
                    snrDb = applyJammer(snrDb, candidate.range, noise);
                    candidate.signalStrength = snrDb;
                    const double threshold = valAt(seeker.snrThresholdDb, i, 0.0);
                    candidate.acquisitionValid = snrDb > threshold;
                    candidate.maintenanceValid = snrDb > threshold - hysteresisDb;
                } else if (seeker.type[i] == SeekerType::IR) {
                    const std::string& profileId = status.irProfileId[target];
                    if (profileId.empty()) {
                        candidate.reject = SeekerRejectReason::NoProfile;
                        return candidate;
                    }
                    if (!irCache.contains(profileId)) {
                        auto db = std::make_unique<Models::IRSignatureDatabase>();
                        if (db->loadProfile(profileId)) irCache[profileId] = std::move(db);
                        else {
                            if (rcsLoadFailed.insert("ir:" + profileId).second) {
                                std::fprintf(stderr, "[seek] IR seeker: signature profile '%s' failed to load; "
                                             "entity %zu will not be acquired (check working directory)\n",
                                             profileId.c_str(), target);
                            }
                            candidate.reject = SeekerRejectReason::NoProfile;
                            return candidate;
                        }
                    }
                    const double radiantIntensity = irCache.at(profileId)->getRadiantIntensity(
                        targetAzimuth, targetElevation);
                    const double irradiance = radiantIntensity /
                        (candidate.range * candidate.range);
                    const double extinction = std::max(
                        0.0, valAt(seeker.irExtinctionPerM, i, 0.0));
                    const double transmissivity = std::exp(-extinction * candidate.range);
                    double finalPower = irradiance * transmissivity;
                    if (decoyCutDb > 0.0) finalPower /= std::pow(10.0, decoyCutDb / 10.0);
                    const double sensitivity = std::max(
                        valAt(seeker.sensitivityW, i, 0.0), std::numeric_limits<double>::min());
                    const double maintenanceSensitivity = sensitivity /
                        std::pow(10.0, hysteresisDb / 10.0);
                    candidate.signalStrength = finalPower;
                    candidate.acquisitionValid = finalPower > sensitivity;
                    candidate.maintenanceValid = finalPower > maintenanceSensitivity;
                }

                return candidate;
            };

            auto commitTrack = [&](std::size_t target, const Candidate& candidate,
                                   bool newLock) {
                const bool sameTrack = seeker.lockActive[i] &&
                    seeker.lockedTargetId[i] == target && seeker.hasPreviousLos[i];

                // Measurement vector: exact truth unless the noise model is
                // enabled, in which case range/rate/angle noise and optional
                // correlated glint are drawn from the seeker stream.
                double mRange = candidate.range;
                double mRangeRate = candidate.rangeRate;
                double mAzimuth = candidate.azimuth;
                double mElevation = candidate.elevation;
                if (flagAt(seeker.measurementNoiseEnabled, i)) {
                    mRange += stdNorm(rng) * valAt(seeker.rangeNoiseStdDevM, i, 0.0);
                    mRangeRate += stdNorm(rng) * valAt(seeker.rangeRateNoiseStdDevMps, i, 0.0);

                    double snrDb = candidate.signalStrength;
                    if (seeker.type[i] == SeekerType::IR) {
                        const double sens = std::max(
                            valAt(seeker.sensitivityW, i, 0.0),
                            std::numeric_limits<double>::min());
                        snrDb = 10.0 * std::log10(
                            std::max(candidate.signalStrength, std::numeric_limits<double>::min()) / sens);
                    }
                    const double ref = valAt(seeker.angleNoiseRefSnrDb, i, 20.0);
                    double angleSigma = valAt(seeker.angleNoiseStdDevRad, i, 0.0);
                    angleSigma *= std::pow(10.0, -(snrDb - ref) / 20.0);
                    mAzimuth += stdNorm(rng) * angleSigma;
                    mElevation += stdNorm(rng) * angleSigma;

                    const double glintSigma = valAt(seeker.glintSigmaM, i, 0.0);
                    if (glintSigma > 0.0) {
                        const double tau = std::max(
                            valAt(seeker.glintCorrelationTauSec, i, 1.0), 1e-9);
                        const double rho = std::exp(-stepDt / tau);
                        const double q = std::sqrt(std::max(0.0, 1.0 - rho * rho));
                        seeker.glintAzM[i] = rho * seeker.glintAzM[i] + q * stdNorm(rng) * glintSigma;
                        seeker.glintElM[i] = rho * seeker.glintElM[i] + q * stdNorm(rng) * glintSigma;
                        if (candidate.range > 1e-6) {
                            mAzimuth += seeker.glintAzM[i] / candidate.range;
                            mElevation += seeker.glintElM[i] / candidate.range;
                        }
                    }
                }

                if (sameTrack && stepDt > 0.0) {
                    const double tau = std::max(valAt(seeker.rateFilterTauSec, i, 0.05), 1e-9);
                    const double rawAzRate =
                        std::remainder(mAzimuth - seeker.previousAzimuth[i],
                                       2.0 * std::numbers::pi) / stepDt;
                    const double rawElRate =
                        (mElevation - seeker.previousElevation[i]) / stepDt;
                    const double blend = 1.0 - std::exp(-stepDt / tau);
                    seeker.targetAzimuthRate[i] += blend *
                        (rawAzRate - seeker.targetAzimuthRate[i]);
                    seeker.targetElevationRate[i] += blend *
                        (rawElRate - seeker.targetElevationRate[i]);
                } else {
                    seeker.targetAzimuthRate[i] = 0.0;
                    seeker.targetElevationRate[i] = 0.0;
                }
                if (newLock) {
                    seeker.glintAzM[i] = 0.0;
                    seeker.glintElM[i] = 0.0;
                }
                seeker.lockActive[i] = true;
                seeker.lockedTargetId[i] = target;
                seeker.previousAzimuth[i] = mAzimuth;
                seeker.previousElevation[i] = mElevation;
                seeker.hasPreviousLos[i] = true;
                seeker.lockLostTimeSec[i] = 0.0;
                seeker.lastSignalStrength[i] = candidate.signalStrength;
                measurementHistory[i].push_back({
                    target, 0.0, mRange, mRangeRate, mAzimuth, mElevation,
                    seeker.targetAzimuthRate[i], seeker.targetElevationRate[i]});
                setReason(SeekerRejectReason::None);
            };

            const bool hadLock = seeker.lockActive[i];
            bool servicedLock = false;
            if (hadLock && seeker.lockedTargetId[i] < physics.size) {
                const std::size_t target = seeker.lockedTargetId[i];
                if (target != i && physics.active[target] && status.isAlive[target] &&
                    status.allegiance[i] != status.allegiance[target]) {
                    const double rateLimit = valAt(seeker.gimbalRateLimitRadPerSec, i, 0.0);
                    const bool useServo = rateLimit > 0.0;
                    double servoAz = seeker.gimbalAzimuthRad[i];
                    double servoEl = seeker.gimbalElevationRad[i];
                    if (useServo) {
                        double range = 0.0, rangeRate = 0.0, desAz = 0.0, desEl = 0.0;
                        glm::dvec3 losWorld;
                        losAngles(target, range, rangeRate, desAz, desEl, losWorld);
                        servoAz = slewToward(servoAz, desAz, rateLimit * stepDt);
                        servoEl = slewToward(servoEl, desEl, rateLimit * stepDt);
                        seeker.gimbalAzimuthRad[i] = servoAz;
                        seeker.gimbalElevationRad[i] = servoEl;
                    }
                    const Candidate candidate = evaluateTarget(target, useServo, servoAz, servoEl);
                    if (candidate.geometryValid && candidate.maintenanceValid) {
                        commitTrack(target, candidate, /*newLock=*/false);
                        publishAvailable();
                        refreshLockState();
                        servicedLock = true;
                    } else if (candidate.geometryValid) {
                        seeker.lockLostTimeSec[i] += stepDt;
                        const double dropout = std::max(
                            0.0, valAt(seeker.lockDropoutTimeSec, i, 0.0));
                        if (seeker.lockLostTimeSec[i] <= dropout) {
                            setReason(candidate.reject);
                            publishAvailable();
                            refreshLockState();
                            servicedLock = true;
                        }
                    }
                }
            }
            if (servicedLock) continue;

            clearTrack();
            setReason(SeekerRejectReason::NoTarget);

            // Iterate through all potential targets and acquire the strongest
            // signal (single-track lock). Friendly targets are rejected.
            std::size_t bestTarget = 0;
            Candidate bestCandidate;
            bool foundCandidate = false;
            SeekerRejectReason scanReject = SeekerRejectReason::NoTarget;
            for (std::size_t t = 0; t < physics.size; ++t) {
                if (t == i || !physics.active[t] || !status.isAlive[t]) continue;
                if (status.allegiance[i] == status.allegiance[t]) continue; // Don't lock onto friendlies
                const Candidate candidate = evaluateTarget(t, /*useServo=*/false, 0.0, 0.0);
                if (candidate.reject != SeekerRejectReason::None) scanReject = candidate.reject;
                if (candidate.acquisitionValid &&
                    (!foundCandidate || candidate.signalStrength > bestCandidate.signalStrength)) {
                    bestTarget = t;
                    bestCandidate = candidate;
                    foundCandidate = true;
                }
            }
            if (foundCandidate) {
                // Acquire: snap the gimbal onto the target (the seeker scanned
                // onto it), then the servo lags during track.
                seeker.gimbalAzimuthRad[i] = bestCandidate.azimuth;
                seeker.gimbalElevationRad[i] = bestCandidate.elevation;
                commitTrack(bestTarget, bestCandidate, /*newLock=*/true);
            } else {
                // Report why the strongest/scanned candidate failed (range
                // gate, terrain, geometry, signal, missing profile), not a
                // blanket "no signal".
                setReason(scanReject);
            }
            publishAvailable();
            refreshLockState();
        }
    }

} // namespace StrikeEngine::Kernel
