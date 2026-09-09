#include <strikeengine/kernel/systems/TrackManagerSystem.hpp>
#include <cmath>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace StrikeEngine::Kernel {

    namespace {

        // Deterministic quality/uncertainty model for a seeker-converted LOS
        // fix. These are model constants (documented in the SPEC); the
        // per-entity state timers come from GuidanceAutopilotConfig.
        constexpr double kQualityTauSec = 1.0;       // quality exp decay time constant
        constexpr double kPosStdBaseM   = 5.0;       // single-fix position uncertainty
        constexpr double kPosDriftMps   = 25.0;      // position uncertainty growth rate
        constexpr double kVelStdBaseMps = 25.0;      // single-fix velocity uncertainty
        constexpr double kVelDriftMps2  = 50.0;      // velocity uncertainty growth rate

        bool trackActive(TrackState s) {
            return s != TrackState::None && s != TrackState::Lost;
        }

    }

    void TrackManagerSystem::update(
        const NavigationBlock& nav,
        const SeekerBlock& seeker,
        TrackBlock& tracks,
        double timeSec,
        double dt)
    {
        const std::size_t n = std::min({tracks.size, nav.size, seeker.size});
        for (std::size_t i = 0; i < n; ++i) {
            auto& state = tracks.state[i];
            const bool hasMeasurement =
                seeker.type[i] != SeekerType::None && seeker.isLocked[i];

            // --- No new measurement: age, predict, decay, and age states ----
            if (!hasMeasurement) {
                if (state == TrackState::None) continue;

                tracks.ageSec[i] += dt;
                tracks.dropoutCount[i] += 1;
                // Multi-rate kinematic prediction at the simulation rate.
                tracks.posX[i] += tracks.velX[i] * dt;
                tracks.posY[i] += tracks.velY[i] * dt;
                tracks.posZ[i] += tracks.velZ[i] * dt;
                if (tracks.accelAvailable[i]) {
                    tracks.velX[i] += tracks.accelX[i] * dt;
                    tracks.velY[i] += tracks.accelY[i] * dt;
                    tracks.velZ[i] += tracks.accelZ[i] * dt;
                }

                const double coastT = tracks.coastTimeoutSec[i];
                const double lossT  = tracks.lossTimeoutSec[i];
                if (state == TrackState::Acquire || state == TrackState::Reacquire) {
                    if (tracks.ageSec[i] >= coastT) state = TrackState::Coast;
                } else if (state == TrackState::Maintain) {
                    if (tracks.ageSec[i] >= coastT) state = TrackState::Coast;
                } else if (state == TrackState::Coast) {
                    if (tracks.ageSec[i] >= lossT) state = TrackState::Lost;
                }

                if (trackActive(state)) {
                    const double age = tracks.ageSec[i];
                    tracks.positionStdM[i] =
                        kPosStdBaseM + kPosDriftMps * age;
                    tracks.velocityStdMs[i] =
                        kVelStdBaseMps + kVelDriftMps2 * age;
                    tracks.quality01[i] = std::exp(-age / kQualityTauSec);
                }
                continue;
            }

            // --- New measurement: convert LOS fix to the world frame --------
            const double rng = seeker.targetRange[i];
            const double az  = seeker.targetAzimuth[i];
            const double el  = seeker.targetElevation[i];
            if (!std::isfinite(rng) || rng <= 0.0) continue;

            // Seeker body frame: X forward, Y right, Z down (same aerospace
            // convention as the airframe). Elevation is below the X axis.
            const double cEl = std::cos(el), sEl = std::sin(el);
            const glm::dvec3 losBody(cEl * std::cos(az), cEl * std::sin(az), -sEl);
            glm::dquat estQ(nav.estQw[i], nav.estQx[i], nav.estQy[i], nav.estQz[i]);
            const glm::dvec3 losWorld = estQ * losBody;
            const glm::dvec3 navPos(nav.estPx[i], nav.estPy[i], nav.estPz[i]);
            const glm::dvec3 measPos = navPos + losWorld * rng;

            // Track identity: counted from the seeker lock; a re-lock on a
            // different target starts a fresh track.
            const std::int64_t newId =
                static_cast<std::int64_t>(seeker.lockedTargetId[i]);
            const bool sameTarget = tracks.trackId[i] == newId;

            tracks.timestampSec[i] = timeSec;
            tracks.ageSec[i] = 0.0;
            tracks.dropoutCount[i] = 0;
            tracks.positionStdM[i] = kPosStdBaseM;
            tracks.velocityStdMs[i] = kVelStdBaseMps;
            tracks.quality01[i] = 1.0;

            // Velocity: finite difference between consecutive fixes, smoothed
            // with a low-pass so noisy long-range LOS fixes don't blow up the
            // estimate (a 0.1 deg angle jitter at 100 km is ~175 m/step, which
            // over a 0.01 s step would otherwise be ~17500 m/s).
            if (state != TrackState::None && sameTarget &&
                tracks.updateCount[i] > 0)
            {
                const double dtMeas = std::max(1e-9, timeSec - tracks.measTimeSec[i]);
                const double nvx = (measPos.x - tracks.measPosX[i]) / dtMeas;
                const double nvy = (measPos.y - tracks.measPosY[i]) / dtMeas;
                const double nvz = (measPos.z - tracks.measPosZ[i]) / dtMeas;
                const double alpha = 0.08;   // low-pass blend (0.08 -> ~12 s tau)
                tracks.velX[i] = (1.0 - alpha) * tracks.velX[i] + alpha * nvx;
                tracks.velY[i] = (1.0 - alpha) * tracks.velY[i] + alpha * nvy;
                tracks.velZ[i] = (1.0 - alpha) * tracks.velZ[i] + alpha * nvz;
            } else if (state == TrackState::None) {
                tracks.velX[i] = tracks.velY[i] = tracks.velZ[i] = 0.0;
                tracks.accelAvailable[i] = false;
            }
            tracks.measPosX[i] = measPos.x;
            tracks.measPosY[i] = measPos.y;
            tracks.measPosZ[i] = measPos.z;
            tracks.measTimeSec[i] = timeSec;

            // State lifecycle.
            if (state == TrackState::None) {
                state = TrackState::Acquire;
                tracks.updateCount[i] = 1;
                tracks.trackId[i] = newId;
                tracks.accelAvailable[i] = false;
            } else if (!sameTarget) {
                // Re-lock on a different target: fresh track (identity from
                // the seeker now; the command seed's accel no longer applies).
                state = TrackState::Acquire;
                tracks.updateCount[i] = 1;
                tracks.trackId[i] = newId;
                tracks.accelAvailable[i] = false;
            } else if (state == TrackState::Coast || state == TrackState::Lost) {
                const bool wasLost = state == TrackState::Lost;
                state = TrackState::Reacquire;
                tracks.updateCount[i] = 1;
                if (wasLost) tracks.accelAvailable[i] = false;  // stale after loss
            } else {
                tracks.updateCount[i] += 1;
            }

            // Promotion to Maintain after `confirmations` consistent updates.
            if ((state == TrackState::Acquire || state == TrackState::Reacquire) &&
                tracks.updateCount[i] >=
                    static_cast<std::uint32_t>(std::max(1, tracks.confirmations[i])))
            {
                state = TrackState::Maintain;
            }

            tracks.posX[i] = measPos.x;
            tracks.posY[i] = measPos.y;
            tracks.posZ[i] = measPos.z;
        }
    }

} // namespace StrikeEngine::Kernel
