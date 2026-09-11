#pragma once
#include <vector>
#include <array>
#include <cstdint>
#include <cstddef>

namespace StrikeEngine::Kernel {

    // Persistent target-track lifecycle.
    //
    // Acquire:  initial state (external command seed or first measurement),
    //           waiting for `confirmations` consistent measurement updates.
    // Maintain: track confirmed; estimates refreshed by each measurement.
    // Coast:    no new measurement for `coastTimeoutSec`; multi-rate kinematic
    //           prediction continues at the simulation rate.
    // Lost:     no measurement for `lossTimeoutSec`; the track is dropped from
    //           guidance consumption (external command state becomes the aim).
    // Reacquire: a measurement (or command refresh) arrives for a Coast/Lost
    //           track; returns to Maintain after `confirmations` updates.
    enum class TrackState : uint8_t { None, Acquire, Maintain, Coast, Lost, Reacquire };

    /**
     * @brief Per-entity persistent target-track state.
     *
     * The track manager fuses external command seeds (SimulationCommand) and
     * seeker LOS measurements (converted to the world frame through the
     * navigation estimate) into a single estimate consumed by guidance. The
     * guidance layer reads ONLY these estimates (never physics truth).
     */
    struct TrackBlock {
        // --- Per-entity track configuration (from GuidanceAutopilotConfig) ---
        std::vector<int>    confirmations;     // measurement updates to promote to Maintain
        std::vector<double> coastTimeoutSec;   // no measurement: Maintain/Acquire -> Coast
        std::vector<double> lossTimeoutSec;    // no measurement: Coast -> Lost
        // Estimator + association options (legacy defaults).
        std::vector<bool>   filterEnabled;         // constant-acceleration Kalman
        std::vector<double> processNoiseMps2;      // target accel PSD
        std::vector<double> angleStdRad;           // assumed seeker angular sigma
        std::vector<double> measNoiseScale;        // derived-R scale
        std::vector<double> residualGateSigma;     // 0 = off
        std::vector<double> maxAccelMps2;          // 0 = no clamp
        std::vector<int>    retargetConfirmations; // consistent ids before retarget
        std::vector<int>    seedPolicy;            // 0 clobber, 1 init-only, 2 refresh-stale
        std::vector<double> minQuality01;          // active() quality gate
        std::vector<double> qualityTauSec;         // quality decay
        std::vector<double> velocityBlend;         // legacy finite-diff low-pass

        // --- Per-entity track state -----------------------------------------
        std::vector<TrackState> state;         // None = no track
        std::vector<std::int64_t> trackId;     // target identity; -1 = unknown
        // Estimated world-frame target state (local ENU convention, same as nav)
        std::vector<double> posX, posY, posZ;
        std::vector<double> velX, velY, velZ;
        std::vector<double> accelX, accelY, accelZ;
        std::vector<bool>   accelAvailable;
        // Measurement provenance: sim time of the last measurement/seed and the
        // time since it (track age).
        std::vector<double> timestampSec;
        std::vector<double> ageSec;
        // Track quality / uncertainty (deterministic model: uncertainty grows
        // with age since the last measurement; quality decays exponentially).
        std::vector<double> positionStdM;
        std::vector<double> velocityStdMs;
        std::vector<double> quality01;
        // Measurement bookkeeping
        std::vector<std::uint32_t> updateCount;   // consecutive measurement updates
        std::vector<std::uint32_t> dropoutCount;  // consecutive steps without a measurement
        // Last measurement position/time (for finite-difference velocity)
        std::vector<double> measPosX, measPosY, measPosZ;
        std::vector<double> measTimeSec;

        // Filter state (constant-acceleration Kalman, 3x3 per axis packed as
        // [axis*9 + i*3 + j]). Only touched when filterEnabled.
        std::vector<std::array<double, 27>> kfCov;
        // Retarget debounce: candidate identity + consecutive-update count.
        std::vector<std::int64_t> retargetCandidateId;
        std::vector<std::uint32_t> retargetCount;
        // Diagnostics.
        std::vector<double> lastInnovationM;
        std::vector<std::uint32_t> residualRejectCount;

        std::size_t size = 0;

        bool active(std::size_t i) const {
            if (i >= size || state[i] == TrackState::None ||
                state[i] == TrackState::Lost) {
                return false;
            }
            if (i < minQuality01.size() && quality01[i] < minQuality01[i]) {
                return false;
            }
            return true;
        }
    };

} // namespace StrikeEngine::Kernel
