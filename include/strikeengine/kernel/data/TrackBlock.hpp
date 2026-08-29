#pragma once
#include <vector>
#include <cstdint>
#include <cstddef>

namespace StrikeEngine::Kernel {

    // Persistent target-track lifecycle (W39 track manager).
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
     * @brief Per-entity persistent target-track state (W39).
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

        std::size_t size = 0;

        bool active(std::size_t i) const {
            return i < size && state[i] != TrackState::None &&
                   state[i] != TrackState::Lost;
        }
    };

} // namespace StrikeEngine::Kernel
