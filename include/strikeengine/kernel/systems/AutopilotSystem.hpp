#pragma once

#include <strikeengine/kernel/data/ControlBlock.hpp>
#include <strikeengine/kernel/data/GuidanceBlock.hpp>
#include <strikeengine/kernel/data/NavigationBlock.hpp>
#include <strikeengine/kernel/data/SensorBlock.hpp>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <vector>
#include <cstddef>

namespace StrikeEngine::Kernel {

    /**
     * @brief Bounded acceleration-command autopilot.
     *
     * Maps the guidance world-frame acceleration demand to fin deflections:
     * feed-forward per channel (pitch/yaw) plus body-rate and AoA/sideslip
     * damping for short-period stability, with a wings-level P-D roll loop.
     * Optional trims: bounded integral on the specific-force error, Mach
     * control-effectiveness and q scheduling, in-loop actuator lag/rate
     * limits, measured-rate damping, roll suppression under lateral demand
     * (all off by default; see ControlBlock).
     *
     * Sign conventions (verified against the body-frame aero model):
     *   +az body (Z down)  -> pitch DOWN (negative wy) -> negative finPitch
     *   +ay body (Y right) -> yaw RIGHT (positive wz)  -> positive finYaw
     *   +finPitch          -> nose-up moment (positive wy)
     *   +finYaw            -> nose-right moment (positive wz)
     */
    class AutopilotSystem {
    public:
        AutopilotSystem();

        /**
         * @brief Steps every live guided entity; entities with
         * GuidanceMode::None get zeroed commands. Grows the ControlBlock
         * arrays when the caller has not sized them.
         */
        void update(
            const EntityStatusBlock& status,
            const NavigationBlock& nav,
            const SensorBlock& sensor,
            const GuidanceBlock& guidance,
            ControlBlock& control,
            double dt,
            const EnvironmentConfig& environment);

    private:
        /// Single-entity control law (sections 1-8 in the implementation).
        void updateFlightController(
            std::size_t id,
            const NavigationBlock& nav,
            const SensorBlock& sensor,
            const GuidanceBlock& guidance,
            ControlBlock& control,
            double dt,
            const EnvironmentConfig& environment);
    };

} // namespace StrikeEngine::Kernel
