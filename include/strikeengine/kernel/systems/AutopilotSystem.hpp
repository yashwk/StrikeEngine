#pragma once

#include <strikeengine/kernel/data/ControlBlock.hpp>
#include <strikeengine/kernel/data/GuidanceBlock.hpp>
#include <strikeengine/kernel/data/NavigationBlock.hpp>
#include <strikeengine/kernel/data/SensorBlock.hpp>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>
#include <vector>
#include <cstddef>

namespace StrikeEngine::Kernel {

    /**
     * @brief Bounded acceleration-command autopilot (W3 spine).
     *
     * Direct feed-forward acceleration demand per channel (pitch/yaw), with
     * body-rate and AoA/sideslip damping for short-period stability. Commands
     * are transformed from world to aerospace body axes before fin mapping.
     *   roll loop:   wings-level P-D on estimated roll angle/rate
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

        void update(
            const EntityStatusBlock& status,
            const NavigationBlock& nav,
            const SensorBlock& sensor,
            const GuidanceBlock& guidance,
            ControlBlock& control,
            double dt);

    private:
        void updateFlightController(
            std::size_t id,
            const NavigationBlock& nav,
            const GuidanceBlock& guidance,
            ControlBlock& control);

        // Direct acceleration-command controller: feed-forward fin demand
        // plus body-rate and AoA damping. Per-entity gains now live in
        // ControlBlock (design-time configurable); see SimulationKernel.
    };

} // namespace StrikeEngine::Kernel
