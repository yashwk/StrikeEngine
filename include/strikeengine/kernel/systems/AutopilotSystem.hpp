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
     * @brief Rate-command autopilot (W3 spine).
     *
     * Two-loop structure per channel (pitch/yaw), aerospace NED body axes:
     *   outer loop:  commanded accel (world, from guidance) -> desired body
     *                rate via P + integral on accel error (kills steady-state
     *                miss) using the measured specific force
     *   inner loop:  desired rate -> fin deflection (P on rate error) with
     *                AoA/sideslip damping for static-stability augmentation
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
            const SensorBlock& sensor,
            const GuidanceBlock& guidance,
            ControlBlock& control,
            double dt);

        // Per-entity integrator state for the outer loops
        std::vector<double> pitchIntegral;
        std::vector<double> yawIntegral;
        std::vector<bool>   wasCommanded;

        // Gains (tuned for the W3 DoD intercept validation)
        static constexpr double kAccelP     = 0.08;  // (rad/s) per (m/s^2)
        static constexpr double kAccelI     = 0.06;  // (rad/s) per (m/s^2 * s)
        static constexpr double kRateP      = 0.15;  // rad deflection per (rad/s)
        static constexpr double kAlphaP     = 0.15;  // rad deflection per rad AoA/beta (stability augmentation)
        static constexpr double kRollP      = 0.10;  // roll deflection per rad roll
        static constexpr double kRollD      = 0.05;  // roll deflection per (rad/s)
    };

} // namespace StrikeEngine::Kernel
