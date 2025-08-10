#pragma once

#include "strikeengine/ecs/System.hpp"

namespace StrikeEngine {
    class Registry;
}

namespace StrikeEngine {

    /**
     * @brief Integrates forces and torques to update entity position and orientation.
     *
     * This is the final system in the physics pipeline for a given tick. It uses the
     * total accumulated forces and torques to calculate linear and angular acceleration.
     * It then employs a fourth-order Runge-Kutta (RK4) integrator to update the
     * kinematic state (position, velocity, orientation) of each physical entity.
     *
     * After integration, it clears the ForceAccumulatorComponent for all entities,
     * preparing them for the next simulation tick.
     */
    class IntegrationSystem final : public System {
    public:
        /**
         * @brief Updates the system, integrating the physics state for all relevant entities.
         * @param registry A reference to the ECS registry to access components.
         * @param dt The time elapsed since the last frame (delta time), in seconds.
         */
        void update(Registry& registry, double dt) override;
    };

} // namespace StrikeEngine
