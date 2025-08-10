#pragma once

#include "strikeengine/ecs/System.hpp"

namespace StrikeEngine {

    // Forward-declare Registry to avoid including the full header here.
    class Registry;

    /**
     * @brief Applies gravitational force to all physical entities.
     *
     * This system calculates gravity based on the universal law of gravitation,
     * accounting for changes in force due to altitude. It assumes a simplified,
     * non-rotating spherical Earth model.
     *
     * The calculated gravitational force is added to each entity's
     * ForceAccumulatorComponent each tick.
     */
    class GravitySystem : public System {
    public:
        /**
         * @brief Updates the system, applying gravitational force to all relevant entities.
         * @param registry A reference to the ECS registry to access components.
         * @param dt The time elapsed since the last frame (delta time), in seconds.
         */
        void update(Registry& registry, double dt) override;
    };

} // namespace StrikeEngine
