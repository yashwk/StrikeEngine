#pragma once

#include "strikeengine/ecs/System.hpp"

namespace StrikeEngine {
    class Registry;
}

namespace StrikeEngine {
    /**
     * @brief Implements guidance laws to steer entities towards their targets.
     *
     * This system is the core of an entity's autonomous decision-making. It reads
     * the entity's current state, its target's state, and uses a specified
     * guidance law (e.g., Proportional Navigation) to calculate a required
     * acceleration command. This command is then written to the
     * AutopilotCommandComponent for the ControlSystem to execute.
     */
    class GuidanceSystem final : public System {
    public:
        /**
         * @brief Updates the system, calculating guidance commands for all guided entities.
         * @param registry A reference to the ECS registry to access components.
         * @param dt The time elapsed since the last frame (delta time), in seconds.
         */
        void update(Registry& registry, double dt) override;
    };
} // namespace StrikeEngine
