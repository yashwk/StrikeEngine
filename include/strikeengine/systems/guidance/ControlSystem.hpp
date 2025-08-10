#pragma once

#include "strikeengine/ecs/System.hpp"

namespace StrikeEngine {
    class Registry;
}

namespace StrikeEngine {
    /**
     * @brief Implements the autopilot logic to translate guidance commands into fin deflections.
     *
     * This system reads the commanded acceleration from the AutopilotCommandComponent
     * and uses a control law (e.g., a PID controller) to calculate the necessary
     * control surface deflections. It then writes these desired deflections into the
     * ControlSurfaceComponent. This system models the missile's flight control computer
     * and actuator response.
     */
    class ControlSystem final : public System {
    public:
        /**
         * @brief Updates the system, calculating fin deflections for all guided entities.
         * @param registry A reference to the ECS registry to access components.
         * @param dt The time elapsed since the last frame (delta time), in seconds.
         */
        void update(Registry& registry, double dt) override;
    };
} // namespace StrikeEngine
