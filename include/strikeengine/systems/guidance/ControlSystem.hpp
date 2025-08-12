#pragma once

#include "strikeengine/ecs/System.hpp"

namespace StrikeEngine {
    class ControlSystem final : public System {
    public:
        /**
         * @brief Executes the autopilot logic for one time step.
         * @param registry The ECS registry containing all entities and components.
         * @param dt The simulation time step (delta time).
         */
        void update(Registry& registry, double dt) override;
    };

} // namespace StrikeEngine
