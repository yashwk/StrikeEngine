#pragma once

#include "strikeengine/ecs/System.hpp"

namespace StrikeEngine {
    class Registry;
}

namespace StrikeEngine {

    /**
     * @brief Simulates the behavior of onboard sensors (seekers).
     *
     * This system is responsible for detecting and tracking targets. It iterates
     * through all entities with an active seeker, scans for potential targets,
     * and updates the seeker's state (e.g., has_lock, locked_target) based on
     * simple geometric and range checks.
     */
    class SensorSystem final : public System {
    public:
        void update(Registry& registry, double dt) override;
    };

} // namespace StrikeEngine
