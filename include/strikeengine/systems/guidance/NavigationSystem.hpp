#pragma once

#include "strikeengine/ecs/System.hpp"

namespace StrikeEngine {
    class Registry;
}

namespace StrikeEngine {
    /**
     * @brief Simulates the Inertial Measurement Unit (IMU) and updates the navigational state.
     *
     * This system reads the "perfect" ground truth accelerations and rotations from
     * the physics engine, corrupts them with noise and bias based on the IMUComponent,
     * and integrates them to update the entity's own estimated state in the
     * NavigationStateComponent.
     */
    class NavigationSystem final : public System {
    public:
        NavigationSystem();
        void update(Registry& registry, double dt) override;
    };
} // namespace StrikeEngine
