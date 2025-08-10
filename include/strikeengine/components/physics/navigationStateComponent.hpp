#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace StrikeEngine {

    /**
     * @brief Stores an entity's own, internally calculated estimate of its state.
     *
     * This data is updated by the NavigationSystem by integrating noisy sensor
     * readings. It will slowly drift away from the "ground truth" state
     * represented by the TransformComponent and VelocityComponent.
     */
    struct NavigationStateComponent final : public Component {
        glm::dvec3 estimated_position{0.0};
        glm::dvec3 estimated_velocity{0.0};
        glm::dquat estimated_orientation{1.0, 0.0, 0.0, 0.0};

        bool is_initialized = false;
    };

} // namespace StrikeEngine
