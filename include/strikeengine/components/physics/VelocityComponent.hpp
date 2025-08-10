#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <glm/glm.hpp>

namespace StrikeEngine {

    /**
     * @brief Stores the linear and angular velocity of an entity.
     *
     * This component is updated by the IntegrationSystem each tick based on the
     * total forces and torques applied to the entity.
     */
    struct VelocityComponent : public Component {
        /**
         * @brief Linear velocity in meters per second (m/s).
         * This vector represents the direction and speed of the entity's
         * movement through world space.
         */
        glm::dvec3 linear{0.0};

        /**
         * @brief Angular velocity in radians per second (rad/s).
         * This vector represents the axis of rotation and the rate of rotation
         * around that axis, relative to the entity's own body frame.
         */
        glm::dvec3 angular{0.0};
    };

} // namespace StrikeEngine
