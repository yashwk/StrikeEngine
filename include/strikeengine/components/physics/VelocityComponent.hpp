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
    struct VelocityComponent final : public Component {
    private:
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

    public:
        VelocityComponent() = default;

        VelocityComponent(const glm::dvec3& linear, const glm::dvec3& angular)
            : linear(linear), angular(angular)
        {
        }

        /**
         * @brief Retrieves the linear velocity of the entity.
         *
         * This method returns a reference to the linear velocity vector, which
         * describes the direction and magnitude of the entity's translational
         * movement in world space.
         *
         * @return A constant reference to the entity's linear velocity.
         */
        [[nodiscard]] const glm::dvec3& getLinear() const noexcept { return linear; }
        /**
         * @brief Sets the linear velocity of the entity.
         *
         * Updates the entity's linear velocity vector, which represents its speed
         * and direction of translational movement in world space.
         *
         * @param linearVelocity A 3D vector representing the desired linear velocity
         *                       in meters per second (m/s).
         */
        void setLinear(const glm::dvec3& linearVelocity) noexcept { linear = linearVelocity; }
        /**
         * @brief Adds a velocity vector to the entity's linear velocity.
         *
         * This method increments the current linear velocity of the entity by the
         * specified velocity vector. The resulting linear velocity represents the new
         * direction and speed of translational movement in world space.
         *
         * @param linearVelocity A 3D vector representing the linear velocity to add,
         *                       in meters per second (m/s).
         */
        void addLinear(const glm::dvec3& linearVelocity) noexcept { linear += linearVelocity; }

        /**
         * @brief Retrieves the angular velocity of the entity.
         *
         * This method returns a reference to the angular velocity vector, which
         * describes the axis and rate of the entity’s rotation in its local frame
         * of reference.
         *
         * @return A constant reference to the entity's angular velocity.
         */
        [[nodiscard]] const glm::dvec3& getAngular() const noexcept { return angular; }
        /**
         * @brief Sets the angular velocity of the entity.
         *
         * Updates the entity's angular velocity vector, which represents the axis of rotation
         * and the rate of rotation around that axis relative to the entity's local frame.
         *
         * @param angularVelocity A 3D vector representing the desired angular velocity
         *                        in radians per second (rad/s).
         */
        void setAngular(const glm::dvec3& angularVelocity) noexcept { angular = angularVelocity; }
        /**
         * @brief Increments the angular velocity of the entity.
         *
         * This method adds the specified angular velocity vector to the entity's
         * current angular velocity. The resulting angular velocity determines the
         * axis and rate of rotation around that axis relative to the entity's local frame.
         *
         * @param angularVelocity A 3D vector representing the angular velocity to add,
         *                        in radians per second (rad/s).
         */
        void addAngular(const glm::dvec3& angularVelocity) noexcept { angular += angularVelocity; }
    };
} // namespace StrikeEngine
