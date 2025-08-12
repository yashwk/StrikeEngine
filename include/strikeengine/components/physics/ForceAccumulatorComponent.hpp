#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <glm/glm.hpp>

namespace StrikeEngine {
    /**
     * @brief Accumulates all forces and torques acting on an entity over a single frame.
     *
     * Physics systems (like Gravity, Propulsion, Aerodynamics, etc.) add their calculated
     * forces and torques to this component. The IntegrationSystem then reads the
     * final sum to calculate the accelerations for the frame.
     */
    struct ForceAccumulatorComponent final : public Component {
        /** @brief The vector sum of all linear forces acting on the entity's center of mass. */
        glm::dvec3 totalForce{0.0};

        /** @brief The vector sum of all torques acting on the entity. */
        glm::dvec3 totalTorque{0.0};

        /**
         * @brief Adds a linear force to the accumulator.
         * @param force The force vector to add, in Newtons.
         */
        void addForce(const glm::dvec3& force)
        {
            totalForce += force;
        }

        /**
         * @brief Adds a torque to the accumulator.
         * @param torque The torque vector to add, in Newton meters.
         */
        void addTorque(const glm::dvec3& torque)
        {
            totalTorque += torque;
        }

        /**
         * @brief Resets both the total force and total torque to zero.
         * This should be called by the IntegrationSystem at the end of each frame.
         */
        void clear()
        {
            totalForce = glm::dvec3(0.0);
            totalTorque = glm::dvec3(0.0);
        }
    };
} // namespace StrikeEngine
