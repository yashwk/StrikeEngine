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
    private:
        /** @brief The vector sum of all linear forces acting on the entity's center of mass. */
        glm::dvec3 totalForce{0.0};

        /** @brief The vector sum of all torques acting on the entity. */
        glm::dvec3 totalTorque{0.0};

    public:
        ForceAccumulatorComponent() = default;

        ForceAccumulatorComponent(const glm::dvec3& force, const glm::dvec3& torque)
            : totalForce(force), totalTorque(torque)
        {
        }

        /**
         * @brief Computes and retrieves the total force acting on an entity.
         *
         * Aggregates all individual forces applied to an entity to determine the net force.
         * Typically used in physics calculations to assess motion or interactions.
         *
         * @return The total force as a vector representing magnitude and direction.
         */
        [[nodiscard]] const glm::dvec3& getTotalForce() const noexcept { return totalForce; }
        /**
         * @brief Sets the total force acting on an entity.
         *
         * Replaces the current total force value with the provided vector. This vector
         * represents the complete linear force acting on the entity for the current frame.
         *
         * @param force A 3D vector specifying the new total force in Newtons.
         */
        void setTotalForce(const glm::dvec3& force) noexcept { totalForce = force; }
        /**
         * @brief Adds a force vector to the total force acting on the entity.
         *
         * This method accumulates forces over time by summing individual force vectors.
         * It is typically used by various physics systems to apply additional forces
         * (e.g., gravity, propulsion, collision responses) to the entity during a simulation frame.
         *
         * @param force A 3D vector representing the magnitude and direction of the force to add, in Newtons.
         */
        void addForce(const glm::dvec3& force)
        {
            totalForce += force;
        }

        /**
         * @brief Retrieves the total torque acting on the entity.
         *
         * Provides the accumulated torque vector, which is the result of combining all torques
         * applied to the entity during the current frame. Torque is typically used in physics
         * calculations to determine rotational effects on an object.
         *
         * @return A constant reference to a 3D vector representing the total torque in Newton meters.
         */
        [[nodiscard]] const glm::dvec3& getTotalTorque() const noexcept { return totalTorque; }
        /**
         * @brief Sets the total torque acting on an entity.
         *
         * This method updates the total torque value to the provided vector, which represents
         * the complete rotational force acting on the entity. Torque is typically calculated
         * based on forces and their application points relative to the entity's center of mass.
         *
         * @param torque A 3D vector specifying the new total torque in Newton meters.
         */
        void setTotalTorque(const glm::dvec3& torque) noexcept { totalTorque = torque; }
        /**
         * @brief Adds a torque vector to the total torque acting on the entity.
         *
         * This method contributes to the accumulation of torques over time by adding
         * an individual torque vector to the existing total. It is typically used by
         * various physics systems to apply rotational forces (e.g., from collisions,
         * engines, or external influences) to the entity.
         *
         * @param torque A 3D vector representing the magnitude and direction of the torque to add, in Newton meters.
         */
        void addTorque(const glm::dvec3& torque)
        {
            totalTorque += torque;
        }

        /**
         * @brief Resets all accumulated forces and torques acting on the entity.
         *
         * This method clears the totalForce and totalTorque vectors, setting them to zero.
         * It is typically called at the beginning of a new simulation frame to ensure
         * forces and torques are recalculated without residuals from the previous frame.
         */
        void clear()
        {
            totalForce = glm::dvec3(0.0);
            totalTorque = glm::dvec3(0.0);
        }
    };
} // namespace StrikeEngine
