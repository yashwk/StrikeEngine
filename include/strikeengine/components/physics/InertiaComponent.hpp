#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <glm/glm.hpp>

namespace StrikeEngine {
    /**
     * @brief Represents the rotational inertia of a physical body.
     *
     * This component stores the moment of inertia tensor, which describes how the
     * entity's mass is distributed relative to its center of mass. It is the
     * rotational equivalent of mass and is used by the IntegrationSystem to
     * calculate angular acceleration from torque (τ = Iα).
     *
     * The tensor is defined in the entity's local body space. For many symmetrical
     * objects like missiles, the off-diagonal elements will be zero.
     */
    struct InertiaComponent final : public Component {
    private:
        /**
         * @brief The 3x3 moment of inertia tensor in body space (kg·m²).
         * This matrix relates applied torque to the resulting angular acceleration.
         * For rigid bodies, this is a symmetric matrix where:
         * - Diagonal elements (I_xx, I_yy, I_zz) represent resistance to rotation around each axis
         * - Off-diagonal elements represent coupling between different axes of rotation
         * - All elements are constant in body space if the body is rigid
         */
        glm::dmat3 inertiaTensor{1.0}; // Default to identity matrix

        /**
         * @brief The inverse of the inertia tensor.
         * Pre-calculated to optimize physics calculations by replacing matrix
         * inversion with matrix multiplication. Used primarily in the angular
         * motion calculations where we need to solve I⁻¹τ = α for angular
         * acceleration α given torque τ. Updated automatically when the
         * inertia tensor changes.
         */
        glm::dmat3 inverseInertiaTensor{1.0};

    public:
        /**
         * @brief Gets the current inertia tensor.
         * @return The 3x3 moment of inertia tensor in body space.
         * The returned tensor is expressed in the body's local coordinate system
         * and remains constant for rigid bodies. Any changes in the world-space
         * inertia tensor are handled by the physics system using the body's orientation.
         */
        [[nodiscard]] const glm::dmat3& getInertiaTensor() const
        {
            return inertiaTensor;
        }

        /**
         * @brief Sets the inertia tensor and updates its inverse.
         * @param tensor The new inertia tensor to set. Must be a symmetric, positive-definite
         * matrix representing the body's resistance to angular acceleration. The tensor should
         * be calculated in the body's local coordinate system and remain constant for rigid bodies.
         */
        void setInertiaTensor(const glm::dmat3& tensor)
        {
            inertiaTensor = tensor;
            updateInverseTensor();
        }


        /**
         * @brief Gets the inverse of the inertia tensor.
         * @return The 3x3 inverse moment of inertia tensor in body space.
         * The returned tensor is used to efficiently compute angular acceleration
         * from applied torques without performing matrix inversion during physics updates.
         */
        [[nodiscard]] const glm::dmat3& getInverseInertiaTensor() const
        {
            return inverseInertiaTensor;
        }

        /**
         * @brief Sets the inverse inertia tensor directly.
         * @param tensor The new inverse inertia tensor to set.
         * This method should be used with caution as it bypasses the automatic
         * update of the inverse from the primary inertia tensor.
         */
        void setInverseInertiaTensor(const glm::dmat3& tensor)
        {
            inverseInertiaTensor = tensor;
        }

        /**
         * @brief Updates the inverse of the moment of inertia tensor.
         *
         * This method calculates and stores the updated inverse of the inertia tensor
         * for the entity based on its current orientation and rotational properties.
         * The inverse tensor is used for efficient computation of angular velocities
         * and other rotation-related physics calculations.
         *
         * The inversion process assumes that the inertia tensor is diagonal or symmetric,
         * ensuring numerical stability. Changes in the entity's rotation or configuration
         * require this update to maintain accurate physical behavior in simulations.
         *
         * It is typically used in physics systems to relate torque and angular velocity
         * during simulation steps.
         */
        void updateInverseTensor()
        {
            inverseInertiaTensor = glm::inverse(inertiaTensor);
        }
    };
} // namespace StrikeEngine
