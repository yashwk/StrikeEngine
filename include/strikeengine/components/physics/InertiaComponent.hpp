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
    struct InertiaComponent : public Component {
        /**
         * @brief The 3x3 moment of inertia tensor in body space (kg·m²).
         * This matrix relates applied torque to the resulting angular acceleration.
         */
        glm::dmat3 inertiaTensor{1.0}; // Default to identity matrix

        /**
         * @brief The inverse of the inertia tensor.
         * Pre-calculated to optimize physics calculations by replacing matrix
         * inversion with matrix multiplication.
         */
        glm::dmat3 inverseInertiaTensor{1.0};

        /**
         * @brief Updates the inverseInertiaTensor from the current inertiaTensor.
         * Should be called if the mass distribution of the entity changes (e.g., fuel sloshing, not modeled yet).
         */
        void updateInverseTensor() {
            inverseInertiaTensor = glm::inverse(inertiaTensor);
        }
    };

} // namespace StrikeEngine
