#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <glm/glm.hpp>

namespace StrikeEngine {

    /**
     * @brief A temporary container to sum up all forces and torques applied to an
     * entity within a single simulation tick.
     *
     * This component is central to the force-based physics model. Each physics system
     * (Gravity, Thrust, Drag, etc.) calculates its respective forces and torques
     * and adds them to this component. The IntegrationSystem then reads the final
     * totals to calculate the resulting linear and angular acceleration.
     *
     * The values in this component are cleared to zero at the end of each tick
     * by the IntegrationSystem after the physics state has been updated.
     */
    struct ForceAccumulatorComponent : public Component {
        /** @brief The vector sum of all forces applied this tick, in Newtons (N). */
        glm::dvec3 totalForce{0.0};

        /** @brief The vector sum of all torques applied this tick, in Newton meters (N·m). */
        glm::dvec3 totalTorque{0.0};

        /**
         * @brief Resets the accumulated forces and torques to zero.
         * Called by the IntegrationSystem at the end of a physics update.
         */
        void clear() {
            totalForce = glm::dvec3(0.0);
            totalTorque = glm::dvec3(0.0);
        }
    };

} // namespace StrikeEngine
