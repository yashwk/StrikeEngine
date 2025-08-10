#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <glm/glm.hpp>

namespace StrikeEngine {

    /**
     * @brief Stores the internal state for an entity's autopilot/control system.
     *
     * This component holds variables that need to persist between simulation ticks
     * for the control logic to function correctly, such as the accumulated error
     * for a PID controller's integral term.
     */
    struct AutopilotStateComponent final : public Component {
        /**
         * @brief The accumulated integral of the error for the PID controller.
         * This helps the controller eliminate steady-state error over time.
         */
        glm::dvec3 integral_error{0.0};
    };

} // namespace StrikeEngine
