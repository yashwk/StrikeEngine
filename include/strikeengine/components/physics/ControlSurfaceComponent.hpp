#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <glm/glm.hpp>

namespace StrikeEngine {

    /**
     * @brief Stores the real-time state of an entity's control surface actuators.
     *
     * This component is intentionally lean, containing only the data that changes
     * tick-by-tick. The physical limitations of the actuators (e.g., max deflection,
     * max rate) are defined in data profiles and are managed by the ControlSystem.
     * This keeps the component universal and applicable to any entity with actuators.
     */
    struct ControlSurfaceComponent : public Component {
        /**
         * @brief The current, actual deflection angle for each control surface, in radians.
         * This value is updated by the ControlSystem to move towards the commanded state.
         */
        glm::dvec4 currentDeflections_rad{0.0};

        /**
         * @brief The target deflection angles commanded by the guidance logic, in radians.
         * The ControlSystem reads this value and drives the currentDeflections towards it.
         */
        glm::dvec4 commandedDeflections_rad{0.0};
    };

} // namespace StrikeEngine
