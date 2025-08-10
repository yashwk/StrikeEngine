#pragma once

#include "strikeengine/ecs/System.hpp"

namespace StrikeEngine {
    class Registry;
}

namespace StrikeEngine {

    /**
     * @brief Manages multi-stage propulsion, stage transitions, and fuel consumption.
     *
     * This system handles the complex logic of multi-stage engines. It activates
     * stages in sequence, applies the correct thrust for the active stage, models
     * fuel consumption, and handles the jettisoning of mass when a stage is spent.
     */
    class PropulsionSystem final : public System {
    public:
        void update(Registry& registry, double dt) override;
    };

} // namespace StrikeEngine
