#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <glm/glm.hpp>
#include <vector>
#include <string>

namespace StrikeEngine {
    /**
     * @brief Represents a single stage of a propulsion system.
     */
    struct PropulsionStage {
        std::string name;
        double stage_mass_kg = 0.0; // The mass of this stage (casing + propellant)
        double thrust_newtons = 0.0; // Thrust magnitude
        double burnTime_seconds = 0.0; // How long this stage fires
        double specificImpulse_seconds = 0.0; // Engine efficiency (Isp)
    };

    /**
     * @brief Manages the state of a multi-stage propulsion system for an entity.
     */
    struct PropulsionComponent final : public Component {
        // The definition of all stages for this entity, loaded from a profile.
        std::vector<PropulsionStage> stages;

        // --- State Variables ---
        int currentStageIndex = -1; // -1 means inactive, 0 is the first stage
        double timeInCurrentStage_seconds = 0.0;
        bool active = false;
    };
} // namespace StrikeEngine
