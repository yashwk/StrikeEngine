#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <glm/glm.hpp>
#include <vector>
#include <string>

namespace StrikeEngine {

    /**
     * @brief Represents a single point on a thrust curve.
     * @param first Time in seconds since stage ignition.
     * @param second Thrust magnitude in Newtons at that time.
     */
    using ThrustDataPoint = std::pair<double, double>;

    /**
     * @brief Represents a single stage of a propulsion system with variable thrust.
     */
    struct PropulsionStage {
        std::string name;
        double stage_mass_kg = 0.0; // The mass of this stage (casing and propellant)

        std::vector<ThrustDataPoint> thrust_curve;
        double burnTime_seconds = 0.0; // Total duration of the stage burn

        // --- UPGRADED ---
        // Engine efficiency (Isp) is now defined at sea level and in a vacuum.
        double isp_sea_level_s = 0.0;
        double isp_vacuum_s = 0.0;
    };

    /**
     * @brief Manages the state of a multi-stage propulsion system for an entity.
     */
    struct PropulsionComponent final : public Component {
        std::vector<PropulsionStage> stages;
        int currentStageIndex = -1;
        double timeInCurrentStage_seconds = 0.0;
        bool active = false;
    };
} // namespace StrikeEngine
