#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <string>

namespace StrikeEngine {

    /**
     * @brief Defines the aerodynamic properties and current state of an entity.
     */
    struct AerodynamicProfileComponent final : public Component {
        // --- Static Properties (Loaded from Profile) ---

        /**
         * @brief A unique identifier used to look up the correct set of
         * aerodynamic tables from a central database.
         */
        std::string profileID;

        /**
         * @brief The reference area (in m^2) used in aerodynamic force calculations.
         */
        double referenceArea_m2 = 1.0;

        /**
         * @brief The wingspan of the vehicle (in meters), used for ground effect calculations.
         */
        double wingspan_m = 1.0;


        // --- State Variables (Updated by Systems each tick) ---

        /**
         * @brief The current angle of attack (AoA) in radians.
         */
        double currentAngleOfAttack_rad = 0.0;

        /**
         * @brief The current sideslip angle (Beta) in radians.
         */
        double currentSideslipAngle_rad = 0.0;

        /**
         * @brief The current Mach number (speed / speed of sound).
         */
        double currentMachNumber = 0.0;
    };

} // namespace StrikeEngine
