#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <string>

namespace StrikeEngine {

    /**
     * @brief Defines the aerodynamic properties and current state of an entity.
     *
     * This component acts as a link between an entity and its detailed aerodynamic
     * data (like lift and drag coefficient tables), which are stored externally
     * and managed by an AerodynamicsDatabase. Physics systems use this component
     * to look up the correct coefficients based on the current flight conditions.
     */
    struct AerodynamicProfileComponent : public Component {
        /**
         * @brief A unique identifier (e.g., "missile_mk1_aero") used to look up
         * the correct set of aerodynamic tables from a central database.
         */
        std::string profileID;

        /**
         * @brief The reference area (in meters squared, m^2) used in aerodynamic
         * force calculations. This is typically the cross-sectional area or wing area.
         */
        double referenceArea_m2{1.0};

        // --- State Variables (Updated by Systems each tick) ---

        /**
         * @brief The current angle of attack (AoA) in radians.
         * Calculated by a physics system each frame based on the entity's velocity
         * vector and its forward orientation.
         */
        double currentAngleOfAttack_rad{0.0};

        /**
         * @brief The current sideslip angle (Beta) in radians.
         * Similar to AoA but for the side-to-side direction.
         */
        double currentSideslipAngle_rad{0.0};

        /**
         * @brief The current Mach number (speed / speed of sound).
         * Calculated by a physics system based on the entity's velocity and
         * the current atmospheric conditions (temperature).
         */
        double currentMachNumber{0.0};
    };

} // namespace StrikeEngine
