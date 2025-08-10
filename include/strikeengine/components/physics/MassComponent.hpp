#pragma once

#include "strikeengine/ecs/Component.hpp"

namespace StrikeEngine {

    /**
     * @brief Represents the physical mass of an entity, including changes from fuel consumption.
     *
     * This component is essential for all physics calculations involving force and acceleration.
     * It tracks the initial (wet) mass, the final (dry) mass, and the current mass,
     * which allows systems like the ThrustSystem to model fuel usage realistically.
     */
    struct MassComponent : public Component {
        /** @brief The initial mass of the entity at launch, including all fuel (in kg). */
        double initialMass_kg{1.0};

        /** @brief The mass of the entity after all propellant is consumed (in kg). */
        double dryMass_kg{1.0};

        /** @brief The current mass of the entity at the current simulation tick (in kg). */
        double currentMass_kg{1.0};

        /**
         * @brief The inverse of the current mass (1.0 / currentMass_kg).
         * Pre-calculated to optimize physics calculations by replacing division with multiplication.
         * A value of 0.0 represents an object with infinite mass.
         */
        double inverseMass{1.0};

        /**
         * @brief Updates the inverseMass based on the currentMass_kg.
         * Should be called by a system whenever the currentMass_kg changes.
         */
        void updateInverseMass() {
            inverseMass = (currentMass_kg > 1e-9) ? 1.0 / currentMass_kg : 0.0;
        }
    };

} // namespace StrikeEngine
