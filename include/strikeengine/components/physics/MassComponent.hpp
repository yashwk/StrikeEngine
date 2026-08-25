#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <algorithm>

namespace StrikeEngine {

    /**
     * @brief Represents the mass properties of a rigid body.
     *
     * This component separates immutable structural mass from
     * consumable mass (fuel/propellant). All derived quantities
     * are computed, never stored redundantly.
     */
    struct MassComponent final : public Component {

        // --------------------------------------------------
        // Immutable structural properties
        // --------------------------------------------------

        /** @brief Structural (dry) mass of the entity in kg. */
        double dryMass_kg{1.0};

        // --------------------------------------------------
        // Consumable mass (fuel / propellant)
        // --------------------------------------------------

        /** @brief Total fuel mass at launch (kg). */
        double fuelMassInitial_kg{0.0};

        /** @brief Remaining fuel mass (kg). */
        double fuelMassRemaining_kg{0.0};

        // --------------------------------------------------
        // Derived quantities (computed, not stored)
        // --------------------------------------------------

        /** @brief Returns the current total mass (kg). */
        [[nodiscard]] double totalMass() const noexcept {
            return dryMass_kg + fuelMassRemaining_kg;
        }

        /** @brief Returns inverse total mass (1/kg). */
        [[nodiscard]] double inverseMass() const noexcept {
            const double m = totalMass();
            return (m > 1e-9) ? 1.0 / m : 0.0;
        }

        /** @brief Returns true if fuel remains. */
        [[nodiscard]] bool hasFuel() const noexcept {
            return fuelMassRemaining_kg > 0.0;
        }

        /** @brief Consumes fuel (kg). Clamped safely. */
        void consumeFuel(double mass_kg) noexcept {
            fuelMassRemaining_kg =
                std::max(0.0, fuelMassRemaining_kg - mass_kg);
        }
    };

} // namespace StrikeEngine
