#pragma once

#include "strikeengine/ecs/System.hpp"

#include <glm/glm.hpp>

namespace StrikeEngine {

    class GuidanceSystem final : public System {
    public:
        void update(Registry& registry, double dt) override;

    private:

        /**
         * @brief Calculates the acceleration command using Proportional Navigation (PN).
         * @param missile_position The position of the missile.
         * @param missile_velocity The velocity of the missile.
         * @param target_position The position of the target.
         * @param target_velocity The velocity of the target.
         * @param navigation_constant The navigation gain (N).
         * @return The commanded acceleration vector.
         */
        glm::dvec3 calculateProportionalNavigation(
            const glm::dvec3& missile_position, const glm::dvec3& missile_velocity,
            const glm::dvec3& target_position, const glm::dvec3& target_velocity,
            double navigation_constant
        );

        /**
         * @brief Calculates the acceleration command using Augmented Proportional Navigation (APN).
         * @param missile_position The position of the missile.
         * @param missile_velocity The velocity of the missile.
         * @param target_position The position of the target.
         * @param target_velocity The velocity of the target.
         * @param target_acceleration The acceleration of the target.
         * @param navigation_constant The navigation gain (N).
         * @return The commanded acceleration vector.
         */
        glm::dvec3 calculateAugmentedProportionalNavigation(
            const glm::dvec3& missile_position, const glm::dvec3& missile_velocity,
            const glm::dvec3& target_position, const glm::dvec3& target_velocity, const glm::dvec3& target_acceleration,
            double navigation_constant
        );

        /**
         * @brief Calculates the acceleration command using Pure Pursuit (PP).
         * @param missile_position The position of the missile.
         * @param missile_velocity The velocity of the missile.
         * @param target_position The position of the target.
         * @return The commanded acceleration vector.
         */
        glm::dvec3 calculatePurePursuit(
            const glm::dvec3& missile_position, const glm::dvec3& missile_velocity,
            const glm::dvec3& target_position
        );
    };

} // namespace StrikeEngine
