#pragma once

#include "strikeengine/ecs/Component.hpp"
#include "strikeengine/ecs/Entity.hpp"
namespace StrikeEngine {
    enum class GuidanceLaw {
        ProportionalNavigation,
        AugmentedProportionalNavigation,
        PurePursuit
    };

    /**
     * @brief Configures and stores the state for a guided entity.
     *
     * This component identifies an entity as being capable of guidance and holds
     * the necessary parameters, such as its current target and the specific
     * guidance law to be used.
     */
    struct GuidanceComponent final : public Component {
        /** @brief The unique ID of the entity this component is trying to intercept. */
        Entity targetEntity = NULL_ENTITY;

        /** @brief A string identifier for the guidance law (e.g., "ProportionalNavigation"). */
        GuidanceLaw law = GuidanceLaw::ProportionalNavigation;

        /**
         * @brief The navigation constant (N) for Proportional Navigation.
         * A dimensionless value, typically between 3 and 5. It determines the
         * "aggressiveness" of the guidance corrections.
         */
        double navigation_constant = 4.0;

        /** @brief A flag to enable or disable the guidance logic for this entity. */
        bool enabled = true;
    };

} // namespace StrikeEngine
