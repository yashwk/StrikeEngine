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

        /**
         * @brief The guidance law to be chosen from the GuidanceLaw enum
         */
        GuidanceLaw law;

        /**
         * @brief The navigation constant (N) for Proportional Navigation.
         * A dimensionless value, typically between 3 and 5. It determines the
         * "aggressiveness" of the guidance corrections.
         */
        double navigation_constant;

        /** @brief A flag to enable or disable the guidance logic for this entity. */
        bool enabled;
    };
} // namespace StrikeEngine
