#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <string>

namespace StrikeEngine {

    /**
     * @brief Defines the trigger logic for a warhead.
     */
    struct FuzeComponent final : public Component {
        /**
         * @brief A string identifier for the fuze type.
         * Examples: "proximity_radar", "proximity_laser", "impact"
         */
        std::string type = "proximity_radar";

        /**
         * @brief The distance from the target, in meters, at which the proximity
         * fuze will trigger the warhead detonation.
         */
        double trigger_distance_m = 5.0;
    };

} // namespace StrikeEngine
