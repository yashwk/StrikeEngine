#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <string>

namespace StrikeEngine {
    enum FuzeType {
        PROXIMITY_RADAR,
        PROXIMITY_LASER,
        IMPACT,
        NONE,
    };

    /**
     * @brief Defines the trigger logic for a warhead.
     */
    struct FuzeComponent final : public Component {
        FuzeType type;
        double trigger_angle_deg;
        double trigger_distance_m;
    };
} // namespace StrikeEngine
