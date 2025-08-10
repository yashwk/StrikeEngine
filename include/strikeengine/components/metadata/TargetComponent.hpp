#pragma once

#include "strikeengine/ecs/Component.hpp"

namespace StrikeEngine {

    /**
     * @brief Tags an entity as a target and stores its signature properties.
     */
    struct TargetComponent final : public Component {
        /** @brief The entity's Radar Cross-Section (RCS) in square meters (m^2). */
        double rcs_m2 = 1.0;
    };

} // namespace StrikeEngine
