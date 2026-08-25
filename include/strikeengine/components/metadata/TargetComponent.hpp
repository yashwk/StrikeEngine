#pragma once

#include "strikeengine/ecs/Component.hpp"
#include "strikeengine/flight/RCSDatabase.hpp"

namespace StrikeEngine {

    /**
     * @brief Tags an entity as a target and stores its signature properties.
     */
    struct TargetComponent final : public Component {
        // TODO : Expand this component with more metadata

        double rcs_m2 {};
    };

} // namespace StrikeEngine
