#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <string>

namespace StrikeEngine {
    enum WarheadType {
        BLAST_FRAGMENTATION,
        CONTINUOUS_ROD,
        SHAPED_CHARGE
    };

    /**
     * @brief Models the lethal payload of a missile.
     */
    struct WarheadComponent final : public Component {
        WarheadType type;
        double mass_kg;
        double lethal_radius_m;

        bool has_detonated;
    };
} // namespace StrikeEngine
