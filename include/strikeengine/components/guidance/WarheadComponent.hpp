#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <string>

namespace StrikeEngine {

    /**
     * @brief Models the lethal payload of a missile.
     */
    struct WarheadComponent final : public Component {
        /**
         * @brief A string identifier for the warhead type.
         * Examples: "blast_fragmentation", "continuous_rod", "shaped_charge"
         */
        std::string type = "blast_fragmentation";

        /**
         * @brief The effective lethal radius of the warhead's blast and
         * fragmentation effect, in meters. If a target is within this
         * radius upon detonation, it is considered destroyed.
         */
        double lethal_radius_m = 10.0;

        /**
         * @brief A state flag that is set to true by the EndgameSystem when
         * the fuze triggers detonation.
         */
        bool has_detonated = false;
    };

} // namespace StrikeEngine
