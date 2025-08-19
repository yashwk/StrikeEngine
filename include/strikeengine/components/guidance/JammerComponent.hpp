#pragma once

#include "strikeengine/ecs/Component.hpp"

namespace StrikeEngine {

    /**
     * @brief Models an electronic noise jammer.
     */
    struct JammerComponent final : public Component {
        /**
         * @brief The effective radiated power (ERP) of the jammer, in Watts.
         * This combines the transmitter power and the antenna gain of the jammer.
         */
        double effective_radiated_power_W = 1000.0;

        /**
         * @brief A flag to turn the jammer on or off.
         */
        bool active = false;
    };

} // namespace StrikeEngine
