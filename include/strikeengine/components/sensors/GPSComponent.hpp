#pragma once

#include "strikeengine/ecs/Component.hpp"

namespace StrikeEngine {

    /**
     * @brief Models a GPS receiver for sensor fusion.
     */
    struct GPSComponent final : public Component {
        /**
         * @brief The rate at which the GPS provides a new position fix, in Hertz.
         * A value of 1.0 means one update per second.
         */
        double update_rate_hz = 1.0;

        /**
         * @brief The accuracy of the GPS position fix, in meters. This represents
         * the 1-sigma standard deviation of the position error.
         */
        double position_error_m = 0.1;

        /**
         * @brief An internal timer used by the NavigationSystem to track when the
         * next GPS fix is due.
         */
        double time_since_last_update_s = 0.0;
    };

} // namespace StrikeEngine
