#pragma once

#include "strikeengine/ecs/Component.hpp"

namespace StrikeEngine {

    /**
     * @brief Holds the state and physical properties of an entity's control surfaces.
     */
    struct ControlSurfaceComponent final : public Component {
        /**
         * @brief The maximum physical deflection angle of the control surfaces, in radians.
         * This value is typically loaded from a vehicle's JSON profile.
         */
        double max_deflection_rad = 0.349; // Default to ~20 degrees

        /**
         * @brief The maximum rotational speed of the control surface actuators, in radians per second.
         * This value is typically loaded from a vehicle's JSON profile.
         */
        double max_rate_rad_per_sec = 5.236; // Default to ~300 deg/s

        /**
         * @brief The current, actual deflection of the pitch control surface, in radians.
         * This value is updated by the ControlSystem each frame.
         */
        double current_deflection_rad_pitch = 0.0;

        /**
         * @brief The current, actual deflection of the yaw control surface, in radians.
         * This value is updated by the ControlSystem each frame.
         */
        double current_deflection_rad_yaw = 0.0;
    };

} // namespace StrikeEngine
