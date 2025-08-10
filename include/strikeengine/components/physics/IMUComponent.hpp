#pragma once

#include "strikeengine/ecs/Component.hpp"

namespace StrikeEngine {

    /**
     * @brief Defines the error characteristics of an Inertial Measurement Unit (IMU).
     *
     * These parameters model the imperfections of real-world gyroscopes and
     * accelerometers, which are the primary source of navigational drift.
     */
    struct IMUComponent final : public Component {
        // Gyroscope Error Parameters
        double gyro_bias_drift_rate_deg_per_hr = 0.1;
        double gyro_noise_density_deg_per_sqrt_hr = 0.01;

        // Accelerometer Error Parameters
        double accelerometer_bias_milli_g = 1.0;
        double accelerometer_noise_density_g_per_sqrt_hz = 0.001;
    };

} // namespace StrikeEngine
