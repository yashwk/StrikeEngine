#pragma once

#include "strikeengine/ecs/Component.hpp"

namespace StrikeEngine {
    struct GyroTypes {
        enum Type {
            MEMS,
            FiberOptic,
            RingLaser,
            HemisphericalResonator,
        };
    };
    struct AccelerometerTypes {
        enum Type {
            MEMS,
            Piezoelectric,
            Capacitive,
            VibratingBeam,
        };
    };
    /**
     * @brief Defines the error characteristics of an Inertial Measurement Unit (IMU).
     *
     * These parameters model the imperfections of real-world gyroscopes and
     * accelerometers, which are the primary source of navigational drift.
     */
    struct IMUComponent final : public Component {
        // Gyroscope Error Parameters
        double gyro_bias_drift_rate_deg_per_hr;
        double gyro_noise_density_deg_per_sqrt_hr;

        // Accelerometer Error Parameters
        double accelerometer_bias_milli_g;
        double accelerometer_noise_density_g_per_sqrt_hz;

        // Magnetometer Error Parameters
        double magnetometer_bias_uT;
        double magnetometer_noise_density_uT_per_sqrt_hz;
    };
} // namespace StrikeEngine
