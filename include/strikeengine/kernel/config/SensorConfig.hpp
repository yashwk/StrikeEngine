#pragma once

namespace StrikeEngine::Kernel {

    struct SensorConfig {
        bool imuEnabled = true;
        bool gpsEnabled = true;
        // Realistic tactical-INS noise floors (tight enough for the strapdown
        // EKF to track attitude over long engagements). Gyro noise is ~0.01
        // deg/s (1.7e-4 rad/s); accelerometer ~0.02 m/s^2; a coarse 1-sigma
        // gyro/accel bias.
        double accelNoiseStdDev = 0.02;
        double accelBiasStdDev  = 0.005;
        double gyroNoiseStdDev  = 0.0002;
        double gyroBiasStdDev   = 0.00005;
        double gpsPosNoiseStdDev = 1.5;
        double gpsVelNoiseStdDev = 0.15;
        double gpsUpdateRateHz = 1.0;
        // Normalized innovation gate for each scalar GPS position/velocity
        // measurement. <= 0 disables rejection; the default is a 5-sigma gate.
        double gpsInnovationGateSigma = 5.0;
        double imuLeverArmX = 0.0;
        double imuLeverArmY = 0.0;
        double imuLeverArmZ = 0.0;
    };

} // namespace StrikeEngine::Kernel
