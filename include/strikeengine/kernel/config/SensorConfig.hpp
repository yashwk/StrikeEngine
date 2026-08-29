#pragma once

namespace StrikeEngine::Kernel {

    struct SensorConfig {
        bool imuEnabled = true;
        bool gpsEnabled = true;
        double accelNoiseStdDev = 0.1;
        double accelBiasStdDev  = 0.01;
        double gyroNoiseStdDev  = 0.01;
        double gyroBiasStdDev   = 0.001;
        double gpsPosNoiseStdDev = 5.0;
        double gpsVelNoiseStdDev = 0.5;
        double gpsUpdateRateHz = 1.0;
        // Normalized innovation gate for each scalar GPS position/velocity
        // measurement. <= 0 disables rejection; the default is a 5-sigma gate.
        double gpsInnovationGateSigma = 5.0;
        double imuLeverArmX = 0.0;
        double imuLeverArmY = 0.0;
        double imuLeverArmZ = 0.0;
    };

} // namespace StrikeEngine::Kernel
