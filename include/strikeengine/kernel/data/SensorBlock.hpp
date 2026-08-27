#pragma once

#include <vector>
#include <cstddef>

namespace StrikeEngine::Kernel {

    struct SensorBlock {
        // IMU Measurements (High Frequency)
        std::vector<double> accelX, accelY, accelZ; // Specific force (m/s^2)
        std::vector<double> gyroX, gyroY, gyroZ;    // Angular rates (rad/s)

        // GPS Measurements (Low Frequency)
        std::vector<bool> gpsUpdated;               // True if new GPS data arrived this tick
        std::vector<double> gpsPosX, gpsPosY, gpsPosZ; // Position (m)
        std::vector<double> gpsVelX, gpsVelY, gpsVelZ; // Velocity (m/s)

        // Sensor configurations per entity
        std::vector<double> accelNoiseStdDev;
        std::vector<double> accelBiasStdDev;
        std::vector<double> gyroNoiseStdDev;
        std::vector<double> gyroBiasStdDev;
        std::vector<double> gpsPosNoiseStdDev;
        std::vector<double> gpsVelNoiseStdDev;
        std::vector<double> imuLeverArmX, imuLeverArmY, imuLeverArmZ;  // body-frame IMU lever arm (m)
        
        std::size_t size = 0;
    };

} // namespace StrikeEngine::Kernel
