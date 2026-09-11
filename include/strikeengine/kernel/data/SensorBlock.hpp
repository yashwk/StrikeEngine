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
        std::vector<double> gpsInnovationGateSigma;
        std::vector<double> baroInnovationGateSigma; // <0 = follow the GPS gate
        std::vector<double> magInnovationGateSigma;  // <0 = follow the GPS gate
        std::vector<double> imuLeverArmX, imuLeverArmY, imuLeverArmZ;  // body-frame IMU lever arm (m)
        
        // Per-entity device enablement + GPS rate (config wiring).
        std::vector<bool> imuEnabled;           // IMU measurements produced while true
        std::vector<bool> gpsEnabled;           // GPS samples produced while true
        std::vector<double> gpsUpdateRateHz;    // per-entity GPS update rate (Hz)

        // Barometer measurements (opt-in altitude aiding).
        std::vector<bool> baroUpdated;          // true if new baro data arrived this tick
        std::vector<double> baroAlt;            // pressure altitude (m, datum-relative)

        // Magnetometer measurements (opt-in heading aiding, body frame T).
        std::vector<bool> magUpdated;
        std::vector<double> magX, magY, magZ;

        // Aiding + realism configuration per entity (from SensorConfig).
        std::vector<bool> baroEnabled;
        std::vector<double> baroNoiseStdDev;
        std::vector<double> baroBiasStdDev;
        std::vector<double> baroUpdateRateHz;
        std::vector<bool> magEnabled;
        std::vector<double> magNoiseStdDev;
        std::vector<double> magUpdateRateHz;
        std::vector<double> magDisturbanceGateRel;
        std::vector<double> gpsLatencySec;
        std::vector<double> gpsLeverArmX, gpsLeverArmY, gpsLeverArmZ;
        std::vector<bool> gpsFixConsistencyEnabled;
        std::vector<bool> insConingCompensationEnabled;
        std::vector<bool> insAdaptiveQEnabled;
        std::vector<double> insAdaptiveQGain;
        std::vector<double> initialAttitudeErrorDeg;
        std::vector<double> initialPositionErrorM;
        std::vector<double> initialVelocityErrorMps;

        // INS error-model fidelity + GPS fusion options (from SensorConfig).
        std::vector<bool> insGravityGradientEnabled;
        std::vector<bool> insEarthRotationCouplingEnabled;
        std::vector<bool> gpsBatchUpdateEnabled;
        std::vector<bool> gpsLeverArmCompensationEnabled;
        std::vector<double> gpsYawCorrectionDamping;
        std::vector<double> gpsFixConsistencyThreshold;
        std::vector<double> gpsFixConsistencyConfidence;
        std::vector<int> gpsFixConsistencyDof;
        std::vector<double> maxAccelBiasEstimate;
        std::vector<double> maxGyroBiasEstimate;
        std::vector<bool> baroAttitudeCorrectionEnabled;

        std::size_t size = 0;
    };

} // namespace StrikeEngine::Kernel
