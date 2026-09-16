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

        /**
         * @brief Grows every vector to @p n entries; see
         *        PhysicsBlock::ensureSize.
         */
        void ensureSize(std::size_t n) {
            accelX.resize(n, 0.0); accelY.resize(n, 0.0); accelZ.resize(n, 0.0);
            gyroX.resize(n, 0.0); gyroY.resize(n, 0.0); gyroZ.resize(n, 0.0);
            gpsUpdated.resize(n, false);
            gpsPosX.resize(n, 0.0); gpsPosY.resize(n, 0.0); gpsPosZ.resize(n, 0.0);
            gpsVelX.resize(n, 0.0); gpsVelY.resize(n, 0.0); gpsVelZ.resize(n, 0.0);
            accelNoiseStdDev.resize(n, 0.1);
            accelBiasStdDev.resize(n, 0.01);
            gyroNoiseStdDev.resize(n, 0.01);
            gyroBiasStdDev.resize(n, 0.001);
            gpsPosNoiseStdDev.resize(n, 5.0);
            gpsVelNoiseStdDev.resize(n, 0.5);
            gpsInnovationGateSigma.resize(n, 5.0);
            baroInnovationGateSigma.resize(n, -1.0);
            magInnovationGateSigma.resize(n, -1.0);
            imuLeverArmX.resize(n, 0.0);
            imuLeverArmY.resize(n, 0.0);
            imuLeverArmZ.resize(n, 0.0);
            imuEnabled.resize(n, true);
            gpsEnabled.resize(n, true);
            gpsUpdateRateHz.resize(n, 1.0);
            baroUpdated.resize(n, false);
            baroAlt.resize(n, 0.0);
            magUpdated.resize(n, false);
            magX.resize(n, 0.0); magY.resize(n, 0.0); magZ.resize(n, 0.0);
            baroEnabled.resize(n, false);
            baroNoiseStdDev.resize(n, 1.0);
            baroBiasStdDev.resize(n, 0.0);
            baroUpdateRateHz.resize(n, 1.0);
            magEnabled.resize(n, false);
            magNoiseStdDev.resize(n, 50e-9);
            magUpdateRateHz.resize(n, 10.0);
            magDisturbanceGateRel.resize(n, 0.25);
            gpsLatencySec.resize(n, 0.0);
            gpsLeverArmX.resize(n, 0.0);
            gpsLeverArmY.resize(n, 0.0);
            gpsLeverArmZ.resize(n, 0.0);
            gpsFixConsistencyEnabled.resize(n, false);
            insConingCompensationEnabled.resize(n, false);
            insAdaptiveQEnabled.resize(n, false);
            insAdaptiveQGain.resize(n, 1.0);
            initialAttitudeErrorDeg.resize(n, 0.0);
            initialPositionErrorM.resize(n, 0.0);
            initialVelocityErrorMps.resize(n, 0.0);
            insGravityGradientEnabled.resize(n, false);
            insEarthRotationCouplingEnabled.resize(n, false);
            gpsBatchUpdateEnabled.resize(n, false);
            gpsLeverArmCompensationEnabled.resize(n, false);
            gpsYawCorrectionDamping.resize(n, 0.1);
            gpsFixConsistencyThreshold.resize(n, 16.81);
            gpsFixConsistencyConfidence.resize(n, 0.99);
            gpsFixConsistencyDof.resize(n, 6);
            maxAccelBiasEstimate.resize(n, 0.5);
            maxGyroBiasEstimate.resize(n, 0.02);
            baroAttitudeCorrectionEnabled.resize(n, false);
            size = n;
        }
    };

} // namespace StrikeEngine::Kernel
