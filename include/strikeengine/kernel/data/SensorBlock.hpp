#pragma once
#include <strikeengine/kernel/data/BlockGrowth.hpp>

#include <vector>
#include <cstddef>

namespace StrikeEngine::Kernel {

    struct RadarMeasurement {
        int sourceEntityId = -1;
        int targetEntityId = -1;
        double timestampSec = 0.0;
        double rangeM = 0.0;
        double rangeRateMps = 0.0;
        double azimuthRad = 0.0;
        double elevationRad = 0.0;
        double signalStrengthDb = 0.0;
    };

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

        // Active fire-control radar configuration and the current scan's
        // target-specific measurements. The vector is transient output and is
        // consumed before the next kernel step.
        std::vector<double> antennaPositionX, antennaPositionY, antennaPositionZ;
        std::vector<double> antennaScanRateHz;
        std::vector<bool> radarEnabled;
        std::vector<double> radarMaxRangeM;
        std::vector<double> radarFieldOfViewHalfAngleRad;
        std::vector<double> radarRangeNoiseStdDevM;
        std::vector<double> radarRangeRateNoiseStdDevMps;
        std::vector<double> radarAngleNoiseStdDevRad;
        std::vector<double> radarMeasurementLatencySec;
        std::vector<bool> radarTerrainMaskingEnabled;
        std::vector<double> radarTrackCoastTimeoutSec;
        std::vector<double> radarTrackLossTimeoutSec;
        std::vector<double> radarTrackQualityTauSec;
        std::vector<RadarMeasurement> radarMeasurements;

        std::size_t size = 0;

        /**
         * @brief Grows every vector to @p n entries; see
         *        PhysicsBlock::ensureSize.
         */
        void ensureSize(std::size_t n) {
            growTo(accelX, n, 0.0); growTo(accelY, n, 0.0); growTo(accelZ, n, 0.0);
            growTo(gyroX, n, 0.0); growTo(gyroY, n, 0.0); growTo(gyroZ, n, 0.0);
            growTo(gpsUpdated, n, false);
            growTo(gpsPosX, n, 0.0); growTo(gpsPosY, n, 0.0); growTo(gpsPosZ, n, 0.0);
            growTo(gpsVelX, n, 0.0); growTo(gpsVelY, n, 0.0); growTo(gpsVelZ, n, 0.0);
            growTo(accelNoiseStdDev, n, 0.1);
            growTo(accelBiasStdDev, n, 0.01);
            growTo(gyroNoiseStdDev, n, 0.01);
            growTo(gyroBiasStdDev, n, 0.001);
            growTo(gpsPosNoiseStdDev, n, 5.0);
            growTo(gpsVelNoiseStdDev, n, 0.5);
            growTo(gpsInnovationGateSigma, n, 5.0);
            growTo(baroInnovationGateSigma, n, -1.0);
            growTo(magInnovationGateSigma, n, -1.0);
            growTo(imuLeverArmX, n, 0.0);
            growTo(imuLeverArmY, n, 0.0);
            growTo(imuLeverArmZ, n, 0.0);
            growTo(imuEnabled, n, true);
            growTo(gpsEnabled, n, true);
            growTo(gpsUpdateRateHz, n, 1.0);
            growTo(baroUpdated, n, false);
            growTo(baroAlt, n, 0.0);
            growTo(magUpdated, n, false);
            growTo(magX, n, 0.0); growTo(magY, n, 0.0); growTo(magZ, n, 0.0);
            growTo(baroEnabled, n, false);
            growTo(baroNoiseStdDev, n, 1.0);
            growTo(baroBiasStdDev, n, 0.0);
            growTo(baroUpdateRateHz, n, 1.0);
            growTo(magEnabled, n, false);
            growTo(magNoiseStdDev, n, 50e-9);
            growTo(magUpdateRateHz, n, 10.0);
            growTo(magDisturbanceGateRel, n, 0.25);
            growTo(gpsLatencySec, n, 0.0);
            growTo(gpsLeverArmX, n, 0.0);
            growTo(gpsLeverArmY, n, 0.0);
            growTo(gpsLeverArmZ, n, 0.0);
            growTo(gpsFixConsistencyEnabled, n, false);
            growTo(insConingCompensationEnabled, n, false);
            growTo(insAdaptiveQEnabled, n, false);
            growTo(insAdaptiveQGain, n, 1.0);
            growTo(initialAttitudeErrorDeg, n, 0.0);
            growTo(initialPositionErrorM, n, 0.0);
            growTo(initialVelocityErrorMps, n, 0.0);
            growTo(insGravityGradientEnabled, n, false);
            growTo(insEarthRotationCouplingEnabled, n, false);
            growTo(gpsBatchUpdateEnabled, n, false);
            growTo(gpsLeverArmCompensationEnabled, n, false);
            growTo(gpsYawCorrectionDamping, n, 0.1);
            growTo(gpsFixConsistencyThreshold, n, 16.81);
            growTo(gpsFixConsistencyConfidence, n, 0.99);
            growTo(gpsFixConsistencyDof, n, 6);
            growTo(maxAccelBiasEstimate, n, 0.5);
            growTo(maxGyroBiasEstimate, n, 0.02);
            growTo(baroAttitudeCorrectionEnabled, n, false);
            growTo(antennaPositionX, n, 0.0);
            growTo(antennaPositionY, n, 0.0);
            growTo(antennaPositionZ, n, 0.0);
            growTo(antennaScanRateHz, n, 0.0);
            growTo(radarEnabled, n, false);
            growTo(radarMaxRangeM, n, 0.0);
            growTo(radarFieldOfViewHalfAngleRad, n, 3.14159265358979323846);
            growTo(radarRangeNoiseStdDevM, n, 0.0);
            growTo(radarRangeRateNoiseStdDevMps, n, 0.0);
            growTo(radarAngleNoiseStdDevRad, n, 0.0);
            growTo(radarMeasurementLatencySec, n, 0.0);
            growTo(radarTerrainMaskingEnabled, n, false);
            growTo(radarTrackCoastTimeoutSec, n, 0.5);
            growTo(radarTrackLossTimeoutSec, n, 2.0);
            growTo(radarTrackQualityTauSec, n, 1.0);
            if (n > size) size = n;
        }
    };

} // namespace StrikeEngine::Kernel
