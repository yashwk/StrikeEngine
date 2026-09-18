#include <strikeengine/kernel/profiles/SensorProfileDatabase.hpp>
#include <nlohmann/json.hpp>
#include <fstream>

namespace StrikeEngine::Kernel {

    bool SensorProfileDatabase::loadProfile(const std::string& file_path, std::string* error) {
        std::ifstream f(file_path);
        if (!f.is_open()) {
            if (error) *error = "cannot open file";
            return false;
        }

        try {
            nlohmann::json data = nlohmann::json::parse(f);

            // Start from the struct defaults; every key is optional.
            SensorConfig cfg;
            cfg.imuEnabled       = data.value("imu_enabled", cfg.imuEnabled);
            cfg.gpsEnabled       = data.value("gps_enabled", cfg.gpsEnabled);
            cfg.accelNoiseStdDev = data.value("accel_noise_std_dev", cfg.accelNoiseStdDev);
            cfg.accelBiasStdDev  = data.value("accel_bias_std_dev", cfg.accelBiasStdDev);
            cfg.gyroNoiseStdDev  = data.value("gyro_noise_std_dev", cfg.gyroNoiseStdDev);
            cfg.gyroBiasStdDev   = data.value("gyro_bias_std_dev", cfg.gyroBiasStdDev);
            cfg.gpsPosNoiseStdDev = data.value("gps_pos_noise_std_dev", cfg.gpsPosNoiseStdDev);
            cfg.gpsVelNoiseStdDev = data.value("gps_vel_noise_std_dev", cfg.gpsVelNoiseStdDev);
            cfg.gpsUpdateRateHz  = data.value("gps_update_rate_hz", cfg.gpsUpdateRateHz);
            cfg.gpsInnovationGateSigma = data.value("gps_innovation_gate_sigma", cfg.gpsInnovationGateSigma);
            cfg.baroInnovationGateSigma = data.value("baro_innovation_gate_sigma", cfg.baroInnovationGateSigma);
            cfg.magInnovationGateSigma = data.value("mag_innovation_gate_sigma", cfg.magInnovationGateSigma);
            cfg.imuLeverArmX     = data.value("imu_lever_arm_x", cfg.imuLeverArmX);
            cfg.imuLeverArmY     = data.value("imu_lever_arm_y", cfg.imuLeverArmY);
            cfg.imuLeverArmZ     = data.value("imu_lever_arm_z", cfg.imuLeverArmZ);
            cfg.baroEnabled      = data.value("baro_enabled", cfg.baroEnabled);
            cfg.baroNoiseStdDev  = data.value("baro_noise_std_dev", cfg.baroNoiseStdDev);
            cfg.baroBiasStdDev   = data.value("baro_bias_std_dev", cfg.baroBiasStdDev);
            cfg.baroUpdateRateHz = data.value("baro_update_rate_hz", cfg.baroUpdateRateHz);
            cfg.magEnabled       = data.value("mag_enabled", cfg.magEnabled);
            cfg.magNoiseStdDev   = data.value("mag_noise_std_dev", cfg.magNoiseStdDev);
            cfg.magUpdateRateHz  = data.value("mag_update_rate_hz", cfg.magUpdateRateHz);
            cfg.magDisturbanceGateRel = data.value("mag_disturbance_gate_rel", cfg.magDisturbanceGateRel);
            cfg.gpsLatencySec    = data.value("gps_latency_sec", cfg.gpsLatencySec);
            cfg.gpsLeverArmX     = data.value("gps_lever_arm_x", cfg.gpsLeverArmX);
            cfg.gpsLeverArmY     = data.value("gps_lever_arm_y", cfg.gpsLeverArmY);
            cfg.gpsLeverArmZ     = data.value("gps_lever_arm_z", cfg.gpsLeverArmZ);
            cfg.gpsFixConsistencyEnabled = data.value("gps_fix_consistency_enabled", cfg.gpsFixConsistencyEnabled);
            cfg.insConingCompensationEnabled = data.value("ins_coning_compensation_enabled", cfg.insConingCompensationEnabled);
            cfg.insAdaptiveQEnabled = data.value("ins_adaptive_q_enabled", cfg.insAdaptiveQEnabled);
            cfg.insAdaptiveQGain = data.value("ins_adaptive_q_gain", cfg.insAdaptiveQGain);
            cfg.initialAttitudeErrorDeg = data.value("initial_attitude_error_deg", cfg.initialAttitudeErrorDeg);
            cfg.initialPositionErrorM = data.value("initial_position_error_m", cfg.initialPositionErrorM);
            cfg.initialVelocityErrorMps = data.value("initial_velocity_error_mps", cfg.initialVelocityErrorMps);
            cfg.insGravityGradientEnabled = data.value("ins_gravity_gradient_enabled", cfg.insGravityGradientEnabled);
            cfg.insEarthRotationCouplingEnabled = data.value("ins_earth_rotation_coupling_enabled", cfg.insEarthRotationCouplingEnabled);
            cfg.gpsBatchUpdateEnabled = data.value("gps_batch_update_enabled", cfg.gpsBatchUpdateEnabled);
            cfg.gpsLeverArmCompensationEnabled = data.value("gps_lever_arm_compensation_enabled", cfg.gpsLeverArmCompensationEnabled);
            cfg.gpsYawCorrectionDamping = data.value("gps_yaw_correction_damping", cfg.gpsYawCorrectionDamping);
            cfg.gpsFixConsistencyThreshold = data.value("gps_fix_consistency_threshold", cfg.gpsFixConsistencyThreshold);
            cfg.gpsFixConsistencyConfidence = data.value("gps_fix_consistency_confidence", cfg.gpsFixConsistencyConfidence);
            cfg.gpsFixConsistencyDof = data.value("gps_fix_consistency_dof", cfg.gpsFixConsistencyDof);
            cfg.maxAccelBiasEstimate = data.value("max_accel_bias_estimate", cfg.maxAccelBiasEstimate);
            cfg.maxGyroBiasEstimate = data.value("max_gyro_bias_estimate", cfg.maxGyroBiasEstimate);
            cfg.baroAttitudeCorrectionEnabled = data.value("baro_attitude_correction_enabled", cfg.baroAttitudeCorrectionEnabled);
            cfg.antennaPositionX = data.value("antenna_position_x", cfg.antennaPositionX);
            cfg.antennaPositionY = data.value("antenna_position_y", cfg.antennaPositionY);
            cfg.antennaPositionZ = data.value("antenna_position_z", cfg.antennaPositionZ);
            cfg.antennaScanRateHz = data.value("antenna_scan_rate_hz", cfg.antennaScanRateHz);
            _sensor = cfg;
        } catch (const std::exception& e) {
            if (error) *error = e.what();
            return false;
        }

        return true;
    }

    const SensorConfig& SensorProfileDatabase::sensor() const {
        return _sensor;
    }

} // namespace StrikeEngine::Kernel
