#include <strikeengine/kernel/profiles/SeekerProfileDatabase.hpp>
#include <strikeengine/kernel/config/SeekerTypeStrings.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

namespace StrikeEngine::Kernel {

    bool SeekerProfileDatabase::loadProfile(const std::string& file_path, std::string* error) {
        std::ifstream f(file_path);
        if (!f.is_open()) {
            if (error) *error = "cannot open file";
            return false;
        }

        try {
            nlohmann::json data = nlohmann::json::parse(f);

            // `type` is the only required key (no sensible default). The
            // remaining fields start from the struct defaults.
            SeekerConfig cfg;
            cfg.type = seekerTypeFromString(data.at("type").get<std::string>());

            // --- RF (monostatic radar) ---
            cfg.transmitterPowerW = data.value("transmitter_power_w", cfg.transmitterPowerW);
            cfg.antennaGainDb = data.value("antenna_gain_db", cfg.antennaGainDb);
            cfg.wavelengthM = data.value("wavelength_m", cfg.wavelengthM);
            cfg.noiseFloorW = data.value("noise_floor_w", cfg.noiseFloorW);
            cfg.snrThresholdDb = data.value("snr_threshold_db", cfg.snrThresholdDb);

            // --- IR ---
            cfg.sensitivityW = data.value("sensitivity_w", cfg.sensitivityW);
            cfg.wavelengthBand = data.value("wavelength_band", cfg.wavelengthBand);
            cfg.irExtinctionPerM = data.value("ir_extinction_per_m", cfg.irExtinctionPerM);

            // --- SARH illuminator (static position) ---
            cfg.illuminatorPx = data.value("illuminator_px", cfg.illuminatorPx);
            cfg.illuminatorPy = data.value("illuminator_py", cfg.illuminatorPy);
            cfg.illuminatorPz = data.value("illuminator_pz", cfg.illuminatorPz);
            cfg.illuminatorPowerW = data.value("illuminator_power_w", cfg.illuminatorPowerW);
            cfg.illuminatorGainDb = data.value("illuminator_gain_db", cfg.illuminatorGainDb);
            cfg.illuminatorWavelengthM = data.value("illuminator_wavelength_m", cfg.illuminatorWavelengthM);

            // --- Geometry and tracking (half-angles, radians) ---
            cfg.fieldOfViewHalfAngleRad = data.value("field_of_view_half_angle_rad", cfg.fieldOfViewHalfAngleRad);
            cfg.gimbalAzimuthLimitRad = data.value("gimbal_azimuth_limit_rad", cfg.gimbalAzimuthLimitRad);
            cfg.gimbalElevationLimitRad = data.value("gimbal_elevation_limit_rad", cfg.gimbalElevationLimitRad);
            cfg.lockHysteresisDb = data.value("lock_hysteresis_db", cfg.lockHysteresisDb);
            cfg.lockDropoutTimeSec = data.value("lock_dropout_time_sec", cfg.lockDropoutTimeSec);
            cfg.measurementLatencySec = data.value("measurement_latency_sec", cfg.measurementLatencySec);
            cfg.measurementNoiseEnabled = data.value("measurement_noise_enabled", cfg.measurementNoiseEnabled);
            cfg.angleNoiseStdDevRad = data.value("angle_noise_std_dev_rad", cfg.angleNoiseStdDevRad);
            cfg.angleNoiseRefSnrDb = data.value("angle_noise_ref_snr_db", cfg.angleNoiseRefSnrDb);
            cfg.rangeNoiseStdDevM = data.value("range_noise_std_dev_m", cfg.rangeNoiseStdDevM);
            cfg.rangeRateNoiseStdDevMps = data.value("range_rate_noise_std_dev_mps", cfg.rangeRateNoiseStdDevMps);
            cfg.glintSigmaM = data.value("glint_sigma_m", cfg.glintSigmaM);
            cfg.glintCorrelationTauSec = data.value("glint_correlation_tau_sec", cfg.glintCorrelationTauSec);
            cfg.swerlingEnabled = data.value("swerling_enabled", cfg.swerlingEnabled);
            cfg.gimbalRateLimitRadPerSec = data.value("gimbal_rate_limit_rad_per_sec", cfg.gimbalRateLimitRadPerSec);
            cfg.minRangeGateM = data.value("min_range_gate_m", cfg.minRangeGateM);
            cfg.maxRangeGateM = data.value("max_range_gate_m", cfg.maxRangeGateM);
            cfg.terrainMaskingEnabled = data.value("terrain_masking_enabled", cfg.terrainMaskingEnabled);
            cfg.minClosingRateMps = data.value("min_closing_rate_mps", cfg.minClosingRateMps);
            cfg.rateFilterTauSec = data.value("rate_filter_tau_sec", cfg.rateFilterTauSec);
            cfg.decoyRejectionDb = data.value("decoy_rejection_db", cfg.decoyRejectionDb);
            cfg.passiveRfDutyCycle = data.value("passive_rf_duty_cycle", cfg.passiveRfDutyCycle);
            cfg.illuminatorEntityId = data.value("illuminator_entity_id", cfg.illuminatorEntityId);
            cfg.aperturePositionX = data.value("aperture_position_x", cfg.aperturePositionX);
            cfg.aperturePositionY = data.value("aperture_position_y", cfg.aperturePositionY);
            cfg.aperturePositionZ = data.value("aperture_position_z", cfg.aperturePositionZ);
            cfg.apertureNormalX = data.value("aperture_normal_x", cfg.apertureNormalX);
            cfg.apertureNormalY = data.value("aperture_normal_y", cfg.apertureNormalY);
            cfg.apertureNormalZ = data.value("aperture_normal_z", cfg.apertureNormalZ);
            _seeker = cfg;
        } catch (const std::exception& e) {
            if (error) *error = e.what();
            return false;
        }

        return true;
    }

    const SeekerConfig& SeekerProfileDatabase::seeker() const {
        return _seeker;
    }

} // namespace StrikeEngine::Kernel
