#include <strikeengine/kernel/profiles/SeekerProfileDatabase.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

namespace StrikeEngine::Kernel {

namespace {

// Local copy of the ConfigSerialization seeker-type string map (never
// declares to_json/from_json for SeekerConfig in this TU; ODR safety).
SeekerType seekerTypeFromString(const std::string& s) {
    if (s == "none") return SeekerType::None;
    if (s == "rf") return SeekerType::RF;
    if (s == "ir") return SeekerType::IR;
    if (s == "passive_rf") return SeekerType::PassiveRF;
    if (s == "sarh") return SeekerType::SARH;
    throw std::runtime_error("SeekerProfileDatabase: unknown SeekerType string '" + s + "'");
}

} // namespace

    bool SeekerProfileDatabase::loadProfile(const std::string& file_path) {
        std::ifstream f(file_path);
        if (!f.is_open()) {
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
            _seeker = cfg;
        } catch (const std::exception&) {
            // Any load failure (syntax, missing required key, wrong type,
            // unknown type string) makes the profile unloadable;
            // createVehicle translates this into a friendly std::runtime_error.
            return false;
        }

        return true;
    }

    const SeekerConfig& SeekerProfileDatabase::seeker() const {
        return _seeker;
    }

} // namespace StrikeEngine::Kernel
