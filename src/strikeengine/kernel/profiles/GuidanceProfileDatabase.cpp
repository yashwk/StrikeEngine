#include <strikeengine/kernel/profiles/GuidanceProfileDatabase.hpp>
#include <nlohmann/json.hpp>
#include <fstream>

namespace StrikeEngine::Kernel {

    static double getDoubleKey(const nlohmann::json& j, const char* snake, const char* camel, double def) {
        if (j.contains(snake)) return j.at(snake).get<double>();
        if (j.contains(camel)) return j.at(camel).get<double>();
        return def;
    }

    static int getIntKey(const nlohmann::json& j, const char* snake, const char* camel, int def) {
        if (j.contains(snake)) return j.at(snake).get<int>();
        if (j.contains(camel)) return j.at(camel).get<int>();
        return def;
    }

    static bool getBoolKey(const nlohmann::json& j, const char* snake, const char* camel, bool def) {
        if (j.contains(snake)) return j.at(snake).get<bool>();
        if (j.contains(camel)) return j.at(camel).get<bool>();
        return def;
    }

    bool GuidanceProfileDatabase::loadProfile(const std::string& file_path) {
        std::ifstream f(file_path);
        if (!f.is_open()) {
            return false;
        }

        try {
            nlohmann::json data = nlohmann::json::parse(f);

            GuidanceAutopilotConfig cfg;
            cfg.navigationConstant = getDoubleKey(data, "navigation_constant", "navigationConstant", cfg.navigationConstant);
            cfg.waypointGain = getDoubleKey(data, "waypoint_gain", "waypointGain", cfg.waypointGain);
            cfg.kAccelP = getDoubleKey(data, "k_accel_p", "kAccelP", cfg.kAccelP);
            cfg.kRateP = getDoubleKey(data, "k_rate_p", "kRateP", cfg.kRateP);
            cfg.kAlphaP = getDoubleKey(data, "k_alpha_p", "kAlphaP", cfg.kAlphaP);
            cfg.kRollP = getDoubleKey(data, "k_roll_p", "kRollP", cfg.kRollP);
            cfg.kRollD = getDoubleKey(data, "k_roll_d", "kRollD", cfg.kRollD);
            cfg.maxDeflectionRad = getDoubleKey(data, "max_deflection_rad", "maxDeflectionRad", cfg.maxDeflectionRad);
            cfg.servoTimeConstantSec = getDoubleKey(data, "servo_time_constant_sec", "servoTimeConstantSec", cfg.servoTimeConstantSec);
            cfg.maxServoRateRadPerSec = getDoubleKey(data, "max_servo_rate_rad_per_sec", "maxServoRateRadPerSec", cfg.maxServoRateRadPerSec);

            cfg.handoffBlendTimeSec = getDoubleKey(data, "handoff_blend_time_sec", "handoffBlendTimeSec", cfg.handoffBlendTimeSec);
            cfg.lockLossRetentionSec = getDoubleKey(data, "lock_loss_retention_sec", "lockLossRetentionSec", cfg.lockLossRetentionSec);
            cfg.apnFeedforwardEnabled = getBoolKey(data, "apn_feedforward_enabled", "apnFeedforwardEnabled", cfg.apnFeedforwardEnabled);

            cfg.trackConfirmations = getIntKey(data, "track_confirmations", "trackConfirmations", cfg.trackConfirmations);
            cfg.trackCoastTimeoutSec = getDoubleKey(data, "track_coast_timeout_sec", "trackCoastTimeoutSec", cfg.trackCoastTimeoutSec);
            cfg.trackLossTimeoutSec = getDoubleKey(data, "track_loss_timeout_sec", "trackLossTimeoutSec", cfg.trackLossTimeoutSec);

            cfg.trajectoryMinSpeedMps = getDoubleKey(data, "trajectory_min_speed_mps", "trajectoryMinSpeedMps", cfg.trajectoryMinSpeedMps);
            cfg.trajectoryFeasibilityAccelFactor = getDoubleKey(data, "trajectory_feasibility_accel_factor", "trajectoryFeasibilityAccelFactor", cfg.trajectoryFeasibilityAccelFactor);

            cfg.gainSchedulingEnabled = getBoolKey(data, "gain_scheduling_enabled", "gainSchedulingEnabled", cfg.gainSchedulingEnabled);
            cfg.refDynamicPressurePa = getDoubleKey(data, "ref_dynamic_pressure_pa", "refDynamicPressurePa", cfg.refDynamicPressurePa);
            cfg.minDynamicPressurePa = getDoubleKey(data, "min_dynamic_pressure_pa", "minDynamicPressurePa", cfg.minDynamicPressurePa);
            cfg.maxDynamicPressurePa = getDoubleKey(data, "max_dynamic_pressure_pa", "maxDynamicPressurePa", cfg.maxDynamicPressurePa);

            _guidanceAutopilot = cfg;
        } catch (const std::exception&) {
            return false;
        }

        return true;
    }

    const GuidanceAutopilotConfig& GuidanceProfileDatabase::guidanceAutopilot() const {
        return _guidanceAutopilot;
    }

} // namespace StrikeEngine::Kernel
