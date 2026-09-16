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
            cfg.navScheduleEnabled = getBoolKey(data, "nav_schedule_enabled", "navScheduleEnabled", cfg.navScheduleEnabled);
            cfg.navConstantTerminal = getDoubleKey(data, "nav_constant_terminal", "navConstantTerminal", cfg.navConstantTerminal);
            cfg.navScheduleTgoSec = getDoubleKey(data, "nav_schedule_tgo_sec", "navScheduleTgoSec", cfg.navScheduleTgoSec);
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
            cfg.trackFilterEnabled = getBoolKey(data, "track_filter_enabled", "trackFilterEnabled", cfg.trackFilterEnabled);
            cfg.trackProcessNoiseMps2 = getDoubleKey(data, "track_process_noise_mps2", "trackProcessNoiseMps2", cfg.trackProcessNoiseMps2);
            cfg.trackAngleStdRad = getDoubleKey(data, "track_angle_std_rad", "trackAngleStdRad", cfg.trackAngleStdRad);
            cfg.trackMeasNoiseScale = getDoubleKey(data, "track_meas_noise_scale", "trackMeasNoiseScale", cfg.trackMeasNoiseScale);
            cfg.trackResidualGateSigma = getDoubleKey(data, "track_residual_gate_sigma", "trackResidualGateSigma", cfg.trackResidualGateSigma);
            cfg.trackMaxAccelMps2 = getDoubleKey(data, "track_max_accel_mps2", "trackMaxAccelMps2", cfg.trackMaxAccelMps2);
            cfg.trackRetargetConfirmations = getIntKey(data, "track_retarget_confirmations", "trackRetargetConfirmations", cfg.trackRetargetConfirmations);
            cfg.trackSeedPolicy = getIntKey(data, "track_seed_policy", "trackSeedPolicy", cfg.trackSeedPolicy);
            cfg.trackMinQuality01 = getDoubleKey(data, "track_min_quality01", "trackMinQuality01", cfg.trackMinQuality01);
            cfg.trackQualityTauSec = getDoubleKey(data, "track_quality_tau_sec", "trackQualityTauSec", cfg.trackQualityTauSec);
            cfg.trackVelocityBlend = getDoubleKey(data, "track_velocity_blend", "trackVelocityBlend", cfg.trackVelocityBlend);

            cfg.trajectoryMinSpeedMps = getDoubleKey(data, "trajectory_min_speed_mps", "trajectoryMinSpeedMps", cfg.trajectoryMinSpeedMps);
            cfg.trajectoryFeasibilityAccelFactor = getDoubleKey(data, "trajectory_feasibility_accel_factor", "trajectoryFeasibilityAccelFactor", cfg.trajectoryFeasibilityAccelFactor);
            cfg.guidanceCommandLagSec = getDoubleKey(data, "guidance_command_lag_sec", "guidanceCommandLagSec", cfg.guidanceCommandLagSec);
            cfg.guidanceCommandSlewLimitMps3 = getDoubleKey(data, "guidance_command_slew_limit_mps3", "guidanceCommandSlewLimitMps3", cfg.guidanceCommandSlewLimitMps3);
            cfg.guidanceScaleDemandOnInfeasible = getBoolKey(data, "guidance_scale_demand_on_infeasible", "guidanceScaleDemandOnInfeasible", cfg.guidanceScaleDemandOnInfeasible);
            cfg.guidanceRangeGainShapingEnabled = getBoolKey(data, "guidance_range_gain_shaping_enabled", "guidanceRangeGainShapingEnabled", cfg.guidanceRangeGainShapingEnabled);
            cfg.guidanceRangeGainRefM = getDoubleKey(data, "guidance_range_gain_ref_m", "guidanceRangeGainRefM", cfg.guidanceRangeGainRefM);
            cfg.guidanceTrackAimMinQuality01 = getDoubleKey(data, "guidance_track_aim_min_quality01", "guidanceTrackAimMinQuality01", cfg.guidanceTrackAimMinQuality01);
            cfg.guidanceApnFeedforwardMinQuality01 = getDoubleKey(data, "guidance_apn_feedforward_min_quality01", "guidanceApnFeedforwardMinQuality01", cfg.guidanceApnFeedforwardMinQuality01);
            cfg.guidanceLoftEnabled = getBoolKey(data, "guidance_loft_enabled", "guidanceLoftEnabled", cfg.guidanceLoftEnabled);
            cfg.guidanceLoftAngleDeg = getDoubleKey(data, "guidance_loft_angle_deg", "guidanceLoftAngleDeg", cfg.guidanceLoftAngleDeg);
            cfg.guidanceLoftAltitudeM = getDoubleKey(data, "guidance_loft_altitude_m", "guidanceLoftAltitudeM", cfg.guidanceLoftAltitudeM);
            cfg.guidanceLoftGain = getDoubleKey(data, "guidance_loft_gain", "guidanceLoftGain", cfg.guidanceLoftGain);
            cfg.guidanceLoftRangeM = getDoubleKey(data, "guidance_loft_range_m", "guidanceLoftRangeM", cfg.guidanceLoftRangeM);

            cfg.gainSchedulingEnabled = getBoolKey(data, "gain_scheduling_enabled", "gainSchedulingEnabled", cfg.gainSchedulingEnabled);
            cfg.refDynamicPressurePa = getDoubleKey(data, "ref_dynamic_pressure_pa", "refDynamicPressurePa", cfg.refDynamicPressurePa);
            cfg.kRatePitchP = getDoubleKey(data, "k_rate_pitch_p", "kRatePitchP", cfg.kRatePitchP);
            cfg.kRateYawP = getDoubleKey(data, "k_rate_yaw_p", "kRateYawP", cfg.kRateYawP);
            cfg.scheduleAllTerms = getBoolKey(data, "schedule_all_terms", "scheduleAllTerms", cfg.scheduleAllTerms);
            cfg.autopilotIntegralEnabled = getBoolKey(data, "autopilot_integral_enabled", "autopilotIntegralEnabled", cfg.autopilotIntegralEnabled);
            cfg.kIntegralPitch = getDoubleKey(data, "k_integral_pitch", "kIntegralPitch", cfg.kIntegralPitch);
            cfg.kIntegralYaw = getDoubleKey(data, "k_integral_yaw", "kIntegralYaw", cfg.kIntegralYaw);
            cfg.integralClampRad = getDoubleKey(data, "integral_clamp_rad", "integralClampRad", cfg.integralClampRad);
            cfg.controlEffectivenessEnabled = getBoolKey(data, "control_effectiveness_enabled", "controlEffectivenessEnabled", cfg.controlEffectivenessEnabled);
            cfg.controlEffBase = getDoubleKey(data, "control_eff_base", "controlEffBase", cfg.controlEffBase);
            cfg.controlEffMachSlope = getDoubleKey(data, "control_eff_mach_slope", "controlEffMachSlope", cfg.controlEffMachSlope);
            cfg.controlEffMachQuad = getDoubleKey(data, "control_eff_mach_quad", "controlEffMachQuad", cfg.controlEffMachQuad);
            cfg.controlEffMin = getDoubleKey(data, "control_eff_min", "controlEffMin", cfg.controlEffMin);
            cfg.controlEffMax = getDoubleKey(data, "control_eff_max", "controlEffMax", cfg.controlEffMax);
            cfg.yawDeadbandSmoothEnabled = getBoolKey(data, "yaw_deadband_smooth_enabled", "yawDeadbandSmoothEnabled", cfg.yawDeadbandSmoothEnabled);
            cfg.yawDeadbandWidthMps2 = getDoubleKey(data, "yaw_deadband_width_mps2", "yawDeadbandWidthMps2", cfg.yawDeadbandWidthMps2);
            cfg.commandLagSec = getDoubleKey(data, "command_lag_sec", "commandLagSec", cfg.commandLagSec);
            cfg.commandRateLimitRadPerSec = getDoubleKey(data, "command_rate_limit_rad_per_sec", "commandRateLimitRadPerSec", cfg.commandRateLimitRadPerSec);
            cfg.useMeasuredRatesEnabled = getBoolKey(data, "use_measured_rates_enabled", "useMeasuredRatesEnabled", cfg.useMeasuredRatesEnabled);
            cfg.rollSuppressLateralAccelMps2 = getDoubleKey(data, "roll_suppress_lateral_accel_mps2", "rollSuppressLateralAccelMps2", cfg.rollSuppressLateralAccelMps2);
            cfg.useTruthGravityModel = getBoolKey(data, "use_truth_gravity_model", "useTruthGravityModel", cfg.useTruthGravityModel);
            cfg.guidanceAuthorityAwareLimitEnabled = getBoolKey(data, "guidance_authority_aware_limit_enabled", "guidanceAuthorityAwareLimitEnabled", cfg.guidanceAuthorityAwareLimitEnabled);
            cfg.minDynamicPressurePa = getDoubleKey(data, "min_dynamic_pressure_pa", "minDynamicPressurePa", cfg.minDynamicPressurePa);
            cfg.maxDynamicPressurePa = getDoubleKey(data, "max_dynamic_pressure_pa", "maxDynamicPressurePa", cfg.maxDynamicPressurePa);

            // Cruise + datalink + TPN-G fields: serializeVehicleConfig writes
            // all of these, so the profile path must read them back too or a
            // Cruise/datalink designer profile silently loads with altitude
            // hold and cooperative aiming disabled.
            cfg.cruiseAltitudeM = getDoubleKey(data, "cruise_altitude_m", "cruiseAltitudeM", cfg.cruiseAltitudeM);
            cfg.cruiseAltitudeGain = getDoubleKey(data, "cruise_altitude_gain", "cruiseAltitudeGain", cfg.cruiseAltitudeGain);
            cfg.cruiseAltitudeDamping = getDoubleKey(data, "cruise_altitude_damping", "cruiseAltitudeDamping", cfg.cruiseAltitudeDamping);
            cfg.cruiseWaypointGain = getDoubleKey(data, "cruise_waypoint_gain", "cruiseWaypointGain", cfg.cruiseWaypointGain);
            cfg.gravityCompensationEnabled = getBoolKey(data, "gravity_compensation_enabled", "gravityCompensationEnabled", cfg.gravityCompensationEnabled);
            cfg.datalinkSourceId = getIntKey(data, "datalink_source_id", "datalinkSourceId", cfg.datalinkSourceId);
            cfg.datalinkTargetId = getIntKey(data, "datalink_target_id", "datalinkTargetId", cfg.datalinkTargetId);

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
