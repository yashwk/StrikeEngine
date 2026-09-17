#include <strikeengine/kernel/config/ConfigSerialization.hpp>
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/SeekerTypeStrings.hpp>
#include <strikeengine/kernel/config/AeroSchema.hpp>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace StrikeEngine::Kernel {

using json = nlohmann::json;

namespace {

// ---------------------------------------------------------------------------
// Enum <-> snake_case string maps
// ---------------------------------------------------------------------------

std::string entityTypeToString(EntityType t) {
    switch (t) {
        case EntityType::Missile: return "missile";
        case EntityType::Aircraft: return "aircraft";
        case EntityType::SurfaceTarget: return "surface_target";
        case EntityType::RadarSite: return "radar_site";
        case EntityType::Chaff: return "chaff";
        case EntityType::Flare: return "flare";
    }
    throw std::runtime_error("ConfigSerialization: unhandled EntityType");
}

EntityType entityTypeFromString(const std::string& s) {
    if (s == "missile") return EntityType::Missile;
    if (s == "aircraft") return EntityType::Aircraft;
    if (s == "surface_target") return EntityType::SurfaceTarget;
    if (s == "radar_site") return EntityType::RadarSite;
    if (s == "chaff") return EntityType::Chaff;
    if (s == "flare") return EntityType::Flare;
    throw std::runtime_error("ConfigSerialization: unknown EntityType string '" + s + "'");
}

std::string allegianceToString(Allegiance a) {
    switch (a) {
        case Allegiance::Friendly: return "friendly";
        case Allegiance::Hostile: return "hostile";
        case Allegiance::Neutral: return "neutral";
    }
    throw std::runtime_error("ConfigSerialization: unhandled Allegiance");
}

Allegiance allegianceFromString(const std::string& s) {
    if (s == "friendly") return Allegiance::Friendly;
    if (s == "hostile") return Allegiance::Hostile;
    if (s == "neutral") return Allegiance::Neutral;
    throw std::runtime_error("ConfigSerialization: unknown Allegiance string '" + s + "'");
}

std::string fusingTypeToString(FusingType t) {
    switch (t) {
        case FusingType::Impact: return "impact";
        case FusingType::Proximity: return "proximity";
        case FusingType::Timed: return "timed";
    }
    throw std::runtime_error("ConfigSerialization: unhandled FusingType");
}

FusingType fusingTypeFromString(const std::string& s) {
    if (s == "impact") return FusingType::Impact;
    if (s == "proximity") return FusingType::Proximity;
    if (s == "timed") return FusingType::Timed;
    throw std::runtime_error("ConfigSerialization: unknown FusingType string '" + s + "'");
}

std::string guidanceModeToString(GuidanceMode m) {
    switch (m) {
        case GuidanceMode::None: return "none";
        case GuidanceMode::ProportionalNavigation: return "proportional_navigation";
        case GuidanceMode::Waypoint: return "waypoint";
        case GuidanceMode::Trajectory: return "trajectory";
        case GuidanceMode::Cruise: return "cruise";
    }
    throw std::runtime_error("ConfigSerialization: unhandled GuidanceMode");
}

GuidanceMode guidanceModeFromString(const std::string& s) {
    if (s == "none") return GuidanceMode::None;
    if (s == "proportional_navigation") return GuidanceMode::ProportionalNavigation;
    if (s == "waypoint") return GuidanceMode::Waypoint;
    if (s == "trajectory") return GuidanceMode::Trajectory;
    if (s == "cruise") return GuidanceMode::Cruise;
    throw std::runtime_error("ConfigSerialization: unknown GuidanceMode string '" + s + "'");
}

// ---------------------------------------------------------------------------
// Parsing / file I/O helpers
// ---------------------------------------------------------------------------

json parseJsonText(const std::string& jsonText, const char* what) {
    try {
        return json::parse(jsonText);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("ConfigSerialization: failed to parse ") +
                                 what + ": " + e.what());
    }
}

std::string readTextFile(const std::string& path, const char* what) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error(std::string("ConfigSerialization: cannot open ") +
                                 what + " file '" + path + "'");
    }
    std::ostringstream buffer;
    buffer << f.rdbuf();
    return buffer.str();
}

bool isBlank(const std::string& s) {
    for (char c : s) {
        if (c != ' ' && c != '\t' && c != '\n' && c != '\r' && c != '\f' && c != '\v') {
            return false;
        }
    }
    return true;
}

template <typename T>
T getChecked(const json& j, const char* what) {
    try {
        return j.get<T>();
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("ConfigSerialization: invalid ") +
                                 what + ": " + e.what());
    }
}

} // namespace

} // namespace StrikeEngine::Kernel

namespace StrikeEngine::Models {

// ThrustDataPoint is in StrikeEngine::Models; ADL finds these overloads when
// (de)serializing the thrust_curve vector inside StageConfig.
void to_json(nlohmann::json& j, const ThrustDataPoint& p) {
    j = nlohmann::json{{"time_s", p.time_s}, {"thrust_n", p.thrust_n}};
}

void from_json(const nlohmann::json& j, ThrustDataPoint& p) {
    p.time_s = j.at("time_s").get<double>();
    p.thrust_n = j.at("thrust_n").get<double>();
}

void to_json(nlohmann::json& j, const AeroTables& t) {
    j = nlohmann::json{
        {"mach_breakpoints", t.machBreakpoints},
        {"aoa_breakpoints_rad", t.aoaBreakpointsRad},
        {"cl_table", t.clTable},
        {"cd_table", t.cdTable}
    };
    if (!t.cmTable.empty()) {
        j["cm_table"] = t.cmTable;
    }
    if (!t.betaBreakpointsRad.empty()) {
        j["beta_breakpoints_rad"] = t.betaBreakpointsRad;
    }
    if (!t.cyTable.empty()) {
        j["cy_table"] = t.cyTable;
    }
    if (!t.cnTable.empty()) {
        j["cn_table"] = t.cnTable;
    }
    if (!t.clRollTable.empty()) {
        j["cl_roll_table"] = t.clRollTable;
    }
}

void from_json(const nlohmann::json& j, AeroTables& t) {
    t.machBreakpoints = j.at("mach_breakpoints").get<std::vector<double>>();
    t.aoaBreakpointsRad = j.at("aoa_breakpoints_rad").get<std::vector<double>>();
    t.clTable = j.at("cl_table").get<std::vector<std::vector<double>>>();
    t.cdTable = j.at("cd_table").get<std::vector<std::vector<double>>>();
    if (j.contains("cm_table")) {
        t.cmTable = j.at("cm_table").get<std::vector<std::vector<double>>>();
    }
    if (j.contains("beta_breakpoints_rad")) {
        t.betaBreakpointsRad = j.at("beta_breakpoints_rad").get<std::vector<double>>();
    }
    if (j.contains("cy_table")) {
        t.cyTable = j.at("cy_table").get<std::vector<std::vector<double>>>();
    }
    if (j.contains("cn_table")) {
        t.cnTable = j.at("cn_table").get<std::vector<std::vector<double>>>();
    }
    if (j.contains("cl_roll_table")) {
        t.clRollTable = j.at("cl_roll_table").get<std::vector<std::vector<double>>>();
    }
    std::string err;
    if (!t.isValid(&err)) {
        throw std::runtime_error("AeroTables invalid: " + err);
    }
}

} // namespace StrikeEngine::Models

namespace StrikeEngine::Kernel {

// ---------------------------------------------------------------------------
// to_json / from_json for the config structs (internal to this translation
// unit; never declared in public headers).
// ---------------------------------------------------------------------------

// --- Pure-scalar structs ---------------------------------------------------

// --- AeroConfig -------------------------------------------------------------

static json finConfigToJson(const FinsConfig& fin) {
    json f = json::object();
    switch (fin.shape) {
        case Models::FinShape::Trapezoidal: f["shape"] = "trapezoidal"; break;
        case Models::FinShape::Elliptical:  f["shape"] = "elliptical";  break;
        case Models::FinShape::Delta:       f["shape"] = "delta";       break;
        case Models::FinShape::Cranked:     f["shape"] = "cranked";     break;
        case Models::FinShape::FreeForm:    f["shape"] = "freeform";    break;
    }
    f["count"] = fin.count;
    f["position_m"] = fin.positionM;
    f["cant_angle_deg"] = fin.cantAngleDeg;
    f["root_chord_m"] = fin.rootChordM;
    f["span_m"] = fin.spanM;
    f["steerable"] = fin.steerable;
    switch (fin.airfoil) {
        case Models::FinAirfoil::FlatPlate:   f["airfoil"] = "flat_plate";   break;
        case Models::FinAirfoil::DoubleWedge: f["airfoil"] = "double_wedge"; break;
        case Models::FinAirfoil::Biconvex:    f["airfoil"] = "biconvex";     break;
        case Models::FinAirfoil::Hexagonal:   f["airfoil"] = "hexagonal";    break;
    }
    f["thickness_ratio"] = fin.thicknessRatio;
    f["max_thickness_location"] = fin.maxThicknessLocation;
    f["leading_edge_radius"] = fin.leadingEdgeRadius;
    f["trailing_edge_thickness"] = fin.trailingEdgeThickness;
    if (fin.shape != Models::FinShape::Delta) {
        f["tip_chord_m"] = fin.tipChordM;
    }
    if (fin.shape == Models::FinShape::Trapezoidal ||
        fin.shape == Models::FinShape::Delta ||
        fin.shape == Models::FinShape::Cranked) {
        f["sweep_length_m"] = fin.sweepLengthM;
    }
    if (fin.shape == Models::FinShape::Cranked) {
        f["crank_fraction"] = fin.crankFraction;
        f["crank_chord_factor"] = fin.crankChordFactor;
    }
    if (fin.shape == Models::FinShape::FreeForm) {
        f["shape_points"] = fin.shapePoints;
    }
    f["control_type"] = (fin.controlType == 1) ? "trailing_edge_flap" : "all_moving";
    f["control_fraction"] = fin.controlFraction;
    return f;
}

static FinsConfig finConfigFromJson(const json& f) {
    return AeroSchema::finsFromJson(f, "AeroConfig fins");
}

void to_json(json& j, const AeroConfig& a) {
    j = json{
        {"reference_area",   a.referenceArea},
        {"reference_length", a.referenceLength},
        {"cd",               a.cd},
        {"cl_alpha",         a.clAlpha},
        {"cl_fin",           a.clFin},
        {"cl_max",           a.clMax},
        // Abstract-fin surface type; absent = canard (historic default).
        {"tail_control",     a.tailControl}
    };
    if (!a.tables.empty()) {
        j["aero_tables"] = a.tables;
    }
    if (!a.finSets.empty()) {
        json arr = json::array();
        // Serialize every set, including disabled ones (count < 3): the
        // round-trip must be faithful even though disabled sets are inert.
        for (const auto& fs : a.finSets) {
            arr.push_back(finConfigToJson(fs));
        }
        j["fin_sets"] = std::move(arr);
        for (const auto& fs : a.finSets) {
            if (fs.enabled()) {
                j["fins"] = finConfigToJson(fs);
                break;
            }
        }
    } else if (a.fins.enabled()) {
        j["fins"] = finConfigToJson(a.fins);
    }
    if (a.airframe.enabled()) {
        j["airframe"] = json{
            {"wing_span_m",       a.airframe.wingSpanM},
            {"wing_root_chord_m", a.airframe.wingRootChordM},
            {"wing_tip_chord_m",  a.airframe.wingTipChordM},
            {"wing_sweep_deg",    a.airframe.wingSweepDeg},
            {"wing_position_m",   a.airframe.wingPositionM},
            {"wing_dihedral_deg", a.airframe.wingDihedralDeg},
            {"htail_span_m",      a.airframe.htailSpanM},
            {"htail_chord_m",     a.airframe.htailChordM},
            {"htail_position_m",  a.airframe.htailPositionM},
            {"vtail_span_m",      a.airframe.vtailSpanM},
            {"vtail_chord_m",     a.airframe.vtailChordM},
            {"vtail_position_m",  a.airframe.vtailPositionM},
            {"fuselage_diameter_m", a.airframe.fuselageDiameterM},
            {"fuselage_length_m",   a.airframe.fuselageLengthM},
            {"cd0",               a.airframe.cd0},
            {"oswald_efficiency", a.airframe.oswaldEfficiency},
            {"cl_max",            a.airframe.clMax}
        };
    }
}

void from_json(const json& j, AeroConfig& a) {
    a.referenceArea = j.at("reference_area").get<double>();
    a.referenceLength = j.at("reference_length").get<double>();
    a.cd = j.at("cd").get<double>();
    a.clAlpha = j.at("cl_alpha").get<double>();
    a.clFin = j.at("cl_fin").get<double>();
    a.clMax = j.at("cl_max").get<double>();
    a.tailControl = j.value("tail_control", j.value("tailControl", false));
    // Optional: existing files without aero_tables must still load.
    if (j.contains("aero_tables")) {
        a.tables = j.at("aero_tables").get<Models::AeroTables>();
    }
    if (j.contains("fin_sets") && j.at("fin_sets").is_array()) {
        for (const auto& item : j.at("fin_sets")) {
            a.finSets.push_back(finConfigFromJson(item));
        }
    }
    if (j.contains("fins")) {
        a.fins = finConfigFromJson(j.at("fins"));
    }
    if (j.contains("airframe")) {
        AeroSchema::airframeFromJson(j.at("airframe"), a.airframe);
    }
}

// --- SensorConfig -----------------------------------------------------------

void to_json(json& j, const SensorConfig& s) {
    j = json();
    j["imu_enabled"] = s.imuEnabled;
    j["gps_enabled"] = s.gpsEnabled;
    j["accel_noise_std_dev"] = s.accelNoiseStdDev;
    j["accel_bias_std_dev"] = s.accelBiasStdDev;
    j["gyro_noise_std_dev"] = s.gyroNoiseStdDev;
    j["gyro_bias_std_dev"] = s.gyroBiasStdDev;
    j["gps_pos_noise_std_dev"] = s.gpsPosNoiseStdDev;
    j["gps_vel_noise_std_dev"] = s.gpsVelNoiseStdDev;
    j["gps_update_rate_hz"] = s.gpsUpdateRateHz;
    j["gps_innovation_gate_sigma"] = s.gpsInnovationGateSigma;
    j["baro_innovation_gate_sigma"] = s.baroInnovationGateSigma;
    j["mag_innovation_gate_sigma"] = s.magInnovationGateSigma;
    j["imu_lever_arm_x"] = s.imuLeverArmX;
    j["imu_lever_arm_y"] = s.imuLeverArmY;
    j["imu_lever_arm_z"] = s.imuLeverArmZ;
    // Aiding + realism keys (optional with defaults; missing = legacy).
    j["baro_enabled"] = s.baroEnabled;
    j["baro_noise_std_dev"] = s.baroNoiseStdDev;
    j["baro_bias_std_dev"] = s.baroBiasStdDev;
    j["baro_update_rate_hz"] = s.baroUpdateRateHz;
    j["mag_enabled"] = s.magEnabled;
    j["mag_noise_std_dev"] = s.magNoiseStdDev;
    j["mag_update_rate_hz"] = s.magUpdateRateHz;
    j["mag_disturbance_gate_rel"] = s.magDisturbanceGateRel;
    j["gps_latency_sec"] = s.gpsLatencySec;
    j["gps_lever_arm_x"] = s.gpsLeverArmX;
    j["gps_lever_arm_y"] = s.gpsLeverArmY;
    j["gps_lever_arm_z"] = s.gpsLeverArmZ;
    j["gps_fix_consistency_enabled"] = s.gpsFixConsistencyEnabled;
    j["ins_coning_compensation_enabled"] = s.insConingCompensationEnabled;
    j["ins_adaptive_q_enabled"] = s.insAdaptiveQEnabled;
    j["ins_adaptive_q_gain"] = s.insAdaptiveQGain;
    j["initial_attitude_error_deg"] = s.initialAttitudeErrorDeg;
    j["initial_position_error_m"] = s.initialPositionErrorM;
    j["initial_velocity_error_mps"] = s.initialVelocityErrorMps;
    // INS error-model fidelity + GPS fusion (optional; defaults = legacy).
    j["ins_gravity_gradient_enabled"] = s.insGravityGradientEnabled;
    j["ins_earth_rotation_coupling_enabled"] = s.insEarthRotationCouplingEnabled;
    j["gps_batch_update_enabled"] = s.gpsBatchUpdateEnabled;
    j["gps_lever_arm_compensation_enabled"] = s.gpsLeverArmCompensationEnabled;
    j["gps_yaw_correction_damping"] = s.gpsYawCorrectionDamping;
    j["gps_fix_consistency_threshold"] = s.gpsFixConsistencyThreshold;
    j["gps_fix_consistency_confidence"] = s.gpsFixConsistencyConfidence;
    j["gps_fix_consistency_dof"] = s.gpsFixConsistencyDof;
    j["max_accel_bias_estimate"] = s.maxAccelBiasEstimate;
    j["max_gyro_bias_estimate"] = s.maxGyroBiasEstimate;
    j["baro_attitude_correction_enabled"] = s.baroAttitudeCorrectionEnabled;
}

void from_json(const json& j, SensorConfig& s) {
    s.imuEnabled = j.at("imu_enabled").get<bool>();
    s.gpsEnabled = j.at("gps_enabled").get<bool>();
    s.accelNoiseStdDev = j.at("accel_noise_std_dev").get<double>();
    s.accelBiasStdDev = j.at("accel_bias_std_dev").get<double>();
    s.gyroNoiseStdDev = j.at("gyro_noise_std_dev").get<double>();
    s.gyroBiasStdDev = j.at("gyro_bias_std_dev").get<double>();
    s.gpsPosNoiseStdDev = j.at("gps_pos_noise_std_dev").get<double>();
    s.gpsVelNoiseStdDev = j.at("gps_vel_noise_std_dev").get<double>();
    s.gpsUpdateRateHz = j.at("gps_update_rate_hz").get<double>();
    s.gpsInnovationGateSigma = j.value("gps_innovation_gate_sigma", 5.0);
    s.baroInnovationGateSigma = j.value("baro_innovation_gate_sigma", -1.0);
    s.magInnovationGateSigma = j.value("mag_innovation_gate_sigma", -1.0);
    s.imuLeverArmX = j.at("imu_lever_arm_x").get<double>();
    s.imuLeverArmY = j.at("imu_lever_arm_y").get<double>();
    s.imuLeverArmZ = j.at("imu_lever_arm_z").get<double>();
    s.baroEnabled = j.value("baro_enabled", false);
    s.baroNoiseStdDev = j.value("baro_noise_std_dev", 1.0);
    s.baroBiasStdDev = j.value("baro_bias_std_dev", 0.0);
    s.baroUpdateRateHz = j.value("baro_update_rate_hz", 1.0);
    s.magEnabled = j.value("mag_enabled", false);
    s.magNoiseStdDev = j.value("mag_noise_std_dev", 50e-9);
    s.magUpdateRateHz = j.value("mag_update_rate_hz", 10.0);
    s.magDisturbanceGateRel = j.value("mag_disturbance_gate_rel", 0.25);
    s.gpsLatencySec = j.value("gps_latency_sec", 0.0);
    s.gpsLeverArmX = j.value("gps_lever_arm_x", 0.0);
    s.gpsLeverArmY = j.value("gps_lever_arm_y", 0.0);
    s.gpsLeverArmZ = j.value("gps_lever_arm_z", 0.0);
    s.gpsFixConsistencyEnabled = j.value("gps_fix_consistency_enabled", false);
    s.insConingCompensationEnabled = j.value("ins_coning_compensation_enabled", false);
    s.insAdaptiveQEnabled = j.value("ins_adaptive_q_enabled", false);
    s.insAdaptiveQGain = j.value("ins_adaptive_q_gain", 1.0);
    s.initialAttitudeErrorDeg = j.value("initial_attitude_error_deg", 0.0);
    s.initialPositionErrorM = j.value("initial_position_error_m", 0.0);
    s.initialVelocityErrorMps = j.value("initial_velocity_error_mps", 0.0);
    s.insGravityGradientEnabled = j.value("ins_gravity_gradient_enabled", false);
    s.insEarthRotationCouplingEnabled = j.value("ins_earth_rotation_coupling_enabled", false);
    s.gpsBatchUpdateEnabled = j.value("gps_batch_update_enabled", false);
    s.gpsLeverArmCompensationEnabled = j.value("gps_lever_arm_compensation_enabled", false);
    s.gpsYawCorrectionDamping = j.value("gps_yaw_correction_damping", 0.1);
    s.gpsFixConsistencyThreshold = j.value("gps_fix_consistency_threshold", 16.81);
    s.gpsFixConsistencyConfidence = j.value("gps_fix_consistency_confidence", 0.99);
    s.gpsFixConsistencyDof = j.value("gps_fix_consistency_dof", 6);
    s.maxAccelBiasEstimate = j.value("max_accel_bias_estimate", 0.5);
    s.maxGyroBiasEstimate = j.value("max_gyro_bias_estimate", 0.02);
    s.baroAttitudeCorrectionEnabled = j.value("baro_attitude_correction_enabled", false);
}

void to_json(json& j, const GuidanceAutopilotConfig& g) {
    j = json::object();
    j["navigationConstant"] = g.navigationConstant;
    j["waypointGain"] = g.waypointGain;
    j["kAccelP"] = g.kAccelP;
    j["kRateP"] = g.kRateP;
    j["kAlphaP"] = g.kAlphaP;
    j["kRollP"] = g.kRollP;
    j["kRollD"] = g.kRollD;
    j["maxDeflectionRad"] = g.maxDeflectionRad;
    j["servoTimeConstantSec"] = g.servoTimeConstantSec;
    j["maxServoRateRadPerSec"] = g.maxServoRateRadPerSec;
    // Phase-manager keys. Optional with legacy defaults on read, so old
    // design manifests / scenarios without them keep the abrupt-override path.
    j["handoffBlendTimeSec"] = g.handoffBlendTimeSec;
    j["lockLossRetentionSec"] = g.lockLossRetentionSec;
    j["apnFeedforwardEnabled"] = g.apnFeedforwardEnabled;
    j["gravityCompensationEnabled"] = g.gravityCompensationEnabled;
    // Track-manager keys (optional with defaults).
    j["trackConfirmations"] = g.trackConfirmations;
    j["trackCoastTimeoutSec"] = g.trackCoastTimeoutSec;
    j["trackLossTimeoutSec"] = g.trackLossTimeoutSec;
    j["trackFilterEnabled"] = g.trackFilterEnabled;
    j["trackProcessNoiseMps2"] = g.trackProcessNoiseMps2;
    j["trackAngleStdRad"] = g.trackAngleStdRad;
    j["trackMeasNoiseScale"] = g.trackMeasNoiseScale;
    j["trackResidualGateSigma"] = g.trackResidualGateSigma;
    j["trackMaxAccelMps2"] = g.trackMaxAccelMps2;
    j["trackRetargetConfirmations"] = g.trackRetargetConfirmations;
    j["trackSeedPolicy"] = g.trackSeedPolicy;
    j["trackMinQuality01"] = g.trackMinQuality01;
    j["trackQualityTauSec"] = g.trackQualityTauSec;
    j["trackVelocityBlend"] = g.trackVelocityBlend;
    // Trajectory-core keys (optional with defaults).
    j["trajectoryMinSpeedMps"] = g.trajectoryMinSpeedMps;
    j["trajectoryFeasibilityAccelFactor"] = g.trajectoryFeasibilityAccelFactor;
    j["guidanceCommandLagSec"] = g.guidanceCommandLagSec;
    j["guidanceCommandSlewLimitMps3"] = g.guidanceCommandSlewLimitMps3;
    j["guidanceScaleDemandOnInfeasible"] = g.guidanceScaleDemandOnInfeasible;
    j["guidanceRangeGainShapingEnabled"] = g.guidanceRangeGainShapingEnabled;
    j["guidanceRangeGainRefM"] = g.guidanceRangeGainRefM;
    j["guidanceTrackAimMinQuality01"] = g.guidanceTrackAimMinQuality01;
    j["guidanceApnFeedforwardMinQuality01"] = g.guidanceApnFeedforwardMinQuality01;
    j["guidanceLoftEnabled"] = g.guidanceLoftEnabled;
    j["guidanceLoftAngleDeg"] = g.guidanceLoftAngleDeg;
    j["guidanceLoftAltitudeM"] = g.guidanceLoftAltitudeM;
    j["guidanceLoftGain"] = g.guidanceLoftGain;
    j["guidanceLoftRangeM"] = g.guidanceLoftRangeM;
    // Cooperative-engagement datalink (optional with defaults).
    j["datalinkSourceId"] = g.datalinkSourceId;
    j["datalinkTargetId"] = g.datalinkTargetId;
    // tgo-scheduled N (optional with defaults; off = constant N).
    j["navScheduleEnabled"] = g.navScheduleEnabled;
    j["navConstantTerminal"] = g.navConstantTerminal;
    j["navScheduleTgoSec"] = g.navScheduleTgoSec;
    // Dynamic pressure gain scheduling keys (optional with defaults).
    j["gainSchedulingEnabled"] = g.gainSchedulingEnabled;
    j["refDynamicPressurePa"] = g.refDynamicPressurePa;
    j["kRatePitchP"] = g.kRatePitchP;
    j["kRateYawP"] = g.kRateYawP;
    j["scheduleAllTerms"] = g.scheduleAllTerms;
    j["autopilotIntegralEnabled"] = g.autopilotIntegralEnabled;
    j["kIntegralPitch"] = g.kIntegralPitch;
    j["kIntegralYaw"] = g.kIntegralYaw;
    j["integralClampRad"] = g.integralClampRad;
    j["kAccelErrP"] = g.kAccelErrP;
    j["autopilotThreeLoopEnabled"] = g.autopilotThreeLoopEnabled;
    j["controlEffectivenessEnabled"] = g.controlEffectivenessEnabled;
    j["controlEffBase"] = g.controlEffBase;
    j["controlEffMachSlope"] = g.controlEffMachSlope;
    j["controlEffMachQuad"] = g.controlEffMachQuad;
    j["controlEffMin"] = g.controlEffMin;
    j["controlEffMax"] = g.controlEffMax;
    j["yawDeadbandSmoothEnabled"] = g.yawDeadbandSmoothEnabled;
    j["yawDeadbandWidthMps2"] = g.yawDeadbandWidthMps2;
    j["commandLagSec"] = g.commandLagSec;
    j["commandRateLimitRadPerSec"] = g.commandRateLimitRadPerSec;
    j["useMeasuredRatesEnabled"] = g.useMeasuredRatesEnabled;
    j["rollSuppressLateralAccelMps2"] = g.rollSuppressLateralAccelMps2;
    j["useTruthGravityModel"] = g.useTruthGravityModel;
    j["guidanceAuthorityAwareLimitEnabled"] = g.guidanceAuthorityAwareLimitEnabled;
    j["minDynamicPressurePa"] = g.minDynamicPressurePa;
    j["maxDynamicPressurePa"] = g.maxDynamicPressurePa;
    // Aircraft cruise keys (optional with defaults; required so a Cruise
    // scenario round-trips instead of silently losing altitude-hold).
    j["cruiseAltitudeM"] = g.cruiseAltitudeM;
    j["cruiseAltitudeGain"] = g.cruiseAltitudeGain;
    j["cruiseAltitudeDamping"] = g.cruiseAltitudeDamping;
    j["cruiseWaypointGain"] = g.cruiseWaypointGain;
}

void from_json(const json& j, GuidanceAutopilotConfig& g) {
    g.navigationConstant = j.at("navigationConstant").get<double>();
    g.waypointGain = j.at("waypointGain").get<double>();
    g.kAccelP = j.at("kAccelP").get<double>();
    g.kRateP = j.at("kRateP").get<double>();
    g.kAlphaP = j.at("kAlphaP").get<double>();
    g.kRollP = j.at("kRollP").get<double>();
    g.kRollD = j.at("kRollD").get<double>();
    g.maxDeflectionRad = j.at("maxDeflectionRad").get<double>();
    g.servoTimeConstantSec = j.at("servoTimeConstantSec").get<double>();
    g.maxServoRateRadPerSec = j.at("maxServoRateRadPerSec").get<double>();
    g.handoffBlendTimeSec = j.value("handoffBlendTimeSec", 0.0);
    g.lockLossRetentionSec = j.value("lockLossRetentionSec", 0.0);
    g.apnFeedforwardEnabled = j.value("apnFeedforwardEnabled", false);
    g.gravityCompensationEnabled = j.value("gravityCompensationEnabled", false);
    g.trackConfirmations = j.value("trackConfirmations", 3);
    g.trackCoastTimeoutSec = j.value("trackCoastTimeoutSec", 0.5);
    g.trackLossTimeoutSec = j.value("trackLossTimeoutSec", 2.0);
    g.trackFilterEnabled = j.value("trackFilterEnabled", false);
    g.trackProcessNoiseMps2 = j.value("trackProcessNoiseMps2", 15.0);
    g.trackAngleStdRad = j.value("trackAngleStdRad", 0.003);
    g.trackMeasNoiseScale = j.value("trackMeasNoiseScale", 1.0);
    g.trackResidualGateSigma = j.value("trackResidualGateSigma", 0.0);
    g.trackMaxAccelMps2 = j.value("trackMaxAccelMps2", 0.0);
    g.trackRetargetConfirmations = j.value("trackRetargetConfirmations", 1);
    g.trackSeedPolicy = j.value("trackSeedPolicy", 0);
    g.trackMinQuality01 = j.value("trackMinQuality01", 0.0);
    g.trackQualityTauSec = j.value("trackQualityTauSec", 1.0);
    g.trackVelocityBlend = j.value("trackVelocityBlend", 0.08);
    g.trajectoryMinSpeedMps = j.value("trajectoryMinSpeedMps", 30.0);
    g.trajectoryFeasibilityAccelFactor = j.value("trajectoryFeasibilityAccelFactor", 0.95);
    g.guidanceCommandLagSec = j.value("guidanceCommandLagSec", 0.0);
    g.guidanceCommandSlewLimitMps3 = j.value("guidanceCommandSlewLimitMps3", 0.0);
    g.guidanceScaleDemandOnInfeasible = j.value("guidanceScaleDemandOnInfeasible", false);
    g.guidanceRangeGainShapingEnabled = j.value("guidanceRangeGainShapingEnabled", false);
    g.guidanceRangeGainRefM = j.value("guidanceRangeGainRefM", 10000.0);
    g.guidanceTrackAimMinQuality01 = j.value("guidanceTrackAimMinQuality01", 0.0);
    g.guidanceApnFeedforwardMinQuality01 = j.value("guidanceApnFeedforwardMinQuality01", 0.0);
    g.guidanceLoftEnabled = j.value("guidanceLoftEnabled", false);
    g.guidanceLoftAngleDeg = j.value("guidanceLoftAngleDeg", 0.0);
    g.guidanceLoftAltitudeM = j.value("guidanceLoftAltitudeM", 0.0);
    g.guidanceLoftGain = j.value("guidanceLoftGain", 0.0);
    g.guidanceLoftRangeM = j.value("guidanceLoftRangeM", 40000.0);
    g.datalinkSourceId = j.value("datalinkSourceId", -1);
    g.datalinkTargetId = j.value("datalinkTargetId", -1);
    g.navScheduleEnabled = j.value("navScheduleEnabled", false);
    g.navConstantTerminal = j.value("navConstantTerminal", 3.0);
    g.navScheduleTgoSec = j.value("navScheduleTgoSec", 8.0);
    g.gainSchedulingEnabled = j.value("gainSchedulingEnabled", false);
    g.refDynamicPressurePa = j.value("refDynamicPressurePa", 50000.0);
    g.kRatePitchP = j.value("kRatePitchP", -1.0);
    g.kRateYawP = j.value("kRateYawP", -1.0);
    g.scheduleAllTerms = j.value("scheduleAllTerms", false);
    g.autopilotIntegralEnabled = j.value("autopilotIntegralEnabled", false);
    g.kIntegralPitch = j.value("kIntegralPitch", 0.0);
    g.kIntegralYaw = j.value("kIntegralYaw", 0.0);
    g.integralClampRad = j.value("integralClampRad", 0.05);
    g.kAccelErrP = j.value("kAccelErrP", -1.0);
    g.autopilotThreeLoopEnabled = j.value("autopilotThreeLoopEnabled", false);
    g.controlEffectivenessEnabled = j.value("controlEffectivenessEnabled", false);
    g.controlEffBase = j.value("controlEffBase", 1.0);
    g.controlEffMachSlope = j.value("controlEffMachSlope", 0.0);
    g.controlEffMachQuad = j.value("controlEffMachQuad", 0.0);
    g.controlEffMin = j.value("controlEffMin", 0.2);
    g.controlEffMax = j.value("controlEffMax", 5.0);
    g.yawDeadbandSmoothEnabled = j.value("yawDeadbandSmoothEnabled", false);
    g.yawDeadbandWidthMps2 = j.value("yawDeadbandWidthMps2", 0.5);
    g.commandLagSec = j.value("commandLagSec", 0.0);
    g.commandRateLimitRadPerSec = j.value("commandRateLimitRadPerSec", 0.0);
    g.useMeasuredRatesEnabled = j.value("useMeasuredRatesEnabled", false);
    g.rollSuppressLateralAccelMps2 = j.value("rollSuppressLateralAccelMps2", 0.0);
    g.useTruthGravityModel = j.value("useTruthGravityModel", false);
    g.guidanceAuthorityAwareLimitEnabled = j.value("guidanceAuthorityAwareLimitEnabled", false);
    g.minDynamicPressurePa = j.value("minDynamicPressurePa", 2000.0);
    g.maxDynamicPressurePa = j.value("maxDynamicPressurePa", 300000.0);
    g.cruiseAltitudeM = j.value("cruiseAltitudeM", 0.0);
    g.cruiseAltitudeGain = j.value("cruiseAltitudeGain", 0.05);
    g.cruiseAltitudeDamping = j.value("cruiseAltitudeDamping", 0.30);
    g.cruiseWaypointGain = j.value("cruiseWaypointGain", 0.8);
}

// --- StageConfig / PropulsionConfig ----------------------------------------

void to_json(json& j, const StageConfig& s) {
    j = json();
    j["thrust_curve"] = s.thrustCurve;
    j["vacuum_isp"] = s.vacuumIsp;
    j["sea_level_isp"] = s.seaLevelIsp;
    j["propellant_mass_kg"] = s.propellantMassKg;
    j["dry_mass_kg"] = s.dryMassKg;
    j["ignition_delay_sec"] = s.ignitionDelaySec;
    j["ignition_ramp_sec"] = s.ignitionRampSec;
    j["shutdown_time_sec"] = s.shutdownTimeSec;
    j["shutdown_ramp_sec"] = s.shutdownRampSec;
    j["max_gimbal_pitch_rad"] = s.maxGimbalPitchRad;
    j["max_gimbal_yaw_rad"] = s.maxGimbalYawRad;
    j["gimbal_time_constant_sec"] = s.gimbalTimeConstantSec;
    j["max_gimbal_rate_rad_per_sec"] = s.maxGimbalRateRadPerSec;
    j["engine_position_x"] = s.enginePositionX;
    j["engine_position_y"] = s.enginePositionY;
    j["engine_position_z"] = s.enginePositionZ;
}

void from_json(const json& j, StageConfig& s) {
    s.thrustCurve = j.at("thrust_curve").get<std::vector<Models::ThrustDataPoint>>();
    s.vacuumIsp = j.at("vacuum_isp").get<double>();
    s.seaLevelIsp = j.at("sea_level_isp").get<double>();
    s.propellantMassKg = j.at("propellant_mass_kg").get<double>();
    s.dryMassKg = j.at("dry_mass_kg").get<double>();
    s.ignitionDelaySec = j.value("ignition_delay_sec", s.ignitionDelaySec);
    s.ignitionRampSec = j.value("ignition_ramp_sec", s.ignitionRampSec);
    s.shutdownTimeSec = j.value("shutdown_time_sec", s.shutdownTimeSec);
    s.shutdownRampSec = j.value("shutdown_ramp_sec", s.shutdownRampSec);
    s.maxGimbalPitchRad = j.value("max_gimbal_pitch_rad", s.maxGimbalPitchRad);
    s.maxGimbalYawRad = j.value("max_gimbal_yaw_rad", s.maxGimbalYawRad);
    s.gimbalTimeConstantSec = j.value("gimbal_time_constant_sec", s.gimbalTimeConstantSec);
    s.maxGimbalRateRadPerSec = j.value("max_gimbal_rate_rad_per_sec", s.maxGimbalRateRadPerSec);
    s.enginePositionX = j.value("engine_position_x", s.enginePositionX);
    s.enginePositionY = j.value("engine_position_y", s.enginePositionY);
    s.enginePositionZ = j.value("engine_position_z", s.enginePositionZ);
}

void to_json(json& j, const PropulsionConfig& p) {
    j = json();
    j["stages"] = p.stages;
}

void from_json(const json& j, PropulsionConfig& p) {
    p.stages = j.at("stages").get<std::vector<StageConfig>>();
    std::string error;
    if (!validatePropulsionConfig(p, &error)) {
        throw std::runtime_error("Invalid propulsion configuration: " + error);
    }
}

// --- SeekerConfig -----------------------------------------------------------

void to_json(json& j, const SeekerConfig& s) {
    j = json();
    j["type"] = seekerTypeToString(s.type);
    j["transmitter_power_w"] = s.transmitterPowerW;
    j["antenna_gain_db"] = s.antennaGainDb;
    j["wavelength_m"] = s.wavelengthM;
    j["noise_floor_w"] = s.noiseFloorW;
    j["snr_threshold_db"] = s.snrThresholdDb;
    j["sensitivity_w"] = s.sensitivityW;
    j["wavelength_band"] = s.wavelengthBand;
    j["ir_extinction_per_m"] = s.irExtinctionPerM;
    j["illuminator_px"] = s.illuminatorPx;
    j["illuminator_py"] = s.illuminatorPy;
    j["illuminator_pz"] = s.illuminatorPz;
    j["illuminator_power_w"] = s.illuminatorPowerW;
    j["illuminator_gain_db"] = s.illuminatorGainDb;
    j["illuminator_wavelength_m"] = s.illuminatorWavelengthM;
    j["field_of_view_half_angle_rad"] = s.fieldOfViewHalfAngleRad;
    j["gimbal_azimuth_limit_rad"] = s.gimbalAzimuthLimitRad;
    j["gimbal_elevation_limit_rad"] = s.gimbalElevationLimitRad;
    j["lock_hysteresis_db"] = s.lockHysteresisDb;
    j["lock_dropout_time_sec"] = s.lockDropoutTimeSec;
    j["measurement_latency_sec"] = s.measurementLatencySec;
    j["measurement_noise_enabled"] = s.measurementNoiseEnabled;
    j["angle_noise_std_dev_rad"] = s.angleNoiseStdDevRad;
    j["angle_noise_ref_snr_db"] = s.angleNoiseRefSnrDb;
    j["range_noise_std_dev_m"] = s.rangeNoiseStdDevM;
    j["range_rate_noise_std_dev_mps"] = s.rangeRateNoiseStdDevMps;
    j["glint_sigma_m"] = s.glintSigmaM;
    j["glint_correlation_tau_sec"] = s.glintCorrelationTauSec;
    j["swerling_enabled"] = s.swerlingEnabled;
    j["gimbal_rate_limit_rad_per_sec"] = s.gimbalRateLimitRadPerSec;
    j["min_range_gate_m"] = s.minRangeGateM;
    j["max_range_gate_m"] = s.maxRangeGateM;
    j["terrain_masking_enabled"] = s.terrainMaskingEnabled;
    j["min_closing_rate_mps"] = s.minClosingRateMps;
    j["rate_filter_tau_sec"] = s.rateFilterTauSec;
    j["decoy_rejection_db"] = s.decoyRejectionDb;
    j["passive_rf_duty_cycle"] = s.passiveRfDutyCycle;
    j["illuminator_entity_id"] = s.illuminatorEntityId;
}

void from_json(const json& j, SeekerConfig& s) {
    s.type = seekerTypeFromString(j.at("type").get<std::string>());
    s.transmitterPowerW = j.at("transmitter_power_w").get<double>();
    s.antennaGainDb = j.at("antenna_gain_db").get<double>();
    s.wavelengthM = j.at("wavelength_m").get<double>();
    s.noiseFloorW = j.at("noise_floor_w").get<double>();
    s.snrThresholdDb = j.at("snr_threshold_db").get<double>();
    s.sensitivityW = j.at("sensitivity_w").get<double>();
    s.wavelengthBand = j.at("wavelength_band").get<int>();
    s.irExtinctionPerM = j.at("ir_extinction_per_m").get<double>();
    s.illuminatorPx = j.at("illuminator_px").get<double>();
    s.illuminatorPy = j.at("illuminator_py").get<double>();
    s.illuminatorPz = j.at("illuminator_pz").get<double>();
    s.illuminatorPowerW = j.at("illuminator_power_w").get<double>();
    s.illuminatorGainDb = j.at("illuminator_gain_db").get<double>();
    s.illuminatorWavelengthM = j.at("illuminator_wavelength_m").get<double>();
    s.fieldOfViewHalfAngleRad = j.at("field_of_view_half_angle_rad").get<double>();
    s.gimbalAzimuthLimitRad = j.at("gimbal_azimuth_limit_rad").get<double>();
    s.gimbalElevationLimitRad = j.at("gimbal_elevation_limit_rad").get<double>();
    s.lockHysteresisDb = j.at("lock_hysteresis_db").get<double>();
    s.lockDropoutTimeSec = j.at("lock_dropout_time_sec").get<double>();
    s.measurementLatencySec = j.at("measurement_latency_sec").get<double>();
    s.measurementNoiseEnabled = j.value("measurement_noise_enabled", false);
    s.angleNoiseStdDevRad = j.value("angle_noise_std_dev_rad", 0.001);
    s.angleNoiseRefSnrDb = j.value("angle_noise_ref_snr_db", 20.0);
    s.rangeNoiseStdDevM = j.value("range_noise_std_dev_m", 1.0);
    s.rangeRateNoiseStdDevMps = j.value("range_rate_noise_std_dev_mps", 0.5);
    s.glintSigmaM = j.value("glint_sigma_m", 0.0);
    s.glintCorrelationTauSec = j.value("glint_correlation_tau_sec", 1.0);
    s.swerlingEnabled = j.value("swerling_enabled", false);
    s.gimbalRateLimitRadPerSec = j.value("gimbal_rate_limit_rad_per_sec", 0.0);
    s.minRangeGateM = j.value("min_range_gate_m", 0.0);
    s.maxRangeGateM = j.value("max_range_gate_m", 0.0);
    s.terrainMaskingEnabled = j.value("terrain_masking_enabled", false);
    s.minClosingRateMps = j.value("min_closing_rate_mps", 0.0);
    s.rateFilterTauSec = j.value("rate_filter_tau_sec", 0.05);
    s.decoyRejectionDb = j.value("decoy_rejection_db", 0.0);
    s.passiveRfDutyCycle = j.value("passive_rf_duty_cycle", 1.0);
    s.illuminatorEntityId = j.value("illuminator_entity_id", -1);
}

// --- WarheadConfig ----------------------------------------------------------

void to_json(json& j, const WarheadConfig& w) {
    j = json();
    j["mass_kg"] = w.massKg;
    j["fusing"] = fusingTypeToString(w.fusing);
    j["proximity_trigger_m"] = w.proximityTriggerM;
    j["timed_delay_sec"] = w.timedDelaySec;
    j["lethal_radius_m"] = w.lethalRadiusM;
    j["falloff_radius_m"] = w.falloffRadiusM;
    j["fuse_enabled"] = w.fuseEnabled;
    j["cpa_fuzing_enabled"] = w.cpaFuzingEnabled;
    j["fuse_lookahead_sec"] = w.fuseLookaheadSec;
    j["arming_delay_sec"] = w.armingDelaySec;
    j["min_closing_speed_mps"] = w.minClosingSpeedMps;
    j["self_destruct_time_sec"] = w.selfDestructTimeSec;
    j["damage"] = w.damage;
    j["fuse_detection_probability"] = w.fuseDetectionProbability;
    j["head_on_lethality_factor"] = w.headOnLethalityFactor;
    j["tail_on_lethality_factor"] = w.tailOnLethalityFactor;
}

void from_json(const json& j, WarheadConfig& w) {
    w.massKg = j.at("mass_kg").get<double>();
    w.fusing = fusingTypeFromString(j.at("fusing").get<std::string>());
    w.proximityTriggerM = j.at("proximity_trigger_m").get<double>();
    w.timedDelaySec = j.at("timed_delay_sec").get<double>();
    w.lethalRadiusM = j.at("lethal_radius_m").get<double>();
    // Optional: pre-existing files without the key load the flat law (0.0).
    w.falloffRadiusM = j.value("falloff_radius_m", 0.0);
    // Optional terminal fuse/damage keys (defaults preserve legacy).
    w.fuseEnabled = j.value("fuse_enabled", true);
    w.cpaFuzingEnabled = j.value("cpa_fuzing_enabled", false);
    w.fuseLookaheadSec = j.value("fuse_lookahead_sec", 0.02);
    w.armingDelaySec = j.value("arming_delay_sec", 0.0);
    w.minClosingSpeedMps = j.value("min_closing_speed_mps", 0.0);
    w.selfDestructTimeSec = j.value("self_destruct_time_sec", 0.0);
    w.damage = j.value("damage", 100.0);
    w.fuseDetectionProbability = j.value("fuse_detection_probability", 1.0);
    w.headOnLethalityFactor = j.value("head_on_lethality_factor", 1.0);
    w.tailOnLethalityFactor = j.value("tail_on_lethality_factor", 1.0);
    if (w.falloffRadiusM > 0.0 && w.falloffRadiusM < w.lethalRadiusM) {
        throw std::runtime_error("ConfigSerialization: invalid falloff_radius_m (" +
                                 std::to_string(w.falloffRadiusM) + " m) below "
                                 "lethal_radius_m (" +
                                 std::to_string(w.lethalRadiusM) + " m)");
    }
}

// --- VehicleConfig ----------------------------------------------------------

void to_json(json& j, const VehicleConfig& v) {
    j = json();
    j["type"] = entityTypeToString(v.type);
    j["initial_mass"] = v.initialMass;
    j["mass_dry"] = v.massDry;
    j["structural_hardness"] = v.structuralHardness;
    j["inertia_xx"] = v.Ixx;
    j["inertia_yy"] = v.Iyy;
    j["inertia_zz"] = v.Izz;
    j["inertia_xy"] = v.Ixy;
    j["inertia_xz"] = v.Ixz;
    j["inertia_yz"] = v.Iyz;
    j["aero"] = v.aero;
    j["propulsion"] = v.propulsion;
    j["seeker"] = v.seeker;
    j["sensor"] = v.sensor;
    j["guidance_autopilot"] = v.guidanceAutopilot;
    j["warhead"] = v.warhead;
    j["rcs_profile_id"] = v.rcsProfileId;
    j["ir_profile_id"] = v.irProfileId;
    j["aero_profile_id"] = v.aeroProfileId;
    j["motor_profile_id"] = v.motorProfileId;
    j["seeker_profile_id"] = v.seekerProfileId;
    j["sensor_profile_id"] = v.sensorProfileId;
    j["guidance_profile_id"] = v.guidanceProfileId;
    j["warhead_profile_id"] = v.warheadProfileId;
    j["emitter_eirp_w"] = v.emitterEirpW;
    j["jammer_eirp_w"] = v.jammerEirpW;
}

void from_json(const json& j, VehicleConfig& v) {
    v.type = entityTypeFromString(j.at("type").get<std::string>());
    v.initialMass = j.at("initial_mass").get<double>();
    v.massDry = j.at("mass_dry").get<double>();
    v.structuralHardness = j.value("structural_hardness", 100.0);
    v.Ixx = j.at("inertia_xx").get<double>();
    v.Iyy = j.at("inertia_yy").get<double>();
    v.Izz = j.at("inertia_zz").get<double>();
    v.Ixy = j.value("inertia_xy", 0.0);
    v.Ixz = j.value("inertia_xz", 0.0);
    v.Iyz = j.value("inertia_yz", 0.0);
    v.aero = j.at("aero").get<AeroConfig>();
    v.propulsion = j.at("propulsion").get<PropulsionConfig>();
    v.seeker = j.at("seeker").get<SeekerConfig>();
    v.sensor = j.at("sensor").get<SensorConfig>();
    v.guidanceAutopilot = j.at("guidance_autopilot").get<GuidanceAutopilotConfig>();
    v.warhead = j.at("warhead").get<WarheadConfig>();
    v.rcsProfileId = j.at("rcs_profile_id").get<std::string>();
    v.irProfileId = j.at("ir_profile_id").get<std::string>();
    v.aeroProfileId = j.value("aero_profile_id", std::string(""));
    v.motorProfileId = j.value("motor_profile_id", std::string(""));
    v.seekerProfileId = j.value("seeker_profile_id", std::string(""));
    v.sensorProfileId = j.value("sensor_profile_id", std::string(""));
    v.guidanceProfileId = j.value("guidance_profile_id", std::string(""));
    v.warheadProfileId = j.value("warhead_profile_id", std::string(""));
    v.emitterEirpW = j.at("emitter_eirp_w").get<double>();
    v.jammerEirpW = j.value("jammer_eirp_w", 0.0);
}

// --- EarthEnvironmentConfig / EnvironmentConfig ------------------------------

void to_json(json& j, const EarthEnvironmentConfig& e) {
    j = json();
    j["use_ecef_truth"] = e.useEcefTruth;
    j["use_wgs84_gravity"] = e.useWgs84Gravity;
    j["use_spherical_gravity"] = e.useSphericalGravity;
    j["include_j2_gravity"] = e.includeJ2Gravity;
    j["include_coriolis"] = e.includeCoriolis;
    j["include_centrifugal"] = e.includeCentrifugal;
    j["include_transport_rate"] = e.includeTransportRate;
    j["include_earth_rate_gyro"] = e.includeEarthRateGyro;
    j["reference_latitude_rad"] = e.referenceLatitudeRad;
    j["reference_longitude_rad"] = e.referenceLongitudeRad;
}

void from_json(const json& j, EarthEnvironmentConfig& e) {
    e.useEcefTruth = j.at("use_ecef_truth").get<bool>();
    e.useWgs84Gravity = j.at("use_wgs84_gravity").get<bool>();
    e.useSphericalGravity = j.at("use_spherical_gravity").get<bool>();
    // Optional for compatibility with pre-J2 environment documents.
    e.includeJ2Gravity = j.value("include_j2_gravity", false);
    e.includeCoriolis = j.at("include_coriolis").get<bool>();
    e.includeCentrifugal = j.at("include_centrifugal").get<bool>();
    e.includeTransportRate = j.at("include_transport_rate").get<bool>();
    e.includeEarthRateGyro = j.at("include_earth_rate_gyro").get<bool>();
    e.referenceLatitudeRad = j.at("reference_latitude_rad").get<double>();
    e.referenceLongitudeRad = j.at("reference_longitude_rad").get<double>();
}

void to_json(json& j, const EnvironmentConfig& e) {
    j = json();
    j["earth"] = e.earth;
    // Behavioral scalars: without these a save/load round-trip silently
    // reverts event generation to the struct defaults (kinetic impacts
    // re-enabled, swept/zero-rate ground impact lost). Terrain/wind
    // callbacks and globalTerrain remain non-serializable by contract.
    j["kinetic_impact_radius_m"] = e.kineticImpactRadiusM;
    j["kinetic_impact_latch_enabled"] = e.kineticImpactLatchEnabled;
    j["swept_ground_impact_enabled"] = e.sweptGroundImpactEnabled;
    j["ground_impact_zero_rates"] = e.groundImpactZeroRates;
}

void from_json(const json& j, EnvironmentConfig& e) {
    e = EnvironmentConfig();  // callbacks reset to flat/zero defaults
    e.earth = j.at("earth").get<EarthEnvironmentConfig>();
    // Optional keys so pre-existing documents keep loading with defaults.
    e.kineticImpactRadiusM = j.value("kinetic_impact_radius_m", e.kineticImpactRadiusM);
    e.kineticImpactLatchEnabled = j.value("kinetic_impact_latch_enabled", e.kineticImpactLatchEnabled);
    e.sweptGroundImpactEnabled = j.value("swept_ground_impact_enabled", e.sweptGroundImpactEnabled);
    e.groundImpactZeroRates = j.value("ground_impact_zero_rates", e.groundImpactZeroRates);
}

// --- VehicleInitState -------------------------------------------------------

void to_json(json& j, const VehicleInitState& s) {
    j = json();
    j["px"] = s.px;
    j["py"] = s.py;
    j["pz"] = s.pz;
    j["vx"] = s.vx;
    j["vy"] = s.vy;
    j["vz"] = s.vz;
    j["qx"] = s.qx;
    j["qy"] = s.qy;
    j["qz"] = s.qz;
    j["qw"] = s.qw;
    j["wx"] = s.wx;
    j["wy"] = s.wy;
    j["wz"] = s.wz;
    j["mass"] = s.mass;
    j["allegiance"] = allegianceToString(s.allegiance);
    if (!s.name.empty()) j["name"] = s.name;
    if (!s.role.empty()) j["role"] = s.role;
}

void from_json(const json& j, VehicleInitState& s) {
    // Missing fields fall back to: positions/velocities/rates 0, identity
    // quaternion, mass 0, allegiance "friendly".
    s.px = j.value("px", 0.0);
    s.py = j.value("py", 0.0);
    s.pz = j.value("pz", 0.0);
    s.vx = j.value("vx", 0.0);
    s.vy = j.value("vy", 0.0);
    s.vz = j.value("vz", 0.0);
    s.qx = j.value("qx", 0.0);
    s.qy = j.value("qy", 0.0);
    s.qz = j.value("qz", 0.0);
    s.qw = j.value("qw", 1.0);
    s.wx = j.value("wx", 0.0);
    s.wy = j.value("wy", 0.0);
    s.wz = j.value("wz", 0.0);
    s.mass = j.value("mass", 0.0);
    s.allegiance = allegianceFromString(j.value("allegiance", std::string("friendly")));
    s.name = j.value("name", std::string(""));
    s.role = j.value("role", std::string(""));
}

// --- ScenarioEntityConfig / ScenarioConfig -----------------------------------

void to_json(json& j, const ScenarioEntityConfig& e) {
    j = json();
    j["init_state"] = e.initState;
    j["vehicle_config"] = e.vehicleConfig;
    j["design_ref"] = e.designRef;
    if (!e.name.empty()) j["name"] = e.name;
    if (!e.role.empty()) j["role"] = e.role;
    j["initial_guidance_mode"] = guidanceModeToString(e.initialGuidanceMode);
    j["initial_target_x"] = e.initialTargetX;
    j["initial_target_y"] = e.initialTargetY;
    j["initial_target_z"] = e.initialTargetZ;
    j["initial_target_vx"] = e.initialTargetVx;
    j["initial_target_vy"] = e.initialTargetVy;
    j["initial_target_vz"] = e.initialTargetVz;
    j["initial_max_accel"] = e.initialMaxAccel;
    // W38: APN feed-forward target acceleration (optional; legacy default off).
    if (e.initialTargetAccelAvailable) {
        j["initial_target_accel_x"] = e.initialTargetAccelX;
        j["initial_target_accel_y"] = e.initialTargetAccelY;
        j["initial_target_accel_z"] = e.initialTargetAccelZ;
        j["initial_target_accel_available"] = true;
    }
    if (e.initialTargetId >= 0) {
        j["initial_target_id"] = e.initialTargetId;
    }
    // Rail-launch spec (optional; omitted entirely when disabled so legacy
    // scenario files stay byte-stable).
    if (e.launch.enabled) {
        j["launch"] = {
            {"enabled", true},
            {"parentIndex", e.launch.parentIndex},
            {"targetIndex", e.launch.targetIndex},
            {"dropM", e.launch.dropM},
            {"pushMps", e.launch.pushMps},
            {"lockHoldSec", e.launch.lockHoldSec},
            {"rangeGateM", e.launch.rangeGateM},
            {"flyoutSec", e.launch.flyoutSec},
            {"flyoutAheadM", e.launch.flyoutAheadM},
            {"flyoutClimbM", e.launch.flyoutClimbM},
            {"launchElevationDeg", e.launch.launchElevationDeg},
            {"ejectElevationDeg", e.launch.ejectElevationDeg},
            {"targetEvadeDistanceM", e.launch.targetEvadeDistanceM},
        };
    }
}

void from_json(const json& j, ScenarioEntityConfig& e) {
    e.initState = j.at("init_state").get<VehicleInitState>();
    e.name = j.value("name", std::string(""));
    e.role = j.value("role", std::string(""));
    if (e.name.empty() && !e.initState.name.empty()) e.name = e.initState.name;
    if (e.role.empty() && !e.initState.role.empty()) e.role = e.initState.role;
    e.initialGuidanceMode = guidanceModeFromString(
        j.at("initial_guidance_mode").get<std::string>());
    e.initialTargetX = j.at("initial_target_x").get<double>();
    e.initialTargetY = j.at("initial_target_y").get<double>();
    e.initialTargetZ = j.at("initial_target_z").get<double>();
    e.initialTargetVx = j.at("initial_target_vx").get<double>();
    e.initialTargetVy = j.at("initial_target_vy").get<double>();
    e.initialTargetVz = j.at("initial_target_vz").get<double>();
    e.initialMaxAccel = j.at("initial_max_accel").get<double>();
    // Optional (legacy scenario files omit them).
    e.initialTargetAccelX = j.value("initial_target_accel_x", 0.0);
    e.initialTargetAccelY = j.value("initial_target_accel_y", 0.0);
    e.initialTargetAccelZ = j.value("initial_target_accel_z", 0.0);
    e.initialTargetAccelAvailable = j.value("initial_target_accel_available", false);
    e.initialTargetId = j.value("initial_target_id", static_cast<std::int64_t>(-1));

    e.designRef = j.value("design_ref", std::string(""));
    // Rail-launch spec (optional; legacy scenario files omit it).
    if (j.contains("launch")) {
        const auto& jl = j.at("launch");
        e.launch.enabled = jl.value("enabled", false);
        e.launch.parentIndex = jl.value("parentIndex", std::size_t{0});
        e.launch.targetIndex = jl.value("targetIndex", static_cast<std::int64_t>(-1));
        e.launch.dropM = jl.value("dropM", 0.0);
        e.launch.pushMps = jl.value("pushMps", 0.0);
        e.launch.lockHoldSec = jl.value("lockHoldSec", 0.0);
        e.launch.rangeGateM = jl.value("rangeGateM", 0.0);
        e.launch.flyoutSec = jl.value("flyoutSec", 0.0);
        e.launch.flyoutAheadM = jl.value("flyoutAheadM", 6000.0);
        e.launch.flyoutClimbM = jl.value("flyoutClimbM", 0.0);
        e.launch.launchElevationDeg = jl.value("launchElevationDeg", 0.0);
        e.launch.ejectElevationDeg = jl.value("ejectElevationDeg", 0.0);
        e.launch.targetEvadeDistanceM = jl.value("targetEvadeDistanceM", 0.0);
    }
    if (!e.designRef.empty()) {
        // A design file overrides any inline vehicle config.
        e.vehicleConfig = loadDesignPhysics(e.designRef);
    } else {
        e.vehicleConfig = j.at("vehicle_config").get<VehicleConfig>();
    }
}

void to_json(json& j, const ScenarioConfig& s) {
    j = json();
    j["name"] = s.name;
    j["description"] = s.description;
    j["environment"] = s.environment;
    j["primary_entity_index"] = s.primaryEntityIndex;
    j["random_seed"] = s.randomSeed;
    j["entities"] = s.entities;
}

void from_json(const json& j, ScenarioConfig& s) {
    s.name = j.at("name").get<std::string>();
    s.description = j.at("description").get<std::string>();
    s.environment = j.at("environment").get<EnvironmentConfig>();
    s.primaryEntityIndex = j.at("primary_entity_index").get<std::size_t>();
    s.randomSeed = j.value("random_seed", 0xDEADBEEFu);
    s.entities = j.at("entities").get<std::vector<ScenarioEntityConfig>>();
    validateScenarioConfig(s);
}

void validateScenarioConfig(const ScenarioConfig& scenario) {
    if (scenario.entities.empty()) {
        throw std::runtime_error(
            "ConfigSerialization: scenario '" + scenario.name +
            "' declares no entities");
    }
    if (scenario.primaryEntityIndex >= scenario.entities.size()) {
        throw std::runtime_error(
            "ConfigSerialization: scenario '" + scenario.name +
            "' primary_entity_index " + std::to_string(scenario.primaryEntityIndex) +
            " is outside the entity list (size " +
            std::to_string(scenario.entities.size()) + ")");
    }
}

std::size_t resolvePrimaryEntityId(const ScenarioConfig& scenario,
                                   std::size_t kernelEntityCount) {
    validateScenarioConfig(scenario);
    const std::size_t index = scenario.primaryEntityIndex;
    if (index >= kernelEntityCount) {
        // The usual cause is an index that addresses a rail-launched entity:
        // it exists in the file but not in the kernel until it spawns.
        throw std::invalid_argument(
            "scenario '" + scenario.name + "' primary_entity_index " +
            std::to_string(index) + " addresses no entity at t=0 (kernel has " +
            std::to_string(kernelEntityCount) +
            "); a rail-launched entity is created in flight, so point "
            "primary_entity_index at an entity that exists from the start");
    }
    return index;
}

void ScenarioConfig::loadInto(SimulationKernel& kernel) const {
    kernel.reset();
    kernel.setEnvironment(environment);

    for (const auto& entityCfg : entities) {
        // Rail-launched entities are held by the kernel and spawned
        // in-flight when their launch conditions are met.
        if (entityCfg.launch.enabled) {
            kernel.addPendingLaunch(entityCfg);
            continue;
        }
        VehicleInitState init = entityCfg.initState;
        if (init.name.empty()) init.name = entityCfg.name;
        if (init.role.empty()) init.role = entityCfg.role;
        PhysicsId id = kernel.createVehicle(init, entityCfg.vehicleConfig);

        if (entityCfg.initialGuidanceMode != GuidanceMode::None) {
            SimulationCommand cmd;
            cmd.entityId = id;
            cmd.mode = entityCfg.initialGuidanceMode;
            cmd.targetX = entityCfg.initialTargetX;
            cmd.targetY = entityCfg.initialTargetY;
            cmd.targetZ = entityCfg.initialTargetZ;
            cmd.targetVx = entityCfg.initialTargetVx;
            cmd.targetVy = entityCfg.initialTargetVy;
            cmd.targetVz = entityCfg.initialTargetVz;
            cmd.maxAccel = entityCfg.initialMaxAccel;
            cmd.targetAccelX = entityCfg.initialTargetAccelX;
            cmd.targetAccelY = entityCfg.initialTargetAccelY;
            cmd.targetAccelZ = entityCfg.initialTargetAccelZ;
            cmd.targetAccelAvailable = entityCfg.initialTargetAccelAvailable;
            cmd.targetId = entityCfg.initialTargetId;
            kernel.queueCommand(cmd);
        }
    }

    // Single fan-out point: the scenario seed becomes the kernel
    // seed, which setRandomSeed copies into every stochastic system
    // inside (sensor, nav alignment, split warhead streams).
    kernel.setRandomSeed(randomSeed);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

std::string serializeVehicleConfig(const VehicleConfig& config) {
    return json(config).dump();
}

VehicleConfig deserializeVehicleConfig(const std::string& jsonText) {
    json j = parseJsonText(jsonText, "VehicleConfig");
    return getChecked<VehicleConfig>(j, "VehicleConfig");
}

std::string serializeEnvironment(const EnvironmentConfig& environment) {
    return json(environment).dump();
}

EnvironmentConfig deserializeEnvironment(const std::string& jsonText) {
    json j = parseJsonText(jsonText, "EnvironmentConfig");
    return getChecked<EnvironmentConfig>(j, "EnvironmentConfig");
}

std::string serializeScenario(const ScenarioConfig& scenario) {
    return json(scenario).dump();
}

ScenarioConfig deserializeScenario(const std::string& jsonText) {
    json j = parseJsonText(jsonText, "ScenarioConfig");
    return getChecked<ScenarioConfig>(j, "ScenarioConfig");
}

std::string serializeDesign(const std::string& name,
                            const std::string& geometryJson,
                            const VehicleConfig& physics) {
    json j;
    j["name"] = name;
    if (!isBlank(geometryJson)) {
        try {
            j["geometry"] = json::parse(geometryJson);
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("ConfigSerialization: failed to parse "
                                                 "design geometry: ") + e.what());
        }
    }
    j["physics"] = physics;
    return j.dump();
}

VehicleConfig loadDesignPhysics(const std::string& filePath) {
    const std::string text = readTextFile(filePath, "design");
    json j = parseJsonText(text, "design file");
    try {
        if (j.contains("vehicle_config")) {
            return j.at("vehicle_config").get<VehicleConfig>();
        } else if (j.contains("physics")) {
            return j.at("physics").get<VehicleConfig>();
        } else {
            return j.get<VehicleConfig>();
        }
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("ConfigSerialization: design file '") +
                                 filePath + "' has invalid or missing 'physics' or 'vehicle_config': " +
                                 e.what());
    }
}

// --- ScenarioConfig::save / load --------------------------------------------

bool ScenarioConfig::save(const std::string& path) const {
    // Write to a sibling temporary and rename over the destination, so a
    // failure mid-write (full disk, crash) cannot destroy the previously
    // saved scenario. std::filesystem::rename is atomic on the same volume.
    const std::string tempPath = path + ".tmp";
    {
        std::ofstream f(tempPath, std::ios::trunc);
        if (!f.is_open()) {
            return false;
        }
        f << serializeScenario(*this);
        if (!f) {
            std::error_code ec;
            std::filesystem::remove(tempPath, ec);
            return false;
        }
    }
    std::error_code ec;
    std::filesystem::rename(tempPath, path, ec);
    if (ec) {
        std::filesystem::remove(tempPath, ec);
        return false;
    }
    return true;
}

ScenarioConfig ScenarioConfig::load(const std::string& path) {
    const std::string text = readTextFile(path, "scenario");
    // Resolve relative design_ref paths against the scenario file's own
    // directory. loadDesignPhysics otherwise resolves against the process
    // working directory, so loading a scenario by path silently breaks any
    // scenario that is not laid out relative to the CWD. A relative ref is
    // only rewritten when the scenario-relative file actually exists —
    // scenarios authored against the legacy CWD semantics keep loading.
    try {
        json parsed = json::parse(text);
        if (parsed.contains("entities") && parsed["entities"].is_array()) {
            std::string base = path;
            const auto slash = base.find_last_of("/\\");
            base = (slash == std::string::npos) ? std::string() : base.substr(0, slash + 1);
            for (auto& entity : parsed["entities"]) {
                if (!entity.is_object() || !entity.contains("design_ref") ||
                    !entity["design_ref"].is_string()) {
                    continue;
                }
                const std::string ref = entity["design_ref"].get<std::string>();
                if (ref.empty() || ref.front() == '/') continue;
                const std::string scenarioRelative = base + ref;
                std::error_code ec;
                if (!base.empty() && std::filesystem::exists(scenarioRelative, ec)) {
                    entity["design_ref"] = scenarioRelative;
                }
            }
        }
        return deserializeScenario(parsed.dump());
    } catch (const nlohmann::json::parse_error&) {
        // Malformed document: defer to the canonical deserializeScenario
        // error path (std::runtime_error with the scenario context).
        return deserializeScenario(text);
    }
}

} // namespace StrikeEngine::Kernel
