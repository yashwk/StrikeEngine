#include <strikeengine/kernel/config/ConfigSerialization.hpp>
#include <strikeengine/kernel/config/SeekerTypeStrings.hpp>

#include <nlohmann/json.hpp>

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
        case Models::FinShape::FreeForm:    f["shape"] = "freeform";    break;
    }
    f["count"] = fin.count;
    f["position_m"] = fin.positionM;
    f["cant_angle_deg"] = fin.cantAngleDeg;
    f["root_chord_m"] = fin.rootChordM;
    f["span_m"] = fin.spanM;
    f["steerable"] = fin.steerable;
    if (fin.shape == Models::FinShape::Trapezoidal) {
        f["tip_chord_m"] = fin.tipChordM;
        f["sweep_length_m"] = fin.sweepLengthM;
    }
    if (fin.shape == Models::FinShape::FreeForm) {
        f["shape_points"] = fin.shapePoints;
    }
    return f;
}

static FinsConfig finConfigFromJson(const json& f) {
    FinsConfig fin;
    const std::string shape = f.value("shape", std::string("trapezoidal"));
    if (shape == "trapezoidal") fin.shape = Models::FinShape::Trapezoidal;
    else if (shape == "elliptical") fin.shape = Models::FinShape::Elliptical;
    else if (shape == "freeform") fin.shape = Models::FinShape::FreeForm;
    else throw std::runtime_error("AeroConfig fins: unknown shape '" + shape + "'");
    fin.count = f.at("count").get<int>();
    fin.positionM = f.value("position_m", 0.0);
    fin.cantAngleDeg = f.value("cant_angle_deg", 0.0);
    fin.rootChordM = f.value("root_chord_m", 0.0);
    fin.spanM = f.value("span_m", 0.0);
    fin.tipChordM = f.value("tip_chord_m", 0.0);
    fin.sweepLengthM = f.value("sweep_length_m", -1.0);
    fin.steerable = f.value("steerable", true);
    if (f.contains("shape_points")) {
        fin.shapePoints = f.at("shape_points").get<std::vector<std::array<double, 2>>>();
    }
    return fin;
}

void to_json(json& j, const AeroConfig& a) {
    j = json{
        {"reference_area",   a.referenceArea},
        {"reference_length", a.referenceLength},
        {"cd",               a.cd},
        {"cl_alpha",         a.clAlpha},
        {"cl_fin",           a.clFin},
        {"cl_max",           a.clMax}
    };
    if (!a.tables.empty()) {
        j["aero_tables"] = a.tables;
    }
    if (!a.finSets.empty()) {
        json arr = json::array();
        for (const auto& fs : a.finSets) {
            if (fs.enabled()) {
                arr.push_back(finConfigToJson(fs));
            }
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
        const auto& af = j.at("airframe");
        a.airframe.wingSpanM       = af.at("wing_span_m").get<double>();
        a.airframe.wingRootChordM  = af.at("wing_root_chord_m").get<double>();
        a.airframe.wingTipChordM   = af.at("wing_tip_chord_m").get<double>();
        a.airframe.wingSweepDeg    = af.at("wing_sweep_deg").get<double>();
        a.airframe.wingPositionM   = af.at("wing_position_m").get<double>();
        a.airframe.wingDihedralDeg = af.at("wing_dihedral_deg").get<double>();
        a.airframe.htailSpanM      = af.at("htail_span_m").get<double>();
        a.airframe.htailChordM     = af.at("htail_chord_m").get<double>();
        a.airframe.htailPositionM  = af.at("htail_position_m").get<double>();
        a.airframe.vtailSpanM      = af.at("vtail_span_m").get<double>();
        a.airframe.vtailChordM     = af.at("vtail_chord_m").get<double>();
        a.airframe.vtailPositionM  = af.at("vtail_position_m").get<double>();
        a.airframe.fuselageDiameterM = af.at("fuselage_diameter_m").get<double>();
        a.airframe.fuselageLengthM   = af.at("fuselage_length_m").get<double>();
        a.airframe.cd0              = af.at("cd0").get<double>();
        a.airframe.oswaldEfficiency = af.at("oswald_efficiency").get<double>();
        a.airframe.clMax            = af.at("cl_max").get<double>();
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
    // W36 phase-manager keys. Optional with legacy defaults on read, so old
    // design manifests / scenarios without them keep the abrupt-override path.
    j["handoffBlendTimeSec"] = g.handoffBlendTimeSec;
    j["lockLossRetentionSec"] = g.lockLossRetentionSec;
    j["apnFeedforwardEnabled"] = g.apnFeedforwardEnabled;
    j["gravityCompensationEnabled"] = g.gravityCompensationEnabled;
    // W39 track-manager keys (optional with defaults).
    j["trackConfirmations"] = g.trackConfirmations;
    j["trackCoastTimeoutSec"] = g.trackCoastTimeoutSec;
    j["trackLossTimeoutSec"] = g.trackLossTimeoutSec;
    // W40 trajectory-core keys (optional with defaults).
    j["trajectoryMinSpeedMps"] = g.trajectoryMinSpeedMps;
    j["trajectoryFeasibilityAccelFactor"] = g.trajectoryFeasibilityAccelFactor;
    // W41 cooperative-engagement datalink (optional with defaults).
    j["datalinkSourceId"] = g.datalinkSourceId;
    j["datalinkTargetId"] = g.datalinkTargetId;
    // tgo-scheduled N (optional with defaults; off = constant N).
    j["navScheduleEnabled"] = g.navScheduleEnabled;
    j["navConstantTerminal"] = g.navConstantTerminal;
    j["navScheduleTgoSec"] = g.navScheduleTgoSec;
    // Dynamic pressure gain scheduling keys (optional with defaults).
    j["gainSchedulingEnabled"] = g.gainSchedulingEnabled;
    j["refDynamicPressurePa"] = g.refDynamicPressurePa;
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
    g.trajectoryMinSpeedMps = j.value("trajectoryMinSpeedMps", 30.0);
    g.trajectoryFeasibilityAccelFactor = j.value("trajectoryFeasibilityAccelFactor", 0.95);
    g.datalinkSourceId = j.value("datalinkSourceId", -1);
    g.datalinkTargetId = j.value("datalinkTargetId", -1);
    g.navScheduleEnabled = j.value("navScheduleEnabled", false);
    g.navConstantTerminal = j.value("navConstantTerminal", 3.0);
    g.navScheduleTgoSec = j.value("navScheduleTgoSec", 8.0);
    g.gainSchedulingEnabled = j.value("gainSchedulingEnabled", false);
    g.refDynamicPressurePa = j.value("refDynamicPressurePa", 50000.0);
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
}

void from_json(const json& j, WarheadConfig& w) {
    w.massKg = j.at("mass_kg").get<double>();
    w.fusing = fusingTypeFromString(j.at("fusing").get<std::string>());
    w.proximityTriggerM = j.at("proximity_trigger_m").get<double>();
    w.timedDelaySec = j.at("timed_delay_sec").get<double>();
    w.lethalRadiusM = j.at("lethal_radius_m").get<double>();
    // Optional: pre-existing files without the key load the flat law (0.0).
    w.falloffRadiusM = j.value("falloff_radius_m", 0.0);
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
}

void from_json(const json& j, VehicleConfig& v) {
    v.type = entityTypeFromString(j.at("type").get<std::string>());
    v.initialMass = j.at("initial_mass").get<double>();
    v.massDry = j.at("mass_dry").get<double>();
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
}

void from_json(const json& j, EnvironmentConfig& e) {
    e = EnvironmentConfig();  // callbacks reset to flat/zero defaults
    e.earth = j.at("earth").get<EarthEnvironmentConfig>();
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
}

// --- ScenarioEntityConfig / ScenarioConfig -----------------------------------

void to_json(json& j, const ScenarioEntityConfig& e) {
    j = json();
    j["init_state"] = e.initState;
    j["vehicle_config"] = e.vehicleConfig;
    j["design_ref"] = e.designRef;
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
}

void from_json(const json& j, ScenarioEntityConfig& e) {
    e.initState = j.at("init_state").get<VehicleInitState>();
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
        return j.at("physics").get<VehicleConfig>();
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("ConfigSerialization: design file '") +
                                 filePath + "' has invalid or missing 'physics': " +
                                 e.what());
    }
}

// --- ScenarioConfig::save / load --------------------------------------------

bool ScenarioConfig::save(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) {
        return false;
    }
    f << serializeScenario(*this);
    return static_cast<bool>(f);
}

ScenarioConfig ScenarioConfig::load(const std::string& path) {
    return deserializeScenario(readTextFile(path, "scenario"));
}

} // namespace StrikeEngine::Kernel
