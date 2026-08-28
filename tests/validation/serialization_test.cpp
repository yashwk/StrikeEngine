// JSON (de)serialization for the config subsystem structures: VehicleConfig
// round-trip, environment earth-block round-trip, ScenarioConfig serialize/
// deserialize plus save/load, and the designRef/design-file concept.
#include <strikeengine/kernel/config/ConfigSerialization.hpp>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

using namespace StrikeEngine::Kernel;

static int failures = 0;
static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

namespace {

VehicleConfig makeRichConfig()
{
    VehicleConfig cfg;
    cfg.type = EntityType::Aircraft;
    cfg.initialMass = 1200.0;
    cfg.massDry = 900.0;
    cfg.Ixx = 40.0; cfg.Iyy = 800.0; cfg.Izz = 800.0;

    cfg.aero.referenceArea = 1.4;
    cfg.aero.referenceLength = 1.2;
    cfg.aero.cd = 0.35;
    cfg.aero.clAlpha = 3.0;
    cfg.aero.clFin = 1.5;
    cfg.aero.clMax = 1.9;
    cfg.aero.tables.machBreakpoints = {0.0, 0.5, 1.0, 2.0};
    cfg.aero.tables.aoaBreakpointsRad = {0.0, 0.087, 0.174};
    cfg.aero.tables.clTable = {
        {0.0, 0.5, 1.0},
        {0.0, 0.55, 1.1},
        {0.0, 0.45, 0.9},
        {0.0, 0.2, 0.4}
    };
    cfg.aero.tables.cdTable = {
        {0.02, 0.03, 0.05},
        {0.02, 0.035, 0.055},
        {0.05, 0.06, 0.08},
        {0.06, 0.07, 0.10}
    };

    StageConfig stage;
    stage.thrustCurve = { {0.0, 60000.0}, {3.0, 70000.0}, {5.0, 0.0} };
    stage.vacuumIsp = 260.0;
    stage.seaLevelIsp = 225.0;
    stage.propellantMassKg = 300.0;
    stage.dryMassKg = 600.0;
    cfg.propulsion.stages.push_back(stage);

    cfg.seeker.type = SeekerType::RF;
    cfg.seeker.transmitterPowerW = 2000.0;
    cfg.seeker.antennaGainDb = 28.0;
    cfg.seeker.wavelengthM = 0.02;
    cfg.seeker.noiseFloorW = 2.0e-12;
    cfg.seeker.snrThresholdDb = 12.0;
    cfg.seeker.sensitivityW = 3.0e-9;
    cfg.seeker.wavelengthBand = 2;
    cfg.seeker.irExtinctionPerM = 5.0e-5;
    cfg.seeker.illuminatorPx = 10.0;
    cfg.seeker.illuminatorPy = 20.0;
    cfg.seeker.illuminatorPz = 30.0;
    cfg.seeker.illuminatorPowerW = 1.0e6;
    cfg.seeker.illuminatorGainDb = 40.0;
    cfg.seeker.illuminatorWavelengthM = 0.03;
    cfg.seeker.fieldOfViewHalfAngleRad = 0.5;
    cfg.seeker.gimbalAzimuthLimitRad = 0.6;
    cfg.seeker.gimbalElevationLimitRad = 0.7;
    cfg.seeker.lockHysteresisDb = 4.0;
    cfg.seeker.lockDropoutTimeSec = 0.2;
    cfg.seeker.measurementLatencySec = 0.05;

    cfg.sensor.imuEnabled = false;
    cfg.sensor.gpsEnabled = true;
    cfg.sensor.accelNoiseStdDev = 0.3;
    cfg.sensor.accelBiasStdDev = 0.05;
    cfg.sensor.gyroNoiseStdDev = 0.02;
    cfg.sensor.gyroBiasStdDev = 0.005;
    cfg.sensor.gpsPosNoiseStdDev = 7.0;
    cfg.sensor.gpsVelNoiseStdDev = 0.8;
    cfg.sensor.gpsUpdateRateHz = 2.0;
    cfg.sensor.imuLeverArmX = 0.1;
    cfg.sensor.imuLeverArmY = 0.2;
    cfg.sensor.imuLeverArmZ = 0.3;

    cfg.guidanceAutopilot.navigationConstant = 4.0;
    cfg.guidanceAutopilot.waypointGain = 25.0;
    cfg.guidanceAutopilot.kAccelP = 0.05;
    cfg.guidanceAutopilot.kRateP = 1.2;
    cfg.guidanceAutopilot.kAlphaP = 0.3;
    cfg.guidanceAutopilot.kRollP = 0.15;
    cfg.guidanceAutopilot.kRollD = 0.08;
    cfg.guidanceAutopilot.maxDeflectionRad = 0.5;
    cfg.guidanceAutopilot.servoTimeConstantSec = 0.03;
    cfg.guidanceAutopilot.maxServoRateRadPerSec = 6.0;

    cfg.warhead.massKg = 25.0;
    cfg.warhead.fusing = FusingType::Proximity;
    cfg.warhead.proximityTriggerM = 8.0;
    cfg.warhead.timedDelaySec = 2.0;
    cfg.warhead.lethalRadiusM = 15.0;

    cfg.rcsProfileId = "rcs-big";
    cfg.irProfileId = "ir-hot";
    cfg.aeroProfileId = "aero-mk1";
    cfg.motorProfileId = "motor-mk1";
    cfg.seekerProfileId = "seeker-mk1";
    cfg.sensorProfileId = "sensor-mk1";
    cfg.emitterEirpW = 2.5e6;
    return cfg;
}

} // namespace

int main()
{
    std::printf("=== serialization_test: JSON config (de)serialization ===\n");

    // ---- 1. VehicleConfig round-trip ----
    {
        VehicleConfig cfg = makeRichConfig();
        std::string text;
        VehicleConfig cfg2;
        std::string text2;
        try {
            text = serializeVehicleConfig(cfg);
            cfg2 = deserializeVehicleConfig(text);
            text2 = serializeVehicleConfig(cfg2);
        } catch (const std::exception& e) {
            check(false, "VehicleConfig round-trip did not throw");
            std::printf("  unexpected exception: %s\n", e.what());
        }
        check(text == text2, "VehicleConfig string round-trip is byte-identical");
        check(cfg2.type == EntityType::Aircraft, "type survives as Aircraft");
        check(cfg2.initialMass == 1200.0 && cfg2.massDry == 900.0,
              "structural mass fields survive");
        check(cfg2.Ixx == 40.0 && cfg2.Iyy == 800.0 && cfg2.Izz == 800.0,
              "inertia values survive");
        check(cfg2.aero.cd == 0.35 && cfg2.aero.clAlpha == 3.0 &&
                  cfg2.aero.clFin == 1.5 && cfg2.aero.clMax == 1.9,
              "aero coefficients survive");
        check(cfg2.aero.tables.machBreakpoints == cfg.aero.tables.machBreakpoints &&
                  cfg2.aero.tables.aoaBreakpointsRad == cfg.aero.tables.aoaBreakpointsRad &&
                  cfg2.aero.tables.clTable == cfg.aero.tables.clTable &&
                  cfg2.aero.tables.cdTable == cfg.aero.tables.cdTable,
              "aero cd(M,a)/cl(M,a) tables survive the round-trip");
        check(cfg2.aero.referenceArea == 1.4 && cfg2.aero.referenceLength == 1.2,
              "aero geometry survives");
        check(cfg2.propulsion.stages.size() == 1 &&
                  cfg2.propulsion.stages[0].thrustCurve.size() == 3 &&
                  cfg2.propulsion.stages[0].thrustCurve[1].thrust_n == 70000.0 &&
                  cfg2.propulsion.stages[0].vacuumIsp == 260.0 &&
                  cfg2.propulsion.stages[0].propellantMassKg == 300.0 &&
                  cfg2.propulsion.stages[0].dryMassKg == 600.0,
              "propulsion stage (thrust points + Isps + masses) survives");
        check(cfg2.seeker.type == SeekerType::RF, "seeker type survives as RF");
        check(cfg2.seeker.transmitterPowerW == 2000.0 &&
                  cfg2.seeker.antennaGainDb == 28.0 &&
                  cfg2.seeker.wavelengthBand == 2 &&
                  cfg2.seeker.lockDropoutTimeSec == 0.2,
              "seeker RF/tracking fields survive");
        check(cfg2.sensor.imuEnabled == false && cfg2.sensor.gpsEnabled == true,
              "sensor enable flags survive (imu disabled)");
        check(cfg2.sensor.accelNoiseStdDev == 0.3 &&
                  cfg2.sensor.gyroBiasStdDev == 0.005 &&
                  cfg2.sensor.gpsUpdateRateHz == 2.0 &&
                  cfg2.sensor.imuLeverArmZ == 0.3,
              "sensor noise/bias/lever-arm fields survive");
        check(cfg2.guidanceAutopilot.navigationConstant == 4.0 &&
                  cfg2.guidanceAutopilot.kAccelP == 0.05 &&
                  cfg2.guidanceAutopilot.maxServoRateRadPerSec == 6.0,
              "guidance/autopilot gains survive");
        check(cfg2.warhead.fusing == FusingType::Proximity &&
                  cfg2.warhead.proximityTriggerM == 8.0 &&
                  cfg2.warhead.lethalRadiusM == 15.0,
              "warhead fusing type and radii survive");
        check(cfg2.rcsProfileId == "rcs-big" && cfg2.irProfileId == "ir-hot" &&
                  cfg2.emitterEirpW == 2.5e6,
              "signature metadata survives");
        check(cfg2.aeroProfileId == "aero-mk1" &&
                  cfg2.motorProfileId == "motor-mk1" &&
                  cfg2.seekerProfileId == "seeker-mk1" &&
                  cfg2.sensorProfileId == "sensor-mk1",
              "profile ids survive the round-trip");
    }

    // ---- 1.5 Legacy VehicleConfig JSON (no profile-id keys) still loads ----
    {
        std::string legacy = serializeVehicleConfig(VehicleConfig{});
        // Strip the four profile-id keys to emulate a pre-feature persisted file.
        for (const char* key : {"aero_profile_id", "motor_profile_id",
                                "seeker_profile_id", "sensor_profile_id"}) {
            const std::string entry = std::string("\"") + key + "\":\"\",";
            const std::size_t pos = legacy.find(entry);
            if (pos != std::string::npos) legacy.erase(pos, entry.size());
        }
        VehicleConfig parsed;
        bool loadedOk = true;
        try {
            parsed = deserializeVehicleConfig(legacy);
        } catch (const std::exception& e) {
            check(false, "legacy VehicleConfig (no profile-id keys) did not throw");
            std::printf("  unexpected exception: %s\n", e.what());
            loadedOk = false;
        }
        if (loadedOk) {
            check(parsed.aeroProfileId == "" && parsed.motorProfileId == "" &&
                      parsed.seekerProfileId == "" && parsed.sensorProfileId == "",
                  "omitted profile-id keys deserialize as empty strings");
        }
    }

    // ---- 2. Environment earth-block round-trip ----
    {
        EnvironmentConfig env;
        env.earth.useEcefTruth = true;
        env.earth.useWgs84Gravity = true;
        env.earth.includeEarthRateGyro = true;
        env.earth.referenceLatitudeRad = 0.5;
        env.earth.referenceLongitudeRad = 0.25;
        std::string text;
        EnvironmentConfig env2;
        try {
            text = serializeEnvironment(env);
            env2 = deserializeEnvironment(text);
        } catch (const std::exception& e) {
            check(false, "Environment round-trip did not throw");
            std::printf("  unexpected exception: %s\n", e.what());
        }
        check(serializeEnvironment(env2) == text,
              "Environment string round-trip is byte-identical");
        check(env2.earth.useEcefTruth && env2.earth.useWgs84Gravity &&
                  env2.earth.includeEarthRateGyro,
              "earth flags survive");
        check(env2.earth.referenceLatitudeRad == 0.5 &&
                  env2.earth.referenceLongitudeRad == 0.25,
              "reference latitude/longitude survive");
        check(env2.terrainElevation(0.0, 0.0) == 0.0,
              "deserialized terrain callback is the default flat zero");
        const auto wind = env2.windVelocity(0.0, 0.0, 0.0, 0.0);
        check(wind[0] == 0.0 && wind[1] == 0.0 && wind[2] == 0.0,
              "deserialized wind callback is the default zero");
    }

    // ---- 3. Scenario round-trip + save/load ----
    {
        ScenarioConfig scenario;
        scenario.name = "serialization-scenario";
        scenario.description = "round-trip test";
        scenario.primaryEntityIndex = 0;
        ScenarioEntityConfig entity;
        entity.initState.px = 100.0;
        entity.initState.py = 200.0;
        entity.initState.pz = 1000.0;
        entity.initState.vx = 50.0;
        entity.initState.vy = 0.0;
        entity.initState.vz = 0.0;
        entity.initState.qw = 1.0;
        entity.initState.mass = 500.0;
        entity.initState.allegiance = Allegiance::Hostile;
        entity.vehicleConfig = makeRichConfig();
        entity.initialGuidanceMode = GuidanceMode::Waypoint;
        entity.initialTargetX = 5000.0;
        entity.initialTargetY = 0.0;
        entity.initialTargetZ = -200.0;
        entity.initialTargetVx = 10.0;
        entity.initialTargetVy = 20.0;
        entity.initialTargetVz = 30.0;
        entity.initialMaxAccel = 40.0;
        scenario.entities.push_back(entity);

        std::string text;
        ScenarioConfig loaded;
        std::string text2;
        try {
            text = serializeScenario(scenario);
            loaded = deserializeScenario(text);
            text2 = serializeScenario(loaded);
        } catch (const std::exception& e) {
            check(false, "Scenario round-trip did not throw");
            std::printf("  unexpected exception: %s\n", e.what());
        }
        check(text == text2, "Scenario string round-trip is byte-identical");
        check(loaded.name == scenario.name && loaded.description == scenario.description,
              "scenario name/description survive");
        check(loaded.entities.size() == 1, "entity count survives");
        check(loaded.entities[0].initState.mass == 500.0 &&
                  loaded.entities[0].initState.px == 100.0 &&
                  loaded.entities[0].initState.qw == 1.0,
              "entity init state survives");
        check(loaded.entities[0].initState.allegiance == Allegiance::Hostile,
              "entity allegiance survives");
        check(loaded.entities[0].initialGuidanceMode == GuidanceMode::Waypoint,
              "initial guidance mode survives as Waypoint");
        check(loaded.entities[0].initialTargetZ == -200.0 &&
                  loaded.entities[0].initialTargetVx == 10.0 &&
                  loaded.entities[0].initialMaxAccel == 40.0,
              "initial guidance target survives");
        check(loaded.entities[0].vehicleConfig.type == EntityType::Aircraft &&
                  loaded.entities[0].vehicleConfig.seeker.type == SeekerType::RF,
              "entity vehicle config survives");

        // File-based save/load in CWD.
        const std::string path = "serialization_test_tmp_scenario.json";
        check(scenario.save(path), "ScenarioConfig::save writes the file");
        ScenarioConfig fromFile;
        try {
            fromFile = ScenarioConfig::load(path);
        } catch (const std::exception& e) {
            check(false, "ScenarioConfig::load did not throw");
            std::printf("  unexpected exception: %s\n", e.what());
        }
        check(fromFile.name == scenario.name &&
                  fromFile.entities.size() == 1 &&
                  fromFile.entities[0].initialGuidanceMode == GuidanceMode::Waypoint &&
                  fromFile.entities[0].initState.mass == 500.0,
              "ScenarioConfig::load restores the same fields");
        std::remove(path.c_str());
    }

    // ---- 4. Design file ----
    {
        VehicleConfig cfg = makeRichConfig();
        std::string design;
        try {
            design = serializeDesign("demo", R"({"nose":"cone"})", cfg);
        } catch (const std::exception& e) {
            check(false, "serializeDesign did not throw");
            std::printf("  unexpected exception: %s\n", e.what());
        }
        check(design.find("nose") != std::string::npos &&
                  design.find("cone") != std::string::npos,
              "design geometry is embedded verbatim");
        check(design.find("demo") != std::string::npos,
              "design name is embedded");

        const std::string path = "serialization_test_tmp_design.json";
        {
            std::ofstream f(path);
            f << design;
        }
        VehicleConfig physics;
        try {
            physics = loadDesignPhysics(path);
        } catch (const std::exception& e) {
            check(false, "loadDesignPhysics did not throw");
            std::printf("  unexpected exception: %s\n", e.what());
        }
        check(serializeVehicleConfig(physics) == serializeVehicleConfig(cfg),
              "loadDesignPhysics round-trips the design physics");
        std::remove(path.c_str());
    }

    // ---- 4.5 designRef overrides the inline vehicle config ----
    {
        VehicleConfig cfg;
        cfg.aero.cd = 0.77;
        const std::string design = serializeDesign("demo", R"({"nose":"cone"})", cfg);
        const std::string path = "serialization_test_tmp_design.json";
        {
            std::ofstream f(path);
            f << design;
        }

        ScenarioConfig scenario;
        scenario.name = "designref-scenario";
        ScenarioEntityConfig entity;
        entity.initState.px = 0.0;
        entity.initState.py = 0.0;
        entity.initState.pz = 1000.0;
        entity.initState.qw = 1.0;
        entity.initState.mass = 100.0;
        entity.designRef = path;
        entity.vehicleConfig.aero.cd = 0.123;   // inline config, weaker than the design file
        scenario.entities.push_back(entity);

        bool loadedOk = true;
        ScenarioConfig loaded;
        try {
            loaded = deserializeScenario(serializeScenario(scenario));
        } catch (const std::exception& e) {
            check(false, "designRef scenario round-trip did not throw");
            std::printf("  unexpected exception: %s\n", e.what());
            loadedOk = false;
        }
        if (loadedOk) {
            check(loaded.entities[0].vehicleConfig.aero.cd == 0.77,
                  "designRef overrides the inline vehicle config (cd = 0.77)");
            check(loaded.entities[0].designRef == path,
                  "designRef is preserved through the scenario round-trip");
        }
        std::remove(path.c_str());
    }

    // ---- 5. Malformed JSON and unknown enums throw std::runtime_error ----
    {
        bool threw = false;
        try {
            (void)deserializeVehicleConfig("{not valid json");
        } catch (const std::runtime_error&) { threw = true; }
        catch (const std::exception&) { threw = false; }
        check(threw, "malformed JSON throws std::runtime_error");

        threw = false;
        try {
            (void)deserializeVehicleConfig(R"({"type":"submarine"})");
        } catch (const std::runtime_error&) { threw = true; }
        catch (const std::exception&) { threw = false; }
        check(threw, "unknown EntityType string throws std::runtime_error");

        threw = false;
        try {
            (void)deserializeEnvironment(R"({"earth":{"use_ecef_truth":"yes"}})");
        } catch (const std::runtime_error&) { threw = true; }
        catch (const std::exception&) { threw = false; }
        check(threw, "type-mismatched environment field throws std::runtime_error");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
