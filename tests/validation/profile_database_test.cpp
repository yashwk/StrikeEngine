// Profile-id database layer: per-subsystem profile loaders (aero / motor /
// seeker / sensor), their failure modes, and SimulationKernel::createVehicle
// resolution (profile replaces inline sub-config; missing profile throws).
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>
#include <strikeengine/kernel/profiles/AeroProfileDatabase.hpp>
#include <strikeengine/kernel/profiles/MotorProfileDatabase.hpp>
#include <strikeengine/kernel/profiles/SeekerProfileDatabase.hpp>
#include <strikeengine/kernel/profiles/SensorProfileDatabase.hpp>

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

using namespace StrikeEngine::Kernel;

static int failures = 0;
static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

static const std::string kFixtures =
    std::string(STRIKEENGINE_SOURCE_DIR) + "/tests/data/profiles/";

static VehicleInitState makeInit(double mass = 100.0) {
    VehicleInitState init{};
    init.px = 0; init.py = 0; init.pz = 1000.0;
    init.vx = 80.0; init.vy = 0; init.vz = 0;
    init.qw = 1; init.qx = 0; init.qy = 0; init.qz = 0;
    init.wx = 0; init.wy = 0; init.wz = 0;
    init.mass = mass;
    return init;
}

int main() {
    std::printf("=== profile_database_test: per-subsystem profile ids ===\n");

    // ---- 1. Each DB loads its fixture and exposes the parsed config ----
    {
        AeroProfileDatabase aero;
        check(aero.loadProfile(kFixtures + "aero_mk1.json"),
              "AeroProfileDatabase loads aero_mk1.json");
        check(aero.aero().referenceArea == 0.123 &&
                  aero.aero().referenceLength == 1.234 &&
                  aero.aero().cd == 0.456 &&
                  aero.aero().clAlpha == 5.5 &&
                  aero.aero().clFin == 2.2 &&
                  aero.aero().clMax == 1.7,
              "AeroProfileDatabase parses all AeroConfig fields");

        MotorProfileDatabase motor;
        check(motor.loadProfile(kFixtures + "motor_mk1.json"),
              "MotorProfileDatabase loads motor_mk1.json");
        const auto& prop = motor.propulsion();
        check(prop.stages.size() == 2,
              "MotorProfileDatabase parses 2 stages");
        check(prop.stages[0].thrustCurve.size() == 2 &&
                  prop.stages[0].thrustCurve[1].thrust_n == 100000.0 &&
                  prop.stages[0].vacuumIsp == 300.0 &&
                  prop.stages[0].seaLevelIsp == 265.0 &&
                  prop.stages[0].propellantMassKg == 500.0 &&
                  prop.stages[0].dryMassKg == 80.0,
              "MotorProfileDatabase parses stage-0 fields");
        check(prop.stages[1].vacuumIsp == 290.0 &&
                  prop.stages[1].propellantMassKg == 200.0 &&
                  prop.stages[1].thrustCurve[1].time_s == 10.0,
              "MotorProfileDatabase parses stage-1 fields");

        SeekerProfileDatabase seeker;
        check(seeker.loadProfile(kFixtures + "seeker_mk1.json"),
              "SeekerProfileDatabase loads seeker_mk1.json");
        const auto& sk = seeker.seeker();
        check(sk.type == SeekerType::RF &&
                  sk.transmitterPowerW == 2500.0 &&
                  sk.antennaGainDb == 35.0 &&
                  sk.snrThresholdDb == 14.0 &&
                  sk.sensitivityW == 7e-10 &&
                  sk.wavelengthBand == 3 &&
                  sk.fieldOfViewHalfAngleRad == 0.4 &&
                  sk.gimbalAzimuthLimitRad == 0.8 &&
                  sk.gimbalElevationLimitRad == 0.9 &&
                  sk.lockHysteresisDb == 5.0,
              "SeekerProfileDatabase parses set seeker fields");
        check(sk.wavelengthM == 0.03 &&        // omitted -> struct default
                  sk.noiseFloorW == 1e-12 &&
                  sk.lockDropoutTimeSec == 0.10 &&
                  sk.measurementLatencySec == 0.0 &&
                  sk.illuminatorPx == 0.0 &&
                  sk.illuminatorWavelengthM == 0.03,
              "SeekerProfileDatabase falls back to defaults for omitted keys");

        SensorProfileDatabase sensor;
        check(sensor.loadProfile(kFixtures + "sensor_mk1.json"),
              "SensorProfileDatabase loads sensor_mk1.json");
        const auto& sn = sensor.sensor();
        check(sn.imuEnabled == false && sn.gpsEnabled == true &&
                  sn.accelNoiseStdDev == 0.25 &&
                  sn.accelBiasStdDev == 0.02 &&
                  sn.gyroNoiseStdDev == 0.015 &&
                  sn.gyroBiasStdDev == 0.002 &&
                  sn.gpsPosNoiseStdDev == 8.0 &&
                  sn.gpsVelNoiseStdDev == 0.9 &&
                  sn.gpsUpdateRateHz == 5.0 &&
                  sn.imuLeverArmX == 0.1 &&
                  sn.imuLeverArmY == 0.2 &&
                  sn.imuLeverArmZ == 0.3,
              "SensorProfileDatabase parses all SensorConfig fields");
    }

    // ---- 1.5 Shipped production example profiles also parse ----
    {
        AeroProfileDatabase aero;
        check(aero.loadProfile(std::string(STRIKEENGINE_SOURCE_DIR) +
                                   "/data/aero/sa_missile_mk1_aero.json") &&
                  aero.aero().referenceArea == 0.04 && aero.aero().cd == 0.45,
              "shipped data/aero example parses");

        MotorProfileDatabase motor;
        check(motor.loadProfile(std::string(STRIKEENGINE_SOURCE_DIR) +
                                    "/data/motors/sa_missile_mk1_motor.json") &&
                  motor.propulsion().stages.size() == 2 &&
                  motor.propulsion().stages[0].propellantMassKg == 70.0,
              "shipped data/motors example parses");

        SeekerProfileDatabase seeker;
        check(seeker.loadProfile(std::string(STRIKEENGINE_SOURCE_DIR) +
                                     "/data/seekers/aesa_tracker_v1.json") &&
                  seeker.seeker().type == SeekerType::RF &&
                  seeker.seeker().transmitterPowerW == 1200.0,
              "shipped data/seekers example parses");

        SensorProfileDatabase sensor;
        check(sensor.loadProfile(std::string(STRIKEENGINE_SOURCE_DIR) +
                                     "/data/sensors/sa_missile_mk1_imu.json") &&
                  sensor.sensor().imuEnabled == true &&
                  sensor.sensor().accelNoiseStdDev == 0.00980665,
              "shipped data/sensors example parses");
    }

    // ---- 2. Load failures ----
    {
        AeroProfileDatabase aero;
        check(!aero.loadProfile(kFixtures + "does_not_exist.json"),
              "nonexistent path returns false from loadProfile");

        // Malformed JSON syntax -> false.
        const std::string badSyntax = "profile_database_test_bad_syntax.json";
        {
            std::ofstream f(badSyntax);
            f << "{not valid json";
        }
        MotorProfileDatabase motor;
        check(!motor.loadProfile(badSyntax),
              "malformed JSON syntax returns false from loadProfile");

        // Wrong-typed field -> false (chosen contract: any load failure -> false).
        const std::string badTypes = "profile_database_test_bad_types.json";
        {
            std::ofstream f(badTypes);
            f << R"({"cd":"not-a-number"})";
        }
        check(!aero.loadProfile(badTypes),
              "wrong-typed field returns false from loadProfile");

        // Missing required key -> false (seeker `type` and motor `stages` required).
        const std::string noType = "profile_database_test_no_type.json";
        {
            std::ofstream f(noType);
            f << R"({"field_of_view_half_angle_rad":0.4})";
        }
        SeekerProfileDatabase seeker;
        check(!seeker.loadProfile(noType),
              "missing required seeker type returns false from loadProfile");
        const std::string noStages = "profile_database_test_no_stages.json";
        {
            std::ofstream f(noStages);
            f << R"({"vacuum_isp":300.0})";
        }
        check(!motor.loadProfile(noStages),
              "missing required motor stages returns false from loadProfile");

        std::remove(badSyntax.c_str());
        std::remove(badTypes.c_str());
        std::remove(noType.c_str());
        std::remove(noStages.c_str());
    }

    // ---- 3. createVehicle resolves profile ids (profile wins over inline) ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0xDEADBEEFu);

        VehicleConfig cfg;
        cfg.aeroProfileId = kFixtures + "aero_mk1.json";
        cfg.motorProfileId = kFixtures + "motor_mk1.json";
        cfg.seekerProfileId = kFixtures + "seeker_mk1.json";
        cfg.sensorProfileId = kFixtures + "sensor_mk1.json";

        // Conflicting inline values: the profiles must win.
        cfg.aero.cd = 0.99;
        cfg.aero.referenceArea = 0.99;
        StageConfig inlineStage;
        inlineStage.thrustCurve = {{0.0, 1.0}, {1.0, 0.0}};
        inlineStage.propellantMassKg = 1.0;
        cfg.propulsion.stages.push_back(inlineStage);
        cfg.seeker.type = SeekerType::None;
        cfg.seeker.transmitterPowerW = 1000.0;
        cfg.sensor.imuEnabled = true;
        cfg.sensor.gpsUpdateRateHz = 1.0;
        cfg.sensor.accelNoiseStdDev = 0.1;

        // Non-profile-resolved subsystems still come from the inline config.
        cfg.guidanceAutopilot.navigationConstant = 7.7;
        cfg.warhead.lethalRadiusM = 3.3;

        const auto id = kernel.createVehicle(makeInit(), cfg);

        const auto& phys = kernel.getPhysics();
        check(phys.referenceArea[id] == 0.123 &&
                  phys.referenceLength[id] == 1.234 &&
                  phys.cd[id] == 0.456 &&
                  phys.clAlpha[id] == 5.5 &&
                  phys.clFin[id] == 2.2 &&
                  phys.clMax[id] == 1.7,
              "aero profile replaces inline aero config in PhysicsBlock");
        check(phys.stageCount[id] == 2 && phys.stageIndex[id] == 0 &&
                  phys.propulsionId[id] >= 0,
              "motor profile registers both stages from the profile");
        check(phys.stageCount[id] != 1,
              "inline single-stage propulsion was overridden by the motor profile");

        const auto& sk = kernel.getSeekers();
        check(sk.type[id] == SeekerType::RF &&
                  sk.transmitterPowerW[id] == 2500.0 &&
                  sk.antennaGainDb[id] == 35.0 &&
                  sk.snrThresholdDb[id] == 14.0 &&
                  sk.fieldOfViewHalfAngleRad[id] == 0.4 &&
                  sk.gimbalAzimuthLimitRad[id] == 0.8,
              "seeker profile replaces inline seeker config in SeekerBlock");
        check(sk.wavelengthM[id] == 0.03 &&
                  sk.lockDropoutTimeSec[id] == 0.10,
              "seeker profile omitted keys fall back to defaults in SeekerBlock");

        const auto& sn = kernel.getSensors();
        check(sn.imuEnabled[id] == false &&
                  sn.gpsEnabled[id] == true &&
                  sn.gpsUpdateRateHz[id] == 5.0 &&
                  sn.accelNoiseStdDev[id] == 0.25 &&
                  sn.gyroBiasStdDev[id] == 0.002 &&
                  sn.imuLeverArmZ[id] == 0.3,
              "sensor profile replaces inline sensor config in SensorBlock");

        check(kernel.getGuidance().navigationConstant[id] == 7.7 &&
                  kernel.getControl().kAccelP[id] ==
                      cfg.guidanceAutopilot.kAccelP,
              "guidance/autopilot are NOT profile-resolved (inline still used)");
    }

    // ---- 3.5 Empty profile ids keep using inline config (regression) ----
    {
        SimulationKernel kernel;
        VehicleConfig cfg;
        cfg.aero.cd = 0.77;
        cfg.sensor.gpsUpdateRateHz = 2.5;
        cfg.seeker.type = SeekerType::IR;
        const auto id = kernel.createVehicle(makeInit(), cfg);
        const auto& phys = kernel.getPhysics();
        const auto& sn = kernel.getSensors();
        const auto& sk = kernel.getSeekers();
        check(phys.cd[id] == 0.77 && sn.gpsUpdateRateHz[id] == 2.5 &&
                  sk.type[id] == SeekerType::IR,
              "empty profile ids leave inline sub-configs untouched");
    }

    // ---- 4. Missing profile file -> createVehicle throws runtime_error ----
    {
        SimulationKernel kernel;
        VehicleConfig cfg;
        cfg.aeroProfileId = kFixtures + "does_not_exist.json";
        bool threw = false;
        try {
            (void)kernel.createVehicle(makeInit(), cfg);
        } catch (const std::runtime_error&) { threw = true; }
        catch (const std::exception&) { threw = false; }
        check(threw, "createVehicle throws std::runtime_error for a missing profile");

        threw = false;
        SimulationKernel kernel2;
        VehicleConfig cfg2;
        cfg2.motorProfileId = kFixtures + "does_not_exist.json";
        try {
            (void)kernel2.createVehicle(makeInit(), cfg2);
        } catch (const std::runtime_error&) { threw = true; }
        catch (const std::exception&) { threw = false; }
        check(threw, "motor missing profile also throws std::runtime_error");

        threw = false;
        SimulationKernel kernel3;
        VehicleConfig cfg3;
        cfg3.seekerProfileId = kFixtures + "does_not_exist.json";
        try {
            (void)kernel3.createVehicle(makeInit(), cfg3);
        } catch (const std::runtime_error&) { threw = true; }
        catch (const std::exception&) { threw = false; }
        check(threw, "seeker missing profile also throws std::runtime_error");

        threw = false;
        SimulationKernel kernel4;
        VehicleConfig cfg4;
        cfg4.sensorProfileId = kFixtures + "does_not_exist.json";
        try {
            (void)kernel4.createVehicle(makeInit(), cfg4);
        } catch (const std::runtime_error&) { threw = true; }
        catch (const std::exception&) { threw = false; }
        check(threw, "sensor missing profile also throws std::runtime_error");
    }

    // ---- 4.5 Schema-broken (valid JSON) profile -> runtime_error naming the file ----
    {
        const std::string brokenAero = "profile_database_test_broken_aero.json";
        {
            std::ofstream f(brokenAero);
            f << R"({"cd":"not-a-number"})";
        }
        const std::string brokenSeeker = "profile_database_test_broken_seeker.json";
        {
            std::ofstream f(brokenSeeker);
            f << R"({"type":"submarine"})";
        }

        SimulationKernel kernel;
        VehicleConfig cfg;
        cfg.aeroProfileId = brokenAero;
        bool threw = false;
        std::string msg;
        try {
            (void)kernel.createVehicle(makeInit(), cfg);
        } catch (const std::runtime_error& e) {
            threw = true;
            msg = e.what();
        } catch (const std::exception&) { threw = false; }
        check(threw,
              "schema-broken aero profile makes createVehicle throw std::runtime_error");
        check(msg.find(brokenAero) != std::string::npos,
              "runtime_error message names the broken profile file");

        threw = false;
        SimulationKernel kernel2;
        VehicleConfig cfg2;
        cfg2.seekerProfileId = brokenSeeker;
        try {
            (void)kernel2.createVehicle(makeInit(), cfg2);
        } catch (const std::runtime_error&) { threw = true; }
        catch (const std::exception&) { threw = false; }
        check(threw,
              "schema-broken seeker profile (unknown type) throws std::runtime_error");

        std::remove(brokenAero.c_str());
        std::remove(brokenSeeker.c_str());
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
