// navigation_aiding_test: baro/mag aiding, GPS consistency + latency/lever,
// alignment realism, tgo-scheduled N, and serialization round-trips.
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/systems/NavigationSystem.hpp>
#include <strikeengine/kernel/systems/SensorSystem.hpp>
#include <strikeengine/kernel/profiles/SensorProfileDatabase.hpp>
#include <strikeengine/kernel/profiles/GuidanceProfileDatabase.hpp>
#include <strikeengine/kernel/config/ScenarioConfig.hpp>
#include <strikeengine/models/physics/earth/MagneticModel.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>

using namespace StrikeEngine::Kernel;

namespace {

int g_failures = 0;
void check(bool condition, const char* message)
{
    if (condition) std::printf("  [PASS] %s\n", message);
    else { std::printf("  [FAIL] %s\n", message); ++g_failures; }
}

void makeStaticBlocks(PhysicsBlock& physics, SensorBlock& sensors, EntityStatusBlock& status)
{
    physics.size = 1;
    physics.px = {0.0}; physics.py = {0.0}; physics.pz = {1000.0};
    physics.vx = {0.0}; physics.vy = {0.0}; physics.vz = {0.0};
    physics.ax = {0.0}; physics.ay = {0.0}; physics.az = {0.0};
    physics.qw = {1.0}; physics.qx = {0.0}; physics.qy = {0.0}; physics.qz = {0.0};
    physics.wx = {0.0}; physics.wy = {0.0}; physics.wz = {0.0};
    physics.alphax = {0.0}; physics.alphay = {0.0}; physics.alphaz = {0.0};
    physics.mass = {100.0};
    physics.active = {true};
    physics.referenceArea = {1.0};

    sensors.size = 1;
    sensors.accelX = {0.0}; sensors.accelY = {0.0}; sensors.accelZ = {9.80665};
    sensors.gyroX = {0.0}; sensors.gyroY = {0.0}; sensors.gyroZ = {0.0};
    sensors.gpsUpdated = {false};
    sensors.gpsPosX = {0.0}; sensors.gpsPosY = {0.0}; sensors.gpsPosZ = {1000.0};
    sensors.gpsVelX = {0.0}; sensors.gpsVelY = {0.0}; sensors.gpsVelZ = {0.0};
    sensors.accelNoiseStdDev = {0.0};
    sensors.accelBiasStdDev = {0.0};
    sensors.gyroNoiseStdDev = {0.0};
    sensors.gyroBiasStdDev = {0.0};
    sensors.gpsPosNoiseStdDev = {1.0};
    sensors.gpsVelNoiseStdDev = {0.1};
    sensors.gpsInnovationGateSigma = {5.0};
    sensors.imuLeverArmX = {0.0}; sensors.imuLeverArmY = {0.0}; sensors.imuLeverArmZ = {0.0};
    sensors.imuEnabled = {true};
    sensors.gpsEnabled = {false};
    sensors.gpsUpdateRateHz = {1.0};
    sensors.baroEnabled = {false};
    sensors.baroNoiseStdDev = {1.0};
    sensors.baroBiasStdDev = {0.0};
    sensors.baroUpdateRateHz = {100.0};
    sensors.baroAlt = {1000.0};
    sensors.baroUpdated = {false};
    sensors.magEnabled = {false};
    sensors.magNoiseStdDev = {50e-9};
    sensors.magUpdateRateHz = {100.0};
    sensors.magDisturbanceGateRel = {0.25};
    sensors.magX = {0.0}; sensors.magY = {0.0}; sensors.magZ = {0.0};
    sensors.magUpdated = {false};
    sensors.gpsLatencySec = {0.0};
    sensors.gpsLeverArmX = {0.0}; sensors.gpsLeverArmY = {0.0}; sensors.gpsLeverArmZ = {0.0};
    sensors.gpsFixConsistencyEnabled = {false};
    sensors.insConingCompensationEnabled = {false};
    sensors.insAdaptiveQEnabled = {false};
    sensors.insAdaptiveQGain = {1.0};
    sensors.initialAttitudeErrorDeg = {0.0};
    sensors.initialPositionErrorM = {0.0};
    sensors.initialVelocityErrorMps = {0.0};

    status.size = 1;
}

double quatAngleError(const NavigationBlock& nav)
{
    const double c = std::clamp(std::abs(nav.estQw[0]), -1.0, 1.0);
    return 2.0 * std::acos(c);
}

} // namespace

int main()
{
    std::printf("=== navigation_aiding_test ===\n");

    // ---- 1. Barometer bounds the vertical channel ----
    {
        PhysicsBlock physics; SensorBlock sensors; EntityStatusBlock status; NavigationBlock nav;
        makeStaticBlocks(physics, sensors, status);
        NavigationSystem system;
        system.update(sensors, physics, nav, 0.01); // align
        nav.estPz[0] += 8.0;                         // simulate vertical drift
        sensors.baroEnabled[0] = true;
        sensors.baroUpdated[0] = true;
        sensors.baroAlt[0] = 1000.0;
        sensors.baroNoiseStdDev[0] = 1.0;
        system.update(sensors, physics, nav, 0.01);
        check(std::abs(nav.estPz[0] - 1000.0) < 8.0, "baro fix pulls vertical estimate toward truth");
        check(!nav.lastBaroRejected[0], "sane baro fix is accepted");
    }

    // ---- 2. Barometer gate rejects gross outliers ----
    {
        PhysicsBlock physics; SensorBlock sensors; EntityStatusBlock status; NavigationBlock nav;
        makeStaticBlocks(physics, sensors, status);
        NavigationSystem system;
        system.update(sensors, physics, nav, 0.01); // align
        const double before = nav.estPz[0];
        sensors.baroEnabled[0] = true;
        sensors.baroUpdated[0] = true;
        sensors.baroAlt[0] = 100000.0;
        sensors.baroNoiseStdDev[0] = 1.0;
        system.update(sensors, physics, nav, 0.01);
        check(nav.lastBaroRejected[0], "wild baro fix is rejected");
        check(std::abs(nav.estPz[0] - before) < 1.0, "rejected baro fix leaves state alone");
    }

    // ---- 2b. Baro/mag gates decouple from the GPS gate ----
    {
        PhysicsBlock physics; SensorBlock sensors; EntityStatusBlock status; NavigationBlock nav;
        makeStaticBlocks(physics, sensors, status);
        NavigationSystem system;
        system.update(sensors, physics, nav, 0.01); // align
        sensors.baroEnabled[0] = true;
        sensors.baroUpdated[0] = true;
        sensors.baroAlt[0] = 100000.0;
        sensors.baroNoiseStdDev[0] = 1.0;
        sensors.gpsInnovationGateSigma[0] = 0.0; // GPS gating disabled...
        sensors.baroInnovationGateSigma = {3.0}; // ...but the baro keeps its own
        system.update(sensors, physics, nav, 0.01);
        check(nav.lastBaroRejected[0], "explicit baro gate rejects despite disabled GPS gate");
    }

    // ---- 3/4. Magnetometer corrects yaw; disturbed fields rejected ----
    {
        PhysicsBlock physics; SensorBlock sensors; EntityStatusBlock status; NavigationBlock nav;
        makeStaticBlocks(physics, sensors, status);
        sensors.magEnabled[0] = true;
        sensors.magNoiseStdDev[0] = 20e-9;
        NavigationSystem system;
        SensorSystem gen;
        gen.setSeed(0xBEEF);
        EnvironmentConfig env;
        // Loose attitude covariance: a rail-launched round starts with a
        // coarse transfer alignment, not the tight lab alignment.
        nav.covarianceDiag = {{10.0, 10.0, 10.0, 1.0, 1.0, 1.0,
                               0.04, 0.04, 0.04, 0.01, 0.01, 0.01,
                               4e-5, 4e-5, 4e-5}};
        // Align, then twist the estimate 10 deg in yaw.
        system.update(sensors, physics, nav, 0.01);
        const double yawErr = 10.0 * 3.14159265358979323846 / 180.0;
        nav.estQw[0] = std::cos(yawErr * 0.5);
        nav.estQx[0] = 0.0; nav.estQy[0] = 0.0; nav.estQz[0] = std::sin(yawErr * 0.5);
        const double before = quatAngleError(nav);
        double t = 0.0;
        for (int k = 0; k < 30; ++k) {
            t += 0.01;
            gen.update(physics, sensors, status, t, 0.01, env);
            system.update(sensors, physics, nav, 0.01, env);
        }
        const double after = quatAngleError(nav);
        check(after < before, "magnetometer fixes reduce heading error");
        check(!nav.lastMagRejected[0], "clean mag field is accepted");
        // Spoof a 3x field: magnitude gate must reject before fusion.
        // Twin control (same step, no sample) isolates fusion from the
        // legitimate strapdown propagation underneath.
        NavigationBlock twin = nav;
        sensors.magX[0] *= 3.0; sensors.magY[0] *= 3.0; sensors.magZ[0] *= 3.0;
        sensors.magUpdated[0] = true;
        system.update(sensors, physics, nav, 0.01, env);
        sensors.magUpdated[0] = false;
        system.update(sensors, physics, twin, 0.01, env);
        check(nav.lastMagRejected[0], "disturbed mag field is rejected");
        check(nav.estQw[0] == twin.estQw[0] && nav.estQx[0] == twin.estQx[0] &&
              nav.estQy[0] == twin.estQy[0] && nav.estQz[0] == twin.estQz[0],
              "rejected mag fix fuses nothing (bit-identical to no sample)");
    }

    // ---- 5. Whole-fix consistency gate ----
    {
        PhysicsBlock physics; SensorBlock sensors; EntityStatusBlock status; NavigationBlock nav;
        makeStaticBlocks(physics, sensors, status);
        sensors.gpsEnabled[0] = true;
        sensors.gpsUpdateRateHz[0] = 100.0;
        sensors.gpsFixConsistencyEnabled[0] = true;
        NavigationSystem system;
        SensorSystem gen;
        gen.setSeed(11);
        EnvironmentConfig env;
        double t = 0.0;
        for (int k = 0; k < 12; ++k) { t += 0.01; gen.update(physics, sensors, status, t, 0.01, env); system.update(sensors, physics, nav, 0.01, env); }
        // Corrupt one axis grossly: joint chi-square must drop the whole fix.
        sensors.gpsPosX[0] = 100000.0;
        sensors.gpsUpdated[0] = true;
        NavigationBlock twin = nav;
        system.update(sensors, physics, nav, 0.01, env);
        sensors.gpsUpdated[0] = false;
        system.update(sensors, physics, twin, 0.01, env);
        check(nav.lastGpsUpdateRejected[0], "consistency gate flags the corrupted fix");
        check(nav.estPx[0] == twin.estPx[0] && nav.estPy[0] == twin.estPy[0] &&
              nav.estVx[0] == twin.estVx[0],
              "rejected fix fuses nothing (bit-identical to no sample)");
        // Same corruption with the gate off: bad axis rejected solo, good fix fuses.
        sensors.gpsFixConsistencyEnabled[0] = false;
        sensors.gpsPosY[0] = 25.0; // 25 m off on a good axis
        sensors.gpsUpdated[0] = true;
        const double beforeY = twin.estPy[0];
        system.update(sensors, physics, nav, 0.01, env);
        check(std::abs(nav.estPy[0] - beforeY) > 0.0, "per-axis gating still fuses sane channels");
    }

    // ---- 6. Alignment realism: seeded errors + determinism ----
    {
        auto runAlign = [](std::uint32_t seed) {
            PhysicsBlock physics; SensorBlock sensors; EntityStatusBlock status; NavigationBlock nav;
            makeStaticBlocks(physics, sensors, status);
            sensors.initialAttitudeErrorDeg[0] = 2.0;
            sensors.initialPositionErrorM[0] = 5.0;
            sensors.initialVelocityErrorMps[0] = 0.5;
            NavigationSystem system;
            system.setSeed(seed);
            system.update(sensors, physics, nav, 0.01);
            return nav;
        };
        const NavigationBlock a = runAlign(7);
        const NavigationBlock b = runAlign(7);
        check(std::abs(a.estQw[0]) < 1.0, "alignment attitude error is nonzero");
        check(a.estPx[0] != 0.0 && a.estVx[0] != 0.0, "alignment pos/vel errors are nonzero");
        check(a.estPx[0] == b.estPx[0] && a.estQw[0] == b.estQw[0] && a.estVx[0] == b.estVx[0],
              "alignment errors are seed-deterministic");
    }

    // ---- 7. tgo-scheduled N switches inside the window ----
    {
        SimulationKernel k;
        k.setRandomSeed(0xA11CE);
        VehicleInitState init{};
        init.px = 0.0; init.py = 0.0; init.pz = 5000.0;
        init.vx = 300.0; init.vy = 0.0; init.vz = 0.0;
        init.qw = 1.0; init.mass = 200.0;
        VehicleConfig cfg;
        cfg.type = EntityType::Missile;
        cfg.initialMass = 200.0; cfg.massDry = 200.0;
        cfg.guidanceAutopilot.navigationConstant = 4.0;
        cfg.guidanceAutopilot.navScheduleEnabled = true;
        cfg.guidanceAutopilot.navConstantTerminal = 2.0;
        cfg.guidanceAutopilot.navScheduleTgoSec = 1000.0; // everything is "terminal"
        const PhysicsId id = k.createVehicle(init, cfg);
        SimulationCommand cmd{};
        cmd.entityId = id;
        cmd.mode = GuidanceMode::ProportionalNavigation;
        cmd.targetX = 50000.0; cmd.targetY = 0.0; cmd.targetZ = 5000.0;
        cmd.targetVx = 0.0; cmd.targetVy = 0.0; cmd.targetVz = 0.0;
        cmd.maxAccel = 200.0;
        k.queueCommand(cmd);
        k.step(0.01); // tgo publishes; schedule still base (tgo was 0)
        const double first = k.getGuidance().scheduledNavN[id];
        k.step(0.01); // tgo > 0 now -> terminal gain
        const double second = k.getGuidance().scheduledNavN[id];
        check(first == 4.0, "scheduled N holds base gain before tgo exists");
        check(second == 2.0, "scheduled N switches to terminal gain inside the window");
    }

    // ---- 8. GPS lever arm + latency ----
    {
        PhysicsBlock physics; SensorBlock sensors; EntityStatusBlock status;
        makeStaticBlocks(physics, sensors, status);
        sensors.gpsEnabled[0] = true;
        sensors.gpsUpdateRateHz[0] = 100.0;
        sensors.gpsPosNoiseStdDev[0] = 0.0;
        sensors.gpsVelNoiseStdDev[0] = 0.0;
        sensors.gpsLeverArmX[0] = 2.0; // 2 m forward, identity attitude
        SensorSystem gen;
        gen.setSeed(3);
        gen.update(physics, sensors, status, 0.01, 0.01, EnvironmentConfig{});
        check(sensors.gpsUpdated[0], "GPS sample publishes with zero latency");
        check(std::abs(sensors.gpsPosX[0] - 2.0) < 1e-9, "GPS fix sits at the antenna, not the CM");
        sensors.gpsLatencySec[0] = 0.5;
        sensors.gpsLeverArmX[0] = 0.0;
        gen.update(physics, sensors, status, 0.02, 0.01, EnvironmentConfig{});
        check(!sensors.gpsUpdated[0], "latent GPS withholds the sample before its delay elapses");
    }

    // ---- 9. Scenario save/load round-trips the new keys ----
    {
        ScenarioConfig scn;
        scn.name = "aiding-roundtrip";
        scn.randomSeed = 0x12345678u;
        ScenarioEntityConfig e;
        e.initState.px = 1.0;
        e.vehicleConfig.sensor.baroEnabled = true;
        e.vehicleConfig.sensor.baroNoiseStdDev = 2.5;
        e.vehicleConfig.sensor.magEnabled = true;
        e.vehicleConfig.sensor.gpsLatencySec = 0.2;
        e.vehicleConfig.sensor.gpsLeverArmX = 1.5;
        e.vehicleConfig.sensor.insAdaptiveQEnabled = true;
        e.vehicleConfig.sensor.initialAttitudeErrorDeg = 0.5;
        e.vehicleConfig.guidanceAutopilot.navScheduleEnabled = true;
        e.vehicleConfig.guidanceAutopilot.navConstantTerminal = 2.5;
        e.vehicleConfig.guidanceAutopilot.navScheduleTgoSec = 6.0;
        scn.entities = {e};
        const std::string path = "/tmp/opencode/aiding_roundtrip.json";
        check(scn.save(path), "scenario with aiding keys saves");
        const ScenarioConfig back = ScenarioConfig::load(path);
        const auto& s = back.entities[0].vehicleConfig.sensor;
        const auto& g = back.entities[0].vehicleConfig.guidanceAutopilot;
        check(s.baroEnabled && s.baroNoiseStdDev == 2.5, "baro keys round-trip");
        check(s.magEnabled && s.gpsLatencySec == 0.2 && s.gpsLeverArmX == 1.5, "mag/GPS keys round-trip");
        check(s.insAdaptiveQEnabled && s.initialAttitudeErrorDeg == 0.5, "INS/alignment keys round-trip");
        e.vehicleConfig.sensor.insGravityGradientEnabled = true;
        e.vehicleConfig.sensor.insEarthRotationCouplingEnabled = true;
        e.vehicleConfig.sensor.gpsBatchUpdateEnabled = true;
        e.vehicleConfig.sensor.gpsLeverArmCompensationEnabled = true;
        e.vehicleConfig.sensor.gpsYawCorrectionDamping = 0.4;
        e.vehicleConfig.sensor.gpsFixConsistencyThreshold = 0.0;
        e.vehicleConfig.sensor.maxGyroBiasEstimate = 0.005;
        e.vehicleConfig.sensor.baroAttitudeCorrectionEnabled = true;
        scn.entities = {e};
        check(scn.save(path), "scenario with error-model keys saves");
        const ScenarioConfig back2 = ScenarioConfig::load(path);
        const auto& s2 = back2.entities[0].vehicleConfig.sensor;
        check(s2.insGravityGradientEnabled && s2.insEarthRotationCouplingEnabled,
              "INS error-model keys round-trip");
        check(s2.gpsBatchUpdateEnabled && s2.gpsYawCorrectionDamping == 0.4,
              "GPS fusion keys round-trip");
        check(s2.gpsLeverArmCompensationEnabled, "GPS lever-compensation key round-trips");
        check(s2.gpsFixConsistencyThreshold == 0.0 && s2.maxGyroBiasEstimate == 0.005 &&
              s2.baroAttitudeCorrectionEnabled,
              "gate/clamp/baro keys round-trip");
        check(g.navScheduleEnabled && g.navConstantTerminal == 2.5 && g.navScheduleTgoSec == 6.0,
              "scheduled-N keys round-trip");
        check(back.randomSeed == 0x12345678u, "scenario global seed round-trips");
    }

    // ---- 10. Profile DBs parse the new keys ----
    {
        {
            std::ofstream f("/tmp/opencode/sensor_newkeys.json");
            f << R"({"imu_enabled": true, "gps_enabled": true, "baro_enabled": true, "baro_noise_std_dev": 3.0, "mag_enabled": true, "gps_latency_sec": 0.25, "ins_adaptive_q_enabled": true, "initial_attitude_error_deg": 1.0, "ins_gravity_gradient_enabled": true, "gps_batch_update_enabled": true, "gps_yaw_correction_damping": 0.25, "max_gyro_bias_estimate": 0.01})";
        }
        SensorProfileDatabase db;
        check(db.loadProfile("/tmp/opencode/sensor_newkeys.json"), "sensor profile with new keys loads");
        check(db.sensor().baroEnabled && db.sensor().baroNoiseStdDev == 3.0, "sensor DB reads baro keys");
        check(db.sensor().magEnabled && db.sensor().gpsLatencySec == 0.25, "sensor DB reads mag/latency keys");
        check(db.sensor().insAdaptiveQEnabled && db.sensor().initialAttitudeErrorDeg == 1.0,
              "sensor DB reads INS/alignment keys");
        check(db.sensor().insGravityGradientEnabled && db.sensor().gpsBatchUpdateEnabled &&
              db.sensor().gpsYawCorrectionDamping == 0.25 &&
              db.sensor().maxGyroBiasEstimate == 0.01,
              "sensor DB reads INS error-model + GPS fusion keys");
        {
            std::ofstream f("/tmp/opencode/guidance_newkeys.json");
            f << R"({"navigationConstant": 4.0, "navScheduleEnabled": true, "navConstantTerminal": 2.0, "navScheduleTgoSec": 7.0})";
        }
        GuidanceProfileDatabase gdb;
        check(gdb.loadProfile("/tmp/opencode/guidance_newkeys.json"), "guidance profile with new keys loads");
        check(gdb.guidanceAutopilot().navScheduleEnabled &&
              gdb.guidanceAutopilot().navConstantTerminal == 2.0 &&
              gdb.guidanceAutopilot().navScheduleTgoSec == 7.0,
              "guidance DB reads scheduled-N keys");
    }

    // ---- 11. loadInto fans the scenario seed out to every system ----
    {
        auto runLoad = [](bool dirtyFirst) {
            ScenarioConfig c;
            c.name = "seed-fanout";
            c.randomSeed = 0xABCDu;
            ScenarioEntityConfig e;
            e.initState.px = 1.0;
            e.initState.mass = 200.0;
            e.vehicleConfig.initialMass = 200.0;
            e.vehicleConfig.massDry = 200.0;
            // Alignment draws come from the nav stream: identical only if
            // loadInto reseeded it (dirtyFirst poisons all three streams).
            e.vehicleConfig.sensor.initialAttitudeErrorDeg = 2.0;
            c.entities = {e};
            SimulationKernel k;
            if (dirtyFirst) k.setRandomSeed(999u); // poison all three streams
            c.loadInto(k); // no explicit setRandomSeed call
            for (int i = 0; i < 10; ++i) k.step(0.01);
            return k.getNavigation().estQw[0];
        };
        check(runLoad(false) == runLoad(true),
              "loadInto reseeds sensor+nav+warhead streams from the scenario seed");
    }

    // ---- 12. Alignment sigmas seed the state covariance ----
    {
        PhysicsBlock physics; SensorBlock sensors; EntityStatusBlock status; NavigationBlock nav;
        makeStaticBlocks(physics, sensors, status);
        sensors.initialAttitudeErrorDeg = {2.0};
        sensors.initialPositionErrorM = {5.0};
        sensors.initialVelocityErrorMps = {0.5};
        NavigationSystem system;
        system.update(sensors, physics, nav, 0.01); // align only
        const double attRad = 2.0 * 3.14159265358979323846 / 180.0;
        check(std::abs(nav.covarianceDiag[0][6] - attRad * attRad) < 1e-15,
              "attitude covariance seeded from the injected alignment sigma");
        check(nav.covarianceDiag[0][0] == 25.0 && nav.covarianceDiag[0][3] == 0.25,
              "position/velocity covariance seeded from their sigmas");
    }

    // ---- 13. Gravity-gradient + earth-rotation Jacobian blocks ----
    {
        auto runTransition = [](bool gravityGradient, bool earthRotation, int r, int c) {
            PhysicsBlock physics; SensorBlock sensors; EntityStatusBlock status; NavigationBlock nav;
            makeStaticBlocks(physics, sensors, status);
            physics.px[0] = 6371000.0; physics.py[0] = 0.0; physics.pz[0] = 0.0;
            physics.ax[0] = 9.80665;   // specific force cancels radial gravity
            sensors.accelX[0] = 9.80665;
            sensors.gpsEnabled[0] = false;
            sensors.insGravityGradientEnabled = {gravityGradient};
            sensors.insEarthRotationCouplingEnabled = {earthRotation};
            NavigationSystem system;
            EnvironmentConfig env;
            env.earth.useEcefTruth = true;
            env.earth.useSphericalGravity = true;
            env.earth.includeCoriolis = earthRotation;
            env.earth.includeCentrifugal = earthRotation;
            system.update(sensors, physics, nav, 0.01); // align
            system.update(sensors, physics, nav, 0.01, env); // one propagation
            return nav.covarianceFull[0][r * 15 + c];
        };
        check(runTransition(true, false, 3, 0) > runTransition(false, false, 3, 0),
              "gravity gradient couples position error into velocity covariance");
        check(std::abs(runTransition(false, true, 0, 4)) >
              std::abs(runTransition(false, false, 0, 4)),
              "earth-rate coupling adds Coriolis velocity covariance");
    }

    // ---- 14. GPS lever-arm compensation in the filter ----
    {
        PhysicsBlock physics; SensorBlock sensors; EntityStatusBlock status; NavigationBlock nav;
        makeStaticBlocks(physics, sensors, status);
        sensors.gpsEnabled[0] = true;
        sensors.gpsUpdateRateHz[0] = 100.0;
        sensors.gpsPosNoiseStdDev[0] = 0.0;
        sensors.gpsVelNoiseStdDev[0] = 0.0;
        sensors.gpsLeverArmX[0] = 2.0; // 2 m forward, identity attitude
        sensors.gpsLeverArmCompensationEnabled = {true};
        NavigationSystem system;
        system.update(sensors, physics, nav, 0.01); // align at the CM (x=0)
        sensors.gpsUpdated[0] = true;
        sensors.gpsPosX[0] = 2.0; sensors.gpsPosY[0] = 0.0; sensors.gpsPosZ[0] = 1000.0;
        system.update(sensors, physics, nav, 0.01);
        check(std::abs(nav.estPx[0]) < 1e-9,
              "nav refers the antenna fix to the CM before fusion");
    }

    // ---- 15. Batch GPS update fuses a clean fix and gates a bad one ----
    {
        PhysicsBlock physics; SensorBlock sensors; EntityStatusBlock status; NavigationBlock nav;
        makeStaticBlocks(physics, sensors, status);
        sensors.gpsEnabled[0] = true;
        sensors.gpsUpdateRateHz[0] = 100.0;
        sensors.gpsBatchUpdateEnabled = {true};
        sensors.gpsFixConsistencyEnabled = {true};
        NavigationSystem system;
        SensorSystem gen;
        gen.setSeed(5);
        EnvironmentConfig env;
        double t = 0.0;
        for (int k = 0; k < 12; ++k) {
            t += 0.01;
            gen.update(physics, sensors, status, t, 0.01, env);
            system.update(sensors, physics, nav, 0.01, env);
        }
        check(std::isfinite(nav.estPx[0]) && std::isfinite(nav.estPz[0]),
              "batch GPS update stays finite on a clean fix");
        // Corrupt one axis: the joint NIS gate must drop the whole fix.
        sensors.gpsPosX[0] = 100000.0;
        sensors.gpsUpdated[0] = true;
        NavigationBlock twin = nav;
        system.update(sensors, physics, nav, 0.01, env);
        sensors.gpsUpdated[0] = false;
        system.update(sensors, physics, twin, 0.01, env);
        check(nav.lastGpsUpdateRejected[0], "batch joint gate rejects the corrupted fix");
        check(nav.estPx[0] == twin.estPx[0] && nav.estVx[0] == twin.estVx[0],
              "rejected batch fix fuses nothing");
    }

    // ---- 16. resetEntity clears recycled-slot filter state ----
    {
        NavigationBlock nav;
        NavigationSystem system;
        system.resetEntity(nav, 0);
        nav.isAligned[0] = true;
        nav.estAccelBiasX[0] = 0.3;
        nav.estGyroBiasZ[0] = 0.01;
        nav.covarianceFull[0][0] = 123.0;
        system.resetEntity(nav, 0);
        check(!nav.isAligned[0] && nav.estAccelBiasX[0] == 0.0 &&
              nav.estGyroBiasZ[0] == 0.0 && nav.covarianceFull[0][0] != 123.0,
              "resetEntity clears alignment, biases and covariance");
    }

    if (g_failures == 0) std::printf("ALL AIDING TESTS PASSED\n");
    else std::printf("FAILED with %d failures\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}