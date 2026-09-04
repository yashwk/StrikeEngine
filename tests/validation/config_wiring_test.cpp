#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>

#include <cstdio>
#include <cmath>
#include <algorithm>

using namespace StrikeEngine::Kernel;

static int failures = 0;
static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

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
    std::printf("=== config_wiring_test: sensor enablement + guidance/autopilot gains ===\n");

    // ---- Part A: sensor enablement + per-entity GPS rate ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0xC0FFEEu);

        // A: IMU disabled, GPS enabled at 2 Hz -> GPS samples arrive, IMU frozen.
        VehicleConfig cfgA;
        cfgA.sensor.imuEnabled = false;
        cfgA.sensor.gpsEnabled = true;
        cfgA.sensor.gpsUpdateRateHz = 2.0;
        const auto idA = kernel.createVehicle(makeInit(), cfgA);

        // B: GPS disabled -> no GPS samples ever.
        VehicleConfig cfgB;
        cfgB.sensor.gpsEnabled = false;
        const auto idB = kernel.createVehicle(makeInit(), cfgB);

        bool aGotGps = false;
        bool bGotGps = false;
        bool aImuFrozen = true;

        constexpr double dt = 0.01;
        for (int step = 0; step < 300; ++step) {   // 3 s
            kernel.step(dt);
            if (kernel.getSensors().gpsUpdated[idA]) aGotGps = true;
            if (kernel.getSensors().gpsUpdated[idB]) bGotGps = true;
            if (kernel.getSensors().accelX[idA] != 0.0) aImuFrozen = false;
        }

        check(aGotGps,    "IMU-off/GPS-on entity still produces GPS samples at its rate");
        check(aImuFrozen, "IMU-off entity freezes its IMU measurements");
        check(!bGotGps,   "GPS-off entity never produces a GPS sample");
    }

    // ---- Part B: guidance/autopilot gains propagation + configurable clamp ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(7u);

        VehicleConfig cfg;
        cfg.guidanceAutopilot.navigationConstant = 5.0;
        cfg.guidanceAutopilot.waypointGain = 10.0;
        cfg.guidanceAutopilot.maxDeflectionRad = 0.1;
        cfg.guidanceAutopilot.kAccelP = 0.045;
        const auto id = kernel.createVehicle(makeInit(), cfg);

        check(kernel.getGuidance().navigationConstant[id] == 5.0,
              "navigationConstant propagates into GuidanceBlock");
        check(kernel.getGuidance().waypointGain[id] == 10.0,
              "waypointGain propagates into GuidanceBlock");
        check(kernel.getControl().maxDeflectionRad[id] == 0.1,
              "maxDeflectionRad propagates into ControlBlock");
        check(kernel.getPhysics().maxDeflectionRad[id] == 0.1,
              "maxDeflectionRad propagates into PhysicsBlock (achieved clamp)");
        check(kernel.getControl().kAccelP[id] == 0.045,
              "kAccelP propagates into ControlBlock");

        // A saturated climb demand would drive the fins to ~0.35 rad with the
        // default gains, so a 0.1 rad configurable clamp must bind the commands.
        SimulationCommand cmd;
        cmd.entityId = id;
        cmd.mode = GuidanceMode::Waypoint;
        cmd.targetX = 2000.0; cmd.targetY = 0.0; cmd.targetZ = 1200.0;
        cmd.targetVx = 0.0; cmd.targetVy = 0.0; cmd.targetVz = 0.0;
        cmd.maxAccel = 100.0;
        kernel.queueCommand(cmd);

        double maxPitch = 0.0, maxYaw = 0.0, maxRoll = 0.0;
        double maxFinPitch = 0.0, maxFinYaw = 0.0, maxFinRoll = 0.0;
        constexpr double dt = 0.01;
        for (int step = 0; step < 200; ++step) {   // 2 s
            kernel.step(dt);
            if (!kernel.getPhysics().active[id]) break;
            const auto& c = kernel.getControl();
            const auto& p = kernel.getPhysics();
            if (!p.active[id]) break;
            maxPitch = std::max(maxPitch, std::abs(c.pitchCommand[id]));
            maxYaw   = std::max(maxYaw,   std::abs(c.yawCommand[id]));
            maxRoll  = std::max(maxRoll,  std::abs(c.rollCommand[id]));
            maxFinPitch = std::max(maxFinPitch, std::abs(p.finPitch[id]));
            maxFinYaw   = std::max(maxFinYaw,   std::abs(p.finYaw[id]));
            maxFinRoll  = std::max(maxFinRoll,  std::abs(p.finRoll[id]));
        }

        check(maxPitch <= 0.1 + 1e-9, "pitch command respects configurable maxDeflectionRad");
        check(maxYaw   <= 0.1 + 1e-9, "yaw command respects configurable maxDeflectionRad");
        check(maxRoll  <= 0.1 + 1e-9, "roll command respects configurable maxDeflectionRad");
        check(maxFinPitch <= 0.1 + 1e-9, "achieved finPitch respects the configurable achieved clamp");
        check(maxFinYaw   <= 0.1 + 1e-9, "achieved finYaw respects the configurable achieved clamp");
        check(maxFinRoll  <= 0.1 + 1e-9, "achieved finRoll respects the configurable achieved clamp");
    }

    // ---- Part C: Dynamic pressure gain scheduling wiring & command scaling ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(42u);

        // Vehicle 1: Gain scheduling disabled (constant kAccelP)
        VehicleConfig cfg1;
        cfg1.guidanceAutopilot.gainSchedulingEnabled = false;
        cfg1.guidanceAutopilot.kAccelP = 0.030;
        cfg1.guidanceAutopilot.maxDeflectionRad = 0.40;
        auto init1 = makeInit();
        init1.px = 0.0; init1.py = 0.0; init1.pz = 100.0; // sea level (dense air, rho~1.21)
        init1.vx = 600.0; init1.vy = 0.0; init1.vz = 0.0; // high speed (q ~ 218 kPa >> qRef 50 kPa)
        const auto id1 = kernel.createVehicle(init1, cfg1);

        // Vehicle 2: Gain scheduling enabled
        VehicleConfig cfg2;
        cfg2.guidanceAutopilot.gainSchedulingEnabled = true;
        cfg2.guidanceAutopilot.refDynamicPressurePa = 50000.0;
        cfg2.guidanceAutopilot.minDynamicPressurePa = 2000.0;
        cfg2.guidanceAutopilot.maxDynamicPressurePa = 300000.0;
        cfg2.guidanceAutopilot.kAccelP = 0.030;
        cfg2.guidanceAutopilot.maxDeflectionRad = 0.40;
        auto init2 = makeInit();
        init2.px = 0.0; init2.py = 0.0; init2.pz = 100.0;
        init2.vx = 600.0; init2.vy = 0.0; init2.vz = 0.0;
        const auto id2 = kernel.createVehicle(init2, cfg2);

        check(!kernel.getControl().gainSchedulingEnabled[id1], "gainSchedulingEnabled=false wired to ControlBlock");
        check(kernel.getControl().gainSchedulingEnabled[id2],  "gainSchedulingEnabled=true wired to ControlBlock");
        check(kernel.getControl().refDynamicPressurePa[id2] == 50000.0, "refDynamicPressurePa wired to ControlBlock");

        // Issue identical mild climb waypoint command (small demand so neither saturates)
        SimulationCommand cmd1{};
        cmd1.entityId = id1;
        cmd1.mode = GuidanceMode::Waypoint;
        cmd1.targetX = 1000.0; cmd1.targetY = 0.0; cmd1.targetZ = 200.0;
        cmd1.maxAccel = 10.0;
        kernel.queueCommand(cmd1);

        SimulationCommand cmd2{};
        cmd2.entityId = id2;
        cmd2.mode = GuidanceMode::Waypoint;
        cmd2.targetX = 1000.0; cmd2.targetY = 0.0; cmd2.targetZ = 200.0;
        cmd2.maxAccel = 10.0;
        kernel.queueCommand(cmd2);

        // Step 1 iteration
        kernel.step(0.01);

        const double cmdPitchUnscheduled = std::abs(kernel.getControl().pitchCommand[id1]);
        const double cmdPitchScheduled   = std::abs(kernel.getControl().pitchCommand[id2]);

        std::printf("  Part C: unscheduled pitchCmd=%.4f rad, scheduled pitchCmd=%.4f rad\n",
                    cmdPitchUnscheduled, cmdPitchScheduled);

        // At q ~ 218 kPa > 50 kPa, scheduled fin deflection should be significantly smaller (around sqrt(50/218) ~ 0.48x)
        check(cmdPitchUnscheduled > 0.01, "unscheduled entity produces positive pitch command");
        check(cmdPitchScheduled < cmdPitchUnscheduled,
              "gain-scheduled entity scales down fin command at high dynamic pressure (anti-flutter)");
        const double ratio = cmdPitchScheduled / cmdPitchUnscheduled;
        std::printf("  Part C: scaling ratio: %.3f (expected ~0.45 - 0.55)\n", ratio);
        check(ratio > 0.35 && ratio < 0.65, "gain scheduling matches theoretical sqrt(q_ref/q) scaling");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
