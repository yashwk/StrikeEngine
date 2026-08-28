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
        }

        check(maxPitch <= 0.1 + 1e-9, "pitch command respects configurable maxDeflectionRad");
        check(maxYaw   <= 0.1 + 1e-9, "yaw command respects configurable maxDeflectionRad");
        check(maxRoll  <= 0.1 + 1e-9, "roll command respects configurable maxDeflectionRad");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
