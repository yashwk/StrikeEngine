#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/systems/CommandProcessor.hpp>

#include <cmath>
#include <cstdio>
#include <stdexcept>

using namespace StrikeEngine::Kernel;

namespace {

bool approx(double a, double b, double eps = 1e-9)
{
    return std::abs(a - b) < eps;
}

} // namespace

int main()
{
    std::printf("=== kernel_lifecycle_test: slot reuse and timestep validation ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    // (a) Reusing a freed slot must not retain the removed entity's state.
    {
        SimulationKernel kernel;

        VehicleInitState init;
        init.px = 0.0; init.py = 0.0; init.pz = 1000.0;
        init.qw = 1.0;
        init.mass = 100.0;
        const PhysicsId first = kernel.createVehicle(init);

        // Configure the first vehicle with guidance state that must not leak
        // into a later occupant of the same slot.
        SimulationCommand cmd;
        cmd.entityId = first;
        cmd.mode = GuidanceMode::Waypoint;
        cmd.targetX = 5000.0; cmd.targetY = 3000.0; cmd.targetZ = -200.0;
        cmd.targetVx = 10.0; cmd.targetVy = 20.0; cmd.targetVz = 30.0;
        cmd.maxAccel = 50.0;
        kernel.queueCommand(cmd);
        kernel.step(0.01); // processes the queued command

        const auto& guidance = kernel.getGuidance();
        check(guidance.mode[first] == GuidanceMode::Waypoint &&
                  approx(guidance.targetX[first], 5000.0) &&
                  approx(guidance.maxAccel[first], 50.0),
              "precondition: first vehicle carries waypoint guidance state");

        // Free the slot, then recreate at the same index.
        kernel.removeVehicle(first);

        VehicleInitState second;
        second.px = 0.0; second.py = 0.0; second.pz = 2000.0;
        second.qw = 1.0;
        second.mass = 150.0;
        const PhysicsId reused = kernel.createVehicle(second);
        check(reused == first, "recreated vehicle reuses the freed slot index");

        const auto& guidance2 = kernel.getGuidance();
        const auto& control = kernel.getControl();
        const auto& sensors = kernel.getSensors();
        check(guidance2.mode[reused] == GuidanceMode::None,
              "reused slot resets guidance mode to None");
        check(approx(guidance2.targetX[reused], 0.0) &&
                  approx(guidance2.targetY[reused], 0.0) &&
                  approx(guidance2.targetZ[reused], 0.0) &&
                  approx(guidance2.targetVx[reused], 0.0) &&
                  approx(guidance2.targetVy[reused], 0.0) &&
                  approx(guidance2.targetVz[reused], 0.0),
              "reused slot resets guidance targets to zero");
        check(approx(guidance2.commandedAccelX[reused], 0.0) &&
                  approx(guidance2.commandedAccelY[reused], 0.0) &&
                  approx(guidance2.commandedAccelZ[reused], 0.0) &&
                  approx(guidance2.maxAccel[reused], 0.0),
              "reused slot resets guidance commands and maxAccel");
        check(approx(control.thrustCommand[reused], 0.0) &&
                  approx(control.pitchCommand[reused], 0.0) &&
                  approx(control.yawCommand[reused], 0.0) &&
                  approx(control.rollCommand[reused], 0.0),
              "reused slot resets control commands to zero");
        check(approx(sensors.accelNoiseStdDev[reused], 0.02) &&
                  approx(sensors.accelBiasStdDev[reused], 0.005) &&
                  approx(sensors.gyroNoiseStdDev[reused], 0.0002) &&
                  approx(sensors.gyroBiasStdDev[reused], 0.00005) &&
                  approx(sensors.gpsPosNoiseStdDev[reused], 1.5) &&
                  approx(sensors.gpsVelNoiseStdDev[reused], 0.15),
              "reused slot carries the default sensor noise settings");

        const auto& physics = kernel.getPhysics();
        check(approx(physics.pz[reused], 2000.0) && approx(physics.mass[reused], 150.0) &&
                  physics.active[reused],
              "reused slot applies the new vehicle init state");
    }

    // (b) Non-positive timesteps are rejected by step().
    {
        SimulationKernel kernel;
        VehicleInitState init;
        init.px = 0.0; init.py = 0.0; init.pz = 1000.0;
        init.qw = 1.0;
        init.mass = 100.0;
        kernel.createVehicle(init);

        bool threwZero = false;
        try { kernel.step(0.0); }
        catch (const std::invalid_argument&) { threwZero = true; }
        check(threwZero, "step(0.0) throws std::invalid_argument");

        bool threwNegative = false;
        try { kernel.step(-0.01); }
        catch (const std::invalid_argument&) { threwNegative = true; }
        check(threwNegative, "step(-0.01) throws std::invalid_argument");

        bool steppedOk = true;
        try { kernel.step(0.01); }
        catch (const std::invalid_argument&) { steppedOk = false; }
        check(steppedOk, "step(0.01) still advances normally");
    }

    // (c) Every supported integrator is reachable through the kernel API and
    // produces finite, mass-preserving, quaternion-normalized states.
    {
        const IntegratorType types[] = {
            IntegratorType::Euler,
            IntegratorType::RK4,
            IntegratorType::Symplectic,
            IntegratorType::RK45,
        };
        const char* names[] = { "Euler", "RK4", "Symplectic", "RK45" };
        for (std::size_t t = 0; t < 4; ++t)
        {
            SimulationKernel kernel(BackendType::CPU, types[t]);

            VehicleInitState init;
            init.px = 0.0; init.py = 0.0; init.pz = 1000.0;
            init.vx = 100.0; init.vy = 0.0; init.vz = 50.0;
            init.qw = 1.0;
            init.mass = 10.0;

            VehicleConfig config;
            config.massDry = 5.0;
            const PhysicsId id = kernel.createVehicle(init, config);

            kernel.runSteps(50, 0.01);

            const auto& physics = kernel.getPhysics();
            const bool finite =
                std::isfinite(physics.px[id]) &&
                std::isfinite(physics.py[id]) &&
                std::isfinite(physics.pz[id]) &&
                std::isfinite(physics.mass[id]);
            const double quatNorm = std::sqrt(
                physics.qw[id] * physics.qw[id] +
                physics.qx[id] * physics.qx[id] +
                physics.qy[id] * physics.qy[id] +
                physics.qz[id] * physics.qz[id]);

            char message[128];
            std::snprintf(message, sizeof(message),
                          "%s integrator: finite state, mass >= massDry, "
                          "quaternion normalized", names[t]);
            check(finite &&
                      physics.mass[id] >= physics.massDry[id] &&
                      approx(quatNorm, 1.0, 1e-9),
                  message);
        }
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
