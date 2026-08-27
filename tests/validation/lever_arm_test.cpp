// Sensor lever-arm GNC fidelity (MVP): a rigid body rotating at body rate
// omega with a body-frame IMU offset l senses
//     f_imu = f_cm + alpha x l + omega x (omega x l)
// at the IMU location. Two bit-identical kernels (same seed, same scenario,
// zero vs Y-offset lever arm) cancel the shared noise/bias draws, so the
// measurement difference must equal the analytic lever-arm term.
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

int main() {
    std::printf("=== lever_arm_test: IMU lever-arm specific-force correction ===\n");

    constexpr double dt = 0.01;
    constexpr int steps = 10;

    // Identical vehicle for both kernels: zero reference area disables all
    // aero forces and moments, so the CM is in free fall (its specific force
    // is exactly (0,0,0)) and the body holds a steady 1 rad/s rotation about
    // body Z with exactly zero angular acceleration. The attitude is
    // initially identity and guidance is idle (mode None => no commands; the
    // autopilot does not consume sensor measurements), so physics stays
    // bit-identical across the two kernels.
    VehicleInitState init{};
    init.px = 0; init.py = 0; init.pz = 1000.0;
    init.vx = 0; init.vy = 0; init.vz = 0;
    init.qw = 1; init.qx = 0; init.qy = 0; init.qz = 0;
    init.wx = 0; init.wy = 0; init.wz = 1.0;
    init.mass = 100.0;
    init.Ixx = 1.0; init.Iyy = 10.0; init.Izz = 10.0;

    VehicleConfig base;
    base.referenceArea = 0.0;   // coasting point mass: no aero, steady spin

    // Kernel A: IMU offset 1 m along body +Y from the centre of mass.
    VehicleConfig cfgA = base;
    cfgA.imuLeverArmY = 1.0;
    SimulationKernel kernelA;
    kernelA.setRandomSeed(42u);
    kernelA.createVehicle(init, cfgA);

    // Kernel B: IMU at the centre of mass (legacy default, zero lever arm).
    SimulationKernel kernelB;
    kernelB.setRandomSeed(42u);
    kernelB.createVehicle(init, base);

    // Analytic lever-arm term for omega=(0,0,1), l=(0,1,0):
    //     omega x l = (-1, 0, 0)
    //     omega x (omega x l) = (0, -1, 0)
    // The angular acceleration is zero (steady spin), so alpha x l = 0.
    constexpr double expectedDx = 0.0;
    constexpr double expectedDy = -1.0;
    constexpr double expectedDz = 0.0;

    double maxDx = 0.0, maxDy = 0.0, maxDz = 0.0;
    double maxAbsZeroLever = 0.0;
    for (int step = 0; step < steps; ++step) {
        kernelA.step(dt);
        kernelB.step(dt);
        const auto& sensorsA = kernelA.getSensors();
        const auto& sensorsB = kernelB.getSensors();
        maxDx = std::max(maxDx, std::fabs(sensorsA.accelX[0] - sensorsB.accelX[0] - expectedDx));
        maxDy = std::max(maxDy, std::fabs(sensorsA.accelY[0] - sensorsB.accelY[0] - expectedDy));
        maxDz = std::max(maxDz, std::fabs(sensorsA.accelZ[0] - sensorsB.accelZ[0] - expectedDz));
        maxAbsZeroLever = std::max(maxAbsZeroLever, std::max(
            std::fabs(sensorsB.accelX[0]),
            std::max(std::fabs(sensorsB.accelY[0]), std::fabs(sensorsB.accelZ[0]))));
    }

    std::printf("  max |(A-B) - expected| = (%.3g, %.3g, %.3g)\n", maxDx, maxDy, maxDz);
    check(maxDx < 1e-9 && maxDy < 1e-9 && maxDz < 1e-9,
          "lever-arm IMU reads CM specific force plus omega x (omega x l)");
    std::printf("  max |zero-lever-arm measurement| = %.3g\n", maxAbsZeroLever);
    check(maxAbsZeroLever < 0.75,
          "zero lever arm preserves the CM measurement (backward compatible)");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
