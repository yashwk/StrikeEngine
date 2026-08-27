// ECEF earth-rate gyro modeling + compensation (opt-in): with
// EnvironmentConfig::earth.includeEarthRateGyro, the gyro measures the
// INERTIAL body rate (earth rotation included) and the INS subtracts it, so
// the attitude estimate sees the earth-fixed body rate. Two bit-identical
// ECEF kernels (same seed, same scenario, flag on vs off) cancel the shared
// noise/bias draws; the gyro difference must equal omega_ie^b = C_e^b (0,0,Om).
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/math/Quaternion.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>
#include <cstdio>
#include <cmath>
#include <algorithm>

using namespace StrikeEngine::Kernel;
using namespace StrikeEngine::Models;

static int failures = 0;
static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

int main() {
    std::printf("=== earth_rate_gyro_test: ECEF earth-rate gyro modeling + compensation ===\n");

    constexpr double dt = 0.01;
    constexpr int steps = 5;

    // Identical stationary ECEF vehicle for both kernels: zero velocity, zero
    // body rate, a fixed ~15 deg roll about body X (non-identity attitude),
    // zero reference area (no aero), guidance idle.
    const auto position = geodeticToEcef({0.3, 1.2, 1000.0});
    VehicleInitState init{};
    init.px = position.x;
    init.py = position.y;
    init.pz = position.z;
    init.vx = 0; init.vy = 0; init.vz = 0;
    constexpr double kPi = 3.14159265358979323846;
    const double rollRad = 15.0 * kPi / 180.0;
    init.qw = std::cos(0.5 * rollRad);
    init.qx = std::sin(0.5 * rollRad);
    init.qy = 0.0;
    init.qz = 0.0;
    init.wx = 0; init.wy = 0; init.wz = 0;
    init.mass = 100.0;
    init.Ixx = 1.0; init.Iyy = 10.0; init.Izz = 10.0;

    VehicleConfig cfg;
    cfg.referenceArea = 0.0;   // coasting point mass: no aero torques

    // Kernel A: gyro models the earth rate; INS compensates it.
    EnvironmentConfig envA;
    envA.earth.useEcefTruth = true;
    envA.earth.useSphericalGravity = true;
    envA.earth.includeEarthRateGyro = true;
    SimulationKernel kernelA;
    kernelA.setEnvironment(envA);
    kernelA.setRandomSeed(42u);
    kernelA.createVehicle(init, cfg);

    // Kernel B: legacy ECEF path (earth rate not modeled).
    EnvironmentConfig envB;
    envB.earth.useEcefTruth = true;
    envB.earth.useSphericalGravity = true;
    envB.earth.includeEarthRateGyro = false;
    SimulationKernel kernelB;
    kernelB.setEnvironment(envB);
    kernelB.setRandomSeed(42u);
    kernelB.createVehicle(init, cfg);

    double maxDiffErr = 0.0;
    double maxNormErr = 0.0;
    double maxEstWErr = 0.0;
    double maxEstQErr = 0.0;
    double maxAbsGyroB = 0.0;
    for (int step = 0; step < steps; ++step) {
        kernelA.step(dt);
        kernelB.step(dt);
        const auto& sensorsA = kernelA.getSensors();
        const auto& sensorsB = kernelB.getSensors();
        const auto& navA = kernelA.getNavigation();
        const auto& navB = kernelB.getNavigation();
        const auto& phys = kernelA.getPhysics();

        // Expected omega_ie^b from the truth attitude (constant: no rotation).
        double ex, ey, ez;
        quatRotateToBody(phys.qw[0], phys.qx[0], phys.qy[0], phys.qz[0],
                         0.0, 0.0, EarthModel::earthRotationRateRadPerSec,
                         ex, ey, ez);
        const double dx = (sensorsA.gyroX[0] - sensorsB.gyroX[0]) - ex;
        const double dy = (sensorsA.gyroY[0] - sensorsB.gyroY[0]) - ey;
        const double dz = (sensorsA.gyroZ[0] - sensorsB.gyroZ[0]) - ez;
        maxDiffErr = std::max(maxDiffErr, std::max(std::abs(dx), std::max(std::abs(dy), std::abs(dz))));

        const double gx = sensorsA.gyroX[0] - sensorsB.gyroX[0];
        const double gy = sensorsA.gyroY[0] - sensorsB.gyroY[0];
        const double gz = sensorsA.gyroZ[0] - sensorsB.gyroZ[0];
        const double norm = std::sqrt(gx * gx + gy * gy + gz * gz);
        maxNormErr = std::max(maxNormErr, std::abs(norm - EarthModel::earthRotationRateRadPerSec));

        maxEstWErr = std::max(maxEstWErr, std::max(
            std::abs(navA.estWx[0] - navB.estWx[0]),
            std::max(std::abs(navA.estWy[0] - navB.estWy[0]),
                     std::abs(navA.estWz[0] - navB.estWz[0]))));
        maxEstQErr = std::max(maxEstQErr, std::max(
            std::abs(navA.estQw[0] - navB.estQw[0]),
            std::max(std::abs(navA.estQx[0] - navB.estQx[0]),
            std::max(std::abs(navA.estQy[0] - navB.estQy[0]),
                     std::abs(navA.estQz[0] - navB.estQz[0])))));
        maxAbsGyroB = std::max(maxAbsGyroB, std::max(
            std::abs(sensorsB.gyroX[0]),
            std::max(std::abs(sensorsB.gyroY[0]), std::abs(sensorsB.gyroZ[0]))));
    }

    std::printf("  max |(gyroA-gyroB) - omega_ie^b| = %.3g\n", maxDiffErr);
    check(maxDiffErr < 1e-9,
          "gyro difference equals omega_ie^b = C_e^b (0,0,Om)");
    std::printf("  max |norm(gyroA-gyroB) - Om| = %.3g\n", maxNormErr);
    check(maxNormErr < 1e-9,
          "earth-rate vector magnitude is the WGS84 rotation rate");
    std::printf("  max |navA.estW - navB.estW| = %.3g\n", maxEstWErr);
    // The compensation resolves the earth rate with the ESTIMATED attitude,
    // which drifts from truth only by the (identical) noise draws, so the
    // residual is ~Omega times the noise-driven attitude error -- far below
    // the earth rate itself.
    check(maxEstWErr < 1e-6,
          "INS earth-rate compensation leaves the estimated rate identical (no drift)");
    std::printf("  max |navA.estQ - navB.estQ| = %.3g\n", maxEstQErr);
    check(maxEstQErr < 1e-6,
          "attitude propagation is unchanged by the compensation");
    std::printf("  max |gyroB| = %.3g\n", maxAbsGyroB);
    check(maxAbsGyroB < 0.1,
          "without the flag the gyro reads the body rate (legacy ECEF path)");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
