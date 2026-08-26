#include <strikeengine/kernel/systems/NavigationSystem.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace StrikeEngine::Kernel;

namespace {

void makeBlocks(PhysicsBlock& physics, SensorBlock& sensors)
{
    physics.size = 1;
    physics.px = {0.0}; physics.py = {0.0}; physics.pz = {1000.0};
    physics.vx = {0.0}; physics.vy = {0.0}; physics.vz = {0.0};
    physics.qw = {1.0}; physics.qx = {0.0}; physics.qy = {0.0}; physics.qz = {0.0};
    physics.wx = {0.0}; physics.wy = {0.0}; physics.wz = {0.0};
    physics.active = {true};

    sensors.size = 1;
    sensors.accelX = {0.0}; sensors.accelY = {0.0}; sensors.accelZ = {9.80665};
    sensors.gyroX = {0.0}; sensors.gyroY = {0.0}; sensors.gyroZ = {0.0};
    sensors.gpsUpdated = {false};
    sensors.gpsPosX = {0.0}; sensors.gpsPosY = {0.0}; sensors.gpsPosZ = {1000.0};
    sensors.gpsVelX = {0.0}; sensors.gpsVelY = {0.0}; sensors.gpsVelZ = {0.0};
    sensors.accelNoiseStdDev = {0.01};
    sensors.accelBiasStdDev = {0.005};
    sensors.gyroNoiseStdDev = {0.001};
    sensors.gyroBiasStdDev = {0.001};
    sensors.gpsPosNoiseStdDev = {1000.0}; // isolate velocity/bias observability
    sensors.gpsVelNoiseStdDev = {0.05};
}

}

int main()
{
    std::printf("=== navigation_test: coupled EKF and bias convergence ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    PhysicsBlock physics;
    SensorBlock sensors;
    NavigationBlock nav;
    makeBlocks(physics, sensors);
    // Hold the initially known level attitude tightly so this regression
    // isolates accelerometer-bias observability from the separate
    // accelerometer-bias/tilt ambiguity of a stationary vehicle.
    nav.covarianceDiag = {{10.0, 10.0, 10.0, 1.0, 1.0, 1.0,
                           1e-8, 1e-8, 1e-8, 0.25, 0.25, 0.25,
                           1e-8, 1e-8, 1e-8}};
    NavigationSystem system;

    // Align at truth first, then feed a constant +X accelerometer bias while
    // GPS reports a stationary vehicle at the known truth state.
    system.update(sensors, physics, nav, 0.01);
    constexpr double dt = 0.01;
    constexpr double injectedAccelBias = 0.5;
    double maxVelocityBiasCrossCov = 0.0;
    for (int step = 0; step < 30000; ++step) {
        sensors.accelX[0] = injectedAccelBias;
        sensors.gpsUpdated[0] = (step % 100) == 0; // documented 1 Hz GPS
        system.update(sensors, physics, nav, dt);
        maxVelocityBiasCrossCov = std::max(
            maxVelocityBiasCrossCov, std::abs(nav.covarianceFull[0][3 * 15 + 9]));
    }

    const auto& covariance = nav.covarianceFull[0];
    const double positionVelocityCrossCov = covariance[0 * 15 + 3];
    const double velocityBiasCrossCov = covariance[3 * 15 + 9];
    const bool bounded = [&]() {
        for (double value : covariance) {
            if (!std::isfinite(value) || std::abs(value) > 1.0e6) return false;
        }
        return true;
    }();
    const double biasError = std::abs(nav.estAccelBiasX[0] - injectedAccelBias);
    const double qNorm = std::sqrt(
        nav.estQw[0] * nav.estQw[0] + nav.estQx[0] * nav.estQx[0] +
        nav.estQy[0] * nav.estQy[0] + nav.estQz[0] * nav.estQz[0]);

    std::printf("  estimated accel bias x=%.4f m/s^2, cross Ppv=%.6g, Pvb=%.6g\n",
                nav.estAccelBiasX[0], positionVelocityCrossCov, velocityBiasCrossCov);
    check(biasError < 0.10,
          "GPS velocity corrections drive the accelerometer bias estimate toward truth");
    check(std::abs(positionVelocityCrossCov) > 1e-10 &&
              maxVelocityBiasCrossCov > 1e-10,
          "INS covariance develops position/velocity and velocity/bias coupling");
    check(bounded, "full covariance remains finite and bounded");
    check(std::abs(qNorm - 1.0) < 1e-10,
          "EKF attitude correction preserves quaternion normalization");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
