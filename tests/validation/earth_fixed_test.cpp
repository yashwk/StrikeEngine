#include <strikeengine/models/physics/earth/EarthFixedPropagator.hpp>

#include <cmath>
#include <cstdio>

using namespace StrikeEngine::Models;

int main()
{
    std::printf("=== earth_fixed_test: rotating-ECEF propagation ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    const EcefCoordinate equator{
        EarthModel::semiMajorAxisM, 0.0, 0.0};
    const auto rotatingAcceleration = EarthFixed::acceleration(
        equator, {}, {}, {true, false, true});
    check(rotatingAcceleration.x < 0.0 &&
              std::abs(rotatingAcceleration.y) < 1e-12 &&
              std::abs(rotatingAcceleration.z) < 1e-12,
          "rotating ECEF gravity and centrifugal terms point radially inward");

    EcefDynamicsOptions j2Options;
    j2Options.includeCoriolis = false;
    j2Options.includeCentrifugal = false;
    j2Options.includeJ2Gravity = true;
    const auto j2Acceleration = EarthFixed::acceleration(
        equator, {}, {}, j2Options);
    const auto pointMassAcceleration = sphericalGravityAccelerationEcef(equator);
    check(j2Acceleration.x < pointMassAcceleration.x &&
              std::abs(j2Acceleration.y) < 1e-12 &&
              std::abs(j2Acceleration.z) < 1e-12,
          "standalone ECEF propagator supports opt-in J2 gravity");

    EcefDynamicsOptions noEarthTerms;
    noEarthTerms.includeGravity = false;
    noEarthTerms.includeCoriolis = false;
    noEarthTerms.includeCentrifugal = false;
    const EcefState initial{{1000.0, -2000.0, 3000.0}, {4.0, -5.0, 6.0}};
    const auto free = propagateEcef(initial, 2.0, noEarthTerms, {}, 4);
    check(std::abs(free.position.x - 1008.0) < 1e-12 &&
              std::abs(free.position.y + 2010.0) < 1e-12 &&
              std::abs(free.position.z - 3012.0) < 1e-12 &&
              std::abs(free.velocity.x - 4.0) < 1e-12 &&
              std::abs(free.velocity.y + 5.0) < 1e-12 &&
              std::abs(free.velocity.z - 6.0) < 1e-12,
          "RK4 ECEF propagator preserves the exact constant-velocity limit");

    const auto coriolis = EarthFixed::acceleration(
        equator, {0.0, 100.0, 0.0}, {}, {false, true, false});
    check(coriolis.x > 0.0 && std::abs(coriolis.y) < 1e-12 &&
              std::abs(coriolis.z) < 1e-12,
          "rotating ECEF Coriolis acceleration has the expected sign");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
