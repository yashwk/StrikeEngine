#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/models/physics/earth/EarthFrames.hpp>

#include <cmath>
#include <cstdio>

using namespace StrikeEngine::Kernel;
using namespace StrikeEngine::Models;

namespace {

VehicleInitState makeVehicle()
{
    VehicleInitState init{};
    init.pz = 1000.0;
    init.qw = 1.0;
    init.mass = 100.0;
    return init;
}

VehicleConfig vacuumConfig()
{
    VehicleConfig config;
    config.aero.referenceArea = 0.0;
    config.aero.cd = 0.0;
    return config;
}

} // namespace

int main()
{
    std::printf("=== earth_frames_test: ENU/NED frame transforms ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    const GeodeticCoordinate origin{
        35.0 * std::acos(-1.0) / 180.0,
        72.0 * std::acos(-1.0) / 180.0,
        1500.0};
    const EarthFrames::Vector3 local{12.0, -7.0, 3.5};
    const auto reconstructed = EarthFrames::enuToEcef(local, origin);
    const auto roundTrip = EarthFrames::ecefToEnu(reconstructed, origin);
    check(std::abs(roundTrip[0] - local[0]) < 1e-8 &&
              std::abs(roundTrip[1] - local[1]) < 1e-8 &&
              std::abs(roundTrip[2] - local[2]) < 1e-8,
          "ECEF-to-ENU and ENU-to-ECEF round-trip local displacement");

    const EarthFrames::Vector3 ned{4.0, -2.0, 8.0};
    const auto nedRoundTrip = EarthFrames::nedToEnu(EarthFrames::enuToNed(ned));
    check(std::abs(nedRoundTrip[0] - ned[0]) < 1e-12 &&
              std::abs(nedRoundTrip[1] - ned[1]) < 1e-12 &&
              std::abs(nedRoundTrip[2] - ned[2]) < 1e-12,
          "ENU/NED axis permutation round-trips with down/up sign");

    const auto equatorCentrifugal = EarthFrames::localCentrifugalAcceleration({
        0.0, 0.0, 0.0});
    check(equatorCentrifugal[2] > 0.03 &&
              std::abs(equatorCentrifugal[0]) < 1e-12 &&
              std::abs(equatorCentrifugal[1]) < 1e-12,
          "equatorial centrifugal acceleration points upward in ENU");

    EnvironmentConfig environment;
    environment.earth.includeCentrifugal = true;
    environment.earth.referenceLatitudeRad = 0.0;
    environment.earth.referenceLongitudeRad = 0.0;
    SimulationKernel kernel;
    kernel.setEnvironment(environment);
    kernel.createVehicle(makeVehicle(), vacuumConfig());
    kernel.step(0.1);
    check(kernel.getPhysics().az[0] > -9.80665,
          "CPU truth dynamics applies opt-in centrifugal acceleration");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
