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
    init.vx = 100.0;
    init.qw = 1.0;
    init.mass = 100.0;
    return init;
}

VehicleConfig vacuumConfig()
{
    VehicleConfig config;
    config.referenceArea = 0.0;
    config.cd = 0.0;
    return config;
}

} // namespace

int main()
{
    std::printf("=== earth_transport_test: moving-origin transport rates ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    const GeodeticCoordinate equator{0.0, 0.0, 0.0};
    const auto rate = localTransportRateEnu(equator, {100.0, 0.0, 0.0});
    const auto acceleration = localTransportAcceleration(
        equator, {100.0, 0.0, 0.0});
    check(rate[1] > 0.0 && std::abs(rate[0]) < 1e-15 &&
              acceleration[2] > 0.001 && acceleration[2] < 0.002,
          "eastward equatorial motion produces the expected transport lift");

    const GeodeticCoordinate origin{
        35.0 * std::acos(-1.0) / 180.0,
        72.0 * std::acos(-1.0) / 180.0,
        0.0};
    const EarthFrames::Vector3 displacement{2500.0, 4000.0, 125.0};
    const auto moved = EarthFrames::enuToGeodetic(displacement, origin);
    check(moved.latitudeRad > origin.latitudeRad &&
              moved.longitudeRad > origin.longitudeRad &&
              std::abs(moved.altitudeM - displacement[2]) < 3.0,
          "local ENU displacement resolves to a moving geodetic origin");

    EnvironmentConfig environment;
    environment.earth.includeTransportRate = true;
    environment.earth.referenceLatitudeRad = 0.0;
    environment.earth.referenceLongitudeRad = 0.0;
    SimulationKernel kernel;
    kernel.setEnvironment(environment);
    kernel.createVehicle(makeVehicle(), vacuumConfig());
    kernel.step(1.0);
    check(kernel.getPhysics().az[0] > -9.8052,
          "CPU truth dynamics applies opt-in transport acceleration");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
