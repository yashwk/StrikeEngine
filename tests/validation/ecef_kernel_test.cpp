#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>

#include <cmath>
#include <cstdio>

using namespace StrikeEngine::Kernel;
using namespace StrikeEngine::Models;

namespace {

VehicleInitState makeVehicle(const EcefCoordinate& position, double vx = 0.0)
{
    VehicleInitState init{};
    init.px = position.x;
    init.py = position.y;
    init.pz = position.z;
    init.vx = vx;
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

EnvironmentConfig ecefEnvironment()
{
    EnvironmentConfig environment;
    environment.earth.useEcefTruth = true;
    environment.earth.useSphericalGravity = true;
    environment.earth.includeCoriolis = true;
    environment.earth.includeCentrifugal = true;
    environment.earth.referenceLatitudeRad = 0.0;
    environment.earth.referenceLongitudeRad = 0.0;
    return environment;
}

} // namespace

int main()
{
    std::printf("=== ecef_kernel_test: kernel ECEF truth mode ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    const auto launchPosition = geodeticToEcef({0.0, 0.0, 1000.0});
    auto environment = ecefEnvironment();
    SimulationKernel kernel;
    kernel.setEnvironment(environment);
    kernel.setRandomSeed(7);
    kernel.createVehicle(makeVehicle(launchPosition), vacuumConfig());
    kernel.step(0.1);

    const auto& physics = kernel.getPhysics();
    const auto afterStep = ecefToGeodetic({
        physics.px[0], physics.py[0], physics.pz[0]});
    check(std::abs(physics.px[0]) > 6.0e6 &&
              std::abs(afterStep.altitudeM - 1000.0) < 1.0 &&
              physics.px[0] < launchPosition.x,
          "ECEF truth keeps absolute position while applying radial gravity");
    check(std::abs(kernel.getNavigation().estPx[0]) > 6.0e6,
          "navigation alignment preserves the ECEF position frame");

    kernel.runSteps(11, 0.1);
    check(std::abs(kernel.getSensors().gpsPosX[0]) > 1.0e6,
          "GPS measurements remain in the selected ECEF frame");

    auto j2Environment = ecefEnvironment();
    j2Environment.earth.includeJ2Gravity = true;
    j2Environment.earth.useSphericalGravity = false;
    j2Environment.earth.useWgs84Gravity = true;
    SimulationKernel j2Kernel;
    j2Kernel.setEnvironment(j2Environment);
    j2Kernel.createVehicle(makeVehicle(launchPosition), vacuumConfig());
    j2Kernel.step(0.1);
    check(j2Kernel.getPhysics().px[0] < launchPosition.x &&
              std::abs(j2Kernel.getPhysics().py[0]) < 1e-3,
          "ECEF truth applies opt-in J2 gravity consistently with WGS84 position");

    SimulationKernel impactKernel;
    impactKernel.setEnvironment(ecefEnvironment());
    const auto impactPosition = geodeticToEcef({0.0, 0.0, 10.0});
    impactKernel.createVehicle(makeVehicle(impactPosition, -20.0), vacuumConfig());
    impactKernel.step(1.0);
    const auto& impactPhysics = impactKernel.getPhysics();
    const auto impactGeodetic = ecefToGeodetic({
        impactPhysics.px[0], impactPhysics.py[0], impactPhysics.pz[0]});
    check(!impactPhysics.active[0] && std::abs(impactGeodetic.altitudeM) < 1e-5,
          "ECEF ground impact deactivates and clamps to the WGS84 ellipsoid");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
