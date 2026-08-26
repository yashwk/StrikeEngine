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
    config.referenceArea = 0.0;
    config.cd = 0.0;
    return config;
}

} // namespace

int main()
{
    std::printf("=== spherical_gravity_test: ECEF point-mass gravity ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    const EcefCoordinate equator{
        EarthModel::semiMajorAxisM, 0.0, 0.0};
    const auto equatorGravity = sphericalGravityAccelerationEcef(equator);
    check(equatorGravity.x < 0.0 &&
              std::abs(equatorGravity.y) < 1e-12 &&
              std::abs(equatorGravity.z) < 1e-12 &&
              std::abs(std::abs(equatorGravity.x) -
                       sphericalGravityMagnitude(EarthModel::semiMajorAxisM)) < 1e-12,
          "spherical gravity is radial and has the expected equatorial magnitude");

    const auto localGravity = EarthFrames::localSphericalGravityAcceleration({
        0.0, 0.0, 1000.0});
    check(localGravity[2] < -9.7 &&
              std::abs(localGravity[0]) < 1e-10 &&
              std::abs(localGravity[1]) < 1e-10,
          "local ENU spherical gravity points down at the equator");

    EnvironmentConfig sphericalEnvironment;
    sphericalEnvironment.earth.useSphericalGravity = true;
    sphericalEnvironment.earth.referenceLatitudeRad = 0.0;
    sphericalEnvironment.earth.referenceLongitudeRad = 0.0;
    SimulationKernel flatKernel;
    SimulationKernel sphericalKernel;
    flatKernel.setEnvironment(EnvironmentConfig{});
    sphericalKernel.setEnvironment(sphericalEnvironment);
    const auto vacuum = vacuumConfig();
    flatKernel.createVehicle(makeVehicle(), vacuum);
    sphericalKernel.createVehicle(makeVehicle(), vacuum);
    flatKernel.step(0.1);
    sphericalKernel.step(0.1);
    check(sphericalKernel.getPhysics().az[0] > flatKernel.getPhysics().az[0],
          "opt-in spherical gravity changes CPU truth acceleration");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
