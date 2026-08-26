#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>

#include <cmath>
#include <cstdio>

using namespace StrikeEngine::Kernel;
using namespace StrikeEngine::Models;

namespace {

VehicleInitState makeVehicle(double vx, double pz)
{
    VehicleInitState init{};
    init.px = 0.0;
    init.py = 0.0;
    init.pz = pz;
    init.vx = vx;
    init.vy = 0.0;
    init.vz = 0.0;
    init.qw = 1.0;
    init.qx = 0.0;
    init.qy = 0.0;
    init.qz = 0.0;
    init.mass = 100.0;
    return init;
}

VehicleConfig makeVacuumConfig()
{
    VehicleConfig config;
    config.referenceArea = 0.0;
    config.cd = 0.0;
    config.clAlpha = 0.0;
    config.clFin = 0.0;
    return config;
}

} // namespace

int main()
{
    std::printf("=== earth_test: WGS84 conversions and local earth effects ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    const auto equator = geodeticToEcef({0.0, 0.0, 0.0});
    check(std::abs(equator.x - EarthModel::semiMajorAxisM) < 1e-6 &&
              std::abs(equator.y) < 1e-12 && std::abs(equator.z) < 1e-12,
          "WGS84 equator origin maps to the semi-major axis");

    const auto pole = geodeticToEcef({0.5 * std::acos(-1.0), 0.0, 0.0});
    check(std::abs(pole.z - EarthModel::semiMinorAxisM) < 1e-6 &&
              std::abs(pole.x) < 1e-9 && std::abs(pole.y) < 1e-9,
          "WGS84 pole maps to the semi-minor axis");

    const GeodeticCoordinate original{
        35.0 * std::acos(-1.0) / 180.0,
        72.0 * std::acos(-1.0) / 180.0,
        1500.0};
    const auto roundTrip = ecefToGeodetic(geodeticToEcef(original));
    check(std::abs(roundTrip.latitudeRad - original.latitudeRad) < 1e-11 &&
              std::abs(roundTrip.longitudeRad - original.longitudeRad) < 1e-11 &&
              std::abs(roundTrip.altitudeM - original.altitudeM) < 1e-5,
          "ECEF and geodetic conversion round-trip preserves position");

    check(normalGravity(0.0) < normalGravity(0.5 * std::acos(-1.0) / 2.0) &&
              normalGravity(0.0, 1000.0) < normalGravity(0.0),
          "normal gravity varies with latitude and decreases with altitude");

    const auto coriolis = localCoriolisAcceleration(
        0.25 * std::acos(-1.0), {100.0, 0.0, 0.0});
    check(coriolis[1] < 0.0 && coriolis[2] > 0.0 &&
              std::abs(coriolis[0]) < 1e-15,
          "local ENU Coriolis acceleration has the expected eastward signs");

    SimulationKernel flatKernel;
    SimulationKernel wgs84Kernel;
    EnvironmentConfig wgs84Environment;
    wgs84Environment.earth.useWgs84Gravity = true;
    wgs84Environment.earth.referenceLatitudeRad = 0.0;
    flatKernel.setEnvironment(EnvironmentConfig{});
    wgs84Kernel.setEnvironment(wgs84Environment);
    const auto vacuum = makeVacuumConfig();
    flatKernel.createVehicle(makeVehicle(0.0, 1000.0), vacuum);
    wgs84Kernel.createVehicle(makeVehicle(0.0, 1000.0), vacuum);
    flatKernel.step(0.1);
    wgs84Kernel.step(0.1);
    check(wgs84Kernel.getPhysics().vz[0] > flatKernel.getPhysics().vz[0],
          "opt-in WGS84 gravity changes CPU truth acceleration");

    EnvironmentConfig coriolisEnvironment;
    coriolisEnvironment.earth.includeCoriolis = true;
    coriolisEnvironment.earth.referenceLatitudeRad =
        0.25 * std::acos(-1.0);
    SimulationKernel coriolisKernel;
    coriolisKernel.setEnvironment(coriolisEnvironment);
    coriolisKernel.createVehicle(makeVehicle(100.0, 1000.0), vacuum);
    coriolisKernel.step(1.0);
    check(coriolisKernel.getPhysics().vy[0] < 0.0,
          "opt-in Coriolis produces the expected northward deflection");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
