#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>
#include <strikeengine/models/terrain/GlobalTerrain.hpp>

#include <cmath>
#include <cstdio>
#include <string>

using namespace StrikeEngine::Kernel;
using namespace StrikeEngine::Models;

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    std::printf("  [%s] %s\n", condition ? "PASS" : "FAIL", message);
    if (!condition) ++failures;
}

VehicleConfig draglessConfig() {
    VehicleConfig config;
    config.initialMass = 100.0;
    config.massDry = 100.0;
    config.Ixx = config.Iyy = config.Izz = 1.0;
    config.aero.referenceArea = 0.0;
    return config;
}

VehicleInitState localInit() {
    VehicleInitState init{};
    init.pz = 20.0;
    init.vz = -20.0;
    init.mass = 100.0;
    init.qw = 1.0;
    return init;
}

void testRasterSampling() {
    std::printf("=== global terrain raster sampling ===\n");
    const double step = 0.1;
    GlobalTerrain terrain(0.0, 0.0, step, step, 2, 2,
                          {10.0, 20.0, 30.0, 40.0});
    check(terrain.isValid(), "in-memory geodetic raster validates");
    check(std::abs(terrain.elevationM(0.05, 0.05) - 25.0) < 1e-12,
          "terrain uses bilinear interpolation");
    check(terrain.elevationM(1.0, 1.0) == 0.0,
          "out-of-coverage queries return the safe zero fallback");

    GlobalTerrain wrapped(-0.1, -3.14159265358979323846, 0.1,
                          1.5707963267948966, 2, 4,
                          {1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0},
                          0.0, true);
    check(std::abs(wrapped.elevationM(0.0, 3.14159265358979323846) -
                   wrapped.elevationM(0.0, -3.14159265358979323846)) < 1e-12,
          "wrapped rasters normalize the dateline consistently");
}

void testAsciiLoader() {
    std::printf("=== ESRI ASCII terrain loading ===\n");
    std::string error;
    const auto terrain = GlobalTerrain::loadEsriAsciiGrid(
        std::string(STRIKEENGINE_SOURCE_DIR) + "/tests/data/terrain/test.asc", &error);
    check(terrain != nullptr && terrain->isValid(),
          "ESRI ASCII Grid loads into a valid terrain database");
    check(terrain && std::abs(terrain->elevationM(
              0.0 * 3.14159265358979323846 / 180.0,
              180.0 * 3.14159265358979323846 / 180.0) - 10.0) < 1e-12,
          "loader reverses north-first rows and samples cell centers");
}

void testLocalImpact() {
    std::printf("=== local ENU global-terrain impact ===\n");
    auto terrain = std::make_shared<const GlobalTerrain>(
        -0.01, -0.01, 0.01, 0.01, 2, 2,
        std::vector<double>{10.0, 10.0, 10.0, 10.0});
    EnvironmentConfig environment;
    environment.globalTerrain = terrain;

    SimulationKernel kernel;
    kernel.setEnvironment(environment);
    const auto id = kernel.createVehicle(localInit(), draglessConfig());
    kernel.step(1.0);
    check(!kernel.getPhysics().active[id] && !kernel.getStatus().isAlive[id],
          "local ENU flight deactivates on global terrain");
    check(std::abs(kernel.getPhysics().pz[id] - 10.0) < 1e-9,
          "local impact clamps to geodetic terrain elevation");
}

void testEcefImpact() {
    std::printf("=== ECEF global-terrain impact ===\n");
    auto terrain = std::make_shared<const GlobalTerrain>(
        -0.01, -0.01, 0.01, 0.01, 2, 2,
        std::vector<double>{10.0, 10.0, 10.0, 10.0});
    EnvironmentConfig environment;
    environment.earth.useEcefTruth = true;
    environment.globalTerrain = terrain;

    const auto start = geodeticToEcef({0.0, 0.0, 20.0});
    VehicleInitState init{};
    init.px = start.x;
    init.py = start.y;
    init.pz = start.z;
    init.vx = -20.0;
    init.mass = 100.0;
    init.qw = 1.0;

    SimulationKernel kernel;
    kernel.setEnvironment(environment);
    const auto id = kernel.createVehicle(init, draglessConfig());
    kernel.step(1.0);
    const auto impact = ecefToGeodetic({kernel.getPhysics().px[id],
                                        kernel.getPhysics().py[id],
                                        kernel.getPhysics().pz[id]});
    check(!kernel.getPhysics().active[id] && impact.altitudeM >= 9.99,
          "ECEF flight uses geodetic terrain and clamps on the ellipsoid");
}

} // namespace

int main() {
    testRasterSampling();
    testAsciiLoader();
    testLocalImpact();
    testEcefImpact();
    std::printf("global_terrain_test: %s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
