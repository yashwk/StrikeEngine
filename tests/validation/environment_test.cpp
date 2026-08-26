#include <strikeengine/kernel/SimulationKernel.hpp>

#include <cmath>
#include <cstdio>

using namespace StrikeEngine::Kernel;

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

VehicleConfig makeDragOnlyConfig()
{
    VehicleConfig config;
    config.referenceArea = 1.0;
    config.cd = 1.0;
    config.clAlpha = 0.0;
    config.clFin = 0.0;
    return config;
}

} // namespace

int main()
{
    std::printf("=== environment_test: terrain and wind coupling ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    EnvironmentConfig windEnvironment;
    windEnvironment.windVelocity = [](double, double, double, double) {
        return std::array<double, 3>{50.0, 0.0, 0.0};
    };

    SimulationKernel calmKernel;
    SimulationKernel windKernel;
    calmKernel.setEnvironment(EnvironmentConfig{});
    windKernel.setEnvironment(windEnvironment);
    calmKernel.createVehicle(makeVehicle(100.0, 1000.0), makeDragOnlyConfig());
    windKernel.createVehicle(makeVehicle(100.0, 1000.0), makeDragOnlyConfig());
    calmKernel.step(0.1);
    windKernel.step(0.1);

    const double calmSpeed = calmKernel.getPhysics().vx[0];
    const double windSpeed = windKernel.getPhysics().vx[0];
    check(windSpeed > calmSpeed + 1e-6,
          "wind is removed from world velocity before aerodynamic drag");

    EnvironmentConfig terrainEnvironment;
    terrainEnvironment.terrainElevation = [](double x, double) {
        return 50.0 + 0.1 * x;
    };
    SimulationKernel terrainKernel;
    terrainKernel.setEnvironment(terrainEnvironment);
    auto terrainVehicle = makeVehicle(30.0, 70.0);
    terrainVehicle.vz = -20.0;
    const auto terrainId = terrainKernel.createVehicle(
        terrainVehicle, makeDragOnlyConfig());
    terrainKernel.step(1.0);

    const auto& physics = terrainKernel.getPhysics();
    const double expectedGround = terrainEnvironment.terrainElevation(
        physics.px[terrainId], physics.py[terrainId]);
    check(!physics.active[terrainId],
          "terrain impact deactivates the entity");
    check(std::abs(physics.pz[terrainId] - expectedGround) < 1e-10,
          "terrain impact clamps altitude to the local terrain elevation");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
