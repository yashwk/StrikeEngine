#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/ScenarioConfig.hpp>
#include <strikeengine/simulation/BatchRunner.hpp>

#include <cmath>
#include <cstdio>

using namespace StrikeEngine::Kernel;

namespace {

ScenarioConfig makeScenario(double initialSpeed)
{
    ScenarioConfig scenario;
    scenario.name = "batch-scenario";
    ScenarioEntityConfig entity;
    entity.initState.px = 0.0;
    entity.initState.py = 0.0;
    entity.initState.pz = 1000.0;
    entity.initState.vx = initialSpeed;
    entity.initState.vy = 0.0;
    entity.initState.vz = 0.0;
    entity.initState.qw = 1.0;
    entity.initState.qx = 0.0;
    entity.initState.qy = 0.0;
    entity.initState.qz = 0.0;
    entity.initState.mass = 100.0;
    entity.vehicleConfig.aero.referenceArea = 0.4;
    entity.vehicleConfig.massDry = 90.0;
    entity.vehicleConfig.aero.cd = 0.2;
    scenario.entities.push_back(entity);
    return scenario;
}

} // namespace

int main()
{
    std::printf("=== scenario_test: config propagation and batch execution ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    ScenarioConfig scenario = makeScenario(100.0);
    scenario.environment.terrainElevation = [](double, double) { return 10.0; };
    scenario.entities[0].initialGuidanceMode = GuidanceMode::Waypoint;
    scenario.entities[0].initialTargetZ = 1000.0;
    scenario.entities[0].initialMaxAccel = 25.0;

    SimulationKernel kernel;
    scenario.loadInto(kernel);
    check(kernel.getPhysics().massDry[0] == 90.0 &&
              std::abs(kernel.getPhysics().referenceArea[0] - 0.4) < 1e-12,
          "scenario loading preserves per-vehicle configuration");
    kernel.step(0.01);
    check(kernel.getGuidance().mode[0] == GuidanceMode::Waypoint &&
              std::abs(kernel.getGuidance().maxAccel[0] - 25.0) < 1e-12,
          "scenario loading preserves guidance configuration");

    StrikeEngine::Simulation::BatchRunner runner(0.01, 0.05);
    const auto results = runner.execute({scenario, makeScenario(50.0)});
    check(results.size() == 2 && results[0].scenarioIndex == 0 &&
              results[1].scenarioIndex == 1,
          "batch runner returns one isolated result per scenario");
    check(results[0].entityCount == 1 && results[0].endTime >= 0.05 &&
              results[0].maxSpeed > 0.0,
          "batch runner reports structured execution metrics");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
