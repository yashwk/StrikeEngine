#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/ScenarioConfig.hpp>
#include <strikeengine/kernel/config/ConfigSerialization.hpp>
#include <strikeengine/simulation/BatchRunner.hpp>

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

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

    // primaryEntityIndex is written as a scenario-list index but consumed as a
    // kernel entity id. Those differ when a scenario has rail-launched
    // entities, which do not exist until they spawn: resolving against the list
    // size alone let an id past the end of the physics block through.
    {
        ScenarioConfig rail;
        rail.name = "rail-launch-primary";
        for (int k = 0; k < 3; ++k) {
            ScenarioEntityConfig e = makeScenario(0.0).entities[0];
            e.name = "e" + std::to_string(k);
            e.launch.enabled = (k == 2);
            rail.entities.push_back(e);
        }
        rail.primaryEntityIndex = 2;

        SimulationKernel railKernel;
        rail.loadInto(railKernel);
        check(railKernel.getPhysics().size == 2,
              "rail-launched entity is not created at t=0");

        bool threw = false;
        try {
            (void)resolvePrimaryEntityId(rail, railKernel.getPhysics().size);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        check(threw,
              "primaryEntityIndex past the t=0 entity count is rejected, not indexed");

        rail.primaryEntityIndex = 0;
        check(resolvePrimaryEntityId(rail, railKernel.getPhysics().size) == 0,
              "a t=0 primary entity still resolves");

        // A primary index outside the entity list must fail at load.
        ScenarioConfig bad;
        bad.name = "bad-primary";
        bad.entities.push_back(makeScenario(0.0).entities[0]);
        bad.primaryEntityIndex = 7;
        bool loadThrew = false;
        try {
            (void)deserializeScenario(serializeScenario(bad));
        } catch (const std::runtime_error&) {
            loadThrew = true;
        }
        check(loadThrew, "out-of-range primary_entity_index fails at load");

        ScenarioConfig empty;
        empty.name = "no-entities";
        bool emptyThrew = false;
        try {
            (void)deserializeScenario(serializeScenario(empty));
        } catch (const std::runtime_error&) {
            emptyThrew = true;
        }
        check(emptyThrew, "a scenario with no entities fails at load");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
