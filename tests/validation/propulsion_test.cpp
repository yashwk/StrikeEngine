#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/ConfigSerialization.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>
#include <strikeengine/models/physics/propulsion/PropulsionModel.hpp>
#include <strikeengine/models/physics/propulsion/ThrustCurve.hpp>

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

using namespace StrikeEngine::Kernel;
using namespace StrikeEngine::Models;

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    std::printf("  [%s] %s\n", condition ? "PASS" : "FAIL", message);
    if (!condition) ++failures;
}

VehicleInitState initState() {
    VehicleInitState init{};
    init.qw = 1.0;
    init.pz = 100.0;
    init.mass = 100.0;
    return init;
}

VehicleConfig stageConfig() {
    VehicleConfig config;
    config.initialMass = 100.0;
    config.massDry = 90.0;
    config.Ixx = config.Iyy = config.Izz = 1.0;
    config.aero.referenceArea = 0.0;

    StageConfig stage;
    stage.thrustCurve = {{0.0, 10000.0}, {10.0, 10000.0}};
    stage.propellantMassKg = 10.0;
    stage.maxGimbalPitchRad = 0.2;
    stage.maxGimbalYawRad = 0.15;
    stage.gimbalTimeConstantSec = 0.01;
    stage.enginePositionZ = 1.0;
    config.propulsion.stages.push_back(stage);
    return config;
}

void testCurveAndConfigValidation() {
    std::printf("=== strict propulsion validation ===\n");
    std::string error;
    check(ThrustCurve({{0.0, 1.0}, {1.0, 0.0}}).validate(&error),
          "valid thrust curve is accepted");
    check(!ThrustCurve({{0.0, 1.0}, {0.0, 0.0}}).validate(&error) && !error.empty(),
          "duplicate thrust times are rejected");
    check(!ThrustCurve({{0.0, -1.0}, {1.0, 0.0}}).validate(&error),
          "negative thrust is rejected");
    check(!ThrustCurve({{1.0, 1.0}, {2.0, 0.0}}).validate(&error),
          "curves must start at ignition time zero");

    bool threw = false;
    try {
        auto config = stageConfig();
        config.propulsion.stages[0].vacuumIsp = 0.0;
        SimulationKernel kernel;
        kernel.createVehicle(initState(), config);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    check(threw, "invalid Isp is rejected by createVehicle");

    threw = false;
    try {
        auto config = stageConfig();
        config.propulsion.stages[0].thrustCurve[1].time_s = 0.0;
        SimulationKernel kernel;
        kernel.createVehicle(initState(), config);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    check(threw, "invalid curve is rejected before registration");
}

void testTransients() {
    std::printf("=== ignition and shutdown transients ===\n");
    PropulsionModelOptions options;
    options.ignitionDelaySec = 1.0;
    options.ignitionRampSec = 1.0;
    options.shutdownTimeSec = 3.0;
    options.shutdownRampSec = 1.0;
    PropulsionModel model(
        ThrustCurve({{0.0, 1000.0}, {10.0, 1000.0}}), 250.0, 250.0, options);

    check(model.evaluate(0.5, 0.0).massFlowRate_kg_s == 0.0,
          "ignition delay holds thrust and flow at zero");
    check(std::abs(model.evaluate(1.5, 0.0).thrustBodyX - 500.0) < 1e-9,
          "ignition ramp reaches half thrust at half ramp time");
    check(std::abs(model.evaluate(2.0, 0.0).thrustBodyX - 1000.0) < 1e-9,
          "ignition ramp reaches full thrust");
    check(std::abs(model.evaluate(4.5, 0.0).thrustBodyX - 500.0) < 1e-9,
          "shutdown ramp reaches half thrust");
    check(model.evaluate(5.0, 0.0).massFlowRate_kg_s == 0.0,
          "shutdown ramp fully cuts thrust and flow");
    check(std::abs(model.burnDuration() - 5.0) < 1e-9,
          "burn duration includes ignition delay and shutdown ramp");
}

void testVectoringAndTorque() {
    std::printf("=== thrust vector control ===\n");
    auto config = stageConfig();
    config.propulsion.stages[0].enginePositionZ = 0.0;
    SimulationKernel kernel;
    const PhysicsId id = kernel.createVehicle(initState(), config);
    kernel.setThrustVectorCommand(id, 0.4, 0.3);
    kernel.step(0.1);

    const auto& physics = kernel.getPhysics();
    check(std::abs(physics.gimbalPitch[id] - 0.2) < 1e-9,
          "pitch gimbal is limited to configured envelope");
    check(std::abs(physics.gimbalYaw[id] - 0.15) < 1e-9,
          "yaw gimbal is limited to configured envelope");
    check(physics.vy[id] > 0.0 && physics.vz[id] < 0.0,
          "vector thrust produces lateral and nose-up acceleration");

    auto torqueConfig = stageConfig();
    SimulationKernel torqueKernel;
    const PhysicsId torqueId = torqueKernel.createVehicle(initState(), torqueConfig);
    torqueKernel.setThrustVectorCommand(torqueId, 0.0, 0.0);
    torqueKernel.step(0.01);
    check(torqueKernel.getPhysics().wy[torqueId] > 0.0,
          "engine offset produces the expected propulsion pitch torque");
}

void testFailures() {
    std::printf("=== engine and tank failures ===\n");
    auto config = stageConfig();
    SimulationKernel kernel;
    const PhysicsId engine = kernel.createVehicle(initState(), config);
    const PhysicsId tank = kernel.createVehicle(initState(), config);
    int engineEvents = 0;
    int tankEvents = 0;
    kernel.getEventSystem().subscribe([&](const SimulationEvent& event) {
        if (event.type == EventType::EngineFailure) ++engineEvents;
        if (event.type == EventType::TankFailure) ++tankEvents;
    });

    kernel.failEntity(engine, FailureMode::EngineFailure);
    kernel.failEntity(tank, FailureMode::TankFailure);
    kernel.step(0.01);
    const auto& physics = kernel.getPhysics();
    const auto& status = kernel.getStatus();
    check(status.engineFailed[engine] && physics.engineFailed[engine],
          "engine failure is mirrored into status and physics truth");
    check(status.tankFailed[tank] && physics.tankFailed[tank],
          "tank failure is mirrored into status and physics truth");
    check(std::abs(physics.vx[engine]) < 1e-12 && std::abs(physics.vx[tank]) < 1e-12,
          "engine and tank failures stop thrust without deactivating the vehicle");
    check(engineEvents == 1 && tankEvents == 1,
          "engine and tank failure events are dispatched exactly once");
}

void testSerialization() {
    std::printf("=== propulsion serialization ===\n");
    auto config = stageConfig();
    auto& stage = config.propulsion.stages[0];
    stage.ignitionDelaySec = 0.3;
    stage.ignitionRampSec = 0.4;
    stage.shutdownTimeSec = 8.0;
    stage.shutdownRampSec = 0.2;
    stage.maxGimbalRateRadPerSec = 3.0;
    stage.enginePositionY = -0.25;
    const auto restored = deserializeVehicleConfig(serializeVehicleConfig(config));
    const auto& copy = restored.propulsion.stages[0];
    check(std::abs(copy.ignitionDelaySec - 0.3) < 1e-12 &&
          std::abs(copy.ignitionRampSec - 0.4) < 1e-12 &&
          std::abs(copy.shutdownTimeSec - 8.0) < 1e-12 &&
          std::abs(copy.maxGimbalRateRadPerSec - 3.0) < 1e-12 &&
          std::abs(copy.enginePositionY + 0.25) < 1e-12,
          "transient, TVC, and engine-position fields survive round-trip");
}

} // namespace

int main() {
    testCurveAndConfigValidation();
    testTransients();
    testVectoringAndTorque();
    testFailures();
    testSerialization();
    std::printf("propulsion_test: %s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
