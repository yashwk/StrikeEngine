// Deterministic failure and damage semantics (backlog MVP).
//  - motor failure stops thrust and mass flow
//  - actuator failure freezes achieved fin deflections
//  - sensor failure stops measurement updates (GPS goes dark)
//  - communication failure zeroes guidance and flies ballistic
//  - damage/structural failure deactivates the entity and dispatches events
//  - each failure type dispatches exactly one event when triggered
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>
#include <strikeengine/kernel/systems/CommandProcessor.hpp>

#include <cmath>
#include <cstdio>
#include <stdexcept>

using namespace StrikeEngine::Kernel;

namespace {

struct EventCounters {
    int motorFailure = 0;
    int actuatorFailure = 0;
    int sensorFailure = 0;
    int structuralFailure = 0;
    int communicationFailure = 0;
    int groundImpact = 0;
    int other = 0;
    std::vector<SimulationEvent> events;
};

void subscribeCounter(SimulationKernel& kernel, EventCounters& counters)
{
    kernel.getEventSystem().subscribe([&counters](const SimulationEvent& evt) {
        counters.events.push_back(evt);
        switch (evt.type) {
            case EventType::MotorFailure: ++counters.motorFailure; break;
            case EventType::ActuatorFailure: ++counters.actuatorFailure; break;
            case EventType::SensorFailure: ++counters.sensorFailure; break;
            case EventType::StructuralFailure: ++counters.structuralFailure; break;
            case EventType::CommunicationFailure: ++counters.communicationFailure; break;
            case EventType::GroundImpact: ++counters.groundImpact; break;
            default: ++counters.other; break;
        }
    });
}

VehicleInitState makeInit(double pz, double mass)
{
    VehicleInitState init;
    init.px = 0.0; init.py = 0.0; init.pz = pz;
    init.vx = 50.0; init.vy = 0.0; init.vz = 0.0;
    init.qw = 1.0; init.qx = 0.0; init.qy = 0.0; init.qz = 0.0;
    init.wx = 0.0; init.wy = 0.0; init.wz = 0.0;
    init.mass = mass;
    init.Ixx = 3.0; init.Iyy = 10.0; init.Izz = 10.0;
    return init;
}

} // namespace

int main()
{
    std::printf("=== failure_test: deterministic failure and damage semantics ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    // ---- Motor failure: thrust stops, mass flow stops ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0xF171u);
        VehicleInitState init = makeInit(5000.0, 500.0);
        VehicleConfig cfg;
        cfg.massDry = 370.0;
        cfg.referenceArea = 0.5;
        cfg.cd = 0.15;
        cfg.clAlpha = 2.5;
        cfg.clFin = 2.0;
        cfg.thrustCurve = { {0.0, 50000.0}, {20.0, 50000.0}, {20.1, 0.0}, {100.0, 0.0} };
        cfg.vacuumIsp = 250.0;
        cfg.seaLevelIsp = 220.0;
        const PhysicsId id = kernel.createVehicle(init, cfg);
        const auto& phys = kernel.getPhysics();

        constexpr double dt = 0.1;
        for (int s = 0; s < 5; ++s) kernel.step(dt);
        const double massAfterBurn = phys.mass[id];
        check(massAfterBurn < 500.0 - 1e-9,
              "motor burns fuel: mass decreases during boost");

        kernel.failEntity(id, FailureMode::MotorFailure);
        for (int s = 0; s < 3; ++s) kernel.step(dt);
        const double massAfterFailure = phys.mass[id];
        check(massAfterFailure == massAfterBurn,
              "motor failure stops mass flow (mass frozen, thrust = 0)");
    }

    // ---- Actuator failure: achieved fins freeze (no servo motion) ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0xF172u);
        VehicleInitState init = makeInit(2000.0, 200.0);
        VehicleConfig cfg;
        cfg.referenceArea = 0.5;
        cfg.cd = 0.15;
        cfg.clAlpha = 2.5;
        cfg.clFin = 2.0;
        const PhysicsId id = kernel.createVehicle(init, cfg);

        SimulationCommand cmd;
        cmd.entityId = id;
        cmd.mode = GuidanceMode::Waypoint;
        cmd.targetX = 2000.0; cmd.targetY = 0.0; cmd.targetZ = 0.0;
        cmd.targetVx = 0.0; cmd.targetVy = 0.0; cmd.targetVz = 0.0;
        cmd.maxAccel = 30.0;
        kernel.queueCommand(cmd);

        constexpr double dt = 0.05;
        for (int s = 0; s < 6; ++s) kernel.step(dt);
        const auto& phys = kernel.getPhysics();
        const double finBefore = phys.finPitch[id];
        check(std::abs(finBefore) > 1e-6,
              "actuator precondition: fins have moved under guidance");

        kernel.failEntity(id, FailureMode::ActuatorFailure);
        for (int s = 0; s < 3; ++s) kernel.step(dt);
        const double finAfter = phys.finPitch[id];
        check(finAfter == finBefore,
              "actuator failure freezes achieved fin deflection");
    }

    // ---- Sensor failure: measurements stop updating (GPS goes dark) ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0xF173u);
        const PhysicsId id = kernel.createVehicle(makeInit(3000.0, 150.0));
        const auto& sensors = kernel.getSensors();

        constexpr double dt = 0.1;
        bool sawGps = false;
        for (int s = 0; s < 12; ++s) {
            kernel.step(dt);
            if (sensors.gpsUpdated[id]) sawGps = true;
        }
        check(sawGps, "sensor precondition: GPS updated at least once (1 Hz)");

        kernel.failEntity(id, FailureMode::SensorFailure);
        bool gpsStayedDark = true;
        for (int s = 0; s < 9; ++s) {
            kernel.step(dt);
            if (sensors.gpsUpdated[id]) gpsStayedDark = false;
        }
        check(gpsStayedDark, "sensor failure stops GPS updates thereafter");
    }

    // ---- Communication failure: guidance ignored, commanded accel zeroed ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0xF174u);
        VehicleInitState init = makeInit(2000.0, 200.0);
        VehicleConfig cfg;
        cfg.referenceArea = 0.5;
        cfg.cd = 0.15;
        cfg.clAlpha = 2.5;
        cfg.clFin = 2.0;
        const PhysicsId id = kernel.createVehicle(init, cfg);

        SimulationCommand cmd;
        cmd.entityId = id;
        cmd.mode = GuidanceMode::Waypoint;
        cmd.targetX = 5000.0; cmd.targetY = 3000.0; cmd.targetZ = -200.0;
        cmd.targetVx = 0.0; cmd.targetVy = 0.0; cmd.targetVz = 0.0;
        cmd.maxAccel = 50.0;
        kernel.queueCommand(cmd);

        constexpr double dt = 0.1;
        for (int s = 0; s < 2; ++s) kernel.step(dt);
        const auto& guidance = kernel.getGuidance();
        const double magBefore = std::sqrt(
            guidance.commandedAccelX[id] * guidance.commandedAccelX[id] +
            guidance.commandedAccelY[id] * guidance.commandedAccelY[id] +
            guidance.commandedAccelZ[id] * guidance.commandedAccelZ[id]);
        check(magBefore > 1.0,
              "communication precondition: guidance commands non-zero acceleration");

        kernel.failEntity(id, FailureMode::CommunicationFailure);
        for (int s = 0; s < 2; ++s) kernel.step(dt);
        const double magAfter = std::sqrt(
            guidance.commandedAccelX[id] * guidance.commandedAccelX[id] +
            guidance.commandedAccelY[id] * guidance.commandedAccelY[id] +
            guidance.commandedAccelZ[id] * guidance.commandedAccelZ[id]);
        check(magAfter == 0.0,
              "communication failure zeroes commanded acceleration (fly ballistic)");
    }

    // ---- Damage and structural failure ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0xF175u);
        EventCounters counters;
        subscribeCounter(kernel, counters);
        const PhysicsId id = kernel.createVehicle(makeInit(2000.0, 200.0));
        const auto& phys = kernel.getPhysics();
        const auto& status = kernel.getStatus();

        kernel.applyDamage(id, 999.0);
        check(!status.isAlive[id] && !phys.active[id] && status.health[id] == 0.0,
              "applyDamage(999) deactivates the entity and zeroes health");
        kernel.step(0.1);
        check(counters.structuralFailure == 1,
              "applyDamage dispatches exactly one StructuralFailure event");

        const PhysicsId id2 = kernel.createVehicle(makeInit(2000.0, 200.0));
        kernel.failEntity(id2, FailureMode::StructuralFailure);
        check(!status.isAlive[id2] && !phys.active[id2] && status.health[id2] == 0.0,
              "failEntity(StructuralFailure) deactivates the entity");
        kernel.step(0.1);
        check(counters.structuralFailure == 2,
              "failEntity(StructuralFailure) dispatches the event");
    }

    // ---- Events: each failure type dispatches exactly once when triggered ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0xF176u);
        EventCounters counters;
        subscribeCounter(kernel, counters);
        const PhysicsId id = kernel.createVehicle(makeInit(3000.0, 150.0));
        const auto& status = kernel.getStatus();
        const auto& phys = kernel.getPhysics();

        constexpr double dt = 0.1;
        kernel.failEntity(id, FailureMode::MotorFailure);
        const double tMotor = kernel.getSimulationTime();
        kernel.step(dt);
        check(counters.motorFailure == 1 && status.motorFailed[id] &&
                  phys.motorFailed[id] &&
                  counters.events[0].entityId == id &&
                  counters.events[0].timestamp == tMotor,
              "MotorFailure event dispatched with entity id and timestamp");

        kernel.failEntity(id, FailureMode::ActuatorFailure);
        kernel.step(dt);
        check(counters.actuatorFailure == 1 && status.actuatorFailed[id] &&
                  phys.actuatorFailed[id],
              "ActuatorFailure event dispatched and flags set");

        kernel.failEntity(id, FailureMode::SensorFailure);
        kernel.step(dt);
        check(counters.sensorFailure == 1 && status.sensorFailed[id],
              "SensorFailure event dispatched and flag set");

        kernel.failEntity(id, FailureMode::CommunicationFailure);
        kernel.step(dt);
        check(counters.communicationFailure == 1 && status.commsFailed[id],
              "CommunicationFailure event dispatched and flag set");

        kernel.failEntity(id, FailureMode::None);
        kernel.step(dt);
        check(counters.structuralFailure == 0 && counters.groundImpact == 0 &&
                  counters.other == 0,
              "failEntity(None) is a no-op (no event dispatched)");

        kernel.failEntity(id, FailureMode::StructuralFailure);
        kernel.step(dt);
        check(counters.structuralFailure == 1 && !status.isAlive[id] &&
                  !phys.active[id] && status.health[id] == 0.0,
              "StructuralFailure event dispatched and entity deactivated");

        const int totalFailure = counters.motorFailure + counters.actuatorFailure +
            counters.sensorFailure + counters.communicationFailure +
            counters.structuralFailure;
        check(totalFailure == 5 && counters.groundImpact == 0 && counters.other == 0,
              "exactly one event per triggered failure type, no stray events");
    }

    // ---- Argument validation and partial damage ----
    {
        SimulationKernel kernel;
        const PhysicsId id = kernel.createVehicle(makeInit(2000.0, 100.0));
        const auto& status = kernel.getStatus();

        kernel.applyDamage(id, 40.0);
        check(status.health[id] == 60.0 && status.isAlive[id],
              "partial damage reduces health without deactivation");

        bool threwRange = false;
        try { kernel.failEntity(id + 100, FailureMode::MotorFailure); }
        catch (const std::out_of_range&) { threwRange = true; }
        check(threwRange, "failEntity with out-of-range id throws std::out_of_range");

        bool threwDamage = false;
        try { kernel.applyDamage(id, -5.0); }
        catch (const std::invalid_argument&) { threwDamage = true; }
        check(threwDamage, "applyDamage with negative damage throws std::invalid_argument");

        bool threwDamageRange = false;
        try { kernel.applyDamage(id + 100, 1.0); }
        catch (const std::out_of_range&) { threwDamageRange = true; }
        check(threwDamageRange, "applyDamage with out-of-range id throws std::out_of_range");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
