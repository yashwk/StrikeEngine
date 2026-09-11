#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>

#include <cstdio>
#include <cmath>
#include <algorithm>

using namespace StrikeEngine::Kernel;

static int failures = 0;
static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

static VehicleInitState makeInit(double px, double py, double pz, double mass = 100.0,
                                 Allegiance allegiance = Allegiance::Friendly) {
    VehicleInitState init{};
    init.px = px; init.py = py; init.pz = pz;
    init.vx = 0; init.vy = 0; init.vz = 0;
    init.qw = 1; init.qx = 0; init.qy = 0; init.qz = 0;
    init.wx = 0; init.wy = 0; init.wz = 0;
    init.mass = mass;
    init.allegiance = allegiance;
    return init;
}

int main() {
    std::printf("=== staging_warhead_test: multi-stage propulsion + warhead fusing ===\n");

    // ---- Part A: two-stage propulsion ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0x5A17u);

        int separations = 0, detonations = 0;
        kernel.getEventSystem().subscribe([&](const SimulationEvent& e) {
            if (e.type == EventType::StageSeparation) ++separations;
            if (e.type == EventType::Detonation) ++detonations;
        });

        VehicleConfig cfg;
        cfg.initialMass = 200.0;
        cfg.massDry = 100.0;
        cfg.Ixx = 10.0; cfg.Iyy = 20.0; cfg.Izz = 20.0;

        StageConfig s0;
        s0.thrustCurve = {{0.0, 50000.0}, {2.0, 50000.0}, {2.001, 0.0}, {100.0, 0.0}};
        s0.dryMassKg = 50.0;
        StageConfig s1;
        s1.thrustCurve = {{0.0, 20000.0}, {1.0, 20000.0}, {1.001, 0.0}, {100.0, 0.0}};
        s1.dryMassKg = 0.0;
        cfg.propulsion.stages = {s0, s1};

        const auto id = kernel.createVehicle(makeInit(0, 0, 1000.0), cfg);
        const auto& phys = kernel.getPhysics();

        check(phys.stageCount[id] == 2, "two stages registered");
        check(phys.stageIndex[id] == 0, "stage 0 active at launch");
        check(std::abs(phys.massDry[id] - 150.0) < 1e-9,
              "initial massDry is final dry + separable dry (100 + 50)");

        for (int step = 0; step < 250; ++step) kernel.step(0.01);  // 2.5 s (> stage-0 burnout)

        check(separations == 1, "exactly one StageSeparation event fired");
        check(phys.stageIndex[id] == 1, "stage 1 is active after separation");
        check(std::abs(phys.massDry[id] - 100.0) < 1e-9,
              "separation drops the spent stage dry mass (150 -> 100)");
        check(phys.mass[id] < 200.0 - 49.0, "mass dropped spent structure and propellant");
        check(phys.Ixx[id] < 10.0, "inertia rescaled down at separation");
        check(detonations == 0, "no warhead detonation on a motor-only vehicle");
    }

    // ---- Part A2: propellant-drawdown burnout (stage cap < fuel pool) ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0xABADu);
        int separations = 0;
        double dumpedMass = -1.0;
        kernel.getEventSystem().subscribe([&](const SimulationEvent& e) {
            if (e.type == EventType::StageSeparation) {
                ++separations;
                dumpedMass = e.dumpedMassKg;
            }
        });

        VehicleConfig cfg;
        cfg.initialMass = 200.0;
        cfg.massDry = 100.0;
        cfg.Ixx = 10.0; cfg.Iyy = 20.0; cfg.Izz = 20.0;

        StageConfig s0;
        // Long thrust curve: positive thrust well past the step window
        // (t = 100 s); burnout must come from propellant drawdown instead.
        s0.thrustCurve = {{0.0, 80000.0}, {100.0, 80000.0}, {100.001, 0.0}};
        s0.vacuumIsp = 250.0;
        s0.seaLevelIsp = 220.0;
        s0.propellantMassKg = 10.0;
        s0.dryMassKg = 50.0;
        StageConfig s1;
        s1.thrustCurve = {{0.0, 20000.0}, {1.0, 20000.0}, {1.001, 0.0}, {100.0, 0.0}};
        s1.dryMassKg = 0.0;
        cfg.propulsion.stages = {s0, s1};

        const auto id = kernel.createVehicle(makeInit(0, 0, 1000.0), cfg);
        const auto& phys = kernel.getPhysics();

        // Fuel pool = initial mass - (final dry + separable dry) = 50 kg,
        // comfortably above the 10 kg stage cap.
        check(std::abs(phys.massDry[id] - 150.0) < 1e-9,
              "drawdown case: initial massDry is final dry + separable dry (100 + 50)");
        check(std::abs(phys.stageMinMass[id] - 150.0) < 1e-9,
              "drawdown case: stage-0 floor is dry mass + reserved (150)");

        for (int step = 0; step < 200; ++step) kernel.step(0.01);  // 2 s (curve would burn to t=100)

        check(separations == 1,
              "drawdown case: propellant exhaustion fires StageSeparation");
        check(phys.stageIndex[id] == 1,
              "drawdown case: stage 1 active (separation from exhaustion, not the t=100 curve end)");
        check(dumpedMass <= 1e-6,
              "drawdown case: exhaustion burnout dumps no leftover propellant (mass already at the floor)");
        check(std::abs(phys.mass[id] - 100.0) < 1e-6,
              "drawdown case: post-separation mass is exactly dry mass (no dump)");
    }

    // ---- Part A3: curve-end burnout dumps leftover propellant ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0xD1CEu);
        int separations = 0;
        double dumpedMass = -1.0;
        kernel.getEventSystem().subscribe([&](const SimulationEvent& e) {
            if (e.type == EventType::StageSeparation) {
                ++separations;
                dumpedMass = e.dumpedMassKg;
            }
        });

        VehicleConfig cfg;
        cfg.initialMass = 200.0;
        cfg.massDry = 100.0;
        cfg.Ixx = 10.0; cfg.Iyy = 20.0; cfg.Izz = 20.0;

        StageConfig s0;
        // Equal vacuum/sea-level Isps so the mass flow is pressure-independent
        // and the burned mass is exactly mdot * burn duration.
        s0.thrustCurve = {{0.0, 50000.0}, {0.5, 50000.0}, {0.501, 0.0}, {100.0, 0.0}};
        s0.vacuumIsp = 200.0;
        s0.seaLevelIsp = 200.0;
        s0.propellantMassKg = 50.0;   // cap = fuel pool (200 - 150 = 50)
        s0.dryMassKg = 50.0;
        StageConfig s1;
        s1.thrustCurve = {{0.0, 20000.0}, {1.0, 20000.0}, {1.001, 0.0}, {100.0, 0.0}};
        s1.dryMassKg = 0.0;
        cfg.propulsion.stages = {s0, s1};

        const auto id = kernel.createVehicle(makeInit(0, 0, 1000.0), cfg);
        const auto& phys = kernel.getPhysics();

        // The curve ends at t = 0.5 s well before the 50 kg cap is drawn down.
        // RK4 samples the thrust curve at the base, two midpoints and the end
        // of each step; the step whose base is the burn cutoff (t = 0.5) burns
        // only its first sample (h/6), so the exact burned mass is
        // mdot * (burnDuration - dt + dt/6) and leftover = cap - burned.
        const double dt = 0.01;
        const double mdot = 50000.0 / (200.0 * 9.80665);
        const double burned = mdot * (0.5 - dt + dt / 6.0);
        const double expectedLeftover = 50.0 - burned;
        const double expectedOldMass = 200.0 - burned;

        for (int step = 0; step < 100; ++step) kernel.step(0.01);  // 1 s (> stage-0 curve end)

        check(separations == 1, "curve-end case: exactly one StageSeparation event fired");
        check(phys.stageIndex[id] == 1, "curve-end case: stage 1 active after separation");
        check(std::abs(phys.massDry[id] - 100.0) < 1e-9,
              "curve-end case: separation drops the spent stage dry mass (150 -> 100)");
        check(std::abs(dumpedMass - expectedLeftover) < 1e-6,
              "curve-end case: dumped mass equals cap minus burned (leftover propellant)");
        check(std::abs(phys.mass[id] - 100.0) < 1e-6,
              "curve-end case: mass after separation is exactly dry + later-stage reserves (leftover dumped)");
        check(std::abs(phys.Ixx[id] - 10.0 * (100.0 / expectedOldMass)) < 1e-9,
              "curve-end case: inertia rescaled by the post-dump mass ratio");
    }

    // ---- Part A4: final-stage burnout reports MotorBurnout once ----
    {
        SimulationKernel kernel;
        int burnouts = 0;
        kernel.getEventSystem().subscribe([&](const SimulationEvent& e) {
            if (e.type == EventType::MotorBurnout) ++burnouts;
        });

        VehicleConfig cfg;
        cfg.initialMass = 200.0;
        cfg.massDry = 100.0;
        cfg.Ixx = 10.0; cfg.Iyy = 20.0; cfg.Izz = 20.0;
        StageConfig s0;
        s0.thrustCurve = {{0.0, 50000.0}, {0.5, 50000.0}, {0.501, 0.0}, {100.0, 0.0}};
        s0.dryMassKg = 0.0;
        cfg.propulsion.stages = {s0};

        kernel.createVehicle(makeInit(0, 0, 1000.0), cfg);
        for (int step = 0; step < 100; ++step) kernel.step(0.01);  // 1 s (> 0.5 s burn)
        check(burnouts == 1, "final-stage burnout dispatches exactly one MotorBurnout");
        for (int step = 0; step < 100; ++step) kernel.step(0.01);
        check(burnouts == 1, "MotorBurnout does not repeat on later steps");
    }

    // ---- Part B: proximity fuse ----
    {
        SimulationKernel kernel;
        int detonations = 0;
        kernel.getEventSystem().subscribe([&](const SimulationEvent& e) {
            if (e.type == EventType::Detonation) ++detonations;
        });

        VehicleConfig cfg;
        cfg.warhead.fusing = FusingType::Proximity;
        cfg.warhead.proximityTriggerM = 40.0;
        cfg.warhead.lethalRadiusM = 60.0;
        const auto id = kernel.createVehicle(makeInit(0, 0, 100.0), cfg);
        const auto tid = kernel.createVehicle(makeInit(30.0, 0, 100.0, 100.0, Allegiance::Hostile));  // ~30 m away, within 40 m

        kernel.step(0.01);
        check(detonations >= 1, "proximity fuse detonates near the target");
        check(!kernel.getStatus().isAlive[tid], "target destroyed within lethal radius");
        check(!kernel.getStatus().isAlive[id] && !kernel.getPhysics().active[id],
              "carrier destroyed by its own detonation");
    }

    // ---- Part C: timed fuse ----
    {
        SimulationKernel kernel;
        int detonations = 0;
        kernel.getEventSystem().subscribe([&](const SimulationEvent& e) {
            if (e.type == EventType::Detonation) ++detonations;
        });

        VehicleConfig cfg;
        cfg.warhead.fusing = FusingType::Timed;
        cfg.warhead.timedDelaySec = 0.2;
        cfg.warhead.lethalRadiusM = 60.0;
        const auto id = kernel.createVehicle(makeInit(0, 0, 100.0), cfg);
        const auto tid = kernel.createVehicle(makeInit(20.0, 0, 100.0, 100.0, Allegiance::Hostile));  // ~20 m away

        for (int step = 0; step < 30; ++step) kernel.step(0.01);  // 0.3 s
        check(detonations >= 1, "timed fuse detonates after its delay");
        check(!kernel.getStatus().isAlive[tid], "target destroyed within lethal radius");
        check(!kernel.getStatus().isAlive[id], "timed detonation also consumes the carrier");
    }

    // ---- Part D: impact fuse (on ground impact) ----
    {
        SimulationKernel kernel;
        int detonations = 0;
        kernel.getEventSystem().subscribe([&](const SimulationEvent& e) {
            if (e.type == EventType::Detonation) ++detonations;
        });

        VehicleConfig cfg;
        cfg.warhead.fusing = FusingType::Impact;
        cfg.warhead.lethalRadiusM = 120.0;
        const auto id = kernel.createVehicle(makeInit(0, 0, 0.5), cfg);   // falls to ground
        const auto tid = kernel.createVehicle(makeInit(0, 0, 100.0, 100.0, Allegiance::Hostile));     // 100 m above

        for (int step = 0; step < 60; ++step) kernel.step(0.01);  // 0.6 s (ground impact)
        check(detonations >= 1, "impact fuse detonates on ground impact");
        check(!kernel.getStatus().isAlive[id], "warhead vehicle killed by ground impact");
        check(!kernel.getStatus().isAlive[tid], "nearby target killed by the warhead");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
