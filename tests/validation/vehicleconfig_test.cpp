// W1 spine DoD: per-entity vehicle config.
//  - a config without a motor COASTS (no boost, mass constant)
//  - a config with a motor + fuel burns to dryMass and never goes below it
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

int main() {
    std::printf("=== vehicleconfig_test: per-entity config ===\n");

    // ---- Part A: coasting drone (no motor configured) ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0xAB21u);   // deterministic sensor noise
        VehicleInitState init{};
        init.px = 0; init.py = 0; init.pz = 1000.0;
        init.vx = 80.0; init.vy = 0; init.vz = 0;
        init.qw = 1; init.qx = 0; init.qy = 0; init.qz = 0;
        init.wx = 0; init.wy = 0; init.wz = 0;
        init.mass = 100.0;
        init.Ixx = 2.0; init.Iyy = 8.0; init.Izz = 8.0;

        VehicleConfig cfg;   // default: no thrust curve => coasting
        cfg.referenceArea = 0.2;
        cfg.cd = 0.3;
        cfg.clAlpha = 0.5;

        const auto id = kernel.createVehicle(init, cfg);
        auto& phys = kernel.getPhysics();

        constexpr double dt = 0.01;
        for (int step = 0; step < 500; ++step) kernel.step(dt);   // 5 s

        const bool decelerated = phys.vx[id] < 80.0;       // drag acted
        const bool notCrashed  = phys.vx[id] > 40.0;       // no numeric blowup
        const bool massHeld    = phys.mass[id] == 100.0;   // no mass flow without motor

        std::printf("  drone vx=%.2f m/s, mass=%.6f kg\n", phys.vx[id], phys.mass[id]);
        check(decelerated, "coasting drone decelerates under drag (no boost)");
        check(notCrashed,  "coasting drone speed remains sane");
        check(massHeld,    "coasting drone mass unchanged (no motor, no flow)");
    }

    // ---- Part B: fueled missile burns to dry mass, never below ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0xAB22u);   // deterministic sensor noise
        VehicleInitState init{};
        init.px = 0; init.py = 0; init.pz = 5000.0;
        init.vx = 10.0; init.vy = 0; init.vz = 0;
        init.qw = 1; init.qx = 0; init.qy = 0; init.qz = 0;
        init.wx = 0; init.wy = 0; init.wz = 0;
        init.mass = 500.0;
        init.Ixx = 3.0; init.Iyy = 10.0; init.Izz = 10.0;

        VehicleConfig cfg;
        cfg.massDry = 370.0;                 // 130 kg fuel
        cfg.referenceArea = 0.5;
        cfg.cd = 0.15;
        cfg.clAlpha = 2.5;
        cfg.clFin = 2.0;
        cfg.thrustCurve = { {0.0, 50000.0}, {5.0, 50000.0}, {5.1, 0.0}, {100.0, 0.0} };
        cfg.vacuumIsp = 250.0;
        cfg.seaLevelIsp = 220.0;

        const auto id = kernel.createVehicle(init, cfg);

        auto& phys = kernel.getPhysics();
        constexpr double dt = 0.01;

        double minMass = phys.mass[id];
        double maxVx = phys.vx[id];
        for (int step = 0; step < 700; ++step) {   // 7 s (burn ends at 5 s)
            kernel.step(dt);
            minMass = std::min(minMass, phys.mass[id]);
            maxVx = std::max(maxVx, phys.vx[id]);
        }

        const double finalMass = phys.mass[id];
        const bool burnedFuel   = finalMass < 500.0;
        const bool aboveDryMass = finalMass >= 370.0 - 1e-6;
        const bool neverBelow   = minMass >= 370.0 - 1e-6;
        const bool accelerated  = maxVx > 200.0;   // 50 kN on ~400 kg for 5 s
        const bool motorOff     = phys.vx[id] < maxVx;  // decelerating at end (post-burnout drag)

        std::printf("  missile mass: %.2f -> %.2f kg (dry 370), min=%.2f, max vx=%.2f\n",
                    init.mass, finalMass, minMass, maxVx);
        check(burnedFuel,   "fueled missile burns mass during boost");
        check(aboveDryMass, "final mass is at/above dry mass");
        check(neverBelow,   "mass never drops below dry mass");
        check(accelerated,  "motor accelerates the missile (max vx > 200 m/s)");
        check(motorOff,     "after curve end the vehicle coasts (decelerating)");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
