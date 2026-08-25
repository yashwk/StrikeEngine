// W2 spine DoD: 6-DOF rigid-body truth model.
//  - gyroscopic coupling conserves rotational energy (torque-free case)
//  - quaternion stays unit-normalized throughout
//  - pitch sign convention: +finPitch deflection => nose-UP rotation
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>
#include <strikeengine/kernel/systems/CommandProcessor.hpp>
#include <cstdio>
#include <cmath>

using namespace StrikeEngine::Kernel;

static int failures = 0;
static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

int main() {
    std::printf("=== rigidbody_test: 6-DOF truth model ===\n");

    // ---- Part A: torque-free precession conserves rotational energy ----
    {
        SimulationKernel kernel;
        VehicleInitState init{};
        init.px = 0; init.py = 0; init.pz = 100000.0;  // near-vacuum: no aero
        init.vx = 0; init.vy = 0; init.vz = 0;
        init.qw = 1; init.qx = 0; init.qy = 0; init.qz = 0;
        init.wx = 10.0; init.wy = 2.0; init.wz = 3.0;   // off-axis spin (Iyy != Izz)
        init.mass = 100.0;
        init.Ixx = 3.0; init.Iyy = 10.0; init.Izz = 12.0;

        VehicleConfig cfg;   // no motor
        cfg.cd = 0.0;        // no drag; lift cannot matter at zero velocity

        const auto id = kernel.createVehicle(init, cfg);

        auto& phys = kernel.getPhysics();
        const double E0 = 0.5 * (init.Ixx * init.wx * init.wx +
                                 init.Iyy * init.wy * init.wy +
                                 init.Izz * init.wz * init.wz);

        double maxNormErr = 0.0;
        constexpr double dt = 0.01;
        for (int step = 0; step < 1000; ++step) {   // 10 s
            kernel.step(dt);
            const double qn = std::sqrt(phys.qw[id]*phys.qw[id] + phys.qx[id]*phys.qx[id] +
                                        phys.qy[id]*phys.qy[id] + phys.qz[id]*phys.qz[id]);
            maxNormErr = std::max(maxNormErr, std::abs(qn - 1.0));
        }

        const double E1 = 0.5 * (phys.Ixx[id] * phys.wx[id] * phys.wx[id] +
                                 phys.Iyy[id] * phys.wy[id] * phys.wy[id] +
                                 phys.Izz[id] * phys.wz[id] * phys.wz[id]);
        const double relEnergyErr = std::abs(E1 - E0) / E0;
        std::printf("  rotational energy drift: %.3e (%.3e -> %.3e)\n", relEnergyErr, E0, E1);
        std::printf("  quaternion norm error max: %.3e\n", maxNormErr);

        check(relEnergyErr < 5e-3,
              "torque-free precession conserves rotational energy (<0.5%)");
        check(maxNormErr < 1e-9,
              "quaternion stays unit-normalized (<1e-9)");

        // Gyroscopic coupling must actually engage: with Iyy != Izz and an
        // off-axis spin, wx cannot stay constant (w x (I w) term).
        const bool coupled = std::abs(phys.wx[id] - init.wx) > 0.1;
        check(coupled, "gyroscopic coupling active (wx drifts from initial value)");
    }

    // ---- Part B: pitch sign convention through the full chain ----
    // Waypoint directly ABOVE a belly-down missile => guidance commands a
    // world +Z (up) acceleration => the autopilot must pitch the nose up.
    // Verifies guidance->autopilot->aero sign conventions end to end.
    {
        SimulationKernel kernel;
        VehicleInitState init{};
        init.px = 0; init.py = 0; init.pz = 5000.0;
        init.vx = 100.0; init.vy = 0; init.vz = 0;
        // Aerospace initial attitude: nose +X, belly down (body Z = world -Z)
        init.qw = 0.0; init.qx = 1.0; init.qy = 0.0; init.qz = 0.0;
        init.wx = 0; init.wy = 0; init.wz = 0;
        init.mass = 100.0;
        init.Ixx = 3.0; init.Iyy = 10.0; init.Izz = 10.0;

        VehicleConfig cfg;   // no motor, fins can pitch
        cfg.referenceArea = 0.3;   // realistic fin authority for a 100 kg airframe
        cfg.clFin = 2.5;           // fin lift coefficient
        cfg.clAlpha = 2.5;         // AoA lift
        cfg.cd = 0.3;

        const auto id = kernel.createVehicle(init, cfg);

        // Waypoint straight above the missile: commanded accel = (0,0,+5) world
        SimulationCommand cmd{};
        cmd.entityId = id;
        cmd.mode = GuidanceMode::Waypoint;
        cmd.targetX = 0.0; cmd.targetY = 0.0; cmd.targetZ = 5500.0;
        kernel.queueCommand(cmd);

        constexpr double dt = 0.01;
        for (int step = 0; step < 200; ++step) kernel.step(dt);   // 2 s

        auto& phys = kernel.getPhysics();
        const bool noseUpRate = phys.wy[id] > 0.0;   // +wy = nose-up body pitch rate
        const bool gained = phys.pz[id] > 5000.0;

        std::printf("  wy=%.4f rad/s, altitude change=%.2f m\n",
                    phys.wy[id], phys.pz[id] - 5000.0);
        check(noseUpRate, "climb command produces positive (nose-up) body pitch rate");
        check(gained, "nose-up rotation results in a climb (world +Z)");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
