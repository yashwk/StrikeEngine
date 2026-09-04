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
        kernel.setRandomSeed(0xAB12u);   // deterministic sensor noise
        VehicleInitState init{};
        init.px = 0; init.py = 0; init.pz = 100000.0;  // near-vacuum: no aero
        init.vx = 0; init.vy = 0; init.vz = 0;
        init.qw = 1; init.qx = 0; init.qy = 0; init.qz = 0;
        init.wx = 10.0; init.wy = 2.0; init.wz = 3.0;   // off-axis spin (Iyy != Izz)
        init.mass = 100.0;

        VehicleConfig cfg;   // no motor
        cfg.Ixx = 3.0; cfg.Iyy = 10.0; cfg.Izz = 12.0;
        cfg.aero.cd = 0.0;   // no drag; lift cannot matter at zero velocity

        const auto id = kernel.createVehicle(init, cfg);

        auto& phys = kernel.getPhysics();
        const double E0 = 0.5 * (cfg.Ixx * init.wx * init.wx +
                                 cfg.Iyy * init.wy * init.wy +
                                 cfg.Izz * init.wz * init.wz);

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
        kernel.setRandomSeed(0xAB13u);   // deterministic sensor noise
        VehicleInitState init{};
        // Fly the sign test in DENSE AIR: the guidance demand is 2 g (k=20),
        // and at 5000 m (rho~0.74) this 100 kg / S=0.8 m2 airframe can only
        // produce ~1 g - it would hold altitude at best and the "climb"
        // assertion would be unphysical. At 100 m (rho~1.21) the same demand
        // is achievable with margin.
        init.px = 0; init.py = 0; init.pz = 100.0;
        init.vx = 100.0; init.vy = 0; init.vz = 0;
        // Aerospace initial attitude: nose +X, belly down (body Z = world -Z)
        init.qw = 0.0; init.qx = 1.0; init.qy = 0.0; init.qz = 0.0;
        init.wx = 0; init.wy = 0; init.wz = 0;
        init.mass = 100.0;

        VehicleConfig cfg;   // no motor, fins can pitch
        cfg.Ixx = 3.0; cfg.Iyy = 33.0; cfg.Izz = 33.0;   // ~2 m airframe
        cfg.aero.referenceArea = 0.8;   // lifting area for a 100 kg sign-test airframe
        cfg.aero.clFin = 1.0;           // fin lift coefficient (fins are small surfaces:
                                        // force authority well below the body lift)
        cfg.aero.clAlpha = 4.0;         // body+tail AoA lift slope
        cfg.aero.cd = 0.3;

        const auto id = kernel.createVehicle(init, cfg);

        // Waypoint straight above the missile: commanded accel = (0,0,+k) world.
        // Demand shaped to 0.5 g via the guidance limiter so the deflection
        // stays in the linear regime (a 2 g demand saturates the fins and
        // bang-bangs against the servo rate limit -- not a sign test).
        SimulationCommand cmd{};
        cmd.entityId = id;
        cmd.mode = GuidanceMode::Waypoint;
        cmd.targetX = 0.0; cmd.targetY = 0.0; cmd.targetZ = 600.0;
        cmd.maxAccel = 5.0;   // guidance demand limit (m/s^2)
        kernel.queueCommand(cmd);

        auto& phys = kernel.getPhysics();
        constexpr double dt = 0.01;
        double maxWy = 0.0;
        for (int step = 0; step < 200; ++step) {
            kernel.step(dt);
            maxWy = std::max(maxWy, phys.wy[id]);   // +wy = nose-up body pitch rate
        }

        const bool noseUpRate = maxWy > 0.3;   // rotated nose-up during the transient
        const bool gained = phys.pz[id] > 100.0;

        std::printf("  max wy=%.4f rad/s, altitude change=%.2f m\n",
                    maxWy, phys.pz[id] - 100.0);
        check(noseUpRate, "climb command produces positive (nose-up) body pitch rate");
        check(gained, "nose-up rotation results in a climb (world +Z)");
    }

    // ---- Part C: Full 3x3 inertia tensor cross-coupling & energy conservation ----
    // When products of inertia are non-zero (e.g. Ixz != 0), a multi-axis spin
    // precesses with coupled Euler dynamics, and torque-free rotational
    // kinetic energy E = 0.5 * w^T * I * w is conserved.
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0x5678u);
        VehicleInitState init{};
        init.px = 0; init.py = 0; init.pz = 100000.0; // vacuum
        init.vx = 0; init.vy = 0; init.vz = 0;
        init.qw = 1; init.qx = 0; init.qy = 0; init.qz = 0;
        init.wx = 4.0; init.wy = 3.0; init.wz = 2.0; // multi-axis spin
        init.mass = 50.0;

        VehicleConfig cfg;
        cfg.Ixx = 8.0; cfg.Iyy = 15.0; cfg.Izz = 20.0;
        cfg.Ixy = 0.5; cfg.Ixz = 1.2;  cfg.Iyz = 0.8; // full 3x3 tensor
        cfg.aero.cd = 0.0;

        const auto id = kernel.createVehicle(init, cfg);
        auto& phys = kernel.getPhysics();

        auto computeRotEnergy = [](double wx, double wy, double wz,
                                   double Ixx, double Iyy, double Izz,
                                   double Ixy, double Ixz, double Iyz) {
            return 0.5 * (Ixx * wx * wx + Iyy * wy * wy + Izz * wz * wz
                          - 2.0 * Ixy * wx * wy
                          - 2.0 * Ixz * wx * wz
                          - 2.0 * Iyz * wy * wz);
        };

        const double E0 = computeRotEnergy(init.wx, init.wy, init.wz,
                                           cfg.Ixx, cfg.Iyy, cfg.Izz,
                                           cfg.Ixy, cfg.Ixz, cfg.Iyz);

        double maxNormErr = 0.0;
        constexpr double dt = 0.005;
        for (int step = 0; step < 2000; ++step) { // 10 s
            kernel.step(dt);
            const double qn = std::sqrt(phys.qw[id]*phys.qw[id] + phys.qx[id]*phys.qx[id] +
                                        phys.qy[id]*phys.qy[id] + phys.qz[id]*phys.qz[id]);
            maxNormErr = std::max(maxNormErr, std::abs(qn - 1.0));
        }

        const double E1 = computeRotEnergy(phys.wx[id], phys.wy[id], phys.wz[id],
                                           phys.Ixx[id], phys.Iyy[id], phys.Izz[id],
                                           phys.Ixy[id], phys.Ixz[id], phys.Iyz[id]);

        const double relEnergyErr = std::abs(E1 - E0) / E0;
        std::printf("  Part C: full 3x3 rot energy drift: %.3e (%.3e -> %.3e)\n", relEnergyErr, E0, E1);
        std::printf("  Part C: full 3x3 quat norm error: %.3e\n", maxNormErr);

        check(relEnergyErr < 5e-3,
              "full 3x3 inertia tensor conserves rotational energy (<0.5%)");
        check(maxNormErr < 1e-9,
              "full 3x3 quaternion stays unit-normalized (<1e-9)");
        check(phys.Ixy[id] == 0.5 && phys.Ixz[id] == 1.2 && phys.Iyz[id] == 0.8,
              "products of inertia properly preserved in PhysicsBlock");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
