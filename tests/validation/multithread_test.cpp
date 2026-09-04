#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>
#include <cstdio>
#include <cmath>
#include <vector>

using namespace StrikeEngine::Kernel;

static int failures = 0;
static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

int main() {
    std::printf("=== multithread_test: CPUBackend parallel worker pool ===\n");

    // 1. Thread count configuration and query
    {
        SimulationKernel kernel;
        check(kernel.threadCount() == 1, "default thread count is 1");

        kernel.setThreadCount(4);
        check(kernel.threadCount() == 4, "setThreadCount(4) updates thread count to 4");

        kernel.setThreadCount(1);
        check(kernel.threadCount() == 1, "setThreadCount(1) restores thread count to 1");

        kernel.setThreadCount(8);
        check(kernel.threadCount() == 8, "setThreadCount(8) resizes pool to 8 workers");
    }

    // 2. Trajectory determinism: 1 thread vs 4 threads across 32 entities
    {
        constexpr std::size_t kNumEntities = 32;
        constexpr std::size_t kNumSteps = 100;
        constexpr double kDt = 0.01;

        struct Snapshot {
            std::vector<double> px, py, pz;
            std::vector<double> vx, vy, vz;
            std::vector<double> qw, qx, qy, qz;
        };

        auto runSimulation = [](std::size_t threads) -> Snapshot {
            SimulationKernel kernel;
            kernel.setThreadCount(threads);
            kernel.setRandomSeed(0x12345678u);

            std::vector<PhysicsId> ids;
            ids.reserve(kNumEntities);

            for (std::size_t i = 0; i < kNumEntities; ++i) {
                VehicleInitState init{};
                init.px = static_cast<double>(i) * 100.0;
                init.py = static_cast<double>(i) * -50.0;
                init.pz = 5000.0 + static_cast<double>(i) * 20.0;
                init.vx = 200.0 + static_cast<double>(i) * 5.0;
                init.vy = 10.0 * std::sin(static_cast<double>(i));
                init.vz = -5.0 * std::cos(static_cast<double>(i));
                init.qw = 1.0; init.qx = 0.0; init.qy = 0.0; init.qz = 0.0;
                init.wx = 0.05 * static_cast<double>(i);
                init.wy = 0.02 * static_cast<double>(i);
                init.wz = 0.01 * static_cast<double>(i);
                init.mass = 120.0 + static_cast<double>(i);

                VehicleConfig cfg{};
                cfg.Ixx = 2.0; cfg.Iyy = 15.0; cfg.Izz = 15.0;
                cfg.aero.cd = 0.35;
                cfg.aero.referenceArea = 0.08;
                cfg.aero.referenceLength = 2.5;

                ids.push_back(kernel.createVehicle(init, cfg));
            }

            for (std::size_t s = 0; s < kNumSteps; ++s) {
                kernel.step(kDt);
            }

            const auto& phys = kernel.getPhysics();
            Snapshot snap;
            snap.px = phys.px; snap.py = phys.py; snap.pz = phys.pz;
            snap.vx = phys.vx; snap.vy = phys.vy; snap.vz = phys.vz;
            snap.qw = phys.qw; snap.qx = phys.qx; snap.qy = phys.qy; snap.qz = phys.qz;
            return snap;
        };

        const Snapshot snap1 = runSimulation(1);
        const Snapshot snap4 = runSimulation(4);

        double maxPosDiff = 0.0;
        double maxVelDiff = 0.0;
        double maxQuatDiff = 0.0;

        for (std::size_t i = 0; i < kNumEntities; ++i) {
            const double dpx = std::abs(snap1.px[i] - snap4.px[i]);
            const double dpy = std::abs(snap1.py[i] - snap4.py[i]);
            const double dpz = std::abs(snap1.pz[i] - snap4.pz[i]);
            maxPosDiff = std::max(maxPosDiff, std::max({dpx, dpy, dpz}));

            const double dvx = std::abs(snap1.vx[i] - snap4.vx[i]);
            const double dvy = std::abs(snap1.vy[i] - snap4.vy[i]);
            const double dvz = std::abs(snap1.vz[i] - snap4.vz[i]);
            maxVelDiff = std::max(maxVelDiff, std::max({dvx, dvy, dvz}));

            const double dqw = std::abs(snap1.qw[i] - snap4.qw[i]);
            const double dqx = std::abs(snap1.qx[i] - snap4.qx[i]);
            const double dqy = std::abs(snap1.qy[i] - snap4.qy[i]);
            const double dqz = std::abs(snap1.qz[i] - snap4.qz[i]);
            maxQuatDiff = std::max(maxQuatDiff, std::max({dqw, dqx, dqy, dqz}));
        }

        std::printf("  multithread max pos diff:  %.3e m\n", maxPosDiff);
        std::printf("  multithread max vel diff:  %.3e m/s\n", maxVelDiff);
        std::printf("  multithread max quat diff: %.3e\n", maxQuatDiff);

        check(maxPosDiff < 1e-9, "positions between 1-thread and 4-thread runs are bit-identical");
        check(maxVelDiff < 1e-9, "velocities between 1-thread and 4-thread runs are bit-identical");
        check(maxQuatDiff < 1e-9, "attitudes between 1-thread and 4-thread runs are bit-identical");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
