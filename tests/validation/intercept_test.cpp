// W3 spine DoD: control authority makes guided interception physically
// possible. Head-on engagement, stationary target, ProNav + rate-command
// autopilot: miss distance must drop from the pre-spine ~475 m to < 50 m.
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>
#include <strikeengine/kernel/systems/CommandProcessor.hpp>
#include <cstdio>
#include <cmath>

using namespace StrikeEngine::Kernel;

int main() {
    std::printf("=== intercept_test: guided head-on intercept ===\n");

    SimulationKernel kernel;

    // --- Interceptor: boosted missile with fin authority (W1+W3 config) ---
    VehicleInitState missile{};
    missile.px = 0.0; missile.py = 0.0; missile.pz = 1000.0;
    missile.vx = 200.0; missile.vy = 0.0; missile.vz = 0.0;
    // Aerospace attitude: nose forward (+X), belly down (body Z = world -Z)
    missile.qw = 0.0; missile.qx = 1.0; missile.qy = 0.0; missile.qz = 0.0;
    missile.wx = 0.0; missile.wy = 0.0; missile.wz = 0.0;
    missile.mass = 500.0;
    // Realistic inertia: 500 kg, ~3.5 m long, ~0.8 m diameter
    // Iyy ~= m*L^2/12 ~= 510, Ixx ~= m*D^2/8 ~= 40
    missile.Ixx = 40.0; missile.Iyy = 510.0; missile.Izz = 510.0;

    VehicleConfig missileCfg;
    missileCfg.massDry = 370.0;                 // 130 kg fuel
    missileCfg.referenceArea = 0.5;
    missileCfg.referenceLength = 1.0;
    missileCfg.cd = 0.15;
    missileCfg.clAlpha = 2.5;                   // AoA lift
    missileCfg.clFin = 2.0;                     // fin lift (control authority!)
    missileCfg.thrustCurve = { {0.0, 50000.0}, {5.0, 50000.0}, {5.1, 0.0}, {100.0, 0.0} };
    missileCfg.vacuumIsp = 250.0;
    missileCfg.seaLevelIsp = 220.0;

    const auto missileId = kernel.createVehicle(missile, missileCfg);

    // --- Target: coasting drone (no motor — W1) ---
    VehicleInitState target{};
    target.px = 4000.0; target.py = 0.0; target.pz = 1.0;
    target.vx = 0.0; target.vy = 0.0; target.vz = 0.0;
    target.qw = 1.0; target.qx = 0.0; target.qy = 0.0; target.qz = 0.0;
    target.mass = 100.0;
    // 100 kg, 2 m long, 0.4 m diameter drone: Iyy ~= 33, Ixx ~= 2
    target.Ixx = 2.0; target.Iyy = 33.0; target.Izz = 33.0;

    VehicleConfig targetCfg;   // coasting
    targetCfg.referenceArea = 0.2;
    targetCfg.cd = 0.3;
    targetCfg.clAlpha = 0.5;

    const auto targetId = kernel.createVehicle(target, targetCfg);

    // --- ProNav on the stationary target ---
    SimulationCommand cmd{};
    cmd.entityId = missileId;
    cmd.mode = GuidanceMode::ProportionalNavigation;
    cmd.targetX = target.px; cmd.targetY = target.py; cmd.targetZ = target.pz;
    kernel.queueCommand(cmd);

    auto& phys = kernel.getPhysics();
    constexpr double dt = 0.01;
    constexpr int maxSteps = 6000;   // 60 s

    double minMiss = 1e18;
    double minMissTime = 0.0;
    int impactStep = -1;
    double maxSpeed = 0.0;

    for (int step = 0; step < maxSteps; ++step) {
        kernel.step(dt);

        const double dx = phys.px[missileId] - phys.px[targetId];
        const double dy = phys.py[missileId] - phys.py[targetId];
        const double dz = phys.pz[missileId] - phys.pz[targetId];
        const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        const double vx = phys.vx[missileId], vy = phys.vy[missileId], vz = phys.vz[missileId];
        maxSpeed = std::max(maxSpeed, std::sqrt(vx*vx + vy*vy + vz*vz));

        if (dist < minMiss) { minMiss = dist; minMissTime = step * dt; }

        // Impact = missile crosses below z=0 (ground) or passes the target plane
        if (impactStep < 0 && (phys.pz[missileId] <= 0.0 || phys.px[missileId] > target.px + 50.0)) {
            impactStep = step;
        }
    }

    const bool minMissOk = minMiss < 50.0;
    const bool saneSpeed = maxSpeed < 1500.0;   // no numeric blowup during boost+coast

    std::printf("  min miss: %.2f m at t=%.2f s\n", minMiss, minMissTime);
    std::printf("  missile max speed: %.1f m/s\n", maxSpeed);
    std::printf("  missile final: p=(%.0f, %.0f, %.0f) v=(%.1f, %.1f, %.1f)\n",
                phys.px[missileId], phys.py[missileId], phys.pz[missileId],
                phys.vx[missileId], phys.vy[missileId], phys.vz[missileId]);

    if (minMissOk && saneSpeed) {
        std::printf("ALL PASS (guided intercept < 50 m)\n");
        return 0;
    }
    std::printf("FAILED (min miss %.2f m, max speed %.0f m/s)\n", minMiss, maxSpeed);
    return 1;
}
