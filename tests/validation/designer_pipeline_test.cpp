// Designer -> engine pipeline: the designer-manifest artifacts under
// data/profiles and data/scenarios are consumed exactly as the engine schema
// requires, and the whole chain is proven end-to-end:
//   design_ref -> loadDesignPhysics -> subsystem profile ids -> kernel wiring
//   -> RF seeker + RCS signature -> guidance -> < 50 m intercept.
#include <strikeengine/kernel/config/ScenarioConfig.hpp>
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/data/SeekerBlock.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

using namespace StrikeEngine::Kernel;

int main()
{
    std::printf("=== designer_pipeline_test: design files -> engine -> intercept ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    // Relative paths inside the data files (design_ref, profile ids, rcs) are
    // resolved against the repo root at runtime.
    std::filesystem::current_path(STRIKEENGINE_SOURCE_DIR);
    const std::string scenarioPath =
        std::string(STRIKEENGINE_SOURCE_DIR) + "/data/scenarios/intercept_test_01.json";

    // ---- 1. The designer scenario loads as an engine ScenarioConfig ----
    ScenarioConfig scenario;
    try {
        scenario = ScenarioConfig::load(scenarioPath);
    } catch (const std::exception& e) {
        std::printf("  [FAIL] scenario load threw: %s\n", e.what());
        return 1;
    }
    check(scenario.entities.size() == 2, "scenario has two entities");
    check(scenario.entities[0].initState.allegiance == Allegiance::Friendly &&
              scenario.entities[1].initState.allegiance == Allegiance::Hostile,
          "missile and drone have opposing allegiances (seeker engages hostiles)");
    check(scenario.entities[0].initialGuidanceMode == GuidanceMode::ProportionalNavigation,
          "missile guidance mode came from the scenario (ProNav)");
    // The scenario was re-baselined to 15 km downrange / 8 km up with the
    // drone descending at vz = -60 m/s (aero tables make the missile faster
    // and lower-drag, so the old 20 km / 10 km geometry no longer fits the
    // seeker's ~3 km acquisition range). Lock the new geometry here.
    check(scenario.entities[1].initState.px == 15000.0 &&
              scenario.entities[1].initState.py == 0.0 &&
              scenario.entities[1].initState.pz == 8000.0 &&
              scenario.entities[1].initState.vx == -250.0 &&
              scenario.entities[1].initState.vz == -60.0,
          "drone re-baselined to 15 km downrange / 8 km up, closing at 250 m/s with vz=-60");
    check(scenario.entities[0].initialTargetX == 15000.0 &&
              scenario.entities[0].initialTargetZ == 8000.0 &&
              scenario.entities[0].initialTargetVx == -250.0 &&
              scenario.entities[0].initialTargetVz == -60.0,
          "missile guidance aim matches the re-baselined drone geometry");

    // ---- 2. design_ref resolved into VehicleConfig with profile ids ----
    const VehicleConfig& missileCfg = scenario.entities[0].vehicleConfig;
    check(missileCfg.aeroProfileId == "data/aero/sa_missile_mk1_aero.json" &&
              missileCfg.motorProfileId == "data/motors/sa_missile_mk1_motor.json" &&
              missileCfg.seekerProfileId == "data/seekers/aesa_tracker_v1.json" &&
              missileCfg.sensorProfileId == "data/sensors/sa_missile_mk1_imu.json",
          "missile design file resolved all four subsystem profile ids");
    check(missileCfg.type == EntityType::Missile &&
              std::abs(missileCfg.initialMass - 150.0) < 1e-9 &&
              std::abs(missileCfg.massDry - 50.0) < 1e-9,
          "missile design physics (type + masses) parsed");
    check(std::abs(missileCfg.Ixx - 0.5) < 1e-9 &&
              std::abs(missileCfg.Iyy - 10.0) < 1e-9 &&
              std::abs(missileCfg.Izz - 10.0) < 1e-9,
          "missile inertia: slender-body roll axis -> inertia_xx");
    check(missileCfg.guidanceAutopilot.navigationConstant == 4.0,
          "guidance_autopilot navigationConstant = 4.0 from the designer");
    check(missileCfg.warhead.lethalRadiusM == 50.0 && missileCfg.warhead.massKg == 15.0 &&
              missileCfg.warhead.fusing == FusingType::Proximity &&
              missileCfg.warhead.falloffRadiusM == 90.0,
          "SAM warhead parsed (15 kg, proximity-fused, 50 m kill radius, 90 m falloff)");

    const VehicleConfig& droneCfg = scenario.entities[1].vehicleConfig;
    check(droneCfg.type == EntityType::Aircraft &&
              std::abs(droneCfg.initialMass - 500.0) < 1e-9 &&
              droneCfg.propulsion.stages.empty(),
          "drone design physics: coasting aircraft with no propulsion");
    check(droneCfg.rcsProfileId == "data/rcs/target_drone_rcs.json",
          "drone carries the flat RCS profile id");

    // ---- 3. Profile ids actually take effect in the kernel blocks ----
    SimulationKernel kernel;
    kernel.setRandomSeed(0xDEADBEEFu);
    scenario.loadInto(kernel);
    const auto& phys = kernel.getPhysics();
    const auto& sk = kernel.getSeekers();
    const auto& sn = kernel.getSensors();
    const auto& status = kernel.getStatus();
    check(std::abs(phys.referenceArea[0] - 0.04) < 1e-12 &&
              std::abs(phys.cd[0] - 0.45) < 1e-12,
          "aero profile replaced the inline aero config (ref area 0.04, cd 0.45)");
    check(phys.aeroTables[0] != nullptr &&
              phys.aeroTables[0]->machBreakpoints.size() == 6 &&
              phys.aeroTables[0]->aoaBreakpointsRad.size() == 4 &&
              phys.aeroTables[0]->clTable.size() == 6 &&
              phys.aeroTables[0]->clTable[0].size() == 4 &&
              phys.aeroTables[0]->cdTable.size() == 6 &&
              phys.aeroTables[0]->cdTable[0].size() == 4,
          "aero profile cd(M,a)/cl(M,a) tables reached the physics block (6 mach x 4 aoa)");
    check(phys.stageCount[0] == 2 && phys.stageIndex[0] == 0,
          "motor profile registered booster + sustainer stages");
    check(sk.type[0] == SeekerType::RF &&
              std::abs(sk.transmitterPowerW[0] - 1200.0) < 1e-9 &&
              std::abs(sk.antennaGainDb[0] - 32.0) < 1e-9,
          "seeker profile wired an RF seeker (not the None default)");
    // The design manifest's inline seeker placeholder also says "rf"/1200 W/32 dB
    // (type/power/gain cannot tell inline from profile), but its FOV is 60 deg
    // and gimbal limits 60 deg. The aesa_tracker_v1 profile narrows FOV to 6 deg
    // and widens gimbal to 65 deg, so these values prove the profile — not the
    // inline placeholder — reached the kernel block.
    check(std::abs(sk.fieldOfViewHalfAngleRad[0] - 0.10471975511965977) < 1e-12 &&
              std::abs(sk.gimbalAzimuthLimitRad[0] - 1.1344640137963142) < 1e-12 &&
              std::abs(sk.gimbalElevationLimitRad[0] - 1.1344640137963142) < 1e-12,
          "seeker profile FOV/gimbal (6 deg / 65 deg) replaced the inline 60 deg placeholder");
    check(std::abs(sn.accelNoiseStdDev[0] - 0.00980665) < 1e-12 && sn.imuEnabled[0],
          "sensor profile wired the IMU error model");
    check(kernel.getGuidance().navigationConstant[0] == 4.0,
          "navigation_constant 4.0 reached the guidance block");
    check(status.rcsProfileId[1] == "data/rcs/target_drone_rcs.json",
          "drone RCS profile id reached the entity status block");
    check(status.allegiance[0] == Allegiance::Friendly &&
              status.allegiance[1] == Allegiance::Hostile,
          "opposing allegiances reached the status block");

    // ---- 4. Run the intercept ----
    constexpr double dt = 0.01;
    constexpr int maxSteps = 6000;   // 60 s
    double minMiss = 1e18;
    double minMissTime = 0.0;
    bool seekerLockedDrone = false;
    double seekerLockRange = 0.0;
    double maxSpeed = 0.0;
    double droneKillTime = -1.0;
    bool warheadDetonated = false;
    kernel.getEventSystem().subscribe([&](const SimulationEvent& evt) {
        if (evt.type == EventType::Detonation) warheadDetonated = true;
        if (evt.type == EventType::StructuralFailure && evt.entityId == 1 && droneKillTime < 0.0) {
            droneKillTime = evt.timestamp;
        }
    });

    for (int step = 0; step < maxSteps; ++step) {
        kernel.step(dt);
        const auto& p = kernel.getPhysics();
        const auto& s = kernel.getSeekers();
        const double dx = p.px[0] - p.px[1];
        const double dy = p.py[0] - p.py[1];
        const double dz = p.pz[0] - p.pz[1];
        const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        const double vx = p.vx[0], vy = p.vy[0], vz = p.vz[0];
        maxSpeed = std::max(maxSpeed, std::sqrt(vx*vx + vy*vy + vz*vz));
        if (dist < minMiss) { minMiss = dist; minMissTime = step * dt; }
        if (!seekerLockedDrone && s.isLocked[0] && s.lockedTargetId[0] == 1) {
            seekerLockedDrone = true;
            seekerLockRange = s.targetRange[0];
        }
    }

    const bool minMissOk = minMiss < 50.0;
    check(minMissOk, "missile intercepts the drone (closest approach < 50 m)");
    std::printf("    min miss: %.2f m at t=%.2f s\n", minMiss, minMissTime);
    std::printf("    missile max speed: %.1f m/s\n", maxSpeed);
    std::printf("    RF seeker locked the drone at range %.1f m\n", seekerLockRange);
    check(seekerLockedDrone,
          "RF seeker (from seeker profile) acquired and locked the drone");
    check(warheadDetonated && droneKillTime > 0.0 &&
              std::abs(droneKillTime - minMissTime) < 1.0,
          "proximity warhead detonated at closest approach and killed the drone");
    check(maxSpeed < 3000.0 && maxSpeed > 500.0,
          "missile speed stays physical (boosted, no numeric blowup)");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
