// Fixed installations: EntityType::RadarSite must behave like the ground
// installation it represents — pinned in place with no velocity or rates —
// while ordinary vehicles in the same kernel keep integrating normally. This
// guards the "ground radar as a datalink/track source" scenarios (e.g. an
// S-400 battery feeding a 40N6 midcourse).
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/ScenarioConfig.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>

#include <array>
#include <cmath>
#include <cstdio>

using namespace StrikeEngine::Kernel;
using namespace StrikeEngine::Models;

namespace {

VehicleInitState makeVehicle(const EcefCoordinate& position)
{
    VehicleInitState init{};
    init.px = position.x;
    init.py = position.y;
    init.pz = position.z;
    init.qw = 1.0;
    init.mass = 100.0;
    return init;
}

VehicleConfig vacuumConfig(EntityType type)
{
    VehicleConfig config;
    config.type = type;
    config.aero.referenceArea = 0.0;
    config.aero.cd = 0.0;
    return config;
}

EnvironmentConfig ecefEnvironment()
{
    EnvironmentConfig environment;
    environment.earth.useEcefTruth = true;
    environment.earth.useSphericalGravity = true;
    environment.earth.referenceLatitudeRad = 0.0;
    environment.earth.referenceLongitudeRad = 0.0;
    return environment;
}

} // namespace

int main()
{
    std::printf("=== radar_site_test: fixed installations do not integrate ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    // A ground site and a free-flying reference vehicle side by side.
    SimulationKernel kernel;
    kernel.setEnvironment(ecefEnvironment());
    kernel.setRandomSeed(11);

    const auto siteGround = geodeticToEcef({0.30, 1.30, 0.0});   // Punjab, sea level
    const auto siteInit = makeVehicle(siteGround);
    kernel.createVehicle(siteInit, vacuumConfig(EntityType::RadarSite));

    const auto flyerGround = geodeticToEcef({0.30, 1.30, 500.0});
    kernel.createVehicle(makeVehicle(flyerGround), vacuumConfig(EntityType::Missile));

    // 5 s: a free body would fall ~123 m.
    kernel.runSteps(500, 0.01);

    const auto& physics = kernel.getPhysics();
    check(physics.size == 2, "both entities are present");

    const double siteDrift = std::sqrt(
        (physics.px[0] - siteGround.x) * (physics.px[0] - siteGround.x) +
        (physics.py[0] - siteGround.y) * (physics.py[0] - siteGround.y) +
        (physics.pz[0] - siteGround.z) * (physics.pz[0] - siteGround.z));
    const double siteSpeed = std::sqrt(
        physics.vx[0] * physics.vx[0] + physics.vy[0] * physics.vy[0] +
        physics.vz[0] * physics.vz[0]);
    check(siteDrift < 1.0e-9, "radar site holds its position exactly");
    check(siteSpeed < 1.0e-9, "radar site has zero velocity");
    check(physics.wx[0] == 0.0 && physics.wy[0] == 0.0 && physics.wz[0] == 0.0,
          "radar site has zero body rates");
    check(physics.ax[0] == 0.0 && physics.ay[0] == 0.0 && physics.az[0] == 0.0,
          "radar site reports zero acceleration");

    const double flyerR0 = std::sqrt(flyerGround.x * flyerGround.x +
                                     flyerGround.y * flyerGround.y +
                                     flyerGround.z * flyerGround.z);
    const double flyerR1 = std::sqrt(physics.px[1] * physics.px[1] +
                                     physics.py[1] * physics.py[1] +
                                     physics.pz[1] * physics.pz[1]);
    check(flyerR0 - flyerR1 > 50.0, "an ordinary vehicle in the same kernel still falls");

    // Pinning must survive continued stepping (no slow creep).
    kernel.runSteps(500, 0.01);
    const double siteDrift2 = std::sqrt(
        (physics.px[0] - siteGround.x) * (physics.px[0] - siteGround.x) +
        (physics.py[0] - siteGround.y) * (physics.py[0] - siteGround.y) +
        (physics.pz[0] - siteGround.z) * (physics.pz[0] - siteGround.z));
    check(siteDrift2 < 1.0e-9, "radar site stays pinned over 10 s");

    // --- Cold launch: vertical clearance, then the thruster pitch-over -----
    // A ground battery launches vertically: the round leaves the canister on
    // the clearance axis, the gas thrusters reorient it to the loft axis
    // before the main motor lights, and the body and the ejection velocity
    // arrive on that axis together. Fins cannot do this at 40 m/s, which is
    // why real vertical launchers carry thrusters -- so the kernel models the
    // reorientation explicitly, at the first stage's ignition delay.
    {
        SimulationKernel launcher;
        launcher.setRandomSeed(0xC01D1CEu);

        // Battery: fixed radar site with a long-range RF seeker so the launch
        // gate (seeker lock hold) opens.
        VehicleConfig siteCfg;
        siteCfg.type = EntityType::RadarSite;
        siteCfg.aero.referenceArea = 0.0;
        siteCfg.aero.cd = 0.0;
        siteCfg.seeker.type = SeekerType::RF;
        siteCfg.seeker.transmitterPowerW = 120000.0;
        siteCfg.seeker.antennaGainDb = 45.0;
        siteCfg.seeker.wavelengthM = 0.09;
        siteCfg.seeker.noiseFloorW = 1.0e-14;
        siteCfg.seeker.snrThresholdDb = 9.0;
        siteCfg.seeker.fieldOfViewHalfAngleRad = 1.0471976;
        siteCfg.seeker.gimbalAzimuthLimitRad = 1.4835299;
        siteCfg.seeker.gimbalElevationLimitRad = 1.4835299;
        siteCfg.seeker.minRangeGateM = 0.0;
        siteCfg.seeker.maxRangeGateM = 400000.0;
        siteCfg.seeker.terrainMaskingEnabled = false;

        VehicleInitState site{};
        site.px = 0.0; site.py = 0.0; site.pz = 0.0;
        site.qw = 1.0; site.mass = 50000.0;
        launcher.createVehicle(site, siteCfg);

        VehicleConfig intruderCfg;
        intruderCfg.type = EntityType::Missile;
        intruderCfg.massDry = 300.0;
        intruderCfg.aero.referenceArea = 0.04;
        intruderCfg.aero.referenceLength = 2.0;
        intruderCfg.aero.cd = 0.35;
        intruderCfg.rcsProfileId =
            std::string(STRIKEENGINE_SOURCE_DIR) + "/data/rcs/target_drone_rcs.json";

        VehicleInitState intruder{};
        intruder.px = 20000.0; intruder.py = 0.0; intruder.pz = 1000.0;
        intruder.vx = 100.0;
        intruder.qw = 1.0; intruder.mass = 300.0;
        intruder.allegiance = Allegiance::Hostile;
        launcher.createVehicle(intruder, intruderCfg);

        ScenarioEntityConfig round;
        round.name = "cold round";
        round.role = "interceptor";
        round.initState.px = 0.0; round.initState.py = 0.0; round.initState.pz = 0.0;
        round.initState.qw = 1.0; round.initState.mass = 100.0;
        round.initState.allegiance = Allegiance::Friendly;
        // PN with a tiny authority: exercises the launch seed without letting
        // the guidance bend the launch kinematics the checks below assert.
        round.initialGuidanceMode = GuidanceMode::ProportionalNavigation;
        round.initialMaxAccel = 1.0;
        round.vehicleConfig.type = EntityType::Missile;
        round.vehicleConfig.massDry = 60.0;
        round.vehicleConfig.Ixx = 1.0;
        round.vehicleConfig.Iyy = 50.0;
        round.vehicleConfig.Izz = 50.0;
        round.vehicleConfig.aero.referenceArea = 0.02;
        round.vehicleConfig.aero.referenceLength = 2.0;
        round.vehicleConfig.aero.cd = 0.3;
        StageConfig stage;
        stage.ignitionDelaySec = 0.5;
        stage.ignitionRampSec = 0.05;
        round.vehicleConfig.propulsion.stages.push_back(stage);
        // Datalink source = the battery's track of entity 1 (the intruder), so
        // the launch seed takes the track path.
        round.vehicleConfig.guidanceAutopilot.datalinkSourceId = 0;
        round.vehicleConfig.guidanceAutopilot.datalinkTargetId = 1;
        round.launch.enabled = true;
        round.launch.parentIndex = 0;
        round.launch.targetIndex = 1;
        round.launch.pushMps = 30.0;
        // Hold long enough for the battery's track to confirm, so the launch
        // seed takes the TRACK path (the one the S-400 shot uses).
        round.launch.lockHoldSec = 0.20;
        // Config fallback aim: a marker, so a wrong seed is unmistakable.
        round.initialTargetX = 1.0;
        round.initialTargetY = 2.0;
        round.initialTargetZ = 3.0;
        round.launch.rangeGateM = 0.0;
        round.launch.ejectElevationDeg = 90.0;   // vertical clearance
        round.launch.launchElevationDeg = 30.0;  // loft axis at ignition
        launcher.addPendingLaunch(round);

        // Body X in the world frame (first column of the body->world matrix).
        auto bodyX = [](const PhysicsBlock& p, std::size_t i) {
            const double qw = p.qw[i], qx = p.qx[i], qy = p.qy[i], qz = p.qz[i];
            return std::array<double, 3>{
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy + qw * qz),
                2.0 * (qx * qz - qw * qy)};
        };

        launcher.runSteps(30, 0.01);  // t = 0.30 s: spawned, still on the clearance axis
        {
            const auto& p = launcher.getPhysics();
            const bool spawned = p.size == 3 && p.active[2];
            check(spawned, "cold-launch round spawns once the battery holds the track");
            if (spawned) {
                const auto x = bodyX(p, 2);
                check(x[2] > 0.999,
                      "ejection holds the 90 deg clearance axis");
                check(p.vz[2] > 25.0 && std::abs(p.vx[2]) < 1.0,
                      "ejection push is straight up");
            }
        }

        // Past the ignition delay (the spawn lands one step in, so give the
        // delay time): the body sits in the launch plane at the loft angle,
        // and the ejection velocity went with it. Gravity trims a couple of
        // degrees off the flight path in the meantime, so the body/velocity
        // alignment is checked as an angle, not as an exact ratio.
        launcher.runSteps(75, 0.01);  // t = 1.05 s: past ignition (spawn+0.5)
        {
            const auto& p = launcher.getPhysics();
            if (p.size == 3 && p.active[2]) {
                const auto x = bodyX(p, 2);
                check(std::abs(x[2] - 0.5) < 0.05 && x[0] > 0.80,
                      "pitch-over puts the body on the 30 deg loft axis");
                const double speed = std::sqrt(p.vx[2] * p.vx[2] +
                                               p.vy[2] * p.vy[2] +
                                               p.vz[2] * p.vz[2]);
                const double align = (speed > 1e-6)
                    ? (x[0] * p.vx[2] + x[1] * p.vy[2] + x[2] * p.vz[2]) / speed
                    : 0.0;
                check(speed > 1.0 && align > 0.99 && p.vx[2] > 15.0,
                      "ejection velocity follows the pitch-over");
                // The INS must have turned with the airframe: a stale estimate
                // is a ~60 deg attitude mismatch the control loops then chase.
                const auto& nav = launcher.getNavigation();
                if (2 < nav.estQw.size()) {
                    const double dot = std::abs(
                        p.qw[2] * nav.estQw[2] + p.qx[2] * nav.estQx[2] +
                        p.qy[2] * nav.estQy[2] + p.qz[2] * nav.estQz[2]);
                    const double errDeg = 2.0 * std::acos(std::clamp(dot, 0.0, 1.0)) *
                        180.0 / 3.14159265358979323846;
                        check(errDeg < 0.5, "navigation attitude follows the pitch-over");
                } else {
                    check(false, "navigation block is sized for the launch case");
                }
                // Launch seed: the aim must be the battery's TRACK of the
                // target (position and velocity), never the config marker and
                // never with an acceleration snapshot -- a remote radar
                // track's accel estimate rails at the filter bound and its APN
                // feed-forward threw the S-400 midcourse off by tens of km.
                const auto& gd = launcher.getGuidance();
                if (2 < gd.targetX.size()) {
                    const double dx = gd.targetX[2] - p.px[1];
                    const double dy = gd.targetY[2] - p.py[1];
                    const double dz = gd.targetZ[2] - p.pz[1];
                    const double dPos = std::sqrt(dx * dx + dy * dy + dz * dz);
                    check(dPos < 1000.0, "spawned round's aim is the battery's target track");
                    check(!gd.targetAccelAvailable[2] &&
                          gd.targetAccelX[2] == 0.0 && gd.targetAccelY[2] == 0.0 &&
                          gd.targetAccelZ[2] == 0.0,
                          "launch seed carries no acceleration snapshot");
                } else {
                    check(false, "guidance block is sized for the launch case");
                }
            } else {
                check(false, "round is still alive through the pitch-over");
            }
        }
    }

    if (failures == 0) {
        std::printf("All Radar Site Tests Passed\n");
        return 0;
    }
    std::printf("Radar Site Tests FAILED (%d)\n", failures);
    return 1;
}
