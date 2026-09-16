// seeker_fidelity_test: measurement noise/glint/Swerling, gimbal servo,
// range gates, terrain masking, decoy rejection, emitter duty, lock
// publication consistency, and config round-trips. Every capability is
// opt-in; the legacy path (all flags off) is exercised by seeker_test.
#include <strikeengine/kernel/systems/SeekerSystem.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <strikeengine/kernel/config/ConfigSerialization.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <cstdio>
#include <string>

using namespace StrikeEngine::Kernel;

namespace {

constexpr double pi = 3.14159265358979323846;
int g_failures = 0;
void check(bool condition, const char* message)
{
    if (condition) std::printf("  [PASS] %s\n", message);
    else { std::printf("  [FAIL] %s\n", message); ++g_failures; }
}

const std::string kRcs = std::string(STRIKEENGINE_SOURCE_DIR) + "/tests/data/flat_rcs.json";

void makeBlocks(PhysicsBlock& physics, EntityStatusBlock& status, SeekerBlock& seeker,
                int nEntities = 2)
{
    physics.size = static_cast<std::size_t>(nEntities);
    physics.px.assign(nEntities, 0.0);
    physics.py.assign(nEntities, 0.0);
    physics.pz.assign(nEntities, 1000.0);
    physics.vx.assign(nEntities, 0.0);
    physics.vy.assign(nEntities, 0.0);
    physics.vz.assign(nEntities, 0.0);
    physics.qw.assign(nEntities, 1.0);
    physics.qx.assign(nEntities, 0.0);
    physics.qy.assign(nEntities, 0.0);
    physics.qz.assign(nEntities, 0.0);
    physics.active.assign(nEntities, true);
    physics.px[1] = 100.0;

    status.size = physics.size;
    status.type.assign(nEntities, EntityType::Aircraft);
    status.allegiance.assign(nEntities, Allegiance::Hostile);
    status.allegiance[0] = Allegiance::Friendly;
    status.isAlive.assign(nEntities, true);
    status.rcsProfileId.assign(nEntities, kRcs);
    status.irProfileId.assign(nEntities, "");
    status.emitterEirpW.assign(nEntities, 0.0);

    seeker.size = physics.size;
    seeker.type.assign(nEntities, SeekerType::None);
    seeker.type[0] = SeekerType::RF;
    seeker.transmitterPowerW.assign(nEntities, 0.0);
    seeker.transmitterPowerW[0] = 1.0e6;
    seeker.antennaGainDb.assign(nEntities, 0.0);
    seeker.antennaGainDb[0] = 30.0;
    seeker.wavelengthM.assign(nEntities, 0.0);
    seeker.wavelengthM[0] = 0.03;
    seeker.noiseFloorW.assign(nEntities, 1.0);
    seeker.noiseFloorW[0] = 1.0e-12;
    seeker.snrThresholdDb.assign(nEntities, 0.0);
    seeker.snrThresholdDb[0] = 13.0;
    seeker.sensitivityW.assign(nEntities, 1.0e-9);
    seeker.wavelengthBand.assign(nEntities, 0);
    seeker.irExtinctionPerM.assign(nEntities, 1e-4);
    seeker.illuminatorPx.assign(nEntities, 0.0);
    seeker.illuminatorPy.assign(nEntities, 0.0);
    seeker.illuminatorPz.assign(nEntities, 0.0);
    seeker.illuminatorPowerW.assign(nEntities, 5.0e5);
    seeker.illuminatorGainDb.assign(nEntities, 38.0);
    seeker.illuminatorWavelengthM.assign(nEntities, 0.03);
    seeker.fieldOfViewHalfAngleRad.assign(nEntities, pi / 4.0);
    seeker.gimbalAzimuthLimitRad.assign(nEntities, pi / 3.0);
    seeker.gimbalElevationLimitRad.assign(nEntities, pi / 3.0);
    seeker.lockHysteresisDb.assign(nEntities, 3.0);
    seeker.lockDropoutTimeSec.assign(nEntities, 0.10);
    seeker.measurementLatencySec.assign(nEntities, 0.0);
    seeker.isLocked.assign(nEntities, false);
    seeker.lockedTargetId.assign(nEntities, 0);
    seeker.targetRange.assign(nEntities, 0.0);
    seeker.targetRangeRate.assign(nEntities, 0.0);
    seeker.targetAzimuth.assign(nEntities, 0.0);
    seeker.targetElevation.assign(nEntities, 0.0);
    seeker.targetAzimuthRate.assign(nEntities, 0.0);
    seeker.targetElevationRate.assign(nEntities, 0.0);
    seeker.previousAzimuth.assign(nEntities, 0.0);
    seeker.previousElevation.assign(nEntities, 0.0);
    seeker.timeSinceCommitSec.assign(nEntities, 0.0);
    seeker.seekerClockSec.assign(nEntities, 0.0);
    seeker.losRateWorldX.assign(nEntities, 0.0);
    seeker.losRateWorldY.assign(nEntities, 0.0);
    seeker.losRateWorldZ.assign(nEntities, 0.0);
    seeker.losRateWorldValid.assign(nEntities, false);
    seeker.losRateWorldStateX.assign(nEntities, 0.0);
    seeker.losRateWorldStateY.assign(nEntities, 0.0);
    seeker.losRateWorldStateZ.assign(nEntities, 0.0);
    seeker.prevLosWorldX.assign(nEntities, 0.0);
    seeker.prevLosWorldY.assign(nEntities, 0.0);
    seeker.prevLosWorldZ.assign(nEntities, 0.0);
    seeker.prevLosWorldTimeSec.assign(nEntities, 0.0);
    seeker.losRateFilterAz.assign(nEntities, 0.0);
    seeker.losRateFilterEl.assign(nEntities, 0.0);
    seeker.bodyRateFilteredX.assign(nEntities, 0.0);
    seeker.bodyRateFilteredY.assign(nEntities, 0.0);
    seeker.bodyRateFilteredZ.assign(nEntities, 0.0);
    seeker.bodyRateStateX.assign(nEntities, 0.0);
    seeker.bodyRateStateY.assign(nEntities, 0.0);
    seeker.bodyRateStateZ.assign(nEntities, 0.0);
    seeker.prevBodyRateX.assign(nEntities, 0.0);
    seeker.prevBodyRateY.assign(nEntities, 0.0);
    seeker.prevBodyRateZ.assign(nEntities, 0.0);
    seeker.lockLostTimeSec.assign(nEntities, 0.0);
    seeker.hasPreviousLos.assign(nEntities, false);
}

} // namespace

int main()
{
    std::printf("=== seeker_fidelity_test ===\n");
    const double dt = 0.01;

    // ---- 1. Measurement noise perturbs + is seed-deterministic ----
    {
        auto run = [&](bool noise, std::uint32_t seed, double& azOut) {
            PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
            NavigationBlock nav;
            makeBlocks(physics, status, seeker);
            seeker.measurementNoiseEnabled = {noise, false};
            seeker.angleNoiseStdDevRad = {0.01, 0.0};
            seeker.rangeNoiseStdDevM = {2.0, 0.0};
            seeker.rangeRateNoiseStdDevMps = {1.0, 0.0};
            SeekerSystem system;
            system.setSeed(seed);
            for (int k = 0; k < 10; ++k) system.update(physics, status, seeker, nav, dt);
            azOut = seeker.targetAzimuth[0];
        };
        double exact = 0.0, noisyA = 0.0, noisyB = 0.0, noisySeed2 = 0.0;
        run(false, 7, exact);
        run(true, 7, noisyA);
        run(true, 7, noisyB);
        run(true, 8, noisySeed2);
        check(exact == 0.0, "noise-off measurement is the exact boresight angle");
        check(std::abs(noisyA - exact) > 1e-9, "enabling noise perturbs the published angle");
        check(noisyA == noisyB, "noise draws are seed-deterministic");
        check(noisyA != noisySeed2, "a different seed gives a different noise realization");
    }

    // ---- 2. Angular glint adds a nonzero, range-scaled deviation ----
    {
        auto run = [&](double glintSigma, double& azOut) {
            PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
            NavigationBlock nav;
            makeBlocks(physics, status, seeker);
            seeker.measurementNoiseEnabled = {true, false};
            seeker.angleNoiseStdDevRad = {0.0, 0.0};
            seeker.glintSigmaM = {glintSigma, 0.0};
            SeekerSystem system;
            system.setSeed(3);
            for (int k = 0; k < 20; ++k) system.update(physics, status, seeker, nav, dt);
            azOut = seeker.targetAzimuth[0];
        };
        double noGlint = 0.0, glint = 0.0;
        run(0.0, noGlint);
        run(2.0, glint);
        check(noGlint == 0.0, "zero glint sigma leaves the angle exact");
        check(std::abs(glint) > 1e-6, "glint injects an angular deviation");
    }

    // ---- 3. Range gate rejects an out-of-gate target ----
    {
        PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
            NavigationBlock nav;
        makeBlocks(physics, status, seeker);
        seeker.maxRangeGateM = {50.0, 0.0}; // target is at 100 m
        SeekerSystem system;
        system.update(physics, status, seeker, nav, dt);
        check(!seeker.isLocked[0], "target beyond the max range gate is not locked");
        check(seeker.lockRejectReason[0] == static_cast<int>(SeekerRejectReason::RangeGate),
              "range-gate rejection is reported in the diagnostics");
        seeker.maxRangeGateM[0] = 200.0;
        seeker.minRangeGateM = {50.0, 0.0};
        system.update(physics, status, seeker, nav, dt);
        check(seeker.isLocked[0], "target inside the range gate acquires");
    }

    // ---- 4. Gimbal servo slews toward the LOS instead of snapping ----
    {
        PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
            NavigationBlock nav;
        makeBlocks(physics, status, seeker);
        seeker.gimbalRateLimitRadPerSec = {0.2, 0.0};
        SeekerSystem system;
        system.update(physics, status, seeker, nav, dt); // acquire at az 0
        check(seeker.gimbalAzimuthRad[0] == 0.0, "gimbal starts on the acquired LOS");
        // Jump the target to ~0.5 rad azimuth (within the 45 deg FOV).
        physics.py[1] = 100.0 * std::tan(0.5);
        system.update(physics, status, seeker, nav, dt);
        const double afterOne = seeker.gimbalAzimuthRad[0];
        check(afterOne > 0.0 && afterOne < 0.1,
              "gimbal lags the LOS on the first step (rate limited)");
        for (int k = 0; k < 300; ++k) system.update(physics, status, seeker, nav, dt);
        check(std::abs(seeker.gimbalAzimuthRad[0] - 0.5) < 0.02,
              "gimbal converges onto the stepped LOS");
        check(seeker.isLocked[0], "servo-tracked target stays locked");
    }

    // ---- 5. Lock publication waits for the first delivered measurement ----
    {
        PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
            NavigationBlock nav;
        makeBlocks(physics, status, seeker);
        seeker.measurementLatencySec = {0.15, 0.0};
        SeekerSystem system;
        system.update(physics, status, seeker, nav, dt);
        check(seeker.lockActive[0] && !seeker.hasPublishedMeasurement[0] && !seeker.isLocked[0],
              "latency-delayed lock is not reported until a measurement is published");
        for (int k = 0; k < 20; ++k) system.update(physics, status, seeker, nav, dt);
        check(seeker.hasPublishedMeasurement[0] && seeker.isLocked[0],
              "lock is reported once the delayed measurement arrives");
    }

    // ---- 6. Decoy (Chaff/Flare) rejection hook ----
    {
        auto run = [&](double rejection) {
            PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
            NavigationBlock nav;
            makeBlocks(physics, status, seeker);
            status.type[1] = EntityType::Chaff;
            seeker.decoyRejectionDb = {rejection, 0.0};
            SeekerSystem system;
            system.update(physics, status, seeker, nav, dt);
            return static_cast<bool>(seeker.isLocked[0]);
        };
        check(run(0.0), "chaff is a valid RF target with no rejection configured");
        check(!run(100.0), "configured decoy rejection blinds the seeker to chaff");
    }

    // ---- 7. PassiveRF emitter duty cycle ----
    {
        auto run = [&](double duty) {
            PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
            NavigationBlock nav;
            makeBlocks(physics, status, seeker);
            seeker.type[0] = SeekerType::PassiveRF;
            status.emitterEirpW[1] = 1000.0;
            seeker.passiveRfDutyCycle = {duty, 0.0};
            SeekerSystem system;
            system.setSeed(11);
            bool everLocked = false;
            for (int k = 0; k < 5; ++k) {
                system.update(physics, status, seeker, nav, dt);
                everLocked = everLocked || seeker.isLocked[0];
            }
            return everLocked;
        };
        check(run(1.0), "continuous emitter supports a PassiveRF lock");
        check(!run(0.0), "a zero duty cycle never illuminates the PassiveRF seeker");
    }

    // ---- 8. Terrain line-of-sight masking (local mode) ----
    {
        auto run = [&](bool masking) {
            PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
            NavigationBlock nav;
            makeBlocks(physics, status, seeker);
            seeker.terrainMaskingEnabled = {masking, false};
            EnvironmentConfig env;
            env.terrainElevation = [](double, double) { return 2000.0; }; // ridge above both
            SeekerSystem system;
            system.update(physics, status, seeker, nav, dt, env);
            return static_cast<bool>(seeker.isLocked[0]);
        };
        check(run(false), "terrain masking off locks through the ridge");
        check(!run(true), "terrain masking blocks the masked line of sight");
    }

    // ---- 9. SARH live illuminator binding ----
    {
        PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
            NavigationBlock nav;
        makeBlocks(physics, status, seeker, 3);
        physics.px[2] = 50.0;   // third entity is the illuminator
        status.allegiance[2] = Allegiance::Friendly;
        seeker.type[0] = SeekerType::SARH;
        seeker.illuminatorEntityId = {2, -1, -1};
        SeekerSystem system;
        system.update(physics, status, seeker, nav, dt);
        check(seeker.isLocked[0], "SARH locks with a live illuminator entity bound");
        seeker.illuminatorEntityId[0] = -1; // fall back to the static configured point
        system.update(physics, status, seeker, nav, dt);
        check(seeker.isLocked[0], "SARH locks with the static illuminator fallback");
    }

    // ---- 9b. Velocity gate + RF jammer ----
    {
        PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
            NavigationBlock nav;
        makeBlocks(physics, status, seeker);
        seeker.minClosingRateMps = {100.0, 0.0}; // static target is not closing
        SeekerSystem system;
        system.update(physics, status, seeker, nav, dt);
        check(!seeker.isLocked[0] &&
              seeker.lockRejectReason[0] == static_cast<int>(SeekerRejectReason::VelocityGate),
              "velocity gate rejects a non-closing target");
    }
    {
        auto runJammer = [&](double jammerW) {
            PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
            NavigationBlock nav;
            makeBlocks(physics, status, seeker);
            status.jammerEirpW = {0.0, jammerW};
            SeekerSystem system;
            system.update(physics, status, seeker, nav, dt);
            return static_cast<bool>(seeker.isLocked[0]);
        };
        check(runJammer(0.0), "no jammer acquires the RF target");
        check(!runJammer(1.0e6), "a strong self-protection jammer denies the lock");
    }

    // ---- 10. Config round-trip of the new seeker keys ----
    {
        VehicleConfig cfg;
        cfg.seeker.type = SeekerType::RF;
        cfg.seeker.measurementNoiseEnabled = true;
        cfg.seeker.angleNoiseStdDevRad = 0.004;
        cfg.seeker.glintSigmaM = 1.5;
        cfg.seeker.glintCorrelationTauSec = 0.3;
        cfg.seeker.swerlingEnabled = true;
        cfg.seeker.gimbalRateLimitRadPerSec = 1.25;
        cfg.seeker.minRangeGateM = 150.0;
        cfg.seeker.maxRangeGateM = 40000.0;
        cfg.seeker.terrainMaskingEnabled = true;
        cfg.seeker.minClosingRateMps = 42.0;
        cfg.seeker.rateFilterTauSec = 0.08;
        cfg.seeker.decoyRejectionDb = 6.0;
        cfg.seeker.passiveRfDutyCycle = 0.5;
        cfg.seeker.illuminatorEntityId = 3;
        cfg.jammerEirpW = 5000.0;
        const std::string text = serializeVehicleConfig(cfg);
        const VehicleConfig back = deserializeVehicleConfig(text);
        const auto& s = back.seeker;
        check(s.measurementNoiseEnabled && s.angleNoiseStdDevRad == 0.004 &&
              s.glintSigmaM == 1.5 && s.glintCorrelationTauSec == 0.3 && s.swerlingEnabled,
              "seeker noise/glint/Swerling keys round-trip");
        check(s.gimbalRateLimitRadPerSec == 1.25 && s.minRangeGateM == 150.0 &&
              s.maxRangeGateM == 40000.0 && s.terrainMaskingEnabled && s.minClosingRateMps == 42.0,
              "seeker servo/gate/terrain keys round-trip");
        check(s.rateFilterTauSec == 0.08 && s.decoyRejectionDb == 6.0 &&
              s.passiveRfDutyCycle == 0.5 && s.illuminatorEntityId == 3 && back.jammerEirpW == 5000.0,
              "seeker estimator/decoy/emitter keys round-trip");
    }

    // ---- Measurement latency: the published LOS rate must survive it ----
    // The published rates are outputs: they are copied out of the latency
    // queue. Using them as the rate filter's own state made the filter restart
    // from a stale delayed sample every step, so the published rate came out
    // 3-6x too small whenever latency > 0 -- and the long-range terminal dive
    // steered wrong because of it.
    {
        const double dt = 0.01;
        PhysicsBlock physics;
        EntityStatusBlock status;
        SeekerBlock seeker;
        makeBlocks(physics, status, seeker);
        seeker.measurementLatencySec[0] = 0.05;
        seeker.rateFilterTauSec.assign(2, 0.05);
        NavigationBlock nav;
        nav.size = 2;
        nav.estWx.assign(2, 0.0);
        nav.estWy.assign(2, 0.0);
        nav.estWz.assign(2, 0.0);
        EnvironmentConfig env;
        SeekerSystem system;

        // Target crossing at 20 m/s, 1 km ahead: a constant 0.02 rad/s azimuth
        // rate, no host rotation.
        physics.px[1] = 1000.0;
        physics.vy[1] = 20.0;
        for (int k = 0; k < 200; ++k) {
            physics.py[1] += physics.vy[1] * dt;
            system.update(physics, status, seeker, nav, dt, env);
        }
        check(seeker.isLocked[0], "latency seeker holds the lock");
        check(std::abs(seeker.targetAzimuthRate[0] - 0.02) < 0.002,
              "published azimuth rate survives measurement latency (0.02 rad/s)");
        check(seeker.losRateWorldValid[0] &&
              std::abs(std::sqrt(seeker.losRateWorldX[0] * seeker.losRateWorldX[0] +
                                 seeker.losRateWorldY[0] * seeker.losRateWorldY[0] +
                                 seeker.losRateWorldZ[0] * seeker.losRateWorldZ[0]) - 0.02) < 0.004,
              "geometric inertial LOS rate is published (0.02 rad/s)");

        // Host pitching at 0.05 rad/s with the target fixed in the world: the
        // frame rate must cancel the published body rate exactly as the gyro
        // decoupling expects (they are paired over the same interval).
        PhysicsBlock rotP;
        EntityStatusBlock rotS;
        SeekerBlock rotK;
        makeBlocks(rotP, rotS, rotK);
        rotK.measurementLatencySec[0] = 0.05;
        rotK.rateFilterTauSec.assign(2, 0.05);
        rotP.px[1] = 1000.0;
        const double q = 0.05;
        NavigationBlock rotNav;
        rotNav.size = 2;
        rotNav.estQw.assign(2, 1.0);
        rotNav.estQx.assign(2, 0.0);
        rotNav.estQy.assign(2, 0.0);
        rotNav.estQz.assign(2, 0.0);
        rotNav.estWx.assign(2, 0.0);
        rotNav.estWy.assign(2, q);
        rotNav.estWz.assign(2, 0.0);
        SeekerSystem rotSystem;
        for (int k = 0; k < 200; ++k) {
            const double half = 0.5 * q * dt;
            const double cw = std::cos(half), sw = std::sin(half);
            const double pw = rotP.qw[0], px = rotP.qx[0];
            const double pyq = rotP.qy[0], pz = rotP.qz[0];
            rotP.qw[0] = cw * pw - sw * pyq;
            rotP.qx[0] = cw * px + sw * pz;
            rotP.qy[0] = cw * pyq + sw * pw;
            rotP.qz[0] = cw * pz - sw * px;
            rotNav.estQw[0] = rotP.qw[0]; rotNav.estQx[0] = rotP.qx[0];
            rotNav.estQy[0] = rotP.qy[0]; rotNav.estQz[0] = rotP.qz[0];
            rotSystem.update(rotP, rotS, rotK, rotNav, dt, env);
        }
        check(rotK.isLocked[0], "rotating seeker holds the lock");
        // Frame rate and body rate are near-opposite: the paired sum (the
        // inertial LOS rate of a world-fixed target) must stay small.
        const double inertialEl = rotK.targetElevationRate[0] + rotK.bodyRateFilteredY[0];
        check(std::abs(rotK.targetElevationRate[0]) > 0.02,
              "rotating host still publishes the frame rate");
        check(std::abs(inertialEl) < 0.01,
              "paired frame + body rate cancels to the inertial LOS rate");

        // Same cancellation from a NON-identity, rolled attitude: a body-frame
        // pairing must not care how the body is oriented in the world.
        PhysicsBlock skP;
        EntityStatusBlock skS;
        SeekerBlock skK;
        makeBlocks(skP, skS, skK);
        skK.rateFilterTauSec.assign(2, 0.02);
        skP.px[1] = 1000.0; skP.py[1] = 200.0; skP.pz[1] = 100.0;  // world-fixed
        skP.vx[1] = 0.0; skP.vy[1] = 0.0; skP.vz[1] = 0.0;
        const double roll = 1.0471975512;                        // 60 deg
        skP.qw[0] = std::cos(0.5 * roll); skP.qx[0] = std::sin(0.5 * roll);
        NavigationBlock skNav;
        skNav.size = 2;
        skNav.estQw.assign(2, skP.qw[0]); skNav.estQx.assign(2, skP.qx[0]);
        skNav.estQy.assign(2, 0.0); skNav.estQz.assign(2, 0.0);
        skNav.estWx.assign(2, 0.0);
        skNav.estWy.assign(2, q);
        skNav.estWz.assign(2, 0.0);
        SeekerSystem skSystem;
        for (int k = 0; k < 200; ++k) {
            const double half = 0.5 * q * dt;
            const double cw = std::cos(half), sw = std::sin(half);
            const double pw = skP.qw[0], px = skP.qx[0];
            const double pyq = skP.qy[0], pz = skP.qz[0];
            skP.qw[0] = cw * pw - sw * pyq;
            skP.qx[0] = cw * px + sw * pz;
            skP.qy[0] = cw * pyq + sw * pw;
            skP.qz[0] = cw * pz - sw * px;
            skNav.estQw[0] = skP.qw[0]; skNav.estQx[0] = skP.qx[0];
            skNav.estQy[0] = skP.qy[0]; skNav.estQz[0] = skP.qz[0];
            skSystem.update(skP, skS, skK, skNav, dt, env);
        }
        check(skK.isLocked[0], "rolled seeker holds the lock");
        // Only the component perpendicular to the LOS drives the demand; the
        // along-LOS part is the body rotation about the sightline and is
        // removed by the cross product in pnDemand.
        {
            const glm::dvec3 wW(skK.losRateWorldX[0], skK.losRateWorldY[0],
                                skK.losRateWorldZ[0]);
            const glm::dvec3 uW(1000.0 - skP.px[0], 200.0 - skP.py[0], 100.0 - skP.pz[0]);
            const glm::dvec3 perp = glm::cross(wW, glm::normalize(uW));
            const glm::dquat qT(skP.qw[0], skP.qx[0], skP.qy[0], skP.qz[0]);
            const glm::dvec3 lbN2 = glm::normalize(glm::inverse(qT) * uW);
            // Diagnostic kept for the open bug: the measured angles match the
            // truth, so the fault is in the (az, el, dAz, dEl) -> rate
            // reconstruction for out-of-plane geometries.
            std::printf("      [rolled] az=%.4f/%.4f el=%.4f/%.4f | azR=%.4f elR=%.4f "
                        "br=(%.4f,%.4f,%.4f) | wW=(%.4f,%.4f,%.4f) perp=%.4f\n",
                        skK.targetAzimuth[0], std::atan2(lbN2.y, lbN2.x),
                        skK.targetElevation[0], std::asin(std::clamp(-lbN2.z, -1.0, 1.0)),
                        skK.targetAzimuthRate[0], skK.targetElevationRate[0],
                        skK.bodyRateFilteredX[0], skK.bodyRateFilteredY[0],
                        skK.bodyRateFilteredZ[0], wW.x, wW.y, wW.z, glm::length(perp));
            check(skK.losRateWorldValid[0] && glm::length(perp) < 0.01,
                  "rolled attitude: perpendicular world rate ~0 for a fixed LOS");
        }

        check(rotK.losRateWorldValid[0] &&
              glm::length(glm::dvec3(rotK.losRateWorldX[0], rotK.losRateWorldY[0],
                                     rotK.losRateWorldZ[0])) < 0.01,
              "world-frame rate stays ~0 while the host pitches (no residual)");

        // Skipped maintenance steps: the target is hidden every other step
        // (geometry rejection, lock kept by the dropout window). The rate must
        // divide by the elapsed time, not by the nominal step.
        PhysicsBlock gapP;
        EntityStatusBlock gapS;
        SeekerBlock gapK;
        makeBlocks(gapP, gapS, gapK);
        gapK.rateFilterTauSec.assign(2, 0.02);
        // Hidden = beyond the range gate for one step: the lock is retained by
        // the dropout window but the measurement is not committed.
        gapK.maxRangeGateM.assign(2, 1500.0);
        NavigationBlock gapNav;
        gapNav.size = 2;
        gapNav.estWx.assign(2, 0.0);
        gapNav.estWy.assign(2, 0.0);
        gapNav.estWz.assign(2, 0.0);
        SeekerSystem gapSystem;
        // Committed azimuths step by +0.02 rad every TWO steps: a steady
        // 1.0 rad/s LOS rate whose backward difference spans the skipped step.
        for (int k = 0; k < 60; ++k) {
            const bool hidden = (k % 2) == 1;
            const double az = 0.10 + 0.02 * (k / 2);
            const double r = hidden ? 3000.0 : 1000.0;
            gapP.px[1] = r * std::cos(az);
            gapP.py[1] = r * std::sin(az);
            gapP.pz[1] = 1000.0;
            gapSystem.update(gapP, gapS, gapK, gapNav, dt, env);
        }
        check(gapK.isLocked[0], "lock survives intermittent geometry rejection");
        check(std::abs(gapK.targetAzimuthRate[0] - 1.0) < 0.2,
              "skipped-step rate divides by elapsed time (1.0 rad/s over 2 steps)");
    }

    if (g_failures == 0) std::printf("ALL SEEKER FIDELITY TESTS PASSED\n");
    else std::printf("FAILED with %d failures\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
