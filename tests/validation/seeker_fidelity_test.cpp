// seeker_fidelity_test: measurement noise/glint/Swerling, gimbal servo,
// range gates, terrain masking, decoy rejection, emitter duty, lock
// publication consistency, and config round-trips. Every capability is
// opt-in; the legacy path (all flags off) is exercised by seeker_test.
#include <strikeengine/kernel/systems/SeekerSystem.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <strikeengine/kernel/config/ConfigSerialization.hpp>

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
            makeBlocks(physics, status, seeker);
            seeker.measurementNoiseEnabled = {noise, false};
            seeker.angleNoiseStdDevRad = {0.01, 0.0};
            seeker.rangeNoiseStdDevM = {2.0, 0.0};
            seeker.rangeRateNoiseStdDevMps = {1.0, 0.0};
            SeekerSystem system;
            system.setSeed(seed);
            for (int k = 0; k < 10; ++k) system.update(physics, status, seeker, dt);
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
            makeBlocks(physics, status, seeker);
            seeker.measurementNoiseEnabled = {true, false};
            seeker.angleNoiseStdDevRad = {0.0, 0.0};
            seeker.glintSigmaM = {glintSigma, 0.0};
            SeekerSystem system;
            system.setSeed(3);
            for (int k = 0; k < 20; ++k) system.update(physics, status, seeker, dt);
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
        makeBlocks(physics, status, seeker);
        seeker.maxRangeGateM = {50.0, 0.0}; // target is at 100 m
        SeekerSystem system;
        system.update(physics, status, seeker, dt);
        check(!seeker.isLocked[0], "target beyond the max range gate is not locked");
        check(seeker.lockRejectReason[0] == static_cast<int>(SeekerRejectReason::RangeGate),
              "range-gate rejection is reported in the diagnostics");
        seeker.maxRangeGateM[0] = 200.0;
        seeker.minRangeGateM = {50.0, 0.0};
        system.update(physics, status, seeker, dt);
        check(seeker.isLocked[0], "target inside the range gate acquires");
    }

    // ---- 4. Gimbal servo slews toward the LOS instead of snapping ----
    {
        PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
        makeBlocks(physics, status, seeker);
        seeker.gimbalRateLimitRadPerSec = {0.2, 0.0};
        SeekerSystem system;
        system.update(physics, status, seeker, dt); // acquire at az 0
        check(seeker.gimbalAzimuthRad[0] == 0.0, "gimbal starts on the acquired LOS");
        // Jump the target to ~0.5 rad azimuth (within the 45 deg FOV).
        physics.py[1] = 100.0 * std::tan(0.5);
        system.update(physics, status, seeker, dt);
        const double afterOne = seeker.gimbalAzimuthRad[0];
        check(afterOne > 0.0 && afterOne < 0.1,
              "gimbal lags the LOS on the first step (rate limited)");
        for (int k = 0; k < 300; ++k) system.update(physics, status, seeker, dt);
        check(std::abs(seeker.gimbalAzimuthRad[0] - 0.5) < 0.02,
              "gimbal converges onto the stepped LOS");
        check(seeker.isLocked[0], "servo-tracked target stays locked");
    }

    // ---- 5. Lock publication waits for the first delivered measurement ----
    {
        PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
        makeBlocks(physics, status, seeker);
        seeker.measurementLatencySec = {0.15, 0.0};
        SeekerSystem system;
        system.update(physics, status, seeker, dt);
        check(seeker.lockActive[0] && !seeker.hasPublishedMeasurement[0] && !seeker.isLocked[0],
              "latency-delayed lock is not reported until a measurement is published");
        for (int k = 0; k < 20; ++k) system.update(physics, status, seeker, dt);
        check(seeker.hasPublishedMeasurement[0] && seeker.isLocked[0],
              "lock is reported once the delayed measurement arrives");
    }

    // ---- 6. Decoy (Chaff/Flare) rejection hook ----
    {
        auto run = [&](double rejection) {
            PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
            makeBlocks(physics, status, seeker);
            status.type[1] = EntityType::Chaff;
            seeker.decoyRejectionDb = {rejection, 0.0};
            SeekerSystem system;
            system.update(physics, status, seeker, dt);
            return static_cast<bool>(seeker.isLocked[0]);
        };
        check(run(0.0), "chaff is a valid RF target with no rejection configured");
        check(!run(100.0), "configured decoy rejection blinds the seeker to chaff");
    }

    // ---- 7. PassiveRF emitter duty cycle ----
    {
        auto run = [&](double duty) {
            PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
            makeBlocks(physics, status, seeker);
            seeker.type[0] = SeekerType::PassiveRF;
            status.emitterEirpW[1] = 1000.0;
            seeker.passiveRfDutyCycle = {duty, 0.0};
            SeekerSystem system;
            system.setSeed(11);
            bool everLocked = false;
            for (int k = 0; k < 5; ++k) {
                system.update(physics, status, seeker, dt);
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
            makeBlocks(physics, status, seeker);
            seeker.terrainMaskingEnabled = {masking, false};
            EnvironmentConfig env;
            env.terrainElevation = [](double, double) { return 2000.0; }; // ridge above both
            SeekerSystem system;
            system.update(physics, status, seeker, dt, env);
            return static_cast<bool>(seeker.isLocked[0]);
        };
        check(run(false), "terrain masking off locks through the ridge");
        check(!run(true), "terrain masking blocks the masked line of sight");
    }

    // ---- 9. SARH live illuminator binding ----
    {
        PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
        makeBlocks(physics, status, seeker, 3);
        physics.px[2] = 50.0;   // third entity is the illuminator
        status.allegiance[2] = Allegiance::Friendly;
        seeker.type[0] = SeekerType::SARH;
        seeker.illuminatorEntityId = {2, -1, -1};
        SeekerSystem system;
        system.update(physics, status, seeker, dt);
        check(seeker.isLocked[0], "SARH locks with a live illuminator entity bound");
        seeker.illuminatorEntityId[0] = -1; // fall back to the static configured point
        system.update(physics, status, seeker, dt);
        check(seeker.isLocked[0], "SARH locks with the static illuminator fallback");
    }

    // ---- 9b. Velocity gate + RF jammer ----
    {
        PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
        makeBlocks(physics, status, seeker);
        seeker.minClosingRateMps = {100.0, 0.0}; // static target is not closing
        SeekerSystem system;
        system.update(physics, status, seeker, dt);
        check(!seeker.isLocked[0] &&
              seeker.lockRejectReason[0] == static_cast<int>(SeekerRejectReason::VelocityGate),
              "velocity gate rejects a non-closing target");
    }
    {
        auto runJammer = [&](double jammerW) {
            PhysicsBlock physics; EntityStatusBlock status; SeekerBlock seeker;
            makeBlocks(physics, status, seeker);
            status.jammerEirpW = {0.0, jammerW};
            SeekerSystem system;
            system.update(physics, status, seeker, dt);
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

    if (g_failures == 0) std::printf("ALL SEEKER FIDELITY TESTS PASSED\n");
    else std::printf("FAILED with %d failures\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
