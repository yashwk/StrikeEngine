// Richer seekers: PassiveRF, SARH (bistatic), Beer-Lambert IR transmittance,
// chaff/flare decoys, strongest-signal acquisition, and the public SeekerConfig
// surface. All scenarios are deterministic (the seeker path has no RNG).
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/SeekerConfig.hpp>
#include <cmath>
#include <cstdio>
#include <string>

using namespace StrikeEngine::Kernel;

namespace {

const std::string kRcsFlat = std::string(STRIKEENGINE_SOURCE_DIR) + "/tests/data/flat_rcs.json";
const std::string kRcsLarge = std::string(STRIKEENGINE_SOURCE_DIR) + "/tests/data/flat_rcs_large.json";
const std::string kIrFlat = std::string(STRIKEENGINE_SOURCE_DIR) + "/tests/data/flat_ir.json";
const std::string kIrHot = std::string(STRIKEENGINE_SOURCE_DIR) + "/tests/data/flat_ir_hot.json";

VehicleInitState makeInit(double x, double y, double z,
                          EntityType type, Allegiance alleg,
                          const std::string& rcs = "",
                          const std::string& ir = "",
                          double eirp = 0.0)
{
    VehicleInitState init{};
    init.px = x; init.py = y; init.pz = z;
    init.vx = 0; init.vy = 0; init.vz = 0;
    init.qw = 1; init.qx = 0; init.qy = 0; init.qz = 0;
    init.wx = 0; init.wy = 0; init.wz = 0;
    init.mass = 100.0;
    init.type = type;
    init.allegiance = alleg;
    init.rcsProfileId = rcs;
    init.irProfileId = ir;
    init.emitterEirpW = eirp;
    return init;
}

// Static-geometry config: no aero, wide FOV, zero dropout so single-step
// checks behave cleanly.
VehicleConfig baseConfig()
{
    VehicleConfig cfg;
    cfg.referenceArea = 0.0;
    cfg.seeker.fieldOfViewHalfAngleRad = 3.14159265358979323846 / 2.0;
    cfg.seeker.gimbalAzimuthLimitRad = 3.14159265358979323846 / 2.0;
    cfg.seeker.gimbalElevationLimitRad = 3.14159265358979323846 / 2.0;
    cfg.seeker.lockDropoutTimeSec = 0.0;
    return cfg;
}

PhysicsId addVehicle(SimulationKernel& kernel, const VehicleInitState& init,
                     const VehicleConfig& cfg)
{
    return kernel.createVehicle(init, cfg);
}

} // namespace

int main()
{
    std::printf("=== seeker_rich_test: PassiveRF, SARH, IR transmittance, decoys, config ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };
    constexpr double dt = 0.01;

    // ---- 1. PassiveRF ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(42u);
        VehicleConfig cfg = baseConfig();
        cfg.seeker.type = SeekerType::PassiveRF;
        cfg.seeker.antennaGainDb = 30.0;
        cfg.seeker.wavelengthM = 0.03;
        cfg.seeker.noiseFloorW = 1e-12;
        cfg.seeker.snrThresholdDb = 13.0;
        const auto seekerId = addVehicle(kernel,
            makeInit(0.0, 0.0, 10000.0, EntityType::Missile, Allegiance::Friendly), cfg);

        // Emitter at 100 km: SNR ~57.6 dB -> detected.
        const auto emitterFar = addVehicle(kernel,
            makeInit(1.0e5, 0.0, 10000.0, EntityType::Aircraft, Allegiance::Hostile, "", "", 1.0e6),
            baseConfig());
        kernel.step(dt);
        check(kernel.getSeekers().isLocked[seekerId] &&
                  kernel.getSeekers().lockedTargetId[seekerId] == emitterFar,
              "PassiveRF acquires a target with emitterEirpW > 0 at 100 km");
    }
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(42u);
        VehicleConfig cfg = baseConfig();
        cfg.seeker.type = SeekerType::PassiveRF;
        cfg.seeker.antennaGainDb = 30.0;
        cfg.seeker.wavelengthM = 0.03;
        cfg.seeker.noiseFloorW = 1e-12;
        cfg.seeker.snrThresholdDb = 13.0;
        const auto seekerId = addVehicle(kernel,
            makeInit(0.0, 0.0, 10000.0, EntityType::Missile, Allegiance::Friendly), cfg);

        // Same emitter at 40000 km: SNR ~5.5 dB -> not detected.
        addVehicle(kernel,
            makeInit(4.0e7, 0.0, 10000.0, EntityType::Aircraft, Allegiance::Hostile, "", "", 1.0e6),
            baseConfig());
        kernel.step(dt);
        check(!kernel.getSeekers().isLocked[seekerId],
              "PassiveRF does not detect the emitter beyond the SNR threshold range");
    }
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(42u);
        VehicleConfig cfg = baseConfig();
        cfg.seeker.type = SeekerType::PassiveRF;
        cfg.seeker.antennaGainDb = 30.0;
        cfg.seeker.wavelengthM = 0.03;
        cfg.seeker.noiseFloorW = 1e-12;
        cfg.seeker.snrThresholdDb = 13.0;
        const auto seekerId = addVehicle(kernel,
            makeInit(0.0, 0.0, 10000.0, EntityType::Missile, Allegiance::Friendly), cfg);

        // No emitter at all -> never detected, even closer than an emitter.
        addVehicle(kernel,
            makeInit(5.0e4, 0.0, 10000.0, EntityType::Aircraft, Allegiance::Hostile, "", "", 0.0),
            baseConfig());
        kernel.step(dt);
        check(!kernel.getSeekers().isLocked[seekerId],
              "PassiveRF ignores targets with emitterEirpW = 0");
    }
    {
        // Strongest-signal acquisition: the only emitting target wins over a
        // closer non-emitting one.
        SimulationKernel kernel;
        kernel.setRandomSeed(42u);
        VehicleConfig cfg = baseConfig();
        cfg.seeker.type = SeekerType::PassiveRF;
        cfg.seeker.antennaGainDb = 30.0;
        cfg.seeker.wavelengthM = 0.03;
        cfg.seeker.noiseFloorW = 1e-12;
        cfg.seeker.snrThresholdDb = 13.0;
        const auto seekerId = addVehicle(kernel,
            makeInit(0.0, 0.0, 10000.0, EntityType::Missile, Allegiance::Friendly), cfg);

        const auto emitterId = addVehicle(kernel,
            makeInit(1.0e5, 0.0, 10000.0, EntityType::Aircraft, Allegiance::Hostile, "", "", 1.0e6),
            baseConfig());
        addVehicle(kernel,
            makeInit(5.0e4, 0.0, 10000.0, EntityType::Aircraft, Allegiance::Hostile, "", "", 0.0),
            baseConfig());
        kernel.step(dt);
        check(kernel.getSeekers().isLocked[seekerId] &&
                  kernel.getSeekers().lockedTargetId[seekerId] == emitterId,
              "PassiveRF locks the emitting target (strongest signal) over a closer silent one");
    }

    // ---- 2. SARH (bistatic) ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(42u);
        // Three co-located friendly seekers at (1000,0,0), one hostile target
        // at (2010,0,0) with sigma = 1 m^2.
        VehicleConfig rfCfg = baseConfig();
        rfCfg.seeker.type = SeekerType::RF;
        rfCfg.seeker.transmitterPowerW = 1000.0;
        rfCfg.seeker.snrThresholdDb = 100.0;
        addVehicle(kernel, makeInit(1000.0, 0.0, 10000.0, EntityType::Missile, Allegiance::Friendly), rfCfg);

        VehicleConfig sarhNear = baseConfig();
        sarhNear.seeker.type = SeekerType::SARH;
        sarhNear.seeker.snrThresholdDb = 100.0;
        sarhNear.seeker.illuminatorPx = 2000.0;   // 10 m from the target (same altitude)
        sarhNear.seeker.illuminatorPz = 10000.0;
        sarhNear.seeker.illuminatorPowerW = 5.0e5;
        sarhNear.seeker.illuminatorGainDb = 38.0;
        sarhNear.seeker.illuminatorWavelengthM = 0.03;
        const auto sarhNearId = addVehicle(kernel,
            makeInit(1000.0, 0.0, 10000.0, EntityType::Missile, Allegiance::Friendly), sarhNear);

        VehicleConfig sarhFar = sarhNear;
        sarhFar.seeker.illuminatorPx = 3000.0;    // 990 m from the target
        const auto sarhFarId = addVehicle(kernel,
            makeInit(1000.0, 0.0, 10000.0, EntityType::Missile, Allegiance::Friendly), sarhFar);

        addVehicle(kernel,
            makeInit(2010.0, 0.0, 10000.0, EntityType::Aircraft, Allegiance::Hostile, kRcsFlat),
            baseConfig());
        kernel.step(dt);

        // Monostatic at the same range: SNR ~26 dB < 100 dB -> fails.
        check(!kernel.getSeekers().isLocked[0],
              "monostatic RF at the same range does not pass the 100 dB threshold");
        // Bistatic with the illuminator 10 m from the target: SNR ~101 dB -> passes.
        check(kernel.getSeekers().isLocked[sarhNearId],
              "SARH detects the target when the illuminator is near it (Rt=10 m)");
        // Bistatic with the illuminator moved far: SNR ~62 dB -> fails.
        check(!kernel.getSeekers().isLocked[sarhFarId],
              "SARH loses detection when the illuminator moves far (Rt=990 m)");
    }

    // ---- 3. IR Beer-Lambert transmittance ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(42u);
        VehicleConfig clearCfg = baseConfig();
        clearCfg.seeker.type = SeekerType::IR;
        clearCfg.seeker.irExtinctionPerM = 0.0;    // vacuum
        const auto clearId = addVehicle(kernel,
            makeInit(0.0, 0.0, 10000.0, EntityType::Missile, Allegiance::Friendly), clearCfg);

        VehicleConfig hazyCfg = baseConfig();
        hazyCfg.seeker.type = SeekerType::IR;
        hazyCfg.seeker.irExtinctionPerM = 1e-4;    // 0.1/km (legacy placeholder)
        const auto hazyId = addVehicle(kernel,
            makeInit(0.0, 0.0, 10000.0, EntityType::Missile, Allegiance::Friendly), hazyCfg);

        addVehicle(kernel,
            makeInit(60000.0, 0.0, 10000.0, EntityType::Aircraft, Allegiance::Hostile, "", kIrFlat),
            baseConfig());
        kernel.step(dt);
        // At 60 km the hazy seeker sees exp(-6) ~ 2.5e-3 transmittance; the
        // received power drops below sensitivity.
        check(kernel.getSeekers().isLocked[clearId],
              "IR with zero extinction acquires at 60 km");
        check(!kernel.getSeekers().isLocked[hazyId],
              "IR with 1e-4 extinction fails to acquire at 60 km");
    }
    {
        // Closing to 40 km: transmittance exp(-4) recovers acquisition.
        SimulationKernel kernel;
        kernel.setRandomSeed(42u);
        VehicleConfig clearCfg = baseConfig();
        clearCfg.seeker.type = SeekerType::IR;
        clearCfg.seeker.irExtinctionPerM = 0.0;
        const auto clearId = addVehicle(kernel,
            makeInit(0.0, 0.0, 10000.0, EntityType::Missile, Allegiance::Friendly), clearCfg);

        VehicleConfig hazyCfg = baseConfig();
        hazyCfg.seeker.type = SeekerType::IR;
        hazyCfg.seeker.irExtinctionPerM = 1e-4;
        const auto hazyId = addVehicle(kernel,
            makeInit(0.0, 0.0, 10000.0, EntityType::Missile, Allegiance::Friendly), hazyCfg);

        addVehicle(kernel,
            makeInit(40000.0, 0.0, 10000.0, EntityType::Aircraft, Allegiance::Hostile, "", kIrFlat),
            baseConfig());
        kernel.step(dt);
        check(kernel.getSeekers().isLocked[clearId] &&
                  kernel.getSeekers().isLocked[hazyId],
              "Beer-Lambert extinction only shortens the acquisition range");
    }

    // ---- 4. Chaff / flare decoys (strongest signal seduces the seeker) ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(42u);
        VehicleConfig rfCfg = baseConfig();
        rfCfg.seeker.type = SeekerType::RF;
        const auto seekerId = addVehicle(kernel,
            makeInit(0.0, 0.0, 10000.0, EntityType::Missile, Allegiance::Friendly), rfCfg);
        const auto realTarget = addVehicle(kernel,
            makeInit(100.0, 0.0, 10000.0, EntityType::Aircraft, Allegiance::Hostile, kRcsFlat),
            baseConfig());
        const auto chaffId = addVehicle(kernel,
            makeInit(50.0, 0.0, 10000.0, EntityType::Chaff, Allegiance::Hostile, kRcsLarge),
            baseConfig());
        kernel.step(dt);
        check(kernel.getSeekers().isLocked[seekerId] &&
                  kernel.getSeekers().lockedTargetId[seekerId] == chaffId,
              "RF seeker acquires the nearer, larger-RCS chaff (strongest signal)");

        kernel.removeVehicle(chaffId);
        kernel.step(dt);
        check(kernel.getSeekers().isLocked[seekerId] &&
                  kernel.getSeekers().lockedTargetId[seekerId] == realTarget,
              "RF seeker re-acquires the real target after the chaff is gone");
    }
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(42u);
        VehicleConfig irCfg = baseConfig();
        irCfg.seeker.type = SeekerType::IR;
        const auto seekerId = addVehicle(kernel,
            makeInit(0.0, 0.0, 10000.0, EntityType::Missile, Allegiance::Friendly), irCfg);
        addVehicle(kernel,
            makeInit(100.0, 0.0, 10000.0, EntityType::Aircraft, Allegiance::Hostile, "", kIrFlat),
            baseConfig());
        const auto flareId = addVehicle(kernel,
            makeInit(50.0, 0.0, 10000.0, EntityType::Flare, Allegiance::Hostile, "", kIrHot),
            baseConfig());
        kernel.step(dt);
        check(kernel.getSeekers().isLocked[seekerId] &&
                  kernel.getSeekers().lockedTargetId[seekerId] == flareId,
              "IR seeker acquires the hotter, nearer flare (strongest signal)");
    }

    // ---- 5. Public SeekerConfig round-trip ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(42u);
        VehicleConfig cfg = baseConfig();
        cfg.seeker.type = SeekerType::SARH;
        cfg.seeker.transmitterPowerW = 2000.0;
        cfg.seeker.antennaGainDb = 25.0;
        cfg.seeker.wavelengthM = 0.05;
        cfg.seeker.noiseFloorW = 2.0e-11;
        cfg.seeker.snrThresholdDb = 20.0;
        cfg.seeker.sensitivityW = 5.0e-9;
        cfg.seeker.wavelengthBand = 1;
        cfg.seeker.irExtinctionPerM = 5.0e-4;
        cfg.seeker.illuminatorPx = 100.0;
        cfg.seeker.illuminatorPy = 200.0;
        cfg.seeker.illuminatorPz = 300.0;
        cfg.seeker.illuminatorPowerW = 1.0e6;
        cfg.seeker.illuminatorGainDb = 40.0;
        cfg.seeker.illuminatorWavelengthM = 0.04;
        cfg.seeker.fieldOfViewHalfAngleRad = 0.5;
        cfg.seeker.gimbalAzimuthLimitRad = 0.6;
        cfg.seeker.gimbalElevationLimitRad = 0.7;
        cfg.seeker.lockHysteresisDb = 4.0;
        cfg.seeker.lockDropoutTimeSec = 0.2;
        cfg.seeker.measurementLatencySec = 0.05;

        VehicleInitState init = makeInit(0.0, 0.0, 10000.0,
                                         EntityType::Missile, Allegiance::Friendly);
        init.emitterEirpW = 2.5e6;
        const auto id = kernel.createVehicle(init, cfg);

        const auto& seeker = kernel.getSeekers();
        check(seeker.type[id] == SeekerType::SARH, "SeekerConfig.type round-trips");
        check(seeker.transmitterPowerW[id] == 2000.0 &&
                  seeker.antennaGainDb[id] == 25.0 &&
                  seeker.wavelengthM[id] == 0.05 &&
                  seeker.noiseFloorW[id] == 2.0e-11 &&
                  seeker.snrThresholdDb[id] == 20.0,
              "SeekerConfig RF fields round-trip");
        check(seeker.sensitivityW[id] == 5.0e-9 &&
                  seeker.wavelengthBand[id] == 1 &&
                  seeker.irExtinctionPerM[id] == 5.0e-4,
              "SeekerConfig IR fields round-trip");
        check(seeker.illuminatorPx[id] == 100.0 &&
                  seeker.illuminatorPy[id] == 200.0 &&
                  seeker.illuminatorPz[id] == 300.0 &&
                  seeker.illuminatorPowerW[id] == 1.0e6 &&
                  seeker.illuminatorGainDb[id] == 40.0 &&
                  seeker.illuminatorWavelengthM[id] == 0.04,
              "SeekerConfig SARH illuminator fields round-trip");
        check(seeker.fieldOfViewHalfAngleRad[id] == 0.5 &&
                  seeker.gimbalAzimuthLimitRad[id] == 0.6 &&
                  seeker.gimbalElevationLimitRad[id] == 0.7 &&
                  seeker.lockHysteresisDb[id] == 4.0 &&
                  seeker.lockDropoutTimeSec[id] == 0.2 &&
                  seeker.measurementLatencySec[id] == 0.05,
              "SeekerConfig geometry/tracking fields round-trip");
        check(kernel.getStatus().emitterEirpW[id] == 2.5e6,
              "VehicleInitState.emitterEirpW round-trips into statusBlock");
    }
    {
        // Backward compatibility: legacy init.seekerType still drives the type
        // when the config leaves it at the default None.
        SimulationKernel kernel;
        kernel.setRandomSeed(42u);
        VehicleInitState init = makeInit(0.0, 0.0, 10000.0,
                                         EntityType::Missile, Allegiance::Friendly);
        init.seekerType = SeekerType::RF;
        const auto id = kernel.createVehicle(init, baseConfig());
        check(kernel.getSeekers().type[id] == SeekerType::RF,
              "legacy init.seekerType drives the type with a default SeekerConfig");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
