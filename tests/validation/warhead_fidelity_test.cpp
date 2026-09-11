// warhead_fidelity_test: terminal fuse and lethality fidelity.
//
// Covers the opt-in upgrades added by the warhead/event pass:
//   - analytic closest-approach (CPA) fuzing and miss evaluation,
//   - arming delay, minimum closing rate, self-destruct,
//   - probabilistic fuze detection on its own RNG stream,
//   - damage accumulation with structural hardness,
//   - aspect-dependent lethality factors,
//   - detonation telemetry (event + WarheadState diagnostics),
//   - kinetic-contact latch, swept ground impact, impact rate zeroing,
//   - config serialization, profile parsing and fuse validation.
// All defaults are legacy-identical; the existing suites pin the legacy law.
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/ConfigSerialization.hpp>
#include <strikeengine/kernel/profiles/WarheadProfileDatabase.hpp>

#include <cstdio>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>

using namespace StrikeEngine::Kernel;

namespace {

int failures = 0;

void check(bool ok, const char* what)
{
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

VehicleInitState makeInit(double px, double py, double pz,
                          double vx, double vy, double vz,
                          Allegiance allegiance = Allegiance::Friendly)
{
    VehicleInitState init{};
    init.px = px; init.py = py; init.pz = pz;
    init.vx = vx; init.vy = vy; init.vz = vz;
    init.qw = 1; init.qx = 0; init.qy = 0; init.qz = 0;
    init.wx = 0; init.wy = 0; init.wz = 0;
    init.mass = 100.0;
    init.allegiance = allegiance;
    return init;
}

// Drag/lift-free ballistic vehicle so the test geometry is exact.
VehicleConfig ballistic()
{
    VehicleConfig cfg;
    cfg.aero.cd = 0.0;
    cfg.aero.clAlpha = 0.0;
    cfg.aero.clFin = 0.0;
    cfg.aero.clMax = 0.0;
    return cfg;
}

VehicleConfig proximityWarhead(double lethal, double falloff)
{
    VehicleConfig cfg = ballistic();
    cfg.warhead.fusing = FusingType::Proximity;
    cfg.warhead.proximityTriggerM = 90.0;
    cfg.warhead.lethalRadiusM = lethal;
    cfg.warhead.falloffRadiusM = falloff;
    return cfg;
}

void stepN(SimulationKernel& kernel, int steps, double dt = 0.001)
{
    for (int i = 0; i < steps; ++i) kernel.step(dt);
}

// High-closing-speed pass: missile and target close head-on at 2000 m/s with
// a lateral offset of `offsetY` metres, so the true CPA is offsetY. The legacy
// fuse triggers inside a 0.02 s lookahead (~40 m of range) and evaluates PK on
// that pre-CPA range; CPA fuzing evaluates the projected miss.
struct PassResult { bool targetDead; bool detonated; };
PassResult runCpaPass(bool cpaEnabled, double offsetY)
{
    SimulationKernel kernel;
    kernel.setRandomSeed(0x7A57u);
    VehicleConfig missileCfg = proximityWarhead(10.0, 30.0);
    missileCfg.warhead.cpaFuzingEnabled = cpaEnabled;
    const auto m = kernel.createVehicle(makeInit(-50.0, 0.0, 100.0, 1000.0, 0.0, 0.0), missileCfg);
    const auto t = kernel.createVehicle(makeInit(50.0, offsetY, 100.0, -1000.0, 0.0, 0.0, Allegiance::Hostile), ballistic());
    stepN(kernel, 60);
    return { !kernel.getStatus().isAlive[t], kernel.getWarhead(m).detonated };
}

bool atRestKill(const VehicleConfig& warheadCfg, double hardness, int steps)
{
    SimulationKernel kernel;
    kernel.setRandomSeed(0xA11u);
    const auto m = kernel.createVehicle(makeInit(-5.0, 0.0, 100.0, 0.0, 0.0, 0.0), warheadCfg);
    VehicleConfig targetCfg = ballistic();
    targetCfg.structuralHardness = hardness;
    (void)m;
    const auto t = kernel.createVehicle(makeInit(0.0, 0.0, 100.0, 0.0, 0.0, 0.0, Allegiance::Hostile), targetCfg);
    stepN(kernel, steps);
    return !kernel.getStatus().isAlive[t];
}

} // namespace

int main()
{
    std::printf("=== warhead_fidelity_test: terminal fuse + lethality ===\n");

    // ---- 1. CPA fuzing vs legacy on a high-closing-speed pass ----
    {
        const PassResult legacy = runCpaPass(false, 8.0);
        const PassResult cpa = runCpaPass(true, 8.0);
        check(legacy.detonated && !legacy.targetDead,
              "legacy fuse detonates but scores the pre-CPA range (no kill at 8 m true CPA)");
        check(cpa.detonated && cpa.targetDead,
              "CPA fuse scores the projected 8 m miss (guaranteed kill inside lethal)");
    }
    {
        // Lateral offset 20 m in a (10, 30] band: legacy miss ~41 m -> p=0;
        // CPA miss 20 m -> p=0.5 (deterministic under a fixed seed).
        const PassResult first = runCpaPass(true, 20.0);
        const PassResult second = runCpaPass(true, 20.0);
        check(first.targetDead == second.targetDead,
              "CPA in-band kill is deterministic under a fixed seed");
        std::printf("    pinned CPA in-band outcome (miss 20, lethal 10, falloff 30): %s\n",
                    first.targetDead ? "KILL" : "NO KILL");
    }

    // ---- 2. Arming delay ----
    {
        VehicleConfig cfg = proximityWarhead(10.0, 0.0);
        cfg.warhead.armingDelaySec = 0.5;
        check(!atRestKill(cfg, 100.0, 400), "no detonation before the arming delay");
        check(atRestKill(cfg, 100.0, 600), "fuse fires once armed");
        VehicleConfig noArm = proximityWarhead(10.0, 0.0);
        check(atRestKill(noArm, 100.0, 400), "zero arming delay preserves immediate fuzing");
    }

    // ---- 3. Minimum closing-rate gate ----
    {
        VehicleConfig gated = proximityWarhead(10.0, 0.0);
        gated.warhead.minClosingSpeedMps = 1.0;   // at rest the closing rate is 0
        check(!atRestKill(gated, 100.0, 500), "min closing-rate gate rejects a non-closing target");
        check(atRestKill(proximityWarhead(10.0, 0.0), 100.0, 500),
              "zero min closing rate keeps legacy fuzing");
    }

    // ---- 4. Self-destruct ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0x5E1Fu);
        bool sawSelfDestruct = false;
        kernel.getEventSystem().subscribe([&](const SimulationEvent& e) {
            if (e.type == EventType::Detonation) sawSelfDestruct = e.selfDestruct;
        });
        VehicleConfig cfg = proximityWarhead(10.0, 0.0);
        cfg.warhead.selfDestructTimeSec = 0.2;
        const auto m = kernel.createVehicle(makeInit(0.0, 0.0, 100.0, 0.0, 0.0, 0.0), cfg);
        stepN(kernel, 300);
        check(!kernel.getStatus().isAlive[m] && sawSelfDestruct,
              "self-destruct timer destroys the carrier and flags the event");
        check(kernel.getWarhead(m).detonated, "self-destruct marks the warhead spent");

        SimulationKernel noTimer;
        const auto m2 = noTimer.createVehicle(makeInit(0.0, 0.0, 100.0, 0.0, 0.0, 0.0),
                                              proximityWarhead(10.0, 0.0));
        stepN(noTimer, 300);
        check(noTimer.getStatus().isAlive[m2] && !noTimer.getWarhead(m2).detonated,
              "zero self-destruct timer leaves the warhead armed (legacy)");
    }

    // ---- 5. Probabilistic fuze detection ----
    {
        VehicleConfig blind = proximityWarhead(10.0, 0.0);
        blind.warhead.fuseDetectionProbability = 0.0;
        SimulationKernel kernel;
        const auto m = kernel.createVehicle(makeInit(-5.0, 0.0, 100.0, 0.0, 0.0, 0.0), blind);
        const auto t = kernel.createVehicle(makeInit(0.0, 0.0, 100.0, 0.0, 0.0, 0.0, Allegiance::Hostile), ballistic());
        stepN(kernel, 500);
        check(!kernel.getWarhead(m).detonated && kernel.getStatus().isAlive[t],
              "zero fuze detection probability never fires despite being in range");
        check(atRestKill(proximityWarhead(10.0, 0.0), 100.0, 500),
              "unit detection probability keeps deterministic fuzing");
    }

    // ---- 6. Damage accumulation and structural hardness ----
    {
        VehicleConfig hit = proximityWarhead(10.0, 0.0);
        check(atRestKill(hit, 100.0, 200), "legacy hardness 100 + damage 100 kills");
        hit.warhead.damage = 40.0;
        check(!atRestKill(hit, 100.0, 200), "partial damage leaves a 100-hardness target alive");
        hit.warhead.damage = 100.0;
        check(!atRestKill(hit, 150.0, 200), "one hit does not kill a 150-hardness target");
        check(atRestKill(hit, 50.0, 200), "one hit kills a 50-hardness target");
    }

    // ---- 7. Aspect-dependent lethality ----
    {
        const auto aspectKill = [](double targetVx) {
            SimulationKernel kernel;
            kernel.setRandomSeed(0xA5B0u);
            VehicleConfig cfg = proximityWarhead(10.0, 30.0);
            cfg.warhead.cpaFuzingEnabled = true;
            cfg.warhead.headOnLethalityFactor = 2.5;
            cfg.warhead.tailOnLethalityFactor = 0.5;
            kernel.createVehicle(makeInit(0.0, 0.0, 100.0, 100.0, 0.0, 0.0), cfg);
            const auto t = kernel.createVehicle(
                makeInit(5.0, 20.0, 100.0, targetVx, 0.0, 0.0, Allegiance::Hostile), ballistic());
            stepN(kernel, 40);
            return !kernel.getStatus().isAlive[t];
        };
        check(aspectKill(-100.0), "head-on aspect widens the effective lethal band (kill at 20 m)");
        check(!aspectKill(100.0), "tail-on aspect halves the effective band (no kill at 20.6 m)");
    }

    // ---- 8. Detonation telemetry ----
    {
        SimulationKernel kernel;
        kernel.setRandomSeed(0x7E1Eu);
        SimulationEvent lastDet{};
        bool have = false;
        kernel.getEventSystem().subscribe([&](const SimulationEvent& e) {
            if (e.type == EventType::Detonation) { lastDet = e; have = true; }
        });
        VehicleConfig cfg = proximityWarhead(10.0, 30.0);
        cfg.warhead.cpaFuzingEnabled = true;
        // 2.5x head-on scaling puts the 20 m CPA strictly inside the
        // effective lethal radius so the kill probability is exactly 1.
        cfg.warhead.headOnLethalityFactor = 2.5;
        cfg.warhead.tailOnLethalityFactor = 0.5;
        const auto m = kernel.createVehicle(makeInit(0.0, 0.0, 100.0, 100.0, 0.0, 0.0), cfg);
        const auto t = kernel.createVehicle(
            makeInit(5.0, 20.0, 100.0, -100.0, 0.0, 0.0, Allegiance::Hostile), ballistic());
        stepN(kernel, 40);
        check(have && lastDet.targetEntityId == t,
              "Detonation names the engaged target");
        check(std::abs(lastDet.missDistanceM - 20.0) < 1e-6 &&
              std::abs(lastDet.predictedCpaM - 20.0) < 1e-6,
              "Detonation reports the projected CPA as the miss distance");
        check(lastDet.kill && lastDet.killProbability == 1.0,
              "Detonation reports the kill decision and probability");
        const auto& wh = kernel.getWarhead(m);
        check(wh.detonated && wh.lastKill && wh.lastTargetId == t &&
              std::abs(wh.lastMissDistanceM - 20.0) < 1e-6,
              "WarheadState exposes the last-detonation diagnostics");
    }

    // ---- 9. Kinetic-contact latch ----
    {
        const auto count = [](bool latch) {
            SimulationKernel kernel;
            EnvironmentConfig env;
            env.kineticImpactLatchEnabled = latch;
            kernel.setEnvironment(env);
            int n = 0;
            kernel.getEventSystem().subscribe([&](const SimulationEvent& e) {
                if (e.type == EventType::TargetImpact) ++n;
            });
            kernel.createVehicle(makeInit(0.0, 0.0, 100.0, 0.0, 0.0, 0.0), ballistic());
            kernel.createVehicle(makeInit(10.0, 0.0, 100.0, 0.0, 0.0, 0.0, Allegiance::Hostile), ballistic());
            stepN(kernel, 3, 0.01);
            return n;
        };
        check(count(false) == 3, "legacy kinetic contact reports every step in the band");
        check(count(true) == 1, "kinetic latch reports one TargetImpact per contact episode");
    }

    // ---- 10. Swept ground impact ----
    {
        const auto impact = [](bool swept, SimulationEvent* out) {
            SimulationKernel kernel;
            EnvironmentConfig env;
            env.terrainElevation = [](double x, double) {
                return std::abs(x) <= 1.0 ? 60.0 : 0.0;
            };
            env.sweptGroundImpactEnabled = swept;
            kernel.setEnvironment(env);
            if (out) {
                kernel.getEventSystem().subscribe([&](const SimulationEvent& e) {
                    if (e.type == EventType::GroundImpact) *out = e;
                });
            }
            const auto id = kernel.createVehicle(
                makeInit(-10.0, 0.0, 50.0, 1000.0, 0.0, 0.0), ballistic());
            kernel.step(0.02);
            return std::pair<bool, bool>{ !kernel.getStatus().isAlive[id],
                                          !kernel.getPhysics().active[id] };
        };
        check(!impact(false, nullptr).first,
              "legacy endpoint crossing test flies over a mid-segment ridge");
        SimulationEvent evt{};
        const auto sweptResult = impact(true, &evt);
        check(sweptResult.first && sweptResult.second,
              "swept crossing detects the ridge hit");
        check(evt.customCode == 2 && std::abs(evt.terrainElevationM - 60.0) < 1e-9,
              "ridge impact carries the crash code and ridge elevation");
    }

    // ---- 11. Ground-impact rate zeroing ----
    {
        const auto run = [](bool zeroRates) {
            SimulationKernel kernel;
            EnvironmentConfig env;
            env.groundImpactZeroRates = zeroRates;
            kernel.setEnvironment(env);
            VehicleInitState init = makeInit(0.0, 0.0, 0.05, 0.0, 0.0, -10.0);
            init.wx = 1.0;
            const auto id = kernel.createVehicle(init, ballistic());
            stepN(kernel, 1, 0.01);
            return std::pair<bool, double>{ !kernel.getStatus().isAlive[id],
                                            kernel.getPhysics().wx[id] };
        };
        const auto legacy = run(false);
        const auto zeroed = run(true);
        check(legacy.first && std::abs(legacy.second) > 0.0,
              "legacy ground impact leaves the body angular rate untouched");
        check(zeroed.first && zeroed.second == 0.0,
              "groundImpactZeroRates zeroes the body angular state");
    }

    // ---- 12. Config serialization and profile parsing ----
    {
        VehicleConfig cfg;
        cfg.structuralHardness = 250.0;
        cfg.warhead.fusing = FusingType::Proximity;
        cfg.warhead.proximityTriggerM = 50.0;
        cfg.warhead.lethalRadiusM = 10.0;
        cfg.warhead.falloffRadiusM = 30.0;
        cfg.warhead.fuseEnabled = false;
        cfg.warhead.cpaFuzingEnabled = true;
        cfg.warhead.fuseLookaheadSec = 0.03;
        cfg.warhead.armingDelaySec = 0.7;
        cfg.warhead.minClosingSpeedMps = 120.0;
        cfg.warhead.selfDestructTimeSec = 40.0;
        cfg.warhead.damage = 55.0;
        cfg.warhead.fuseDetectionProbability = 0.8;
        cfg.warhead.headOnLethalityFactor = 1.4;
        cfg.warhead.tailOnLethalityFactor = 0.6;
        const VehicleConfig back = deserializeVehicleConfig(serializeVehicleConfig(cfg));
        const auto& w = back.warhead;
        check(back.structuralHardness == 250.0, "structural hardness round-trips");
        check(w.fuseEnabled == false && w.cpaFuzingEnabled && w.fuseLookaheadSec == 0.03 &&
              w.armingDelaySec == 0.7 && w.minClosingSpeedMps == 120.0 &&
              w.selfDestructTimeSec == 40.0,
              "fuse timing keys round-trip");
        check(w.damage == 55.0 && w.fuseDetectionProbability == 0.8 &&
              w.headOnLethalityFactor == 1.4 && w.tailOnLethalityFactor == 0.6,
              "damage/detection/aspect keys round-trip");

        const std::string path = "warhead_fidelity_profile.json";
        {
            std::FILE* f = std::fopen(path.c_str(), "w");
            check(f != nullptr, "profile fixture opens");
            if (f) {
                std::fputs(R"({"fusing":"proximity","proximity_trigger_m":50.0,)"
                           R"("lethal_radius_m":10.0,"falloff_radius_m":30.0,)"
                           R"("cpa_fuzing_enabled":true,"arming_delay_sec":0.7,)"
                           R"("head_on_lethality_factor":1.4})", f);
                std::fclose(f);
            }
        }
        WarheadProfileDatabase db;
        const bool loaded = db.loadProfile(path);
        check(loaded, "WarheadProfileDatabase loads the new keys");
        if (loaded) {
            const auto& pw = db.warhead();
            check(pw.cpaFuzingEnabled && pw.armingDelaySec == 0.7 &&
                  pw.headOnLethalityFactor == 1.4 && pw.fuseEnabled &&
                  pw.selfDestructTimeSec == 0.0 && pw.damage == 100.0,
                  "profile parser applies new keys and legacy defaults");
        }
        std::remove(path.c_str());
    }

    // ---- 13. Fuse validation (explicit-disable footgun) ----
    {
        bool threw = false;
        try {
            SimulationKernel kernel;
            VehicleConfig cfg;
            cfg.warhead.fusing = FusingType::Proximity;
            cfg.warhead.proximityTriggerM = 0.0;
            cfg.warhead.lethalRadiusM = 10.0;
            kernel.createVehicle(makeInit(0.0, 0.0, 100.0, 0.0, 0.0, 0.0), cfg);
        } catch (const std::runtime_error&) { threw = true; }
        check(threw, "enabled proximity fuse with trigger <= 0 throws at createVehicle");

        bool threwDisabled = false;
        try {
            SimulationKernel kernel;
            VehicleConfig cfg;
            cfg.warhead.fusing = FusingType::Proximity;
            cfg.warhead.proximityTriggerM = 0.0;
            cfg.warhead.lethalRadiusM = 10.0;
            cfg.warhead.fuseEnabled = false;
            kernel.createVehicle(makeInit(0.0, 0.0, 100.0, 0.0, 0.0, 0.0), cfg);
        } catch (const std::exception&) { threwDisabled = true; }
        check(!threwDisabled, "explicitly disabled fuse allows trigger <= 0");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
