// Mathematical PN/APN + body-frame signs (guidance law layer) and the W36
// guidance-system state machine: phase selection (Midcourse -> Acquisition ->
// Terminal), acquisition->terminal blend weighting, explicit invalid and
// non-closing diagnostics, target-accelerations feed-forward availability,
// and tgo output.
#include <strikeengine/kernel/systems/GuidanceSystem.hpp>
#include <strikeengine/kernel/systems/CommandProcessor.hpp>
#include <strikeengine/models/guidance/GuidanceModels.hpp>

#include <cmath>
#include <cstdio>
#include <limits>

using namespace StrikeEngine::Kernel;

namespace {

int failures = 0;
void check(bool ok, const char* message)
{
    if (ok) std::printf("  [PASS] %s\n", message);
    else { std::printf("  [FAIL] %s\n", message); ++failures; }
}

// Minimal sized guidance block with the usual test defaults.
GuidanceBlock makeBlock()
{
    GuidanceBlock g;
    g.mode = {GuidanceMode::None};
    g.targetX = {0.0}; g.targetY = {0.0}; g.targetZ = {0.0};
    g.targetVx = {0.0}; g.targetVy = {0.0}; g.targetVz = {0.0};
    g.targetAccelX = {0.0}; g.targetAccelY = {0.0}; g.targetAccelZ = {0.0};
    g.targetAccelAvailable = {false};
    g.maxAccel = {0.0};
    g.navigationConstant = {3.5};
    g.waypointGain = {20.0};
    g.handoffBlendTimeSec = {0.0};
    g.lockLossRetentionSec = {0.0};
    g.apnFeedforwardEnabled = {false};
    g.commandedAccelX = {0.0}; g.commandedAccelY = {0.0}; g.commandedAccelZ = {0.0};
    g.phase = {GuidancePhase::None};
    g.law = {GuidanceLaw::None};
    g.trackId = {-1};
    g.trackAgeSec = {0.0};
    g.handoffWeight = {0.0};
    g.lockLossCount = {0};
    g.rawAccelX = {0.0}; g.rawAccelY = {0.0}; g.rawAccelZ = {0.0};
    g.limitedByMaxAccel = {false};
    g.lawInvalid = {false};
    g.nonClosing = {false};
    g.tgoSec = {0.0};
    g.retainedAccelX = {0.0};
    g.retainedAccelY = {0.0};
    g.retainedAccelZ = {0.0};
    g.trajectoryMinSpeedMps = {30.0};
    g.trajectoryFeasibilityAccelFactor = {0.95};
    g.predictedInterceptX = {0.0}; g.predictedInterceptY = {0.0}; g.predictedInterceptZ = {0.0};
    g.predictedTgoSec = {0.0};
    g.trajectoryRequiredAccel = {0.0};
    g.trajectoryAimSource = {GuidanceAimSource::None};
    g.trajectoryFeasible = {false};
    g.trajectoryReason = {TrajectoryReason::None};
    return g;
}

} // namespace

int main()
{
    std::printf("=== guidance_test: PN, APN, moving target, phases, and handoff ===\n");

    // ---- Pure math: PN geometry cases --------------------------------------
    {
        const StrikeEngine::Models::Vec3 r{1000.0, 0.0, 0.0};
        // Head-on: zero LOS rate -> zero commanded acceleration.
        const auto headOn = StrikeEngine::Models::proportionalNavigation(
            r, StrikeEngine::Models::Vec3{-100.0, 0.0, 0.0}, 3.0);
        check(headOn.valid && std::abs(headOn.acceleration[1]) < 1e-12 &&
                  std::abs(headOn.acceleration[0]) < 1e-12,
              "head-on geometry: PN commands zero acceleration");
        // Crossing: lateral acceleration proportional to N*Vc*LOS rate.
        const auto crossing = StrikeEngine::Models::proportionalNavigation(
            r, StrikeEngine::Models::Vec3{-100.0, 10.0, 0.0}, 3.0);
        check(crossing.valid && std::abs(crossing.closingSpeed - 100.0) < 1e-12 &&
                  std::abs(crossing.acceleration[1] - 3.0) < 1e-12 &&
                  std::abs(crossing.acceleration[0]) < 1e-12,
              "crossing geometry: PN normal to LOS with expected magnitude");
        // Non-closing: explicit invalid status.
        const auto receding = StrikeEngine::Models::proportionalNavigation(
            StrikeEngine::Models::Vec3{-1000.0, 0.0, 0.0},
            StrikeEngine::Models::Vec3{-100.0, 0.0, 0.0}, 3.0);
        check(!receding.valid, "non-closing geometry: PN reports invalid");

        const auto apn = StrikeEngine::Models::augmentedProportionalNavigation(
            r, StrikeEngine::Models::Vec3{-100.0, 0.0, 0.0},
            StrikeEngine::Models::Vec3{0.0, 2.0, 0.0}, 3.5);
        check(apn.valid && std::abs(apn.acceleration[1] - 3.5) < 1e-12,
              "APN adds the normal target-acceleration feed-forward term");
    }

    // ---- Kernel path: state machine ----------------------------------------
    NavigationBlock nav;
    nav.size = 1;
    nav.estPx = {0.0}; nav.estPy = {0.0}; nav.estPz = {0.0};
    nav.estVx = {100.0}; nav.estVy = {0.0}; nav.estVz = {0.0};
    nav.estQw = {1.0}; nav.estQx = {0.0}; nav.estQy = {0.0}; nav.estQz = {0.0};
    nav.estWx = {0.0}; nav.estWy = {0.0}; nav.estWz = {0.0};

    EntityStatusBlock status;
    status.size = 1;
    status.isAlive = {true};

    SeekerBlock seeker;
    seeker.size = 1;
    seeker.type = {SeekerType::None};
    seeker.isLocked = {false};
    seeker.lockedTargetId = {0};
    seeker.targetRange = {0.0};
    seeker.targetRangeRate = {0.0};
    seeker.targetAzimuth = {0.0};
    seeker.targetElevation = {0.0};
    seeker.targetAzimuthRate = {0.0};
    seeker.targetElevationRate = {0.0};

    ControlBlock control;
    TrackBlock tracks;   // empty: guidance falls back to the external command aim
    GuidanceSystem system;
    StrikeEngine::Kernel::EnvironmentConfig env;

    // --- Midcourse PN with diagnostics -------------------------------------
    {
        GuidanceBlock guidance = makeBlock();
        guidance.mode = {GuidanceMode::ProportionalNavigation};
        guidance.targetX = {1000.0}; guidance.targetY = {0.0}; guidance.targetZ = {0.0};
        guidance.targetVx = {0.0}; guidance.targetVy = {10.0}; guidance.targetVz = {0.0};
        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        check(std::abs(guidance.commandedAccelY[0] - 3.5) < 1e-12,
              "kernel PN mode applies the configured navigation constant");
        check(guidance.phase[0] == GuidancePhase::Midcourse &&
                  guidance.law[0] == GuidanceLaw::PureProNav,
              "PN sets phase Midcourse and law PureProNav");
        check(std::abs(guidance.tgoSec[0] - 1000.0 / 100.0) < 1e-6,
              "PN publishes tgo = range / closing speed");
        check(!guidance.lawInvalid[0] && !guidance.nonClosing[0],
              "valid geometry: no invalid/non-closing flags");
    }

    // --- Non-closing geometry: explicit diagnostics -------------------------
    {
        GuidanceBlock guidance = makeBlock();
        guidance.mode = {GuidanceMode::ProportionalNavigation};
        guidance.targetX = {-1000.0};  // target behind the interceptor
        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        check(guidance.lawInvalid[0] == false && guidance.nonClosing[0],
              "receding target: nonClosing diagnostics set, demand zeroed");
        check(std::abs(guidance.commandedAccelY[0]) < 1e-12,
              "non-closing target: zero command");
    }

    // --- Non-finite input: explicit invalid flag ----------------------------
    {
        GuidanceBlock guidance = makeBlock();
        guidance.mode = {GuidanceMode::ProportionalNavigation};
        guidance.targetX = {std::numeric_limits<double>::infinity()};
        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        check(guidance.lawInvalid[0] && !guidance.nonClosing[0],
              "non-finite target input: lawInvalid set");
    }

    // --- APN feed-forward availability --------------------------------------
    {
        GuidanceBlock guidance = makeBlock();
        guidance.mode = {GuidanceMode::ProportionalNavigation};
        guidance.targetX = {1000.0}; guidance.targetY = {0.0}; guidance.targetZ = {0.0};
        guidance.targetVx = {0.0}; guidance.targetVy = {10.0}; guidance.targetVz = {0.0};
        guidance.apnFeedforwardEnabled = {true};
        guidance.targetAccelAvailable = {true};
        guidance.targetAccelY = {2.0};
        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        check(std::abs(guidance.commandedAccelY[0] - 7.0) < 1e-12,
              "augmented APN: 0.5*N*a_t_perp added when available+enabled");
        check(guidance.law[0] == GuidanceLaw::AugmentedProNav,
              "law reports AugmentedProNav when feed-forward active");

        // Availability false -> explicit fallback to pure PN.
        GuidanceBlock g2 = makeBlock();
        g2.mode = {GuidanceMode::ProportionalNavigation};
        g2.targetX = {1000.0}; g2.targetY = {0.0}; g2.targetZ = {0.0};
        g2.targetVx = {0.0}; g2.targetVy = {10.0}; g2.targetVz = {0.0};
        g2.apnFeedforwardEnabled = {true};
        g2.targetAccelAvailable = {false};
        g2.targetAccelY = {2.0};
        system.update(status, nav, seeker, tracks, g2, control, 0.01, env);
        check(std::abs(g2.commandedAccelY[0] - 3.5) < 1e-12 &&
                  g2.law[0] == GuidanceLaw::PureProNav,
              "feed-forward unavailable: pure PN, never reads uninitialized accel");
    }

    // --- Seeker APN body-frame signs (legacy contract) ----------------------
    {
        GuidanceBlock guidance = makeBlock();
        guidance.mode = {GuidanceMode::ProportionalNavigation};
        guidance.targetX = {1000.0}; guidance.targetY = {0.0}; guidance.targetZ = {0.0};
        seeker.type = {SeekerType::RF};
        seeker.isLocked = {true};
        seeker.lockedTargetId = {0};
        seeker.targetRange = {900.0};
        seeker.targetRangeRate = {-100.0};
        seeker.targetAzimuthRate = {0.02};
        seeker.targetElevationRate = {0.0};
        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        check(std::abs(guidance.commandedAccelY[0] - 7.0) < 1e-12 &&
                  std::abs(guidance.commandedAccelZ[0]) < 1e-12 &&
                  guidance.phase[0] == GuidancePhase::Terminal &&
                  guidance.law[0] == GuidanceLaw::SeekerRateAPN &&
                  std::abs(guidance.handoffWeight[0] - 1.0) < 1e-12,
              "azimuth rate -> +body-Y APN; instant handoff gives Terminal+full weight");

        seeker.targetAzimuthRate = {0.0};
        seeker.targetElevationRate = {0.02};
        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        check(std::abs(guidance.commandedAccelY[0]) < 1e-12 &&
                  std::abs(guidance.commandedAccelZ[0] + 7.0) < 1e-12,
              "elevation rate -> upward (-body-Z) APN acceleration");
    }

    // --- Seeker APN with target-acceleration feedforward (true APN) ----------
    {
        GuidanceBlock guidance = makeBlock();
        guidance.mode = {GuidanceMode::ProportionalNavigation};
        guidance.targetX = {1000.0}; guidance.targetY = {0.0}; guidance.targetZ = {0.0};
        guidance.apnFeedforwardEnabled = {true};
        guidance.targetAccelAvailable = {true};
        guidance.targetAccelX = {0.0};
        guidance.targetAccelY = {4.0}; // +4 m/s^2 along body Y (normal to LOS along X)
        guidance.targetAccelZ = {0.0};

        seeker.type = {SeekerType::RF};
        seeker.isLocked = {true};
        seeker.lockedTargetId = {0};
        seeker.targetRange = {900.0};
        seeker.targetRangeRate = {-100.0};
        seeker.targetAzimuth = {0.0};
        seeker.targetElevation = {0.0};
        seeker.targetAzimuthRate = {0.02};
        seeker.targetElevationRate = {0.0};

        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        // Base PN: N * Vc * dAz = 3.5 * 100 * 0.02 = 7.0
        // APN feedforward: 0.5 * N * aT_perp = 0.5 * 3.5 * 4.0 = 7.0
        // Total commandedAccelY = 7.0 + 7.0 = 14.0
        check(std::abs(guidance.commandedAccelY[0] - 14.0) < 1e-12,
              "true APN: adds 0.5*N*a_T_perp target acceleration feedforward in terminal seeker APN");
        check(guidance.law[0] == GuidanceLaw::SeekerRateAPN,
              "seeker APN reports SeekerRateAPN guidance law");
    }

    // --- Acquisition blend ramp ----------------------------------------------
    {
        GuidanceBlock guidance = makeBlock();
        guidance.mode = {GuidanceMode::ProportionalNavigation};
        guidance.targetX = {1000.0}; guidance.targetY = {0.0}; guidance.targetZ = {0.0};
        guidance.targetVx = {0.0}; guidance.targetVy = {10.0}; guidance.targetVz = {0.0};
        guidance.handoffBlendTimeSec = {0.5};
        seeker.isLocked = {true};
        seeker.lockedTargetId = {0};
        seeker.targetRange = {900.0};
        seeker.targetRangeRate = {-100.0};
        seeker.targetAzimuthRate = {0.02};
        seeker.targetElevationRate = {0.0};

        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        check(guidance.phase[0] == GuidancePhase::Acquisition,
              "new lock with blend time starts in Acquisition");
        const double w1 = guidance.handoffWeight[0];
        check(w1 > 0.0 && w1 < 1.0, "blend weight strictly inside (0,1)");
        // 50 steps at dt=0.01 -> weight ramps to 1.0 and phase -> Terminal.
        for (int s = 0; s < 50; ++s) {
            system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        }
        check(std::abs(guidance.handoffWeight[0] - 1.0) < 1e-12 &&
                  guidance.phase[0] == GuidancePhase::Terminal,
              "blend weight ramps to 1 and phase becomes Terminal");
        // During Acquisition the demand blended PN + APN: azimuth 0.02, N 3.5,
        // Vc 100 -> pure APN ay = 7; PN(1000,0,0; v 0,10,0) ay = 3.5.
        check(guidance.commandedAccelY[0] >= 3.475 &&
                  guidance.commandedAccelY[0] <= 7.0,
              "acquisition demand blends midcourse PN and terminal APN");
    }

    // --- Lock-loss retention: bounded predicted terminal command -------------
    {
        GuidanceBlock guidance = makeBlock();
        guidance.mode = {GuidanceMode::ProportionalNavigation};
        guidance.targetX = {1000.0}; guidance.targetY = {0.0}; guidance.targetZ = {0.0};
        guidance.targetVx = {0.0}; guidance.targetVy = {10.0}; guidance.targetVz = {0.0};
        guidance.handoffBlendTimeSec = {0.0};       // instant handoff
        guidance.lockLossRetentionSec = {0.2};      // 0.2 s retention window
        seeker.isLocked = {true};
        seeker.lockedTargetId = {0};
        seeker.targetRange = {900.0};
        seeker.targetRangeRate = {-100.0};
        seeker.targetAzimuthRate = {0.02};
        seeker.targetElevationRate = {0.0};

        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        check(guidance.phase[0] == GuidancePhase::Terminal &&
                  std::abs(guidance.commandedAccelY[0] - 7.0) < 1e-12,
              "locked terminal step commands APN demand");

        // Drop the lock: within the retention window the bounded predicted
        // terminal command (the last valid demand) is applied, phase Terminal.
        seeker.isLocked = {false};
        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        check(guidance.phase[0] == GuidancePhase::Terminal &&
                  guidance.law[0] == GuidanceLaw::SeekerRateAPN &&
                  std::abs(guidance.commandedAccelY[0] - 7.0) < 1e-12,
              "retention window applies the bounded retained terminal command");
        // 18 more unlocked steps reach trackAge 0.19 (inside the 0.2 s window).
        for (int s = 0; s < 18; ++s) {
            system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        }
        check(guidance.phase[0] == GuidancePhase::Terminal,
              "retention holds across the configured window");
        check(guidance.lockLossCount[0] == 1,
              "lock loss counted exactly once");
        // Two more steps: retention expires -> LostTrack recovery via PN.
        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        check(guidance.phase[0] == GuidancePhase::LostTrack &&
                  guidance.law[0] == GuidanceLaw::PureProNav &&
                  std::abs(guidance.commandedAccelY[0] - 3.5) < 1e-12,
              "after retention expiry: LostTrack with midcourse PN recovery");

        // Reacquisition: a fresh lock restarts the blend and reaches Terminal.
        seeker.isLocked = {true};
        guidance.handoffBlendTimeSec = {0.5};
        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        check(guidance.phase[0] == GuidancePhase::Acquisition &&
                  guidance.handoffWeight[0] > 0.0 && guidance.handoffWeight[0] < 1.0,
              "reacquisition restarts the acquisition blend");
        for (int s = 0; s < 50; ++s) {
            system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        }
        check(guidance.phase[0] == GuidancePhase::Terminal,
              "reacquisition blend completes to Terminal");
    }

    // --- Waypoint law: explicit law + non-finite hardening -------------------
    {
        GuidanceBlock guidance = makeBlock();
        guidance.mode = {GuidanceMode::Waypoint};
        guidance.targetX = {500.0}; guidance.targetY = {0.0}; guidance.targetZ = {0.0};
        seeker.type = {SeekerType::None};
        seeker.isLocked = {false};
        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        check(guidance.law[0] == GuidanceLaw::Waypoint &&
                  std::abs(guidance.commandedAccelX[0] - 20.0) < 1e-9 &&
                  guidance.phase[0] == GuidancePhase::Midcourse,
              "waypoint reports GuidanceLaw::Waypoint and the point-seeking demand");

        GuidanceBlock g2 = makeBlock();
        g2.mode = {GuidanceMode::Waypoint};
        g2.targetX = {std::numeric_limits<double>::infinity()};
        system.update(status, nav, seeker, tracks, g2, control, 0.01, env);
        check(g2.lawInvalid[0] && std::abs(g2.commandedAccelX[0]) < 1e-12,
              "non-finite waypoint geometry: lawInvalid set, demand zeroed");
    }

    // --- Seeker APN: non-finite range must not produce a NaN tgo -------------
    {
        GuidanceBlock guidance = makeBlock();
        seeker.type = {SeekerType::RF};
        seeker.isLocked = {true};
        seeker.lockedTargetId = {0};
        seeker.targetRange = {std::numeric_limits<double>::infinity()};
        seeker.targetRangeRate = {-100.0};
        seeker.targetAzimuthRate = {0.0};
        seeker.targetElevationRate = {0.0};
        system.update(status, nav, seeker, tracks, guidance, control, 0.01, env);
        check(guidance.lawInvalid[0] && std::isfinite(guidance.tgoSec[0]) &&
                  std::abs(guidance.commandedAccelY[0]) < 1e-12,
              "non-finite seeker range: lawInvalid set, finite tgo, zero demand");
    }

    // --- Public command input for target acceleration (augmented APN) --------
    {
        GuidanceBlock guidance = makeBlock();
        CommandProcessor processor;
        SimulationCommand cmd{};
        cmd.entityId = 0;
        cmd.mode = GuidanceMode::ProportionalNavigation;
        cmd.targetX = 1000.0;
        cmd.targetAccelY = 2.0;
        cmd.targetAccelAvailable = true;
        processor.enqueueCommand(cmd);
        processor.process(guidance, tracks, 0.0);
        check(guidance.targetAccelY[0] == 2.0 && guidance.targetAccelAvailable[0],
              "SimulationCommand target acceleration reaches the guidance block");

        GuidanceBlock g2 = makeBlock();
        SimulationCommand cmd2{};
        cmd2.entityId = 0;
        cmd2.mode = GuidanceMode::ProportionalNavigation;
        processor.enqueueCommand(cmd2);
        processor.process(g2, tracks, 0.0);
        check(!g2.targetAccelAvailable[0] && g2.targetAccelY[0] == 0.0,
              "default command leaves feed-forward explicitly unavailable");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
