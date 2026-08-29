// W40 trajectory-aware midcourse guidance verification.
//
// Covers the trajectory core over the W39 persistent target track:
//   - Models::predictIntercept: closed-form constant-velocity intercept
//     (PIP + tgo + required accel), target-accel term, velocity-low,
//     receding/no-intercept, and non-finite rejection
//   - GuidanceMode::Trajectory on a measurement-anchored track: law/phase,
//     aim-source tracking, prediction + feasibility diagnostics, PN demand
//     aimed at the predicted intercept point
//   - feasibility vs the maxAccel (energy) budget: AccelLimited + clamp
//   - command-aim fallback (Lost track / bare command seed)
//   - dropout via the track lifecycle: Maintain -> Coast (predictions keep
//     streaming) -> Lost (command aim) -> Reacquire -> Maintain (track aim)
//   - midcourse-only precedence: a seeker lock still overrides trajectory
//     with acquisition/terminal APN, and the trajectory diagnostics reset
//   - legacy parity: ProportionalNavigation is untouched by the new code
//   - determinism: repeated evaluation yields identical results
//
// Guidance never reads physics truth: the predictor uses only the navigation
// estimate plus the command/track aim state.
#include <strikeengine/kernel/systems/TrackManagerSystem.hpp>
#include <strikeengine/kernel/systems/GuidanceSystem.hpp>
#include <strikeengine/kernel/data/TrackBlock.hpp>
#include <strikeengine/models/guidance/GuidanceModels.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace StrikeEngine::Kernel;

namespace {

int failures = 0;
void check(bool ok, const char* message)
{
    if (ok) std::printf("  [PASS] %s\n", message);
    else { std::printf("  [FAIL] %s\n", message); ++failures; }
}

bool near(double a, double b, double tol = 1e-6) { return std::abs(a - b) < tol; }

// Fully sized single-entity track block with the default W39 config.
TrackBlock makeTracks()
{
    TrackBlock t;
    t.size = 1;
    t.confirmations = {3};
    t.coastTimeoutSec = {0.5};
    t.lossTimeoutSec = {2.0};
    t.state = {TrackState::None};
    t.trackId = {-1};
    t.posX = {0.0}; t.posY = {0.0}; t.posZ = {0.0};
    t.velX = {0.0}; t.velY = {0.0}; t.velZ = {0.0};
    t.accelX = {0.0}; t.accelY = {0.0}; t.accelZ = {0.0};
    t.accelAvailable = {false};
    t.timestampSec = {0.0};
    t.ageSec = {0.0};
    t.positionStdM = {5.0};
    t.velocityStdMs = {25.0};
    t.quality01 = {0.0};
    t.updateCount = {0};
    t.dropoutCount = {0};
    t.measPosX = {0.0}; t.measPosY = {0.0}; t.measPosZ = {0.0};
    t.measTimeSec = {0.0};
    return t;
}

// Fully sized guidance block (Trajectory mode by default; caller adjusts).
GuidanceBlock makeGuidance()
{
    GuidanceBlock g;
    g.mode = {GuidanceMode::Trajectory};
    g.targetX = {0.0}; g.targetY = {0.0}; g.targetZ = {0.0};
    g.targetVx = {0.0}; g.targetVy = {0.0}; g.targetVz = {0.0};
    g.targetAccelX = {0.0}; g.targetAccelY = {0.0}; g.targetAccelZ = {0.0};
    g.targetAccelAvailable = {false};
    g.maxAccel = {0.0};
    g.navigationConstant = {4.0};
    g.waypointGain = {20.0};
    g.handoffBlendTimeSec = {0.0};
    g.lockLossRetentionSec = {0.0};
    g.apnFeedforwardEnabled = {false};
    g.trajectoryMinSpeedMps = {30.0};
    g.trajectoryFeasibilityAccelFactor = {0.95};
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
    g.retainedAccelX = {0.0}; g.retainedAccelY = {0.0}; g.retainedAccelZ = {0.0};
    g.predictedInterceptX = {0.0}; g.predictedInterceptY = {0.0}; g.predictedInterceptZ = {0.0};
    g.predictedTgoSec = {0.0};
    g.trajectoryRequiredAccel = {0.0};
    g.trajectoryAimSource = {GuidanceAimSource::None};
    g.trajectoryFeasible = {false};
    g.trajectoryReason = {TrajectoryReason::None};
    return g;
}

// Shared identity-attitude navigation block: entity 0 at the origin moving
// +x at 100 m/s.
NavigationBlock makeNav()
{
    NavigationBlock nav;
    nav.size = 1;
    nav.estPx = {0.0}; nav.estPy = {0.0}; nav.estPz = {0.0};
    nav.estVx = {100.0}; nav.estVy = {0.0}; nav.estVz = {0.0};
    nav.estQw = {1.0}; nav.estQx = {0.0}; nav.estQy = {0.0}; nav.estQz = {0.0};
    nav.estWx = {0.0}; nav.estWy = {0.0}; nav.estWz = {0.0};
    return nav;
}

TrackState trackState(const TrackBlock& t) { return t.state[0]; }

} // namespace

int main()
{
    std::printf("=== trajectory_test: trajectory-aware midcourse guidance ===\n");

    // ---- 1. Models::predictIntercept: closed-form math --------------------
    std::printf("\n-- predictIntercept geometry --\n");
    {
        namespace M = StrikeEngine::Models;
        const M::Vec3 ownP{0.0, 0.0, 0.0};
        const M::Vec3 ownV{100.0, 0.0, 0.0};
        const M::Vec3 zip{0.0, 0.0, 0.0};

        // Head-on: own 100 m/s +x, target 1000 m ahead closing at 100 m/s.
        // The constant-speed intercept solves for the earliest meeting time
        // along straight-line headings: tgo=3.3333 s, PIP at x=666.67.
        {
            const M::InterceptResult r = M::predictIntercept(
                ownP, ownV, {1000.0, 0.0, 0.0}, {-100.0, 0.0, 0.0}, zip,
                false, 4.0, 30.0);
            check(r.valid && r.status == M::InterceptStatus::Ok,
                  "head-on resolves a valid intercept");
            check(near(r.tgoSec, 3.3333333333333335, 1e-6),
                  "head-on tgo = 3.3333 s");
            check(near(r.pip[0], 666.6666666666666, 1e-4) &&
                      near(r.pip[1], 0.0, 1e-9) && near(r.pip[2], 0.0, 1e-9),
                  "head-on PIP = (666.67, 0, 0)");
            check(near(r.requiredAccel, 0.0, 1e-6),
                  "head-on requires zero lateral accel");
        }

        // Crossing offset: target (1000,100,0) moving -x. Closed-form
        // tgo/PIP/required-accel from the same constant-speed model.
        {
            const M::InterceptResult r = M::predictIntercept(
                ownP, ownV, {1000.0, 100.0, 0.0}, {-100.0, 0.0, 0.0}, zip,
                false, 4.0, 30.0);
            check(r.valid && r.status == M::InterceptStatus::Ok,
                  "offset crossing resolves a valid intercept");
            check(near(r.tgoSec, 3.383714066067965, 1e-6),
                  "offset crossing tgo = 3.38371 s");
            check(near(r.pip[0], 661.6285933932035, 1e-4) &&
                      near(r.pip[1], 100.0, 1e-9) && near(r.pip[2], 0.0, 1e-9),
                  "offset crossing PIP = (661.63, 100, 0)");
            check(near(r.requiredAccel, 35.33274435906668, 1e-3),
                  "offset crossing required accel = 35.33 m/s^2");
        }

        // Target acceleration bends the PIP downstream (+y here).
        {
            const M::InterceptResult base = M::predictIntercept(
                ownP, ownV, {1000.0, 100.0, 0.0}, {-100.0, 0.0, 0.0}, zip,
                false, 4.0, 30.0);
            const M::InterceptResult acc = M::predictIntercept(
                ownP, ownV, {1000.0, 100.0, 0.0}, {-100.0, 0.0, 0.0},
                {0.0, 20.0, 0.0}, true, 4.0, 30.0);
            check(acc.valid && near(acc.tgoSec, base.tgoSec, 1e-9),
                  "target accel keeps the same tgo");
            check(near(acc.pip[1], 100.0 + 0.5 * 20.0 * acc.tgoSec * acc.tgoSec, 1e-4),
                  "target accel shifts PIP by 0.5*a*t^2");
            check(acc.requiredAccel > base.requiredAccel,
                  "target accel increases the required lateral accel");
        }

        // Velocity-low: own est speed below the floor.
        {
            const M::InterceptResult r = M::predictIntercept(
                {0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {1000.0, 0.0, 0.0},
                {-100.0, 0.0, 0.0}, zip, false, 4.0, 30.0);
            check(!r.valid && r.status == M::InterceptStatus::VelocityLow,
                  "own speed below the floor -> VelocityLow");
        }

        // Receding target: no positive-time intercept.
        {
            const M::InterceptResult r = M::predictIntercept(
                ownP, ownV, {100.0, 0.0, 0.0}, {200.0, 0.0, 0.0}, zip,
                false, 4.0, 30.0);
            check(!r.valid && r.status == M::InterceptStatus::NoIntercept,
                  "receding target -> NoIntercept");
        }

        // Non-finite input and non-positive navigation constant.
        {
            const M::InterceptResult r = M::predictIntercept(
                ownP, ownV, {std::nan(""), 0.0, 0.0}, {-100.0, 0.0, 0.0}, zip,
                false, 4.0, 30.0);
            check(!r.valid && r.status == M::InterceptStatus::NonFinite,
                  "non-finite aim -> NonFinite");
            const M::InterceptResult n = M::predictIntercept(
                ownP, ownV, {1000.0, 0.0, 0.0}, {-100.0, 0.0, 0.0}, zip,
                false, 0.0, 30.0);
            check(!n.valid && n.status == M::InterceptStatus::NonFinite,
                  "non-positive navigation constant -> NonFinite");
        }
    }

    // ---- 2. Trajectory mode on a measurement-anchored track ---------------
    std::printf("\n-- trajectory mode on track aim --\n");
    {
        NavigationBlock nav = makeNav();
        GuidanceBlock g = makeGuidance();
        TrackBlock tracks = makeTracks();
        // Measurement-anchored Maintain track (the offset-crossing geometry).
        tracks.state = {TrackState::Maintain};
        tracks.updateCount = {5};
        tracks.trackId = {3};
        tracks.posX = {1000.0}; tracks.posY = {100.0}; tracks.posZ = {0.0};
        tracks.velX = {-100.0}; tracks.velY = {0.0}; tracks.velZ = {0.0};
        tracks.accelAvailable = {false};

        SeekerBlock seeker;
        seeker.size = 1;
        seeker.type = {SeekerType::None};
        seeker.isLocked = {false};
        EntityStatusBlock status;
        status.size = 1;
        status.isAlive = {true};
        ControlBlock control;
        GuidanceSystem system;
        system.update(status, nav, seeker, tracks, g, control, 0.01);

        check(g.law[0] == GuidanceLaw::Trajectory && g.phase[0] == GuidancePhase::Midcourse,
              "Trajectory mode publishes law Trajectory + phase Midcourse");
        check(g.trajectoryAimSource[0] == GuidanceAimSource::Track,
              "measurement-anchored track is the aim source");
        check(g.trajectoryFeasible[0] && g.trajectoryReason[0] == TrajectoryReason::Ok,
              "predicted intercept is feasible (unlimited budget)");
        check(near(g.predictedInterceptX[0], 661.6285933932035, 1e-4) &&
                  near(g.predictedInterceptY[0], 100.0, 1e-9),
              "PIP published from the track estimate");
        check(near(g.predictedTgoSec[0], 3.383714066067965, 1e-6),
              "predicted tgo published");
        check(near(g.trajectoryRequiredAccel[0], 35.33274435906668, 1e-3),
              "required accel published");
        check(near(g.commandedAccelX[0], -5.2802977443692924, 1e-4) &&
                  near(g.commandedAccelY[0], 34.9359596930436, 1e-4) &&
                  near(g.commandedAccelZ[0], 0.0, 1e-9),
              "demand is PN aimed at the predicted intercept point");
        check(!g.limitedByMaxAccel[0] && !g.lawInvalid[0] && !g.nonClosing[0],
              "no limit/invalid/non-closing flags for a feasible intercept");
    }

    // ---- 3. Feasibility vs the maxAccel (energy) budget --------------------
    std::printf("\n-- feasibility budget (AccelLimited) --\n");
    {
        NavigationBlock nav = makeNav();
        GuidanceBlock g = makeGuidance();
        g.maxAccel = {10.0};  // required 35.33 > 0.95 * 10 = 9.5
        TrackBlock tracks = makeTracks();
        tracks.state = {TrackState::Maintain};
        tracks.updateCount = {5};
        tracks.trackId = {3};
        tracks.posX = {1000.0}; tracks.posY = {100.0}; tracks.posZ = {0.0};
        tracks.velX = {-100.0}; tracks.velY = {0.0}; tracks.velZ = {0.0};
        SeekerBlock seeker;
        seeker.size = 1;
        seeker.type = {SeekerType::None};
        seeker.isLocked = {false};
        EntityStatusBlock status;
        status.size = 1;
        status.isAlive = {true};
        ControlBlock control;
        GuidanceSystem system;
        system.update(status, nav, seeker, tracks, g, control, 0.01);

        check(!g.trajectoryFeasible[0] && g.trajectoryReason[0] == TrajectoryReason::AccelLimited,
              "required accel over budget -> infeasible AccelLimited");
        check(g.limitedByMaxAccel[0] &&
                  near(std::sqrt(g.commandedAccelX[0] * g.commandedAccelX[0] +
                                 g.commandedAccelY[0] * g.commandedAccelY[0] +
                                 g.commandedAccelZ[0] * g.commandedAccelZ[0]), 10.0, 1e-6),
              "demand clamped to the maxAccel budget (finite)");
        check(g.law[0] == GuidanceLaw::Trajectory,
              "law stays Trajectory while the budget limit is reported");
    }

    // ---- 4. Command-aim fallback (Lost track / bare command seed) ---------
    std::printf("\n-- command-aim fallback --\n");
    {
        NavigationBlock nav = makeNav();
        GuidanceBlock g = makeGuidance();
        g.targetX = {1000.0}; g.targetY = {100.0}; g.targetZ = {0.0};
        g.targetVx = {-100.0}; g.targetVy = {0.0}; g.targetVz = {0.0};
        TrackBlock tracks = makeTracks();
        tracks.state = {TrackState::Lost};   // inactive -> command aim
        SeekerBlock seeker;
        seeker.size = 1;
        seeker.type = {SeekerType::None};
        seeker.isLocked = {false};
        EntityStatusBlock status;
        status.size = 1;
        status.isAlive = {true};
        ControlBlock control;
        GuidanceSystem system;
        system.update(status, nav, seeker, tracks, g, control, 0.01);

        check(g.trajectoryAimSource[0] == GuidanceAimSource::Command,
              "Lost track falls back to the command aim");
        check(g.trajectoryFeasible[0] && g.trajectoryReason[0] == TrajectoryReason::Ok &&
                  near(g.predictedInterceptX[0], 661.6285933932035, 1e-4),
              "command aim predicts the same intercept geometry");

        // Bare command seed: state Acquire with no measurements (updateCount 0)
        // is NOT measurement-anchored -> command aim.
        tracks.state = {TrackState::Acquire};
        tracks.updateCount = {0};
        system.update(status, nav, seeker, tracks, g, control, 0.01);
        check(g.trajectoryAimSource[0] == GuidanceAimSource::Command,
              "Acquire with no measurement updates stays command-anchored");
    }

    // ---- 5. Dropout + reacquisition through the W39 lifecycle -------------
    std::printf("\n-- dropout / reacquisition via track lifecycle --\n");
    {
        NavigationBlock nav = makeNav();
        GuidanceBlock g = makeGuidance();
        g.targetX = {900.0}; g.targetY = {100.0}; g.targetZ = {0.0};  // command aim
        g.targetVx = {-100.0}; g.targetVy = {0.0}; g.targetVz = {0.0};
        TrackBlock tracks = makeTracks();
        tracks.state = {TrackState::Maintain};
        tracks.updateCount = {5};
        tracks.trackId = {3};
        tracks.posX = {1000.0}; tracks.posY = {100.0}; tracks.posZ = {0.0};
        tracks.velX = {-100.0}; tracks.velY = {0.0}; tracks.velZ = {0.0};
        tracks.measPosX = {1000.0}; tracks.measPosY = {100.0}; tracks.measPosZ = {0.0};
        tracks.quality01 = {1.0};
        tracks.ageSec = {0.0};

        SeekerBlock seeker;
        seeker.size = 1;
        seeker.type = {SeekerType::RF};
        seeker.isLocked = {false};
        seeker.lockedTargetId = {3};
        seeker.targetRange = {0.0};
        seeker.targetRangeRate = {0.0};
        seeker.targetAzimuth = {0.0};
        seeker.targetElevation = {0.0};
        seeker.targetAzimuthRate = {0.0};
        seeker.targetElevationRate = {0.0};
        EntityStatusBlock status;
        status.size = 1;
        status.isAlive = {true};
        ControlBlock control;
        GuidanceSystem gs;
        TrackManagerSystem tm;

        constexpr double dt = 0.01;
        double t = 0.0;
        bool coastStreamed = false;
        bool trackAimWhileCoasting = false;
        // Maintain -> Coast (0.5 s): predictions keep streaming from the track.
        while (trackState(tracks) == TrackState::Maintain) {
            t += dt;
            tm.update(nav, seeker, tracks, t, dt);
            gs.update(status, nav, seeker, tracks, g, control, dt);
            if (tracks.state[0] == TrackState::Maintain || tracks.state[0] == TrackState::Coast) {
                if (g.trajectoryAimSource[0] == GuidanceAimSource::Track &&
                    g.trajectoryFeasible[0])
                    coastStreamed = true;
            }
        }
        check(trackState(tracks) == TrackState::Coast, "dropout advances Maintain -> Coast");
        check(coastStreamed && g.trajectoryAimSource[0] == GuidanceAimSource::Track,
              "coasting track keeps streaming feasible track-aim predictions");
        // Coast -> Lost (2.0 s total): guidance falls back to the command aim.
        while (trackState(tracks) != TrackState::Lost) {
            t += dt;
            tm.update(nav, seeker, tracks, t, dt);
        }
        gs.update(status, nav, seeker, tracks, g, control, dt);
        check(trackState(tracks) == TrackState::Lost, "dropout advances Coast -> Lost");
        check(g.trajectoryAimSource[0] == GuidanceAimSource::Command,
              "Lost track -> command aim (bounded fallback)");

        // Reacquisition: a new seeker fix returns the track to Reacquire ->
        // Maintain; guidance switches back to the track aim.
        seeker.isLocked = {true};
        const double rng = std::sqrt(800.0 * 800.0 + 100.0 * 100.0);
        seeker.targetRange = {rng};
        seeker.targetAzimuth = {std::atan2(100.0, 800.0)};
        seeker.targetElevation = {0.0};
        for (int s = 0; s < 3; ++s) {
            t += dt;
            tm.update(nav, seeker, tracks, t, dt);
        }
        check(trackState(tracks) == TrackState::Maintain,
              "re-lock returns the track to Maintain");
        // With the seeker locked, guidance overrides to terminal APN (midcourse
        // trajectory is not active). Once the lock drops, the reacquired
        // measurement-anchored track (updateCount > 0) is again the midcourse
        // aim, rebuilding the trajectory prediction from the reacquired track.
        seeker.isLocked = {false};
        gs.update(status, nav, seeker, tracks, g, control, dt);
        check(g.trajectoryAimSource[0] == GuidanceAimSource::Track &&
                  g.trajectoryFeasible[0],
              "Reacquire -> Maintain restores track aim for midcourse");
    }

    // ---- 6. Midcourse-only precedence + legacy parity ---------------------
    std::printf("\n-- lock override and legacy parity --\n");
    {
        NavigationBlock nav = makeNav();
        GuidanceBlock g = makeGuidance();
        TrackBlock tracks = makeTracks();
        tracks.state = {TrackState::Maintain};
        tracks.updateCount = {5};
        tracks.trackId = {3};
        tracks.posX = {1000.0}; tracks.posY = {100.0}; tracks.posZ = {0.0};
        tracks.velX = {-100.0}; tracks.velY = {0.0}; tracks.velZ = {0.0};
        EntityStatusBlock status;
        status.size = 1;
        status.isAlive = {true};
        ControlBlock control;
        GuidanceSystem system;

        // Seeker-locked: terminal seeker-rate APN overrides Trajectory mode.
        SeekerBlock seeker;
        seeker.size = 1;
        seeker.type = {SeekerType::RF};
        seeker.isLocked = {true};
        seeker.lockedTargetId = {3};
        seeker.targetRange = {100.0};
        seeker.targetRangeRate = {-200.0};
        seeker.targetAzimuth = {0.1};
        seeker.targetElevation = {-0.05};
        seeker.targetAzimuthRate = {0.02};
        seeker.targetElevationRate = {-0.01};
        system.update(status, nav, seeker, tracks, g, control, 0.01);
        check(g.phase[0] == GuidancePhase::Terminal && g.law[0] == GuidanceLaw::SeekerRateAPN,
              "a seeker lock overrides Trajectory with terminal APN");
        check(g.trajectoryAimSource[0] == GuidanceAimSource::None &&
                  !g.trajectoryFeasible[0] &&
                  g.trajectoryReason[0] == TrajectoryReason::None &&
                  near(g.predictedTgoSec[0], 0.0, 1e-12),
              "trajectory diagnostics are cleared outside midcourse trajectory");

        // Legacy parity: PN mode is byte-identical and untouched by the W40
        // code (trajectory diagnostics stay at their cleared defaults).
        g = makeGuidance();
        g.mode = {GuidanceMode::ProportionalNavigation};
        g.navigationConstant = {3.5};
        tracks.state = {TrackState::Maintain};
        tracks.updateCount = {5};
        tracks.trackId = {3};
        tracks.posX = {1000.0}; tracks.posY = {0.0}; tracks.posZ = {0.0};
        tracks.velX = {-100.0}; tracks.velY = {10.0}; tracks.velZ = {0.0};
        seeker.type = {SeekerType::None};
        seeker.isLocked = {false};
        system.update(status, nav, seeker, tracks, g, control, 0.01);
        check(near(g.commandedAccelY[0], 7.0, 1e-9),
              "PN midcourse demand is unchanged (legacy parity)");
        check(g.trajectoryAimSource[0] == GuidanceAimSource::None &&
                  !g.trajectoryFeasible[0],
              "PN mode does not activate the trajectory path");
    }

    // ---- 7. Determinism ---------------------------------------------------
    std::printf("\n-- determinism --\n");
    {
        NavigationBlock nav = makeNav();
        SeekerBlock seeker;
        seeker.size = 1;
        seeker.type = {SeekerType::None};
        seeker.isLocked = {false};
        EntityStatusBlock status;
        status.size = 1;
        status.isAlive = {true};
        ControlBlock control;
        GuidanceSystem system;

        GuidanceBlock a = makeGuidance();
        TrackBlock ta = makeTracks();
        ta.state = {TrackState::Maintain};
        ta.updateCount = {5};
        ta.posX = {1000.0}; ta.posY = {100.0}; ta.posZ = {0.0};
        ta.velX = {-100.0}; ta.velY = {0.0}; ta.velZ = {0.0};

        GuidanceBlock b = a;
        TrackBlock tb = ta;
        system.update(status, nav, seeker, ta, a, control, 0.01);
        system.update(status, nav, seeker, tb, b, control, 0.01);
        check(near(a.predictedInterceptX[0], b.predictedInterceptX[0], 1e-12) &&
                  near(a.predictedTgoSec[0], b.predictedTgoSec[0], 1e-12) &&
                  near(a.commandedAccelY[0], b.commandedAccelY[0], 1e-12),
              "repeated evaluation is bit-identical");
    }

    std::printf("\n%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
