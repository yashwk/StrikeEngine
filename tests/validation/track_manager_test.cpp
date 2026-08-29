// W39 persistent target-track manager verification.
//
// Covers the full lifecycle and handoff contract:
//   - external command seed -> Acquire (identity, pos/vel/accel, timestamp)
//   - seeker LOS fixes -> world-frame estimate + finite-difference velocity
//     (no physics-truth coupling: conversion uses the navigation estimate)
//   - Acquire -> Maintain after `confirmations` consistent measurement updates
//   - multi-rate prediction between measurements (pos += vel*dt at sim rate)
//   - Maintain -> Coast (no measurement for coastTimeoutSec) with quality
//     decay and growing uncertainty
//   - Coast -> Lost (no measurement for lossTimeoutSec); guidance falls back
//     to the external command aim
//   - Coast/Lost + new measurement -> Reacquire -> Maintain
//   - guidance handoff: midcourse PN consumes the measurement-anchored track
//     (never physics truth) and reverts to the command aim when Lost.
#include <strikeengine/kernel/systems/TrackManagerSystem.hpp>
#include <strikeengine/kernel/systems/GuidanceSystem.hpp>
#include <strikeengine/kernel/systems/CommandProcessor.hpp>
#include <strikeengine/kernel/data/TrackBlock.hpp>

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

bool near(double a, double b, double tol = 1e-9) { return std::abs(a - b) < tol; }

// Fully sized single-entity track block with the default config.
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

// Minimal sized guidance block (used by the CommandProcessor).
GuidanceBlock makeGuidance()
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
    g.retainedAccelX = {0.0}; g.retainedAccelY = {0.0}; g.retainedAccelZ = {0.0};
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

// Points the seeker at world position (tx, ty, tz) from the origin with an
// identity attitude (the tests use an identity navigation quaternion).
void seekerFixAt(SeekerBlock& s, double tx, double ty, double tz)
{
    const double rng = std::sqrt(tx * tx + ty * ty + tz * tz);
    const double nx = tx / rng, ny = ty / rng, nz = tz / rng;
    s.targetRange[0] = rng;
    s.targetAzimuth[0] = std::atan2(ny, nx);
    s.targetElevation[0] = std::asin(std::clamp(-nz, -1.0, 1.0));
}

} // namespace

int main()
{
    std::printf("=== track_manager_test: persistent target-track lifecycle ===\n");

    // Shared identity-attitude navigation block (entity 0 at the origin).
    NavigationBlock nav;
    nav.size = 1;
    nav.estPx = {0.0}; nav.estPy = {0.0}; nav.estPz = {0.0};
    nav.estVx = {100.0}; nav.estVy = {0.0}; nav.estVz = {0.0};
    nav.estQw = {1.0}; nav.estQx = {0.0}; nav.estQy = {0.0}; nav.estQz = {0.0};
    nav.estWx = {0.0}; nav.estWy = {0.0}; nav.estWz = {0.0};

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

    TrackManagerSystem tm;

    // ---- 1. External command seed: identity + state + timestamp ------------
    std::printf("\n-- command seed --\n");
    {
        TrackBlock tracks = makeTracks();
        GuidanceBlock g = makeGuidance();
        CommandProcessor proc;
        SimulationCommand cmd{};
        cmd.entityId = 0;
        cmd.mode = GuidanceMode::ProportionalNavigation;
        cmd.targetX = 500.0; cmd.targetY = 10.0; cmd.targetZ = 0.0;
        cmd.targetVx = 20.0; cmd.targetVy = 0.0; cmd.targetVz = -5.0;
        cmd.targetAccelY = 2.0;
        cmd.targetAccelAvailable = true;
        cmd.targetId = 7;
        proc.enqueueCommand(cmd);
        proc.process(g, tracks, 1.0);
        check(tracks.state[0] == TrackState::Acquire,
              "command seed starts the track in Acquire");
        check(tracks.trackId[0] == 7 && near(tracks.posX[0], 500.0) &&
                  near(tracks.velY[0], 0.0) && near(tracks.velZ[0], -5.0),
              "command seeds identity, position, velocity");
        check(tracks.accelAvailable[0] && near(tracks.accelY[0], 2.0),
              "command seeds optional target acceleration");
        check(near(tracks.timestampSec[0], 1.0) && near(tracks.ageSec[0], 0.0) &&
                  near(tracks.quality01[0], 1.0),
              "command seeds measurement timestamp and age");
    }

    // ---- 2. Measurement acquisition, velocity, and promotion ----------------
    std::printf("\n-- measurement acquisition -> Maintain --\n");
    {
        TrackBlock tracks = makeTracks();
        constexpr double dt = 0.01;
        double t = 0.0;
        seeker.isLocked = {true};

        // Fix 1: target at (1000, 100, 50), own-ship origin -> estimate matches.
        seekerFixAt(seeker, 1000.0, 100.0, 50.0);
        t += dt;
        tm.update(nav, seeker, tracks, t, dt);
        check(tracks.state[0] == TrackState::Acquire && tracks.updateCount[0] == 1,
              "first fix starts Acquire");
        check(near(tracks.posX[0], 1000.0) && near(tracks.posY[0], 100.0) &&
                  near(tracks.posZ[0], 50.0),
              "LOS fix converts to the world-frame estimate (no truth coupling)");
        check(near(tracks.quality01[0], 1.0) && near(tracks.timestampSec[0], t),
              "measurement sets quality 1 and the timestamp");

        // Fix 2: target moves +1 m in x over one step -> 100 m/s estimate.
        seekerFixAt(seeker, 1001.0, 100.0, 50.0);
        t += dt;
        tm.update(nav, seeker, tracks, t, dt);
        check(tracks.state[0] == TrackState::Acquire && tracks.updateCount[0] == 2,
              "second fix still Acquire");
        check(near(tracks.velX[0], 100.0, 1e-6),
              "finite-difference velocity from consecutive fixes");

        // Fix 3: promotion to Maintain (confirmations = 3).
        seekerFixAt(seeker, 1002.0, 100.0, 50.0);
        t += dt;
        tm.update(nav, seeker, tracks, t, dt);
        check(tracks.state[0] == TrackState::Maintain && tracks.updateCount[0] == 3,
              "third consistent fix promotes to Maintain");
        check(tracks.trackId[0] == 3, "track carries the seeker target identity");
    }

    // ---- 3. Coast: multi-rate prediction, quality decay, uncertainty --------
    std::printf("\n-- coast (multi-rate prediction between measurements) --\n");
    {
        TrackBlock tracks = makeTracks();
        tracks.state = {TrackState::Maintain};
        tracks.updateCount = {5};
        tracks.trackId = {3};
        tracks.posX = {1000.0}; tracks.posY = {100.0}; tracks.posZ = {50.0};
        tracks.velX = {100.0}; tracks.velY = {0.0}; tracks.velZ = {0.0};
        tracks.measPosX = {1000.0}; tracks.measPosY = {100.0}; tracks.measPosZ = {50.0};
        tracks.measTimeSec = {0.0};
        tracks.quality01 = {1.0};
        seeker.isLocked = {false};

        constexpr double dt = 0.01;
        double t = 0.0;
        // 10 coast steps = 0.1 s: position advances by vel*dt at the sim rate.
        for (int s = 0; s < 10; ++s) {
            t += dt;
            tm.update(nav, seeker, tracks, t, dt);
        }
        check(near(tracks.posX[0], 1010.0, 1e-9),
              "coast predicts position at the simulation rate (multi-rate)");
        check(near(tracks.ageSec[0], 0.1, 1e-6) && tracks.dropoutCount[0] == 10,
              "track age and dropout counter grow without measurements");
        check(near(tracks.quality01[0], std::exp(-0.1), 1e-3),
              "quality decays exponentially with track age");
        check(near(tracks.positionStdM[0], 5.0 + 25.0 * 0.1, 1e-9) &&
                  near(tracks.velocityStdMs[0], 25.0 + 50.0 * 0.1, 1e-9),
              "position/velocity uncertainty grow with age (covariance model)");
        check(tracks.state[0] == TrackState::Maintain,
              "within coastTimeoutSec the track stays Maintain");

        // 40 more steps: total 0.5 s -> Coast.
        for (int s = 0; s < 40; ++s) {
            t += dt;
            tm.update(nav, seeker, tracks, t, dt);
        }
        check(tracks.state[0] == TrackState::Coast,
              "no measurement for coastTimeoutSec: Maintain -> Coast");

        // 150 more steps: total 2.0 s since the last measurement -> Lost.
        for (int s = 0; s < 150; ++s) {
            t += dt;
            tm.update(nav, seeker, tracks, t, dt);
        }
        check(tracks.state[0] == TrackState::Lost,
              "no measurement for lossTimeoutSec: Coast -> Lost");
    }

    // ---- 4. Reacquisition ------------------------------------------------
    std::printf("\n-- reacquisition --\n");
    {
        TrackBlock tracks = makeTracks();
        tracks.state = {TrackState::Lost};
        tracks.trackId = {3};
        tracks.posX = {1000.0}; tracks.posY = {100.0}; tracks.posZ = {50.0};
        tracks.velX = {100.0}; tracks.velY = {0.0}; tracks.velZ = {0.0};
        seeker.isLocked = {true};

        constexpr double dt = 0.01;
        double t = 0.0;
        seekerFixAt(seeker, 1010.0, 100.0, 50.0);
        t += dt;
        tm.update(nav, seeker, tracks, t, dt);
        check(tracks.state[0] == TrackState::Reacquire && tracks.updateCount[0] == 1,
              "a new fix on a Lost track enters Reacquire");
        seekerFixAt(seeker, 1011.0, 100.0, 50.0);
        t += dt;
        tm.update(nav, seeker, tracks, t, dt);
        seekerFixAt(seeker, 1012.0, 100.0, 50.0);
        t += dt;
        tm.update(nav, seeker, tracks, t, dt);
        check(tracks.state[0] == TrackState::Maintain,
              "confirmations after Reacquire return to Maintain");
    }

    // ---- 5. Guidance handoff: track aim, no truth coupling ------------------
    std::printf("\n-- guidance handoff --\n");
    {
        GuidanceBlock g = makeGuidance();
        g.mode = {GuidanceMode::ProportionalNavigation};
        g.targetX = {500.0}; g.targetY = {0.0}; g.targetZ = {0.0};  // command aim
        g.targetVx = {0.0}; g.targetVy = {0.0}; g.targetVz = {0.0};

        // Active measurement-anchored track at (1000, 0, 0), vel (-100, 10, 0):
        // PN on the track -> Vc = 200 m/s, LOS rate 0.01 rad/s -> ay = 7.0.
        TrackBlock tracks = makeTracks();
        tracks.state = {TrackState::Maintain};
        tracks.updateCount = {5};
        tracks.trackId = {3};
        tracks.posX = {1000.0}; tracks.posY = {0.0}; tracks.posZ = {0.0};
        tracks.velX = {-100.0}; tracks.velY = {10.0}; tracks.velZ = {0.0};

        EntityStatusBlock status;
        status.size = 1;
        status.isAlive = {true};
        ControlBlock control;
        GuidanceSystem system;
        seeker.type = {SeekerType::None};   // pure midcourse PN path (no lock)
        system.update(status, nav, seeker, tracks, g, control, 0.01);
        check(near(g.commandedAccelY[0], 7.0, 1e-9),
              "midcourse PN consumes the persistent track (not the command aim)");
        check(g.phase[0] == GuidancePhase::Midcourse,
              "track-guided midcourse keeps phase Midcourse");

        // Lost track: guidance falls back to the external command aim
        // (command target at (500,0,0), stationary vs nav vx=100: head-on ->
        // zero lateral demand), and the feed-forward accel stays command-based.
        tracks.state = {TrackState::Lost};
        system.update(status, nav, seeker, tracks, g, control, 0.01);
        check(near(g.commandedAccelY[0], 0.0, 1e-9),
              "Lost track falls back to the external command aim");
    }

    std::printf("\n%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
