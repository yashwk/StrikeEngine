// guidance_fidelity_test: terminal conditioning (gyro decoupling, 3D body PN),
// command lag/slew shaping, demand scaling, range gain shaping, track-quality
// gating, loft shaping, and diagnostics. All opt-in; the legacy law path is
// covered by guidance_test / trajectory_test.
#include <strikeengine/kernel/systems/GuidanceSystem.hpp>
#include <strikeengine/kernel/config/ConfigSerialization.hpp>

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
    g.scheduledNavN = {3.5};
    g.navScheduleEnabled = {false};
    g.navConstantTerminal = {3.0};
    g.navScheduleTgoSec = {8.0};
    g.waypointGain = {20.0};
    g.handoffBlendTimeSec = {0.0};
    g.lockLossRetentionSec = {0.0};
    g.apnFeedforwardEnabled = {false};
    g.gravityCompensationEnabled = {false};
    g.trajectoryMinSpeedMps = {30.0};
    g.trajectoryFeasibilityAccelFactor = {0.95};
    g.datalinkSourceId = {-1};
    g.datalinkTargetId = {-1};
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
    // New conditioning config/state (sized so shaping operates in tests).
    g.seekerLosRate = {SeekerLosRate::BodyRate};
    g.commandLagSec = {0.0};
    g.commandSlewLimitMps3 = {0.0};
    g.scaleDemandOnInfeasible = {false};
    g.rangeGainShapingEnabled = {false};
    g.rangeGainRefM = {10000.0};
    g.trackAimMinQuality01 = {0.0};
    g.apnFeedforwardMinQuality01 = {0.0};
    g.loftEnabled = {false};
    g.loftAltitudeM = {0.0};
    g.loftGain = {0.0};
    g.loftRangeM = {40000.0};
    g.shapedAccelX = {0.0}; g.shapedAccelY = {0.0}; g.shapedAccelZ = {0.0};
    g.losRateMag = {0.0};
    g.closingSpeed = {0.0};
    g.trackLossActive = {false};
    return g;
}

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

SeekerBlock makeTerminalSeeker()
{
    SeekerBlock s;
    s.size = 1;
    s.type = {SeekerType::RF};
    s.isLocked = {true};
    s.lockedTargetId = {3};
    s.targetRange = {900.0};
    s.targetRangeRate = {-100.0};
    s.targetAzimuth = {0.0};
    s.targetElevation = {0.0};
    s.targetAzimuthRate = {0.02};
    s.targetElevationRate = {0.0};
    s.gimbalAzimuthLimitRad = {1.2};
    s.gimbalElevationLimitRad = {1.2};
    return s;
}

EntityStatusBlock makeStatus()
{
    EntityStatusBlock st;
    st.size = 1;
    st.isAlive = {true};
    st.commsFailed = {false};
    return st;
}

// A seeker that is present but never locked: exercises the midcourse /
// trajectory / waypoint paths (no terminal override).
SeekerBlock makeNoSeeker()
{
    SeekerBlock s;
    s.size = 1;
    s.type = {SeekerType::None};
    s.isLocked = {false};
    s.lockedTargetId = {0};
    s.targetRange = {0.0};
    s.targetRangeRate = {0.0};
    s.targetAzimuth = {0.0};
    s.targetElevation = {0.0};
    s.targetAzimuthRate = {0.0};
    s.targetElevationRate = {0.0};
    return s;
}

void runTerminal(const NavigationBlock& nav, const SeekerBlock& seeker,
                 const TrackBlock& tracks, GuidanceBlock& g, double dt)
{
    EntityStatusBlock status = makeStatus();
    ControlBlock control;
    GuidanceSystem system;
    EnvironmentConfig env;
    system.update(status, nav, seeker, tracks, g, control, dt, env);
}

} // namespace

int main()
{
    std::printf("=== guidance_fidelity_test ===\n");
    const double dt = 0.01;
    NavigationBlock nav = makeNav();
    SeekerBlock seeker = makeTerminalSeeker();
    TrackBlock noTracks; // size 0

    // ---- 1. Legacy body-rate PN commands the analytic demand; InertialPn matches it ----
    {
        GuidanceBlock legacy = makeBlock();
        legacy.mode = {GuidanceMode::ProportionalNavigation};
        runTerminal(nav, seeker, noTracks, legacy, dt);
        check(legacy.law[0] == GuidanceLaw::BodyRatePn &&
              near(legacy.commandedAccelY[0], 7.0, 1e-9),
              "legacy body-rate PN commands N*Vc*dAz (7.0)");
        check(near(legacy.closingSpeed[0], 100.0, 1e-9) &&
              near(legacy.losRateMag[0], 0.02, 1e-12),
              "Vc and LOS-rate diagnostics are published");

        GuidanceBlock body = makeBlock();
        body.mode = {GuidanceMode::ProportionalNavigation};
        body.seekerLosRate = {SeekerLosRate::GyroDecoupled};
        runTerminal(nav, seeker, noTracks, body, dt);
        check(body.law[0] == GuidanceLaw::InertialPn,
              "seekerLosRate=GyroDecoupled selects the InertialPn law");
        check(std::abs(body.commandedAccelY[0] - 7.0) < 0.1,
              "InertialPn reproduces body-rate PN with zero body rate");
    }

    // ---- 2. InertialPn removes the parasitic body-rate term ----
    {
        NavigationBlock rotating = nav;
        rotating.estWz = {0.3}; // 0.3 rad/s yaw
        GuidanceBlock legacy = makeBlock();
        legacy.mode = {GuidanceMode::ProportionalNavigation};
        runTerminal(rotating, seeker, noTracks, legacy, dt);

        GuidanceBlock decoupled = makeBlock();
        decoupled.mode = {GuidanceMode::ProportionalNavigation};
        decoupled.seekerLosRate = {SeekerLosRate::GyroDecoupled};
        runTerminal(rotating, seeker, noTracks, decoupled, dt);
        check(std::abs(decoupled.commandedAccelY[0] - legacy.commandedAccelY[0]) > 1e-6,
              "InertialPn changes the command under a body rotation");
        check(std::isfinite(decoupled.commandedAccelY[0]) &&
              std::abs(decoupled.commandedAccelY[0]) < 1e6,
              "decoupled command stays finite");
    }

    // ---- 3. Command lag shapes a step demand ----
    {
        GuidanceBlock g = makeBlock();
        g.mode = {GuidanceMode::ProportionalNavigation};
        g.commandLagSec = {0.1};
        runTerminal(nav, seeker, noTracks, g, dt);
        check(g.commandedAccelY[0] > 0.0 && g.commandedAccelY[0] < 7.0,
              "first-order lag smooths the step demand");
        for (int k = 0; k < 200; ++k) runTerminal(nav, seeker, noTracks, g, dt);
        check(std::abs(g.commandedAccelY[0] - 7.0) < 0.1,
              "lagged demand converges to the target");
    }

    // ---- 4. Slew limit bounds the per-step change ----
    {
        GuidanceBlock g = makeBlock();
        g.mode = {GuidanceMode::ProportionalNavigation};
        g.commandSlewLimitMps3 = {100.0}; // 1 m/s^2 per 0.01 s step
        runTerminal(nav, seeker, noTracks, g, dt);
        check(near(g.commandedAccelY[0], 1.0, 1e-9),
              "slew limit bounds the first-step demand to rate*dt");
    }

    // ---- 5. Infeasible trajectory demand is scaled to the budget ----
    {
        auto runTraj = [&](bool scale) {
            GuidanceBlock g = makeBlock();
            g.mode = {GuidanceMode::Trajectory};
            g.maxAccel = {10.0};
            g.scaleDemandOnInfeasible = {scale};
            TrackBlock tracks;
            tracks.size = 1;
            tracks.confirmations = {3};
            tracks.coastTimeoutSec = {0.5}; tracks.lossTimeoutSec = {2.0};
            tracks.state = {TrackState::Maintain};
            tracks.trackId = {3}; tracks.updateCount = {5};
            tracks.posX = {1000.0}; tracks.posY = {100.0}; tracks.posZ = {0.0};
            tracks.velX = {-100.0}; tracks.velY = {0.0}; tracks.velZ = {0.0};
            tracks.accelAvailable = {false};
            EntityStatusBlock status = makeStatus();
            ControlBlock control;
            GuidanceSystem system;
            EnvironmentConfig env;
            SeekerBlock ns = makeNoSeeker();
            system.update(status, nav, ns, tracks, g, control, dt, env);
            return std::sqrt(g.commandedAccelX[0] * g.commandedAccelX[0] +
                             g.commandedAccelY[0] * g.commandedAccelY[0] +
                             g.commandedAccelZ[0] * g.commandedAccelZ[0]);
        };
        check(std::abs(runTraj(false) - 10.0) < 1e-6,
              "unscaled infeasible demand is clamped to maxAccel");
        check(std::abs(runTraj(true) - 9.5) < 1e-6,
              "scaled infeasible demand uses the budget factor");
    }

    // ---- 6. Range gain shaping modifies the midcourse command ----
    {
        auto midcourseAt = [&](bool shaping) {
            GuidanceBlock g = makeBlock();
            g.mode = {GuidanceMode::ProportionalNavigation};
            g.rangeGainShapingEnabled = {shaping};
            g.rangeGainRefM = {10000.0};
            TrackBlock tracks;
            tracks.size = 1;
            tracks.confirmations = {3};
            tracks.coastTimeoutSec = {0.5}; tracks.lossTimeoutSec = {2.0};
            tracks.state = {TrackState::Maintain};
            tracks.trackId = {3}; tracks.updateCount = {5};
            tracks.posX = {1000.0}; tracks.posY = {0.0}; tracks.posZ = {0.0};
            tracks.velX = {-100.0}; tracks.velY = {10.0}; tracks.velZ = {0.0};
            tracks.accelAvailable = {false};
            EntityStatusBlock status = makeStatus();
            ControlBlock control;
            GuidanceSystem system;
            EnvironmentConfig env;
            SeekerBlock ns = makeNoSeeker();
            system.update(status, nav, ns, tracks, g, control, dt, env);
            return g.commandedAccelY[0];
        };
        check(std::abs(midcourseAt(true)) > std::abs(midcourseAt(false)),
              "range gain shaping scales the long-range PN command up");
    }

    // ---- 7. Track-quality gate falls back to the command aim ----
    {
        auto aim = [&](double threshold) {
            GuidanceBlock g = makeBlock();
            g.mode = {GuidanceMode::ProportionalNavigation};
            g.targetX = {500.0}; // head-on command aim -> zero lateral
            g.trackAimMinQuality01 = {threshold};
            TrackBlock tracks;
            tracks.size = 1;
            tracks.confirmations = {3};
            tracks.coastTimeoutSec = {0.5}; tracks.lossTimeoutSec = {2.0};
            tracks.state = {TrackState::Maintain};
            tracks.trackId = {3}; tracks.updateCount = {5};
            tracks.posX = {1000.0}; tracks.posY = {0.0}; tracks.posZ = {0.0};
            tracks.velX = {-100.0}; tracks.velY = {10.0}; tracks.velZ = {0.0};
            tracks.quality01 = {0.2};
            tracks.accelAvailable = {false};
            EntityStatusBlock status = makeStatus();
            ControlBlock control;
            GuidanceSystem system;
            EnvironmentConfig env;
            SeekerBlock ns = makeNoSeeker();
            system.update(status, nav, ns, tracks, g, control, dt, env);
            return g.commandedAccelY[0];
        };
        check(std::abs(aim(0.0)) > 1.0, "low-quality track is used when no floor is set");
        check(std::abs(aim(0.5)) < 1e-9, "a quality floor rejects the track for the command aim");
    }

    // ---- 8. Midcourse loft adds a vertical demand ----
    {
        GuidanceBlock g = makeBlock();
        g.mode = {GuidanceMode::ProportionalNavigation};
        g.loftEnabled = {true};
        g.loftAltitudeM = {1000.0};
        g.loftGain = {1.0};
        g.loftRangeM = {40000.0};
        TrackBlock tracks;
        tracks.size = 1;
        tracks.confirmations = {3};
        tracks.coastTimeoutSec = {0.5}; tracks.lossTimeoutSec = {2.0};
        tracks.state = {TrackState::Maintain};
        tracks.trackId = {3}; tracks.updateCount = {5};
        tracks.posX = {1000.0}; tracks.posY = {0.0}; tracks.posZ = {0.0};
        tracks.velX = {-100.0}; tracks.velY = {0.0}; tracks.velZ = {0.0};
        tracks.accelAvailable = {false};
        EntityStatusBlock status = makeStatus();
        ControlBlock control;
        GuidanceSystem system;
        EnvironmentConfig env;
        SeekerBlock ns = makeNoSeeker();
        system.update(status, nav, ns, tracks, g, control, dt, env);
        check(g.commandedAccelZ[0] > 0.0, "loft adds an upward (world +Z local) demand");
    }

    // ---- 9. Config round-trip of the new guidance keys ----
    {
        VehicleConfig cfg;
        cfg.guidanceAutopilot.seekerLosRate = SeekerLosRate::GyroDecoupled;
        cfg.guidanceAutopilot.guidanceCommandLagSec = 0.05;
        cfg.guidanceAutopilot.guidanceCommandSlewLimitMps3 = 250.0;
        cfg.guidanceAutopilot.guidanceScaleDemandOnInfeasible = true;
        cfg.guidanceAutopilot.guidanceRangeGainShapingEnabled = true;
        cfg.guidanceAutopilot.guidanceRangeGainRefM = 8000.0;
        cfg.guidanceAutopilot.guidanceTrackAimMinQuality01 = 0.4;
        cfg.guidanceAutopilot.guidanceApnFeedforwardMinQuality01 = 0.6;
        cfg.guidanceAutopilot.guidanceLoftEnabled = true;
        cfg.guidanceAutopilot.guidanceLoftAltitudeM = 1500.0;
        cfg.guidanceAutopilot.guidanceLoftGain = 0.5;
        cfg.guidanceAutopilot.guidanceLoftRangeM = 30000.0;
        const VehicleConfig back = deserializeVehicleConfig(serializeVehicleConfig(cfg));
        const auto& g = back.guidanceAutopilot;
        check(g.seekerLosRate == SeekerLosRate::GyroDecoupled &&
              g.guidanceCommandLagSec == 0.05 && g.guidanceCommandSlewLimitMps3 == 250.0,
              "terminal conditioning keys round-trip");
        check(g.guidanceScaleDemandOnInfeasible && g.guidanceRangeGainShapingEnabled &&
              g.guidanceRangeGainRefM == 8000.0 && g.guidanceTrackAimMinQuality01 == 0.4 &&
              g.guidanceApnFeedforwardMinQuality01 == 0.6,
              "demand/gain/quality keys round-trip");
        check(g.guidanceLoftEnabled && g.guidanceLoftAltitudeM == 1500.0 &&
              g.guidanceLoftGain == 0.5 && g.guidanceLoftRangeM == 30000.0,
              "loft keys round-trip");
    }

    // ---- 10. Terminal law collapses on receding geometry ----
    {
        GuidanceBlock g = makeBlock();
        g.mode = {GuidanceMode::ProportionalNavigation};
        SeekerBlock receding = seeker;
        receding.targetRangeRate = {50.0}; // opening, not closing
        runTerminal(nav, receding, noTracks, g, dt);
        check(g.commandedAccelX[0] == 0.0 && g.commandedAccelY[0] == 0.0 &&
              g.commandedAccelZ[0] == 0.0 && g.nonClosing[0],
              "receding lock commands zero and flags non-closing (midcourse parity)");
        check(near(g.closingSpeed[0], -50.0, 1e-9),
              "closing-speed diagnostic is signed (negative when opening)");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
