// track_manager_fidelity_test: constant-acceleration Kalman track filter,
// measurement-derived covariance, residual gating, retarget debounce, seed
// policy, and quality gating. Every capability is opt-in; the legacy
// overwrite/lifecycle path is covered by track_manager_test.
#include <strikeengine/kernel/systems/TrackManagerSystem.hpp>
#include <strikeengine/kernel/systems/CommandProcessor.hpp>
#include <strikeengine/kernel/systems/SensorSystem.hpp>
#include <strikeengine/kernel/config/ConfigSerialization.hpp>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>
#include <strikeengine/kernel/data/PhysicsBlock.hpp>
#include <strikeengine/kernel/data/SensorBlock.hpp>
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

TrackBlock makeTracks()
{
    TrackBlock t;
    t.size = 1;
    t.confirmations = {3};
    t.coastTimeoutSec = {0.5};
    t.lossTimeoutSec = {2.0};
    t.filterEnabled = {false};
    t.processNoiseMps2 = {15.0};
    t.angleStdRad = {0.003};
    t.measNoiseScale = {1.0};
    t.residualGateSigma = {0.0};
    t.maxAccelMps2 = {0.0};
    t.retargetConfirmations = {1};
    t.seedPolicy = {0};
    t.minQuality01 = {0.0};
    t.qualityTauSec = {1.0};
    t.velocityBlend = {0.08};
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

NavigationBlock makeNav()
{
    NavigationBlock nav;
    nav.size = 1;
    nav.estPx = {0.0}; nav.estPy = {0.0}; nav.estPz = {0.0};
    nav.estVx = {0.0}; nav.estVy = {0.0}; nav.estVz = {0.0};
    nav.estQw = {1.0}; nav.estQx = {0.0}; nav.estQy = {0.0}; nav.estQz = {0.0};
    return nav;
}

SeekerBlock makeSeeker()
{
    SeekerBlock s;
    s.size = 1;
    s.type = {SeekerType::RF};
    s.isLocked = {true};
    s.lockedTargetId = {3};
    s.targetRange = {0.0};
    s.targetRangeRate = {0.0};
    s.targetAzimuth = {0.0};
    s.targetElevation = {0.0};
    s.targetAzimuthRate = {0.0};
    s.targetElevationRate = {0.0};
    return s;
}

void seekerFixAt(SeekerBlock& s, double tx, double ty, double tz)
{
    const double rng = std::sqrt(tx * tx + ty * ty + tz * tz);
    const double nx = tx / rng, ny = ty / rng, nz = tz / rng;
    s.targetRange[0] = rng;
    s.targetAzimuth[0] = std::atan2(ny, nx);
    s.targetElevation[0] = std::asin(std::clamp(-nz, -1.0, 1.0));
}

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

} // namespace

int main()
{
    std::printf("=== track_manager_fidelity_test ===\n");
    constexpr double dt = 0.01;
    NavigationBlock nav = makeNav();
    TrackManagerSystem tm;

    // ---- 1. Filter smooths a noisy position vs the raw overwrite ----
    {
        TrackBlock tracks = makeTracks();
        tracks.filterEnabled = {true};
        SeekerBlock seeker = makeSeeker();
        double t = 0.0;
        double lastRaw = 0.0;
        for (int k = 0; k < 80; ++k) {
            const double noise = ((k % 2) == 0) ? 30.0 : -30.0;
            lastRaw = 1000.0 + noise;
            seekerFixAt(seeker, lastRaw, 0.0, 0.0);
            t += dt;
            tm.update(nav, seeker, tracks, t, dt);
        }
        check(std::abs(tracks.posX[0] - 1000.0) < std::abs(lastRaw - 1000.0),
              "filtered position is closer to truth than the raw noisy fix");
        check(tracks.filterEnabled[0] && tracks.positionStdM[0] > 0.0,
              "filter reports a derived position uncertainty");
    }

    // ---- 2. Filter estimates constant velocity + acceleration availability ----
    {
        TrackBlock tracks = makeTracks();
        tracks.filterEnabled = {true};
        SeekerBlock seeker = makeSeeker();
        double t = 0.0;
        for (int k = 0; k < 150; ++k) {
            t += dt;
            seekerFixAt(seeker, 1000.0 + 100.0 * t, 0.0, 0.0);
            tm.update(nav, seeker, tracks, t, dt);
        }
        check(std::abs(tracks.velX[0] - 100.0) < 5.0,
              "filter converges to the constant target velocity");
        check(tracks.state[0] == TrackState::Maintain && tracks.accelAvailable[0],
              "filtered track provides target acceleration after Maintain");
    }

    // ---- 3. Residual gate rejects a gross outlier ----
    {
        TrackBlock tracks = makeTracks();
        tracks.filterEnabled = {true};
        tracks.residualGateSigma = {3.0};
        SeekerBlock seeker = makeSeeker();
        double t = 0.0;
        for (int k = 0; k < 30; ++k) {
            t += dt;
            seekerFixAt(seeker, 1000.0, 0.0, 0.0);
            tm.update(nav, seeker, tracks, t, dt);
        }
        const double before = tracks.posX[0];
        const std::uint32_t rejects = tracks.residualRejectCount[0];
        t += dt;
        seekerFixAt(seeker, 6000.0, 0.0, 0.0); // 5 km outlier
        tm.update(nav, seeker, tracks, t, dt);
        check(tracks.residualRejectCount[0] > rejects,
              "residual gate flags the gross outlier");
        check(std::abs(tracks.posX[0] - before) < 50.0,
              "gated outlier does not yank the track position");
    }

    // ---- 4. Retarget debounce ----
    {
        TrackBlock tracks = makeTracks();
        tracks.state = {TrackState::Maintain};
        tracks.updateCount = {5};
        tracks.trackId = {3};
        tracks.retargetConfirmations = {3};
        SeekerBlock seeker = makeSeeker();
        seeker.lockedTargetId = {9};
        double t = 0.0;
        seekerFixAt(seeker, 1000.0, 0.0, 0.0);
        t += dt; tm.update(nav, seeker, tracks, t, dt);
        check(tracks.trackId[0] == 3, "one new id does not immediately retarget");
        t += dt; tm.update(nav, seeker, tracks, t, dt);
        check(tracks.trackId[0] == 3, "two new ids still below the debounce count");
        t += dt; tm.update(nav, seeker, tracks, t, dt);
        check(tracks.trackId[0] == 9, "confirmed id switch retargets the track");
    }

    // ---- 5. First acquisition skips the retarget debounce ----
    {
        TrackBlock tracks = makeTracks();
        tracks.retargetConfirmations = {3};
        SeekerBlock seeker = makeSeeker();
        seeker.lockedTargetId = {9};
        double t = 0.0;
        seekerFixAt(seeker, 1000.0, 0.0, 0.0);
        t += dt; tm.update(nav, seeker, tracks, t, dt);
        check(tracks.trackId[0] == 9, "first fix acquires immediately despite confirmations=3");
    }

    // ---- 6. Command-seed policy vs a live measurement track ----
    {
        auto seed = [&](int policy) {
            TrackBlock tracks = makeTracks();
            tracks.state = {TrackState::Maintain};
            tracks.trackId = {3};
            tracks.posX = {1000.0};
            tracks.seedPolicy = {policy};
            GuidanceBlock g = makeGuidance();
            CommandProcessor proc;
            SimulationCommand cmd{};
            cmd.entityId = 0;
            cmd.mode = GuidanceMode::ProportionalNavigation;
            cmd.targetX = 999.0; cmd.targetY = 0.0; cmd.targetZ = 0.0;
            cmd.targetId = 7;
            proc.enqueueCommand(cmd);
            proc.process(g, tracks, 1.0);
            return tracks;
        };
        const TrackBlock clobber = seed(0);
        check(clobber.state[0] == TrackState::Acquire && near(clobber.posX[0], 999.0),
              "legacy seed policy still clobbers the live track");
        const TrackBlock initOnly = seed(1);
        check(initOnly.state[0] == TrackState::Maintain && near(initOnly.posX[0], 1000.0),
              "init-only seed policy leaves a measurement track untouched");
        const TrackBlock refresh = seed(2);
        check(refresh.state[0] == TrackState::Maintain && near(refresh.posX[0], 1000.0),
              "refresh-stale seed policy also preserves a maintained track");
    }

    // ---- 7. Quality gate on active() ----
    {
        TrackBlock tracks = makeTracks();
        tracks.state = {TrackState::Maintain};
        tracks.quality01 = {0.2};
        tracks.minQuality01 = {0.5};
        check(!tracks.active(0), "low-quality track is inactive above the floor");
        tracks.quality01[0] = 0.6;
        check(tracks.active(0), "track above the quality floor is active");
    }

    // ---- 8. All-or-nothing gated fusion: a reject on one axis must not
    // leave the other axes fused while the step is booked as a dropout ----
    {
        TrackBlock tracks = makeTracks();
        tracks.filterEnabled = {true};
        tracks.residualGateSigma = {3.0};
        SeekerBlock seeker = makeSeeker();
        double t = 0.0;
        for (int k = 0; k < 30; ++k) {
            t += dt;
            seekerFixAt(seeker, 1000.0, 0.0, 0.0);
            tm.update(nav, seeker, tracks, t, dt);
        }
        const double pxx = tracks.kfCov[0][0];
        const std::uint32_t rejects = tracks.residualRejectCount[0];
        t += dt;
        seekerFixAt(seeker, 1000.0, 5000.0, 0.0); // Y-only outlier
        tm.update(nav, seeker, tracks, t, dt);
        check(tracks.residualRejectCount[0] > rejects,
              "single-axis outlier trips the residual gate");
        check(tracks.kfCov[0][0] >= pxx,
              "rejected fix fuses nothing: X covariance not shrunk by the passing axis");
        check(std::abs(tracks.posY[0]) < 50.0,
              "rejected fix leaves the track position alone");
    }

    // ---- 9. Dropout grace coasts: a locked-but-stale fix is not fused ----
    {
        TrackBlock tracks = makeTracks();
        tracks.filterEnabled = {true};
        SeekerBlock seeker = makeSeeker();
        seeker.lockLostTimeSec = {0.0};
        double t = 0.0;
        for (int k = 0; k < 30; ++k) {
            t += dt;
            seekerFixAt(seeker, 1000.0, 0.0, 0.0);
            tm.update(nav, seeker, tracks, t, dt);
        }
        seeker.lockLostTimeSec = {0.05}; // seeker inside its dropout grace
        t += dt;
        seekerFixAt(seeker, 1500.0, 0.0, 0.0); // stale fix would yank +500 m
        tm.update(nav, seeker, tracks, t, dt);
        check(tracks.ageSec[0] > 0.0,
              "grace-window step is booked as a dropout, not a fresh fix");
        check(std::abs(tracks.posX[0] - 1000.0) < 50.0,
              "stale grace fix is not fused into the track");
    }

    // ---- 10. Config round-trip of the new track keys ----
    {
        VehicleConfig cfg;
        cfg.guidanceAutopilot.trackFilterEnabled = true;
        cfg.guidanceAutopilot.trackProcessNoiseMps2 = 42.0;
        cfg.guidanceAutopilot.trackAngleStdRad = 0.006;
        cfg.guidanceAutopilot.trackResidualGateSigma = 3.5;
        cfg.guidanceAutopilot.trackRetargetConfirmations = 4;
        cfg.guidanceAutopilot.trackSeedPolicy = 2;
        cfg.guidanceAutopilot.trackMinQuality01 = 0.25;
        cfg.guidanceAutopilot.trackQualityTauSec = 1.5;
        cfg.guidanceAutopilot.trackVelocityBlend = 0.2;
        const VehicleConfig back = deserializeVehicleConfig(serializeVehicleConfig(cfg));
        const auto& g = back.guidanceAutopilot;
        check(g.trackFilterEnabled && g.trackProcessNoiseMps2 == 42.0 &&
              g.trackAngleStdRad == 0.006 && g.trackResidualGateSigma == 3.5,
              "track estimator keys round-trip");
        check(g.trackRetargetConfirmations == 4 && g.trackSeedPolicy == 2 &&
              g.trackMinQuality01 == 0.25 && g.trackQualityTauSec == 1.5 &&
              g.trackVelocityBlend == 0.2,
              "track policy keys round-trip");
    }

    // ---- 11. Multi-target active radar measurements -------------------------
    {
        PhysicsBlock physics;
        physics.ensureSize(3);
        physics.active = {true, true, true};
        physics.px = {0.0, 1000.0, 1000.0};
        physics.py = {0.0, 0.0, 100.0};
        physics.pz = {1000.0, 1000.0, 1000.0};
        physics.qw = {1.0, 1.0, 1.0};
        physics.qx = {0.0, 0.0, 0.0};
        physics.qy = {0.0, 0.0, 0.0};
        physics.qz = {0.0, 0.0, 0.0};
        physics.vx = {100.0, -50.0, -50.0};
        physics.vy = {0.0, 0.0, 0.0};
        physics.vz = {0.0, 0.0, 0.0};

        EntityStatusBlock status;
        status.ensureSize(3);
        status.allegiance = {Allegiance::Friendly, Allegiance::Hostile,
                             Allegiance::Hostile};
        status.isAlive = {true, true, true};

        SensorBlock sensors;
        sensors.ensureSize(3);
        sensors.imuEnabled = {false, false, false};
        sensors.gpsEnabled = {false, false, false};
        sensors.radarEnabled[0] = true;
        sensors.antennaScanRateHz[0] = 5.0;
        sensors.radarMaxRangeM[0] = 5000.0;
        sensors.radarFieldOfViewHalfAngleRad[0] = 1.0;
        sensors.radarMeasurementLatencySec[0] = 0.15;
        sensors.radarRangeNoiseStdDevM[0] = 2.0;
        sensors.radarRangeRateNoiseStdDevMps[0] = 0.5;
        sensors.radarAngleNoiseStdDevRad[0] = 0.001;

        SensorSystem sensorSystem;
        sensorSystem.setSeed(0x12345678u);
        EnvironmentConfig environment;
        sensorSystem.update(physics, sensors, status, 0.2, 0.01, environment);
        check(sensors.radarMeasurements.empty(),
              "radar latency withholds the first scan until delivery");

        sensorSystem.update(physics, sensors, status, 0.4, 0.01, environment);
        check(sensors.radarMeasurements.size() == 2,
              "one radar scan produces independent returns for both hostile targets");
        check(sensors.radarMeasurements[0].sourceEntityId == 0 &&
                  sensors.radarMeasurements[0].targetEntityId == 1 &&
                  sensors.radarMeasurements[1].sourceEntityId == 0 &&
                  sensors.radarMeasurements[1].targetEntityId == 2,
              "radar returns preserve source and target identity");
        check(sensors.radarMeasurements[0].rangeM > 990.0 &&
                  sensors.radarMeasurements[0].rangeM < 1010.0 &&
                  std::isfinite(sensors.radarMeasurements[0].rangeRateMps),
              "radar return carries bounded noisy range and finite Doppler rate");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
