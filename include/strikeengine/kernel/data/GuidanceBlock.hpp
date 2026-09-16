#pragma once
#include <strikeengine/kernel/data/BlockGrowth.hpp>
#include <vector>
#include <cstdint>
#include <cstddef>

namespace StrikeEngine::Kernel {

    enum class GuidanceMode : uint8_t {
        None,                   // Ballistic or Uncontrolled
        ProportionalNavigation, // ProNav interception
        Waypoint,               // Navigating to static point
        Trajectory,             // predictive intercept management (midcourse)
        Cruise                  // aircraft cruise: altitude-hold + waypoint heading
    };

    // Explicit guidance-phase state. Phase selection is separate from
    // law computation; see GuidanceSystem.cpp.
    enum class GuidancePhase : uint8_t {
        None,        // ballistic / no guidance / comms failure
        Midcourse,   // PN (or APN) on the commanded target track
        Acquisition, // seeker locked: APN weight ramps 0 -> 1 (blend)
        Terminal,    // full seeker-rate APN
        LostTrack    // retention expired; recovering via midcourse PN
    };

    // Active guidance law used to produce the current demand.
    enum class GuidanceLaw : uint8_t {
        None,
        Waypoint,       // point-seeking proportional-to-range law
        Tpn,            // true PN: N * Vc * (LOS-rate cross LOS)
        Apn,            // TPN + target-acceleration feed-forward (0.5*N*a_t_perp)
        Trajectory,     // PN aimed at a predicted intercept point
        Cruise          // aircraft altitude-hold + waypoint course
    };

    // Aim source for the trajectory predictor (diagnostic).
    enum class GuidanceAimSource : uint8_t {
        None,    // no aim selected (ballistic / not in Trajectory mode)
        Command, // external command state (SimulationCommand / scenario)
        Track    // measurement-anchored persistent target track
    };

    // Trajectory-feasibility reason (diagnostic).
    enum class TrajectoryReason : uint8_t {
        None,          // no prediction evaluated this step
        Ok,            // predicted intercept is feasible
        VelocityLow,   // own est speed below trajectoryMinSpeedMps
        NoIntercept,   // no positive-time constant-velocity intercept exists
        AccelLimited,  // required acceleration exceeds the maxAccel budget
        NonFinite      // non-finite input / navigation constant
    };

    struct GuidanceBlock {
        std::vector<GuidanceMode> mode;

        // Target position
        std::vector<double> targetX;
        std::vector<double> targetY;
        std::vector<double> targetZ;

        // Target velocity
        std::vector<double> targetVx;
        std::vector<double> targetVy;
        std::vector<double> targetVz;

        // Target acceleration (feed-forward APN); only consumed when
        // targetAccelAvailable and the augmented law is enabled.
        std::vector<double> targetAccelX;
        std::vector<double> targetAccelY;
        std::vector<double> targetAccelZ;
        std::vector<bool>   targetAccelAvailable;

        // Guidance demand magnitude limit (m/s^2); 0 = unlimited.
        // Set per entity via SimulationCommand::maxAccel.
        std::vector<double> maxAccel;

        // Per-entity guidance-law tuning (design-time configurable).
        std::vector<double> navigationConstant;  // APN navigation constant N
        // tgo-scheduled N (see GuidanceAutopilotConfig): the effective gain
        // actually consumed by the laws this step (diagnostic + scheduling).
        std::vector<double> scheduledNavN;
        // Schedule parameters per entity (from GuidanceAutopilotConfig).
        std::vector<bool> navScheduleEnabled;
        std::vector<double> navConstantTerminal;
        std::vector<double> navScheduleTgoSec;
        std::vector<double> waypointGain;        // m/s^2 per unit range fraction

        // Aircraft cruise (GuidanceMode::Cruise) configuration: hold a reference
        // geodetic altitude and fly a level course toward the waypoint target.
        std::vector<double> cruiseAltitudeM;       // reference altitude (m)
        std::vector<double> cruiseAltitudeGain;    // vertical accel per m of altitude error (1/s^2)
        std::vector<double> cruiseAltitudeDamping; // vertical accel per m/s of climb rate (1/s)
        std::vector<double> cruiseWaypointGain;    // horizontal accel toward the waypoint (1/s^2)

        // Phase/track configuration (defaults keep the legacy path).
        std::vector<double> handoffBlendTimeSec;   // acquisition->terminal ramp; 0 = instant
        std::vector<double> lockLossRetentionSec;  // guidance-layer track retention past lock loss; 0 = none
        std::vector<bool>   apnFeedforwardEnabled; // APN target-accel feed-forward (needs targetAccelAvailable)
        std::vector<bool>   gravityCompensationEnabled; // sim-side TPN-G selector (carried for StrikeSim; not consumed by the kernel)

        // Trajectory-core configuration (active only in Trajectory mode;
        // defaults preserve the legacy midcourse path for all other modes).
        std::vector<double> trajectoryMinSpeedMps;          // own est-speed floor for an intercept prediction (default 30.0)
        std::vector<double> trajectoryFeasibilityAccelFactor; // feasibility: requiredAccel <= factor * maxAccel when maxAccel > 0 (0.95)

        // Terminal conditioning.
        std::vector<double> commandLagSec;           // first-order demand lag (s); 0 = off
        std::vector<double> commandSlewLimitMps3;    // demand slew limit (m/s^3); 0 = off
        std::vector<bool>   scaleDemandOnInfeasible; // scale, not just flag, an over-budget demand
        std::vector<bool>   rangeGainShapingEnabled; // N'(r) gain shaping
        std::vector<double> rangeGainRefM;           // reference range for shaping
        std::vector<double> trackAimMinQuality01;    // min track quality to use as aim
        std::vector<double> apnFeedforwardMinQuality01; // min track quality to trust APN ff
        std::vector<bool>   loftEnabled;             // midcourse loft shaping
        std::vector<double> loftAngleDeg;            // climb bias over the sightline (deg)
        std::vector<double> loftAltitudeM;           // apex ceiling; 0 = no ceiling (m)
        std::vector<double> loftGain;                // angle-hold gain (1/s)
        std::vector<double> loftRangeM;              // range where the bias is full (m)
        // Guidance authority awareness: scale the demand by the autopilot's
        // previous-step delivered/demanded fin margin (0.1..1).
        std::vector<bool>   authorityAwareLimitEnabled;

        // Cooperative-engagement datalink (see GuidanceAutopilotConfig):
        // the source entity whose persistent track provides the midcourse aim,
        // and the target entity that track is of. -1 = disabled.
        std::vector<int> datalinkSourceId;
        std::vector<int> datalinkTargetId;

        // Output: Required acceleration command
        std::vector<double> commandedAccelX;
        std::vector<double> commandedAccelY;
        std::vector<double> commandedAccelZ;

        // Phase/track state + diagnostics (see GuidanceSystem.cpp).
        std::vector<GuidancePhase> phase;
        std::vector<GuidanceLaw>   law;
        std::vector<std::int64_t>  trackId;            // -1 = none
        std::vector<double>        trackAgeSec;        // s since last valid seeker track
        std::vector<double>        handoffWeight;      // 0..1 terminal APN blend weight
        std::vector<std::uint32_t> lockLossCount;      // terminal lock lost transitions
        // raw (pre-clamp) demand and limit flags
        std::vector<double> rawAccelX;
        std::vector<double> rawAccelY;
        std::vector<double> rawAccelZ;
        std::vector<bool>   limitedByMaxAccel;
        std::vector<bool>   lawInvalid;
        std::vector<bool>   nonClosing;
        std::vector<double> tgoSec;                    // range / closing speed

        // Last valid terminal (seeker-APN) demand, post-clamp. Used as the
        // bounded predicted command during the lock-loss retention window
        // (see GuidanceSystem.cpp); refreshed on every locked terminal step.
        std::vector<double> retainedAccelX;
        std::vector<double> retainedAccelY;
        std::vector<double> retainedAccelZ;

        // Trajectory prediction + feasibility diagnostics (midcourse).
        std::vector<double> predictedInterceptX;  // predicted intercept point (world frame)
        std::vector<double> predictedInterceptY;
        std::vector<double> predictedInterceptZ;
        std::vector<double> predictedTgoSec;      // time to the predicted intercept (0 = none)
        std::vector<double> trajectoryRequiredAccel; // |PN demand| aimed at the PIP (m/s^2)
        std::vector<GuidanceAimSource> trajectoryAimSource; // Track/Command/None
        std::vector<bool>   trajectoryFeasible;   // predicted intercept fits the accel budget
        std::vector<TrajectoryReason> trajectoryReason;

        // Terminal conditioning state + diagnostics.
        std::vector<double> shapedAccelX;         // post-lag/slew demand (== commanded when off)
        std::vector<double> shapedAccelY;
        std::vector<double> shapedAccelZ;
        std::vector<double> losRateMag;           // |seeker LOS rate| used (rad/s)
        std::vector<double> closingSpeed;         // closing speed used (m/s)
        std::vector<bool>   trackLossActive;      // a terminal lock-loss episode is being counted

        std::size_t size = 0;

        /**
         * @brief Grows every vector to @p n entries; see
         *        PhysicsBlock::ensureSize.
         *
         * Configuration fields carry block defaults here; the kernel
         * overwrites them from GuidanceAutopilotConfig once the slot exists.
         */
        void ensureSize(std::size_t n) {
            growTo(mode, n, GuidanceMode::None);
            growTo(targetX, n, 0.0); growTo(targetY, n, 0.0); growTo(targetZ, n, 0.0);
            growTo(targetVx, n, 0.0); growTo(targetVy, n, 0.0); growTo(targetVz, n, 0.0);
            growTo(targetAccelX, n, 0.0); growTo(targetAccelY, n, 0.0); growTo(targetAccelZ, n, 0.0);
            growTo(targetAccelAvailable, n, false);
            growTo(commandedAccelX, n, 0.0);
            growTo(commandedAccelY, n, 0.0);
            growTo(commandedAccelZ, n, 0.0);
            growTo(maxAccel, n, 0.0);
            growTo(navigationConstant, n, 3.5);
            growTo(scheduledNavN, n, 3.5);
            growTo(navScheduleEnabled, n, false);
            growTo(navConstantTerminal, n, 3.0);
            growTo(navScheduleTgoSec, n, 8.0);
            growTo(waypointGain, n, 20.0);
            growTo(cruiseAltitudeM, n, 0.0);
            growTo(cruiseAltitudeGain, n, 0.05);
            growTo(cruiseAltitudeDamping, n, 0.30);
            growTo(cruiseWaypointGain, n, 0.8);
            growTo(handoffBlendTimeSec, n, 0.0);
            growTo(lockLossRetentionSec, n, 0.0);
            growTo(apnFeedforwardEnabled, n, false);
            growTo(gravityCompensationEnabled, n, false);
            growTo(trajectoryMinSpeedMps, n, 30.0);
            growTo(trajectoryFeasibilityAccelFactor, n, 0.95);
            growTo(commandLagSec, n, 0.0);
            growTo(commandSlewLimitMps3, n, 0.0);
            growTo(scaleDemandOnInfeasible, n, false);
            growTo(rangeGainShapingEnabled, n, false);
            growTo(rangeGainRefM, n, 10000.0);
            growTo(trackAimMinQuality01, n, 0.0);
            growTo(apnFeedforwardMinQuality01, n, 0.0);
            growTo(loftEnabled, n, false);
            growTo(loftAngleDeg, n, 0.0);
            growTo(loftAltitudeM, n, 0.0);
            growTo(loftGain, n, 0.0);
            growTo(loftRangeM, n, 40000.0);
            growTo(authorityAwareLimitEnabled, n, false);
            growTo(datalinkSourceId, n, -1);
            growTo(datalinkTargetId, n, -1);
            growTo(phase, n, GuidancePhase::None);
            growTo(law, n, GuidanceLaw::None);
            growTo(trackId, n, -1);
            growTo(trackAgeSec, n, 0.0);
            growTo(handoffWeight, n, 0.0);
            growTo(lockLossCount, n, 0u);
            growTo(rawAccelX, n, 0.0); growTo(rawAccelY, n, 0.0); growTo(rawAccelZ, n, 0.0);
            growTo(limitedByMaxAccel, n, false);
            growTo(lawInvalid, n, false);
            growTo(nonClosing, n, false);
            growTo(tgoSec, n, 0.0);
            growTo(retainedAccelX, n, 0.0);
            growTo(retainedAccelY, n, 0.0);
            growTo(retainedAccelZ, n, 0.0);
            growTo(predictedInterceptX, n, 0.0);
            growTo(predictedInterceptY, n, 0.0);
            growTo(predictedInterceptZ, n, 0.0);
            growTo(predictedTgoSec, n, 0.0);
            growTo(trajectoryRequiredAccel, n, 0.0);
            growTo(trajectoryAimSource, n, GuidanceAimSource::None);
            growTo(trajectoryFeasible, n, false);
            growTo(trajectoryReason, n, TrajectoryReason::None);
            growTo(shapedAccelX, n, 0.0); growTo(shapedAccelY, n, 0.0); growTo(shapedAccelZ, n, 0.0);
            growTo(losRateMag, n, 0.0);
            growTo(closingSpeed, n, 0.0);
            growTo(trackLossActive, n, false);
            if (n > size) size = n;
        }
    };

} // namespace StrikeEngine::Kernel
