#pragma once
#include <vector>
#include <cstdint>

namespace StrikeEngine::Kernel {

    enum class GuidanceMode : uint8_t {
        None,                   // Ballistic or Uncontrolled
        ProportionalNavigation, // ProNav interception
        Waypoint,               // Navigating to static point
        Trajectory              // W40 predictive intercept management (midcourse)
    };

    // Explicit guidance-phase state (W36). Phase selection is separate from
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
        PureProNav,     // N * Vc * (LOS-rate cross LOS)
        SeekerRateAPN,  // body-frame LOS-rate APN (seeker)
        AugmentedProNav,// PN + target-acceleration feed-forward (0.5*N*a_t_perp)
        Trajectory      // W40 PN aimed at a predicted intercept point
    };

    // Aim source for the W40 trajectory predictor (diagnostic).
    enum class GuidanceAimSource : uint8_t {
        None,    // no aim selected (ballistic / not in Trajectory mode)
        Command, // external command state (SimulationCommand / scenario)
        Track    // measurement-anchored persistent target track (W39)
    };

    // W40 trajectory-feasibility reason (diagnostic).
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
        std::vector<double> waypointGain;        // m/s^2 per unit range fraction

        // W36 phase/track configuration (defaults keep the legacy path).
        std::vector<double> handoffBlendTimeSec;   // acquisition->terminal ramp; 0 = instant
        std::vector<double> lockLossRetentionSec;  // guidance-layer track retention past lock loss; 0 = none
        std::vector<bool>   apnFeedforwardEnabled; // APN target-accel feed-forward (needs targetAccelAvailable)
        std::vector<bool>   gravityCompensationEnabled; // TPN-G law selector (set by StrikeSim; not consumed by the seeker APN)

        // W40 trajectory-core configuration (active only in Trajectory mode;
        // defaults preserve the legacy midcourse path for all other modes).
        std::vector<double> trajectoryMinSpeedMps;          // own est-speed floor for an intercept prediction (default 30.0)
        std::vector<double> trajectoryFeasibilityAccelFactor; // feasibility: requiredAccel <= factor * maxAccel when maxAccel > 0 (0.95)

        // W41 cooperative-engagement datalink (see GuidanceAutopilotConfig):
        // the source entity whose persistent track provides the midcourse aim,
        // and the target entity that track is of. -1 = disabled.
        std::vector<int> datalinkSourceId;
        std::vector<int> datalinkTargetId;

        // Output: Required acceleration command
        std::vector<double> commandedAccelX;
        std::vector<double> commandedAccelY;
        std::vector<double> commandedAccelZ;

        // W36 phase/track state + diagnostics (see GuidanceSystem.cpp).
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

        // W40 trajectory prediction + feasibility diagnostics (midcourse).
        std::vector<double> predictedInterceptX;  // predicted intercept point (world frame)
        std::vector<double> predictedInterceptY;
        std::vector<double> predictedInterceptZ;
        std::vector<double> predictedTgoSec;      // time to the predicted intercept (0 = none)
        std::vector<double> trajectoryRequiredAccel; // |PN demand| aimed at the PIP (m/s^2)
        std::vector<GuidanceAimSource> trajectoryAimSource; // Track/Command/None
        std::vector<bool>   trajectoryFeasible;   // predicted intercept fits the accel budget
        std::vector<TrajectoryReason> trajectoryReason;
    };

} // namespace StrikeEngine::Kernel
