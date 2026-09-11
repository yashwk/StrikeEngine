#pragma once

namespace StrikeEngine::Kernel {

    struct GuidanceAutopilotConfig {
        double navigationConstant = 3.5;   // APN N
        // tgo-scheduled N (opt-in; off = constant navigationConstant): inside
        // navScheduleTgoSec the laws use navConstantTerminal instead of N.
        // Previous step's tgo drives the switch (deterministic, no peeking).
        bool   navScheduleEnabled = false;
        double navConstantTerminal = 3.0;
        double navScheduleTgoSec = 8.0;
        double waypointGain = 20.0;        // m/s^2 per unit range fraction
        double kAccelP = 0.030;  // rad deflection per (m/s^2)
        double kRateP  = 1.000;  // rad per (rad/s)
        double kAlphaP = 0.200;  // rad per rad AoA/beta
        double kRollP  = 0.10;   // rad per rad roll
        double kRollD  = 0.05;   // rad per (rad/s)
        double maxDeflectionRad = 0.43;      // ~25 deg fin clamp
        double servoTimeConstantSec = 0.02;  // future servo lag (inert)
        double maxServoRateRadPerSec = 5.24; // future servo rate (inert)

        // W36 guidance-phase manager (defaults preserve the legacy behavior):
        // handoffBlendTimeSec = 0  -> instant seeker override (legacy)
        // lockLossRetentionSec = 0 -> no guidance-layer retention past seeker loss
        // apnFeedforwardEnabled = false -> pure PN midcourse (legacy)
        double handoffBlendTimeSec  = 0.0;  // acquisition -> terminal ramp time (s)
        double lockLossRetentionSec = 0.0;  // parent track retention after lock loss (s)
        bool   apnFeedforwardEnabled = false; // use target-accel feed-forward APN
        bool   gravityCompensationEnabled = false; // TPN-G law selector (used by the StrikeSim designer; inert in the kernel seeker APN)

        // W39 persistent target-track manager (defaults keep the external
        // command state as the guidance aim; a track only supersedes it once
        // confirmations of consistent seeker measurements arrive).
        int    trackConfirmations  = 3;    // measurement updates to promote to Maintain
        double trackCoastTimeoutSec = 0.5; // no measurement: Maintain/Acquire -> Coast
        double trackLossTimeoutSec  = 2.0; // no measurement: Coast -> Lost

        // W39 track estimator + association options (defaults reproduce the
        // legacy overwrite/lifecycle behavior).
        bool   trackFilterEnabled = false;       // constant-acceleration Kalman filter
        double trackProcessNoiseMps2 = 15.0;     // target accel PSD (m^2/s^3)
        double trackAngleStdRad = 0.003;         // assumed seeker angular sigma when unmodeled
        double trackMeasNoiseScale = 1.0;        // scales the derived measurement variance
        double trackResidualGateSigma = 0.0;     // 0 = off; else innovation gate
        double trackMaxAccelMps2 = 0.0;          // 0 = no clamp on the estimated acceleration
        int    trackRetargetConfirmations = 1;   // consistent ids before switching target
        int    trackSeedPolicy = 0;              // 0 clobber, 1 init-only, 2 refresh-stale
        double trackMinQuality01 = 0.0;          // active() quality gate (0 = off)
        double trackQualityTauSec = 1.0;         // track quality decay (legacy 1.0 s)
        double trackVelocityBlend = 0.08;        // legacy finite-difference low-pass

        // W41 cooperative-engagement datalink: when datalinkSourceId >= 0 the
        // entity uses the source entity's persistent track (of datalinkTargetId)
        // as its midcourse aim, instead of its own command/track, until its own
        // seeker acquires. This models the launch aircraft (or an AWACS/radar
        // mothership) providing midcourse guidance to a BVRAAM. -1 = disabled.
        int datalinkSourceId = -1;
        int datalinkTargetId = -1;

        // Aircraft cruise (GuidanceMode::Cruise): hold a reference geodetic
        // altitude and fly a level course toward the waypoint target.
        double cruiseAltitudeM       = 0.0;   // reference altitude (m); <=0 => follow the waypoint Z
        double cruiseAltitudeGain    = 0.05;  // vertical accel per m of altitude error (1/s^2)
        double cruiseAltitudeDamping = 0.30;  // vertical accel per m/s of climb rate (1/s)
        double cruiseWaypointGain    = 0.8;   // horizontal accel toward the waypoint (1/s^2)

        // W40 trajectory-core keys (optional with legacy defaults; active only
        // when GuidanceMode::Trajectory is explicitly selected). A seeker lock
        // still overrides midcourse trajectory management exactly as it does
        // for ProportionalNavigation (W36 precedence).
        double trajectoryMinSpeedMps = 30.0;           // own est-speed floor for an intercept prediction
        double trajectoryFeasibilityAccelFactor = 0.95; // feasibility: requiredAccel <= factor * maxAccel (when maxAccel > 0)

        // --- Terminal conditioning + law selection (defaults = legacy) ------
        bool   guidanceGyroDecouplingEnabled = false;   // remove body rate from the seeker LOS rate
        int    terminalLaw = 0;                         // 0 = SeekerRateAPN, 1 = BodyPN (3D)
        double guidanceCommandLagSec = 0.0;             // first-order demand lag (s); 0 = off
        double guidanceCommandSlewLimitMps3 = 0.0;      // demand slew limit (m/s^3); 0 = off
        bool   guidanceScaleDemandOnInfeasible = false; // scale over-budget demand to the limit
        bool   guidanceRangeGainShapingEnabled = false; // N'(r) gain shaping
        double guidanceRangeGainRefM = 10000.0;         // reference range for shaping
        double guidanceTrackAimMinQuality01 = 0.0;      // min track quality to use as aim
        double guidanceApnFeedforwardMinQuality01 = 0.0;// min track quality to trust APN ff
        bool   guidanceLoftEnabled = false;             // midcourse loft shaping
        double guidanceLoftAltitudeM = 0.0;             // loft apex above launch altitude (m)
        double guidanceLoftGain = 0.0;                  // vertical accel per m of loft error
        double guidanceLoftRangeM = 40000.0;            // range beyond which loft applies

        // Dynamic pressure (q) gain scheduling: scales feed-forward fin command
        // by sqrt(q_ref / q) to prevent max-Q control flutter and high-altitude sluggishness.
        bool   gainSchedulingEnabled = false;
        double refDynamicPressurePa = 50000.0; // Reference dynamic pressure (Pa)
        double minDynamicPressurePa = 2000.0;  // Floor dynamic pressure (Pa)
        double maxDynamicPressurePa = 300000.0; // Ceiling dynamic pressure (Pa)
    };

} // namespace StrikeEngine::Kernel
