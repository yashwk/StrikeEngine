#pragma once

namespace StrikeEngine::Kernel {

    struct GuidanceAutopilotConfig {
        double navigationConstant = 3.5;   // APN N
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
    };

} // namespace StrikeEngine::Kernel
