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
    };

} // namespace StrikeEngine::Kernel
