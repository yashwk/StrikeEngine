#pragma once
#include <vector>
#include <cstddef>

namespace StrikeEngine::Kernel {

struct ControlBlock {
	std::vector<double> thrustCommand;
    std::vector<double> pitchCommand;
    std::vector<double> yawCommand;
    std::vector<double> rollCommand;
    std::vector<double> thrustVectorPitchCommand;
    std::vector<double> thrustVectorYawCommand;

	// Per-entity autopilot gains + fin clamp (design-time configurable).
	std::vector<double> kAccelP;          // rad fin deflection per (m/s^2)
	std::vector<double> kRateP;           // rad per (rad/s)
	std::vector<double> kAlphaP;          // rad per rad AoA/beta
	std::vector<double> kRollP;           // rad per rad roll
	std::vector<double> kRollD;           // rad per (rad/s)
	std::vector<double> maxDeflectionRad; // fin clamp (rad)

	// Dynamic pressure gain scheduling
	std::vector<bool>   gainSchedulingEnabled;
	std::vector<double> refDynamicPressurePa;
	std::vector<double> minDynamicPressurePa;
	std::vector<double> maxDynamicPressurePa;

	// --- Autopilot loop-quality config (from GuidanceAutopilotConfig) ----
	std::vector<double> kRatePitchP;
	std::vector<double> kRateYawP;
	std::vector<bool>   scheduleAllTerms;
	std::vector<bool>   integralEnabled;
	std::vector<double> kIntegralPitch;
	std::vector<double> kIntegralYaw;
	std::vector<double> integralClampRad;
	std::vector<double> kAccelErrP;
	std::vector<bool>   controlEffectivenessEnabled;
	std::vector<double> controlEffBase;
	std::vector<double> controlEffMachSlope;
	std::vector<double> controlEffMachQuad;
	std::vector<double> controlEffMin;
	std::vector<double> controlEffMax;
	std::vector<bool>   yawDeadbandSmoothEnabled;
	std::vector<double> yawDeadbandWidthMps2;
	std::vector<double> commandLagSec;
	std::vector<double> commandRateLimitRadPerSec;
	std::vector<bool>   useMeasuredRatesEnabled;
	std::vector<double> rollSuppressLateralAccelMps2;
	std::vector<bool>   useTruthGravityModel;

	// --- Autopilot state (integrators, actuator memory) -----------------
	std::vector<double> pitchIntegral;
	std::vector<double> yawIntegral;
	std::vector<double> pitchCommandPrev;
	std::vector<double> yawCommandPrev;
	std::vector<double> rollCommandPrev;

	// --- Autopilot diagnostics ------------------------------------------
	std::vector<double> specificForceDemandY;   // body-Y specific-force demand
	std::vector<double> specificForceDemandZ;   // body-Z specific-force demand
	std::vector<double> effectiveKAccel;        // scheduled feed-forward gain
	std::vector<double> controlEffectiveness;   // Mach effectiveness factor
	std::vector<double> machNumber;             // estimated Mach
	std::vector<double> feedForwardPitch;       // demand breakdown (rad)
	std::vector<double> feedForwardYaw;
	std::vector<double> rateDampingPitch;
	std::vector<double> rateDampingYaw;
	std::vector<double> aoaDampingPitch;
	std::vector<double> aoaDampingYaw;
	std::vector<double> accelErrPitch;          // demand breakdown (rad)
	std::vector<double> accelErrYaw;
	std::vector<double> achievedSpecificForceY; // filtered body specific force
	std::vector<double> achievedSpecificForceZ;
	std::vector<double> authorityMargin01;      // delivered/demanded fin, <= 1

	// True when the commanded deflection hit the fin clamp.
	std::vector<bool> pitchSaturated;
	std::vector<bool> yawSaturated;
	std::vector<bool> rollSaturated;
};

} // namespace StrikeEngine::Kernel
