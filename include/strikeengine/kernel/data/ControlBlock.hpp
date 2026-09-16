#pragma once
#include <strikeengine/kernel/data/BlockGrowth.hpp>
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
	std::vector<bool>   threeLoopEnabled;
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
	std::vector<double> rateCommandPitch;       // three-loop outer-loop output (rad/s)
	std::vector<double> rateCommandYaw;
	std::vector<double> achievedSpecificForceY; // filtered body specific force
	std::vector<double> achievedSpecificForceZ;
	std::vector<double> authorityMargin01;      // delivered/demanded fin, <= 1

	// True when the commanded deflection hit the fin clamp.
	std::vector<bool> pitchSaturated;
	std::vector<bool> yawSaturated;
	std::vector<bool> rollSaturated;

	std::size_t size = 0;

	/**
	 * @brief Grows every vector to @p n entries; see PhysicsBlock::ensureSize.
	 *
	 * Gains set here are block defaults. The kernel overwrites them from the
	 * vehicle configuration once the slot exists.
	 */
	void ensureSize(std::size_t n) {
		growTo(thrustCommand, n, 0.0);
		growTo(pitchCommand, n, 0.0);
		growTo(yawCommand, n, 0.0);
		growTo(rollCommand, n, 0.0);
		growTo(thrustVectorPitchCommand, n, 0.0);
		growTo(thrustVectorYawCommand, n, 0.0);
		growTo(kAccelP, n, 0.030);
		growTo(kRateP, n, 1.000);
		growTo(kAlphaP, n, 0.200);
		growTo(kRollP, n, 0.10);
		growTo(kRollD, n, 0.05);
		growTo(maxDeflectionRad, n, 0.43);
		growTo(gainSchedulingEnabled, n, false);
		growTo(refDynamicPressurePa, n, 50000.0);
		growTo(minDynamicPressurePa, n, 2000.0);
		growTo(maxDynamicPressurePa, n, 300000.0);
		growTo(kRatePitchP, n, -1.0);
		growTo(kRateYawP, n, -1.0);
		growTo(scheduleAllTerms, n, false);
		growTo(integralEnabled, n, false);
		growTo(kIntegralPitch, n, 0.0);
		growTo(kIntegralYaw, n, 0.0);
		growTo(integralClampRad, n, 0.05);
		growTo(kAccelErrP, n, -1.0);
		growTo(threeLoopEnabled, n, false);
		growTo(controlEffectivenessEnabled, n, false);
		growTo(controlEffBase, n, 1.0);
		growTo(controlEffMachSlope, n, 0.0);
		growTo(controlEffMachQuad, n, 0.0);
		growTo(controlEffMin, n, 0.2);
		growTo(controlEffMax, n, 5.0);
		growTo(yawDeadbandSmoothEnabled, n, false);
		growTo(yawDeadbandWidthMps2, n, 0.5);
		growTo(commandLagSec, n, 0.0);
		growTo(commandRateLimitRadPerSec, n, 0.0);
		growTo(useMeasuredRatesEnabled, n, false);
		growTo(rollSuppressLateralAccelMps2, n, 0.0);
		growTo(useTruthGravityModel, n, false);
		growTo(pitchIntegral, n, 0.0);
		growTo(yawIntegral, n, 0.0);
		growTo(pitchCommandPrev, n, 0.0);
		growTo(yawCommandPrev, n, 0.0);
		growTo(rollCommandPrev, n, 0.0);
		growTo(specificForceDemandY, n, 0.0);
		growTo(specificForceDemandZ, n, 0.0);
		growTo(effectiveKAccel, n, 0.030);
		growTo(controlEffectiveness, n, 1.0);
		growTo(machNumber, n, 0.0);
		growTo(feedForwardPitch, n, 0.0);
		growTo(feedForwardYaw, n, 0.0);
		growTo(rateDampingPitch, n, 0.0);
		growTo(rateDampingYaw, n, 0.0);
		growTo(aoaDampingPitch, n, 0.0);
		growTo(aoaDampingYaw, n, 0.0);
		growTo(accelErrPitch, n, 0.0);
		growTo(accelErrYaw, n, 0.0);
		growTo(rateCommandPitch, n, 0.0);
		growTo(rateCommandYaw, n, 0.0);
		growTo(achievedSpecificForceY, n, 0.0);
		growTo(achievedSpecificForceZ, n, 0.0);
		growTo(authorityMargin01, n, 1.0);
		growTo(pitchSaturated, n, false);
		growTo(yawSaturated, n, false);
		growTo(rollSaturated, n, false);
		if (n > size) size = n;
	}
};

} // namespace StrikeEngine::Kernel
