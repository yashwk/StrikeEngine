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
		thrustCommand.resize(n, 0.0);
		pitchCommand.resize(n, 0.0);
		yawCommand.resize(n, 0.0);
		rollCommand.resize(n, 0.0);
		thrustVectorPitchCommand.resize(n, 0.0);
		thrustVectorYawCommand.resize(n, 0.0);
		kAccelP.resize(n, 0.030);
		kRateP.resize(n, 1.000);
		kAlphaP.resize(n, 0.200);
		kRollP.resize(n, 0.10);
		kRollD.resize(n, 0.05);
		maxDeflectionRad.resize(n, 0.43);
		gainSchedulingEnabled.resize(n, false);
		refDynamicPressurePa.resize(n, 50000.0);
		minDynamicPressurePa.resize(n, 2000.0);
		maxDynamicPressurePa.resize(n, 300000.0);
		kRatePitchP.resize(n, -1.0);
		kRateYawP.resize(n, -1.0);
		scheduleAllTerms.resize(n, false);
		integralEnabled.resize(n, false);
		kIntegralPitch.resize(n, 0.0);
		kIntegralYaw.resize(n, 0.0);
		integralClampRad.resize(n, 0.05);
		kAccelErrP.resize(n, -1.0);
		threeLoopEnabled.resize(n, false);
		controlEffectivenessEnabled.resize(n, false);
		controlEffBase.resize(n, 1.0);
		controlEffMachSlope.resize(n, 0.0);
		controlEffMachQuad.resize(n, 0.0);
		controlEffMin.resize(n, 0.2);
		controlEffMax.resize(n, 5.0);
		yawDeadbandSmoothEnabled.resize(n, false);
		yawDeadbandWidthMps2.resize(n, 0.5);
		commandLagSec.resize(n, 0.0);
		commandRateLimitRadPerSec.resize(n, 0.0);
		useMeasuredRatesEnabled.resize(n, false);
		rollSuppressLateralAccelMps2.resize(n, 0.0);
		useTruthGravityModel.resize(n, false);
		pitchIntegral.resize(n, 0.0);
		yawIntegral.resize(n, 0.0);
		pitchCommandPrev.resize(n, 0.0);
		yawCommandPrev.resize(n, 0.0);
		rollCommandPrev.resize(n, 0.0);
		specificForceDemandY.resize(n, 0.0);
		specificForceDemandZ.resize(n, 0.0);
		effectiveKAccel.resize(n, 0.030);
		controlEffectiveness.resize(n, 1.0);
		machNumber.resize(n, 0.0);
		feedForwardPitch.resize(n, 0.0);
		feedForwardYaw.resize(n, 0.0);
		rateDampingPitch.resize(n, 0.0);
		rateDampingYaw.resize(n, 0.0);
		aoaDampingPitch.resize(n, 0.0);
		aoaDampingYaw.resize(n, 0.0);
		accelErrPitch.resize(n, 0.0);
		accelErrYaw.resize(n, 0.0);
		rateCommandPitch.resize(n, 0.0);
		rateCommandYaw.resize(n, 0.0);
		achievedSpecificForceY.resize(n, 0.0);
		achievedSpecificForceZ.resize(n, 0.0);
		authorityMargin01.resize(n, 1.0);
		pitchSaturated.resize(n, false);
		yawSaturated.resize(n, false);
		rollSaturated.resize(n, false);
		size = n;
	}
};

} // namespace StrikeEngine::Kernel
