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

	// W36 diagnostics: true when the commanded deflection hit the fin clamp.
	std::vector<bool> pitchSaturated;
	std::vector<bool> yawSaturated;
	std::vector<bool> rollSaturated;
};

} // namespace StrikeEngine::Kernel
