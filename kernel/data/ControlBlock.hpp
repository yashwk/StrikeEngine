#pragma once
#include <vector>
#include <cstddef>

namespace StrikeEngine::Kernel {

struct ControlBlock {
	std::vector<double> thrustCommand;
	std::vector<double> pitchCommand;
	std::vector<double> yawCommand;
	std::vector<double> rollCommand;
};

} // namespace StrikeEngine::Kernel
