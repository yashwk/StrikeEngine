#pragma once
#include <vector>
#include <cstdint>

namespace StrikeEngine::Kernel {

    enum class GuidanceMode : uint8_t {
        None,                   // Ballistic or Uncontrolled
        ProportionalNavigation, // ProNav interception
        Waypoint                // Navigating to static point
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

        // Guidance demand magnitude limit (m/s^2); 0 = unlimited.
        // Set per entity via SimulationCommand::maxAccel.
        std::vector<double> maxAccel;

        // Output: Required acceleration command
        std::vector<double> commandedAccelX;
        std::vector<double> commandedAccelY;
        std::vector<double> commandedAccelZ;
    };

} // namespace StrikeEngine::Kernel