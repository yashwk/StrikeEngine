#pragma once

#include <vector>
#include <strikeengine/models/physics/propulsion/ThrustCurve.hpp>

namespace StrikeEngine::Kernel {

    struct StageConfig {
        std::vector<Models::ThrustDataPoint> thrustCurve;  // empty => inactive/coast stage
        double vacuumIsp = 250.0;   // s
        double seaLevelIsp = 220.0; // s
        double propellantMassKg = 0.0;
        double dryMassKg = 0.0;      // dropped at separation (future)
    };

    struct PropulsionConfig {
        std::vector<StageConfig> stages;  // empty => coasting vehicle
    };

} // namespace StrikeEngine::Kernel
