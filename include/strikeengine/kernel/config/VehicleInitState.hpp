#pragma once

#include <cstddef>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>

namespace StrikeEngine::Kernel {

    // Initial truth state for a spawned entity. Extracted from
    // SimulationKernel.hpp so scenario/config headers can reference it
    // without pulling in the full kernel (include-cycle with
    // ScenarioConfig.hpp).
    struct VehicleInitState {
        double px = 0.0, py = 0.0, pz = 0.0;
        double vx = 0.0, vy = 0.0, vz = 0.0;
        double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
        double wx = 0.0, wy = 0.0, wz = 0.0;
        double mass = 0.0;
        Allegiance allegiance = Allegiance::Friendly;
    };

} // namespace StrikeEngine::Kernel
