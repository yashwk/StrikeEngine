#include <strikeengine/simulation/ParamSweep.hpp>
#include <iostream>
#include <cmath>

int main() {
    using namespace StrikeEngine;

    Simulation::ParamSweep sweep(0.01, 100.0); // 100Hz, max 100s

    Kernel::ScenarioConfig baseConfig;
    baseConfig.name = "Launch Angle Sweep";
    
    Kernel::ScenarioEntityConfig entity;
    entity.initState.px = 0.0;
    entity.initState.py = 0.0;
    entity.initState.pz = 0.0; // Ground launch
    
    // Will be overridden by sweep
    entity.initState.vx = 0.0;
    entity.initState.vy = 0.0;
    entity.initState.vz = 0.0;
    
    entity.initState.qx = 0.0; entity.initState.qy = 0.0; entity.initState.qz = 0.0; entity.initState.qw = 1.0;
    entity.initState.wx = 0.0; entity.initState.wy = 0.0; entity.initState.wz = 0.0;
    entity.initState.mass = 500.0;

    baseConfig.entities.push_back(entity);

    auto applyAngle = [](Kernel::ScenarioConfig& cfg, double angleDeg) {
        // Assume constant launch speed of 200 m/s
        double speed = 200.0;
        double angleRad = angleDeg * 3.14159265 / 180.0;
        
        cfg.entities[0].initState.vx = speed * std::cos(angleRad);
        cfg.entities[0].initState.vz = speed * std::sin(angleRad);
    };

    std::cout << "Starting Parameter Sweep (Launch Angle 10 to 80 deg)...\n";
    
    // Sweep from 10 degrees to 80 degrees in 15 steps
    sweep.execute(baseConfig, 10.0, 80.0, 15, applyAngle, "sweep_output.csv");

    return 0;
}
