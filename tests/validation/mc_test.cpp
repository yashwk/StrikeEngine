#include <strikeengine/simulation/MonteCarlo.hpp>
#include <iostream>

int main() {
    using namespace StrikeEngine;

    Simulation::MonteCarlo mc(0.01, 100.0); // 100Hz, max 100s

    Kernel::ScenarioConfig baseConfig;
    baseConfig.name = "Monte Carlo Targeting";
    
    Kernel::ScenarioEntityConfig entity;
    entity.initState.px = 0.0;
    entity.initState.py = 0.0;
    entity.initState.pz = 1000.0; 
    
    entity.initState.vx = 200.0;
    entity.initState.vy = 0.0;
    entity.initState.vz = 0.0;
    
    entity.initState.qx = 0.0; entity.initState.qy = 0.0; entity.initState.qz = 0.0; entity.initState.qw = 1.0;
    entity.initState.wx = 0.0; entity.initState.wy = 0.0; entity.initState.wz = 0.0;
    entity.initState.mass = 500.0;

    // Use Guidance
    entity.initialGuidanceMode = Kernel::GuidanceMode::ProportionalNavigation;
    entity.initialTargetX = 5000.0;
    entity.initialTargetY = 0.0;
    entity.initialTargetZ = 0.0;

    baseConfig.entities.push_back(entity);

    auto applyNoise = [](Kernel::ScenarioConfig& cfg, std::mt19937& gen) {
        // Standard deviations
        std::normal_distribution<double> velNoise(0.0, 10.0); // 10 m/s stddev
        std::normal_distribution<double> massNoise(0.0, 5.0); // 5 kg stddev
        
        cfg.entities[0].initState.vx += velNoise(gen);
        cfg.entities[0].initState.vy += velNoise(gen);
        cfg.entities[0].initState.vz += velNoise(gen);
        
        cfg.entities[0].initState.mass += massNoise(gen);
    };

    std::cout << "Starting Monte Carlo Analysis (100 iterations)...\n";
    mc.execute(baseConfig, 100, applyNoise, "mc_output.csv");

    return 0;
}
