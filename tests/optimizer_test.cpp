#include "../simulation/Optimizer.hpp"
#include <iostream>
#include <cmath>

using namespace StrikeEngine;

int main() {
    std::cout << "======================================\n";
    std::cout << "      StrikeEngine PSO Optimizer      \n";
    std::cout << "======================================\n\n";

    Simulation::Optimizer optimizer(0.01, 100.0);

    // We want to optimize a single parameter: Launch Pitch Angle (Degrees)
    // Parameter 0: Launch Angle (10.0 to 80.0 degrees)
    optimizer.addParameter(10.0, 80.0);

    // We want to maximize the horizontal range (X-axis) at the time of impact.
    auto applyParams = [](std::size_t id, const std::vector<double>& params, Kernel::VehicleInitState& init) {
        init.px = 0.0; init.py = 0.0; init.pz = 100.0; // Start at 100m altitude
        
        double speed = 300.0; // Initial speed
        double pitchRad = params[0] * (3.1415926535 / 180.0);

        init.vx = speed * std::cos(pitchRad);
        init.vy = 0.0;
        init.vz = speed * std::sin(pitchRad);

        init.qx = 0; init.qy = 0; init.qz = 0; init.qw = 1;
        init.wx = 0; init.wy = 0; init.wz = 0;
        init.mass = 250.0;
    };

    auto fitnessFunc = [](std::size_t id, const Kernel::SimulationKernel& kernel) {
        const auto& phys = kernel.getPhysics();
        // Since we want to maximize range, fitness is simply the final X position!
        return phys.px[id];
    };

    std::cout << "Starting Particle Swarm Optimization...\n";
    std::cout << "Swarm Size: 50 | Iterations: 10\n\n";

    auto result = optimizer.optimize(50, 10, applyParams, fitnessFunc);

    std::cout << "\n======================================\n";
    std::cout << "Optimization Complete!\n";
    std::cout << "Best Fitness (Max Range): " << result.bestFitness << " m\n";
    std::cout << "Optimal Launch Angle: " << result.bestParameters[0] << " degrees\n";
    std::cout << "======================================\n";

    return 0;
}
