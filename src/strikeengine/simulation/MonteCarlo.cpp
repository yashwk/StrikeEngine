#include <strikeengine/simulation/MonteCarlo.hpp>
#include <strikeengine/simulation/Reporting.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <stdexcept>

namespace StrikeEngine::Simulation {

    MonteCarlo::MonteCarlo(double timeStep_s, double maxTime_s)
        : dt(timeStep_s), maxTime(maxTime_s) {}

    void MonteCarlo::execute(
        const Kernel::ScenarioConfig& baseConfig,
        int iterations,
        std::function<void(Kernel::ScenarioConfig&, std::mt19937&)> perturbate,
        const std::string& outputFile)
    {
        if (iterations < 0) {
            throw std::invalid_argument("MonteCarlo iterations cannot be negative");
        }

        std::ofstream out(outputFile);
        if (!out.is_open()) {
            std::cerr << "Failed to open output file: " << outputFile << std::endl;
            return;
        }

        writeCsvMetadata(out, "monte_carlo");
        out << "Iteration,EntityId,Frame,EndTime_s,"
               "PositionX_m,PositionY_m,PositionZ_m,"
               "Latitude_rad,Longitude_rad,Altitude_m\n";

        // Initialize RNG
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::mt19937 generator(seed);

        for (int i = 0; i < iterations; ++i) {
            // Apply noise to base config
            Kernel::ScenarioConfig config = baseConfig;
            perturbate(config, generator);

            // Initialize Kernel
            Kernel::SimulationKernel kernel;
            config.loadInto(kernel);

            const auto& physics = kernel.getPhysics();
            if (config.entities.empty()) {
                throw std::invalid_argument("MonteCarlo scenario must contain an entity");
            }
            if (config.primaryEntityIndex >= config.entities.size()) {
                throw std::invalid_argument(
                    "MonteCarlo primaryEntityIndex is outside the scenario entity list");
            }
            const std::size_t targetId = config.primaryEntityIndex;

            while (kernel.getSimulationTime() <= maxTime) {
                if (!physics.active[targetId]) break;

                const double remaining = maxTime - kernel.getSimulationTime();
                if (remaining <= 0.0) break;
                kernel.step(std::min(dt, remaining));
            }

            const auto finalState = reportState(physics, targetId, config.environment);
            out << std::fixed << std::setprecision(9)
                << i << ","
                << targetId << ","
                << reportingFrameName(finalState.frame) << ","
                << kernel.getSimulationTime() << ","
                << finalState.positionX << ","
                << finalState.positionY << ","
                << finalState.positionZ << ","
                << finalState.latitudeRad << ","
                << finalState.longitudeRad << ","
                << finalState.altitudeM << "\n";

            // Print progress occasionally
            if ((i + 1) % 10 == 0 || i == iterations - 1) {
                std::cout << "Monte Carlo Iteration " << (i+1) << "/" << iterations << " completed." << std::endl;
            }
        }

        out.close();
        std::cout << "Monte Carlo Analysis completed. Data saved to " << outputFile << std::endl;
    }

} // namespace StrikeEngine::Simulation
