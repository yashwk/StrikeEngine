#include <strikeengine/simulation/MonteCarlo.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>

namespace StrikeEngine::Simulation {

    MonteCarlo::MonteCarlo(double timeStep_s, double maxTime_s)
        : dt(timeStep_s), maxTime(maxTime_s) {}

    void MonteCarlo::execute(
        const Kernel::ScenarioConfig& baseConfig,
        int iterations,
        std::function<void(Kernel::ScenarioConfig&, std::mt19937&)> perturbate,
        const std::string& outputFile)
    {
        std::ofstream out(outputFile);
        if (!out.is_open()) {
            std::cerr << "Failed to open output file: " << outputFile << std::endl;
            return;
        }

        // CSV Header
        out << "Iteration,ImpactTime,ImpactX,ImpactY,ImpactZ\n";

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
            std::size_t targetId = 0; // Assuming MVP single primary entity

            while (kernel.getSimulationTime() <= maxTime) {
                if (!physics.active[targetId]) break;

                // Stop if hitting ground
                if (kernel.getSimulationTime() > 1.0 && physics.pz[targetId] < 0.0) {
                    break;
                }

                kernel.step(dt);
            }

            // Write result row
            out << std::fixed << std::setprecision(4)
                << i << ","
                << kernel.getSimulationTime() << ","
                << physics.px[targetId] << ","
                << physics.py[targetId] << ","
                << physics.pz[targetId] << "\n";

            // Print progress occasionally
            if ((i + 1) % 10 == 0 || i == iterations - 1) {
                std::cout << "Monte Carlo Iteration " << (i+1) << "/" << iterations << " completed." << std::endl;
            }
        }

        out.close();
        std::cout << "Monte Carlo Analysis completed. Data saved to " << outputFile << std::endl;
    }

} // namespace StrikeEngine::Simulation
