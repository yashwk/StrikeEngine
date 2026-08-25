#include "ParamSweep.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>

namespace StrikeEngine::Simulation {

    ParamSweep::ParamSweep(double timeStep_s, double maxTime_s)
        : dt(timeStep_s), maxTime(maxTime_s) {}

    void ParamSweep::execute(
        const Kernel::ScenarioConfig& baseConfig,
        double startValue,
        double endValue,
        int steps,
        std::function<void(Kernel::ScenarioConfig&, double)> applyParam,
        const std::string& outputFile)
    {
        std::ofstream out(outputFile);
        if (!out.is_open()) {
            std::cerr << "Failed to open output file: " << outputFile << std::endl;
            return;
        }

        // CSV Header
        out << "SweepValue,ImpactTime,ImpactX,ImpactY,ImpactZ,MaxAltitude,MaxVelocity\n";

        double stepSize = (steps > 1) ? (endValue - startValue) / (steps - 1) : 0.0;

        for (int i = 0; i < steps; ++i) {
            double currentParam = startValue + i * stepSize;
            
            // Create modified scenario config
            Kernel::ScenarioConfig config = baseConfig;
            applyParam(config, currentParam);

            // Initialize Kernel
            Kernel::SimulationKernel kernel;
            config.loadInto(kernel);

            const auto& physics = kernel.getPhysics();
            
            // Assuming single entity tracking for MVP sweep
            // For true multi-entity scenarios, we'd need to identify which entity we care about.
            // Here we assume entity 0 is the primary vehicle being analyzed.
            std::size_t targetId = 0;

            double maxAlt = 0.0;
            double maxVel = 0.0;

            while (kernel.getSimulationTime() <= maxTime) {
                if (!physics.active[targetId]) break; // Entity destroyed

                double alt = physics.pz[targetId];
                if (alt > maxAlt) maxAlt = alt;

                double vel = std::sqrt(physics.vx[targetId]*physics.vx[targetId] + 
                                       physics.vy[targetId]*physics.vy[targetId] + 
                                       physics.vz[targetId]*physics.vz[targetId]);
                if (vel > maxVel) maxVel = vel;

                // Stop if hitting ground (Z <= 0)
                if (kernel.getSimulationTime() > 1.0 && physics.pz[targetId] < 0.0) {
                    break;
                }

                kernel.step(dt);
            }

            // Write result row
            out << std::fixed << std::setprecision(4)
                << currentParam << ","
                << kernel.getSimulationTime() << ","
                << physics.px[targetId] << ","
                << physics.py[targetId] << ","
                << physics.pz[targetId] << ","
                << maxAlt << ","
                << maxVel << "\n";

            std::cout << "Sweep Step " << (i+1) << "/" << steps << " | Param: " << currentParam 
                      << " | Range: " << physics.px[targetId] << "m" << std::endl;
        }

        out.close();
        std::cout << "Parameter Sweep completed. Data saved to " << outputFile << std::endl;
    }

} // namespace StrikeEngine::Simulation
