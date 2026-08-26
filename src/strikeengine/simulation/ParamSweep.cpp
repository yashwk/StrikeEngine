#include <strikeengine/simulation/ParamSweep.hpp>
#include <strikeengine/simulation/Reporting.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>

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
        if (steps < 0) {
            throw std::invalid_argument("ParamSweep steps cannot be negative");
        }

        std::ofstream out(outputFile);
        if (!out.is_open()) {
            std::cerr << "Failed to open output file: " << outputFile << std::endl;
            return;
        }

        writeCsvMetadata(out, "parameter_sweep");
        out << "SweepValue,EntityId,Frame,EndTime_s,"
               "PositionX_m,PositionY_m,PositionZ_m,"
               "Latitude_rad,Longitude_rad,Altitude_m,"
               "MaxAltitude_m,MaxSpeed_mps\n";

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

            if (config.entities.empty()) {
                throw std::invalid_argument("ParamSweep scenario must contain an entity");
            }
            if (config.primaryEntityIndex >= config.entities.size()) {
                throw std::invalid_argument(
                    "ParamSweep primaryEntityIndex is outside the scenario entity list");
            }
            const std::size_t targetId = config.primaryEntityIndex;

            double maxAlt = -std::numeric_limits<double>::infinity();
            double maxVel = 0.0;

            while (kernel.getSimulationTime() <= maxTime) {
                if (!physics.active[targetId]) break; // Entity destroyed

                const auto state = reportState(physics, targetId, config.environment);
                maxAlt = std::max(maxAlt, state.altitudeM);
                maxVel = std::max(maxVel, state.speedMps);

                const double remaining = maxTime - kernel.getSimulationTime();
                if (remaining <= 0.0) break;
                kernel.step(std::min(dt, remaining));
            }

            const auto finalState = reportState(physics, targetId, config.environment);
            out << std::fixed << std::setprecision(9)
                << currentParam << ","
                << targetId << ","
                << reportingFrameName(finalState.frame) << ","
                << kernel.getSimulationTime() << ","
                << finalState.positionX << ","
                << finalState.positionY << ","
                << finalState.positionZ << ","
                << finalState.latitudeRad << ","
                << finalState.longitudeRad << ","
                << finalState.altitudeM << ","
                << (std::isfinite(maxAlt) ? maxAlt : finalState.altitudeM) << ","
                << maxVel << "\n";

            std::cout << "Sweep Step " << (i+1) << "/" << steps << " | Param: " << currentParam 
                      << " | PositionX: " << finalState.positionX << "m" << std::endl;
        }

        out.close();
        std::cout << "Parameter Sweep completed. Data saved to " << outputFile << std::endl;
    }

} // namespace StrikeEngine::Simulation
