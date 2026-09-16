#include <strikeengine/simulation/ParamSweep.hpp>
#include <strikeengine/simulation/Reporting.hpp>
#include <iostream>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

namespace StrikeEngine::Simulation {

    ParamSweep::ParamSweep(double timeStep_s, double maxTime_s)
        : dt(timeStep_s), maxTime(maxTime_s)
    {
        if (dt <= 0.0) {
            throw std::invalid_argument("ParamSweep timeStep_s must be positive");
        }
        if (maxTime < 0.0) {
            throw std::invalid_argument("ParamSweep maxTime_s cannot be negative");
        }
    }

    void ParamSweep::execute(
        const Kernel::ScenarioConfig& baseConfig,
        double startValue,
        double endValue,
            int steps,
            std::function<void(Kernel::ScenarioConfig&, double)> applyParam,
            const std::string& outputFile,
            const StudyOutputConfig& outputConfig)
    {
        if (steps < 0) {
            throw std::invalid_argument("ParamSweep steps cannot be negative");
        }

        std::vector<StudyOutputRecord> records;
        records.reserve(static_cast<std::size_t>(steps));

        double stepSize = (steps > 1) ? (endValue - startValue) / (steps - 1) : 0.0;

        for (int i = 0; i < steps; ++i) {
            double currentParam = startValue + i * stepSize;
            
            // Create modified scenario config
            Kernel::ScenarioConfig config = baseConfig;
            applyParam(config, currentParam);

            // Initialize Kernel. setRandomSeed MUST come after loadInto:
            // loadInto fans the scenario's static seed out to every kernel
            // stream, which would otherwise clobber the per-point seed and
            // leave the documented (seed + i) contract dead.
            Kernel::SimulationKernel kernel;
            config.loadInto(kernel);
            if (seedSet) {
                kernel.setRandomSeed(seed + static_cast<std::uint32_t>(i));
            }

            const auto& physics = kernel.getPhysics();

            const std::size_t targetId =
                resolvePrimaryEntityId(config, physics.size);

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
            // Fold the terminal state into the aggregates: a vehicle that
            // peaks on the step it impacts otherwise under-reports its maxima.
            maxAlt = std::max(maxAlt, finalState.altitudeM);
            maxVel = std::max(maxVel, finalState.speedMps);
            StudyOutputRecord record;
            record.scenarioIndex = static_cast<std::size_t>(i);
            record.sweepValue = currentParam;
            record.entityId = targetId;
            record.timeS = kernel.getSimulationTime();
            record.frame = finalState.frame;
            record.positionX = finalState.positionX;
            record.positionY = finalState.positionY;
            record.positionZ = finalState.positionZ;
            record.latitudeRad = finalState.latitudeRad;
            record.longitudeRad = finalState.longitudeRad;
            record.altitudeM = finalState.altitudeM;
            record.maxAltitudeM = std::isfinite(maxAlt) ? maxAlt : finalState.altitudeM;
            record.maxSpeedMps = maxVel;
            record.active = physics.active[targetId];
            record.status = !record.active
                ? StudyStatus::Impacted
                : (record.timeS >= maxTime ? StudyStatus::Completed : StudyStatus::Active);
            records.push_back(record);

            std::cout << "Sweep Step " << (i+1) << "/" << steps << " | Param: " << currentParam 
                      << " | PositionX: " << finalState.positionX << "m" << std::endl;
        }

        StudyOutputWriter::write(
            outputFile, StudyRecordType::ParameterSweep, records, outputConfig);
        std::cout << "Parameter Sweep completed. Data saved to " << outputFile << std::endl;
    }

} // namespace StrikeEngine::Simulation
