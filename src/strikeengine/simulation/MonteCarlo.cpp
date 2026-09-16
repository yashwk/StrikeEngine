#include <strikeengine/simulation/MonteCarlo.hpp>
#include <strikeengine/simulation/Reporting.hpp>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace StrikeEngine::Simulation {

    MonteCarlo::MonteCarlo(double timeStep_s, double maxTime_s)
        : dt(timeStep_s), maxTime(maxTime_s)
    {
        if (dt <= 0.0) {
            throw std::invalid_argument("MonteCarlo timeStep_s must be positive");
        }
        if (maxTime < 0.0) {
            throw std::invalid_argument("MonteCarlo maxTime_s cannot be negative");
        }
    }

    void MonteCarlo::execute(
        const Kernel::ScenarioConfig& baseConfig,
            int iterations,
            std::function<void(Kernel::ScenarioConfig&, std::mt19937&)> perturb,
            const std::string& outputFile,
            const StudyOutputConfig& outputConfig)
    {
        if (iterations < 0) {
            throw std::invalid_argument("MonteCarlo iterations cannot be negative");
        }

        std::vector<StudyOutputRecord> records;
        records.reserve(static_cast<std::size_t>(iterations));

        // Initialize RNG: pinned seed when setSeed was used, wall clock
        // otherwise (legacy behavior).
        unsigned useed = seedSet
            ? seed
            : static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
        std::mt19937 generator(useed);

        for (int i = 0; i < iterations; ++i) {
            // Draw the iteration kernel seed first so kernel streams stay
            // stable even if the perturb callback changes its draw count.
            const std::uint32_t kernelSeed = static_cast<std::uint32_t>(generator());
            // Apply noise to base config
            Kernel::ScenarioConfig config = baseConfig;
            perturb(config, generator);

            // Initialize Kernel. setRandomSeed MUST come after loadInto:
            // loadInto fans the scenario's static seed out to every kernel
            // stream, which would otherwise clobber the per-iteration kernel
            // seed and make every iteration share one noise stream.
            Kernel::SimulationKernel kernel;
            config.loadInto(kernel);
            kernel.setRandomSeed(kernelSeed);

            const auto& physics = kernel.getPhysics();
            const std::size_t targetId =
                resolvePrimaryEntityId(config, physics.size);

            while (kernel.getSimulationTime() <= maxTime) {
                if (!physics.active[targetId]) break;

                const double remaining = maxTime - kernel.getSimulationTime();
                if (remaining <= 0.0) break;
                kernel.step(std::min(dt, remaining));
            }

            const auto finalState = reportState(physics, targetId, config.environment);
            StudyOutputRecord record;
            record.iteration = static_cast<std::size_t>(i);
            record.entityId = targetId;
            record.timeS = kernel.getSimulationTime();
            record.frame = finalState.frame;
            record.positionX = finalState.positionX;
            record.positionY = finalState.positionY;
            record.positionZ = finalState.positionZ;
            record.latitudeRad = finalState.latitudeRad;
            record.longitudeRad = finalState.longitudeRad;
            record.altitudeM = finalState.altitudeM;
            record.active = physics.active[targetId];
            record.status = !record.active
                ? StudyStatus::Impacted
                : (record.timeS >= maxTime ? StudyStatus::Completed : StudyStatus::Active);
            records.push_back(record);

            // Print progress occasionally
            if ((i + 1) % 10 == 0 || i == iterations - 1) {
                std::cout << "Monte Carlo Iteration " << (i+1) << "/" << iterations << " completed." << std::endl;
            }
        }

        StudyOutputWriter::write(
            outputFile, StudyRecordType::MonteCarlo, records, outputConfig);
        std::cout << "Monte Carlo Analysis completed. Data saved to " << outputFile << std::endl;
    }

} // namespace StrikeEngine::Simulation
