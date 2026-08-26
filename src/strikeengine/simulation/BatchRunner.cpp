#include <strikeengine/simulation/BatchRunner.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace StrikeEngine::Simulation {

    BatchRunner::BatchRunner(double timeStep_s, double maxTime_s)
        : dt(timeStep_s), maxTime(maxTime_s)
    {
        if (dt <= 0.0) {
            throw std::invalid_argument("BatchRunner timestep must be positive");
        }
        if (maxTime < 0.0) {
            throw std::invalid_argument("BatchRunner max time cannot be negative");
        }
    }

    std::vector<BatchRunResult> BatchRunner::execute(
        const std::vector<Kernel::ScenarioConfig>& scenarios) const
    {
        std::vector<BatchRunResult> results;
        results.reserve(scenarios.size());

        for (std::size_t scenarioIndex = 0; scenarioIndex < scenarios.size(); ++scenarioIndex) {
            Kernel::SimulationKernel kernel;
            scenarios[scenarioIndex].loadInto(kernel);

            BatchRunResult result;
            result.scenarioIndex = scenarioIndex;
            result.entityCount = kernel.getPhysics().size;

            while (kernel.getSimulationTime() < maxTime) {
                bool anyActive = false;
                for (std::size_t id = 0; id < kernel.getPhysics().size; ++id) {
                    anyActive = anyActive || kernel.getPhysics().active[id];
                }
                if (!anyActive) break;
                const double stepDt = std::min(
                    dt, maxTime - kernel.getSimulationTime());
                kernel.step(stepDt);
            }

            const auto& physics = kernel.getPhysics();
            result.endTime = kernel.getSimulationTime();
            for (std::size_t id = 0; id < physics.size; ++id) {
                if (physics.active[id]) ++result.activeEntities;
                result.maxAltitude = std::max(result.maxAltitude, physics.pz[id]);
                const double speed = std::sqrt(
                    physics.vx[id] * physics.vx[id] +
                    physics.vy[id] * physics.vy[id] +
                    physics.vz[id] * physics.vz[id]);
                result.maxSpeed = std::max(result.maxSpeed, speed);
            }
            results.push_back(result);
        }
        return results;
    }

} // namespace StrikeEngine::Simulation
