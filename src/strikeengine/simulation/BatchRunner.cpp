#include <strikeengine/simulation/BatchRunner.hpp>
#include <strikeengine/simulation/Reporting.hpp>

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
            result.frame = reportingFrameName(reportingFrame(
                scenarios[scenarioIndex].environment));

            if (result.entityCount > 0 &&
                scenarios[scenarioIndex].primaryEntityIndex >= result.entityCount) {
                throw std::invalid_argument(
                    "BatchRunner primaryEntityIndex is outside the scenario entity list");
            }
            result.primaryEntityId = scenarios[scenarioIndex].primaryEntityIndex;

            const auto& physics = kernel.getPhysics();

            while (kernel.getSimulationTime() < maxTime) {
                bool anyActive = false;
                for (std::size_t id = 0; id < physics.size; ++id) {
                    anyActive = anyActive || physics.active[id];
                }
                if (!anyActive) break;

                // Track peak trajectory metrics from the pre-step state so the
                // initial condition and every post-step state are covered.
                for (std::size_t id = 0; id < physics.size; ++id) {
                    const auto state = reportState(
                        physics, id, scenarios[scenarioIndex].environment);
                    result.maxAltitudeM = std::max(result.maxAltitudeM, state.altitudeM);
                    result.maxSpeedMps = std::max(result.maxSpeedMps, state.speedMps);
                }

                const double stepDt = std::min(
                    dt, maxTime - kernel.getSimulationTime());
                kernel.step(stepDt);
            }

            result.endTime = kernel.getSimulationTime();
            for (std::size_t id = 0; id < physics.size; ++id) {
                if (physics.active[id]) ++result.activeEntities;
                const auto state = reportState(
                    physics, id, scenarios[scenarioIndex].environment);
                result.maxAltitudeM = std::max(result.maxAltitudeM, state.altitudeM);
                result.maxSpeedMps = std::max(result.maxSpeedMps, state.speedMps);
            }

            if (result.entityCount > 0) {
                const auto primaryState = reportState(
                    physics,
                    result.primaryEntityId,
                    scenarios[scenarioIndex].environment);
                result.finalPositionX = primaryState.positionX;
                result.finalPositionY = primaryState.positionY;
                result.finalPositionZ = primaryState.positionZ;
                result.finalLatitudeRad = primaryState.latitudeRad;
                result.finalLongitudeRad = primaryState.longitudeRad;
                result.finalAltitudeM = primaryState.altitudeM;
                result.primaryEntityActive = physics.active[result.primaryEntityId];
            }
            result.status = result.primaryEntityActive ? "COMPLETED" : "IMPACTED";
            result.maxAltitude = result.maxAltitudeM;
            result.maxSpeed = result.maxSpeedMps;
            results.push_back(result);
        }
        return results;
    }

} // namespace StrikeEngine::Simulation
