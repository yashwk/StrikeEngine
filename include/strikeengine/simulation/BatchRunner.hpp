#pragma once

#include <strikeengine/kernel/config/ScenarioConfig.hpp>

#include <cstddef>
#include <vector>

namespace StrikeEngine::Simulation {

    struct BatchRunResult {
        std::size_t scenarioIndex = 0;
        double endTime = 0.0;
        std::size_t entityCount = 0;
        std::size_t activeEntities = 0;
        double maxAltitude = 0.0;
        double maxSpeed = 0.0;
    };

    /**
     * @brief Executes a collection of scenarios and returns structured results.
     *
     * Each scenario is loaded into a fresh kernel, so environment, vehicle,
     * guidance, and random state cannot leak between batch members.
     */
    class BatchRunner {
    public:
        BatchRunner(double timeStep_s, double maxTime_s);

        std::vector<BatchRunResult> execute(
            const std::vector<Kernel::ScenarioConfig>& scenarios) const;

    private:
        double dt;
        double maxTime;
    };

} // namespace StrikeEngine::Simulation
