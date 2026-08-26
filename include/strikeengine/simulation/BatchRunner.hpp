#pragma once

#include <strikeengine/kernel/config/ScenarioConfig.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace StrikeEngine::Simulation {

    struct BatchRunResult {
        std::size_t scenarioIndex = 0;
        std::size_t primaryEntityId = 0;
        double endTime = 0.0;
        std::size_t entityCount = 0;
        std::size_t activeEntities = 0;
        double finalPositionX = 0.0;
        double finalPositionY = 0.0;
        double finalPositionZ = 0.0;
        double finalLatitudeRad = 0.0;
        double finalLongitudeRad = 0.0;
        double finalAltitudeM = 0.0;
        double maxAltitudeM = 0.0;
        double maxSpeedMps = 0.0;
        std::string frame = "LOCAL_ENU";
        bool primaryEntityActive = false;

        // Legacy aliases retained for source compatibility. New code should
        // use the unit-suffixed fields above.
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
