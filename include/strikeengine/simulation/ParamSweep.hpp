#pragma once

#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/ScenarioConfig.hpp>
#include <strikeengine/simulation/StudyOutput.hpp>
#include <string>
#include <functional>
#include <vector>
#include <cstddef>
#include <cstdint>

namespace StrikeEngine::Simulation {

    // Represents a single frame-aware outcome from a sweep run.
    struct SimulationResult {
        double sweepValue = 0.0;
        std::size_t entityId = 0;
        double endTime = 0.0;
        double positionX = 0.0;
        double positionY = 0.0;
        double positionZ = 0.0;
        double latitudeRad = 0.0;
        double longitudeRad = 0.0;
        double altitudeM = 0.0;
        double maxAltitudeM = 0.0;
        double maxSpeedMps = 0.0;
        std::string frame = "LOCAL_ENU";

        // Legacy field names retained for source compatibility with callers
        // that used this internal result shape before the versioned CSV output.
        double impactTime = 0.0;
        double impactX = 0.0;
        double impactY = 0.0;
        double impactZ = 0.0;
        double maxAltitude = 0.0;
        double maxVelocity = 0.0;
    };

    class ParamSweep {
    public:
        ParamSweep(double timeStep_s, double maxTime_s);

        /**
         * @brief Pins the per-point kernel sensor stream for reproducible runs.
         *
         * Point i is seeded with (seed + i). Without it kernels draw
         * chrono-seeded sensor noise.
         */
        void setSeed(std::uint32_t s) { seedSet = true; seed = s; }

        /**
         * @brief Runs a parameter sweep.
         * @param baseConfig The base scenario configuration.
         * @param startValue Sweep start value.
         * @param endValue Sweep end value.
         * @param steps Number of steps to sweep.
         * @param applyParam Callback that modifies the baseConfig for the current sweep value.
         * @param outputFile CSV file to output the results.
         */
        void execute(
            const Kernel::ScenarioConfig& baseConfig,
            double startValue,
            double endValue,
            int steps,
            std::function<void(Kernel::ScenarioConfig&, double)> applyParam,
            const std::string& outputFile,
            const StudyOutputConfig& outputConfig = {}
        );

    private:
        double dt;
        double maxTime;
        bool seedSet = false;
        std::uint32_t seed = 0u;
    };

} // namespace StrikeEngine::Simulation
