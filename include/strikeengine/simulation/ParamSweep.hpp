#pragma once

#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/ScenarioConfig.hpp>
#include <string>
#include <functional>
#include <vector>

namespace StrikeEngine::Simulation {

    // Represents a single outcome from a simulation run
    struct SimulationResult {
        double sweepValue;
        double impactTime;
        double impactX;
        double impactY;
        double impactZ;
        double maxAltitude;
        double maxVelocity;
    };

    class ParamSweep {
    public:
        ParamSweep(double timeStep_s, double maxTime_s);

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
            const std::string& outputFile
        );

    private:
        double dt;
        double maxTime;
    };

} // namespace StrikeEngine::Simulation
