#pragma once

#include "../kernel/SimulationKernel.hpp"
#include "../kernel/config/ScenarioConfig.hpp"
#include <string>
#include <functional>
#include <vector>
#include <random>

namespace StrikeEngine::Simulation {

    class MonteCarlo {
    public:
        MonteCarlo(double timeStep_s, double maxTime_s);

        /**
         * @brief Runs a Monte Carlo statistical analysis.
         * @param baseConfig The nominal scenario configuration.
         * @param iterations Number of simulation runs.
         * @param perturbate Callback to apply random noise to the config before each run.
         * @param outputFile CSV file to output the results of all iterations.
         */
        void execute(
            const Kernel::ScenarioConfig& baseConfig,
            int iterations,
            std::function<void(Kernel::ScenarioConfig&, std::mt19937&)> perturbate,
            const std::string& outputFile
        );

    private:
        double dt;
        double maxTime;
    };

} // namespace StrikeEngine::Simulation
