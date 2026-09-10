#pragma once

#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/ScenarioConfig.hpp>
#include <strikeengine/simulation/StudyOutput.hpp>
#include <string>
#include <cstdint>
#include <functional>
#include <vector>
#include <random>

namespace StrikeEngine::Simulation {

    class MonteCarlo {
    public:
        MonteCarlo(double timeStep_s, double maxTime_s);

        /**
         * @brief Pins the study RNG for reproducible runs.
         *
         * Seeds both the perturbation stream and every iteration kernel's
         * sensor stream. Without it the study seeds from the wall clock and
         * kernels draw chrono-seeded sensor noise (not reproducible).
         */
        void setSeed(std::uint32_t s) { seedSet = true; seed = s; }

        /**
         * @brief Runs a Monte Carlo statistical analysis.
         * @param baseConfig The nominal scenario configuration.
         * @param iterations Number of simulation runs.
         * @param perturb Callback to apply random noise to the config before each run.
         * @param outputFile CSV file to output the results of all iterations.
         */
        void execute(
            const Kernel::ScenarioConfig& baseConfig,
            int iterations,
            std::function<void(Kernel::ScenarioConfig&, std::mt19937&)> perturb,
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
