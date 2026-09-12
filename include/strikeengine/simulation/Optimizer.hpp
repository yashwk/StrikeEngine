#pragma once

#include <strikeengine/kernel/SimulationKernel.hpp>
#include <vector>
#include <functional>
#include <cstddef>
#include <cstdint>

namespace StrikeEngine::Simulation {

    struct OptimizationResult {
        std::vector<double> bestParameters;
        double bestFitness;
        int iterations;
    };

    struct ParameterBound {
        double minVal;
        double maxVal;
    };

    class Optimizer {
    public:
        Optimizer(double timeStep_s, double maxTime_s);

        /**
         * @brief Pin the PSO drive stream (particle initialization and the
         * r1/r2 velocity draws). Without it the stream is seeded from the
         * wall clock and optimization runs are not reproducible. Mirrors
         * MonteCarlo::setSeed / ParamSweep::setSeed.
         */
        void setSeed(std::uint32_t seed);

        // Add a parameter bounds to the optimizer space
        void addParameter(double minBound, double maxBound);

        // Clear parameter bounds
        void clearParameters();

        /**
         * @brief Runs Particle Swarm Optimization leveraging SoA concurrency.
         * @param swarmSize Number of particles (vehicles) simulated simultaneously.
         * @param maxIterations Number of generations.
         * @param applyParamsFunc Maps array of generic parameters into vehicle/physics initialization.
         * @param fitnessFunc Evaluates the vehicle's outcome score. Higher is better!
         * @return Best parameters and fitness achieved.
         */
        OptimizationResult optimize(
            std::size_t swarmSize,
            int maxIterations,
            std::function<void(std::size_t vehicleId, const std::vector<double>& params, Kernel::VehicleInitState& init)> applyParamsFunc,
            std::function<double(std::size_t vehicleId, const Kernel::SimulationKernel& kernel)> fitnessFunc
        );

    private:
        double dt;
        double maxTime;
        std::vector<ParameterBound> bounds;
        std::uint32_t seed = 0u;
        bool seedSet = false;
    };

} // namespace StrikeEngine::Simulation
