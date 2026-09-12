#include <strikeengine/simulation/Optimizer.hpp>
#include <random>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace StrikeEngine::Simulation {

    Optimizer::Optimizer(double timeStep_s, double maxTime_s)
        : dt(timeStep_s), maxTime(maxTime_s)
    {
        if (dt <= 0.0) {
            throw std::invalid_argument("Optimizer timeStep_s must be positive");
        }
        if (maxTime < 0.0) {
            throw std::invalid_argument("Optimizer maxTime_s cannot be negative");
        }
    }

    void Optimizer::setSeed(std::uint32_t seed) {
        this->seed = seed;
        seedSet = true;
    }

    void Optimizer::addParameter(double minBound, double maxBound) {
        bounds.push_back({minBound, maxBound});
    }

    void Optimizer::clearParameters() {
        bounds.clear();
    }

    OptimizationResult Optimizer::optimize(
        std::size_t swarmSize,
        int maxIterations,
        std::function<void(std::size_t, const std::vector<double>&, Kernel::VehicleInitState&)> applyParamsFunc,
        std::function<double(std::size_t, const Kernel::SimulationKernel&)> fitnessFunc)
    {
        std::size_t numParams = bounds.size();
        if (numParams == 0 || swarmSize == 0 || maxIterations <= 0) {
            return {{}, 0.0, 0};
        }
        if (!applyParamsFunc || !fitnessFunc) {
            throw std::invalid_argument(
                "Optimizer::optimize requires non-empty applyParamsFunc and fitnessFunc");
        }

        // PSO drive stream: pinned when setSeed was used, wall clock
        // otherwise (legacy behavior).
        unsigned useed = seedSet
            ? seed
            : static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
        std::mt19937 gen(useed);

        // PSO State
        std::vector<std::vector<double>> positions(swarmSize, std::vector<double>(numParams));
        std::vector<std::vector<double>> velocities(swarmSize, std::vector<double>(numParams));
        
        std::vector<std::vector<double>> pBestPos(swarmSize, std::vector<double>(numParams));
        std::vector<double> pBestFitness(swarmSize, -1e18); // Maximize fitness

        std::vector<double> gBestPos(numParams);
        double gBestFitness = -1e18;

        // Initialize particles
        for (std::size_t i = 0; i < swarmSize; ++i) {
            for (std::size_t j = 0; j < numParams; ++j) {
                std::uniform_real_distribution<> disPos(bounds[j].minVal, bounds[j].maxVal);
                positions[i][j] = disPos(gen);

                double range = bounds[j].maxVal - bounds[j].minVal;
                std::uniform_real_distribution<> disVel(-range*0.1, range*0.1);
                velocities[i][j] = disVel(gen);

                pBestPos[i][j] = positions[i][j];
            }
        }

        // PSO Hyperparameters
        const double w = 0.5;   // Inertia weight
        const double c1 = 1.5;  // Cognitive (personal best) weight
        const double c2 = 1.5;  // Social (global best) weight

        std::uniform_real_distribution<> randDist(0.0, 1.0);

        Kernel::SimulationKernel kernel;

        for (int iter = 0; iter < maxIterations; ++iter) {
            kernel.reset();

            // 1. Spawns swarm concurrently
            std::vector<Kernel::PhysicsId> vehicleIds(swarmSize);
            for (std::size_t i = 0; i < swarmSize; ++i) {
                Kernel::VehicleInitState init;
                applyParamsFunc(i, positions[i], init);
                vehicleIds[i] = kernel.createVehicle(init);
            }

            // 2. Simulate until timeout or all dead. Clamp the step count so
            // an enormous maxTime cannot overflow the int cast (UB).
            const double rawSteps = maxTime / dt;
            const int steps = rawSteps >= static_cast<double>(std::numeric_limits<int>::max())
                ? std::numeric_limits<int>::max()
                : static_cast<int>(rawSteps);
            const auto& physics = kernel.getPhysics();
            for (int step = 0; step < steps; ++step) {
                kernel.step(dt);
                // Optimization: if all entities are inactive, break early.
                // Ground impact sets active=false without freeing the slot, so
                // an all-inactive test must read the physics active flags.
                bool anyActive = false;
                for (std::size_t id = 0; id < physics.size; ++id) {
                    if (physics.active[id]) { anyActive = true; break; }
                }
                if (!anyActive) break;
            }

            // 3. Evaluate fitness and update bests
            for (std::size_t i = 0; i < swarmSize; ++i) {
                double fitness = fitnessFunc(vehicleIds[i], kernel);

                if (fitness > pBestFitness[i]) {
                    pBestFitness[i] = fitness;
                    pBestPos[i] = positions[i];

                    if (fitness > gBestFitness) {
                        gBestFitness = fitness;
                        gBestPos = positions[i];
                    }
                }
            }

            std::cout << "Iteration " << iter + 1 << "/" << maxIterations 
                      << " | Best Fitness: " << gBestFitness << "\n";

            // 4. Update Velocities and Positions for next iteration
            for (std::size_t i = 0; i < swarmSize; ++i) {
                for (std::size_t j = 0; j < numParams; ++j) {
                    double r1 = randDist(gen);
                    double r2 = randDist(gen);

                    velocities[i][j] = w * velocities[i][j] 
                                     + c1 * r1 * (pBestPos[i][j] - positions[i][j])
                                     + c2 * r2 * (gBestPos[j] - positions[i][j]);

                    positions[i][j] += velocities[i][j];

                    // Clamp to bounds
                    if (positions[i][j] < bounds[j].minVal) {
                        positions[i][j] = bounds[j].minVal;
                        velocities[i][j] *= -0.5; // bounce
                    } else if (positions[i][j] > bounds[j].maxVal) {
                        positions[i][j] = bounds[j].maxVal;
                        velocities[i][j] *= -0.5; // bounce
                    }
                }
            }
        }

        return {gBestPos, gBestFitness, maxIterations};
    }

} // namespace StrikeEngine::Simulation
