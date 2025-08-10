#pragma once

#include "strikeengine/ecs/Registry.hpp"
#include "strikeengine/atmosphere/AtmosphereManager.hpp"
#include <string>
#include <vector>
#include <memory>

namespace StrikeEngine {

    class System; // Forward-declaration

    /**
     * @brief Manages the setup, execution, and teardown of a simulation scenario.
     *
     * This class is the main driver for the simulation. It loads a scenario from
     * a file, initializes all necessary systems and managers, creates the entities,
     * and runs the main simulation loop at a fixed time step.
     */
    class ScenarioRunner {
    public:
        ScenarioRunner();
        ~ScenarioRunner();

        /**
         * @brief Loads and prepares a scenario from a JSON definition file.
         * @param scenarioPath Path to the scenario JSON file.
         * @return True if loading was successful, false otherwise.
         */
        bool loadScenario(const std::string& scenarioPath);

        /**
         * @brief Runs the main simulation loop until completion.
         */
        void run();

    private:
        void initializeSystems();
        void update(double dt);

        Registry _registry;
        AtmosphereManager _atmosphereManager;
        std::vector<std::unique_ptr<System>> _systems;

        double _simulationTime = 0.0;
        double _simulationDuration = 60.0;
        double _timeStep = 0.01; // 100 Hz
    };

} // namespace StrikeEngine
