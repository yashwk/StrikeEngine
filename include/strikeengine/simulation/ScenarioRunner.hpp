#pragma once

#include "strikeengine/core/Engine.hpp"
#include <string>
#include <memory>

namespace StrikeEngine {

    // The ScenarioRunner is now a high-level controller that owns the Engine
    // and is responsible for loading data into it.
    class ScenarioRunner {
    public:
        ScenarioRunner();
        ~ScenarioRunner() = default;

        // Loads a scenario from a file, creating entities in the Engine's registry.
        bool loadScenario(const std::string& scenarioPath);

        // Runs the entire simulation using the Engine.
        void run();

    private:
        std::unique_ptr<Engine> _engine;
        double _simulationDuration = 0.0;
        double _timeStep = 0.01667; // Default to 60hz
    };
} // namespace StrikeEngine
