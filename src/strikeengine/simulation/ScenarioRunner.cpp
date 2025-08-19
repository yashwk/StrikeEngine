#include "strikeengine/simulation/ScenarioRunner.hpp"
#include "strikeengine/simulation/EntityFactory.hpp"
#include "nlohmann/json.hpp"

#include "strikeengine/systems/guidance/SensorSystem.hpp"
#include "strikeengine/systems/guidance/GuidanceSystem.hpp"
#include "strikeengine/systems/guidance/ControlSystem.hpp"
#include "strikeengine/systems/physics/GravitySystem.hpp"
#include "strikeengine/systems/physics/PropulsionSystem.hpp"
#include "strikeengine/systems/physics/AerodynamicsSystem.hpp"
#include "strikeengine/systems/physics/IntegrationSystem.hpp"

#include "strikeengine/components/guidance/GuidanceComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include <fstream>
#include <iostream>
#include <map>

namespace StrikeEngine {

    using json = nlohmann::json;

    ScenarioRunner::ScenarioRunner() {
        // Pre-initialize managers. This ensures they are ready before any system needs them.
        if (!_atmosphereManager.loadTable("data/atmosphere_table.bin")) {
            throw std::runtime_error("ScenarioRunner: CRITICAL - Failed to load atmosphere table on initialization.");

        }
    }

    ScenarioRunner::~ScenarioRunner() = default;

    void ScenarioRunner::initializeSystems() {
        _systems.clear();
        // 1. GNC systems run first to generate commands for the current frame.
        _systems.push_back(std::make_unique<SensorSystem>());
        _systems.push_back(std::make_unique<GuidanceSystem>());
        _systems.push_back(std::make_unique<ControlSystem>());

        // 2. Physics systems run next to calculate all forces based on the current state and commands.
        _systems.push_back(std::make_unique<GravitySystem>());
        _systems.push_back(std::make_unique<PropulsionSystem>());
        _systems.push_back(std::make_unique<AerodynamicsSystem>(_atmosphereManager));

        // 3. The integration system runs LAST to apply the accumulated forces and update the state.
        _systems.push_back(std::make_unique<IntegrationSystem>());
    }

    bool ScenarioRunner::loadScenario(const std::string& scenarioPath) {
        std::cout << "Loading scenario: " << scenarioPath << std::endl;

        initializeSystems();
        std::cout << "All systems initialized." << std::endl;

        std::ifstream f(scenarioPath);
        if (!f.is_open()) {
            std::cerr << "Error: Could not open scenario file: " << scenarioPath << std::endl;
            return false;
        }
        json data = json::parse(f);

        _simulationDuration = data.at("simulation").at("duration_s").get<double>();
        _timeStep = 1.0 / data.at("simulation").at("time_step_hz").get<double>();

        EntityFactory factory(_registry);
        std::map<std::string, Entity> createdEntities;
        for (const auto& entityDef : data.at("entities")) {
            std::string name = entityDef.at("name");
            std::string profile = entityDef.at("profile");
            createdEntities[name] = factory.createFromProfile(profile);
        }

        const auto& engagement = data.at("engagement");
        Entity shooter = createdEntities.at(engagement.at("shooter"));
        Entity target = createdEntities.at(engagement.at("target"));

        if (_registry.has<GuidanceComponent>(shooter)) {
            auto& guidance = _registry.get<GuidanceComponent>(shooter);
            guidance.targetEntity = target;
            std::cout << "Engagement set: '" << engagement.at("shooter") << "' (ID " << shooter.id
                      << ") is targeting '" << engagement.at("target") << "' (ID " << target.id << ")" << std::endl;
        }

        std::cout << "Scenario loaded successfully." << std::endl;
        return true;
    }

    void ScenarioRunner::run() {
        std::cout << "\n--- Starting Simulation ---" << std::endl;
        _simulationTime = 0.0;

        // Store entity IDs for logging
        Entity missile_id = NULL_ENTITY;
        Entity target_id = NULL_ENTITY;
        auto view = _registry.view<GuidanceComponent>();
        if (!view.empty()) missile_id = std::get<0>(view[0]);
        if (missile_id != NULL_ENTITY) target_id = _registry.get<GuidanceComponent>(missile_id).targetEntity;


        while (_simulationTime < _simulationDuration) {
            update(_timeStep);
            _simulationTime += _timeStep;

            // Simple console output for telemetry (e.g., print every second of sim time)
            if (static_cast<int>(_simulationTime / _timeStep) % static_cast<int>(1.0 / _timeStep) == 0) {
                 std::cout << "Sim Time: " << _simulationTime << "s" << std::endl;
                 if(missile_id != NULL_ENTITY && target_id != NULL_ENTITY) {
                     const auto& missile_pos = _registry.get<TransformComponent>(missile_id).position;
                     const auto& target_pos = _registry.get<TransformComponent>(target_id).position;
                     double range = glm::length(target_pos - missile_pos);
                     std::cout << "  > Range to target: " << range << "m" << std::endl;
                 }
            }
        }
        std::cout << "--- Simulation Finished ---" << std::endl;
    }

    void ScenarioRunner::update(double dt) {
        for (const auto& system : _systems) {
            system->update(_registry, dt);
        }
    }

} // namespace StrikeEngine
