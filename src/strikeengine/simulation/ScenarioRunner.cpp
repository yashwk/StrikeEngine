#include "strikeengine/simulation/ScenarioRunner.hpp"
#include "strikeengine/simulation/EntityFactory.hpp"
#include "strikeengine/components/guidance/GuidanceComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "nlohmann/json.hpp"

#include <fstream>
#include <iostream>
#include <map>

namespace StrikeEngine {

    using json = nlohmann::json;

    ScenarioRunner::ScenarioRunner() {
        // The runner's main job is to create the engine.
        // The Engine's constructor now handles all initialization.
        _engine = std::make_unique<Engine>();
    }

    bool ScenarioRunner::loadScenario(const std::string& scenarioPath) {
        std::cout << "Loading scenario: " << scenarioPath << std::endl;

        std::ifstream f(scenarioPath);
        if (!f.is_open()) {
            std::cerr << "Error: Could not open scenario file: " << scenarioPath << std::endl;
            return false;
        }
        json data;
        try {
            data = json::parse(f);
        } catch (const json::parse_error& e) {
            std::cerr << "Error: Failed to parse scenario JSON: " << e.what() << std::endl;
            return false;
        }


        _simulationDuration = data.at("simulation").at("duration_s").get<double>();
        _timeStep = 1.0 / data.at("simulation").at("time_step_hz").get<double>();

        // Use the EntityFactory that belongs to the Engine.
        EntityFactory& factory = _engine->getEntityFactory();
        std::map<std::string, Entity> createdEntities;
        for (const auto& entityDef : data.at("entities")) {
            std::string name = entityDef.at("name");
            std::string profile = entityDef.at("profile");
            createdEntities[name] = factory.createFromProfile(profile);
        }

        // Set up the engagement using the Engine's registry.
        const auto& engagement = data.at("engagement");
        Entity shooter = createdEntities.at(engagement.at("shooter"));
        Entity target = createdEntities.at(engagement.at("target"));

        Registry& registry = _engine->getRegistry();
        if (registry.has<GuidanceComponent>(shooter)) {
            auto& guidance = registry.get<GuidanceComponent>(shooter);
            guidance.targetEntity = target;
            std::cout << "Engagement set: '" << engagement.at("shooter") << "' (ID " << shooter.index()
                      << ") is targeting '" << engagement.at("target") << "' (ID " << target.index() << ")" << std::endl;
        }

        std::cout << "Scenario loaded successfully." << std::endl;
        return true;
    }

    void ScenarioRunner::run() {
        std::cout << "\n--- Starting Simulation ---" << std::endl;
        double simulationTime = 0.0;

        Registry& registry = _engine->getRegistry();

        // Correctly find the missile and target IDs before the loop starts.
        Entity missile_id = NULL_ENTITY;
        Entity target_id = NULL_ENTITY;
        for (auto entity : registry.view<GuidanceComponent>()) {
            missile_id = entity;
            target_id = registry.get<GuidanceComponent>(entity).targetEntity;
            break; // Assuming one missile for now
        }

        while (simulationTime < _simulationDuration) {
            // The runner tells the engine to update by one time step.
            _engine->update(_timeStep);
            simulationTime += _timeStep;

            // Simple console output for telemetry
            if (static_cast<int>(simulationTime / _timeStep) % static_cast<int>(1.0 / _timeStep) == 0) {
                 std::cout << "Sim Time: " << simulationTime << "s" << std::endl;
                 if(registry.isAlive(missile_id) && registry.isAlive(target_id)) {
                     const auto& missile_pos = registry.get<TransformComponent>(missile_id).position;
                     const auto& target_pos = registry.get<TransformComponent>(target_id).position;
                     double range = glm::length(target_pos - missile_pos);
                     std::cout << "  > Range to target: " << range << "m" << std::endl;
                 } else {
                     std::cout << "  > Engagement finished." << std::endl;
                     break; // End simulation if missile or target is destroyed
                 }
            }
        }
        std::cout << "--- Simulation Finished ---" << std::endl;
    }

} // namespace StrikeEngine
