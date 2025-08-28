#include "strikeengine/core/Engine.hpp"
#include "strikeengine/simulation/EntityFactory.hpp"
#include "strikeengine/ecs/System.hpp"

#include "strikeengine/systems/physics/GravitySystem.hpp"
#include "strikeengine/systems/physics/PropulsionSystem.hpp"
#include "strikeengine/systems/physics/AerodynamicsSystem.hpp"
#include "strikeengine/systems/physics/IntegrationSystem.hpp"
#include "strikeengine/systems/guidance/NavigationSystem.hpp"
#include "strikeengine/systems/guidance/SensorSystem.hpp"
#include "strikeengine/systems/guidance/GuidanceSystem.hpp"
#include "strikeengine/systems/guidance/ControlSystem.hpp"
#include "strikeengine/systems/guidance/EndgameSystem.hpp"

#include <iostream>

namespace StrikeEngine {
    Engine::Engine() : _entity_factory(_registry)
    {
        _atmosphere_manager.loadTable("data/atmosphere_table.bin");
        initializeSystems();
        _execution_order = _system_graph.getExecutionOrder();
    }

    void Engine::initializeSystems()
    {
        // --- 1. Create instances of all systems ---
        auto gravity_system = std::make_unique<GravitySystem>();
        auto propulsion_system = std::make_unique<PropulsionSystem>(_atmosphere_manager);
        auto nav_system = std::make_unique<NavigationSystem>();
        auto sensor_system = std::make_unique<SensorSystem>();
        auto guidance_system = std::make_unique<GuidanceSystem>();
        auto control_system = std::make_unique<ControlSystem>();
        auto aero_system = std::make_unique<AerodynamicsSystem>(_atmosphere_manager);
        auto integration_system = std::make_unique<IntegrationSystem>();
        auto endgame_system = std::make_unique<EndgameSystem>();


        // --- 2. Add systems to the graph (and get raw pointers for dependencies) ---
        System* p_gravity = _system_graph.addSystem(std::move(gravity_system));
        System* p_propulsion = _system_graph.addSystem(std::move(propulsion_system));
        System* p_nav = _system_graph.addSystem(std::move(nav_system));
        System* p_sensor = _system_graph.addSystem(std::move(sensor_system));
        System* p_guidance = _system_graph.addSystem(std::move(guidance_system));
        System* p_control = _system_graph.addSystem(std::move(control_system));
        System* p_aero = _system_graph.addSystem(std::move(aero_system));
        System* p_integration = _system_graph.addSystem(std::move(integration_system));
        System* p_endgame = _system_graph.addSystem(std::move(endgame_system));


        // --- 3. Define the execution dependencies ---
        _system_graph.addDependency(p_guidance, p_nav);
        _system_graph.addDependency(p_guidance, p_sensor);
        _system_graph.addDependency(p_control, p_guidance);
        _system_graph.addDependency(p_aero, p_control);
        _system_graph.addDependency(p_integration, p_gravity);
        _system_graph.addDependency(p_integration, p_propulsion);
        _system_graph.addDependency(p_integration, p_aero);
        _system_graph.addDependency(p_endgame, p_integration);
    }


    void Engine::update(double dt)
    {
        for (const auto& stage : _execution_order)
        {
            for (System* system : stage)
            {
                _job_system.submit([system, this, dt]()
                {
                    system->update(_registry, dt);
                });
            }
            _job_system.wait();
        }
    }

    void Engine::run(double simulation_time_s, double dt)
    {
        std::cout << "Engine: Starting simulation run." << std::endl;
        double current_time = 0.0;

        while (current_time < simulation_time_s)
        {
            update(dt);
            current_time += dt;
        }
        std::cout << "Engine: Simulation run complete." << std::endl;
    }
} // namespace StrikeEngine
