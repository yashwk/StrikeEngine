#include "strikeengine/core/Engine.hpp"

#include "strikeengine/systems/physics/GravitySystem.hpp"
#include "strikeengine/systems/physics/PropulsionSystem.hpp"
#include "strikeengine/systems/physics/AerodynamicsSystem.hpp"
#include "strikeengine/systems/physics/IntegrationSystem.hpp"
#include "strikeengine/systems/guidance/NavigationSystem.hpp"
#include "strikeengine/systems/guidance/SensorSystem.hpp"
#include "strikeengine/systems/guidance/GuidanceSystem.hpp"
#include "strikeengine/systems/guidance/ControlSystem.hpp"

#include <iostream>

namespace StrikeEngine {

    Engine::Engine() : _entity_factory(_registry) {
        initializeSystems();

        // After defining systems and dependencies, get the execution order once.
        _execution_order = _system_graph.getExecutionOrder();
    }

    void Engine::initializeSystems() {
        // --- 1. Create instances of all systems ---
        auto gravity_system = std::make_unique<GravitySystem>();
        auto propulsion_system = std::make_unique<PropulsionSystem>();
        auto nav_system = std::make_unique<NavigationSystem>();
        auto sensor_system = std::make_unique<SensorSystem>();
        auto guidance_system = std::make_unique<GuidanceSystem>();
        auto control_system = std::make_unique<ControlSystem>();
        auto aero_system = std::make_unique<AerodynamicsSystem>();
        auto integration_system = std::make_unique<IntegrationSystem>();

        // --- 2. Add systems to the graph (and get raw pointers for dependencies) ---
        System* p_gravity = gravity_system.get();
        _system_graph.addSystem(std::move(gravity_system));

        System* p_propulsion = propulsion_system.get();
        _system_graph.addSystem(std::move(propulsion_system));

        System* p_nav = nav_system.get();
        _system_graph.addSystem(std::move(nav_system));

        System* p_sensor = sensor_system.get();
        _system_graph.addSystem(std::move(sensor_system));

        System* p_guidance = guidance_system.get();
        _system_graph.addSystem(std::move(guidance_system));

        System* p_control = control_system.get();
        _system_graph.addSystem(std::move(control_system));

        System* p_aero = aero_system.get();
        _system_graph.addSystem(std::move(aero_system));

        System* p_integration = integration_system.get();
        _system_graph.addSystem(std::move(integration_system));

        // --- 3. Define the execution dependencies ---
        // The GNC loop runs first to determine control inputs.
        _system_graph.addDependency(p_guidance, p_nav);
        _system_graph.addDependency(p_guidance, p_sensor);
        _system_graph.addDependency(p_control, p_guidance);

        // Aerodynamics depends on the control system's output (fin deflections).
        _system_graph.addDependency(p_aero, p_control);

        // The integrator runs last, after all forces for the frame have been calculated.
        _system_graph.addDependency(p_integration, p_gravity);
        _system_graph.addDependency(p_integration, p_propulsion);
        _system_graph.addDependency(p_integration, p_aero);
    }

    void Engine::run(double simulation_time_s, double dt) {
        std::cout << "Engine: Starting simulation run." << std::endl;
        double current_time = 0.0;

        while (current_time < simulation_time_s) {
            // --- The New Parallel Simulation Loop ---

            // Iterate through the pre-calculated execution stages.
            for (const auto& stage : _execution_order) {
                // Submit all systems in the current stage to the job system.
                // These jobs can all run in parallel.
                for (System* system : stage) {
                    _job_system.submit([system, this, dt]() {
                        system->update(_registry, dt);
                    });
                }
                // Wait for all jobs in the current stage to complete before
                // moving to the next stage. This respects the dependencies.
                _job_system.wait();
            }

            current_time += dt;
        }
        std::cout << "Engine: Simulation run complete." << std::endl;
    }

} // namespace StrikeEngine
