#pragma once

#include "strikeengine/ecs/Registry.hpp"
#include "strikeengine/core/JobSystem.hpp"
#include "strikeengine/core/SystemGraph.hpp"
#include "strikeengine/simulation/EntityFactory.hpp"

#include <memory>
#include <vector>

#include "strikeengine/atmosphere/AtmosphereManager.hpp"

namespace StrikeEngine {

    class Engine {
    public:
        Engine();
        /**
         * @brief Runs the simulation for a single time step.
         * @param dt The delta time for the frame.
         */
        void update(double dt);

        /**
         * @brief Provides access to the simulation's entity-component registry.
         */
        Registry& getRegistry() { return _registry; }

        /**
         * @brief Provides access to the factory for creating entities from profiles.
         */
        EntityFactory& getEntityFactory() { return _entity_factory; }


        // --- EXISTING METHOD ---

        /**
         * @brief Runs the main simulation loop for a specified duration.
         * @param simulation_time_s The total duration to simulate.
         * @param dt The fixed time step for each frame.
         */
        void run(double simulation_time_s, double dt);

    private:
        /**
         * @brief Initializes all ECS systems and defines their dependencies.
         */
        void initializeSystems();

        Registry _registry;
        EntityFactory _entity_factory;
        AtmosphereManager _atmosphere_manager;

        JobSystem _job_system;
        SystemGraph _system_graph;

        std::vector<std::vector<System*>> _execution_order;
    };

} // namespace StrikeEngine
