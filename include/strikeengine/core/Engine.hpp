#pragma once

#include "strikeengine/ecs/Registry.hpp"
#include "strikeengine/core/JobSystem.hpp"
#include "strikeengine/core/SystemGraph.hpp"
#include "strikeengine/simulation/EntityFactory.hpp"

#include <memory>
#include <vector>

namespace StrikeEngine {

    class Engine {
    public:
        Engine();

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

        // The new multi-threading and dependency management systems.
        JobSystem _job_system;
        SystemGraph _system_graph;

        // Stores the calculated parallel execution stages.
        std::vector<std::vector<System*>> _execution_order;
    };

} // namespace StrikeEngine
