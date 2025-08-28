#pragma once

#include "strikeengine/ecs/System.hpp"
#include <vector>
#include <unordered_map>
#include <memory>

namespace StrikeEngine {

    class SystemGraph {
    public:
        /**
         * @brief Adds a system to the graph.
         * @param system A unique pointer to the system to be added.
         */
        System* addSystem(std::unique_ptr<System> system);

        /**
         * @brief Defines a dependency between two systems.
         * @param dependent The system that must run AFTER the prerequisite.
         * @param prerequisite The system that must run BEFORE the dependent.
         */
        void addDependency(System* dependent, System* prerequisite);

        /**
         * @brief Calculates and returns a valid parallel execution order.
         * @return A vector of vectors, where each inner vector is a "stage" of
         * systems that can all be run in parallel.
         */
        std::vector<std::vector<System*>> getExecutionOrder();

    private:
        // A map to store the graph structure, mapping a system to the list of systems that depend on it.
        std::unordered_map<System*, std::vector<System*>> _adjacency_list;

        // A map to store the number of prerequisites for each system.
        std::unordered_map<System*, int> _in_degree;

        // A vector to own the system pointers.
        std::vector<std::unique_ptr<System>> _systems;
    };

} // namespace StrikeEngine
