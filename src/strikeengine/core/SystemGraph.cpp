#include "strikeengine/core/SystemGraph.hpp"
#include <queue>
#include <stdexcept>

namespace StrikeEngine {

    void SystemGraph::addSystem(std::unique_ptr<System> system) {
        System* system_ptr = system.get();
        _systems.push_back(std::move(system));
        _adjacency_list[system_ptr] = {};
        _in_degree[system_ptr] = 0;
    }

    void SystemGraph::addDependency(System* dependent, System* prerequisite) {
        if (!_adjacency_list.contains(prerequisite) || !_adjacency_list.contains(dependent)) {
            throw std::runtime_error("SystemGraph: Attempted to add dependency with an unregistered system.");
        }
        _adjacency_list[prerequisite].push_back(dependent);
        _in_degree[dependent]++;
    }

    std::vector<std::vector<System*>> SystemGraph::getExecutionOrder() {
        std::vector<std::vector<System*>> execution_stages;
        std::queue<System*> q;

        // --- Kahn's Algorithm for Topological Sort ---

        // 1. Initialize the queue with all nodes that have an in-degree of 0 (no prerequisites).
        for (const auto& pair : _in_degree) {
            if (pair.second == 0) {
                q.push(pair.first);
            }
        }

        while (!q.empty()) {
            // 2. Create a new stage for the current set of parallelizable systems.
            std::vector<System*> current_stage;
            int stage_size = q.size();
            for (int i = 0; i < stage_size; ++i) {
                System* u = q.front();
                q.pop();
                current_stage.push_back(u);

                // 3. For each neighbor of the current system, decrement its in-degree.
                if (_adjacency_list.contains(u)) {
                    for (System* v : _adjacency_list[u]) {
                        _in_degree[v]--;
                        // 4. If a neighbor's in-degree becomes 0, it can be added to the queue for the next stage.
                        if (_in_degree[v] == 0) {
                            q.push(v);
                        }
                    }
                }
            }
            execution_stages.push_back(current_stage);
        }

        // 5. Check for cycles in the graph. If not all systems are in a stage, there was a cycle.
        size_t count = 0;
        for(const auto& stage : execution_stages) {
            count += stage.size();
        }
        if (count != _systems.size()) {
            throw std::runtime_error("SystemGraph: Cycle detected in system dependencies.");
        }

        return execution_stages;
    }

} // namespace StrikeEngine
