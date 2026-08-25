#pragma once

#include <vector>
#include <string>
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/data/GuidanceBlock.hpp>

namespace StrikeEngine::Kernel {

    // Represents an initial state and configuration for an entity in a scenario
    struct ScenarioEntityConfig {
        VehicleInitState initState;
        GuidanceMode initialGuidanceMode = GuidanceMode::None;
        
        double initialTargetX = 0.0;
        double initialTargetY = 0.0;
        double initialTargetZ = 0.0;
    };

    struct ScenarioConfig {
        std::string name;
        std::string description;
        
        std::vector<ScenarioEntityConfig> entities;

        // Apply this scenario to the given kernel
        void loadInto(SimulationKernel& kernel) const {
            kernel.reset();

            for (const auto& entityCfg : entities) {
                PhysicsId id = kernel.createVehicle(entityCfg.initState);
                
                if (entityCfg.initialGuidanceMode != GuidanceMode::None) {
                    SimulationCommand cmd;
                    cmd.entityId = id;
                    cmd.mode = entityCfg.initialGuidanceMode;
                    cmd.targetX = entityCfg.initialTargetX;
                    cmd.targetY = entityCfg.initialTargetY;
                    cmd.targetZ = entityCfg.initialTargetZ;
                    kernel.queueCommand(cmd);
                }
            }
        }
    };

} // namespace StrikeEngine::Kernel
