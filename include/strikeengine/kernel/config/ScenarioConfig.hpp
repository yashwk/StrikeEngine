#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <string>
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/data/GuidanceBlock.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>

namespace StrikeEngine::Kernel {

    // Represents an initial state and configuration for an entity in a scenario
    struct ScenarioEntityConfig {
        VehicleInitState initState;
        VehicleConfig vehicleConfig;

        // Path to a design file (JSON). During scenario deserialization a
        // non-empty designRef is resolved via loadDesignPhysics and OVERRIDES
        // any inline vehicleConfig. loadInto does not resolve it.
        std::string designRef = "";

        GuidanceMode initialGuidanceMode = GuidanceMode::None;
        
        double initialTargetX = 0.0;
        double initialTargetY = 0.0;
        double initialTargetZ = 0.0;

        // Target velocity. ProNav needs a correct closing velocity against a
        // moving target (LOS-rate guidance derives omega from relative
        // velocity); defaults keep legacy scenarios stationary-aimpoint.
        double initialTargetVx = 0.0;
        double initialTargetVy = 0.0;
        double initialTargetVz = 0.0;
        double initialMaxAccel = 0.0;

        // Target acceleration for APN feed-forward (augmented PN). Used only
        // when initialTargetAccelAvailable is true; see SimulationCommand.
        double initialTargetAccelX = 0.0;
        double initialTargetAccelY = 0.0;
        double initialTargetAccelZ = 0.0;
        bool   initialTargetAccelAvailable = false;

        // Optional target identity for the persistent track (W39); -1 unknown
        // (the seeker supplies identity once it locks).
        std::int64_t initialTargetId = -1;
    };

    struct ScenarioConfig {
        std::string name;
        std::string description;

        EnvironmentConfig environment;
        
        std::vector<ScenarioEntityConfig> entities;

        // Entity selected by study wrappers for per-scenario summaries and
        // sweep/Monte Carlo rows. Zero preserves legacy behavior.
        std::size_t primaryEntityIndex = 0;

        // THE global seed for this scenario. loadInto copies it into the
        // kernel, which fans it out to every stochastic system inside
        // (sensor stream, nav alignment stream, split warhead stream). One
        // seed per scenario, no per-system seeding from outside.
        std::uint32_t randomSeed = 0xDEADBEEFu;

        // Serialize/deserialize this scenario to/from a JSON file.
        // save() returns false if the file cannot be opened; load() throws
        // std::runtime_error if the file is missing or the JSON is malformed.
        bool save(const std::string& path) const;
        static ScenarioConfig load(const std::string& path);

        // Apply this scenario to the given kernel
        void loadInto(SimulationKernel& kernel) const {
            kernel.reset();
            kernel.setEnvironment(environment);

            for (const auto& entityCfg : entities) {
                PhysicsId id = kernel.createVehicle(
                    entityCfg.initState, entityCfg.vehicleConfig);
                
                if (entityCfg.initialGuidanceMode != GuidanceMode::None) {
                    SimulationCommand cmd;
                    cmd.entityId = id;
                    cmd.mode = entityCfg.initialGuidanceMode;
                    cmd.targetX = entityCfg.initialTargetX;
                    cmd.targetY = entityCfg.initialTargetY;
                    cmd.targetZ = entityCfg.initialTargetZ;
                    cmd.targetVx = entityCfg.initialTargetVx;
                    cmd.targetVy = entityCfg.initialTargetVy;
                    cmd.targetVz = entityCfg.initialTargetVz;
                    cmd.maxAccel = entityCfg.initialMaxAccel;
                    cmd.targetAccelX = entityCfg.initialTargetAccelX;
                    cmd.targetAccelY = entityCfg.initialTargetAccelY;
                    cmd.targetAccelZ = entityCfg.initialTargetAccelZ;
                    cmd.targetAccelAvailable = entityCfg.initialTargetAccelAvailable;
                    cmd.targetId = entityCfg.initialTargetId;
                    kernel.queueCommand(cmd);
                }
            }

            // Single fan-out point: the scenario seed becomes the kernel
            // seed, which setRandomSeed copies into every stochastic system
            // inside (sensor, nav alignment, split warhead streams).
            kernel.setRandomSeed(randomSeed);
        }
    };

} // namespace StrikeEngine::Kernel
