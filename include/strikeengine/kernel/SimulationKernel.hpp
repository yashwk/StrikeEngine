#pragma once

#include <cstddef>
#include <vector>
#include <memory>

#include <strikeengine/kernel/data/PhysicsBlock.hpp>
#include <strikeengine/kernel/data/ControlBlock.hpp>
#include <strikeengine/kernel/data/GuidanceBlock.hpp>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>
#include <strikeengine/kernel/time/KernelTime.hpp>
#include <strikeengine/kernel/backend/PhysicsBackend.hpp>
#include <strikeengine/kernel/systems/GuidanceSystem.hpp>
#include <strikeengine/kernel/systems/AutopilotSystem.hpp>
#include <strikeengine/kernel/systems/EventSystem.hpp>
#include <strikeengine/kernel/systems/CommandProcessor.hpp>
#include <strikeengine/kernel/systems/SeekerSystem.hpp>
#include <strikeengine/kernel/data/SensorBlock.hpp>
#include <strikeengine/kernel/data/SeekerBlock.hpp>
#include <strikeengine/kernel/data/NavigationBlock.hpp>
#include <strikeengine/kernel/systems/SensorSystem.hpp>
#include <strikeengine/kernel/systems/NavigationSystem.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>

namespace StrikeEngine::Kernel {

    using PhysicsId = std::size_t;

    struct VehicleInitState {
        double px, py, pz;
        double vx, vy, vz;
        double qx, qy, qz, qw;
        double wx, wy, wz;
        double mass;
        double Ixx = 1.0, Iyy = 10.0, Izz = 10.0;

        EntityType type = EntityType::Missile;
        Allegiance allegiance = Allegiance::Friendly;

        SeekerType seekerType = SeekerType::None;
        std::string rcsProfileId = "";
        std::string irProfileId = "";
    };

    enum class BackendType {
        CPU,
        Vulkan
    };

    class SimulationKernel {
    public:
        explicit SimulationKernel(BackendType backendType = BackendType::CPU);
        ~SimulationKernel();

        void initialize();
        void reset();

        // Entity management
        PhysicsId createVehicle(const VehicleInitState& init);
        PhysicsId createVehicle(const VehicleInitState& init, const VehicleConfig& config);
        void removeVehicle(PhysicsId id);

        // Simulation control
        void queueCommand(const SimulationCommand& cmd);
        void step(double dt);
        void runSteps(std::size_t steps, double dt);

        /**
         * @brief Make all stochastic models (sensor noise, biases) deterministic.
         * Same seed + same scenario + same step sequence => bit-identical runs.
         * Call before stepping. Default (unseeded) keeps a wall-clock seed.
         */
        void setRandomSeed(std::uint32_t seed);

        // Accessors
        const PhysicsBlock& getPhysics() const { return physicsBlock; }
        const ControlBlock& getControl() const { return controlBlock; }
        const GuidanceBlock& getGuidance() const { return guidanceBlock; }
        const NavigationBlock& getNavigation() const { return navigationBlock; }
        const SensorBlock& getSensors() const { return sensorBlock; }
        SeekerBlock& getSeekers() { return seekerBlock; }
        const SeekerBlock& getSeekers() const { return seekerBlock; }
        double getSimulationTime() const { return time.currentTime(); }
        std::size_t getEntityCount() const { return physicsBlock.size - freeList.size(); }

    private:
        PhysicsBlock physicsBlock;
        ControlBlock controlBlock;
        GuidanceBlock guidanceBlock;
        EntityStatusBlock statusBlock;
        SensorBlock sensorBlock;
        NavigationBlock navigationBlock;
        SeekerBlock seekerBlock;

        // Systems
        SensorSystem sensorSystem;
        NavigationSystem navigationSystem;
        SeekerSystem seekerSystem;
        GuidanceSystem guidanceSystem;
        AutopilotSystem autopilotSystem;
        EventSystem eventSystem;
        CommandProcessor commandProcessor;

        KernelTime time;

        std::vector<PhysicsId> freeList;
        std::unique_ptr<PhysicsBackend> backend;
    };

} // namespace StrikeEngine::Kernel
