#pragma once

#include <cstddef>
#include <vector>
#include <memory>

#include "data/PhysicsBlock.hpp"
#include "data/ControlBlock.hpp"
#include "data/GuidanceBlock.hpp"
#include "data/EntityStatusBlock.hpp"
#include "time/KernelTime.hpp"
#include "backend/PhysicsBackend.hpp"
#include "systems/GuidanceSystem.hpp"
#include "systems/AutopilotSystem.hpp"
#include "systems/EventSystem.hpp"
#include "systems/CommandProcessor.hpp"
#include "systems/SeekerSystem.hpp"
#include "data/SensorBlock.hpp"
#include "data/SeekerBlock.hpp"
#include "data/NavigationBlock.hpp"
#include "systems/SensorSystem.hpp"
#include "systems/NavigationSystem.hpp"

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
        void removeVehicle(PhysicsId id);

        // Simulation control
        void queueCommand(const SimulationCommand& cmd);
        void step(double dt);
        void runSteps(std::size_t steps, double dt);

        // Accessors
        const PhysicsBlock& getPhysics() const { return physicsBlock; }
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
