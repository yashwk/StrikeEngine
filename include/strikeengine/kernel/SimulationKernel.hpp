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
#include <strikeengine/kernel/data/TrackBlock.hpp>
#include <strikeengine/kernel/systems/SensorSystem.hpp>
#include <strikeengine/kernel/systems/NavigationSystem.hpp>
#include <strikeengine/kernel/systems/TrackManagerSystem.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <strikeengine/kernel/integrator/IntegratorFactory.hpp>

namespace StrikeEngine::Kernel {

    using PhysicsId = std::size_t;

    struct VehicleInitState {
        double px, py, pz;
        double vx, vy, vz;
        double qx, qy, qz, qw;
        double wx, wy, wz;
        double mass;
        Allegiance allegiance = Allegiance::Friendly;
    };

    // Per-entity multi-stage propulsion plan (see SimulationKernel::processStaging).
    struct StagePlan {
        std::vector<int>    poolIds;       // backend propulsion pool id per stage
        std::vector<double> burnDurations; // thrust-curve end time per stage (s)
        std::vector<double> dropMasses;    // structure dropped after each stage (kg)
        std::vector<double> propellantCaps; // raw StageConfig::propellantMassKg per stage
        std::vector<double> reservedAfter;  // sum of later stages' positive propellant caps (kg)
        std::vector<double> maxGimbalPitchRad;
        std::vector<double> maxGimbalYawRad;
        std::vector<double> gimbalTimeConstantSec;
        std::vector<double> maxGimbalRateRadPerSec;
        std::vector<double> enginePositionX;
        std::vector<double> enginePositionY;
        std::vector<double> enginePositionZ;
    };

    // Per-entity warhead state (see SimulationKernel::processWarheads).
    struct WarheadState {
        double lethalRadiusM = 0.0;
        double falloffRadiusM = 0.0;
        FusingType fusing = FusingType::Impact;
        double proximityTriggerM = 0.0;
        double timedDelaySec = 0.0;
        double launchTime = 0.0;
        bool detonated = false;
    };

    enum class BackendType {
        CPU,
        Vulkan
    };

    class SimulationKernel {
    public:
        /**
         * @brief Constructs the simulation kernel.
         * @param backendType Selects the physics backend (CPU or Vulkan).
         * @param integratorType Selects the CPU truth integrator (Euler, RK4,
         *        Symplectic/Velocity-Verlet, or adaptive RK45). It is unused
         *        for the Vulkan backend.
         */
        explicit SimulationKernel(BackendType backendType = BackendType::CPU,
                                  IntegratorType integratorType = IntegratorType::RK4);
        ~SimulationKernel();

        void initialize();
        void reset();

        // Entity management
        PhysicsId createVehicle(const VehicleInitState& init);
        PhysicsId createVehicle(const VehicleInitState& init, const VehicleConfig& config);
        void removeVehicle(PhysicsId id);

        // Deterministic failure and damage injection
        void failEntity(PhysicsId id, FailureMode mode);
        void applyDamage(PhysicsId id, double damage);

        // Simulation control
        void queueCommand(const SimulationCommand& cmd);
        void setThrustVectorCommand(PhysicsId id, double pitchRad, double yawRad);
        void step(double dt);
        void runSteps(std::size_t steps, double dt);

        /**
         * @brief Make all stochastic models (sensor noise, biases) deterministic.
         * Same seed + same scenario + same step sequence => bit-identical runs.
         * Call before stepping. Default (unseeded) keeps a wall-clock seed.
         */
        void setRandomSeed(std::uint32_t seed);

        // Configure terrain and world-frame wind before stepping.
        void setEnvironment(const EnvironmentConfig& environment);

        // Accessors
        const PhysicsBlock& getPhysics() const { return physicsBlock; }
        const ControlBlock& getControl() const { return controlBlock; }
        const GuidanceBlock& getGuidance() const { return guidanceBlock; }
        const NavigationBlock& getNavigation() const { return navigationBlock; }
        const SensorBlock& getSensors() const { return sensorBlock; }
        const SeekerBlock& getSeekers() const { return seekerBlock; }
        const TrackBlock& getTracks() const { return trackBlock; }
        const EntityStatusBlock& getStatus() const { return statusBlock; }
        EventSystem& getEventSystem() { return eventSystem; }
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
        TrackBlock trackBlock;   // W39 persistent target tracks

        // Systems
        SensorSystem sensorSystem;
        NavigationSystem navigationSystem;
        SeekerSystem seekerSystem;
        TrackManagerSystem trackManagerSystem;  // W39 (after seekers, before guidance)
        GuidanceSystem guidanceSystem;
        AutopilotSystem autopilotSystem;
        EventSystem eventSystem;
        CommandProcessor commandProcessor;

        KernelTime time;

        std::vector<PhysicsId> freeList;
        std::unique_ptr<PhysicsBackend> backend;
        EnvironmentConfig environment;

        // Per-entity staging + warhead state (parallel to physics entities).
        std::vector<StagePlan> stagePlans;
        std::vector<WarheadState> warheads;

        void processStaging();
        void processWarheads();
    };

} // namespace StrikeEngine::Kernel
