#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <unordered_map>
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
#include <strikeengine/kernel/config/VehicleInitState.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <strikeengine/kernel/config/ScenarioConfig.hpp>
#include <strikeengine/kernel/integrator/IntegratorFactory.hpp>

namespace StrikeEngine::Kernel {

    using PhysicsId = std::size_t;

    // Per-entity multi-stage propulsion plan (see SimulationKernel::processStaging).
    struct StagePlan {
        std::vector<int>    poolIds;       // backend propulsion pool id per stage
        std::vector<double> burnDurations; // thrust-curve end time per stage (s)
        std::vector<double> dropMasses;    // structure dropped after each stage (kg)
        std::vector<double> propellantCaps; // raw StageConfig::propellantMassKg per stage
        std::vector<double> reservedAfter;  // sum of later stages' positive propellant caps (kg)
        bool burnoutReported = false;       // final-stage MotorBurnout dispatched
        std::vector<double> maxGimbalPitchRad;
        std::vector<double> maxGimbalYawRad;
        std::vector<double> gimbalTimeConstantSec;
        std::vector<double> maxGimbalRateRadPerSec;
        std::vector<double> enginePositionX;
        std::vector<double> enginePositionY;
        std::vector<double> enginePositionZ;
    };

    // A scenario entity deferred by its LaunchSpec: held by the kernel until
    // the launch conditions (parent seeker lock hold + range gate) are met,
    // then spawned with rail-release geometry and its initial guidance
    // command. See ScenarioEntityConfig::LaunchSpec.
    struct PendingLaunch {
        ScenarioEntityConfig cfg;
        double lockSince = -1.0;
    };

    // A spawned rail round still in its straight-ahead separation flyout;
    // at switchTime the kernel queues the entity's initial guidance command
    // (seeded from the parent's relayed datalink track).
    struct ActiveFlyout {
        PhysicsId entityId = 0;
        double switchTime = 0.0;
        ScenarioEntityConfig cfg;
    };

    // A cold-launched round still on its clearance axis: at atTime (the first
    // stage's ignition delay) the body and the ejection velocity rotate by
    // deltaDeg about the body Y axis, onto the loft axis.
    struct PendingPitchOver {
        PhysicsId entityId = 0;
        double atTime = 0.0;
        double deltaDeg = 0.0;
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
        // Terminal fuse/damage options (see WarheadConfig).
        bool fuseEnabled = true;
        bool cpaFuzingEnabled = false;
        double fuseLookaheadSec = 0.02;
        double armingDelaySec = 0.0;
        double minClosingSpeedMps = 0.0;
        double selfDestructTimeSec = 0.0;
        double damage = 100.0;
        double fuseDetectionProbability = 1.0;
        double headOnLethalityFactor = 1.0;
        double tailOnLethalityFactor = 1.0;
        // Last detonation diagnostics (for the designer/telemetry).
        std::size_t lastTargetId = 0;
        double lastMissDistanceM = 0.0;
        double lastPredictedCpaM = 0.0;
        double lastKillProbability = 0.0;
        bool   lastKill = false;
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

        // Hold a launch-enabled scenario entity for in-flight spawning
        // (called by ScenarioConfig::loadInto; see LaunchSpec).
        void addPendingLaunch(const ScenarioEntityConfig& entityCfg);
        // True while any rail-launched entity is still gating on its
        // launch conditions (run-end logic treats the opening as pre-launch).
        [[nodiscard]] bool hasPendingLaunches() const { return !pendingLaunches.empty(); }

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

        // Backend threading
        void setThreadCount(std::size_t threads);
        [[nodiscard]] std::size_t threadCount() const;

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
        // Warhead fusing/lethality state and last-detonation diagnostics.
        const WarheadState& getWarhead(PhysicsId id) const;
        double getSimulationTime() const { return time.currentTime(); }
        std::size_t getEntityCount() const { return physicsBlock.size - freeList.size(); }
        const EnvironmentConfig& getEnvironment() const { return environment; }
        EnvironmentConfig& getEnvironment() { return environment; }

    private:
        PhysicsBlock physicsBlock;
        ControlBlock controlBlock;
        GuidanceBlock guidanceBlock;
        EntityStatusBlock statusBlock;
        SensorBlock sensorBlock;
        NavigationBlock navigationBlock;
        SeekerBlock seekerBlock;
        TrackBlock trackBlock;   // persistent target tracks

        // Systems
        SensorSystem sensorSystem;
        NavigationSystem navigationSystem;
        SeekerSystem seekerSystem;
        TrackManagerSystem trackManagerSystem;  // after seekers, before guidance
        GuidanceSystem guidanceSystem;
        AutopilotSystem autopilotSystem;
        EventSystem eventSystem;
        CommandProcessor commandProcessor;

        KernelTime time;

        std::vector<PhysicsId> freeList;
        std::unique_ptr<PhysicsBackend> backend;
        EnvironmentConfig environment;

        /**
         * @brief Parsed profile cache, keyed by profile file path.
         *
         * Profile files are immutable for the lifetime of the kernel, so
         * re-parsing them on every createVehicle is redundant. Load failures
         * are not cached, so correcting a file takes effect on the next spawn.
         */
        struct ProfileCache {
            std::unordered_map<std::string, AeroConfig> aero;
            std::unordered_map<std::string, PropulsionConfig> motor;
            std::unordered_map<std::string, SeekerConfig> seeker;
            std::unordered_map<std::string, SensorConfig> sensor;
            std::unordered_map<std::string, GuidanceAutopilotConfig> guidance;
            std::unordered_map<std::string, WarheadConfig> warhead;
        };
        ProfileCache profileCache;

        // Per-entity staging + warhead state (parallel to physics entities).
        std::vector<StagePlan> stagePlans;
        std::vector<WarheadState> warheads;

        // RNG streams. Sensor noise and warhead kill draws live on SEPARATE
        // streams: sharing one entangles future sensor noise with engagement
        // outcomes (kill draws shift the sensor stream position), breaking
        // noise reproducibility across lethality variants. Both derive
        // deterministically from the seed set via setRandomSeed.
        std::uint32_t randomSeed = 0u;
        std::mt19937 warheadRng;
        // Fuze-detection stream (proximity detection probability < 1). Kept
        // separate so a probabilistic fuze never shifts the lethality draws.
        std::mt19937 fuzeRng;

        void processStaging();
        void processWarheads();

        /**
         * @brief Marks an entity dead: deactivates it and zeroes its motion.
         */
        void killEntity(PhysicsId id);

        // Fixed installations (EntityType::RadarSite) do not fly: they are
        // pinned to the position where they first went active, with zero
        // velocity and rates, so a ground radar behaves like the installation
        // it is while its sensors, tracks and datalink keep running.
        void pinFixedInstallations();
        /// Grows the fixed-installation anchor buffers to @p n entries.
        void ensureFixedAnchors(std::size_t n);
        std::vector<double> fixedAnchorPx, fixedAnchorPy, fixedAnchorPz;

        // Rail-launch deferred spawns (see ScenarioEntityConfig::LaunchSpec).
        void processPendingLaunches();
        void processActiveFlyouts();
        void processPitchOvers();

        /// Reused pre-step position snapshot for eventSystem.evaluate.
        std::vector<double> stepPrevPx_, stepPrevPy_, stepPrevPz_;
        // Body->world quaternion with body X on the launch axis (elevationDeg
        // above the horizon toward the target), Y horizontal right, Z = X x Y.
        // False when the geometry has no usable axis.
        [[nodiscard]] bool launchAxisQuaternion(
            std::size_t parent, const ScenarioEntityConfig::LaunchSpec& spec,
            double ux, double uy, double uz, double elevationDeg,
            double& qw, double& qx, double& qy, double& qz) const;
        PhysicsId spawnPendingLaunch(PendingLaunch& pl);
        // Queue the entity's initial guidance command, seeded from the
        // parent's relayed datalink track when one exists.
        void queueInitialGuidance(PhysicsId id, const ScenarioEntityConfig& cfg);
        std::vector<PendingLaunch> pendingLaunches;
        std::vector<ActiveFlyout> activeFlyouts;
        std::vector<PendingPitchOver> pendingPitchOvers;
    };

} // namespace StrikeEngine::Kernel
