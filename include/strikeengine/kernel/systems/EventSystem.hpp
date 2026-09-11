#pragma once

#include <array>
#include <vector>
#include <functional>
#include <set>
#include <utility>
#include <cstddef>
#include <strikeengine/kernel/data/PhysicsBlock.hpp>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>

namespace StrikeEngine::Kernel {

    enum class EventType {
        MotorBurnout,
        StageSeparation,
        TargetImpact,
        GroundImpact,
        Detonation,
        Custom,
        MotorFailure,
        EngineFailure,
        TankFailure,
        ActuatorFailure,
        SensorFailure,
        StructuralFailure,
        CommunicationFailure
    };

    struct SimulationEvent {
        EventType type;
        std::size_t entityId;
        double timestamp;
        // Optional custom data
        int customCode = 0;
        // Stage-separation payload: unburned propellant jettisoned with the
        // spent stage (kg). 0.0 when the stage burned out by exhaustion.
        double dumpedMassKg = 0.0;
        // Ground-impact terrain surface sampled at the crossing location.
        double terrainElevationM = 0.0;
        std::array<double, 3> terrainNormalEnu{0.0, 0.0, 1.0};
        double terrainSlopeRad = 0.0;
        // Detonation payload: engaged target and lethality outcome. For a
        // Proximity detonation targetEntityId is the threat that tripped the
        // fuse; otherwise it is the closest hostile in the blast.
        std::size_t targetEntityId = 0;
        double missDistanceM = 0.0;    // instantaneous or projected CPA miss
        double predictedCpaM = 0.0;    // projected CPA distance (CPA fuzing)
        double killProbability = 0.0;  // p at the reported miss distance
        bool   kill = false;           // at least one hostile killed
        bool   selfDestruct = false;   // timer expiry, no lethality applied
    };

    using EventCallback = std::function<void(const SimulationEvent&)>;

    /**
     * @brief Truth-event detection (ground impact, kinetic contact).
     *
     * Evaluates the end-of-step truth state and queues GroundImpact /
     * TargetImpact events. Ground impact snaps the entity to the terrain,
     * zeroes its motion and kills it; kinetic contact is report-only
     * (lethality stays warhead-governed). All overloads delegate to the
     * full form below.
     */
    class EventSystem {
    public:
        /**
         * @brief Flat-ground crossing test from previous heights only.
         * @param previousPz  Per-entity Z at the previous step; X/Y are
         *                    treated as irrelevant (flat ground).
         */
        void evaluate(
            PhysicsBlock& physics,
            EntityStatusBlock& status,
            double currentTime,
            double dt,
            const std::vector<double>& previousPz
        );

        /**
         * @brief Full terrain-aware evaluation.
         * @param previousPx/Py/Pz  Per-entity position at the previous step,
         *                          used to timestamp the crossing within dt.
         * @param environment  Terrain source, kinetic band + latch, swept
         *                     crossing and rate-zeroing switches.
         */
        void evaluate(
            PhysicsBlock& physics,
            EntityStatusBlock& status,
            double currentTime,
            double dt,
            const std::vector<double>& previousPx,
            const std::vector<double>& previousPy,
            const std::vector<double>& previousPz,
            const EnvironmentConfig& environment
        );

        /// Queues one event for end-of-step fan-out.
        void dispatch(const SimulationEvent& evt);

        /// Registers a listener invoked for every queued event.
        void subscribe(EventCallback callback);

        /// Fans queued events out to listeners, then clears the queue.
        void processQueue();

        /// Clears the kinetic-contact latch and any queued events so a
        /// reset/seed replay starts from a clean slate.
        void resetTransientState();

    private:
        std::vector<SimulationEvent> eventQueue;
        std::vector<EventCallback> listeners;
        // Kinetic-impact latch: pair -> in contact during the previous step.
        // Only used with EnvironmentConfig::kineticImpactLatchEnabled.
        std::set<std::pair<std::size_t, std::size_t>> kineticContactLatch;
    };

} // namespace StrikeEngine::Kernel
