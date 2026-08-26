#pragma once

#include <vector>
#include <functional>
#include <strikeengine/kernel/data/PhysicsBlock.hpp>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>

namespace StrikeEngine::Kernel {

    enum class EventType {
        MotorBurnout,
        TargetImpact,
        GroundImpact,
        Detonation,
        Custom
    };

    struct SimulationEvent {
        EventType type;
        std::size_t entityId;
        double timestamp;
        // Optional custom data
        int customCode = 0;
    };

    using EventCallback = std::function<void(const SimulationEvent&)>;

    class EventSystem {
    public:
        // Evaluates continuous conditions (like ground impact) and fires events
        void evaluate(
            PhysicsBlock& physics,
            EntityStatusBlock& status,
            double currentTime
        );

        // Evaluates events using the previous step's ground heights so a
        // crossing event can be timestamped within the step.
        void evaluate(
            PhysicsBlock& physics,
            EntityStatusBlock& status,
            double currentTime,
            double dt,
            const std::vector<double>& previousPz
        );

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

        // Dispatches an event directly
        void dispatch(const SimulationEvent& evt);

        // Subscribes a listener to all events
        void subscribe(EventCallback callback);

        // Process queued events
        void processQueue();

    private:
        std::vector<SimulationEvent> eventQueue;
        std::vector<EventCallback> listeners;
    };

} // namespace StrikeEngine::Kernel
