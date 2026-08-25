#pragma once

#include <vector>
#include <functional>
#include "../data/PhysicsBlock.hpp"
#include "../data/EntityStatusBlock.hpp"

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
            const PhysicsBlock& physics,
            EntityStatusBlock& status,
            double currentTime
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