#include <strikeengine/kernel/systems/EventSystem.hpp>

namespace StrikeEngine::Kernel {

    void EventSystem::evaluate(
        const PhysicsBlock& physics,
        EntityStatusBlock& status,
        double currentTime)
    {
        for (std::size_t i = 0; i < physics.size; ++i) {
            if (!physics.active[i] || !status.isAlive[i]) continue;

            // Example continuous check: Ground Impact (Flat Earth approximation Z <= 0)
            if (physics.pz[i] <= 0.0) {
                // If it hits the ground, it is destroyed
                status.isAlive[i] = false;
                
                SimulationEvent evt;
                evt.type = EventType::GroundImpact;
                evt.entityId = i;
                evt.timestamp = currentTime;
                dispatch(evt);
            }
        }
    }

    void EventSystem::dispatch(const SimulationEvent& evt) {
        eventQueue.push_back(evt);
    }

    void EventSystem::subscribe(EventCallback callback) {
        listeners.push_back(std::move(callback));
    }

    void EventSystem::processQueue() {
        for (const auto& evt : eventQueue) {
            for (const auto& listener : listeners) {
                listener(evt);
            }
        }
        eventQueue.clear();
    }

} // namespace StrikeEngine::Kernel
