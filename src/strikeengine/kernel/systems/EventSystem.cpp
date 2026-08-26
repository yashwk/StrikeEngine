#include <strikeengine/kernel/systems/EventSystem.hpp>
#include <algorithm>

namespace StrikeEngine::Kernel {

    void EventSystem::evaluate(
        PhysicsBlock& physics,
        EntityStatusBlock& status,
        double currentTime)
    {
        evaluate(physics, status, currentTime, 0.0, {});
    }

    void EventSystem::evaluate(
        PhysicsBlock& physics,
        EntityStatusBlock& status,
        double currentTime,
        double dt,
        const std::vector<double>& previousPz)
    {
        for (std::size_t i = 0; i < physics.size; ++i) {
            if (!physics.active[i] || !status.isAlive[i]) continue;

            // Example continuous check: Ground Impact (Flat Earth approximation Z <= 0)
            if (physics.pz[i] <= 0.0) {
                // Ground impact removes the entity from subsequent physics
                // and sensor updates. Clamp the crossing state so callers do
                // not observe a dead entity continuing below the terrain as
                // an active ghost.
                const bool hasCrossingData = dt > 0.0 && i < previousPz.size();
                double impactTime = currentTime;
                if (hasCrossingData && previousPz[i] > 0.0 && physics.pz[i] < 0.0) {
                    const double fraction = previousPz[i] /
                        (previousPz[i] - physics.pz[i]);
                    impactTime = currentTime - dt +
                        dt * std::clamp(fraction, 0.0, 1.0);
                }

                physics.pz[i] = 0.0;
                physics.vx[i] = 0.0;
                physics.vy[i] = 0.0;
                physics.vz[i] = 0.0;
                physics.ax[i] = 0.0;
                physics.ay[i] = 0.0;
                physics.az[i] = 0.0;
                physics.active[i] = false;
                status.isAlive[i] = false;
                
                SimulationEvent evt;
                evt.type = EventType::GroundImpact;
                evt.entityId = i;
                evt.timestamp = impactTime;
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
