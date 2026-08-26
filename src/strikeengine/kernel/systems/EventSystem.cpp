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
        EnvironmentConfig environment;
        // The legacy overload is flat-ground only, so X/Y are irrelevant but
        // still need matching lengths for the terrain-aware crossing path.
        const std::vector<double> previousPx(previousPz.size(), 0.0);
        const std::vector<double> previousPy(previousPz.size(), 0.0);
        evaluate(physics, status, currentTime, dt,
                 previousPx, previousPy, previousPz, environment);
    }

    void EventSystem::evaluate(
        PhysicsBlock& physics,
        EntityStatusBlock& status,
        double currentTime,
        double dt,
        const std::vector<double>& previousPx,
        const std::vector<double>& previousPy,
        const std::vector<double>& previousPz,
        const EnvironmentConfig& environment)
    {
        for (std::size_t i = 0; i < physics.size; ++i) {
            if (!physics.active[i] || !status.isAlive[i]) continue;

            const auto terrain = [&environment](double x, double y) {
                return environment.terrainElevation
                    ? environment.terrainElevation(x, y) : 0.0;
            };
            const double currentGround = terrain(physics.px[i], physics.py[i]);

            // Continuous ground check against the configured terrain surface.
            if (physics.pz[i] <= currentGround) {
                // Ground impact removes the entity from subsequent physics
                // and sensor updates. Clamp the crossing state so callers do
                // not observe a dead entity continuing below the terrain as
                // an active ghost.
                const bool hasCrossingData = dt > 0.0 &&
                    i < previousPx.size() && i < previousPy.size() &&
                    i < previousPz.size();
                double impactTime = currentTime;
                if (hasCrossingData) {
                    const double previousGround = terrain(previousPx[i], previousPy[i]);
                    const double previousHeight = previousPz[i] - previousGround;
                    const double currentHeight = physics.pz[i] - currentGround;
                    if (previousHeight > 0.0 && currentHeight < 0.0) {
                        const double fraction = previousHeight /
                            (previousHeight - currentHeight);
                        impactTime = currentTime - dt +
                            dt * std::clamp(fraction, 0.0, 1.0);
                    }
                }

                physics.pz[i] = currentGround;
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
