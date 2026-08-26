#include <strikeengine/kernel/systems/EventSystem.hpp>
#include <strikeengine/models/physics/earth/EarthFrames.hpp>
#include <algorithm>
#include <array>

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
        const bool ecefTruth = environment.earth.useEcefTruth;
        const Models::GeodeticCoordinate reference{
            environment.earth.referenceLatitudeRad,
            environment.earth.referenceLongitudeRad,
            0.0};
        const auto localPosition = [&](double x, double y, double z) {
            if (!ecefTruth) return std::array<double, 3>{x, y, z};
            const Models::EcefCoordinate ecef{x, y, z};
            const auto geodetic = Models::ecefToGeodetic(ecef);
            const auto enu = Models::EarthFrames::ecefToEnu(ecef, reference);
            return std::array<double, 3>{enu[0], enu[1], geodetic.altitudeM};
        };

        for (std::size_t i = 0; i < physics.size; ++i) {
            if (!physics.active[i] || !status.isAlive[i]) continue;

            const auto terrain = [&environment](double x, double y) {
                return environment.terrainElevation
                    ? environment.terrainElevation(x, y) : 0.0;
            };
            const auto currentLocal = localPosition(
                physics.px[i], physics.py[i], physics.pz[i]);
            const double currentGround = terrain(currentLocal[0], currentLocal[1]);
            const double currentHeight = currentLocal[2] - currentGround;

            // Continuous ground check against the configured terrain surface.
            if (currentHeight <= 0.0) {
                // Ground impact removes the entity from subsequent physics
                // and sensor updates. Clamp the crossing state so callers do
                // not observe a dead entity continuing below the terrain as
                // an active ghost.
                const bool hasCrossingData = dt > 0.0 &&
                    i < previousPx.size() && i < previousPy.size() &&
                    i < previousPz.size();
                double impactTime = currentTime;
                if (hasCrossingData) {
                    const auto previousLocal = localPosition(
                        previousPx[i], previousPy[i], previousPz[i]);
                    const double previousGround = terrain(
                        previousLocal[0], previousLocal[1]);
                    const double previousHeight = previousLocal[2] - previousGround;
                    if (previousHeight > 0.0 && currentHeight < 0.0) {
                        const double fraction = previousHeight /
                            (previousHeight - currentHeight);
                        impactTime = currentTime - dt +
                            dt * std::clamp(fraction, 0.0, 1.0);
                    }
                }

                if (ecefTruth) {
                    auto impactGeodetic = Models::ecefToGeodetic({
                        physics.px[i], physics.py[i], physics.pz[i]});
                    impactGeodetic.altitudeM = currentGround;
                    const auto impactEcef = Models::geodeticToEcef(impactGeodetic);
                    physics.px[i] = impactEcef.x;
                    physics.py[i] = impactEcef.y;
                    physics.pz[i] = impactEcef.z;
                } else {
                    physics.pz[i] = currentGround;
                }
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
