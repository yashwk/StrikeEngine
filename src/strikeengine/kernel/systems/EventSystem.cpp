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
        struct TerrainLocation {
            std::array<double, 3> local{};
            Models::GeodeticCoordinate geodetic{};
        };
        const auto location = [&](double x, double y, double z) {
            if (!ecefTruth) {
                const auto geodetic = Models::EarthFrames::enuToGeodetic(
                    {x, y, 0.0}, reference);
                return TerrainLocation{{x, y, z}, geodetic};
            }
            const Models::EcefCoordinate ecef{x, y, z};
            const auto geodetic = Models::ecefToGeodetic(ecef);
            const auto enu = Models::EarthFrames::ecefToEnu(ecef, reference);
            return TerrainLocation{{enu[0], enu[1], geodetic.altitudeM}, geodetic};
        };

        for (std::size_t i = 0; i < physics.size; ++i) {
            if (!physics.active[i] || !status.isAlive[i]) continue;

            const auto terrain = [&environment](const TerrainLocation& point) {
                if (environment.globalTerrain) {
                    return environment.globalTerrain->sample(
                        point.geodetic.latitudeRad, point.geodetic.longitudeRad).elevationM;
                }
                return environment.terrainElevation
                    ? environment.terrainElevation(point.local[0], point.local[1]) : 0.0;
            };
            const auto currentLocal = location(
                physics.px[i], physics.py[i], physics.pz[i]);
            const double currentGround = terrain(currentLocal);
            const double currentHeight = currentLocal.local[2] - currentGround;

            // Ground-impact detection. A vehicle resting exactly on the surface
            // (a ground launch rail / platform at height 0) is NOT an impact;
            // only a genuine crossing (descending from above onto/below the
            // surface this step) or a position beneath the surface is.
            const bool hasCrossingData = dt > 0.0 &&
                i < previousPx.size() && i < previousPy.size() &&
                i < previousPz.size();
            bool crossedBelow = false;
            double impactTime = currentTime;
            if (hasCrossingData) {
                const auto previousLocal = location(
                    previousPx[i], previousPy[i], previousPz[i]);
                const double previousGround = terrain(previousLocal);
                const double previousHeight = previousLocal.local[2] - previousGround;
                // Descended from above and now at/below the surface.
                crossedBelow = (previousHeight > 0.0 && currentHeight <= 0.0);
                if (crossedBelow) {
                    const double fraction = previousHeight /
                        (previousHeight - currentHeight);
                    impactTime = currentTime - dt +
                        dt * std::clamp(fraction, 0.0, 1.0);
                }
            }
            const bool isGroundImpact = crossedBelow || (currentHeight < 0.0);
            if (isGroundImpact) {
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
                evt.terrainElevationM = currentGround;
                if (environment.globalTerrain) {
                    const auto surface = environment.globalTerrain->surface(
                        currentLocal.geodetic.latitudeRad,
                        currentLocal.geodetic.longitudeRad);
                    evt.terrainNormalEnu = surface.normalEnu;
                    evt.terrainSlopeRad = surface.slopeRad;
                }
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
