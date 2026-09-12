#include <strikeengine/kernel/systems/EventSystem.hpp>
#include <strikeengine/models/physics/earth/EarthFrames.hpp>
#include <algorithm>
#include <array>
#include <cmath>

namespace StrikeEngine::Kernel {

    void EventSystem::evaluate(
        PhysicsBlock& physics,
        EntityStatusBlock& status,
        double currentTime,
        double dt,
        const std::vector<double>& previousPz)
    {
        // Flat-ground only: X/Y are irrelevant but need matching lengths
        // for the terrain-aware crossing path.
        const std::vector<double> previousPx(previousPz.size(), 0.0);
        const std::vector<double> previousPy(previousPz.size(), 0.0);
        evaluate(physics, status, currentTime, dt,
                 previousPx, previousPy, previousPz, EnvironmentConfig{});
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
            bool sweptHit = false;
            double impactTime = currentTime;
            double impactGround = currentGround;
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
                } else if (environment.sweptGroundImpactEnabled) {
                    // Ridge sweep: both endpoints can be above the surface
                    // while the segment midpoint is inside a hill.
                    const double midX = 0.5 * (previousPx[i] + physics.px[i]);
                    const double midY = 0.5 * (previousPy[i] + physics.py[i]);
                    const double midZ = 0.5 * (previousPz[i] + physics.pz[i]);
                    const auto midLocal = location(midX, midY, midZ);
                    const double midGround = terrain(midLocal);
                    if (midLocal.local[2] - midGround <= 0.0) {
                        crossedBelow = true;
                        sweptHit = true;
                        impactTime = currentTime - 0.5 * dt;
                        impactGround = midGround;
                    }
                }
            }
            const bool isSubsurface = (currentHeight < -0.10);
            const bool isGroundImpact = crossedBelow || isSubsurface;
            if (isGroundImpact) {
                if (ecefTruth) {
                    auto impactGeodetic = Models::ecefToGeodetic({
                        physics.px[i], physics.py[i], physics.pz[i]});
                    impactGeodetic.altitudeM = impactGround;
                    const auto impactEcef = Models::geodeticToEcef(impactGeodetic);
                    physics.px[i] = impactEcef.x;
                    physics.py[i] = impactEcef.y;
                    physics.pz[i] = impactEcef.z;
                } else {
                    physics.pz[i] = impactGround;
                }
                physics.vx[i] = 0.0;
                physics.vy[i] = 0.0;
                physics.vz[i] = 0.0;
                physics.ax[i] = 0.0;
                physics.ay[i] = 0.0;
                physics.az[i] = 0.0;
                if (environment.groundImpactZeroRates) {
                    physics.wx[i] = 0.0;
                    physics.wy[i] = 0.0;
                    physics.wz[i] = 0.0;
                }
                physics.active[i] = false;
                status.isAlive[i] = false;
                
                SimulationEvent evt;
                evt.type = EventType::GroundImpact;
                evt.entityId = i;
                evt.timestamp = impactTime;
                evt.customCode = sweptHit ? 2 : 1;  // 1 = crossing/subsurface, 2 = ridge sweep
                evt.terrainElevationM = impactGround;
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

        // Kinetic impact (body contact) report: opposing-allegiance entities
        // within the configured contact band dispatch TargetImpact. This is
        // report-only — lethality stays warhead-governed (processWarheads),
        // so a proximity warhead still gets its detonation event and kill
        // roll on a close pass instead of being pre-empted here.
        const double contactR = environment.kineticImpactRadiusM;
        if (contactR > 0.0) {
            const double contactR2 = contactR * contactR;
            std::set<std::pair<std::size_t, std::size_t>> inContact;
            for (std::size_t i = 0; i < physics.size; ++i) {
                if (!physics.active[i] || !status.isAlive[i]) continue;
                for (std::size_t j = i + 1; j < physics.size; ++j) {
                    if (!physics.active[j] || !status.isAlive[j]) continue;
                    if (i >= status.allegiance.size() || j >= status.allegiance.size()) continue;
                    if (status.allegiance[i] == status.allegiance[j]) continue;
                    const double dx = physics.px[j] - physics.px[i];
                    const double dy = physics.py[j] - physics.py[i];
                    const double dz = physics.pz[j] - physics.pz[i];
                    const double dist2 = dx * dx + dy * dy + dz * dz;
                    if (dist2 > contactR2) continue;
                    const std::pair<std::size_t, std::size_t> key{i, j};
                    inContact.insert(key);
                    if (environment.kineticImpactLatchEnabled &&
                        kineticContactLatch.count(key) != 0) {
                        continue;  // already reported this contact episode
                    }
                    // The interceptor is the non-hostile side of the pair.
                    const std::size_t interceptor =
                        (status.allegiance[j] == Allegiance::Hostile) ? i : j;
                    SimulationEvent evt;
                    evt.type = EventType::TargetImpact;
                    evt.entityId = interceptor;
                    evt.timestamp = currentTime;
                    evt.missDistanceM = std::sqrt(dist2);
                    dispatch(evt);
                }
            }
            if (environment.kineticImpactLatchEnabled) {
                kineticContactLatch = std::move(inContact);
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
        // Drain into a local first: a listener that calls back into the
        // kernel (failEntity/applyDamage -> dispatch) appends to eventQueue,
        // and a push during the range-for would reallocate it mid-iteration.
        std::vector<SimulationEvent> pending;
        pending.swap(eventQueue);
        for (const auto& evt : pending) {
            for (const auto& listener : listeners) {
                listener(evt);
            }
        }
        // Events dispatched by listeners during fan-out stay queued for the
        // next processQueue call.
    }

    void EventSystem::resetTransientState() {
        kineticContactLatch.clear();
        eventQueue.clear();
    }

} // namespace StrikeEngine::Kernel
