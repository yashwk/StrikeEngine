#include <strikeengine/kernel/systems/GuidanceSystem.hpp>
#include <strikeengine/models/guidance/GuidanceModels.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>
#include <strikeengine/models/physics/earth/EarthFrames.hpp>
#include <cmath>
#include <glm/glm.hpp>
#include <algorithm>
#include <glm/gtc/quaternion.hpp>

namespace StrikeEngine::Kernel {

    namespace {

        // Effective navigation gain for an entity: the tgo-scheduled value
        // when scheduling is enabled (previous step's tgo drives the switch
        // — deterministic, no peeking at the current step), else the base N.
        // update() maintains scheduledNavN every step; laws only read here.
        inline double effNavN(std::size_t id, const GuidanceBlock& g)
        {
            // Kernel-driven runs always size scheduledNavN (createVehicle
            // maintains it); hand-built blocks fall back to their own N.
            if (id < g.scheduledNavN.size()) return g.scheduledNavN[id];
            if (id < g.navigationConstant.size()) return g.navigationConstant[id];
            return 3.5;
        }

        // Result of one guidance-law evaluation; all outputs are finite.
        struct LawResult {            double ax = 0.0, ay = 0.0, az = 0.0;
            bool valid = true;      // law could be evaluated (geometry OK)
            bool lawInvalid = false; // non-finite input / bad configuration
            bool nonClosing = false; // range > 0 but closing speed <= 0
            double tgoSec = 0.0;    // estimated time to go (diagnostic)
        };

        bool isFinite3(double x, double y, double z)
        {
            return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        }

        // Clamp the commanded acceleration magnitude to the per-entity guidance
        // limit (GuidanceBlock::maxAccel; 0 = unlimited). Sets the limit flag.
        void clampCommandMagnitude(std::size_t id, GuidanceBlock& guidance)
        {
            guidance.limitedByMaxAccel[id] = false;
            const double lim = guidance.maxAccel[id];
            if (lim <= 0.0) return;
            const double ax = guidance.commandedAccelX[id];
            const double ay = guidance.commandedAccelY[id];
            const double az = guidance.commandedAccelZ[id];
            const double mag = std::sqrt(ax * ax + ay * ay + az * az);
            if (mag > lim && mag > 1e-9) {
                const double s = lim / mag;
                guidance.commandedAccelX[id] = ax * s;
                guidance.commandedAccelY[id] = ay * s;
                guidance.commandedAccelZ[id] = az * s;
                guidance.limitedByMaxAccel[id] = true;
            }
        }

        // Range-dependent PN gain shaping (opt-in): classical N'(r), clamped so
        // it only lightly biases the gain. Disabled = the base/scheduled N.
        inline double rangeShapedN(std::size_t id, const GuidanceBlock& g, double range)
        {
            double N = effNavN(id, g);
            if (id < g.rangeGainShapingEnabled.size() && g.rangeGainShapingEnabled[id] &&
                range > 1e-6) {
                const double ref = (id < g.rangeGainRefM.size()) ? g.rangeGainRefM[id] : 10000.0;
                if (ref > 0.0) N *= std::clamp(ref / range, 0.5, 2.0);
            }
            return N;
        }

        // First-order lag + slew limiting on the commanded demand. Both are
        // opt-in (0 = off); with both off the shaped command equals the
        // clamped demand, so legacy behavior is unchanged.
        void shapeCommand(std::size_t id, GuidanceBlock& g, double dt)
        {
            if (id >= g.shapedAccelX.size() || id >= g.commandedAccelX.size()) return;
            const double lag = (id < g.commandLagSec.size()) ? g.commandLagSec[id] : 0.0;
            const double slew = (id < g.commandSlewLimitMps3.size())
                ? g.commandSlewLimitMps3[id] : 0.0;
            if (lag <= 0.0 && slew <= 0.0) {
                g.shapedAccelX[id] = g.commandedAccelX[id];
                g.shapedAccelY[id] = g.commandedAccelY[id];
                g.shapedAccelZ[id] = g.commandedAccelZ[id];
                return;
            }
            double tx = g.commandedAccelX[id];
            double ty = g.commandedAccelY[id];
            double tz = g.commandedAccelZ[id];
            double sx = g.shapedAccelX[id];
            double sy = g.shapedAccelY[id];
            double sz = g.shapedAccelZ[id];
            if (lag > 0.0 && dt > 0.0) {
                const double a = 1.0 - std::exp(-dt / lag);
                sx += a * (tx - sx);
                sy += a * (ty - sy);
                sz += a * (tz - sz);
            } else if (slew <= 0.0) {
                sx = tx; sy = ty; sz = tz;
            }
            if (slew > 0.0 && dt > 0.0) {
                const double step = slew * dt;
                const auto clampStep = [step](double& s, double target) {
                    const double d = target - s;
                    s += std::clamp(d, -step, step);
                };
                clampStep(sx, tx);
                clampStep(sy, ty);
                clampStep(sz, tz);
            }
            g.shapedAccelX[id] = sx;
            g.shapedAccelY[id] = sy;
            g.shapedAccelZ[id] = sz;
            g.commandedAccelX[id] = sx;
            g.commandedAccelY[id] = sy;
            g.commandedAccelZ[id] = sz;
        }

        // Publish a law result as the entity's guidance demand (raw + clamped)
        // with diagnostics; zero demand marks invalid results.
        void applyDemand(std::size_t id, const LawResult& r, GuidanceBlock& g, double dt,
                         double authorityScale = 1.0)
        {
            g.rawAccelX[id] = r.ax;
            g.rawAccelY[id] = r.ay;
            g.rawAccelZ[id] = r.az;
            const double s = std::clamp(authorityScale, 0.0, 1.0);
            g.commandedAccelX[id] = r.valid ? r.ax * s : 0.0;
            g.commandedAccelY[id] = r.valid ? r.ay * s : 0.0;
            g.commandedAccelZ[id] = r.valid ? r.az * s : 0.0;
            g.lawInvalid[id] = r.lawInvalid;
            g.nonClosing[id] = r.nonClosing;
            g.tgoSec[id] = r.tgoSec;
            clampCommandMagnitude(id, g);
            const bool wasLimited = g.limitedByMaxAccel[id];
            shapeCommand(id, g, dt);
            clampCommandMagnitude(id, g); // shaped demand must respect the limit
            g.limitedByMaxAccel[id] = g.limitedByMaxAccel[id] || wasLimited;
        }

        void zeroDemand(std::size_t id, GuidanceBlock& g)
        {
            g.rawAccelX[id] = g.rawAccelY[id] = g.rawAccelZ[id] = 0.0;
            g.commandedAccelX[id] = g.commandedAccelY[id] = g.commandedAccelZ[id] = 0.0;
            if (id < g.shapedAccelX.size()) {
                g.shapedAccelX[id] = g.shapedAccelY[id] = g.shapedAccelZ[id] = 0.0;
            }
            g.limitedByMaxAccel[id] = false;
            g.lawInvalid[id] = false;
            g.nonClosing[id] = false;
            g.tgoSec[id] = 0.0;
        }

        // Reset the trajectory prediction diagnostics. Called at the top
        // of every per-entity guidance step so a mode/phase change (e.g. a
        // seeker lock that overrides trajectory midcourse) never leaks stale
        // PIP/feasibility state from a previous step.
        void clearTrajectoryDiagnostics(std::size_t id, GuidanceBlock& g)
        {
            g.predictedInterceptX[id] = 0.0;
            g.predictedInterceptY[id] = 0.0;
            g.predictedInterceptZ[id] = 0.0;
            g.predictedTgoSec[id] = 0.0;
            g.trajectoryRequiredAccel[id] = 0.0;
            g.trajectoryAimSource[id] = GuidanceAimSource::None;
            g.trajectoryFeasible[id] = false;
            g.trajectoryReason[id] = TrajectoryReason::None;
        }

        // Aircraft cruise (GuidanceMode::Cruise): hold a reference geodetic
        // altitude and fly a level course toward the waypoint target. The
        // missile ProNav/Waypoint laws steer toward a point but do not hold
        // altitude, so a heavy aircraft sinks; this law adds an altitude-hold
        // (P + D on altitude error) along the local vertical plus a horizontal
        // demand toward the waypoint.
        LawResult computeCruise(
            std::size_t id, const NavigationBlock& nav, const GuidanceBlock& g,
            const EnvironmentConfig& env)
        {
            LawResult out;
            const double px = nav.estPx[id], py = nav.estPy[id], pz = nav.estPz[id];
            const double vx = nav.estVx[id], vy = nav.estVy[id], vz = nav.estVz[id];

            // Local vertical (geodetic up) direction, present altitude, climb rate.
            double upx = 0.0, upy = 0.0, upz = 0.0, alt = 0.0;
            if (env.earth.useEcefTruth) {
                const auto geo = Models::ecefToGeodetic({px, py, pz});
                alt = geo.altitudeM;
                const double lat = geo.latitudeRad, lon = geo.longitudeRad;
                upx = std::cos(lat) * std::cos(lon);
                upy = std::cos(lat) * std::sin(lon);
                upz = std::sin(lat);
            } else {
                upx = 0.0; upy = 0.0; upz = 1.0;
                alt = pz;
            }
            const double climbRate = vx * upx + vy * upy + vz * upz;

            // Reference altitude: explicit cruise altitude, else the waypoint's
            // own geodetic altitude (never a raw Cartesian Z delta, which is
            // not an altitude change under ECEF truth).
            double altRef = g.cruiseAltitudeM[id];
            if (altRef <= 0.0) {
                if (env.earth.useEcefTruth) {
                    const auto geoTgt = Models::ecefToGeodetic(
                        {g.targetX[id], g.targetY[id], g.targetZ[id]});
                    altRef = std::isfinite(geoTgt.altitudeM) ? geoTgt.altitudeM : alt;
                } else {
                    altRef = alt + (g.targetZ[id] - pz);
                }
            }

            // Altitude-hold vertical demand (m/s^2 along the local up).
            const double aVert = g.cruiseAltitudeGain[id] * (altRef - alt)
                               - g.cruiseAltitudeDamping[id] * climbRate;

            // Horizontal direction toward the waypoint (remove vertical comp).
            const double dx = g.targetX[id] - px, dy = g.targetY[id] - py, dz = g.targetZ[id] - pz;
            const double dUp = dx * upx + dy * upy + dz * upz;
            const double hx = dx - dUp * upx, hy = dy - dUp * upy, hz = dz - dUp * upz;
            const double hMag = std::sqrt(hx * hx + hy * hy + hz * hz);
            double ax = 0.0, ay = 0.0, az = 0.0;
            if (hMag > 1e-6) {
                const double k = g.cruiseWaypointGain[id];
                ax = k * (hx / hMag);
                ay = k * (hy / hMag);
                az = k * (hz / hMag);
            }
            ax += aVert * upx;
            ay += aVert * upy;
            az += aVert * upz;

            out.ax = ax; out.ay = ay; out.az = az;
            out.valid = std::isfinite(ax) && std::isfinite(ay) && std::isfinite(az);
            out.tgoSec = hMag / 250.0; // diagnostic only
            return out;
        }

        // Track-quality gate helper: no threshold configured (vector short) or
        // a missing quality array means the gate is inactive.
        inline bool qualityAbove(const TrackBlock* t, std::size_t e,
                                 const std::vector<double>& threshold)
        {
            if (!t || e >= t->size) return false;
            if (e >= threshold.size() || e >= t->quality01.size()) return true;
            return t->quality01[e] >= threshold[e];
        }

        // Midcourse PN / APN on the commanded target track (world frame).
        // Aim source: a measurement-anchored persistent target track
        // wins over the raw external command state; see update().
        LawResult computeMidcourse(
            std::size_t id, const NavigationBlock& nav,
            const TrackBlock* tracks, GuidanceBlock& guidance,
            const EnvironmentConfig& env)
        {
            LawResult out;
            const double baseN = effNavN(id, guidance);
            if (!std::isfinite(baseN) || baseN <= 0.0) {
                out.lawInvalid = true;
                out.valid = false;
                return out;
            }

            // Select the aim state: datalink source track (cooperative
            // engagement) > own persistent track (measurement-anchored) >
            // the external command state (legacy). A configured track-quality
            // floor can disqualify a track (default 0 = no gate).
            double tx, ty, tz, tvx, tvy, tvz;
            bool trackAim = false;
            bool datalinkAim = false;
            std::size_t datalinkSrc = 0;
            const int dlSrc = (id < guidance.datalinkSourceId.size())
                ? guidance.datalinkSourceId[id] : -1;
            if (dlSrc >= 0 && tracks) {
                const std::size_t src = static_cast<std::size_t>(dlSrc);
                if (src < tracks->size && tracks->active(src) &&
                    tracks->updateCount[src] > 0 &&
                    qualityAbove(tracks, src, guidance.trackAimMinQuality01))
                {
                    datalinkAim = true;
                    datalinkSrc = src;
                    tx = tracks->posX[src]; ty = tracks->posY[src]; tz = tracks->posZ[src];
                    tvx = tracks->velX[src]; tvy = tracks->velY[src]; tvz = tracks->velZ[src];
                }
            }
            if (!datalinkAim && tracks && id < tracks->size && tracks->active(id) &&
                tracks->updateCount[id] > 0 &&
                qualityAbove(tracks, id, guidance.trackAimMinQuality01))
            {
                trackAim = true;
                tx = tracks->posX[id]; ty = tracks->posY[id]; tz = tracks->posZ[id];
                tvx = tracks->velX[id]; tvy = tracks->velY[id]; tvz = tracks->velZ[id];
            }
            if (!datalinkAim && !trackAim) {
                tx = guidance.targetX[id]; ty = guidance.targetY[id]; tz = guidance.targetZ[id];
                tvx = guidance.targetVx[id]; tvy = guidance.targetVy[id]; tvz = guidance.targetVz[id];
            }

            const double rx = tx - nav.estPx[id];
            const double ry = ty - nav.estPy[id];
            const double rz = tz - nav.estPz[id];
            const double vx = tvx - nav.estVx[id];
            const double vy = tvy - nav.estVy[id];
            const double vz = tvz - nav.estVz[id];
            if (!isFinite3(rx, ry, rz) || !isFinite3(vx, vy, vz)) {
                out.lawInvalid = true;
                out.valid = false;
                return out;
            }

            const Models::Vec3 r{rx, ry, rz};
            const Models::Vec3 v{vx, vy, vz};
            const double range = std::sqrt(rx * rx + ry * ry + rz * rz);
            const double closingSpeed = (range > 1e-9)
                ? -(rx * vx + ry * vy + rz * vz) / range : 0.0;
            if (id < guidance.closingSpeed.size()) guidance.closingSpeed[id] = closingSpeed;

            // Feed-forward target acceleration: from the AIMED track when
            // measurement-anchored, quality-trusted and available (own track,
            // else the datalink source track whose position/velocity steers
            // above), else from the command. Falling back to the command under
            // a datalink aim can augment with the wrong target's accel.
            bool ffAvailable = false;
            double atx = 0.0, aty = 0.0, atz = 0.0;
            if (trackAim && id < tracks->accelAvailable.size() &&
                tracks->accelAvailable[id] &&
                qualityAbove(tracks, id, guidance.apnFeedforwardMinQuality01)) {
                ffAvailable = isFinite3(tracks->accelX[id], tracks->accelY[id],
                                        tracks->accelZ[id]);
                atx = tracks->accelX[id]; aty = tracks->accelY[id]; atz = tracks->accelZ[id];
            } else if (datalinkAim && datalinkSrc < tracks->size &&
                       datalinkSrc < tracks->accelAvailable.size() &&
                       tracks->accelAvailable[datalinkSrc] &&
                       qualityAbove(tracks, datalinkSrc, guidance.apnFeedforwardMinQuality01)) {
                ffAvailable = isFinite3(tracks->accelX[datalinkSrc],
                                        tracks->accelY[datalinkSrc],
                                        tracks->accelZ[datalinkSrc]);
                atx = tracks->accelX[datalinkSrc];
                aty = tracks->accelY[datalinkSrc];
                atz = tracks->accelZ[datalinkSrc];
            } else if (!trackAim && !datalinkAim && guidance.targetAccelAvailable[id]) {
                ffAvailable = isFinite3(guidance.targetAccelX[id],
                                        guidance.targetAccelY[id],
                                        guidance.targetAccelZ[id]);
                atx = guidance.targetAccelX[id]; aty = guidance.targetAccelY[id];
                atz = guidance.targetAccelZ[id];
            }
            const double N = rangeShapedN(id, guidance, range);
            const bool feedforward = guidance.apnFeedforwardEnabled[id] && ffAvailable;
            const Models::GuidanceSolution sol = feedforward
                ? Models::augmentedProportionalNavigation(r, v, {atx, aty, atz}, N)
                : Models::proportionalNavigation(r, v, N);

            if (!sol.valid) {
                out.valid = false;
                out.nonClosing = range > 1e-9 && closingSpeed <= 0.0;
                return out;
            }
            out.ax = sol.acceleration[0];
            out.ay = sol.acceleration[1];
            out.az = sol.acceleration[2];

            // Midcourse loft (opt-in): add a vertical climb demand that fades
            // as the intercept nears, for long-range energy shaping.
            if (id < guidance.loftEnabled.size() && guidance.loftEnabled[id] &&
                range > 0.0) {
                const double loftRange = (id < guidance.loftRangeM.size())
                    ? std::max(1.0, guidance.loftRangeM[id]) : 40000.0;
                const double loftAlt = (id < guidance.loftAltitudeM.size())
                    ? guidance.loftAltitudeM[id] : 0.0;
                const double loftGain = (id < guidance.loftGain.size())
                    ? guidance.loftGain[id] : 0.0;
                const double scale = std::clamp(range / loftRange, 0.0, 1.0);
                if (loftGain > 0.0 && loftAlt != 0.0 && scale > 0.0) {
                    double upx = 0.0, upy = 0.0, upz = 1.0;
                    if (env.earth.useEcefTruth) {
                        const auto geo = Models::ecefToGeodetic(
                            {nav.estPx[id], nav.estPy[id], nav.estPz[id]});
                        upx = std::cos(geo.latitudeRad) * std::cos(geo.longitudeRad);
                        upy = std::cos(geo.latitudeRad) * std::sin(geo.longitudeRad);
                        upz = std::sin(geo.latitudeRad);
                    }
                    const double aLoft = loftGain * loftAlt * scale;
                    out.ax += aLoft * upx;
                    out.ay += aLoft * upy;
                    out.az += aLoft * upz;
                }
            }

            out.tgoSec = range / std::max(sol.closingSpeed, 1e-6);
            return out;
        }

        // Trajectory-aware midcourse guidance on the commanded aim state.
        // Predicts a constant-velocity intercept (PIP + tgo) over the aim
        // (measurement-anchored track wins over the command),
        // gates feasibility against the per-entity maxAccel budget, publishes
        // the prediction/feasibility diagnostics, and commands PN toward the
        // PIP when feasible. When infeasible or unpredicable the demand falls
        // back to bounded PN toward the raw aim (never non-finite) with the
        // reason flagged. Midcourse-only: a seeker lock overrides it upstream.
        LawResult computeTrajectory(
            std::size_t id, const NavigationBlock& nav,
            const TrackBlock* tracks, GuidanceBlock& g,
            const EnvironmentConfig& env)
        {
            LawResult out;
            const double N = effNavN(id, g);
            if (!std::isfinite(N) || N <= 0.0) {
                out.lawInvalid = true;
                out.valid = false;
                g.trajectoryFeasible[id] = false;
                g.trajectoryReason[id] = TrajectoryReason::NonFinite;
                return out;
            }

            // Select the aim state with datalink > track precedence: a
            // measurement-anchored persistent track beats the command state.
            // A datalink source's track (the mothership/AWACS tracking the target)
            // is the best midcourse aim and wins over the receiver's own track.
            double tx, ty, tz, tvx, tvy, tvz;
            double atx = 0.0, aty = 0.0, atz = 0.0;
            bool accelAvailable = false;
            bool datalinkAim = false;
            const int dlSrc = (id < g.datalinkSourceId.size())
                ? g.datalinkSourceId[id] : -1;
            if (dlSrc >= 0 && tracks) {
                const std::size_t src = static_cast<std::size_t>(dlSrc);
                if (src < tracks->size && tracks->active(src) &&
                    tracks->updateCount[src] > 0)
                {
                    datalinkAim = true;
                    g.trajectoryAimSource[id] = GuidanceAimSource::Track;
                    tx = tracks->posX[src]; ty = tracks->posY[src]; tz = tracks->posZ[src];
                    tvx = tracks->velX[src]; tvy = tracks->velY[src]; tvz = tracks->velZ[src];
                    accelAvailable = src < tracks->accelAvailable.size() &&
                                     tracks->accelAvailable[src];
                    if (accelAvailable && src < tracks->accelX.size()) {
                        atx = tracks->accelX[src]; aty = tracks->accelY[src]; atz = tracks->accelZ[src];
                    } else {
                        accelAvailable = false;
                    }
                }
            }
            const bool trackAim = tracks && id < tracks->size &&
                                  tracks->active(id) && tracks->updateCount[id] > 0;
            if (!datalinkAim && trackAim) {
                g.trajectoryAimSource[id] = GuidanceAimSource::Track;
                tx = tracks->posX[id]; ty = tracks->posY[id]; tz = tracks->posZ[id];
                tvx = tracks->velX[id]; tvy = tracks->velY[id]; tvz = tracks->velZ[id];
                accelAvailable = id < tracks->accelAvailable.size() &&
                                 tracks->accelAvailable[id];
                if (accelAvailable && id < tracks->accelX.size()) {
                    atx = tracks->accelX[id]; aty = tracks->accelY[id]; atz = tracks->accelZ[id];
                } else {
                    accelAvailable = false;
                }
            }
            if (!datalinkAim && !trackAim) {
                g.trajectoryAimSource[id] = GuidanceAimSource::Command;
                tx = g.targetX[id]; ty = g.targetY[id]; tz = g.targetZ[id];
                tvx = g.targetVx[id]; tvy = g.targetVy[id]; tvz = g.targetVz[id];
                accelAvailable = g.targetAccelAvailable[id];
                atx = g.targetAccelX[id]; aty = g.targetAccelY[id]; atz = g.targetAccelZ[id];
            }

            const Models::Vec3 ownP{nav.estPx[id], nav.estPy[id], nav.estPz[id]};
            const Models::Vec3 ownV{nav.estVx[id], nav.estVy[id], nav.estVz[id]};
            Models::Vec3 ownA{0.0, 0.0, 0.0};
            bool ownAccelAvailable = false;
            if (id < nav.estAx.size() && id < nav.estAy.size() && id < nav.estAz.size()) {
                ownA = {nav.estAx[id], nav.estAy[id], nav.estAz[id]};
                ownAccelAvailable = std::isfinite(ownA[0]) && std::isfinite(ownA[1]) && std::isfinite(ownA[2]);
            }
            const double minSpeed = g.trajectoryMinSpeedMps[id];
            const double factor = std::isfinite(g.trajectoryFeasibilityAccelFactor[id])
                ? g.trajectoryFeasibilityAccelFactor[id] : 0.95;

            const Models::InterceptResult pred = Models::predictIntercept(
                ownP, ownV,
                {tx, ty, tz}, {tvx, tvy, tvz},
                {atx, aty, atz}, accelAvailable,
                N, minSpeed,
                ownA, ownAccelAvailable);

            // Publish prediction diagnostics.
            g.predictedInterceptX[id] = pred.pip[0];
            g.predictedInterceptY[id] = pred.pip[1];
            g.predictedInterceptZ[id] = pred.pip[2];
            g.predictedTgoSec[id] = pred.valid ? pred.tgoSec : 0.0;
            g.trajectoryRequiredAccel[id] = pred.requiredAccel;

            switch (pred.status) {
                case Models::InterceptStatus::Ok:
                    g.trajectoryReason[id] = TrajectoryReason::Ok;
                    break;
                case Models::InterceptStatus::VelocityLow:
                    g.trajectoryReason[id] = TrajectoryReason::VelocityLow;
                    break;
                case Models::InterceptStatus::NoIntercept:
                    g.trajectoryReason[id] = TrajectoryReason::NoIntercept;
                    break;
                case Models::InterceptStatus::NonFinite:
                    g.trajectoryReason[id] = TrajectoryReason::NonFinite;
                    break;
            }

            const double lim = g.maxAccel[id];
            const bool withinBudget = (lim <= 0.0) ||
                (pred.valid && pred.requiredAccel <= factor * lim);
            if (pred.valid) {
                if (withinBudget) {
                    g.trajectoryFeasible[id] = true;
                } else {
                    g.trajectoryFeasible[id] = false;
                    g.trajectoryReason[id] = TrajectoryReason::AccelLimited;
                }
            } else {
                g.trajectoryFeasible[id] = false;
            }

            if (pred.valid && withinBudget) {
                // Command PN aimed at the predicted intercept point, closed at
                // the intercept-time relative velocity (optional accel).
                const Models::Vec3 rPip = Models::vec3Sub(pred.pip, ownP);
                const Models::Vec3 vClose = {
                    (tvx + (accelAvailable ? atx : 0.0) * pred.tgoSec) - nav.estVx[id],
                    (tvy + (accelAvailable ? aty : 0.0) * pred.tgoSec) - nav.estVy[id],
                    (tvz + (accelAvailable ? atz : 0.0) * pred.tgoSec) - nav.estVz[id]};
                const Models::GuidanceSolution sol =
                    Models::proportionalNavigation(rPip, vClose, N);
                if (sol.valid) {
                    out.ax = sol.acceleration[0];
                    out.ay = sol.acceleration[1];
                    out.az = sol.acceleration[2];
                    out.tgoSec = pred.tgoSec;
                    return out;
                }
                // PIP geometry invalid at evaluation: flag infeasible and fall
                // back to the bounded raw-aim demand below.
                g.trajectoryFeasible[id] = false;
                g.trajectoryReason[id] = TrajectoryReason::NoIntercept;
            }

            // Infeasible / unpredicable: bounded best-effort PN toward the raw
            // aim (same shape as legacy midcourse) while the trajectory
            // diagnostics keep the infeasibility explicit and the demand finite.
            LawResult fallback = computeMidcourse(id, nav, tracks, g, env);
            if (id < g.scaleDemandOnInfeasible.size() && g.scaleDemandOnInfeasible[id] &&
                lim > 0.0 && fallback.valid) {
                const double mag = std::sqrt(fallback.ax * fallback.ax +
                                             fallback.ay * fallback.ay +
                                             fallback.az * fallback.az);
                const double budget = factor * lim;
                if (mag > budget && mag > 1e-9) {
                    const double s = budget / mag;
                    fallback.ax *= s; fallback.ay *= s; fallback.az *= s;
                }
            }
            out.ax = fallback.ax;
            out.ay = fallback.ay;
            out.az = fallback.az;
            out.valid = fallback.valid;
            out.lawInvalid = fallback.lawInvalid;
            out.nonClosing = fallback.nonClosing;
            out.tgoSec = fallback.tgoSec;
            return out;
        }

        // Seeker-rate APN: N * Vc * LOS rate, body-frame mapping per the frame
        // contract (azimuth -> +body-Y, elevation -> -body-Z), rotated to the
        // world frame with the navigation attitude.
        // True APN adds 0.5 * N * a_T_perp target acceleration feedforward when available.
        //
        // Optional gyro decoupling / 3D body PN: the seeker measures the LOS
        // rate in the BODY frame, so it contains the airframe's own rotation.
        // The decoupled path reconstructs the LOS unit vector and its body
        // derivative from (az, el, dAz, dEl), forms the relative angular
        // velocity omega_rel = u x du/dt, adds the body rate (nav gyro) to get
        // the inertial LOS rate, and commands N*Vc*(omega_in x u). With the
        // body rate zero this reproduces the legacy component mapping to first
        // order; the legacy path is kept byte-identical when disabled.
        LawResult computeSeekerAPN(
            std::size_t id, const NavigationBlock& nav,
            const SeekerBlock& seeker, const TrackBlock* tracks, GuidanceBlock& guidance)
        {
            LawResult out;
            const double range = seeker.targetRange[id];
            const double rangeRate = seeker.targetRangeRate[id];
            if (!std::isfinite(range) || !std::isfinite(rangeRate) ||
                !std::isfinite(seeker.targetAzimuthRate[id]) ||
                !std::isfinite(seeker.targetElevationRate[id]))
            {
                // Validate inputs BEFORE publishing diagnostics: a non-finite
                // range or rate must not leak a NaN tgo.
                out.lawInvalid = true;
                out.valid = false;
                return out;
            }
            const double N = rangeShapedN(id, guidance, range);
            if (!std::isfinite(N) || N <= 0.0) {
                out.lawInvalid = true;
                out.valid = false;
                return out;
            }
            const double vc = std::max(std::abs(rangeRate), 1.0);
            const double dAz = seeker.targetAzimuthRate[id];
            const double dEl = seeker.targetElevationRate[id];
            out.nonClosing = rangeRate > 0.0;
            out.tgoSec = range / std::max(std::abs(rangeRate), 1.0);
            if (id < guidance.closingSpeed.size()) guidance.closingSpeed[id] = vc;
            if (id < guidance.losRateMag.size()) {
                guidance.losRateMag[id] = std::sqrt(dAz * dAz + dEl * dEl);
            }

            const double az = seeker.targetAzimuth[id];
            const double el = seeker.targetElevation[id];
            const double cEl = std::cos(el), sEl = std::sin(el);
            glm::dquat estQ(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id]);
            const glm::dvec3 losBody(cEl * std::cos(az), cEl * std::sin(az), -sEl);

            const bool bodyPN = id < guidance.terminalLaw.size() &&
                                guidance.terminalLaw[id] == 1;
            const bool decoupled = bodyPN ||
                (id < guidance.gyroDecouplingEnabled.size() &&
                 guidance.gyroDecouplingEnabled[id]);

            glm::dvec3 aWorld;
            if (!decoupled) {
                const double ayBody = N * vc * dAz;
                const double azBody = -N * vc * dEl;
                aWorld = estQ * glm::dvec3(0.0, ayBody, azBody);
            } else {
                const double ca = std::cos(az), sa = std::sin(az);
                // du/dt in the body frame from the angular rates.
                const glm::dvec3 du(
                    -sEl * ca * dEl - cEl * sa * dAz,
                    -sEl * sa * dEl + cEl * ca * dAz,
                    -cEl * dEl);
                glm::dvec3 omegaIn = glm::cross(losBody, du);
                if (id < nav.estWx.size()) {
                    omegaIn += glm::dvec3(nav.estWx[id], nav.estWy[id], nav.estWz[id]);
                }
                aWorld = estQ * (N * vc * glm::cross(omegaIn, losBody));
            }

            // True APN target-acceleration feedforward augmentation:
            // a_cmd = a_PN + 0.5 * N * a_T_perp
            bool ffAvailable = false;
            double atx = 0.0, aty = 0.0, atz = 0.0;
            if (tracks && id < tracks->accelAvailable.size() &&
                tracks->active(id) && tracks->accelAvailable[id] &&
                qualityAbove(tracks, id, guidance.apnFeedforwardMinQuality01)) {
                ffAvailable = isFinite3(tracks->accelX[id], tracks->accelY[id], tracks->accelZ[id]);
                atx = tracks->accelX[id]; aty = tracks->accelY[id]; atz = tracks->accelZ[id];
            } else if (guidance.targetAccelAvailable[id]) {
                ffAvailable = isFinite3(guidance.targetAccelX[id], guidance.targetAccelY[id], guidance.targetAccelZ[id]);
                atx = guidance.targetAccelX[id]; aty = guidance.targetAccelY[id]; atz = guidance.targetAccelZ[id];
            }

            if (guidance.apnFeedforwardEnabled[id] && ffAvailable) {
                const glm::dvec3 losWorld = glm::normalize(estQ * losBody);
                const glm::dvec3 aT(atx, aty, atz);
                const glm::dvec3 aTperp = aT - glm::dot(aT, losWorld) * losWorld;
                aWorld += 0.5 * N * aTperp;
            }

            out.ax = aWorld.x;
            out.ay = aWorld.y;
            out.az = aWorld.z;
            return out;
        }
    }

    void GuidanceSystem::update(
        const EntityStatusBlock& status,
        const NavigationBlock& nav,
        const SeekerBlock& seeker,
        const TrackBlock& tracks,
        GuidanceBlock& guidance,
        ControlBlock& control,
        double dt,
        const EnvironmentConfig& environment)
    {
        for (std::size_t i = 0; i < nav.size; ++i) {
            if (!status.isAlive[i]) continue;

            // tgo-scheduled N (opt-in): previous step's tgo selects the
            // terminal gain inside the window, else the base gain. Disabled
            // (or non-positive tgo) reproduces the constant-N legacy path.
            {
                double schedN = guidance.navigationConstant[i];
                if (i < guidance.navScheduleEnabled.size() &&
                    guidance.navScheduleEnabled[i] &&
                    i < guidance.navConstantTerminal.size() &&
                    i < guidance.navScheduleTgoSec.size()) {
                    const double tgo = (i < guidance.tgoSec.size()) ? guidance.tgoSec[i] : 0.0;
                    const double win = guidance.navScheduleTgoSec[i];
                    const double nTerm = guidance.navConstantTerminal[i];
                    if (tgo > 0.0 && win > 0.0 && tgo <= win && nTerm > 0.0 &&
                        std::isfinite(tgo) && std::isfinite(nTerm)) {
                        schedN = nTerm;
                    }
                }
                if (i < guidance.scheduledNavN.size()) guidance.scheduledNavN[i] = schedN;
            }

            clearTrajectoryDiagnostics(i, guidance);

            auto& phase = guidance.phase[i];
            auto& law = guidance.law[i];

            // Communication failure: ignore guidance and seeker handoff;
            // the entity flies ballistic with zero commanded acceleration.
            if (i < status.commsFailed.size() && status.commsFailed[i]) {
                zeroDemand(i, guidance);
                phase = GuidancePhase::None;
                law = GuidanceLaw::None;
                continue;
            }

            const bool seekerPresent = seeker.type[i] != SeekerType::None;
            const bool locked = seekerPresent && seeker.isLocked[i];

            // Authority feedback from the previous autopilot step: scale the
            // demand by the delivered/demanded fin margin when enabled.
            double authorityScale = 1.0;
            if (i < guidance.authorityAwareLimitEnabled.size() &&
                guidance.authorityAwareLimitEnabled[i] &&
                i < control.authorityMargin01.size()) {
                authorityScale = std::clamp(control.authorityMargin01[i], 0.1, 1.0);
            }

            // Track bookkeeping. trackAgeSec counts time since the last valid
            // seeker track (diagnostic + retention window).
            if (locked) {
                guidance.trackAgeSec[i] = 0.0;
                guidance.trackId[i] = static_cast<std::int64_t>(seeker.lockedTargetId[i]);
                if (i < guidance.trackLossActive.size()) guidance.trackLossActive[i] = false;
            } else if (seekerPresent) {
                guidance.trackAgeSec[i] += dt;
            }

            // --- Terminal homing path (seeker lock; overrides the configured
            // --- mode, matching the legacy lock-override contract). This override
            // --- applies ONLY to interceptor guidance (PN/Waypoint/Trajectory).
            // --- A Cruise-mode aircraft (e.g. the Tejas mothership with its own
            // --- fire-control radar) must NOT be pulled into seeker homing by its
            // --- radar lock - the lock is for targeting/situational awareness and
            // --- the aircraft keeps flying its altitude-hold + waypoint course.
            if (locked && guidance.mode[i] != GuidanceMode::Cruise) {
                const double ramp = guidance.handoffBlendTimeSec[i];
                const bool freshLock =
                    phase != GuidancePhase::Acquisition &&
                    phase != GuidancePhase::Terminal;
                if (freshLock) {
                    // New lock: either instant handoff (legacy) or start the
                    // acquisition blend at zero APN weight.
                    if (ramp <= 0.0) {
                        guidance.handoffWeight[i] = 1.0;
                        phase = GuidancePhase::Terminal;
                    } else {
                        guidance.handoffWeight[i] = 0.0;
                        phase = GuidancePhase::Acquisition;
                    }
                }
                // Advance the blend weight every locked step (including the
                // fresh-lock step). Deterministic ramp over blend time.
                double w = guidance.handoffWeight[i];
                if (w < 1.0 && ramp > 0.0) {
                    w = std::min(1.0, w + dt / ramp);
                    guidance.handoffWeight[i] = w;
                }
                if (w >= 1.0) phase = GuidancePhase::Terminal;

                law = (i < guidance.terminalLaw.size() && guidance.terminalLaw[i] == 1)
                    ? GuidanceLaw::BodyPN : GuidanceLaw::SeekerRateAPN;
                LawResult apn = computeSeekerAPN(i, nav, seeker, &tracks, guidance);
                LawResult out = apn;
                if (phase == GuidancePhase::Acquisition) {
                    // Blend midcourse PN -> terminal APN (deterministic ramp).
                    const LawResult pn = computeMidcourse(i, nav, &tracks, guidance, environment);
                    if (!apn.valid && pn.valid) {
                        // Seeker solution unusable: fly pure midcourse rather
                        // than zeroing a valid demand.
                        out = pn;
                    } else {
                        const double w = guidance.handoffWeight[i];
                        out.ax = (1.0 - w) * (pn.valid ? pn.ax : 0.0) + w * apn.ax;
                        out.ay = (1.0 - w) * (pn.valid ? pn.ay : 0.0) + w * apn.ay;
                        out.az = (1.0 - w) * (pn.valid ? pn.az : 0.0) + w * apn.az;
                        out.valid = apn.valid || pn.valid;
                        out.lawInvalid = apn.lawInvalid && pn.lawInvalid;
                        out.nonClosing = pn.nonClosing || apn.nonClosing;
                    }
                }
                // Gimbal-edge hold: a target sitting at/near the seeker
                // gimbal edge drives a churny, high-LOS-rate APN command (the
                // observed instability). Realistically the missile keeps flying
                // its midcourse collision course (the datalink aim) and lets the
                // seeker re-centre / scan, rather than steering hard toward an
                // off-boresight target. Blend the APN toward the midcourse course
                // as the target's off-boresight angle approaches the cone edge.
                if (i < seeker.targetAzimuth.size() &&
                    i < seeker.gimbalAzimuthLimitRad.size() &&
                    i < seeker.gimbalElevationLimitRad.size())
                {
                    const double azAbs = std::abs(seeker.targetAzimuth[i]);
                    const double elAbs = std::abs(seeker.targetElevation[i]);
                    const double holdAz = 0.8 * std::max(1e-6, seeker.gimbalAzimuthLimitRad[i]);
                    const double holdEl = 0.8 * std::max(1e-6, seeker.gimbalElevationLimitRad[i]);
                    const double targetOff = std::sqrt(azAbs * azAbs + elAbs * elAbs);
                    const double coneRad = std::sqrt(holdAz * holdAz + holdEl * holdEl);
                    const double w = (coneRad > 1e-6)
                        ? std::clamp(1.0 - targetOff / coneRad, 0.0, 1.0) : 0.0;
                    if (w < 1.0) {
                        const LawResult mc = computeMidcourse(i, nav, &tracks, guidance, environment);
                        if (mc.valid) {
                            out.ax = w * out.ax + (1.0 - w) * mc.ax;
                            out.ay = w * out.ay + (1.0 - w) * mc.ay;
                            out.az = w * out.az + (1.0 - w) * mc.az;
                            out.valid = true;
                        }
                        // NOTE: the phase is deliberately LEFT at Terminal /
                        // Acquisition here even when fully off-cone (w = 0,
                        // demand already pure midcourse). Marking LostTrack
                        // while still locked makes the next step look like a
                        // fresh lock (handoff weight re-zeroed every step at
                        // the cone edge) and, worse, a later genuine unlock
                        // skips the lock-loss counter + retention window
                        // below (they only run from Acquisition/Terminal).
                    }
                }
                applyDemand(i, out, guidance, dt, authorityScale);
                // Refresh the bounded retained terminal command (post-clamp)
                // used during a lock-loss retention window.
                guidance.retainedAccelX[i] = guidance.commandedAccelX[i];
                guidance.retainedAccelY[i] = guidance.commandedAccelY[i];
                guidance.retainedAccelZ[i] = guidance.commandedAccelZ[i];
                continue;
            }

            // --- Not locked / no seeker: midcourse or recovery -------------
            if (seekerPresent &&
                (phase == GuidancePhase::Acquisition || phase == GuidancePhase::Terminal))
            {
                // Terminal track just lost: count the episode once. With the
                // episode flag available it is dt-robust; hand-built blocks
                // without it fall back to the legacy age heuristic.
                bool countLoss;
                if (i < guidance.trackLossActive.size()) {
                    countLoss = !guidance.trackLossActive[i];
                    guidance.trackLossActive[i] = true;
                } else {
                    countLoss = guidance.trackAgeSec[i] <= dt * 1.5;
                }
                if (countLoss) {
                    ++guidance.lockLossCount[i];
                }
                const double retain = guidance.lockLossRetentionSec[i];
                if (retain > 0.0 && guidance.trackAgeSec[i] <= retain) {
                    // Retention window: apply the bounded predicted terminal
                    // command (last valid seeker-APN demand, already clamped by
                    // maxAccel) while the track identity/age is retained.
                    phase = GuidancePhase::Terminal;
                    law = (i < guidance.terminalLaw.size() && guidance.terminalLaw[i] == 1)
                        ? GuidanceLaw::BodyPN : GuidanceLaw::SeekerRateAPN;
                    LawResult retained;
                    retained.ax = guidance.retainedAccelX[i];
                    retained.ay = guidance.retainedAccelY[i];
                    retained.az = guidance.retainedAccelZ[i];
                    retained.valid = isFinite3(retained.ax, retained.ay, retained.az);
                    retained.tgoSec = guidance.tgoSec[i];  // last valid tgo
                    applyDemand(i, retained, guidance, dt, authorityScale);
                    continue;
                }
                phase = GuidancePhase::LostTrack; // recovery via midcourse PN
            }

            auto mode = guidance.mode[i];
            if (mode == GuidanceMode::None) {
                zeroDemand(i, guidance);
                phase = GuidancePhase::None;
                law = GuidanceLaw::None;
                continue;
            }

            // A terminal-lost state (LostTrack, or retained Terminal) keeps its
            // phase marking while the midcourse recovery demand is applied.
            const bool terminalLost =
                phase == GuidancePhase::LostTrack || phase == GuidancePhase::Terminal;

            if (mode == GuidanceMode::ProportionalNavigation) {
                if (!terminalLost) phase = GuidancePhase::Midcourse;
                law = (guidance.apnFeedforwardEnabled[i] &&
                       guidance.targetAccelAvailable[i])
                    ? GuidanceLaw::AugmentedProNav : GuidanceLaw::PureProNav;
                applyDemand(i, computeMidcourse(i, nav, &tracks, guidance, environment), guidance, dt, authorityScale);
            } else if (mode == GuidanceMode::Waypoint) {
                if (!terminalLost) phase = GuidancePhase::Midcourse;
                law = GuidanceLaw::Waypoint;
                double rx = guidance.targetX[i] - nav.estPx[i];
                double ry = guidance.targetY[i] - nav.estPy[i];
                double rz = guidance.targetZ[i] - nav.estPz[i];
                const double k = guidance.waypointGain[i];
                LawResult wp;
                if (!isFinite3(rx, ry, rz) || !std::isfinite(k)) {
                    // Non-finite waypoint geometry must not produce an invalid
                    // acceleration command; mark it explicitly and zero it.
                    wp.lawInvalid = true;
                    wp.valid = false;
                    applyDemand(i, wp, guidance, dt, authorityScale);
                    continue;
                }
                double rMag = std::sqrt(rx * rx + ry * ry + rz * rz);
                if (rMag < 1.0) rMag = 1.0;
                wp.ax = k * (rx / rMag);
                wp.ay = k * (ry / rMag);
                wp.az = k * (rz / rMag);
                wp.tgoSec = rMag / 300.0;  // diagnostic only (nominal 300 m/s)
                applyDemand(i, wp, guidance, dt, authorityScale);
            } else if (mode == GuidanceMode::Trajectory) {
                if (!terminalLost) phase = GuidancePhase::Midcourse;
                law = GuidanceLaw::Trajectory;
                applyDemand(i, computeTrajectory(i, nav, &tracks, guidance, environment), guidance, dt, authorityScale);
            } else if (mode == GuidanceMode::Cruise) {
                if (!terminalLost) phase = GuidancePhase::Midcourse;
                law = GuidanceLaw::Cruise;
                applyDemand(i, computeCruise(i, nav, guidance, environment), guidance, dt, authorityScale);
            } else {
                zeroDemand(i, guidance);
                phase = GuidancePhase::None;
                law = GuidanceLaw::None;
            }
        }
    }

} // namespace StrikeEngine::Kernel
