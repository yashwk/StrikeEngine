#include <strikeengine/kernel/systems/GuidanceSystem.hpp>
#include <strikeengine/models/guidance/GuidanceModels.hpp>
#include <cstdio>
#include <cstdlib>
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
        struct LawResult {
            double ax = 0.0, ay = 0.0, az = 0.0;
            bool valid = true;      // law could be evaluated (geometry OK)
            bool lawInvalid = false; // non-finite input / bad configuration
            bool nonClosing = false; // range > 0 but closing speed <= 0
            double tgoSec = 0.0;    // estimated time to go (diagnostic)
            bool ffUsed = false;    // APN feed-forward actually contributed
        };

        bool isFinite3(double x, double y, double z)
        {
            return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        }

        // Midcourse loft (opt-in): a climb demand that drives the round toward
        // loftAltitudeM and fades out as the intercept nears. The demand is
        // proportional to the ALTITUDE ERROR (a constant climb demand would
        // fly the missile out of the atmosphere — the field was implemented as
        // gain * altitude with no feedback, which is why no scenario used it).
        // Midcourse loft shaping (opt-in). A long-range round that flies the
        // straight PN collision course from a low apex cannot reach far targets:
        // it must trade kinetic for potential energy early and get it back in
        // the endgame. The law is a flight-path-angle hold toward the sightline
        // biased upward by guidanceLoftAngleDeg; the bias fades with range so
        // terminal guidance sees the pure PN collision course again. A pure
        // climb demand is NOT used: it would fly the round out of the
        // atmosphere, where no fin can pull it back down.
        void applyMidcourseLoft(std::size_t id, const NavigationBlock& nav,
                                const GuidanceBlock& guidance,
                                const EnvironmentConfig& env, LawResult& out)
        {
            if (id >= guidance.loftEnabled.size() || !guidance.loftEnabled[id]) return;
            // Midcourse only: once the seeker is in the handoff (Acquisition)
            // or homing (Terminal), the collision course belongs to the
            // terminal law. A residual climb bias there is pure miss distance.
            if (id < guidance.phase.size()) {
                const auto ph = guidance.phase[id];
                if (ph == GuidancePhase::Acquisition || ph == GuidancePhase::Terminal) {
                    return;
                }
            }
            const double angleDeg = (id < guidance.loftAngleDeg.size())
                ? guidance.loftAngleDeg[id] : 0.0;
            const double gain = (id < guidance.loftGain.size()) ? guidance.loftGain[id] : 0.0;
            const double loftRange = (id < guidance.loftRangeM.size())
                ? guidance.loftRangeM[id] : 0.0;
            if (!(angleDeg > 0.0) || !(gain > 0.0) || !(loftRange > 0.0)) return;

            // Sightline to the commanded aim. The law may prefer a datalink or
            // track aim; the shaping bias only needs the direction to within a
            // degree, and the command state is always published. An unset
            // command aim (the origin) has no sightline: skip the shaping.
            if (id >= guidance.targetX.size()) return;
            const double rx = guidance.targetX[id] - nav.estPx[id];
            const double ry = guidance.targetY[id] - nav.estPy[id];
            const double rz = guidance.targetZ[id] - nav.estPz[id];
            const double range = std::sqrt(rx * rx + ry * ry + rz * rz);
            const double aimLen = std::sqrt(guidance.targetX[id] * guidance.targetX[id] +
                                            guidance.targetY[id] * guidance.targetY[id] +
                                            guidance.targetZ[id] * guidance.targetZ[id]);
            if (!(range > 1.0) || !(aimLen > 1.0) || !std::isfinite(range)) return;
            const double sx = rx / range, sy = ry / range, sz = rz / range;

            const double vx = nav.estVx[id], vy = nav.estVy[id], vz = nav.estVz[id];
            const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
            if (!(speed > 1.0) || !std::isfinite(speed)) return;
            const double ux = vx / speed, uy = vy / speed, uz = vz / speed;

            // Local up and the current altitude (geodetic under ECEF truth).
            double wx = 0.0, wy = 0.0, wz = 1.0;
            double currentAlt = nav.estPz[id];
            if (env.earth.useEcefTruth) {
                const auto geo = Models::ecefToGeodetic(
                    {nav.estPx[id], nav.estPy[id], nav.estPz[id]});
                wx = std::cos(geo.latitudeRad) * std::cos(geo.longitudeRad);
                wy = std::cos(geo.latitudeRad) * std::sin(geo.longitudeRad);
                wz = std::sin(geo.latitudeRad);
                currentAlt = geo.altitudeM;
            }

            // Fade the bias out with range: full loft out at loftRangeM, none
            // at the target, so the terminal collision course is untouched.
            const double fade = std::clamp(range / loftRange, 0.0, 1.0);
            constexpr double kPi = 3.14159265358979323846;
            const double losElev = std::asin(std::clamp(sx * wx + sy * wy + sz * wz, -1.0, 1.0));
            const double gammaCmd = losElev + angleDeg * kPi / 180.0 * fade;
            const double gamma = std::asin(std::clamp(ux * wx + uy * wy + uz * wz, -1.0, 1.0));
            const double dGamma = gammaCmd - gamma;

            // Optional apex ceiling: above it the climb bias is off (a round
            // past its design apex must not be driven higher), while a demand
            // that is already descending still applies.
            const double ceiling = (id < guidance.loftAltitudeM.size())
                ? guidance.loftAltitudeM[id] : 0.0;
            if (ceiling > 0.0 && currentAlt >= ceiling && dGamma > 0.0) return;

            // The demand turns the velocity inside its own vertical plane
            // (no azimuth pull: the PN kernel owns the collision heading).
            // A vertical round has no heading: tilt it toward the aim, which
            // is the gas-thruster pitch-over the loft is replacing.
            double hx = ux - (ux * wx + uy * wy + uz * wz) * wx;
            double hy = uy - (ux * wx + uy * wy + uz * wz) * wy;
            double hz = uz - (ux * wx + uy * wy + uz * wz) * wz;
            double hn = std::sqrt(hx * hx + hy * hy + hz * hz);
            if (hn < 1e-6) {
                hx = sx - (sx * wx + sy * wy + sz * wz) * wx;
                hy = sy - (sx * wx + sy * wy + sz * wz) * wy;
                hz = sz - (sx * wx + sy * wy + sz * wz) * wz;
                hn = std::sqrt(hx * hx + hy * hy + hz * hz);
            }
            if (!(hn > 1e-6)) return;
            hx /= hn; hy /= hn; hz /= hn;

            // a = gain * V * (commanded velocity direction - current): a
            // first-order angle chase with time constant ~1/gain. The demand
            // scales with speed, so it can never command a turn the round has
            // no speed to fly, and the chord form stays defined when the round
            // is vertical (where a flight-path-angle gradient is not).
            const double cg = std::cos(gammaCmd), sg = std::sin(gammaCmd);
            out.ax += gain * speed * (cg * hx + sg * wx - ux);
            out.ay += gain * speed * (cg * hy + sg * wy - uy);
            out.az += gain * speed * (cg * hz + sg * wz - uz);
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

        // Is a track's VELOCITY usable for a lead? Used for the round's OWN
        // (seeker) track only: a remote radar's velocity state is
        // noise-dominated when the angular measurements are coarse relative to
        // the range (at 265 km with 0.0025 rad the KF velocity ran to km/s),
        // which is why remote aims take their lead from the command solution
        // via remoteAimVelocity.
        constexpr double kAimVelStdLimitMps = 150.0;

        // The velocity to build a lead from, given a remote (datalink) track.
        // A remote radar's POSITION is fire-control grade, but its velocity
        // state is only as good as the remote filter tracks the angular
        // history -- on a long-range, coarse-angle picture that state can
        // diverge into the km/s range (observed: the S-400 battery's track of
        // a 175 m/s transport read 1.4-2.5 km/s). The launching platform's own
        // solution (the command state, seeded at launch) is the better lead,
        // and the round's own seeker track takes over as it locks.
        inline void remoteAimVelocity(
            const GuidanceBlock& g, const TrackBlock& tracks, std::size_t id,
            std::size_t src, double& vx, double& vy, double& vz)
        {
            const bool commandHasSpeed = id < g.targetVx.size() &&
                (g.targetVx[id] * g.targetVx[id] + g.targetVy[id] * g.targetVy[id] +
                 g.targetVz[id] * g.targetVz[id]) > 1.0;
            if (commandHasSpeed) {
                vx = g.targetVx[id]; vy = g.targetVy[id]; vz = g.targetVz[id];
                return;
            }
            vx = tracks.velX[src]; vy = tracks.velY[src]; vz = tracks.velZ[src];
        }
        inline bool aimVelocityTrusted(const TrackBlock& t, std::size_t i) {
            if (i >= t.velocityStdMs.size()) return true;   // hand-built blocks
            const double vStd = t.velocityStdMs[i];
            if (!(vStd > 0.0) || !std::isfinite(vStd)) return true;
            const double speed = std::sqrt(
                t.velX[i] * t.velX[i] + t.velY[i] * t.velY[i] + t.velZ[i] * t.velZ[i]);
            return vStd <= kAimVelStdLimitMps || vStd <= 0.5 * speed;
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
        // When the seeker is locked, the round's OWN position fix (fresh,
        // short-range, fire-control grade) wins over the remote datalink
        // track: a long-range offboard picture carries the source platform's
        // nav-attitude error projected over the range (kilometres of bias).
        // The lead still comes from the launcher's fire-control solution
        // when it provided one -- no differentiated angle velocity (remote
        // or own) is lead-grade. Before lock (midcourse) the datalink
        // picture is the only aim and keeps precedence.
        LawResult computeAimPn(
            std::size_t id, const NavigationBlock& nav,
            const TrackBlock* tracks, GuidanceBlock& guidance,
            bool seekerLocked)
        {
            LawResult out;
            const double baseN = effNavN(id, guidance);
            if (!std::isfinite(baseN) || baseN <= 0.0) {
                out.lawInvalid = true;
                out.valid = false;
                return out;
            }

            // Select the aim state (precedence below): a configured
            // track-quality floor can disqualify a track (default 0 = no
            // gate), and a track-built lead additionally needs a trusted
            // velocity.
            auto commandHasSpeed = [&]() -> bool {
                return id < guidance.targetVx.size() &&
                    (guidance.targetVx[id] * guidance.targetVx[id] +
                     guidance.targetVy[id] * guidance.targetVy[id] +
                     guidance.targetVz[id] * guidance.targetVz[id]) > 1.0;
            };
            auto ownTrackUsable = [&]() -> bool {
                return tracks && id < tracks->size && tracks->active(id) &&
                    tracks->updateCount[id] > 0 &&
                    qualityAbove(tracks, id, guidance.trackAimMinQuality01) &&
                    aimVelocityTrusted(*tracks, id);
            };
            // Position-only usability: the onboard position fix stays
            // fire-control grade long after the velocity state stops being
            // lead-grade (differentiated from coarse angles). The lead then
            // comes from the command solution (see below).
            auto ownPosUsable = [&]() -> bool {
                return tracks && id < tracks->size && tracks->active(id) &&
                    tracks->updateCount[id] > 0 &&
                    qualityAbove(tracks, id, guidance.trackAimMinQuality01);
            };
            auto datalinkUsable = [&](std::size_t& src) -> bool {
                const int dl = (id < guidance.datalinkSourceId.size())
                    ? guidance.datalinkSourceId[id] : -1;
                if (dl < 0 || !tracks) return false;
                src = static_cast<std::size_t>(dl);
                return src < tracks->size && tracks->active(src) &&
                    tracks->updateCount[src] > 0 &&
                    qualityAbove(tracks, src, guidance.trackAimMinQuality01);
            };
            double tx, ty, tz, tvx, tvy, tvz;
            bool trackAim = false;
            bool datalinkAim = false;
            std::size_t datalinkSrc = 0;
            const bool ownFirst = seekerLocked && ownPosUsable() &&
                (commandHasSpeed() || ownTrackUsable());
            if (ownFirst) {
                // Locked: the onboard position fix (short-range, anchored at
                // the current geometry) with the fire-control lead. A track
                // velocity differentiated from coarse angle measurements is
                // not lead-grade even when it is the round's own, so the
                // command solution leads whenever the launcher provided one.
                trackAim = true;
                tx = tracks->posX[id]; ty = tracks->posY[id]; tz = tracks->posZ[id];
                if (commandHasSpeed()) {
                    tvx = guidance.targetVx[id];
                    tvy = guidance.targetVy[id];
                    tvz = guidance.targetVz[id];
                } else {
                    tvx = tracks->velX[id]; tvy = tracks->velY[id]; tvz = tracks->velZ[id];
                }
            }
            if (!trackAim && datalinkUsable(datalinkSrc)) {
                datalinkAim = true;
                const std::size_t src = datalinkSrc;
                tx = tracks->posX[src]; ty = tracks->posY[src]; tz = tracks->posZ[src];
                remoteAimVelocity(guidance, *tracks, id, src, tvx, tvy, tvz);
            }
            if (!datalinkAim && !trackAim && ownTrackUsable()) {
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
            if (std::getenv("STRIKE_AIM_TRACE") != nullptr && id == 2) {
                static int tick = 0;
                if ((tick++ % 500) == 0) {
                    const double aimV = std::sqrt(tvx * tvx + tvy * tvy + tvz * tvz);
                    const int srcId = (datalinkAim && tracks && datalinkSrc < tracks->size)
                        ? tracks->trackId[datalinkSrc] : -1;
                    const double srcUpd = (datalinkAim && tracks && datalinkSrc < tracks->size)
                        ? static_cast<double>(tracks->updateCount[datalinkSrc]) : -1.0;
                    const double srcVel = (datalinkAim && tracks && datalinkSrc < tracks->size)
                        ? std::sqrt(tracks->velX[datalinkSrc] * tracks->velX[datalinkSrc] +
                                    tracks->velY[datalinkSrc] * tracks->velY[datalinkSrc] +
                                    tracks->velZ[datalinkSrc] * tracks->velZ[datalinkSrc])
                        : -1.0;
                    const double rng = std::sqrt(rx * rx + ry * ry + rz * rz);
                    std::fprintf(stderr,
                                 "[aim] src=%s id=%d upd=%.0f srcV=%.0f aimV=%.0f m/s "
                                 "range=%.0f km ownV=%.0f m/s\n",
                                 datalinkAim ? "datalink" : (trackAim ? "own" : "command"),
                                 srcId, srcUpd, srcVel, aimV, rng / 1000.0,
                                 std::sqrt(nav.estVx[id] * nav.estVx[id] +
                                           nav.estVy[id] * nav.estVy[id] +
                                           nav.estVz[id] * nav.estVz[id]));
                }
            }
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
                ? Models::apn(r, v, {atx, aty, atz}, N)
                : Models::tpn(r, v, N);
            out.ffUsed = feedforward;

            if (!sol.valid) {
                out.valid = false;
                out.nonClosing = range > 1e-9 && closingSpeed <= 0.0;
                return out;
            }
            out.ax = sol.acceleration[0];
            out.ay = sol.acceleration[1];
            out.az = sol.acceleration[2];


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
            const TrackBlock* tracks, GuidanceBlock& g)
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
                    tracks->updateCount[src] > 0 &&
                    qualityAbove(tracks, src, g.trackAimMinQuality01))
                {
                    datalinkAim = true;
                    g.trajectoryAimSource[id] = GuidanceAimSource::Track;
                    tx = tracks->posX[src]; ty = tracks->posY[src]; tz = tracks->posZ[src];
                    remoteAimVelocity(g, *tracks, id, src, tvx, tvy, tvz);
                    // A remote (datalink) track's acceleration estimate is NOT
                    // guidance-grade: it is differentiated from range/angle
                    // measurements and rails to the filter's accel bound on a
                    // stressing geometry, so feeding it into the intercept
                    // prediction threw the aim (and the midcourse) far off --
                    // hundreds of m/s^2 of phantom demand. Position and velocity
                    // from the same track stay usable; acceleration only counts
                    // when it comes from the seeker's own track.
                    accelAvailable = false;
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
                    Models::tpn(rPip, vClose, N);
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
            // Midcourse context: the datalink picture keeps aim precedence.
            LawResult fallback = computeAimPn(id, nav, tracks, g, false);
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
        LawResult computeSeekerPn(
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
            // Collapse on non-closing geometry, mirroring the midcourse law:
            // a locked missile must not steer hard at a receding target. The
            // blend logic falls back to midcourse PN (which collapses the
            // same way) when the seeker solution is unusable.
            if (rangeRate > 0.0) {
                if (id < guidance.closingSpeed.size()) guidance.closingSpeed[id] = -rangeRate;
                out.nonClosing = true;
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

            // Gyro-decoupled inertial LOS rate: reconstruct du/dt from the
            // seeker's measured angular rates, add the host body rate, then
            // evaluate the shared PN kernel (see GuidanceModels) in the world
            // frame — the same demand implementation as the aim-based source.
            const double ca = std::cos(az), sa = std::sin(az);
            const glm::dvec3 du(
                -sEl * ca * dEl - cEl * sa * dAz,
                -sEl * sa * dEl + cEl * ca * dAz,
                -cEl * dEl);
            const glm::dvec3 losWorld = glm::normalize(estQ * losBody);
            glm::dvec3 omegaWorld;
            if (id < seeker.losRateWorldValid.size() && seeker.losRateWorldValid[id]) {
                // Preferred: the seeker reconstructed consecutive world-frame
                // LOS vectors and published their rotation. No body-rate
                // pairing, so latency/filter/skipped-step mismatches cannot
                // leave a residual comparable to the LOS rate itself.
                omegaWorld = glm::dvec3(seeker.losRateWorldX[id],
                                        seeker.losRateWorldY[id],
                                        seeker.losRateWorldZ[id]);
            } else {
                glm::dvec3 omegaIn = glm::cross(losBody, du);
                if (id < seeker.bodyRateFilteredX.size()) {
                    // Legacy fallback: interval-averaged, filter-matched body
                    // rate added to the body-frame LOS rate.
                    omegaIn += glm::dvec3(seeker.bodyRateFilteredX[id],
                                          seeker.bodyRateFilteredY[id],
                                          seeker.bodyRateFilteredZ[id]);
                } else if (id < nav.estWx.size()) {
                    omegaIn += glm::dvec3(nav.estWx[id], nav.estWy[id], nav.estWz[id]);
                }
                omegaWorld = estQ * omegaIn;
            }
            const Models::Vec3 demand = Models::pnDemand(
                {losWorld.x, losWorld.y, losWorld.z},
                {omegaWorld.x, omegaWorld.y, omegaWorld.z}, vc, N);
            glm::dvec3 aWorld(demand[0], demand[1], demand[2]);

            // Target-acceleration feed-forward source: the persistent track
            // when measurement-anchored and available, else the commanded
            // state (same precedence as the original terminal path).
            bool ffAvailable = false;
            double atx = 0.0, aty = 0.0, atz = 0.0;
            const bool ownTrackSelected = tracks && id < tracks->size &&
                tracks->active(id) && id < tracks->updateCount.size() &&
                tracks->updateCount[id] > 0;
            if (ownTrackSelected && id < tracks->accelAvailable.size() &&
                tracks->accelAvailable[id] &&
                qualityAbove(tracks, id, guidance.apnFeedforwardMinQuality01)) {
                ffAvailable = isFinite3(tracks->accelX[id], tracks->accelY[id], tracks->accelZ[id]);
                atx = tracks->accelX[id]; aty = tracks->accelY[id]; atz = tracks->accelZ[id];
            } else if (!ownTrackSelected && guidance.targetAccelAvailable[id]) {
                ffAvailable = isFinite3(guidance.targetAccelX[id], guidance.targetAccelY[id], guidance.targetAccelZ[id]);
                atx = guidance.targetAccelX[id]; aty = guidance.targetAccelY[id]; atz = guidance.targetAccelZ[id];
            }

            // True APN target-acceleration feedforward augmentation:
            // a_cmd = a_PN + 0.5 * N * a_T_perp
            if (guidance.apnFeedforwardEnabled[id] && ffAvailable) {
                const Models::Vec3 ff = Models::normalAcceleration(
                    {losWorld.x, losWorld.y, losWorld.z}, {atx, aty, atz});
                aWorld += 0.5 * N * glm::dvec3(ff[0], ff[1], ff[2]);
                out.ffUsed = true;
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
                    // acquisition blend at zero seeker weight.
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

                // Phase = source + blend: one PN kernel, evaluated through both
                // sources, weighted. The midcourse source (track/command aim)
                // is always evaluated; the seeker source ramps in with the
                // handoff weight and is gated by the gimbal-edge cone. While
                // locked, the midcourse source itself prefers the round's own
                // (seeker) track over the remote datalink picture.
                LawResult out = computeAimPn(i, nav, &tracks, guidance, true);
                // Midcourse energy shaping (opt-in loft) applies to whichever
                // branch the law took: feed-forward, aim/track, or the
                // infeasible-trajectory fallback. Applying it inside one
                // branch silently disabled it for the others.
                applyMidcourseLoft(i, nav, guidance, environment, out);
                const LawResult seek = computeSeekerPn(i, nav, seeker, &tracks, guidance);

                // Gimbal-edge hold: a target at/near the seeker gimbal edge
                // drives a churny, high-LOS-rate demand. Weight the seeker
                // source down as the off-boresight angle approaches the cone
                // edge so the missile keeps flying its midcourse collision
                // course and lets the seeker re-centre / scan.
                double wSeeker = w;
                if (out.valid) {
                    if (i < seeker.targetAzimuth.size() &&
                        i < seeker.gimbalAzimuthLimitRad.size() &&
                        i < seeker.gimbalElevationLimitRad.size()) {
                        const double azAbs = std::abs(seeker.targetAzimuth[i]);
                        const double elAbs = std::abs(seeker.targetElevation[i]);
                        const double holdAz = 0.8 * std::max(1e-6, seeker.gimbalAzimuthLimitRad[i]);
                        const double holdEl = 0.8 * std::max(1e-6, seeker.gimbalElevationLimitRad[i]);
                        const double targetOff = std::sqrt(azAbs * azAbs + elAbs * elAbs);
                        const double coneRad = std::sqrt(holdAz * holdAz + holdEl * holdEl);
                        if (coneRad > 1e-6) {
                            wSeeker *= std::clamp(1.0 - targetOff / coneRad, 0.0, 1.0);
                        }
                    }
                } else {
                    // No midcourse fallback: fly the seeker source whole.
                    wSeeker = 1.0;
                }

                if (seek.valid) {
                    if (!out.valid) {
                        out = seek;
                    } else {
                        out.ax = (1.0 - wSeeker) * out.ax + wSeeker * seek.ax;
                        out.ay = (1.0 - wSeeker) * out.ay + wSeeker * seek.ay;
                        out.az = (1.0 - wSeeker) * out.az + wSeeker * seek.az;
                        out.valid = true;
                        if (wSeeker >= 0.5) out.ffUsed = seek.ffUsed;
                    }
                    out.tgoSec = seek.tgoSec;  // measured terminal tgo
                }
                // Diagnostics merge regardless of which source produced the
                // demand: a receding or non-finite source must still be
                // visible even when the other source commands.
                out.nonClosing = out.nonClosing || seek.nonClosing;
                out.lawInvalid = out.lawInvalid || seek.lawInvalid;
                // A seeker solution that is unusable keeps the midcourse demand
                // (never zero a valid course).
                // NOTE: the phase label deliberately stays Terminal/Acquisition
                // at the cone edge so a re-centre is not treated as a fresh
                // lock, and a later genuine unlock still runs the lock-loss
                // counter + retention window below.
                law = out.ffUsed ? GuidanceLaw::Apn : GuidanceLaw::Tpn;

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
                    law = GuidanceLaw::Tpn;
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
                LawResult mc = computeAimPn(i, nav, &tracks, guidance, false);
                // Midcourse energy shaping (opt-in loft) applies here too: this
                // is the no-seeker / not-yet-locked path, and it must fly the
                // same lofted profile as the locked midcourse.
                applyMidcourseLoft(i, nav, guidance, environment, mc);
                // Label from what the law actually consumed: the feed-forward
                // may come from the persistent track, not just the command.
                law = (mc.ffUsed ||
                       (guidance.apnFeedforwardEnabled[i] &&
                        guidance.targetAccelAvailable[i]))
                    ? GuidanceLaw::Apn : GuidanceLaw::Tpn;
                applyDemand(i, mc, guidance, dt, authorityScale);
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
                applyDemand(i, computeTrajectory(i, nav, &tracks, guidance), guidance, dt, authorityScale);
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
