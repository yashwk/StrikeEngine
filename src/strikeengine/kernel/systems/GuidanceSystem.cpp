#include <strikeengine/kernel/systems/GuidanceSystem.hpp>
#include <strikeengine/models/guidance/GuidanceModels.hpp>
#include <cmath>
#include <glm/glm.hpp>
#include <algorithm>
#include <glm/gtc/quaternion.hpp>

namespace StrikeEngine::Kernel {

    namespace {

        // Result of one guidance-law evaluation; all outputs are finite.
        struct LawResult {
            double ax = 0.0, ay = 0.0, az = 0.0;
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

        // Publish a law result as the entity's guidance demand (raw + clamped)
        // with diagnostics; zero demand marks invalid results.
        void applyDemand(std::size_t id, const LawResult& r, GuidanceBlock& g)
        {
            g.rawAccelX[id] = r.ax;
            g.rawAccelY[id] = r.ay;
            g.rawAccelZ[id] = r.az;
            g.commandedAccelX[id] = r.valid ? r.ax : 0.0;
            g.commandedAccelY[id] = r.valid ? r.ay : 0.0;
            g.commandedAccelZ[id] = r.valid ? r.az : 0.0;
            g.lawInvalid[id] = r.lawInvalid;
            g.nonClosing[id] = r.nonClosing;
            g.tgoSec[id] = r.tgoSec;
            clampCommandMagnitude(id, g);
        }

        void zeroDemand(std::size_t id, GuidanceBlock& g)
        {
            g.rawAccelX[id] = g.rawAccelY[id] = g.rawAccelZ[id] = 0.0;
            g.commandedAccelX[id] = g.commandedAccelY[id] = g.commandedAccelZ[id] = 0.0;
            g.limitedByMaxAccel[id] = false;
            g.lawInvalid[id] = false;
            g.nonClosing[id] = false;
            g.tgoSec[id] = 0.0;
        }

        // Midcourse PN / APN on the commanded target track (world frame).
        // Aim source: a measurement-anchored persistent target track (W39)
        // wins over the raw external command state; see update().
        LawResult computeMidcourse(
            std::size_t id, const NavigationBlock& nav,
            const TrackBlock* tracks, GuidanceBlock& guidance)
        {
            LawResult out;
            const double N = guidance.navigationConstant[id];
            if (!std::isfinite(N) || N <= 0.0) {
                out.lawInvalid = true;
                out.valid = false;
                return out;
            }

            // Select the aim state: persistent track (measurement-anchored)
            // or the external command state (legacy).
            double tx, ty, tz, tvx, tvy, tvz;
            bool trackAim = false;
            if (tracks && id < tracks->size && tracks->active(id) &&
                tracks->updateCount[id] > 0)
            {
                trackAim = true;
                tx = tracks->posX[id]; ty = tracks->posY[id]; tz = tracks->posZ[id];
                tvx = tracks->velX[id]; tvy = tracks->velY[id]; tvz = tracks->velZ[id];
            } else {
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

            // Feed-forward target acceleration: from the track when
            // measurement-anchored and available, else from the command.
            bool ffAvailable = false;
            double atx = 0.0, aty = 0.0, atz = 0.0;
            if (trackAim && tracks->accelAvailable[id]) {
                ffAvailable = isFinite3(tracks->accelX[id], tracks->accelY[id],
                                        tracks->accelZ[id]);
                atx = tracks->accelX[id]; aty = tracks->accelY[id]; atz = tracks->accelZ[id];
            } else if (!trackAim && guidance.targetAccelAvailable[id]) {
                ffAvailable = isFinite3(guidance.targetAccelX[id],
                                        guidance.targetAccelY[id],
                                        guidance.targetAccelZ[id]);
                atx = guidance.targetAccelX[id]; aty = guidance.targetAccelY[id];
                atz = guidance.targetAccelZ[id];
            }
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
            out.tgoSec = range / std::max(sol.closingSpeed, 1e-6);
            return out;
        }

        // Seeker-rate APN: N * Vc * LOS rate, body-frame mapping per the frame
        // contract (azimuth -> +body-Y, elevation -> -body-Z), rotated to the
        // world frame with the navigation attitude.
        LawResult computeSeekerAPN(
            std::size_t id, const NavigationBlock& nav,
            const SeekerBlock& seeker, GuidanceBlock& guidance)
        {
            LawResult out;
            const double N = guidance.navigationConstant[id];
            const double range = seeker.targetRange[id];
            const double rangeRate = seeker.targetRangeRate[id];
            if (!std::isfinite(N) || N <= 0.0 ||
                !std::isfinite(range) || !std::isfinite(rangeRate) ||
                !std::isfinite(seeker.targetAzimuthRate[id]) ||
                !std::isfinite(seeker.targetElevationRate[id]))
            {
                // Validate inputs BEFORE publishing diagnostics: a non-finite
                // range or rate must not leak a NaN tgo.
                out.lawInvalid = true;
                out.valid = false;
                return out;
            }
            const double vc = std::max(std::abs(rangeRate), 1.0);
            const double dAz = seeker.targetAzimuthRate[id];
            const double dEl = seeker.targetElevationRate[id];
            out.nonClosing = rangeRate > 0.0;
            out.tgoSec = range / std::max(std::abs(rangeRate), 1.0);
            const double ayBody = N * vc * dAz;
            const double azBody = -N * vc * dEl;
            glm::dquat estQ(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id]);
            const glm::dvec3 aWorld = estQ * glm::dvec3(0.0, ayBody, azBody);
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
        double dt)
    {
        for (std::size_t i = 0; i < nav.size; ++i) {
            if (!status.isAlive[i]) continue;

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

            // Track bookkeeping. trackAgeSec counts time since the last valid
            // seeker track (diagnostic + retention window).
            if (locked) {
                guidance.trackAgeSec[i] = 0.0;
                guidance.trackId[i] = static_cast<std::int64_t>(seeker.lockedTargetId[i]);
            } else if (seekerPresent) {
                guidance.trackAgeSec[i] += dt;
            }

            // --- Terminal homing path (seeker lock; overrides the configured
            // --- mode, matching the legacy lock-override contract) --------
            if (locked) {
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

                law = GuidanceLaw::SeekerRateAPN;
                LawResult apn = computeSeekerAPN(i, nav, seeker, guidance);
                LawResult out = apn;
                if (phase == GuidancePhase::Acquisition) {
                    // Blend midcourse PN -> terminal APN (deterministic ramp).
                    const LawResult pn = computeMidcourse(i, nav, &tracks, guidance);
                    const double w = guidance.handoffWeight[i];
                    out.ax = (1.0 - w) * (pn.valid ? pn.ax : 0.0) + w * apn.ax;
                    out.ay = (1.0 - w) * (pn.valid ? pn.ay : 0.0) + w * apn.ay;
                    out.az = (1.0 - w) * (pn.valid ? pn.az : 0.0) + w * apn.az;
                    out.valid = apn.valid;
                    out.lawInvalid = apn.lawInvalid && pn.lawInvalid;
                    out.nonClosing = pn.nonClosing || apn.nonClosing;
                }
                applyDemand(i, out, guidance);
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
                // Terminal track just lost: count it once, then either retain
                // (bounded, by configuration) or fall to LostTrack.
                if (guidance.trackAgeSec[i] <= dt * 1.5) {
                    ++guidance.lockLossCount[i];
                }
                const double retain = guidance.lockLossRetentionSec[i];
                if (retain > 0.0 && guidance.trackAgeSec[i] <= retain) {
                    // Retention window: apply the bounded predicted terminal
                    // command (last valid seeker-APN demand, already clamped by
                    // maxAccel) while the track identity/age is retained.
                    phase = GuidancePhase::Terminal;
                    law = GuidanceLaw::SeekerRateAPN;
                    LawResult retained;
                    retained.ax = guidance.retainedAccelX[i];
                    retained.ay = guidance.retainedAccelY[i];
                    retained.az = guidance.retainedAccelZ[i];
                    retained.valid = isFinite3(retained.ax, retained.ay, retained.az);
                    retained.tgoSec = guidance.tgoSec[i];  // last valid tgo
                    applyDemand(i, retained, guidance);
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
                applyDemand(i, computeMidcourse(i, nav, &tracks, guidance), guidance);
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
                    applyDemand(i, wp, guidance);
                    continue;
                }
                double rMag = std::sqrt(rx * rx + ry * ry + rz * rz);
                if (rMag < 1.0) rMag = 1.0;
                wp.ax = k * (rx / rMag);
                wp.ay = k * (ry / rMag);
                wp.az = k * (rz / rMag);
                wp.tgoSec = rMag / 300.0;  // diagnostic only (nominal 300 m/s)
                applyDemand(i, wp, guidance);
            } else {
                zeroDemand(i, guidance);
                phase = GuidancePhase::None;
                law = GuidanceLaw::None;
            }
        }
    }

} // namespace StrikeEngine::Kernel
