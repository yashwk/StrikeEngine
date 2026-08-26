#include <strikeengine/kernel/systems/GuidanceSystem.hpp>
#include <cmath>
#include <glm/glm.hpp>
#include <algorithm>
#include <glm/gtc/quaternion.hpp>

namespace StrikeEngine::Kernel {

    // Clamp the commanded acceleration magnitude to the per-entity guidance
    // limit (GuidanceBlock::maxAccel; 0 = unlimited). Real guidance laws
    // shape commanded g before the autopilot sees it: an unbounded
    // ProNav/Waypoint demand over-drives the fins into permanent saturation.
    static void clampCommandMagnitude(
        std::size_t id, GuidanceBlock& guidance)
    {
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
        }
    }

    void GuidanceSystem::update(
        const EntityStatusBlock& status,
        const NavigationBlock& nav,
        const SeekerBlock& seeker,
        GuidanceBlock& guidance,
        ControlBlock& control,
        double dt) 
    {
        for (std::size_t i = 0; i < nav.size; ++i) {
            if (!status.isAlive[i]) continue;

            // Terminal Homing override!
            if (seeker.type[i] != SeekerType::None && seeker.isLocked[i]) {
                updateSeekerAPN(i, nav, seeker, guidance);
                continue;
            }

            auto mode = guidance.mode[i];
            if (mode == GuidanceMode::None) {
                // Ballistic
                guidance.commandedAccelX[i] = 0.0;
                guidance.commandedAccelY[i] = 0.0;
                guidance.commandedAccelZ[i] = 0.0;
                continue;
            }

            if (mode == GuidanceMode::ProportionalNavigation) {
                updatePredictiveIntercept(i, nav, guidance);
            } else if (mode == GuidanceMode::Waypoint) {
                updateWaypoint(i, nav, guidance);
            }
            clampCommandMagnitude(i, guidance);
        }
    }

    void GuidanceSystem::updateSeekerAPN(
        std::size_t id,
        const NavigationBlock& nav,
        const SeekerBlock& seeker,
        GuidanceBlock& guidance)
    {
        // Augmented Proportional Navigation using filtered seeker LOS rates.
        // The seeker angles and rates are relative to the missile body.
        
        // a_cmd = N * V_c * d(lambda)/dt
        const double N = 3.5;
        double vc = std::abs(seeker.targetRangeRate[id]); // positive closing velocity
        if (vc < 1.0) vc = 1.0;

        // SeekerSystem supplies filtered finite-difference LOS rates in the
        // seeker body frame. These are delayed/noisy track derivatives rather
        // than the old angle-times-gain pursuit approximation.
        const double dAz_dt = seeker.targetAzimuthRate[id];
        const double dEl_dt = seeker.targetElevationRate[id];

        // Commanded acceleration in Body Frame
        double a_cmd_y_body = N * vc * dEl_dt; // Pitch
        double a_cmd_z_body = N * vc * dAz_dt; // Yaw
        double a_cmd_x_body = 0.0;             // Roll

        // Rotate body frame commands to world frame
        glm::dquat estQ(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id]);
        glm::dvec3 a_cmd_world = estQ * glm::dvec3(a_cmd_x_body, a_cmd_y_body, a_cmd_z_body);

        guidance.commandedAccelX[id] = a_cmd_world.x;
        guidance.commandedAccelY[id] = a_cmd_world.y;
        guidance.commandedAccelZ[id] = a_cmd_world.z;
    }

    void GuidanceSystem::updatePredictiveIntercept(
        std::size_t id,
        const NavigationBlock& nav,
        GuidanceBlock& guidance)
    {
        double rx = guidance.targetX[id] - nav.estPx[id];
        double ry = guidance.targetY[id] - nav.estPy[id];
        double rz = guidance.targetZ[id] - nav.estPz[id];

        double rvx = guidance.targetVx[id] - nav.estVx[id];
        double rvy = guidance.targetVy[id] - nav.estVy[id];
        double rvz = guidance.targetVz[id] - nav.estVz[id];

        double r_mag_sq = rx*rx + ry*ry + rz*rz;
        if (r_mag_sq < 1.0) {
            guidance.commandedAccelX[id] = 0.0;
            guidance.commandedAccelY[id] = 0.0;
            guidance.commandedAccelZ[id] = 0.0;
            return;
        }

        // Predictive constant-acceleration intercept correction. Pure LOS-rate PN is
        // appropriate for a vehicle that already has a well-shaped flight
        // path, but this MVP also models gravity explicitly and has no
        // trajectory manager. Use a bounded time-to-go estimate so vertical
        // gravity error is corrected early instead of producing a saturated
        // last-second command near the ground.
        const double v_mag = std::max(
            1.0,
            std::sqrt(nav.estVx[id] * nav.estVx[id] +
                      nav.estVy[id] * nav.estVy[id] +
                      nav.estVz[id] * nav.estVz[id]));
        const double tgo = std::clamp(std::sqrt(r_mag_sq) / v_mag, 1.0, 30.0);
        const double tgoSq = tgo * tgo;

        guidance.commandedAccelX[id] = 2.0 * (rx + rvx * tgo) / tgoSq;
        guidance.commandedAccelY[id] = 2.0 * (ry + rvy * tgo) / tgoSq;
        guidance.commandedAccelZ[id] = 2.0 * (rz + rvz * tgo) / tgoSq;

        // The current airframe has no lateral trajectory manager and the
        // cross-track state is sensor-estimated. Limit lateral demand to keep
        // a small measurement error from becoming a large side excursion;
        // the per-entity maxAccel limit still bounds the complete command.
        constexpr double maxLateralAccel = 5.0; // m/s^2
        guidance.commandedAccelY[id] = (std::abs(ry) < 50.0)
            ? 0.0
            : std::clamp(guidance.commandedAccelY[id], -maxLateralAccel, maxLateralAccel);
    }

    void GuidanceSystem::updateWaypoint(
        std::size_t id,
        const NavigationBlock& nav,
        GuidanceBlock& guidance)
    {
        double rx = guidance.targetX[id] - nav.estPx[id];
        double ry = guidance.targetY[id] - nav.estPy[id];
        double rz = guidance.targetZ[id] - nav.estPz[id];
        
        double r_mag = std::sqrt(rx*rx + ry*ry + rz*rz);
        if (r_mag < 1.0) r_mag = 1.0;

        // Acceleration demand up to the maneuver limit (2 g here); the
        // autopilot/servo limits bound the physical response.
        const double k = 20.0;
        guidance.commandedAccelX[id] = k * (rx / r_mag);
        guidance.commandedAccelY[id] = k * (ry / r_mag);
        guidance.commandedAccelZ[id] = k * (rz / r_mag);
    }

} // namespace StrikeEngine::Kernel
