#include <strikeengine/kernel/systems/GuidanceSystem.hpp>
#include <strikeengine/models/guidance/GuidanceModels.hpp>
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

            // Communication failure: ignore guidance and seeker handoff;
            // the entity flies ballistic with zero commanded acceleration.
            if (i < status.commsFailed.size() && status.commsFailed[i]) {
                guidance.commandedAccelX[i] = 0.0;
                guidance.commandedAccelY[i] = 0.0;
                guidance.commandedAccelZ[i] = 0.0;
                continue;
            }

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
                updateProportionalNavigation(i, nav, guidance);
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
        const double N = guidance.navigationConstant[id];
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

        clampCommandMagnitude(id, guidance);
    }

    void GuidanceSystem::updateProportionalNavigation(
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

        const Models::Vec3 relativePosition{rx, ry, rz};
        const Models::Vec3 relativeVelocity{rvx, rvy, rvz};
        const Models::GuidanceSolution solution = Models::proportionalNavigation(
            relativePosition, relativeVelocity, guidance.navigationConstant[id]);
        if (!solution.valid) {
            guidance.commandedAccelX[id] = 0.0;
            guidance.commandedAccelY[id] = 0.0;
            guidance.commandedAccelZ[id] = 0.0;
            return;
        }

        guidance.commandedAccelX[id] = solution.acceleration[0];
        guidance.commandedAccelY[id] = solution.acceleration[1];
        guidance.commandedAccelZ[id] = solution.acceleration[2];
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
        const double k = guidance.waypointGain[id];
        guidance.commandedAccelX[id] = k * (rx / r_mag);
        guidance.commandedAccelY[id] = k * (ry / r_mag);
        guidance.commandedAccelZ[id] = k * (rz / r_mag);
    }

} // namespace StrikeEngine::Kernel
