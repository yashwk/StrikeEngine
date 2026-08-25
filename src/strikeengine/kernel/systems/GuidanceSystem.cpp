#include <strikeengine/kernel/systems/GuidanceSystem.hpp>
#include <cmath>
#include <glm/glm.hpp>
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
                updateProNav(i, nav, guidance);
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
        // Augmented Proportional Navigation using Seeker LOS rates
        // In the real world, LOS rate is estimated by tracking loops. We'll use a simplified mapping from seeker angles.
        // The seeker gives targetAzimuth and targetElevation relative to the missile's body.
        
        // a_cmd = N * V_c * d(lambda)/dt
        const double N = 3.5;
        double vc = std::abs(seeker.targetRangeRate[id]); // positive closing velocity
        if (vc < 1.0) vc = 1.0;

        // Since the seeker gives us Az/El, the rate of change of Az/El *is* the LOS rate in the body frame.
        // MVP: We approximate the required body-frame acceleration and rotate it back to world-frame
        // for the Autopilot (which expects world-frame commands).
        
        // To properly implement APN, we would need the actual derivative of the angles (dAz/dt, dEl/dt).
        // For MVP, we will use a pseudo-LOS rate proportional to the angle itself (steering to zero).
        // This acts like a pursuit guidance on the body frame angles.
        double dAz_dt = seeker.targetAzimuth[id] * 5.0; // Pseudo-rate
        double dEl_dt = seeker.targetElevation[id] * 5.0;

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

    void GuidanceSystem::updateProNav(
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

        double omega_x = (ry * rvz - rz * rvy) / r_mag_sq;
        double omega_y = (rz * rvx - rx * rvz) / r_mag_sq;
        double omega_z = (rx * rvy - ry * rvx) / r_mag_sq;

        double vc = -(rx*rvx + ry*rvy + rz*rvz) / std::sqrt(r_mag_sq);
        const double N = 3.0;

        double v_mag = std::sqrt(nav.estVx[id]*nav.estVx[id] + nav.estVy[id]*nav.estVy[id] + nav.estVz[id]*nav.estVz[id]);
        if (v_mag < 1.0) v_mag = 1.0;

        double vx_u = nav.estVx[id] / v_mag;
        double vy_u = nav.estVy[id] / v_mag;
        double vz_u = nav.estVz[id] / v_mag;

        guidance.commandedAccelX[id] = N * vc * (omega_y * vz_u - omega_z * vy_u);
        guidance.commandedAccelY[id] = N * vc * (omega_z * vx_u - omega_x * vz_u);
        guidance.commandedAccelZ[id] = N * vc * (omega_x * vy_u - omega_y * vx_u);
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
