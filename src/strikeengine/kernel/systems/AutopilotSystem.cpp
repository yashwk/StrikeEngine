#include <strikeengine/kernel/systems/AutopilotSystem.hpp>
#include <cmath>
#include <algorithm>

namespace StrikeEngine::Kernel {

    // Helper to rotate world vector to body vector
    static void worldToBody(double qw, double qx, double qy, double qz,
                            double wx, double wy, double wz,
                            double& bx, double& by, double& bz) {
        // Inverse of unit quaternion (qw, qx, qy, qz) is (qw, -qx, -qy, -qz)
        double ix =  qw * wx - (-qy) * wz + (-qz) * wy;
        double iy =  qw * wy - (-qz) * wx + (-qx) * wz;
        double iz =  qw * wz - (-qx) * wy + (-qy) * wx;
        double iw = -(-qx) * wx - (-qy) * wy - (-qz) * wz;

        bx = ix * qw + iw * (-qx) + iy * (-qz) - iz * (-qy);
        by = iy * qw + iw * (-qy) + iz * (-qx) - ix * (-qz);
        bz = iz * qw + iw * (-qz) + ix * (-qy) - iy * (-qx);
    }

    void AutopilotSystem::update(
        const EntityStatusBlock& status,
        const NavigationBlock& nav,
        const GuidanceBlock& guidance,
        ControlBlock& control,
        double dt)
    {
        // Resize control block if needed (usually handled in SimulationKernel, but safe to check)
        if (control.pitchCommand.size() < nav.size) {
            control.pitchCommand.resize(nav.size, 0.0);
            control.yawCommand.resize(nav.size, 0.0);
            control.rollCommand.resize(nav.size, 0.0);
            control.thrustCommand.resize(nav.size, 0.0);
        }

        for (std::size_t i = 0; i < nav.size; ++i) {
            if (!status.isAlive[i]) continue;

            if (guidance.mode[i] == GuidanceMode::None) {
                control.pitchCommand[i] = 0.0;
                control.yawCommand[i] = 0.0;
                control.rollCommand[i] = 0.0;
                continue;
            }

            updateFlightController(i, nav, guidance, control);
        }
    }

    void AutopilotSystem::updateFlightController(
        std::size_t id,
        const NavigationBlock& nav,
        const GuidanceBlock& guidance,
        ControlBlock& control)
    {
        // 1. Read commanded acceleration in World frame
        double ax_w = guidance.commandedAccelX[id];
        double ay_w = guidance.commandedAccelY[id];
        double az_w = guidance.commandedAccelZ[id];

        // 2. Rotate commanded acceleration to Body frame
        double ax_b, ay_b, az_b;
        worldToBody(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                    ax_w, ay_w, az_w, ax_b, ay_b, az_b);

        // Body Z is usually Down or Up. Let's assume standard aerospace: X forward, Y right, Z down.
        // Commanded acceleration in body Y means we need to Yaw.
        // Commanded acceleration in body Z means we need to Pitch.
        // Note: If we want positive Z acceleration (downward), we pitch down (positive Pitch rate).
        // It depends on the Aerodynamics model lift convention.
        
        // Simple P-D Controller for Fin Deflection
        // commandedAccel translates to a desired Angle of Attack, which translates to a required fin deflection.
        // For MVP, we map commandedAccel directly to fin deflection with a proportional gain.
        double k_p = -0.05; // P gain mapping Accel Error to Fin Deflection (rad)
        double k_d = 0.1;   // D gain for rate damping (rad*s)

        // Commanded fin pitch to achieve az_b (with rate damping from estWy)
        double pitchDeflection = k_p * az_b - k_d * nav.estWy[id];
        
        // Commanded fin yaw to achieve ay_b (with rate damping from estWz)
        double yawDeflection = k_p * ay_b - k_d * nav.estWz[id];

        // Roll stabilization: simple P-D to keep wings level (estQ roll -> 0, estWx -> 0)
        // Extract roll from quaternion:
        double roll = std::atan2(2.0 * (nav.estQw[id] * nav.estQx[id] + nav.estQy[id] * nav.estQz[id]),
                                 1.0 - 2.0 * (nav.estQx[id] * nav.estQx[id] + nav.estQy[id] * nav.estQy[id]));
        
        double rollDeflection = -0.1 * roll - 0.05 * nav.estWx[id];

        // 3. Clamp fin deflections to physical limits (e.g., +/- 25 degrees = ~0.43 rad)
        const double maxDeflection = 0.43;
        control.pitchCommand[id] = std::clamp(pitchDeflection, -maxDeflection, maxDeflection);
        control.yawCommand[id] = std::clamp(yawDeflection, -maxDeflection, maxDeflection);
        control.rollCommand[id] = std::clamp(rollDeflection, -maxDeflection, maxDeflection);
    }

} // namespace StrikeEngine::Kernel
