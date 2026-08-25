#pragma once
#include <cmath>
#include <algorithm>

namespace StrikeEngine::Models {

    struct AeroWrench {
        double force_x, force_y, force_z;   // BODY frame forces (N)
        double torque_x, torque_y, torque_z; // BODY frame moments (N*m)
    };

    /**
     * @brief Per-vehicle aero parameters (W1: per-entity config).
     */
    struct AeroParams {
        double referenceArea   = 0.1;  // m^2
        double referenceLength = 1.0;  // m
        double cd        = 0.3;        // drag coefficient
        double clAlpha   = 0.0;        // lift slope per rad of AoA
        double clFin     = 0.0;        // fin lift coefficient per rad of deflection
        double clMax     = 2.0;        // max |CL| (stall / control surface limit)
    };

    /**
     * @brief Aerodynamic force/moment model (BODY frame).
     *
     * Conventions (aerospace NED body axes: X forward, Y right, Z down):
     *   - a positive pitch deflection (trailing edge down) produces a
     *     nose-UP moment (+torque_y) and a lift force in -Z
     *   - a positive yaw deflection produces a nose-RIGHT moment (+torque_z)
     *   - positive AoA (velocity from below, w > 0 with Z down) produces
     *     lift in -Z and a restoring (stabilizing) -torque_y
     *   - drag opposes the body-frame velocity vector
     * All outputs are in the BODY frame; the backend rotates forces to the
     * world frame and uses torques directly in the body-frame Euler
     * equations (W2 spine).
     */
    class AeroModel {
    public:
        virtual ~AeroModel() = default;

        virtual AeroWrench computeWrench(
            double u, double v, double w,       // BODY-frame velocity (m/s)
            double wx, double wy, double wz,    // BODY angular rate (rad/s)
            double finPitch, double finYaw, double finRoll,  // achieved deflections (rad)
            double density, double speedOfSound,
            const AeroParams& params) const = 0;
    };

    class BasicAeroModel : public AeroModel {
    public:
        BasicAeroModel() = default;

        AeroWrench computeWrench(
            double u, double v, double w,
            double wx, double wy, double wz,
            double finPitch, double finYaw, double finRoll,
            double density, double speedOfSound,
            const AeroParams& p) const override
        {
            const double speedSq = u * u + v * v + w * w;
            if (speedSq < 1e-6) {
                return {0, 0, 0, 0, 0, 0};
            }

            const double V = std::sqrt(speedSq);
            const double q = 0.5 * density * speedSq;   // dynamic pressure
            const double S = p.referenceArea;
            const double l = p.referenceLength;

            // Angle of attack and sideslip (body frame)
            const double alpha = std::atan2(w, u);      // +w (Z down) => nose up
            const double beta  = std::atan2(v, u);      // +v => airflow from right

            // --- Forces (body frame) ---
            // Drag opposes velocity
            const double dragMag = q * S * p.cd;
            double fx = -dragMag * (u / V);
            double fy = -dragMag * (v / V);
            double fz = -dragMag * (w / V);

            // Lift (pitch plane): positive alpha/deflection => force -Z (up),
            // saturated at CL_max (stall / control limit). A linear lift
            // slope unbounded is the classic way to let a simulation run away
            // to 70+ deg AoA: at |CL| = CL_max the lifting surfaces are
            // stalled and produce no more force.
            double cl = p.clAlpha * alpha + p.clFin * finPitch;
            cl = std::clamp(cl, -p.clMax, p.clMax);
            fz -= q * S * cl;

            // Side force (yaw plane): positive beta/deflection => force -Y
            double cy = p.clAlpha * beta + p.clFin * finYaw;
            cy = std::clamp(cy, -p.clMax, p.clMax);
            fy -= q * S * cy;

            // --- Moments (body frame) ---
            // Fin control authority. Static stability is strong (CM_alpha
            // ~ -6 per rad, a realistic static margin): the fins can trim
            // only ~7 deg of AoA at full deflection (0.43*1.8/6), which keeps
            // the missile out of the high-alpha regime the linear model
            // cannot represent.
            constexpr double CM_delta = 1.8;   // pitch/yaw moment per rad
            constexpr double Cl_delta = 0.5;   // roll moment per rad
            double tx = q * S * l * (Cl_delta * finRoll);
            double ty = q * S * l * (CM_delta * finPitch);
            double tz = q * S * l * (CM_delta * finYaw);

            // Static stability (restoring): CM_alpha, CN_beta < 0
            constexpr double CM_alpha = -6.0;
            constexpr double CN_beta  = -6.0;
            ty += q * S * l * CM_alpha * alpha;
            tz += q * S * l * CN_beta  * beta;

            // Rotational damping (dimensionless rate q_bar*l/V)
            const double lOverV = l / V;
            constexpr double Cq = 10.0;    // pitch/yaw damping
            constexpr double Clp = 3.0;    // roll damping
            ty -= q * S * l * Cq  * lOverV * wy;
            tz -= q * S * l * Cq  * lOverV * wz;
            tx -= q * S * l * Clp * lOverV * wx;

            return {fx, fy, fz, tx, ty, tz};
        }
    };

} // namespace StrikeEngine::Models
