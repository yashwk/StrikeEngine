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

            // Side force (yaw plane): positive yaw-fin deflection produces a
            // nose-right force (+Y), matching its positive nose-right moment.
            // Sideslip force is intentionally deferred until the vehicle has
            // a validated lateral stability model; an incorrectly signed
            // beta term destabilizes the yaw loop.
            constexpr double cyBody = 0.0;
            const double cyFin  = std::clamp(p.clFin * finYaw, -p.clMax, p.clMax);
            fy -= q * S * cyBody;
            fy += q * S * cyFin;

            // --- Moments (body frame) ---
            // Fin control authority and the validated pitch restoring term.
            constexpr double CM_delta = 0.50;  // pitch/yaw moment per rad
            constexpr double Cl_delta = 0.15;  // roll moment per rad
            // Limit control authority as dynamic pressure rises. The linear
            // coefficient model is only valid around modest AoA; allowing its
            // moment to grow without bound at boost speed spins the vehicle
            // faster than this MVP's guidance loop can observe and correct.
            const double qS = q * S;
            const double maxControlMoment = 600.0 * std::clamp(
                6000.0 / std::max(qS, 6000.0), 0.10, 1.0);
            double tx = std::clamp(qS * l * (Cl_delta * finRoll),
                                   -maxControlMoment, maxControlMoment);
            double ty = std::clamp(qS * l * (CM_delta * finPitch),
                                   -maxControlMoment, maxControlMoment);
            double tz = std::clamp(qS * l * (CM_delta * finYaw),
                                   -maxControlMoment, maxControlMoment);

            // Static pitch stability is retained. Lateral beta stability is
            // disabled until its force/moment signs are covered by validation.
            constexpr double CM_alpha = -0.5;
            constexpr double CN_beta  = 0.0; // avoid unmodeled yaw/side-force coupling in MVP
            ty += q * S * l * CM_alpha * alpha;
            tz += q * S * l * CN_beta  * beta;

            // Rotational damping (dimensionless rate q_bar*l/V)
            const double lOverV = l / V;
            constexpr double Cq = 20.0;    // pitch/yaw damping
            constexpr double Clp = 6.0;    // roll damping
            ty -= q * S * l * Cq  * lOverV * wy;
            tz -= q * S * l * Cq  * lOverV * wz;
            tx -= q * S * l * Clp * lOverV * wx;

            // Bound the complete aerodynamic moment, not only the commanded
            // fin contribution. At large AoA the linear static-stability and
            // rate-damping terms can otherwise create thousands of N*m and
            // drive the simplified rigid body into an unrecoverable spin.
            const double maxAeroMoment = maxControlMoment; // model validity limit
            tx = std::clamp(tx, -maxAeroMoment, maxAeroMoment);
            ty = std::clamp(ty, -maxAeroMoment, maxAeroMoment);
            tz = std::clamp(tz, -maxAeroMoment, maxAeroMoment);

            return {fx, fy, fz, tx, ty, tz};
        }
    };

} // namespace StrikeEngine::Models
