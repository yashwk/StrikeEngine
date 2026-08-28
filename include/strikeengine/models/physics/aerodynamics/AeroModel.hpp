#pragma once
#include <cmath>
#include <algorithm>
#include <memory>
#include <strikeengine/models/physics/aerodynamics/CoefficientTable.hpp>
#include <strikeengine/models/physics/aerodynamics/FinsModel.hpp>

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

        // Optional data-driven cd(M,a)/cl(M,a) tables. When present they are
        // authoritative for cd/cl; nullptr keeps the constant-coefficient path
        // (byte-identical to the legacy behavior).
        std::shared_ptr<const Models::AeroTables> tables;

        // Optional geometric fins (trapezoidal/elliptical/free-form). When
        // non-null they replace the abstract fin terms (clFin in lift/side
        // force, CM_delta/Cl_delta moments) with geometry-derived,
        // Mach-dependent terms; nullptr keeps the byte-identical legacy path.
        std::shared_ptr<const Models::FinsGeometry> fins;
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
            const double mach = (speedOfSound > 1e-6) ? V / speedOfSound : 0.0;

            // Angle of attack and sideslip (body frame)
            const double alpha = std::atan2(w, u);      // +w (Z down) => nose up
            const double beta  = std::atan2(v, u);      // +v => airflow from right

            // Defensive gate: engage the table path only for structurally
            // valid grids. Programmatically-built configs bypass load-time
            // validation, so an invalid table must not reach the interpolator
            // (a degenerate 1xN grid would silently produce zero drag/lift);
            // it falls back to the constant-coefficient path instead.
            const AeroTables* tables =
                (p.tables && !p.tables->empty() && p.tables->isValid())
                    ? p.tables.get() : nullptr;

            // --- Forces (body frame) ---
            // Drag opposes velocity. With tables the cd(M,a) grid is
            // authoritative; otherwise the flat p.cd coefficient is used.
            double cd;
            if (tables) {
                cd = interpolateCoefficient(mach, alpha,
                    tables->machBreakpoints, tables->aoaBreakpointsRad,
                    tables->cdTable);
            } else {
                cd = p.cd;
            }
            const double dragMag = q * S * cd;
            double fx = -dragMag * (u / V);
            double fy = -dragMag * (v / V);
            double fz = -dragMag * (w / V);

            // Lift (pitch plane): positive alpha => force -Z (up), saturated at
            // CL_max (stall / control limit). With geometric fins the fin lift
            // slope clAlpha(mach) acts on the fin's local AoA (alpha +
            // finPitch); without them the flat clFin term is used.
            double cl;
            if (p.fins) {
                const double clFin = p.fins->clAlpha(mach);
                if (tables) {
                    cl = interpolateCoefficient(mach, alpha,
                             tables->machBreakpoints, tables->aoaBreakpointsRad,
                             tables->clTable)
                         + clFin * (alpha + finPitch);
                } else {
                    cl = p.clAlpha * alpha + clFin * (alpha + finPitch);
                }
            } else if (tables) {
                cl = interpolateCoefficient(mach, alpha,
                         tables->machBreakpoints, tables->aoaBreakpointsRad,
                         tables->clTable)
                     + p.clFin * finPitch;
            } else {
                cl = p.clAlpha * alpha + p.clFin * finPitch;
            }
            cl = std::clamp(cl, -p.clMax, p.clMax);
            fz -= q * S * cl;

            // Side force (yaw plane). With geometric fins the sideslip/beta
            // term is now modeled (restoring); without them it stays deferred
            // (cyBody = 0) so the flat path is byte-identical.
            if (p.fins) {
                const double clFin = p.fins->clAlpha(mach);
                const double cy = clFin * (beta + finYaw);
                fy -= q * S * std::clamp(cy, -p.clMax, p.clMax);
            } else {
                constexpr double cyBody = 0.0;
                const double cyFin  = std::clamp(p.clFin * finYaw, -p.clMax, p.clMax);
                fy -= q * S * cyBody;
                fy += q * S * cyFin;
            }

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
            constexpr double Cq = 20.0;    // body pitch/yaw damping

            double tx, ty, tz;
            if (p.fins) {
                const double clFin = p.fins->clAlpha(mach);
                const double xcp = p.fins->cpLeverArmM;
                // Fin stability + control moments about the fin CP (lever arm
                // xcp, negative for tail fins => restoring). Roll forcing from
                // cant and roll damping replace the abstract Cl_delta/Clp fin
                // terms; body Cq/Clp damping is still applied below.
                tx = std::clamp(qS * l * p.fins->rollForcingPerRad(mach)
                                    * (p.fins->cantRad + finRoll)
                                - qS * l * l * 0.5 * p.fins->rollDampingCoeff(mach) * wx,
                                -maxControlMoment, maxControlMoment);
                ty = std::clamp(qS * xcp * clFin * (alpha + finPitch),
                                -maxControlMoment, maxControlMoment);
                tz = std::clamp(-qS * xcp * clFin * (beta + finYaw),
                                -maxControlMoment, maxControlMoment);

                // Static stability is now supplied by the fins (via xcp); the
                // bare body term is dropped so it is not double-counted.
                ty -= q * S * l * Cq  * (l / V) * wy;
                tz -= q * S * l * Cq  * (l / V) * wz;
            } else {
                tx = std::clamp(qS * l * (Cl_delta * finRoll),
                                -maxControlMoment, maxControlMoment);
                ty = std::clamp(qS * l * (CM_delta * finPitch),
                                -maxControlMoment, maxControlMoment);
                tz = std::clamp(qS * l * (CM_delta * finYaw),
                                -maxControlMoment, maxControlMoment);

                // Static pitch stability is retained. Lateral beta stability is
                // disabled until its force/moment signs are covered by validation.
                constexpr double CM_alpha = -0.5;
                constexpr double CN_beta  = 0.0; // avoid unmodeled yaw/side-force coupling in MVP
                ty += q * S * l * CM_alpha * alpha;
                tz += q * S * l * CN_beta  * beta;

                // Rotational damping (dimensionless rate q_bar*l/V)
                const double lOverV = l / V;
                constexpr double Clp = 6.0;    // roll damping
                ty -= q * S * l * Cq  * lOverV * wy;
                tz -= q * S * l * Cq  * lOverV * wz;
                tx -= q * S * l * Clp * lOverV * wx;
            }

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
