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

        // Optional multiple geometric fin sets (e.g. canards + tails).
        // When non-empty, all fin sets are composited in the force and moment calculations.
        std::vector<std::shared_ptr<const Models::FinsGeometry>> finSets;

        std::vector<std::shared_ptr<const Models::FinsGeometry>> activeFins() const {
            if (!finSets.empty()) {
                return finSets;
            }
            if (fins) {
                return {fins};
            }
            return {};
        }
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

            const bool hasCmTable = tables && tables->hasCmTable();
            const bool hasCyTable = tables && tables->hasCyTable();
            const bool hasCnTable = tables && tables->hasCnTable();
            const bool hasClRollTable = tables && tables->hasClRollTable();

            const auto tableAtAlpha = [&](const std::vector<std::vector<double>>& table) {
                return interpolateCoefficient(mach, alpha,
                    tables->machBreakpoints, tables->aoaBreakpointsRad, table);
            };
            const auto tableAtBeta = [&](const std::vector<std::vector<double>>& table) {
                return interpolateCoefficient(mach, beta,
                    tables->machBreakpoints, tables->betaBreakpointsRad, table);
            };

            const auto fList = p.activeFins();

            // Lift (pitch plane): positive alpha => force -Z (up), saturated at
            // CL_max (stall / control limit). With geometric fins the fin lift
            // slope clAlpha(mach) acts on the fin's local AoA (alpha +
            // finPitch); without them the flat clFin term is used.
            double cl;
            if (!fList.empty()) {
                double finLift = 0.0;
                for (const auto& f : fList) {
                    if (!f) continue;
                    const double clFin = f->clAlpha(mach);
                    const double effPitch = f->steerable ? finPitch : 0.0;
                    finLift += clFin * (alpha + effPitch);
                }
                if (tables) {
                    cl = interpolateCoefficient(mach, alpha,
                             tables->machBreakpoints, tables->aoaBreakpointsRad,
                             tables->clTable)
                         + finLift;
                } else {
                    cl = p.clAlpha * alpha + finLift;
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

            // Side force (yaw plane). Cy is a body +Y force coefficient. An
            // optional table supplies the body contribution; geometric or
            // abstract fin control terms are then added as before. With no
            // table and no fins, the legacy path remains byte-identical.
            if (hasCyTable) {
                fy += q * S * tableAtBeta(tables->cyTable);
            }
            if (!fList.empty()) {
                double cySum = 0.0;
                for (const auto& f : fList) {
                    if (!f) continue;
                    const double clFin = f->clAlpha(mach);
                    const double effYaw = f->steerable ? finYaw : 0.0;
                    cySum += clFin * (beta + effYaw);
                }
                fy -= q * S * std::clamp(cySum, -p.clMax, p.clMax);
            } else {
                constexpr double cyBody = 0.0;
                const double cyFin  = std::clamp(p.clFin * finYaw, -p.clMax, p.clMax);
                fy -= q * S * cyBody;
                fy += q * S * cyFin;
            }

            // Static aerodynamic coefficients. Cm is pitch moment about +Y,
            // Cn is yaw moment about +Z, and Cl is roll moment about +X.
            // The scalar values below are the established fallback. Tables
            // replace only the corresponding static coefficient and do not
            // replace fin-control or rate-damping terms.
            const double cmStatic = hasCmTable
                ? tableAtAlpha(tables->cmTable)
                : -0.5 * alpha;
            const double cnStatic = hasCnTable
                ? tableAtBeta(tables->cnTable)
                : 0.0 * beta;
            const double clRollStatic = hasClRollTable
                ? tableAtBeta(tables->clRollTable)
                : 0.0;

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
            if (!fList.empty()) {
                double totalRollTorque = 0.0;
                double totalPitchTorque = 0.0;
                double totalYawTorque = 0.0;
                for (const auto& f : fList) {
                    if (!f) continue;
                    const double clFin = f->clAlpha(mach);
                    const double xcp = f->cpLeverArmM;
                    const double effPitch = f->steerable ? finPitch : 0.0;
                    const double effYaw = f->steerable ? finYaw : 0.0;
                    const double effRoll = f->steerable ? finRoll : 0.0;

                    totalRollTorque += (qS * l * f->rollForcingPerRad(mach) * (f->cantRad + effRoll)
                                        - qS * l * l * 0.5 * f->rollDampingCoeff(mach) * wx);
                    totalPitchTorque += (qS * clFin * (xcp * alpha + std::abs(xcp) * effPitch));
                    totalYawTorque += (qS * clFin * (xcp * beta + std::abs(xcp) * effYaw));
                }
                tx = std::clamp(totalRollTorque, -maxControlMoment, maxControlMoment);
                ty = std::clamp(totalPitchTorque, -maxControlMoment, maxControlMoment);
                tz = std::clamp(totalYawTorque, -maxControlMoment, maxControlMoment);

                // A supplied table adds validated body static coefficients to
                // the geometry-derived fin contribution. Without a table the
                // geometric-fin path retains its existing behavior and does
                // not reintroduce the abstract body stability term.
                if (hasCmTable) ty += qS * l * cmStatic;
                if (hasCnTable) tz += qS * l * cnStatic;
                if (hasClRollTable) tx += qS * l * clRollStatic;

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

                // Static body coefficients use tables when supplied and the
                // established scalar fallback otherwise.
                tx += qS * l * clRollStatic;
                ty += qS * l * cmStatic;
                tz += qS * l * cnStatic;

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
