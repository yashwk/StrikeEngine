#pragma once
#include <cmath>
#include <algorithm>
#include <memory>
#include <strikeengine/models/physics/aerodynamics/CoefficientTable.hpp>
#include <strikeengine/models/physics/aerodynamics/FinsModel.hpp>

namespace StrikeEngine::Models {

    struct AirframeParams;  // defined in AirframeModel.hpp (aircraft wing-body-tail)

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

        // Abstract-fin control surface type (see AeroConfig::tailControl).
        // Geometric fin sets derive the sign from their CP lever arm.
        bool tailControl = false;      // false = canard (legacy), true = tail

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

        // Optional aircraft airframe. When present the entity is a wing-body-tail
        // aircraft and uses the DATCOM-light semi-empirical aircraft model
        // (airframeAeroWrench) instead of the axisymmetric missile path.
        std::shared_ptr<const Models::AirframeParams> airframe;

        // Body rotational inertia (kg*m^2), used to bound the aero moment as an
        // angular acceleration. Zero falls back to a fixed moment ceiling.
        double inertiaX = 0.0;
        double inertiaY = 0.0;
        double inertiaZ = 0.0;

        // Ceiling on the angular acceleration the aerodynamic moment may
        // command (rad/s^2). Bounds the linear coefficient model at large alpha.
        static constexpr double kMaxAngularAccelRadPerSec2 = 60.0;
        // Moment ceiling used when no inertia is supplied.
        static constexpr double kFallbackMomentCeiling = 600.0;

        /// Largest |moment| permitted on the given axis (0 = roll, 1 = pitch).
        double momentCeiling(int axis) const {
            const double inertia =
                (axis == 0) ? inertiaX : (axis == 1) ? inertiaY : inertiaZ;
            return (inertia > 0.0)
                ? kMaxAngularAccelRadPerSec2 * inertia
                : kFallbackMomentCeiling;
        }

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

    // Aircraft wing-body-tail aero wrench (defined in AirframeModel.hpp).
    AeroWrench airframeAeroWrench(
        double u, double v, double w,
        double wx, double wy, double wz,
        double finPitch, double finYaw, double finRoll,
        double density, double speedOfSound, const AeroParams& p);

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
     *
     * The total body moment is bounded as an angular acceleration through the
     * airframe's own inertia, so the limit is a property of the vehicle rather
     * than of dynamic pressure.
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

            // Aircraft wing-body-tail path: when an airframe is present, use the
            // DATCOM-light semi-empirical aircraft model. Missiles leave the
            // airframe null and follow the axisymmetric path below.
            if (p.airframe) {
                return airframeAeroWrench(u, v, w, wx, wy, wz,
                                          finPitch, finYaw, finRoll,
                                          density, speedOfSound, p);
            }

            const double V = std::sqrt(speedSq);
            const double q = 0.5 * density * speedSq;   // dynamic pressure
            const double S = p.referenceArea;
            const double l = p.referenceLength;
            const double mach = (speedOfSound > 1e-6) ? V / speedOfSound : 0.0;
            // Rate-damping terms scale with q·(l/V). At exactly zero airspeed
            // q is 0 and l/V is inf, and 0*inf = NaN would poison the moment;
            // clamp the divisor so the (vanishing) damping stays finite.
            const double vSafe = std::max(V, 1e-6);

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
            // Aero tables are exported one-sided (positive angles) for a
            // symmetric airframe and the interpolator clamps out-of-range
            // queries to the first column, so a negative angle would read the
            // zero-lift column and the airframe could not push over or yaw the
            // other way. Odd coefficients (lift and all moments) mirror
            // C(-a) = -C(a); cd is even. Two-sided grids are used as supplied.
            const auto gridIsOneSided = [](const std::vector<double>& grid) {
                return !grid.empty() && grid.front() >= 0.0;
            };
            const auto evalCoeff = [&](double angle, const std::vector<double>& grid,
                                       const std::vector<std::vector<double>>& table,
                                       bool odd) {
                const bool mirror = odd && gridIsOneSided(grid);
                const double query = mirror ? std::abs(angle) : angle;
                const double value = interpolateCoefficient(
                    mach, query, tables->machBreakpoints, grid, table);
                return (mirror && angle < 0.0) ? -value : value;
            };
            const auto tableAtAlpha = [&](const std::vector<std::vector<double>>& table) {
                return evalCoeff(alpha, tables->aoaBreakpointsRad, table, true);
            };
            const auto tableAtBeta = [&](const std::vector<std::vector<double>>& table) {
                return evalCoeff(beta, tables->betaBreakpointsRad, table, true);
            };

            // Drag opposes velocity. With tables the cd(M,a) grid is
            // authoritative; otherwise the flat p.cd coefficient is used. The
            // tables and p.cd are BODY drag: fin drag is computed from the
            // fin geometry and added below.
            double cd;
            if (tables) {
                cd = evalCoeff(alpha, tables->aoaBreakpointsRad, tables->cdTable, false);
            } else {
                cd = p.cd;
            }
            const auto fList = p.activeFins();
            double finDrag = 0.0;
            if (!fList.empty()) {
                constexpr double kGammaAir = 1.4;
                constexpr double kGasConstantAir = 287.05;
                const double airTemperatureK = (speedOfSound > 1e-6)
                    ? (speedOfSound * speedOfSound) / (kGammaAir * kGasConstantAir)
                    : 288.15;
                for (const auto& f : fList) {
                    if (!f) continue;
                    finDrag += f->dragC(mach, density, V, airTemperatureK).total;
                }
            }
            const double dragMag = q * S * (cd + finDrag);
            double fx = -dragMag * (u / V);
            double fy = -dragMag * (v / V);
            double fz = -dragMag * (w / V);

            const bool hasCmTable = tables && tables->hasCmTable();
            const bool hasCyTable = tables && tables->hasCyTable();
            const bool hasCnTable = tables && tables->hasCnTable();
            const bool hasClRollTable = tables && tables->hasClRollTable();

            // Lift (pitch plane): positive alpha => force -Z (up), saturated at
            // CL_max (stall / control limit). With geometric fins the fin lift
            // slope clAlpha(mach) acts on the fin's local AoA (alpha +
            // finPitch); without them the flat clFin term is used. The body
            // slope p.clAlpha and the fin slopes SUM to the airframe slope by
            // design (profile cl_alpha values are tuned with this sum), so
            // the body term is kept on the finned path too.
            double cl;
            if (!fList.empty()) {
                double finLift = 0.0;
                for (const auto& f : fList) {
                    if (!f) continue;
                    const double clFin = f->clAlpha(mach);
                    const double effPitch = f->steerable ? finPitch : 0.0;
                    // The deflection's local-AoA contribution is SIGNED by the
                    // fin CP lever arm so the control FORCE matches the coded
                    // +|xcp|*effPitch nose-up control MOMENT for both canards
                    // and tail fins: a +pitch tail command is trailing-edge UP
                    // (downward fin force at the tail, nose-up moment), not an
                    // up-force. Without the sign the tail-fin control force
                    // opposes the moment it accompanies.
                    const double deflPitch =
                        (f->cpLeverArmM >= 0.0) ? effPitch : -effPitch;
                    finLift += clFin * (alpha + deflPitch);
                }
                if (tables) {
                    cl = tableAtAlpha(tables->clTable) + finLift;
                } else {
                    cl = p.clAlpha * alpha + finLift;
                }
            } else if (tables) {
                cl = tableAtAlpha(tables->clTable) +
                     (p.tailControl ? -p.clFin : p.clFin) * finPitch;
            } else {
                cl = p.clAlpha * alpha +
                     (p.tailControl ? -p.clFin : p.clFin) * finPitch;
            }
            cl = std::clamp(cl, -p.clMax, p.clMax);
            fz -= q * S * cl;

            // Side force (yaw plane). Cy is a body +Y force coefficient. An
            // optional table supplies the body contribution; geometric or
            // abstract fin control terms are then added as before.
            if (hasCyTable) {
                fy += q * S * tableAtBeta(tables->cyTable);
            }
            if (!fList.empty()) {
                double cySum = 0.0;
                for (const auto& f : fList) {
                    if (!f) continue;
                    const double clFin = f->clAlpha(mach);
                    const double effYaw = f->steerable ? finYaw : 0.0;
                    // Yaw mirrors the pitch fix with the OPPOSITE sign: the
                    // coded +|xcp|*effYaw nose-right control moment corresponds
                    // to a tail fin pushing the tail LEFT (-Y force, i.e. a
                    // NEGATIVE cy contribution for xcp < 0) and a canard
                    // pushing the nose RIGHT (+Y force, negative cy for
                    // xcp > 0). tau_z = x*Fy with Fy = -qS*cy.
                    const double deflYaw =
                        (f->cpLeverArmM >= 0.0) ? -effYaw : effYaw;
                    cySum += clFin * (beta + deflYaw);
                }
                // Body sideslip lift. The pitch plane carries cl(M,alpha) or
                // clAlpha*alpha on top of its fin terms; without the same body
                // term in beta the airframe pitches far harder than it yaws
                // (the fin share alone is a fraction of the normal-force
                // slope). Supplying cy_table keeps that table authoritative.
                if (!hasCyTable) {
                    cySum += tables ? evalCoeff(beta, tables->aoaBreakpointsRad,
                                                tables->clTable, true)
                                    : p.clAlpha * beta;
                }
                fy -= q * S * std::clamp(cySum, -p.clMax, p.clMax);
            } else {
                // Abstract (finless) airframe: mirror the pitch body lift onto
                // the yaw axis. An axisymmetric airframe develops the same side
                // force in sideslip as it does lift in angle of attack; the
                // pitch plane got the body term from cl(M,alpha) or
                // clAlpha*alpha, but the yaw plane only ever had the direct fin
                // force, so the vehicle could pitch but not yaw. Supplying
                // cy_table keeps that table authoritative instead.
                const double cyBody = hasCyTable ? 0.0 : std::clamp(
                    tables ? evalCoeff(beta, tables->aoaBreakpointsRad,
                                       tables->clTable, true)
                           : p.clAlpha * beta,
                    -p.clMax, p.clMax);
                // Abstract fin force sign by surface type: canard adds the
                // lifting force (legacy), tail opposes it (geometric parity).
                const double cyFin  = std::clamp(p.clFin * finYaw, -p.clMax, p.clMax);
                fy += p.tailControl
                    ? -q * S * (cyFin + cyBody)
                    :  q * S * (cyFin - cyBody);
            }

            // Static aerodynamic coefficients. Cm is pitch moment about +Y,
            // Cn is yaw moment about +Z, and Cl is roll moment about +X.
            // The scalar values below are the established fallback. Tables
            // replace only the corresponding static coefficient and do not
            // replace fin-control or rate-damping terms.
            const double cmStatic = hasCmTable
                ? tableAtAlpha(tables->cmTable)
                : -0.5 * alpha;
            // Yaw static term mirrors pitch with the opposite sign: tau_z =
            // +x*Fy with Fy = -qS*cy, so a nose-right restoring moment for
            // positive sideslip is +0.5*beta, not -0.5*beta.
            const double cnStatic = hasCnTable
                ? tableAtBeta(tables->cnTable)
                : 0.5 * beta;
            const double clRollStatic = hasClRollTable
                ? tableAtBeta(tables->clRollTable)
                : 0.0;

            // --- Moments (body frame) ---
            // Fin control authority and the validated pitch restoring term.
            constexpr double CM_delta = 0.50;  // pitch/yaw moment per rad
            constexpr double Cl_delta = 0.15;  // roll moment per rad
            // Aero moment ceiling: an angular acceleration through the
            // airframe's own inertia. The previous bound decayed with dynamic
            // pressure (600 N*m falling to a 60 N*m floor), which removed
            // control authority exactly where a fin is most effective.
            const double maxRollMoment = p.momentCeiling(0);
            const double maxPitchMoment = p.momentCeiling(1);
            const double maxYawMoment = p.momentCeiling(2);
            const double qS = q * S;
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

                    // Roll damping needs the 1/V the RocketPy reference
                    // carries (q*S*l^2*cld*omega/(2V)); rollDampingCoeff is
                    // per-metre, so without /V the term is N*m/s, not N*m,
                    // and roll is over-damped by a factor ~V.
                    totalRollTorque += (qS * l * f->rollForcingPerRad(mach) * (f->cantRad + effRoll)
                                        - qS * l * l * 0.5 * f->rollDampingCoeff(mach) * wx / vSafe);
                    totalPitchTorque += (qS * clFin * (xcp * alpha + std::abs(xcp) * effPitch));
                    // Yaw static term carries an explicit minus that pitch
                    // does not: tau_z = +x*Fy with Fy = -qS*cy, while
                    // tau_y = -x*Fz with Fz = -qS*cl. With a bare +xcp*beta a
                    // tail fin (xcp < 0) yaws the nose AWAY from the velocity
                    // (anti-weathercock); -xcp*beta restores it, and stays
                    // correct for canards (xcp > 0 destabilize, as they must).
                    totalYawTorque += (qS * clFin * (-xcp * beta + std::abs(xcp) * effYaw));
                }
                tx = std::clamp(totalRollTorque, -maxRollMoment, maxRollMoment);
                ty = std::clamp(totalPitchTorque, -maxPitchMoment, maxPitchMoment);
                tz = std::clamp(totalYawTorque, -maxYawMoment, maxYawMoment);

                // A supplied table adds validated body static coefficients to
                // the geometry-derived fin contribution. Without a table the
                // geometric-fin path retains its existing behavior and does
                // not reintroduce the abstract body stability term.
                if (hasCmTable) ty += qS * l * cmStatic;
                if (hasCnTable) tz += qS * l * cnStatic;
                if (hasClRollTable) tx += qS * l * clRollStatic;

                // Static stability is now supplied by the fins (via xcp); the
                // bare body term is dropped so it is not double-counted.
                ty -= q * S * l * Cq  * (l / vSafe) * wy;
                tz -= q * S * l * Cq  * (l / vSafe) * wz;
            } else {
                tx = std::clamp(qS * l * (Cl_delta * finRoll),
                                -maxRollMoment, maxRollMoment);
                ty = std::clamp(qS * l * (CM_delta * finPitch),
                                -maxPitchMoment, maxPitchMoment);
                tz = std::clamp(qS * l * (CM_delta * finYaw),
                                -maxYawMoment, maxYawMoment);

                // Static body coefficients use tables when supplied and the
                // established scalar fallback otherwise.
                tx += qS * l * clRollStatic;
                ty += qS * l * cmStatic;
                tz += qS * l * cnStatic;

                // Rotational damping (dimensionless rate q_bar*l/V)
                const double lOverV = l / vSafe;
                constexpr double Clp = 6.0;    // roll damping
                ty -= q * S * l * Cq  * lOverV * wy;
                tz -= q * S * l * Cq  * lOverV * wz;
                tx -= q * S * l * Clp * lOverV * wx;
            }

            // Bound the complete aerodynamic moment, not only the commanded fin
            // contribution: at large AoA the linear static and rate-damping
            // terms can otherwise drive the rigid body into a spin.
            tx = std::clamp(tx, -maxRollMoment, maxRollMoment);
            ty = std::clamp(ty, -maxPitchMoment, maxPitchMoment);
            tz = std::clamp(tz, -maxYawMoment, maxYawMoment);

            return {fx, fy, fz, tx, ty, tz};
        }
    };

} // namespace StrikeEngine::Models

// Pull in the aircraft wing-body-tail model (defines AirframeParams,
// buildAirframeParams and the airframeAeroWrench called by computeWrench).
// Included here so any TU that uses BasicAeroModel also gets the definition.
#include <strikeengine/models/physics/aerodynamics/AirframeModel.hpp>
