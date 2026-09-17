#pragma once
#include <cmath>
#include <algorithm>
#include <strikeengine/models/physics/aerodynamics/AeroModel.hpp>

namespace StrikeEngine::Models {

    /**
     * @brief Aircraft airframe geometry + semi-empirical aero params (W-AC).
     *
     * This is a DATCOM-light wing-body-tail model for aircraft. Unlike the
     * axisymmetric missile path (Barrowman/Diederich fins), an aircraft has a
     * main lifting wing, a horizontal tail (stability + elevator) and a
     * vertical tail (directional stability + rudder), plus a fuselage. Lift is
     * produced by the wing (finite-wing lift slope), the tail provides
     * longitudinal (pitch) static stability, and drag is the classic
     * [parasite + induced] breakdown.
     *
     * Conventions match AeroModel: body axes X forward, Y right, Z down;
     * +alpha (velocity from below, w>0 with Z down) => lift in -Z and a
     * NEGATIVE (restoring) pitch moment for a statically stable aircraft.
     */
    struct AirframeParams {
        // Main wing (reference surface). The wing planform area is the
        // aerodynamic reference (S_ref) for the aircraft.
        double wingAreaM2       = 0.0;
        double wingSpanM        = 0.0;
        double wingAspectRatio  = 0.0;    // AR = b^2 / S
        double wingMeanChordM   = 0.0;    // MAC
        double wingPositionM    = 0.0;    // quarter-chord x-position aft of CG (m, +aft)
        double wingDihedralRad  = 0.0;
        double wingClMax        = 1.6;

        // Horizontal tail (stability + elevator).
        double htailAreaM2      = 0.0;
        double htailSpanM       = 0.0;
        double htailAspectRatio = 0.0;
        double htailPositionM   = 0.0;    // tail x-position aft of CG (m, +aft)

        // Vertical tail (directional stability + rudder).
        double vtailAreaM2      = 0.0;
        double vtailSpanM       = 0.0;
        double vtailPositionM   = 0.0;

        // Fuselage.
        double fuselageDiameterM = 0.0;
        double fuselageLengthM   = 0.0;

        // Control-surface chord fractions (movable chord / surface chord). These
        // set the control effectiveness factor tau; 0.3 is a typical transport
        // elevator/rudder/aileron.
        double elevatorChordFraction = 0.30;
        double rudderChordFraction   = 0.30;
        double aileronChordFraction  = 0.25;
        double aileronSpanFraction   = 0.35;  // aileron span / semispan (per side)

        // Derived semi-empirical terms (Mach-independent base values; the
        // Mach-dependent lift slope is evaluated per-step).
        double cd0              = 0.03;   // parasite + wave drag
        double oswaldEfficiency = 0.8;
        double inducedFactor    = 0.0;    // k = 1/(pi*e*AR)
        double cm0              = 0.0;    // zero-lift / fuselage pitching
        double downwashPerAlpha = 0.30;   // d(eps)/d(alpha) at the horizontal tail
        double tailVolumeH      = 0.0;    // (S_ht * l_ht) / (S_w * MAC)
        double tailVolumeV      = 0.0;    // (S_vt * l_vt) / (S_w * b_w)

        double referenceLength  = 1.0;    // MAC (for moment/non-dimensionalising)

        // Derived control power (per rad), from tail volume and control
        // effectiveness. Filled by buildAirframeParams at the low-Mach slope;
        // the per-step wrench rescales by the current Mach slope ratio.
        double elevatorPowerPerRad = 0.0;
        double rudderPowerPerRad   = 0.0;
        double aileronPowerPerRad  = 0.0;

        // Low-Mach reference slopes, for the per-step Mach rescaling of the
        // control powers above.
        double referenceWingSlope  = 0.0;
        double referenceTailSlopeH = 0.0;
        double referenceTailSlopeV = 0.0;
    };

    /**
     * @brief Control-surface effectiveness factor tau.
     *
     * The standard plain-flap relation tau = 1 - (theta - sin theta)/pi with
     * cos(theta) = 2*cf/c - 1, from thin-airfoil theory (Nelson, "Flight
     * Stability and Automatic Control"). Real values run somewhat lower
     * because of viscosity; DATCOM's charts are the reference for a refined
     * value.
     */
    inline double controlEffectivenessTau(double chordFraction)
    {
        const double f = std::clamp(chordFraction, 0.0, 1.0);
        if (f <= 0.0) return 0.0;
        if (f >= 1.0) return 1.0;
        const double theta = std::acos(std::clamp(2.0 * f - 1.0, -1.0, 1.0));
        return std::clamp(1.0 - (theta - std::sin(theta)) / M_PI, 0.0, 1.0);
    }

    /// Finite-surface lift-curve slope (Helmbold/Prandtl) at a given Mach.
    inline double surfaceLiftSlope(double aspectRatio, double mach)
    {
        const double beta = std::sqrt(std::max(1.0 - mach * mach, 0.05));
        const double ar = std::max(aspectRatio, 0.1);
        const double twoD = 2.0 * M_PI / beta;
        return twoD * ar / (2.0 + std::sqrt(4.0 + ar * ar / (beta * beta)));
    }

    /**
     * @brief Build AirframeParams (finite-wing / tail volume long-hand) from
     * the declared aircraft geometry. Returns nullopt if the airframe is not a
     * supported aircraft (no wing / zero span). This is called once at
     * load-time, not per-step.
     */
    inline std::shared_ptr<const AirframeParams> buildAirframeParams(
        double wingSpanM, double wingRootChordM, double wingTipChordM,
        double wingSweepDeg, double wingPositionM, double wingDihedralDeg,
        double htailSpanM, double htailChordM, double htailPositionM,
        double vtailSpanM, double vtailChordM, double vtailPositionM,
        double fuselageDiameterM, double fuselageLengthM,
        double cd0, double oswaldEff, double clMax,
        double elevatorChordFraction = 0.30, double rudderChordFraction = 0.30,
        double aileronChordFraction = 0.25, double aileronSpanFraction = 0.35)
    {
        (void)wingSweepDeg;  // swept-wing corrections not modelled yet
        if (wingSpanM <= 1e-6 || wingRootChordM <= 1e-6) return nullptr;

        auto a = std::make_shared<AirframeParams>();
        a->elevatorChordFraction = elevatorChordFraction;
        a->rudderChordFraction   = rudderChordFraction;
        a->aileronChordFraction  = aileronChordFraction;
        a->aileronSpanFraction   = aileronSpanFraction;
        a->wingSpanM    = wingSpanM;
        a->wingMeanChordM = 0.5 * (wingRootChordM + wingTipChordM);
        a->wingAreaM2   = wingSpanM * a->wingMeanChordM;
        a->wingAspectRatio = a->wingSpanM * a->wingSpanM / std::max(a->wingAreaM2, 1e-9);
        a->wingPositionM  = wingPositionM;
        a->wingDihedralRad = wingDihedralDeg * (M_PI / 180.0);
        a->wingClMax     = clMax;

        a->htailAreaM2   = htailSpanM * htailChordM;
        a->htailSpanM    = htailSpanM;
        a->htailAspectRatio = (htailSpanM > 1e-6 && a->htailAreaM2 > 1e-9)
            ? htailSpanM * htailSpanM / a->htailAreaM2 : 0.0;
        a->htailPositionM = htailPositionM;

        a->vtailAreaM2   = vtailSpanM * vtailChordM;
        a->vtailSpanM    = vtailSpanM;
        a->vtailPositionM = vtailPositionM;

        a->fuselageDiameterM = fuselageDiameterM;
        a->fuselageLengthM   = fuselageLengthM;

        a->cd0            = cd0;
        a->oswaldEfficiency = oswaldEff;
        a->inducedFactor  = 1.0 / (M_PI * oswaldEff * a->wingAspectRatio);
        a->cm0            = 0.0;
        a->referenceLength = a->wingMeanChordM;

        // Tail volume coefficients (aft-tail => positive, statically stable).
        const double mac = std::max(a->wingMeanChordM, 1e-9);
        a->tailVolumeH = (a->htailAreaM2 * a->htailPositionM) / (a->wingAreaM2 * mac);
        a->tailVolumeV = (a->vtailAreaM2 * a->vtailPositionM) / (a->wingAreaM2 * a->wingSpanM);

        // Control power, derived rather than assumed. DATCOM:
        //   Cm_de = -a_h * V_H * eta * tau_e        (elevator)
        //   Cn_dr = -a_v * V_V * eta * tau_r        (rudder)
        // a_h / a_v are the surface lift-curve slopes and eta the tail dynamic
        // pressure ratio. Stored at the low-Mach slope; the wrench rescales by
        // the ratio of current to low-Mach slope so the Mach dependence is not
        // frozen at load time.
        constexpr double kTailEfficiency = 0.95;
        const double aHlow = surfaceLiftSlope(a->htailAspectRatio, 0.0);
        const double aVlow = surfaceLiftSlope(
            (a->vtailSpanM > 1e-6 && a->vtailAreaM2 > 1e-9)
                ? a->vtailSpanM * a->vtailSpanM / a->vtailAreaM2 : 0.0, 0.0);
        a->elevatorPowerPerRad = aHlow * a->tailVolumeH * kTailEfficiency
            * controlEffectivenessTau(a->elevatorChordFraction);
        a->rudderPowerPerRad = aVlow * a->tailVolumeV * kTailEfficiency
            * controlEffectivenessTau(a->rudderChordFraction);

        // Aileron roll power. The rolling moment of a symmetric aileron pair is
        //   Cl_da = Cl_alpha_w * tau * c * b * (1 - a^2) / (2 * S)
        // for ailerons spanning a..1 of the semispan (each side), from
        // integrating y*c dy over the aileron span.
        const double aileronInboardFrac =
            1.0 - std::clamp(a->aileronSpanFraction, 0.0, 1.0);
        a->aileronPowerPerRad = surfaceLiftSlope(a->wingAspectRatio, 0.0)
            * controlEffectivenessTau(a->aileronChordFraction)
            * (a->wingMeanChordM * a->wingSpanM
               * (1.0 - aileronInboardFrac * aileronInboardFrac))
            / (2.0 * std::max(a->wingAreaM2, 1e-9));

        // Low-Mach reference slopes, for the per-step Mach rescaling.
        a->referenceWingSlope = surfaceLiftSlope(a->wingAspectRatio, 0.0);
        a->referenceTailSlopeH = aHlow;
        a->referenceTailSlopeV = aVlow;

        return a;
    }

    /**
     * @brief Aircraft semi-empirical aero wrench (DATCOM-light wing-body-tail).
     *
     * Returns BODY-frame forces (N) and moments (N*m). Used when
     * AeroParams::airframe is present; otherwise BasicAeroModel follows the
     * axisymmetric missile path.
     *
     * Control deflections map to aircraft surfaces: finPitch -> elevator,
     * finYaw -> rudder, finRoll -> aileron.
     */
    inline AeroWrench airframeAeroWrench(
        double u, double v, double w,
        double wx, double wy, double wz,
        double finPitch, double finYaw, double finRoll,
        double density, double speedOfSound,
        const AeroParams& p)
    {
        const double speedSq = u * u + v * v + w * w;
        if (speedSq < 1e-6) return {0, 0, 0, 0, 0, 0};

        const double V = std::sqrt(speedSq);
        const double q = 0.5 * density * speedSq;
        const AirframeParams& a = *p.airframe;
        const double S = std::max(p.referenceArea, a.wingAreaM2);   // aero reference (wing area)
        // Moment arm MUST be the wing mean aerodynamic chord. The vehicle
        // referenceLength is the fuselage/body length (e.g. 21.9 m for a
        // fighter vs a ~3.3 m MAC); using it here inflated every pitching /
        // yawing / rolling moment ~7x and the rate-damping stiffness ~44x,
        // pushing rotational dynamics far past explicit-integration
        // stability at dt = 0.01 s (integrator-dependent spins and chatter
        // instead of damped motion). Fall back to referenceLength only when
        // the geometry carries no usable chord.
        const double mac = (a.wingMeanChordM > 1e-9) ? a.wingMeanChordM
                                                     : std::max(p.referenceLength, 1e-9);
        const double b  = a.wingSpanM;
        const double mach = (speedOfSound > 1e-6) ? V / speedOfSound : 0.0;

        const double alpha = std::atan2(w, u);
        const double beta  = std::atan2(v, u);

        // Finite-wing lift slope with compressibility (Prandtl/Helmbold).
        const double betaM = std::sqrt(std::max(1.0 - mach * mach, 0.05));
        const double aTwoD = 2.0 * M_PI / betaM;
        const double ar = std::max(a.wingAspectRatio, 0.5);
        const double cLalpha = aTwoD * ar / (2.0 + std::sqrt(4.0 + ar * ar / (betaM * betaM)));

        // --- Lift & drag (longitudinal) ---------------------------------------
        double cL = cLalpha * alpha;
        // small fuselage/belly lift contribution (slender-body Krueger):
        cL += a.fuselageDiameterM / std::max(b, 1e-6) * 0.5 * alpha;
        cL = std::clamp(cL, -a.wingClMax, a.wingClMax);

        const double cD = a.cd0 + a.inducedFactor * cL * cL;

        // --- Pitching moment (static stability) --------------------------------
        // Wing contribution about the CG (x_cg - x_ac ~ 0 by default) + the
        // aft-tail stabilising term, negative for a statically stable aircraft.
        // wingPositionM is +aft (AeroConfig convention): an aerodynamic centre
        // AFT of the CG is stabilising (up-lift behind the CG gives a
        // nose-down moment for +alpha), so the wing term carries a minus,
        // matching the tail term's sign convention.
        const double wingCmAlpha = -(a.wingPositionM / std::max(mac, 1e-9)) * cLalpha;
        const double tailCmAlpha = -cLalpha * a.tailVolumeH * (1.0 - a.downwashPerAlpha);
        const double cmAlpha = wingCmAlpha + tailCmAlpha;
        // Elevator authority, derived from tail volume and the elevator chord
        // fraction rather than assumed. It is the DATCOM control power rescaled
        // from the load-time low-Mach slope to the current Mach. A positive
        // pitch command produces a nose-UP moment, matching the engine's
        // control-sign convention.
        const double aHnow = surfaceLiftSlope(a.htailAspectRatio, mach);
        const double elevatorPower = (a.referenceTailSlopeH > 1e-9)
            ? a.elevatorPowerPerRad * (aHnow / a.referenceTailSlopeH)
            : a.elevatorPowerPerRad;
        const double cM = a.cm0 + cmAlpha * alpha + elevatorPower * finPitch;

        // --- Side / yaw (directional) ------------------------------------------
        // Vertical tail provides directional (weathercock) stability. Body
        // axes are X-forward/Y-right/Z-down and beta = atan2(v, u), so a
        // positive sideslip (velocity right of the nose, wind from the
        // front-right) pushes the tail left (-Y force) and must yaw the nose
        // RIGHT (+tz, toward the velocity) to reduce the slip. Hence cyBeta
        // < 0 and cnBeta > 0 for a stable aircraft (verified open-loop: a
        // +5 deg uncommanded sideslip must develop +wz, not -wz).
        const double cnBeta = cLalpha * a.tailVolumeV;               // >0 stable
        const double aVnow = surfaceLiftSlope(
            (a.vtailSpanM > 1e-6 && a.vtailAreaM2 > 1e-9)
                ? a.vtailSpanM * a.vtailSpanM / a.vtailAreaM2 : 0.0, mach);
        const double rudderPower = (a.referenceTailSlopeV > 1e-9)
            ? a.rudderPowerPerRad * (aVnow / a.referenceTailSlopeV)
            : a.rudderPowerPerRad;
        const double cn = cnBeta * beta + rudderPower * finYaw;
        const double cyBeta = -cLalpha * a.tailVolumeV;              // side force per rad sideslip

        // --- Roll (dihedral + aileron) ------------------------------------------
        // Dihedral produces a restoring roll from sideslip; the aileron commands
        // roll, its power derived from the wing slope and aileron geometry.
        const double clDihedral = -a.wingDihedralRad * cLalpha * (a.wingSpanM / (2.0 * std::max(mac, 1e-9)));
        const double aileronPower = (a.referenceWingSlope > 1e-9)
            ? a.aileronPowerPerRad * (cLalpha / a.referenceWingSlope)
            : a.aileronPowerPerRad;
        const double cl = clDihedral * beta + aileronPower * finRoll;

        // --- Body-frame forces (N) ----------------------------------------------
        // Drag opposes the velocity vector. The rate-damping terms divide by
        // V while q is 0 there, so clamp the divisor (0*inf = NaN otherwise).
        const double vSafe = std::max(V, 1e-6);
        const double dragMag = q * S * cD;
        double fx = -dragMag * (u / vSafe);
        double fy = -dragMag * (v / vSafe) + q * S * cyBeta * beta;
        double fz = -dragMag * (w / vSafe) - q * S * cL;      // -Z lift (up)

        // --- Body-frame moments (N*m) --------------------------------------------
        // Pitch (about Y), yaw (Z), roll (X). Rate damping (non-dimensional).
        const double lOverV = mac / vSafe;
        const double qSL = q * S * mac;
        const double qSb = q * S * b;
        double tx = qSb * cl - q * S * b * 6.0 * lOverV * wx;      // roll damping
        double ty = qSL * cM - q * S * mac * 16.0 * lOverV * wy;   // pitch damping
        double tz = qSb * cn - q * S * b * 12.0 * lOverV * wz;     // yaw damping

        // Clamp the aircraft's control moments to a realistic authority bound
        // (based on the tail/wing control surface size) so the large q*S lead
        // doesn't over-drive the high-inertia airframe into a spin. This must be
        // large enough for a finite elevator/rudder to be effective at the
        // operating dynamic pressure (a Fin at full +/-0.43 rad on a 48 m^2
        // fighter wing at ~40 kPa produces ~2-3 MN*m of moment); 4e5 N*m capped
        // both control AND rate-damping far below the heavy airframe's need and
        // let the aircraft tumble. 3e6 N*m keeps the fins effective through
        // transonic/Mach 1.5 while still bounding the absurd control growth an
        // unconstrained q*S lead would produce.
        const double maxAC = 3.0e6;   // ~Moment authority consistent with a finite elevator/rudder
        tx = std::clamp(tx, -maxAC, maxAC);
        ty = std::clamp(ty, -maxAC, maxAC);
        tz = std::clamp(tz, -maxAC, maxAC);

        return {fx, fy, fz, tx, ty, tz};
    }

} // namespace StrikeEngine::Models
