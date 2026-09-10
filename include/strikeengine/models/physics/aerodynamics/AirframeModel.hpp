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
    };

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
        double cd0, double oswaldEff, double clMax)
    {
        if (wingSpanM <= 1e-6 || wingRootChordM <= 1e-6) return nullptr;

        auto a = std::make_shared<AirframeParams>();
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
        const double mac = std::max(p.referenceLength, a.wingMeanChordM);
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
        const double wingCmAlpha = (a.wingPositionM / std::max(mac, 1e-9)) * cLalpha;
        const double tailCmAlpha = -cLalpha * a.tailVolumeH * (1.0 - a.downwashPerAlpha);
        const double cmAlpha = wingCmAlpha + tailCmAlpha;
        // Elevator authority: realistic finite-elevator effectiveness (~0.8 per
        // rad), NOT the full-tail derivative cLalpha*tailVolumeH (which is far
        // larger than a real elevator and over-drives the heavy airframe). A
        // positive pitch command must produce a nose-UP moment.
        const double cMdelta = 0.80;
        const double cM = a.cm0 + cmAlpha * alpha + cMdelta * finPitch;

        // --- Side / yaw (directional) ------------------------------------------
        // Vertical tail provides directional (weathercock) stability: a positive
        // sideslip must yaw the nose to REDUCE that sideslip (restoring), i.e.
        // cn_beta < 0 for a stable aircraft. Rudder (finYaw) adds a positive
        // (nose-RIGHT) directional command with realistic authority.
        const double cnBeta = -cLalpha * a.tailVolumeV;            // <0 stable
        const double cn = cnBeta * beta + 0.50 * finYaw;
        const double cyBeta = cLalpha * a.tailVolumeV;             // side force per rad sideslip

        // --- Roll (dihedral + aileron) ------------------------------------------
        // Dihedral produces a restoring roll from sideslip; aileron (finRoll)
        // commands roll. Normalised to a ~ unity magnitude.
        const double clDihedral = -a.wingDihedralRad * cLalpha * (a.wingSpanM / (2.0 * std::max(mac, 1e-9)));
        const double cl = clDihedral * beta + 0.08 * finRoll;

        // --- Body-frame forces (N) ----------------------------------------------
        // Drag opposes the velocity vector.
        const double dragMag = q * S * cD;
        double fx = -dragMag * (u / V);
        double fy = -dragMag * (v / V) + q * S * cyBeta * beta;
        double fz = -dragMag * (w / V) - q * S * cL;      // -Z lift (up)

        // --- Body-frame moments (N*m) --------------------------------------------
        // Pitch (about Y), yaw (Z), roll (X). Rate damping (non-dimensional).
        const double lOverV = mac / V;
        const double qSL = q * S * mac;
        const double qSb = q * S * b;
        double tx = qSb * cl - q * S * b * 6.0 * lOverV * wx;      // roll damping
        double ty = qSL * cM - q * S * mac * 16.0 * lOverV * wy;   // pitch damping
        double tz = qSb * cn - q * S * b * 12.0 * lOverV * wz;     // yaw damping

        // Clamp the aircraft's control moments to a realistic authority bound
        // (based on the tail/wing control surface size) so the large q*S lead
        // doesn't over-drive the high-inertia airframe into a spin.
        const double maxAC = 4.0e5;   // ~Moment authority consistent with a finite elevator/rudder
        tx = std::clamp(tx, -maxAC, maxAC);
        ty = std::clamp(ty, -maxAC, maxAC);
        tz = std::clamp(tz, -maxAC, maxAC);

        return {fx, fy, fz, tx, ty, tz};
    }

} // namespace StrikeEngine::Models
