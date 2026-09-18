#pragma once

#include <vector>
#include <array>
#include <strikeengine/models/physics/aerodynamics/CoefficientTable.hpp>
#include <strikeengine/models/physics/aerodynamics/FinsModel.hpp>

namespace StrikeEngine::Kernel {

    /**
     * Optional geometric fin set (RocketPy-style: trapezoidal, elliptical,
     * free-form). count == 0 disables; count >= 3 enables geometry-derived
     * Mach-dependent fin aerodynamics (replacing the flat clFin/CM_delta/
     * Cl_delta abstract-fin terms).
     *
     * positionM: fin root leading edge axial offset from the CG along body
     * +X (nose positive; tail fins are negative). cantAngleDeg: fin cant.
     */
    struct FinsConfig {
        Models::FinShape shape = Models::FinShape::Trapezoidal;
        int    count = 0;
        double rootChordM = 0.0;
        double tipChordM  = 0.0;
        double spanM      = 0.0;
        double sweepLengthM = -1.0;
        double positionM    = 0.0;
        double cantAngleDeg = 0.0;
        std::vector<std::array<double, 2>> shapePoints;
        bool steerable = true;

        Models::FinAirfoil airfoil = Models::FinAirfoil::FlatPlate;
        double thicknessRatio = 0.0;
        double maxThicknessLocation = 0.5;
        double leadingEdgeRadius = 0.0;
        double trailingEdgeThickness = 0.0;

        double crankFraction = 0.5;
        double crankChordFactor = 0.5;
        double midChordLandFraction = 0.33;
        int controlType = 0;
        double controlFraction = 1.0;

        // Mounting offset, azimuth and dihedral (body frame relative to centerline, m/deg)
        double rootOffsetY = 0.0;
        double rootOffsetZ = 0.0;
        double baseAzimuthDeg = 0.0;
        double dihedralDeg = 0.0;

        bool enabled() const { return count >= 3; }
    };

    /**
     * @brief Aircraft airframe geometry (W-AC). When a main wing is present
     * (wingSpanM > 0) the entity is treated as a wing-body-tail aircraft and
     * uses the DATCOM-light semi-empirical aero model (AirframeModel.hpp)
     * instead of the axisymmetric missile/fin model. Missiles leave wingSpanM
     * = 0 (the default) so the existing path is byte-identical.
     *
     * Positions are axial along body +X from the CG (aft positive). The wing
     * and tail chords/positions define the aerodynamic reference and tail
     * volume coefficients, so the aircraft gets real static stability and
     * elevator/rudder authority.
     */
    struct AirframeConfig {
        // Main wing (the aerodynamic reference surface).
        double wingSpanM     = 0.0;
        double wingRootChordM = 0.0;
        double wingTipChordM  = 0.0;
        double wingSweepDeg    = 0.0;
        double wingPositionM   = 0.0;
        double wingDihedralDeg = 0.0;

        // Horizontal tail (stability + elevator).
        double htailSpanM    = 0.0;
        double htailChordM   = 0.0;
        double htailPositionM = 0.0;

        // Vertical tail (directional stability + rudder).
        double vtailSpanM    = 0.0;
        double vtailChordM   = 0.0;
        double vtailPositionM = 0.0;

        // Fuselage.
        double fuselageDiameterM = 0.0;
        double fuselageLengthM   = 0.0;

        // Parasite/wave drag at zero lift (CD0); induced-drag efficiency.
        double cd0 = 0.03;
        double oswaldEfficiency = 0.8;
        double clMax = 1.6;

        bool enabled() const { return wingSpanM > 0.0; }
    };

    struct AeroConfig {
        double referenceArea   = 0.1;   // m^2
        double referenceLength = 1.0;   // m (moment arm for torques)
        double cd      = 0.3;   // drag coefficient
        double clAlpha = 0.0;   // lift slope per rad AoA
        double clFin   = 0.0;   // fin lift coefficient per rad deflection
        double clMax   = 2.0;   // max |CL|

        // Control-surface type for the ABSTRACT fin terms (clFin/CM_delta).
        // Geometric fin sets derive this from each fin's CP lever arm and
        // ignore the flag. false = canard (the abstract default: +deflection
        // pairs a nose-up moment with a lifting force, the historic behavior);
        // true = tail (+deflection pairs the nose-up moment with a download,
        // matching the physical geometric-fin path).
        bool tailControl = false;

        // Data-driven cd(M,a)/cl(M,a) coefficient tables. Empty by default;
        // when non-empty they are authoritative for cd/cl at runtime.
        Models::AeroTables tables;

        // Optional geometric fins (trapezoidal/elliptical/free-form).
        FinsConfig fins;

        // Optional multiple geometric fin sets (e.g. canards + tails).
        // When non-empty, all enabled fin sets in finSets are simulated.
        std::vector<FinsConfig> finSets;

        // Aircraft airframe geometry. When enabled, the entity uses the
        // wing-body-tail semi-empirical aero model instead of the axisymmetric
        // missile model.
        AirframeConfig airframe;

        std::vector<FinsConfig> allFinSets() const {
            if (!finSets.empty()) {
                return finSets;
            }
            if (fins.enabled()) {
                return {fins};
            }
            return {};
        }
    };

} // namespace StrikeEngine::Kernel
