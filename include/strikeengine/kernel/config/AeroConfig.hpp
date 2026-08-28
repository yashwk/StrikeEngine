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
        int    count = 0;                  // 0 = disabled, >= 3 enabled
        double rootChordM = 0.0;
        double tipChordM  = 0.0;           // trapezoidal only
        double spanM      = 0.0;
        double sweepLengthM = -1.0;        // <0 -> root - tip (trapezoidal)
        double positionM    = 0.0;
        double cantAngleDeg = 0.0;
        std::vector<std::array<double, 2>> shapePoints; // free-form only

        bool enabled() const { return count >= 3; }
    };

    struct AeroConfig {
        double referenceArea   = 0.1;   // m^2
        double referenceLength = 1.0;   // m (moment arm for torques)
        double cd      = 0.3;   // drag coefficient
        double clAlpha = 0.0;   // lift slope per rad AoA
        double clFin   = 0.0;   // fin lift coefficient per rad deflection
        double clMax   = 2.0;   // max |CL|

        // Data-driven cd(M,a)/cl(M,a) coefficient tables. Empty by default;
        // when non-empty they are authoritative for cd/cl at runtime.
        Models::AeroTables tables;

        // Optional geometric fins (trapezoidal/elliptical/free-form).
        FinsConfig fins;
    };

} // namespace StrikeEngine::Kernel
