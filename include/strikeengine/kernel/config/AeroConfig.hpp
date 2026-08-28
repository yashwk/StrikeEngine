#pragma once

#include <strikeengine/models/physics/aerodynamics/CoefficientTable.hpp>

namespace StrikeEngine::Kernel {

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
    };

} // namespace StrikeEngine::Kernel
