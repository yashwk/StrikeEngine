#pragma once

namespace StrikeEngine::Kernel {

    struct AeroConfig {
        double referenceArea   = 0.1;   // m^2
        double referenceLength = 1.0;   // m (moment arm for torques)
        double cd      = 0.3;   // drag coefficient
        double clAlpha = 0.0;   // lift slope per rad AoA
        double clFin   = 0.0;   // fin lift coefficient per rad deflection
        double clMax   = 2.0;   // max |CL|
    };

} // namespace StrikeEngine::Kernel
