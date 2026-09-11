#pragma once

// Tilted-dipole Earth magnetic field MVP (heading-aiding only).
//
// A full WMM/IGRF spherical-harmonic model is overkill for an EKF heading
// aid: the filter observes attitude ERROR from the body-frame residual, so a
// smooth reference plus loose measurement noise and a magnitude disturbance
// gate is the honest operating point. Both the sensor truth path and the
// navigation prediction path MUST call this same function (shared reference,
// no model-mismatch bias).
//
// Axis choice: a plain AXIAL dipole (moment along -Z ECEF), not a tilted
// one. Validated at the test range (28.5°N, 71.8°E): axial gives
// declination 0.0° (truth ≈ +1.5°E) and dip ≈ 47° down (truth ≈ 40°),
// while the best-guess tilted axis gave +8.7° declination — a 7° systematic
// yaw bias on every vehicle (7 km of relay error at 60 km), which blew up
// the cooperative scenario on first contact. A dipole cannot match WMM
// regionally; axial happens to be closest HERE. Global coverage wants WMM.
//
// Closed form: B = (μ0/4π r³) [ 3 (m·n̂) n̂ − m ], n̂ = r / |r|.

#include <array>
#include <cmath>

namespace StrikeEngine::Models {

inline std::array<double, 3> dipoleMagneticFieldEcef(double x, double y, double z)
{
    // Dipole moment (A·m²): axial (along ECEF -Z). See header note on why
    // axial beats a guessed tilt at the test range.
    constexpr double kMoment = 7.94e22;
    constexpr double kMu0div4pi = 1e-7;
    const double mx = 0.0;
    const double my = 0.0;
    const double mz = -kMoment;

    const double r2 = x * x + y * y + z * z;
    const double r = std::sqrt(r2);
    if (r <= 1e-9) return {0.0, 0.0, 0.0};
    const double nx = x / r, ny = y / r, nz = z / r;
    const double mdotn = mx * nx + my * ny + mz * nz;
    const double k = kMu0div4pi / (r2 * r);
    return {
        k * (3.0 * mdotn * nx - mx),
        k * (3.0 * mdotn * ny - my),
        k * (3.0 * mdotn * nz - mz)};
}

} // namespace StrikeEngine::Models
