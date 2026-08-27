#pragma once

#include <vector>
#include <strikeengine/models/physics/propulsion/ThrustCurve.hpp>

namespace StrikeEngine::Kernel {

    /**
     * @brief Per-vehicle truth-model configuration (W1: per-entity vehicle config).
     *
     * Passed to SimulationKernel::createVehicle(init, config) to define the
     * physics properties that used to be shared across ALL entities:
     *   - aero geometry (reference area/length) and coefficients
     *   - dry mass (fuel model: mass burns down to dryMass, never below)
     *   - propulsion (thrust curve + Isp); an empty thrust curve = coasting
     *
     * Defaults describe a coasting vehicle with the legacy MVP aero
     * coefficients (CD=0.3, no lift). Explicitly configure motors and
     * lift-related coefficients to get steered/boosted vehicles.
     */
    struct VehicleConfig {
        // --- Geometry / mass ---
        double referenceArea   = 0.1;   // m^2
        double referenceLength = 1.0;   // m (moment arm for torques)
        double massDry         = -1.0;  // kg; < 0  =>  dry mass == total mass (no fuel)

        // --- Aerodynamics (body frame, aerospace X-fwd/Y-right/Z-down) ---
        double cd      = 0.3;   // drag coefficient
        double clAlpha = 0.0;   // lift slope per rad of angle of attack
        double clFin   = 0.0;   // fin lift coefficient per rad of deflection
        double clMax   = 2.0;   // max |CL| (stall/control-surface limit)

        // --- Sensor geometry ---
        double imuLeverArmX = 0.0;  // m, body-frame offset from centre of mass to IMU
        double imuLeverArmY = 0.0;
        double imuLeverArmZ = 0.0;

        // --- Propulsion (empty curve => no motor, vehicle coasts) ---
        std::vector<Models::ThrustDataPoint> thrustCurve;  // time_s vs thrust_N
        double vacuumIsp = 250.0;                          // s
        double seaLevelIsp = 220.0;                        // s
    };

} // namespace StrikeEngine::Kernel
