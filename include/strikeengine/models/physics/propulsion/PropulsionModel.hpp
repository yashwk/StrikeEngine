#pragma once
#include <strikeengine/models/physics/propulsion/ThrustCurve.hpp>
#include <algorithm>

namespace StrikeEngine::Models {

    /**
     * @brief Propulsion evaluation result (BODY frame).
     *
     * Thrust is assumed to act along the body X axis (1, 0, 0) — the
     * standard for a fixed axial motor. The backend rotates the body force
     * to the world frame via the vehicle quaternion.
     */
    struct PropulsionState {
        double thrustBodyX, thrustBodyY, thrustBodyZ;  // N (body frame)
        double massFlowRate_kg_s;
    };

    class PropulsionModel {
    public:
        PropulsionModel(const ThrustCurve& curve, double vacuumIsp, double slIsp)
            : thrustCurve(curve), isp_vacuum_s(vacuumIsp), isp_sl_s(slIsp) {}

        /**
         * @brief Evaluates propulsion forces and mass flow rate.
         * @param timeSinceIgnition_s Time since motor ignition.
         * @param ambientPressure_pa Local atmospheric pressure.
         * @return Evaluated body-frame thrust vector and mass flow rate.
         */
        PropulsionState evaluate(double timeSinceIgnition_s, double ambientPressure_pa) const {
            const double currentThrust = thrustCurve.evaluate(timeSinceIgnition_s);

            if (currentThrust <= 0.0) {
                return {0.0, 0.0, 0.0, 0.0};
            }

            constexpr double sea_level_pressure_pa = 101325.0;
            const double pressure_fraction = std::clamp(ambientPressure_pa / sea_level_pressure_pa, 0.0, 1.0);
            const double current_isp = isp_vacuum_s + (isp_sl_s - isp_vacuum_s) * pressure_fraction;

            constexpr double g0 = 9.80665;
            double massFlowRate = 0.0;
            if (current_isp > 0.0) {
                massFlowRate = currentThrust / (current_isp * g0);
            }

            // Thrust along +X body axis
            return {currentThrust, 0.0, 0.0, massFlowRate};
        }

    private:
        ThrustCurve thrustCurve;
        double isp_vacuum_s;
        double isp_sl_s;
    };

} // namespace StrikeEngine::Models
