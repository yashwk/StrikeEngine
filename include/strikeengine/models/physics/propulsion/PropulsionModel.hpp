#pragma once
#include <strikeengine/models/physics/propulsion/ThrustCurve.hpp>
#include <algorithm>

namespace StrikeEngine::Models {

    struct PropulsionState {
        double thrust_x, thrust_y, thrust_z;
        double massFlowRate_kg_s;
    };

    class PropulsionModel {
    public:
        PropulsionModel(const ThrustCurve& curve, double vacuumIsp, double slIsp) 
            : thrustCurve(curve), isp_vacuum_s(vacuumIsp), isp_sl_s(slIsp) {}

        /**
         * @brief Evaluates propulsion forces and mass flow rate.
         * @param time_s Time since ignition.
         * @param qx, qy, qz, qw Orientation of the vehicle.
         * @param ambientPressure_pa Local atmospheric pressure.
         * @return Evaluated thrust vector and mass flow rate.
         */
        PropulsionState evaluate(double time_s, double qx, double qy, double qz, double qw, double ambientPressure_pa) const {
            double currentThrust = thrustCurve.evaluate(time_s);

            if (currentThrust <= 0.0) {
                return {0.0, 0.0, 0.0, 0.0};
            }

            // Assume thrust is along the body X axis (1, 0, 0)
            // ThrustDir = q * (1, 0, 0) * q^-1
            double dir_x = 1.0 - 2.0 * (qy * qy + qz * qz);
            double dir_y = 2.0 * (qx * qy + qw * qz);
            double dir_z = 2.0 * (qx * qz - qw * qy);

            double thrust_x = dir_x * currentThrust;
            double thrust_y = dir_y * currentThrust;
            double thrust_z = dir_z * currentThrust;

            constexpr double sea_level_pressure_pa = 101325.0;
            double pressure_fraction = std::clamp(ambientPressure_pa / sea_level_pressure_pa, 0.0, 1.0);
            double current_isp = isp_vacuum_s + (isp_sl_s - isp_vacuum_s) * pressure_fraction;

            const double g0 = 9.80665;
            double massFlowRate = 0.0;
            if (current_isp > 0) {
                massFlowRate = currentThrust / (current_isp * g0);
            }

            return {thrust_x, thrust_y, thrust_z, massFlowRate};
        }

    private:
        ThrustCurve thrustCurve;
        double isp_vacuum_s;
        double isp_sl_s;
    };

} // namespace StrikeEngine::Models
