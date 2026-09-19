#pragma once
#include <strikeengine/models/physics/propulsion/ThrustCurve.hpp>
#include <algorithm>
#include <cmath>

namespace StrikeEngine::Models {

    /**
     * @brief Propulsion evaluation result (BODY frame).
     *
     * Thrust is assumed to act along the body X axis (1, 0, 0) — the
     * standard for a fixed axial motor. The backend rotates the body force
     * to the world frame via the vehicle quaternion.
     */
    struct PropulsionState {
        double thrustBodyX = 0.0, thrustBodyY = 0.0, thrustBodyZ = 0.0;  // N
        double massFlowRate_kg_s = 0.0;
    };

    struct PropulsionModelOptions {
        double ignitionDelaySec = 0.0;
        double ignitionRampSec = 0.0;
        double shutdownTimeSec = -1.0;
        double shutdownRampSec = 0.0;
        double maxGimbalPitchRad = 0.0;
        double maxGimbalYawRad = 0.0;

        // Airbreathing engine: when enabled, thrust comes from the deck below
        // instead of the rocket thrust curve, and fuel burns at TSFC rather
        // than by Isp. The stage runs until its fuel floor, not a burn time.
        bool airbreathing = false;
        double seaLevelStaticThrustN = 0.0;
        double pressureLapseExponent = 0.7;
        double tsfcKgPerNPerS = 1.5e-5;
        double throttle = 1.0;
        // Thrust lapse with Mach number: fraction of thrust lost per Mach, so
        // T/T_static = clamp(1 - k*M, minFactor, 1). 0 = no Mach lapse (the
        // pre-existing behaviour). A monotonic linear decrease is a deliberate
        // simplification: a real turbofan has a ram rise through the
        // transonic, which this does not model.
        double machLapsePerMach = 0.0;
        double machLapseMinFactor = 0.1;
    };

    class PropulsionModel {
    public:
        PropulsionModel(const ThrustCurve& curve, double vacuumIsp, double slIsp)
            : thrustCurve(curve), isp_vacuum_s(vacuumIsp), isp_sl_s(slIsp) {}

        PropulsionModel(const ThrustCurve& curve, double vacuumIsp, double slIsp,
                        PropulsionModelOptions options)
            : thrustCurve(curve), isp_vacuum_s(vacuumIsp), isp_sl_s(slIsp),
              options_(options) {}

        /**
         * @brief Evaluates propulsion forces and mass flow rate.
         * @param timeSinceIgnition_s Time since motor ignition.
         * @param ambientPressure_pa Local atmospheric pressure.
         * @return Evaluated body-frame thrust vector and mass flow rate.
         */
        PropulsionState evaluate(double timeSinceIgnition_s, double ambientPressure_pa,
                                 double gimbalPitchRad = 0.0,
                                 double gimbalYawRad = 0.0,
                                 double mach = 0.0) const {
            const double activeTime = timeSinceIgnition_s - options_.ignitionDelaySec;
            if (activeTime < 0.0) return {};

            double multiplier = 1.0;
            if (options_.ignitionRampSec > 0.0) {
                multiplier *= std::clamp(activeTime / options_.ignitionRampSec, 0.0, 1.0);
            }
            if (options_.shutdownTimeSec >= 0.0 && activeTime >= options_.shutdownTimeSec) {
                if (options_.shutdownRampSec <= 0.0) return {};
                multiplier *= std::clamp(
                    1.0 - (activeTime - options_.shutdownTimeSec) / options_.shutdownRampSec,
                    0.0, 1.0);
            }

            constexpr double sea_level_pressure_pa = 101325.0;
            const double pressure_fraction =
                std::clamp(ambientPressure_pa / sea_level_pressure_pa, 0.0, 1.0);

            double currentThrust = 0.0;
            double massFlowRate = 0.0;
            if (options_.airbreathing) {
                // Thrust lapses with ambient pressure (troposphere density),
                // fuel burns at TSFC. Mach lapse is not modelled yet.
                const double machSafe = (std::isfinite(mach) && mach > 0.0) ? mach : 0.0;
                const double machFactor = std::clamp(
                    1.0 - options_.machLapsePerMach * machSafe,
                    options_.machLapseMinFactor, 1.0);
                currentThrust = std::clamp(options_.throttle, 0.0, 1.0) *
                    options_.seaLevelStaticThrustN *
                    std::pow(pressure_fraction, options_.pressureLapseExponent) *
                    machFactor * multiplier;
                massFlowRate = options_.tsfcKgPerNPerS * currentThrust;
            } else {
                currentThrust = thrustCurve.evaluate(activeTime) * multiplier;
                if (currentThrust > 0.0) {
                    const double current_isp =
                        isp_vacuum_s + (isp_sl_s - isp_vacuum_s) * pressure_fraction;
                    constexpr double g0 = 9.80665;
                    if (current_isp > 0.0) {
                        massFlowRate = currentThrust / (current_isp * g0);
                    }
                }
            }

            if (currentThrust <= 0.0) {
                return {0.0, 0.0, 0.0, 0.0};
            }

            // Sanitize the gimbal envelope: std::clamp requires lo <= hi, so a
            // negative configured limit (invalid but unvalidated input) must
            // not reach it as UB.
            const double pitchLimit = std::max(0.0, options_.maxGimbalPitchRad);
            const double yawLimit = std::max(0.0, options_.maxGimbalYawRad);
            const double pitch = std::clamp(gimbalPitchRad, -pitchLimit, pitchLimit);
            const double yaw = std::clamp(gimbalYawRad, -yawLimit, yawLimit);
            const double cp = std::cos(pitch);
            const double cy = std::cos(yaw);
            // Positive pitch is nose-up thrust (-Z); positive yaw is +Y.
            return {currentThrust * cp * cy,
                    currentThrust * std::sin(yaw),
                    -currentThrust * std::sin(pitch) * cy,
                    massFlowRate};
        }

        double burnDuration() const {
            // An airbreathing stage has no thrust-curve end: it runs until its
            // fuel floor, which the kernel's propellant-exhaustion check owns.
            // A huge sentinel keeps the staging curve-elapsed test from firing.
            if (options_.airbreathing) return 1.0e12;
            double duration = thrustCurve.lastPositiveTime();
            if (options_.shutdownTimeSec >= 0.0) {
                duration = std::min(duration,
                    options_.shutdownTimeSec + options_.shutdownRampSec);
            }
            return options_.ignitionDelaySec + std::max(0.0, duration);
        }

    private:
        ThrustCurve thrustCurve;
        double isp_vacuum_s;
        double isp_sl_s;
        PropulsionModelOptions options_;
    };

} // namespace StrikeEngine::Models
