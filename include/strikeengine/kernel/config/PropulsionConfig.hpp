#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <strikeengine/models/physics/propulsion/ThrustCurve.hpp>

namespace StrikeEngine::Kernel {

    // Airbreathing (aircraft) engine. When enabled the stage produces thrust
    // from this deck instead of a rocket thrust curve, so an aircraft no longer
    // fakes endurance with an implausible Isp. Thrust lapses with ambient
    // pressure and fuel burns at TSFC. Mach lapse is not modelled yet (the same
    // honesty as the swept-wing caveat in AirframeModel). The stage's
    // propellantMassKg is the fuel load and dryMassKg the empty mass; throttle
    // is fixed until a throttle command path exists.
    struct AircraftEngineConfig {
        bool enabled = false;
        double seaLevelStaticThrustN = 0.0;
        // T/T0 = (P/P0)^exponent. 0.7 approximates the troposphere density lapse.
        double pressureLapseExponent = 0.7;
        double tsfcKgPerNPerS = 1.5e-5;  // ~0.53 lb/(lbf*h)
        double throttle = 1.0;
        // Thrust lapse with Mach: fraction of thrust lost per Mach (0 = none).
        double machLapsePerMach = 0.0;
        double machLapseMinFactor = 0.1;
    };

    struct StageConfig {
        std::vector<Models::ThrustDataPoint> thrustCurve;  // empty => inactive/coast stage
        AircraftEngineConfig aircraftEngine;  // when enabled, replaces thrustCurve
        double vacuumIsp = 250.0;   // s
        double seaLevelIsp = 220.0; // s
        double propellantMassKg = 0.0;
        double dryMassKg = 0.0;      // dropped at separation

        // Ignition and shutdown are relative to stage ignition. A negative
        // shutdown time means the thrust curve controls the cutoff.
        double ignitionDelaySec = 0.0;
        double ignitionRampSec = 0.0;
        double shutdownTimeSec = -1.0;
        double shutdownRampSec = 0.0;

        // Two-axis thrust-vector control. Zero limits preserve fixed axial
        // thrust. Positive pitch gimbals thrust toward body -Z; positive yaw
        // gimbals it toward body +Y.
        double maxGimbalPitchRad = 0.0;
        double maxGimbalYawRad = 0.0;
        double gimbalTimeConstantSec = 0.02;
        double maxGimbalRateRadPerSec = 0.0; // 0 = unlimited
        double enginePositionX = 0.0;
        double enginePositionY = 0.0;
        double enginePositionZ = 0.0;

        // Explicit nozzle exit position(s) in body frame relative to CG (m, +X noseward).
        // Each entry is [x, y, z]. If empty, the engine nozzle defaults to
        // [enginePositionX, enginePositionY, enginePositionZ].
        std::vector<std::array<double, 3>> nozzlePositions;
    };

    struct PropulsionConfig {
        std::vector<StageConfig> stages;  // empty => coasting vehicle
    };

    inline bool validatePropulsionConfig(const PropulsionConfig& config,
                                         std::string* error = nullptr) {
        for (std::size_t i = 0; i < config.stages.size(); ++i) {
            const auto& stage = config.stages[i];
            auto fail = [&](const std::string& message) {
                if (error) *error = "stage " + std::to_string(i) + ": " + message;
                return false;
            };
            if (stage.aircraftEngine.enabled) {
                // An airbreathing stage carries fuel and empty mass but no
                // rocket thrust curve; it burns to the fuel floor instead.
                const auto& eng = stage.aircraftEngine;
                if (!std::isfinite(eng.seaLevelStaticThrustN) || eng.seaLevelStaticThrustN <= 0.0) {
                    return fail("airbreathing engine needs a positive sea-level static thrust");
                }
                if (!std::isfinite(eng.pressureLapseExponent) || eng.pressureLapseExponent < 0.0) {
                    return fail("airbreathing pressure lapse exponent must be finite and non-negative");
                }
                if (!std::isfinite(eng.tsfcKgPerNPerS) || eng.tsfcKgPerNPerS <= 0.0) {
                    return fail("airbreathing TSFC must be finite and positive");
                }
                if (!std::isfinite(eng.throttle) || eng.throttle < 0.0 || eng.throttle > 1.0) {
                    return fail("airbreathing throttle must be within [0, 1]");
                }
                if (!std::isfinite(eng.machLapsePerMach) || eng.machLapsePerMach < 0.0) {
                    return fail("airbreathing Mach lapse must be finite and non-negative");
                }
                if (!std::isfinite(eng.machLapseMinFactor) ||
                    eng.machLapseMinFactor < 0.0 || eng.machLapseMinFactor > 1.0) {
                    return fail("airbreathing Mach-lapse floor must be within [0, 1]");
                }
                if (!stage.thrustCurve.empty()) {
                    return fail("an airbreathing stage cannot also declare a rocket thrust curve");
                }
            } else if (stage.thrustCurve.empty()) {
                if (stage.propellantMassKg != 0.0 || stage.dryMassKg != 0.0) {
                    return fail("an inactive stage cannot declare propellant or dry mass");
                }
                continue;
            } else {
                std::string curveError;
                if (!Models::ThrustCurve(stage.thrustCurve).validate(&curveError)) {
                    return fail(curveError);
                }
                if (!std::isfinite(stage.vacuumIsp) || stage.vacuumIsp <= 0.0 ||
                    !std::isfinite(stage.seaLevelIsp) || stage.seaLevelIsp <= 0.0) {
                    return fail("vacuum and sea-level Isp must be finite and positive");
                }
            }
            if (!std::isfinite(stage.propellantMassKg) || stage.propellantMassKg < 0.0 ||
                !std::isfinite(stage.dryMassKg) || stage.dryMassKg < 0.0) {
                return fail("propellant and dry masses must be finite and non-negative");
            }
            if (!std::isfinite(stage.ignitionDelaySec) || stage.ignitionDelaySec < 0.0 ||
                !std::isfinite(stage.ignitionRampSec) || stage.ignitionRampSec < 0.0 ||
                !std::isfinite(stage.shutdownTimeSec) ||
                !std::isfinite(stage.shutdownRampSec) || stage.shutdownRampSec < 0.0) {
                return fail("ignition/shutdown timing values are invalid");
            }
            if (!std::isfinite(stage.maxGimbalPitchRad) || stage.maxGimbalPitchRad < 0.0 ||
                !std::isfinite(stage.maxGimbalYawRad) || stage.maxGimbalYawRad < 0.0 ||
                !std::isfinite(stage.gimbalTimeConstantSec) || stage.gimbalTimeConstantSec < 0.0 ||
                !std::isfinite(stage.maxGimbalRateRadPerSec) || stage.maxGimbalRateRadPerSec < 0.0) {
                return fail("TVC limits and servo parameters are invalid");
            }
            if (!std::isfinite(stage.enginePositionX) || !std::isfinite(stage.enginePositionY) ||
                !std::isfinite(stage.enginePositionZ)) {
                return fail("engine position must be finite");
            }
            for (const auto& np : stage.nozzlePositions) {
                if (!std::isfinite(np[0]) || !std::isfinite(np[1]) || !std::isfinite(np[2])) {
                    return fail("nozzle position coordinates must be finite");
                }
            }
        }
        return true;
    }

} // namespace StrikeEngine::Kernel
