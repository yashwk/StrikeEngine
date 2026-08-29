#pragma once

#include <vector>
#include <cmath>
#include <string>
#include <strikeengine/models/physics/propulsion/ThrustCurve.hpp>

namespace StrikeEngine::Kernel {

    struct StageConfig {
        std::vector<Models::ThrustDataPoint> thrustCurve;  // empty => inactive/coast stage
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
            if (stage.thrustCurve.empty()) {
                if (stage.propellantMassKg != 0.0 || stage.dryMassKg != 0.0) {
                    return fail("an inactive stage cannot declare propellant or dry mass");
                }
                continue;
            }
            std::string curveError;
            if (!Models::ThrustCurve(stage.thrustCurve).validate(&curveError)) {
                return fail(curveError);
            }
            if (!std::isfinite(stage.vacuumIsp) || stage.vacuumIsp <= 0.0 ||
                !std::isfinite(stage.seaLevelIsp) || stage.seaLevelIsp <= 0.0) {
                return fail("vacuum and sea-level Isp must be finite and positive");
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
        }
        return true;
    }

} // namespace StrikeEngine::Kernel
