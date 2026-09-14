#pragma once

#include <vector>
#include <cstddef>
#include <string>

namespace StrikeEngine::Kernel {

    enum class SeekerType {
        None,
        RF,       // Monostatic radar
        IR,       // Infrared
        PassiveRF, // Homes on the target's own emitter (EIRP)
        SARH      // Semi-active radar homing (bistatic, off-board illuminator)
    };

    // Why a seeker is not locked on a given target, for diagnostics/UI.
    enum class SeekerRejectReason : int {
        None = 0,
        NoSeeker,
        NoTarget,
        Friendly,
        Geometry,
        RangeGate,
        VelocityGate,
        Terrain,
        Signal,
        NoProfile
    };

    struct SeekerBlock {
        // Configuration
        std::vector<SeekerType> type;
        
        // RF specific params
        std::vector<double> transmitterPowerW;
        std::vector<double> antennaGainDb;
        std::vector<double> wavelengthM;
        std::vector<double> noiseFloorW;
        std::vector<double> snrThresholdDb;

        // IR specific params
        std::vector<double> sensitivityW;
        std::vector<int> wavelengthBand; // To pass to atmosphere
        std::vector<double> irExtinctionPerM; // Beer-Lambert extinction (m^-1)

        // SARH illuminator (static configured position)
        std::vector<double> illuminatorPx;
        std::vector<double> illuminatorPy;
        std::vector<double> illuminatorPz;
        std::vector<double> illuminatorPowerW;
        std::vector<double> illuminatorGainDb;
        std::vector<double> illuminatorWavelengthM;

        // Geometry and tracking configuration. Angular limits are half-angles
        // in the seeker body frame; gimbal limits are independent azimuth and
        // elevation mechanical stops.
        std::vector<double> fieldOfViewHalfAngleRad;
        std::vector<double> gimbalAzimuthLimitRad;
        std::vector<double> gimbalElevationLimitRad;
        std::vector<double> lockHysteresisDb;
        std::vector<double> lockDropoutTimeSec;
        std::vector<double> measurementLatencySec;

        // --- Measurement-noise fidelity (default = legacy exact truth) ------
        // When measurementNoiseEnabled, committed measurements get angle,
        // range and range-rate noise, plus optional angular glint. All draws
        // come from the seeker stream and only happen when enabled.
        std::vector<bool> measurementNoiseEnabled;
        std::vector<double> angleNoiseStdDevRad;      // at angleNoiseRefSnrDb
        std::vector<double> angleNoiseRefSnrDb;       // sigma scales 10^-((snr-ref)/20)
        std::vector<double> rangeNoiseStdDevM;
        std::vector<double> rangeRateNoiseStdDevMps;
        std::vector<double> glintSigmaM;              // 0 = no glint
        std::vector<double> glintCorrelationTauSec;   // glint Markov time constant
        std::vector<bool> swerlingEnabled;            // RCS fluctuation (RF/SARH)

        // --- Gimbal servo (0 rad/s = legacy instantaneous look) -------------
        std::vector<double> gimbalRateLimitRadPerSec;

        // --- Range gates and terrain masking (0/false = legacy) -------------
        std::vector<double> minRangeGateM;
        std::vector<double> maxRangeGateM;
        std::vector<bool> terrainMaskingEnabled;
        // Minimum closing rate (m/s); a target that is not closing at least
        // this fast is rejected. 0 = disabled (legacy).
        std::vector<double> minClosingRateMps;

        // --- Estimator / countermeasure / emitter options -------------------
        std::vector<double> rateFilterTauSec;         // LOS-rate filter (def 0.05)
        std::vector<double> decoyRejectionDb;         // Chaff/Flare apparent-signal cut
        std::vector<double> passiveRfDutyCycle;       // 1.0 = continuous emitter
        std::vector<int> illuminatorEntityId;         // SARH live illuminator (-1 static)

        // Output tracking state
        std::vector<bool> isLocked;
        std::vector<std::size_t> lockedTargetId;

        std::vector<double> targetRange;
        std::vector<double> targetRangeRate;
        std::vector<double> targetAzimuth;
        std::vector<double> targetElevation;
        std::vector<double> targetAzimuthRate;
        std::vector<double> targetElevationRate;
        std::vector<double> previousAzimuth;
        std::vector<double> previousElevation;
        std::vector<double> lockLostTimeSec;
        std::vector<bool> hasPreviousLos;

        // Internal tracking flag (independent of measurement latency) and the
        // publication/consistency diagnostics: isLocked = lockActive &&
        // hasPublishedMeasurement, so a latency-delayed lock is not reported
        // until its first measurement actually reaches the consumers.
        std::vector<bool> lockActive;
        std::vector<bool> hasPublishedMeasurement;
        std::vector<double> measurementAgeSec;

        // Gimbal servo state (body-frame look angles, radians).
        std::vector<double> gimbalAzimuthRad;
        std::vector<double> gimbalElevationRad;

        // Diagnostics: last evaluated signal (dB for RF family, W for IR),
        // reject reason, and correlated glint state (metres, 2 axes).
        std::vector<double> lastSignalStrength;
        std::vector<int> lockRejectReason;
        std::vector<double> glintAzM;
        std::vector<double> glintElM;

        std::size_t size = 0;
    };

} // namespace StrikeEngine::Kernel
