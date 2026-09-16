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
        // Published (latency-delayed) LOS rates. These are OUTPUTS: they are
        // overwritten from the latency queue every publish, so they must never
        // double as filter state.
        std::vector<double> targetAzimuthRate;
        std::vector<double> targetElevationRate;
        // Internal rate-filter state, kept separate from the published values.
        // Aliasing the two made the first-order filter restart from a stale
        // delayed sample each step whenever measurementLatencySec > 0, so the
        // published rate came out several times too small -- a wrong terminal
        // demand, worst exactly in the long-range dive.
        std::vector<double> losRateFilterAz;
        std::vector<double> losRateFilterEl;
        // Inertial (world-frame) LOS rate, reconstructed geometrically from
        // consecutive world-frame LOS vectors. Published for the guidance,
        // which prefers it over the body-frame rate + gyro pairing: that
        // pairing subtracts two large near-equal terms and any latency, filter
        // or skipped-measurement mismatch leaves a residual bigger than the
        // LOS rate itself.
        std::vector<double> losRateWorldX;
        std::vector<double> losRateWorldY;
        std::vector<double> losRateWorldZ;
        std::vector<bool> losRateWorldValid;
        std::vector<double> losRateWorldStateX;   // filter state (internal)
        std::vector<double> losRateWorldStateY;
        std::vector<double> losRateWorldStateZ;
        std::vector<double> prevLosWorldX;        // last published world LOS
        std::vector<double> prevLosWorldY;
        std::vector<double> prevLosWorldZ;
        std::vector<double> prevLosWorldTimeSec;
        // Body rate paired with the az/el backward difference: averaged over
        // the same step and filtered with the same blend, so the gyro
        // decoupling does not leave a half-step/filter residual during host
        // oscillation. These are the PUBLISHED (latency-delayed) values the
        // guidance reads; bodyRateState* below is the live filter state.
        std::vector<double> bodyRateFilteredX;
        std::vector<double> bodyRateFilteredY;
        std::vector<double> bodyRateFilteredZ;
        std::vector<double> bodyRateStateX;
        std::vector<double> bodyRateStateY;
        std::vector<double> bodyRateStateZ;
        std::vector<double> prevBodyRateX;
        std::vector<double> prevBodyRateY;
        std::vector<double> prevBodyRateZ;
        // Integrated body angle over the interval since the last committed
        // measurement (trapezoidal, accumulated every step). The gyro term of
        // the LOS-rate decoupling must be the integral over the SAME interval
        // as the angle difference: an endpoint average of two samples is off
        // by the change in rate across a latency-length gap, which is larger
        // than the LOS rate itself.
        std::vector<double> gyroIntX;
        std::vector<double> gyroIntY;
        std::vector<double> gyroIntZ;
        std::vector<double> prevStepGyroX;
        std::vector<double> prevStepGyroY;
        std::vector<double> prevStepGyroZ;
        std::vector<double> previousAzimuth;
        std::vector<double> previousElevation;
        // Time since the previous COMMITTED measurement. The backward
        // difference that feeds the rate filter must divide by the real
        // elapsed time: when a maintenance check is skipped (SNR at the
        // threshold, exactly the long-range terminal case) the nominal step
        // made the rate several times too large.
        std::vector<double> timeSinceCommitSec;
        // Free-running per-seeker clock (s): stamps latency-queue entries so
        // the inertial LOS rate divides by the exact measurement interval.
        std::vector<double> seekerClockSec;
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
