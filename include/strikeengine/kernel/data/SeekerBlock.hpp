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

        /**
         * @brief Grows every vector to @p n entries; see
         *        PhysicsBlock::ensureSize.
         *
         * This block is where a missed growth bit: bodyRateFiltered* and
         * prevBodyRate* were declared but never allocated by the kernel, so
         * their filters silently no-op'd behind a size guard.
         */
        void ensureSize(std::size_t n) {
            type.resize(n, SeekerType::None);
            transmitterPowerW.resize(n, 1000.0);
            antennaGainDb.resize(n, 30.0);
            wavelengthM.resize(n, 0.03);
            noiseFloorW.resize(n, 1e-12);
            snrThresholdDb.resize(n, 13.0);
            sensitivityW.resize(n, 1e-9);
            wavelengthBand.resize(n, 0);
            irExtinctionPerM.resize(n, 1e-4);
            illuminatorPx.resize(n, 0.0);
            illuminatorPy.resize(n, 0.0);
            illuminatorPz.resize(n, 0.0);
            illuminatorPowerW.resize(n, 5.0e5);
            illuminatorGainDb.resize(n, 38.0);
            illuminatorWavelengthM.resize(n, 0.03);
            fieldOfViewHalfAngleRad.resize(n, 1.0471975512);
            gimbalAzimuthLimitRad.resize(n, 1.0471975512);
            gimbalElevationLimitRad.resize(n, 1.0471975512);
            lockHysteresisDb.resize(n, 3.0);
            lockDropoutTimeSec.resize(n, 0.10);
            measurementLatencySec.resize(n, 0.0);
            measurementNoiseEnabled.resize(n, false);
            angleNoiseStdDevRad.resize(n, 0.001);
            angleNoiseRefSnrDb.resize(n, 20.0);
            rangeNoiseStdDevM.resize(n, 1.0);
            rangeRateNoiseStdDevMps.resize(n, 0.5);
            glintSigmaM.resize(n, 0.0);
            glintCorrelationTauSec.resize(n, 1.0);
            swerlingEnabled.resize(n, false);
            gimbalRateLimitRadPerSec.resize(n, 0.0);
            minRangeGateM.resize(n, 0.0);
            maxRangeGateM.resize(n, 0.0);
            terrainMaskingEnabled.resize(n, false);
            minClosingRateMps.resize(n, 0.0);
            rateFilterTauSec.resize(n, 0.05);
            decoyRejectionDb.resize(n, 0.0);
            passiveRfDutyCycle.resize(n, 1.0);
            illuminatorEntityId.resize(n, -1);
            isLocked.resize(n, false);
            lockedTargetId.resize(n, 0);
            targetRange.resize(n, 0.0);
            targetRangeRate.resize(n, 0.0);
            targetAzimuth.resize(n, 0.0);
            targetElevation.resize(n, 0.0);
            targetAzimuthRate.resize(n, 0.0);
            targetElevationRate.resize(n, 0.0);
            losRateFilterAz.resize(n, 0.0);
            losRateFilterEl.resize(n, 0.0);
            losRateWorldX.resize(n, 0.0);
            losRateWorldY.resize(n, 0.0);
            losRateWorldZ.resize(n, 0.0);
            losRateWorldValid.resize(n, false);
            losRateWorldStateX.resize(n, 0.0);
            losRateWorldStateY.resize(n, 0.0);
            losRateWorldStateZ.resize(n, 0.0);
            prevLosWorldX.resize(n, 0.0);
            prevLosWorldY.resize(n, 0.0);
            prevLosWorldZ.resize(n, 0.0);
            prevLosWorldTimeSec.resize(n, 0.0);
            bodyRateFilteredX.resize(n, 0.0);
            bodyRateFilteredY.resize(n, 0.0);
            bodyRateFilteredZ.resize(n, 0.0);
            bodyRateStateX.resize(n, 0.0);
            bodyRateStateY.resize(n, 0.0);
            bodyRateStateZ.resize(n, 0.0);
            prevBodyRateX.resize(n, 0.0);
            prevBodyRateY.resize(n, 0.0);
            prevBodyRateZ.resize(n, 0.0);
            gyroIntX.resize(n, 0.0);
            gyroIntY.resize(n, 0.0);
            gyroIntZ.resize(n, 0.0);
            prevStepGyroX.resize(n, 0.0);
            prevStepGyroY.resize(n, 0.0);
            prevStepGyroZ.resize(n, 0.0);
            previousAzimuth.resize(n, 0.0);
            previousElevation.resize(n, 0.0);
            timeSinceCommitSec.resize(n, 0.0);
            seekerClockSec.resize(n, 0.0);
            lockLostTimeSec.resize(n, 0.0);
            hasPreviousLos.resize(n, false);
            lockActive.resize(n, false);
            hasPublishedMeasurement.resize(n, false);
            measurementAgeSec.resize(n, 0.0);
            gimbalAzimuthRad.resize(n, 0.0);
            gimbalElevationRad.resize(n, 0.0);
            lastSignalStrength.resize(n, 0.0);
            lockRejectReason.resize(n, static_cast<int>(SeekerRejectReason::None));
            glintAzM.resize(n, 0.0);
            glintElM.resize(n, 0.0);
            size = n;
        }
    };

} // namespace StrikeEngine::Kernel
