#pragma once

#include <strikeengine/kernel/data/SeekerBlock.hpp>

namespace StrikeEngine::Kernel {

    /**
     * @brief Per-entity seeker configuration (defaults match the legacy
     * hardcoded createVehicle values, so existing scenarios are unchanged).
     *
     * `VehicleConfig::seeker` is copied into `SeekerBlock` per entity. The
     * SARH illuminator is a STATIC configured position; tracking
     * a dynamic illuminator entity is future work.
     */
    struct SeekerConfig {
        SeekerType type = SeekerType::None;

        // --- RF (monostatic radar) ---
        double transmitterPowerW = 1000.0;
        double antennaGainDb = 30.0;
        double wavelengthM = 0.03;   // X-band
        double noiseFloorW = 1e-12;
        double snrThresholdDb = 13.0;

        // --- IR ---
        double sensitivityW = 1e-9;
        int wavelengthBand = 0;      // To pass to atmosphere
        double irExtinctionPerM = 1e-4;  // 0.1 / km, matches legacy placeholder

        // --- SARH illuminator (static position) ---
        double illuminatorPx = 0.0;
        double illuminatorPy = 0.0;
        double illuminatorPz = 0.0;
        double illuminatorPowerW = 5.0e5;
        double illuminatorGainDb = 38.0;
        double illuminatorWavelengthM = 0.03;  // defaults to the seeker band

        // --- Geometry and tracking (half-angles, radians) ---
        double fieldOfViewHalfAngleRad = 1.0471975512;  // 60 deg
        double gimbalAzimuthLimitRad = 1.0471975512;
        double gimbalElevationLimitRad = 1.0471975512;
        double lockHysteresisDb = 3.0;
        double lockDropoutTimeSec = 0.10;
        double measurementLatencySec = 0.0;

        // --- Measurement-noise fidelity (off = legacy exact truth) ----------
        bool   measurementNoiseEnabled = false;
        double angleNoiseStdDevRad = 0.001;      // at angleNoiseRefSnrDb
        double angleNoiseRefSnrDb = 20.0;        // sigma scales 10^-((snr-ref)/20)
        double rangeNoiseStdDevM = 1.0;
        double rangeRateNoiseStdDevMps = 0.5;
        double glintSigmaM = 0.0;                // 0 = no glint
        double glintCorrelationTauSec = 1.0;
        bool   swerlingEnabled = false;          // RCS fluctuation (RF/SARH)

        // --- Gimbal servo (0 = legacy instantaneous look) -------------------
        double gimbalRateLimitRadPerSec = 0.0;

        // --- Range gates and terrain masking (0/false = legacy) -------------
        double minRangeGateM = 0.0;
        double maxRangeGateM = 0.0;
        bool   terrainMaskingEnabled = false;
        double minClosingRateMps = 0.0;          // 0 = disabled

        // --- Estimator / countermeasure / emitter options -------------------
        double rateFilterTauSec = 0.05;          // LOS-rate filter time constant
        double decoyRejectionDb = 0.0;           // Chaff/Flare apparent-signal cut
        double passiveRfDutyCycle = 1.0;         // 1.0 = continuous emitter
        int    illuminatorEntityId = -1;         // SARH live illuminator (-1 static)
    };

} // namespace StrikeEngine::Kernel
