#pragma once

#include <strikeengine/kernel/data/SeekerBlock.hpp>

namespace StrikeEngine::Kernel {

    /**
     * @brief Per-entity seeker configuration (defaults match the legacy
     * hardcoded createVehicle values, so existing scenarios are unchanged).
     *
     * `VehicleConfig::seeker` is copied into `SeekerBlock` per entity. The
     * SARH illuminator is a STATIC configured position for this MVP; tracking
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
    };

} // namespace StrikeEngine::Kernel
