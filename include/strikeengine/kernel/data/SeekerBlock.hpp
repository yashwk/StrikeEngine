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

        // SARH illuminator (static configured position for this MVP)
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

        std::size_t size = 0;
    };

} // namespace StrikeEngine::Kernel
