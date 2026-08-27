#pragma once

#include <array>
#include <functional>

namespace StrikeEngine::Kernel {

    /**
     * @brief Optional local-earth corrections for the truth model.
     *
     * In local mode, world X/Y/Z are interpreted as ENU (east, north, up)
     * for earth corrections. With useEcefTruth, PhysicsBlock position and
     * velocity are absolute ECEF values and the reference latitude/longitude
     * anchor terrain callbacks in a local ENU view. The default remains the
     * legacy flat-earth constant-gravity model.
     */
    struct EarthEnvironmentConfig {
        // When enabled, PhysicsBlock position/velocity are absolute ECEF
        // metres and m/s. The default remains local ENU-style coordinates.
        bool useEcefTruth = false;
        bool useWgs84Gravity = false;
        bool useSphericalGravity = false;
        bool includeCoriolis = false;
        bool includeCentrifugal = false;
        bool includeTransportRate = false;
        // In ECEF truth mode, model the gyro as measuring the INERTIAL body
        // rate (earth rotation included) and compensate it in the IMU/INS.
        // Ignored in local flat-earth mode. Default false preserves behavior.
        bool includeEarthRateGyro = false;
        double referenceLatitudeRad = 0.0;
        double referenceLongitudeRad = 0.0;
    };

    /**
     * @brief Flat-earth environment hooks used by the CPU truth model.
     *
     * Terrain elevation is expressed in world metres above the reference
     * datum. Wind is the world-frame air-mass velocity at the queried position
     * and time; the aerodynamic model uses vehicle velocity relative to this
     * air mass. Default callbacks describe the existing zero-wind, flat-ground
     * environment.
     */
    struct EnvironmentConfig {
        EarthEnvironmentConfig earth{};

        std::function<double(double x, double y)> terrainElevation =
            [](double, double) { return 0.0; };

        std::function<std::array<double, 3>(double x, double y, double z, double time)>
            windVelocity = [](double, double, double, double) {
                return std::array<double, 3>{0.0, 0.0, 0.0};
            };
    };

} // namespace StrikeEngine::Kernel
