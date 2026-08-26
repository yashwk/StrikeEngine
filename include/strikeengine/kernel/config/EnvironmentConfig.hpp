#pragma once

#include <array>
#include <functional>

namespace StrikeEngine::Kernel {

    /**
     * @brief Optional local-earth corrections for the truth model.
     *
     * When enabled, the existing local world axes are interpreted as ENU
     * (east, north, up) for Coriolis. The default remains the legacy
     * flat-earth constant-gravity model.
     */
    struct EarthEnvironmentConfig {
        bool useWgs84Gravity = false;
        bool useSphericalGravity = false;
        bool includeCoriolis = false;
        bool includeCentrifugal = false;
        bool includeTransportRate = false;
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
