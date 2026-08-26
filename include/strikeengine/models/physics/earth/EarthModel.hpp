#pragma once

#include <array>
#include <cmath>
#include <limits>

namespace StrikeEngine::Models {

    /**
     * @brief WGS84 geodetic position (radians, radians, metres).
     */
    struct GeodeticCoordinate {
        double latitudeRad = 0.0;
        double longitudeRad = 0.0;
        double altitudeM = 0.0;
    };

    /**
     * @brief Earth-centred, Earth-fixed Cartesian position in metres.
     */
    struct EcefCoordinate {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    namespace EarthModel {

        inline constexpr double semiMajorAxisM = 6378137.0;
        inline constexpr double flattening = 1.0 / 298.257223563;
        inline constexpr double semiMinorAxisM = semiMajorAxisM * (1.0 - flattening);
        inline constexpr double eccentricitySquared =
            flattening * (2.0 - flattening);
        inline constexpr double earthRotationRateRadPerSec = 7.2921150e-5;

        inline double primeVerticalRadiusM(double latitudeRad)
        {
            const double sinLatitude = std::sin(latitudeRad);
            return semiMajorAxisM / std::sqrt(
                1.0 - eccentricitySquared * sinLatitude * sinLatitude);
        }

        /**
         * @brief Convert WGS84 geodetic coordinates to ECEF.
         */
        inline EcefCoordinate geodeticToEcef(const GeodeticCoordinate& geodetic)
        {
            const double sinLatitude = std::sin(geodetic.latitudeRad);
            const double cosLatitude = std::cos(geodetic.latitudeRad);
            const double sinLongitude = std::sin(geodetic.longitudeRad);
            const double cosLongitude = std::cos(geodetic.longitudeRad);
            const double radius = primeVerticalRadiusM(geodetic.latitudeRad);

            return {
                (radius + geodetic.altitudeM) * cosLatitude * cosLongitude,
                (radius + geodetic.altitudeM) * cosLatitude * sinLongitude,
                (radius * (1.0 - eccentricitySquared) + geodetic.altitudeM)
                    * sinLatitude};
        }

        /**
         * @brief Convert ECEF coordinates to WGS84 geodetic coordinates.
         *
         * The fixed-point iteration converges rapidly for ordinary vehicle
         * altitudes and includes explicit handling for the polar axis.
         */
        inline GeodeticCoordinate ecefToGeodetic(const EcefCoordinate& ecef)
        {
            const double horizontal = std::hypot(ecef.x, ecef.y);
            const double longitude = std::atan2(ecef.y, ecef.x);
            if (horizontal < std::numeric_limits<double>::epsilon()) {
                const double latitude = (ecef.z >= 0.0)
                    ? 0.5 * std::acos(-1.0)
                    : -0.5 * std::acos(-1.0);
                return {latitude, 0.0,
                        std::abs(ecef.z) - semiMinorAxisM};
            }

            double latitude = std::atan2(
                ecef.z,
                horizontal * (1.0 - eccentricitySquared));
            double altitude = 0.0;
            for (int iteration = 0; iteration < 12; ++iteration) {
                const double radius = primeVerticalRadiusM(latitude);
                const double cosLatitude = std::cos(latitude);
                const double nextAltitude = horizontal / cosLatitude - radius;
                const double nextLatitude = std::atan2(
                    ecef.z,
                    horizontal * (1.0 - eccentricitySquared *
                                  radius / (radius + nextAltitude)));
                altitude = nextAltitude;
                if (std::abs(nextLatitude - latitude) < 1e-13) {
                    latitude = nextLatitude;
                    break;
                }
                latitude = nextLatitude;
            }

            const double radius = primeVerticalRadiusM(latitude);
            altitude = horizontal / std::cos(latitude) - radius;
            return {latitude, longitude, altitude};
        }

        /**
         * @brief WGS84 normal gravity magnitude at geodetic latitude/altitude.
         *
         * This is the Somigliana normal-gravity formula with a second-order
         * free-air altitude correction, suitable for the local truth-model
         * earth-effects MVP.
         */
        inline double normalGravity(double latitudeRad, double altitudeM = 0.0)
        {
            constexpr double equatorialGravity = 9.7803253359;
            constexpr double polarGravity = 9.8321849378;
            const double sinLatitude = std::sin(latitudeRad);
            const double sinSquared = sinLatitude * sinLatitude;
            const double k =
                (semiMinorAxisM * polarGravity -
                 semiMajorAxisM * equatorialGravity) /
                (semiMajorAxisM * equatorialGravity);
            const double surfaceGravity = equatorialGravity *
                (1.0 + k * sinSquared) /
                std::sqrt(1.0 - eccentricitySquared * sinSquared);
            const double heightRatio = altitudeM / semiMajorAxisM;
            return surfaceGravity *
                (1.0 - 2.0 * heightRatio + 3.0 * heightRatio * heightRatio);
        }

        /**
         * @brief Coriolis acceleration in a local ENU frame.
         *
         * The local world convention is X=east, Y=north, Z=up. The returned
         * acceleration is -2*Omega x velocity; centrifugal acceleration and
         * transport-rate terms are intentionally outside this MVP.
         */
        inline std::array<double, 3> localCoriolisAcceleration(
            double latitudeRad,
            const std::array<double, 3>& velocityEnu)
        {
            const double omegaNorth =
                earthRotationRateRadPerSec * std::cos(latitudeRad);
            const double omegaUp =
                earthRotationRateRadPerSec * std::sin(latitudeRad);
            return {
                2.0 * (omegaUp * velocityEnu[1] -
                       omegaNorth * velocityEnu[2]),
                -2.0 * omegaUp * velocityEnu[0],
                2.0 * omegaNorth * velocityEnu[0]};
        }

    } // namespace EarthModel

    using EarthModel::ecefToGeodetic;
    using EarthModel::geodeticToEcef;
    using EarthModel::localCoriolisAcceleration;
    using EarthModel::normalGravity;

} // namespace StrikeEngine::Models
