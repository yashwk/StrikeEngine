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

    /**
     * @brief WGS84 geodetic position with velocity expressed in local ENU.
     */
    struct GeodeticState {
        GeodeticCoordinate position{};
        std::array<double, 3> velocityEnu{0.0, 0.0, 0.0};
    };

    /**
     * @brief ECEF position with velocity expressed in ECEF.
     */
    struct EcefState {
        EcefCoordinate position{};
        EcefCoordinate velocity{};
    };

    namespace EarthModel {

        inline constexpr double semiMajorAxisM = 6378137.0;
        inline constexpr double flattening = 1.0 / 298.257223563;
        inline constexpr double semiMinorAxisM = semiMajorAxisM * (1.0 - flattening);
        inline constexpr double eccentricitySquared =
            flattening * (2.0 - flattening);
        inline constexpr double earthRotationRateRadPerSec = 7.2921150e-5;
        inline constexpr double standardGravitationalParameterM3PerSec2 =
            3.986004418e14;
        inline constexpr double secondZonalHarmonic = 1.08262668e-3;

        inline double primeVerticalRadiusM(double latitudeRad)
        {
            const double sinLatitude = std::sin(latitudeRad);
            return semiMajorAxisM / std::sqrt(
                1.0 - eccentricitySquared * sinLatitude * sinLatitude);
        }

        inline double meridionalRadiusM(double latitudeRad)
        {
            const double sinLatitude = std::sin(latitudeRad);
            const double denominator = std::pow(
                1.0 - eccentricitySquared * sinLatitude * sinLatitude, 1.5);
            return semiMajorAxisM * (1.0 - eccentricitySquared) / denominator;
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
         * @brief Convert a geodetic position and local ENU velocity to ECEF.
         *
         * The velocity is a coordinate-frame conversion only; transport and
         * Earth-rotation terms are not added implicitly.
         */
        inline EcefState geodeticToEcefState(const GeodeticState& geodetic)
        {
            const double latitude = geodetic.position.latitudeRad;
            const double longitude = geodetic.position.longitudeRad;
            const double sinLatitude = std::sin(latitude);
            const double cosLatitude = std::cos(latitude);
            const double sinLongitude = std::sin(longitude);
            const double cosLongitude = std::cos(longitude);
            const auto& enu = geodetic.velocityEnu;

            return {
                geodeticToEcef(geodetic.position),
                {
                    -sinLongitude * enu[0]
                        - sinLatitude * cosLongitude * enu[1]
                        + cosLatitude * cosLongitude * enu[2],
                    cosLongitude * enu[0]
                        - sinLatitude * sinLongitude * enu[1]
                        + cosLatitude * sinLongitude * enu[2],
                    cosLatitude * enu[1] + sinLatitude * enu[2]
                }
            };
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
         * @brief Convert an ECEF position and velocity to geodetic + ENU.
         *
         * The returned longitude follows atan2's [-pi, pi] convention and
         * the velocity is the local ENU velocity at the returned position.
         */
        inline GeodeticState ecefToGeodeticState(const EcefState& ecef)
        {
            const auto geodetic = ecefToGeodetic(ecef.position);
            const double latitude = geodetic.latitudeRad;
            const double longitude = geodetic.longitudeRad;
            const double sinLatitude = std::sin(latitude);
            const double cosLatitude = std::cos(latitude);
            const double sinLongitude = std::sin(longitude);
            const double cosLongitude = std::cos(longitude);
            const auto& velocity = ecef.velocity;

            return {
                geodetic,
                {
                    -sinLongitude * velocity.x + cosLongitude * velocity.y,
                    -sinLatitude * cosLongitude * velocity.x
                        - sinLatitude * sinLongitude * velocity.y
                        + cosLatitude * velocity.z,
                    cosLatitude * cosLongitude * velocity.x
                        + cosLatitude * sinLongitude * velocity.y
                        + sinLatitude * velocity.z
                }
            };
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
         * @brief Point-mass gravity magnitude at an ECEF radius.
         */
        inline double sphericalGravityMagnitude(double radiusM)
        {
            if (radiusM <= 0.0) return 0.0;
            return standardGravitationalParameterM3PerSec2 /
                (radiusM * radiusM);
        }

        /**
         * @brief Point-mass gravity vector in ECEF, directed toward Earth's
         * center.
         */
        inline EcefCoordinate sphericalGravityAccelerationEcef(
            const EcefCoordinate& position)
        {
            const double radius = std::hypot(
                std::hypot(position.x, position.y), position.z);
            if (radius <= 0.0) return {};
            const double scale = -standardGravitationalParameterM3PerSec2 /
                (radius * radius * radius);
            return {scale * position.x, scale * position.y, scale * position.z};
        }

        /**
         * @brief ECEF gravity using the central term plus the WGS84 J2 term.
         *
         * This is an axisymmetric zonal-harmonic upgrade over point-mass
         * gravity. It is intentionally separate from normal gravity: callers
         * can opt in when a position-dependent ECEF gravity vector is needed.
         */
        inline EcefCoordinate j2GravityAccelerationEcef(
            const EcefCoordinate& position)
        {
            const double radiusSquared = position.x * position.x
                + position.y * position.y + position.z * position.z;
            if (radiusSquared <= 0.0) return {};

            const double radius = std::sqrt(radiusSquared);
            const double zOverRadius = position.z / radius;
            const double radiusRatio = semiMajorAxisM / radius;
            const double j2Factor = 1.5 * secondZonalHarmonic
                * radiusRatio * radiusRatio;
            const double centralFactor =
                -standardGravitationalParameterM3PerSec2 / (radiusSquared * radius);

            return {
                centralFactor * position.x
                    * (1.0 - j2Factor * (5.0 * zOverRadius * zOverRadius - 1.0)),
                centralFactor * position.y
                    * (1.0 - j2Factor * (5.0 * zOverRadius * zOverRadius - 1.0)),
                centralFactor * position.z
                    * (1.0 - j2Factor * (5.0 * zOverRadius * zOverRadius - 3.0))
            };
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

        /**
         * @brief ENU transport angular rate for a moving local-level frame.
         *
         * The input velocity is expressed in ENU. This is the NED transport
         * rate, converted to ENU, using WGS84 curvature radii.
         */
        inline std::array<double, 3> localTransportRateEnu(
            const GeodeticCoordinate& position,
            const std::array<double, 3>& velocityEnu)
        {
            const double meridianRadius = meridionalRadiusM(position.latitudeRad);
            const double primeRadius = primeVerticalRadiusM(position.latitudeRad);
            const double height = position.altitudeM;
            // NED transport rate is [vE/(Re+h), -vN/(Rn+h),
            // -vE*tan(latitude)/(Re+h)]. Convert its [N,E,D] ordering to
            // ENU [E,N,U].
            const double eastRate = -velocityEnu[1] /
                (meridianRadius + height);
            const double northRate = velocityEnu[0] /
                (primeRadius + height);
            const double downRate = -velocityEnu[0] *
                std::tan(position.latitudeRad) /
                (primeRadius + height);
            return {eastRate, northRate, -downRate};
        }

        /**
         * @brief Acceleration contribution -omega_en x v in local ENU.
         */
        inline std::array<double, 3> localTransportAcceleration(
            const GeodeticCoordinate& position,
            const std::array<double, 3>& velocityEnu)
        {
            const auto rate = localTransportRateEnu(position, velocityEnu);
            return {
                -(rate[1] * velocityEnu[2] - rate[2] * velocityEnu[1]),
                -(rate[2] * velocityEnu[0] - rate[0] * velocityEnu[2]),
                -(rate[0] * velocityEnu[1] - rate[1] * velocityEnu[0])};
        }

    } // namespace EarthModel

    using EarthModel::ecefToGeodetic;
    using EarthModel::ecefToGeodeticState;
    using EarthModel::geodeticToEcef;
    using EarthModel::geodeticToEcefState;
    using EarthModel::j2GravityAccelerationEcef;
    using EarthModel::localCoriolisAcceleration;
    using EarthModel::localTransportAcceleration;
    using EarthModel::localTransportRateEnu;
    using EarthModel::sphericalGravityAccelerationEcef;
    using EarthModel::sphericalGravityMagnitude;
    using EarthModel::normalGravity;

} // namespace StrikeEngine::Models
