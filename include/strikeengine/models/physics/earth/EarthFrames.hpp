#pragma once

#include <array>
#include <strikeengine/models/physics/earth/EarthModel.hpp>

namespace StrikeEngine::Models {

    namespace EarthFrames {

        using Vector3 = std::array<double, 3>;
        // Row-major direction-cosine matrix.
        using Matrix3 = std::array<double, 9>;

        inline Vector3 multiply(const Matrix3& matrix, const Vector3& vector)
        {
            return {
                matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
                matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
                matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2]};
        }

        inline Vector3 add(const Vector3& a, const Vector3& b)
        {
            return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
        }

        inline Vector3 subtract(const Vector3& a, const Vector3& b)
        {
            return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
        }

        inline Vector3 toVector(const EcefCoordinate& ecef)
        {
            return {ecef.x, ecef.y, ecef.z};
        }

        inline EcefCoordinate toEcef(const Vector3& vector)
        {
            return {vector[0], vector[1], vector[2]};
        }

        /**
         * @brief Direction cosine matrix from ECEF to local ENU.
         */
        inline Matrix3 ecefToEnuRotation(
            double latitudeRad, double longitudeRad)
        {
            const double sinLatitude = std::sin(latitudeRad);
            const double cosLatitude = std::cos(latitudeRad);
            const double sinLongitude = std::sin(longitudeRad);
            const double cosLongitude = std::cos(longitudeRad);
            return {
                -sinLongitude, cosLongitude, 0.0,
                -sinLatitude * cosLongitude, -sinLatitude * sinLongitude, cosLatitude,
                cosLatitude * cosLongitude, cosLatitude * sinLongitude, sinLatitude};
        }

        /**
         * @brief Direction cosine matrix from ECEF to local NED.
         */
        inline Matrix3 ecefToNedRotation(
            double latitudeRad, double longitudeRad)
        {
            const double sinLatitude = std::sin(latitudeRad);
            const double cosLatitude = std::cos(latitudeRad);
            const double sinLongitude = std::sin(longitudeRad);
            const double cosLongitude = std::cos(longitudeRad);
            return {
                -sinLatitude * cosLongitude, -sinLatitude * sinLongitude, cosLatitude,
                -sinLongitude, cosLongitude, 0.0,
                -cosLatitude * cosLongitude, -cosLatitude * sinLongitude, -sinLatitude};
        }

        inline Vector3 ecefToEnu(
            const EcefCoordinate& ecef,
            const GeodeticCoordinate& origin)
        {
            const auto originEcef = toVector(geodeticToEcef(origin));
            return multiply(
                ecefToEnuRotation(origin.latitudeRad, origin.longitudeRad),
                subtract(toVector(ecef), originEcef));
        }

        inline EcefCoordinate enuToEcef(
            const Vector3& enu,
            const GeodeticCoordinate& origin)
        {
            const auto rotation = ecefToEnuRotation(
                origin.latitudeRad, origin.longitudeRad);
            // The inverse of an orthonormal DCM is its transpose.
            const Vector3 delta{
                rotation[0] * enu[0] + rotation[3] * enu[1] + rotation[6] * enu[2],
                rotation[1] * enu[0] + rotation[4] * enu[1] + rotation[7] * enu[2],
                rotation[2] * enu[0] + rotation[5] * enu[1] + rotation[8] * enu[2]};
            return toEcef(add(toVector(geodeticToEcef(origin)), delta));
        }

        /**
         * @brief Resolve a local ENU displacement into WGS84 geodetic state.
         */
        inline GeodeticCoordinate enuToGeodetic(
            const Vector3& enu,
            const GeodeticCoordinate& origin)
        {
            return ecefToGeodetic(enuToEcef(enu, origin));
        }

        inline Vector3 enuToNed(const Vector3& enu)
        {
            return {enu[1], enu[0], -enu[2]};
        }

        inline Vector3 nedToEnu(const Vector3& ned)
        {
            return {ned[1], ned[0], -ned[2]};
        }

        /**
         * @brief Centrifugal acceleration expressed in the local ENU frame.
         *
         * This is -Omega x (Omega x r) for the WGS84/ECEF position. It is
         * separate from normal gravity so callers can choose the desired
         * physical convention explicitly.
         */
        inline Vector3 localCentrifugalAcceleration(
            const GeodeticCoordinate& position)
        {
            const auto ecef = geodeticToEcef(position);
            const double omegaSquared =
                EarthModel::earthRotationRateRadPerSec *
                EarthModel::earthRotationRateRadPerSec;
            const Vector3 centrifugalEcef{
                omegaSquared * ecef.x,
                omegaSquared * ecef.y,
                0.0};
            return multiply(
                ecefToEnuRotation(position.latitudeRad, position.longitudeRad),
                centrifugalEcef);
        }

        /**
         * @brief Point-mass gravity expressed in the local ENU frame.
         */
        inline Vector3 localSphericalGravityAcceleration(
            const GeodeticCoordinate& position)
        {
            const auto gravityEcef = toVector(
                sphericalGravityAccelerationEcef(geodeticToEcef(position)));
            return multiply(
                ecefToEnuRotation(position.latitudeRad, position.longitudeRad),
                gravityEcef);
        }

    } // namespace EarthFrames

} // namespace StrikeEngine::Models
