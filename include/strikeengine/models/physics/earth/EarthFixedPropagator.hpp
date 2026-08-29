#pragma once

#include <cstddef>
#include <strikeengine/models/physics/earth/EarthModel.hpp>

namespace StrikeEngine::Models {

    /**
     * @brief Terms included by the rotating-Earth ECEF equations.
     */
    struct EcefDynamicsOptions {
        bool includeGravity = true;
        bool includeCoriolis = true;
        bool includeCentrifugal = true;
        bool includeJ2Gravity = false;
    };

    namespace EarthFixed {

        inline EcefCoordinate add(
            const EcefCoordinate& a, const EcefCoordinate& b)
        {
            return {a.x + b.x, a.y + b.y, a.z + b.z};
        }

        inline EcefCoordinate scale(const EcefCoordinate& value, double factor)
        {
            return {value.x * factor, value.y * factor, value.z * factor};
        }

        inline EcefCoordinate cross(
            const EcefCoordinate& a, const EcefCoordinate& b)
        {
            return {a.y * b.z - a.z * b.y,
                    a.z * b.x - a.x * b.z,
                    a.x * b.y - a.y * b.x};
        }

        inline EcefCoordinate earthRotationVector()
        {
            return {0.0, 0.0, EarthModel::earthRotationRateRadPerSec};
        }

        /**
         * @brief Acceleration in the rotating ECEF frame.
         *
         * The equation is a = a_force + a_gravity - 2*Omega x v -
         * Omega x (Omega x r). The optional force term is already expressed
         * as ECEF acceleration, allowing callers to supply thrust or drag.
         */
        inline EcefCoordinate acceleration(
            const EcefCoordinate& position,
            const EcefCoordinate& velocity,
            const EcefCoordinate& externalAcceleration = {},
            EcefDynamicsOptions options = {})
        {
            EcefCoordinate result = externalAcceleration;
            const auto omega = earthRotationVector();
            if (options.includeGravity) {
                result = add(result, options.includeJ2Gravity
                    ? j2GravityAccelerationEcef(position)
                    : sphericalGravityAccelerationEcef(position));
            }
            if (options.includeCoriolis) {
                result = add(result, scale(cross(omega, velocity), -2.0));
            }
            if (options.includeCentrifugal) {
                result = add(result, scale(
                    cross(omega, cross(omega, position)), -1.0));
            }
            return result;
        }

        struct Derivative {
            EcefCoordinate positionRate{};
            EcefCoordinate velocityRate{};
        };

        inline Derivative derivative(
            const EcefState& state,
            const EcefCoordinate& externalAcceleration,
            EcefDynamicsOptions options)
        {
            return {
                state.velocity,
                acceleration(state.position, state.velocity,
                             externalAcceleration, options)};
        }

        /**
         * @brief Propagate a rotating-Earth ECEF state with fixed-step RK4.
         *
         * Substeps provide a deterministic accuracy control without adding
         * adaptive-step state to this stateless model API.
         */
        inline EcefState propagateEcef(
            EcefState state,
            double dt,
            EcefDynamicsOptions options = {},
            EcefCoordinate externalAcceleration = {},
            std::size_t substeps = 1)
        {
            if (substeps == 0) substeps = 1;
            const double h = dt / static_cast<double>(substeps);
            for (std::size_t step = 0; step < substeps; ++step) {
                const auto k1 = derivative(state, externalAcceleration, options);
                const EcefState midpoint1{
                    add(state.position, scale(k1.positionRate, 0.5 * h)),
                    add(state.velocity, scale(k1.velocityRate, 0.5 * h))};
                const auto k2 = derivative(midpoint1, externalAcceleration, options);
                const EcefState midpoint2{
                    add(state.position, scale(k2.positionRate, 0.5 * h)),
                    add(state.velocity, scale(k2.velocityRate, 0.5 * h))};
                const auto k3 = derivative(midpoint2, externalAcceleration, options);
                const EcefState endpoint{
                    add(state.position, scale(k3.positionRate, h)),
                    add(state.velocity, scale(k3.velocityRate, h))};
                const auto k4 = derivative(endpoint, externalAcceleration, options);

                state.position = add(state.position, scale(add(
                    add(k1.positionRate, scale(k2.positionRate, 2.0)),
                    add(scale(k3.positionRate, 2.0), k4.positionRate)), h / 6.0));
                state.velocity = add(state.velocity, scale(add(
                    add(k1.velocityRate, scale(k2.velocityRate, 2.0)),
                    add(scale(k3.velocityRate, 2.0), k4.velocityRate)), h / 6.0));
            }
            return state;
        }

    } // namespace EarthFixed

    using EarthFixed::acceleration;
    using EarthFixed::propagateEcef;

} // namespace StrikeEngine::Models
