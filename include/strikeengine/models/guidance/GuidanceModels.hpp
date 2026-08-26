#pragma once

#include <array>
#include <cstddef>
#include <cmath>

namespace StrikeEngine::Models {

    using Vec3 = std::array<double, 3>;

    struct GuidanceSolution {
        Vec3 acceleration{0.0, 0.0, 0.0};
        double closingSpeed = 0.0;
        bool valid = false;
    };

    inline double dot(const Vec3& a, const Vec3& b)
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    inline Vec3 cross(const Vec3& a, const Vec3& b)
    {
        return {a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]};
    }

    /**
     * @brief Classical true proportional navigation in a world frame.
     *
     * Relative vectors use target-minus-interceptor convention. The returned
     * acceleration is normal to the line of sight and has no gravity or
     * thrust feed-forward component; the autopilot supplies the airframe
     * response to this command.
     */
    inline GuidanceSolution proportionalNavigation(
        const Vec3& relativePosition,
        const Vec3& relativeVelocity,
        double navigationConstant = 3.0)
    {
        const double rangeSquared = dot(relativePosition, relativePosition);
        if (rangeSquared < 1e-12 || navigationConstant <= 0.0) {
            return {};
        }

        const double range = std::sqrt(rangeSquared);
        const Vec3 lineOfSight = {
            relativePosition[0] / range,
            relativePosition[1] / range,
            relativePosition[2] / range};
        const double closingSpeed = -dot(relativePosition, relativeVelocity) / range;
        if (closingSpeed <= 0.0) {
            return {Vec3{0.0, 0.0, 0.0}, closingSpeed, false};
        }

        const Vec3 losRate = {
            cross(relativePosition, relativeVelocity)[0] / rangeSquared,
            cross(relativePosition, relativeVelocity)[1] / rangeSquared,
            cross(relativePosition, relativeVelocity)[2] / rangeSquared};
        const Vec3 accelerationDirection = cross(losRate, lineOfSight);
        return {{navigationConstant * closingSpeed * accelerationDirection[0],
                 navigationConstant * closingSpeed * accelerationDirection[1],
                 navigationConstant * closingSpeed * accelerationDirection[2]},
                closingSpeed, true};
    }

    /**
     * @brief Augmented proportional navigation with target acceleration.
     *
     * Target acceleration is projected normal to the current line of sight
     * before the standard APN feed-forward term is added.
     */
    inline GuidanceSolution augmentedProportionalNavigation(
        const Vec3& relativePosition,
        const Vec3& relativeVelocity,
        const Vec3& targetAcceleration,
        double navigationConstant = 3.5)
    {
        GuidanceSolution solution = proportionalNavigation(
            relativePosition, relativeVelocity, navigationConstant);
        if (!solution.valid) return solution;

        const double range = std::sqrt(dot(relativePosition, relativePosition));
        const Vec3 lineOfSight = {
            relativePosition[0] / range,
            relativePosition[1] / range,
            relativePosition[2] / range};
        const double alongLos = dot(targetAcceleration, lineOfSight);
        const Vec3 normalTargetAcceleration = {
            targetAcceleration[0] - alongLos * lineOfSight[0],
            targetAcceleration[1] - alongLos * lineOfSight[1],
            targetAcceleration[2] - alongLos * lineOfSight[2]};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            solution.acceleration[axis] +=
                0.5 * navigationConstant * normalTargetAcceleration[axis];
        }
        return solution;
    }

} // namespace StrikeEngine::Models
