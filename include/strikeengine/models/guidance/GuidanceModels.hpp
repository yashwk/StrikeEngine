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

    /**
     * @brief W40 constant-velocity intercept prediction (trajectory core).
     *
     * Solves for the earliest positive-time intercept of an interceptor
     * flying at constant speed toward a (optionally constant-acceleration)
     * target, returns the predicted intercept point (PIP), time-to-go, the
     * required acceleration (PN demand magnitude aimed at the PIP), and a
     * status. All inputs are world-frame; the caller supplies navigation
     * estimates and a command/track aim state (guidance never reads physics
     * truth). Deterministic and non-finite-safe.
     *
     * Geometry: r = targetPos - interceptorPos, v = targetVel - interceptorVel.
     * Constant-speed intercept satisfies ||r + v*t||^2 = |Vi|^2 * t^2, a
     * quadratic in t; the smallest positive root is tgo. PIP = T + Vt*tgo
     * (+ 0.5*At*tgo^2 when targetAccelAvailable). requiredAccel is the PN
     * demand toward the PIP evaluated at the intercept-time closing velocity.
     */
    enum class InterceptStatus : uint8_t {
        Ok,          // valid predicted intercept (out.valid == true)
        VelocityLow, // interceptor est speed below minSpeedMps
        NoIntercept, // no positive-time constant-velocity intercept (or no
                     // valid closing PN toward the PIP)
        NonFinite    // non-finite input / non-positive navigation constant
    };

    struct InterceptResult {
        Vec3 pip{0.0, 0.0, 0.0};
        double tgoSec = 0.0;
        double requiredAccel = 0.0; // |PN demand| toward the PIP (m/s^2)
        InterceptStatus status = InterceptStatus::NoIntercept;
        bool valid = false;         // true only when status == Ok
    };

    inline Vec3 vec3Scale(const Vec3& v, double s)
    {
        return {v[0] * s, v[1] * s, v[2] * s};
    }

    inline Vec3 vec3Add(const Vec3& a, const Vec3& b)
    {
        return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    inline Vec3 vec3Sub(const Vec3& a, const Vec3& b)
    {
        return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    inline InterceptResult predictIntercept(
        const Vec3& interceptorPos,
        const Vec3& interceptorVel,
        const Vec3& targetPos,
        const Vec3& targetVel,
        const Vec3& targetAccel,
        bool targetAccelAvailable,
        double navigationConstant,
        double minSpeedMps)
    {
        InterceptResult out;

        auto finite3 = [](const Vec3& v) {
            return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
        };

        if (!finite3(interceptorPos) || !finite3(interceptorVel) ||
            !finite3(targetPos) || !finite3(targetVel) ||
            (targetAccelAvailable && !finite3(targetAccel)) ||
            !std::isfinite(navigationConstant) || navigationConstant <= 0.0 ||
            !std::isfinite(minSpeedMps) || minSpeedMps < 0.0)
        {
            out.status = InterceptStatus::NonFinite;
            return out;
        }

        const Vec3 r = vec3Sub(targetPos, interceptorPos);
        const Vec3 v = vec3Sub(targetVel, interceptorVel);
        if (!finite3(r) || !finite3(v)) {
            out.status = InterceptStatus::NonFinite;
            return out;
        }

        const double vi2 = dot(interceptorVel, interceptorVel);
        const double vi = std::sqrt(vi2);
        if (vi < minSpeedMps) {
            out.status = InterceptStatus::VelocityLow;
            return out;
        }

        // Quadratic: (|v|^2 - |Vi|^2) t^2 + 2 (r.v) t + |r|^2 = 0
        const double a = dot(v, v) - vi2;
        const double b = 2.0 * dot(r, v);
        const double c = dot(r, r);
        double tStar = -1.0;
        if (std::abs(a) < 1e-12) {
            // Degenerate |v| ~= |Vi|: linear  b t + c = 0 requires b < 0.
            if (b < -1e-12) tStar = -c / b;
        } else {
            const double disc = b * b - 4.0 * a * c;
            if (disc >= 0.0 && std::isfinite(disc)) {
                const double sq = std::sqrt(disc);
                const double t1 = (-b - sq) / (2.0 * a);
                const double t2 = (-b + sq) / (2.0 * a);
                if (t1 > 0.0) tStar = t1;
                if (t2 > 0.0 && (tStar < 0.0 || t2 < tStar)) tStar = t2;
            }
        }
        if (!(tStar > 0.0) || !std::isfinite(tStar)) {
            out.status = InterceptStatus::NoIntercept;
            return out;
        }

        const Vec3 at = targetAccelAvailable ? targetAccel : Vec3{0.0, 0.0, 0.0};
        const Vec3 pip = {
            targetPos[0] + targetVel[0] * tStar + 0.5 * at[0] * tStar * tStar,
            targetPos[1] + targetVel[1] * tStar + 0.5 * at[1] * tStar * tStar,
            targetPos[2] + targetVel[2] * tStar + 0.5 * at[2] * tStar * tStar};
        const Vec3 vClose = {
            (targetVel[0] + at[0] * tStar) - interceptorVel[0],
            (targetVel[1] + at[1] * tStar) - interceptorVel[1],
            (targetVel[2] + at[2] * tStar) - interceptorVel[2]};

        const GuidanceSolution sol = proportionalNavigation(
            vec3Sub(pip, interceptorPos), vClose, navigationConstant);
        if (!sol.valid) {
            out.status = InterceptStatus::NoIntercept;
            return out;
        }

        out.pip = pip;
        out.tgoSec = tStar;
        out.requiredAccel = std::sqrt(dot(sol.acceleration, sol.acceleration));
        out.status = InterceptStatus::Ok;
        out.valid = true;
        return out;
    }

} // namespace StrikeEngine::Models
