#pragma once

#include <cmath>

namespace StrikeEngine::Kernel {

    /**
     * Rotates a vector from body to world frame: v' = q * v * q^-1.
     * q = (qw, qx, qy, qz) is assumed unit length.
     */
    inline void quatRotateToWorld(
        double qw, double qx, double qy, double qz,
        double vx, double vy, double vz,
        double& rx, double& ry, double& rz)
    {
        // t = 2 * q_vec x v
        const double tx = 2.0 * (qy * vz - qz * vy);
        const double ty = 2.0 * (qz * vx - qx * vz);
        const double tz = 2.0 * (qx * vy - qy * vx);
        // v' = v + w*t + q_vec x t
        rx = vx + qw * tx + (qy * tz - qz * ty);
        ry = vy + qw * ty + (qz * tx - qx * tz);
        rz = vz + qw * tz + (qx * ty - qy * tx);
    }

    /**
     * Unit quaternion for a rotation of `angle` about a unit axis.
     */
    inline void quatFromAxisAngle(
        double ax, double ay, double az, double angle,
        double& qw, double& qx, double& qy, double& qz)
    {
        const double half = 0.5 * angle;
        const double s = std::sin(half);
        qw = std::cos(half);
        qx = ax * s;
        qy = ay * s;
        qz = az * s;
    }

    /**
     * Composes two body->world rotations: q = a * b (a applied last).
     */
    inline void quatMultiply(
        double aw, double ax, double ay, double az,
        double bw, double bx, double by, double bz,
        double& qw, double& qx, double& qy, double& qz)
    {
        qw = aw * bw - ax * bx - ay * by - az * bz;
        qx = aw * bx + ax * bw + ay * bz - az * by;
        qy = aw * by - ax * bz + ay * bw + az * bx;
        qz = aw * bz + ax * by - ay * bx + az * bw;
    }

    /**
     * Rotates a vector from world to body frame: v' = q^-1 * v * q.
     * For unit q, q^-1 = (qw, -qx, -qy, -qz).
     */
    inline void quatRotateToBody(
        double qw, double qx, double qy, double qz,
        double vx, double vy, double vz,
        double& rx, double& ry, double& rz)
    {
        quatRotateToWorld(qw, -qx, -qy, -qz, vx, vy, vz, rx, ry, rz);
    }

    /**
     * Unit quaternion from a body->world rotation matrix whose COLUMNS are the
     * body axes expressed in world (i.e. m[row][col], row-major m00 = X_body.x).
     * Shepperd's trace branches; the largest diagonal term is used to avoid the
     * near-zero divisor of the naive trace formula.
     */
    inline void quatFromRotationMatrix(
        double m00, double m01, double m02,
        double m10, double m11, double m12,
        double m20, double m21, double m22,
        double& qw, double& qx, double& qy, double& qz)
    {
        const double tr = m00 + m11 + m22;
        if (tr > 0.0) {
            const double sc = std::sqrt(tr + 1.0) * 2.0;
            qw = 0.25 * sc;
            qx = (m21 - m12) / sc;
            qy = (m02 - m20) / sc;
            qz = (m10 - m01) / sc;
        } else if (m00 > m11 && m00 > m22) {
            const double sc = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
            qw = (m21 - m12) / sc;
            qx = 0.25 * sc;
            qy = (m01 + m10) / sc;
            qz = (m02 + m20) / sc;
        } else if (m11 > m22) {
            const double sc = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
            qw = (m02 - m20) / sc;
            qx = (m01 + m10) / sc;
            qy = 0.25 * sc;
            qz = (m12 + m21) / sc;
        } else {
            const double sc = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
            qw = (m10 - m01) / sc;
            qx = (m02 + m20) / sc;
            qy = (m12 + m21) / sc;
            qz = 0.25 * sc;
        }
    }

} // namespace StrikeEngine::Kernel
