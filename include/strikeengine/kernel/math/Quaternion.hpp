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

} // namespace StrikeEngine::Kernel
