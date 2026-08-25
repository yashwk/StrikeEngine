#include "RK4Integrator.hpp"
#include <vector>
#include <cmath>

#include "../data/PhysicsBlock.hpp"

namespace StrikeEngine::Kernel
{

double RK4Integrator::integrate(
    PhysicsBlock& physics,
    double dt)
{
    const std::size_t n = physics.size;

    std::vector<double> px0 = physics.px;
    std::vector<double> py0 = physics.py;
    std::vector<double> pz0 = physics.pz;

    std::vector<double> vx0 = physics.vx;
    std::vector<double> vy0 = physics.vy;
    std::vector<double> vz0 = physics.vz;

    for (std::size_t i = 0; i < n; ++i)
    {
        if (!physics.active[i])
            continue;

        const double ax = physics.ax[i];
        const double ay = physics.ay[i];
        const double az = physics.az[i];

        const double k1_vx = ax;
        const double k1_vy = ay;
        const double k1_vz = az;

        const double k1_px = vx0[i];
        const double k1_py = vy0[i];
        const double k1_pz = vz0[i];

        const double k2_vx = ax;
        const double k2_vy = ay;
        const double k2_vz = az;

        const double k2_px = vx0[i] + 0.5 * dt * k1_vx;
        const double k2_py = vy0[i] + 0.5 * dt * k1_vy;
        const double k2_pz = vz0[i] + 0.5 * dt * k1_vz;

        const double k3_vx = ax;
        const double k3_vy = ay;
        const double k3_vz = az;

        const double k3_px = vx0[i] + 0.5 * dt * k2_vx;
        const double k3_py = vy0[i] + 0.5 * dt * k2_vy;
        const double k3_pz = vz0[i] + 0.5 * dt * k2_vz;

        const double k4_vx = ax;
        const double k4_vy = ay;
        const double k4_vz = az;

        const double k4_px = vx0[i] + dt * k3_vx;
        const double k4_py = vy0[i] + dt * k3_vy;
        const double k4_pz = vz0[i] + dt * k3_vz;

        physics.vx[i] += (dt / 6.0) * (k1_vx + 2*k2_vx + 2*k3_vx + k4_vx);
        physics.vy[i] += (dt / 6.0) * (k1_vy + 2*k2_vy + 2*k3_vy + k4_vy);
        physics.vz[i] += (dt / 6.0) * (k1_vz + 2*k2_vz + 2*k3_vz + k4_vz);

        physics.px[i] += (dt / 6.0) * (k1_px + 2*k2_px + 2*k3_px + k4_px);
        physics.py[i] += (dt / 6.0) * (k1_py + 2*k2_py + 2*k3_py + k4_py);
        physics.pz[i] += (dt / 6.0) * (k1_pz + 2*k2_pz + 2*k3_pz + k4_pz);

        // Angular velocity integration (Euler for MVP)
        physics.wx[i] += physics.alphax[i] * dt;
        physics.wy[i] += physics.alphay[i] * dt;
        physics.wz[i] += physics.alphaz[i] * dt;

        // Quaternion integration
        double qw = physics.qw[i];
        double qx = physics.qx[i];
        double qy = physics.qy[i];
        double qz = physics.qz[i];

        double wx = physics.wx[i];
        double wy = physics.wy[i];
        double wz = physics.wz[i];

        double dqw = 0.5 * (-qx*wx - qy*wy - qz*wz);
        double dqx = 0.5 * ( qw*wx + qy*wz - qz*wy);
        double dqy = 0.5 * ( qw*wy - qx*wz + qz*wx);
        double dqz = 0.5 * ( qw*wz + qx*wy - qy*wx);

        qw += dqw * dt;
        qx += dqx * dt;
        qy += dqy * dt;
        qz += dqz * dt;

        double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (norm > 0) {
            physics.qw[i] = qw / norm;
            physics.qx[i] = qx / norm;
            physics.qy[i] = qy / norm;
            physics.qz[i] = qz / norm;
        }
    }

    return dt;
}

} // namespace StrikeEngine::Kernel