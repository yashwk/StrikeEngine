#include <strikeengine/kernel/integrator/RK45Integrator.hpp>
#include <cmath>
#include <algorithm>

namespace StrikeEngine::Kernel
{

namespace
{
    // Fehlberg 4(5) coefficients
    constexpr double c2  = 1.0 / 4.0;
    constexpr double c3  = 3.0 / 8.0;
    constexpr double c4  = 12.0 / 13.0;
    constexpr double c5  = 1.0;
    constexpr double c6  = 1.0 / 2.0;

    constexpr double a21 = 1.0 / 4.0;
    constexpr double a31 = 3.0 / 32.0;
    constexpr double a32 = 9.0 / 32.0;
    constexpr double a41 = 1932.0 / 2197.0;
    constexpr double a42 = -7200.0 / 2197.0;
    constexpr double a43 = 7296.0 / 2197.0;
    constexpr double a51 = 439.0 / 216.0;
    constexpr double a52 = -8.0;
    constexpr double a53 = 3680.0 / 513.0;
    constexpr double a54 = -845.0 / 4104.0;
    constexpr double a61 = -8.0 / 27.0;
    constexpr double a62 = 2.0;
    constexpr double a63 = -3544.0 / 2565.0;
    constexpr double a64 = 1859.0 / 4104.0;
    constexpr double a65 = -11.0 / 40.0;

    // 4th-order (local extrapolation uses 5th order as the accepted solution)
    constexpr double b41 = 25.0 / 216.0;
    constexpr double b43 = 1408.0 / 2565.0;
    constexpr double b44 = 2197.0 / 4104.0;
    constexpr double b45 = -1.0 / 5.0;

    // 5th-order
    constexpr double b51 = 16.0 / 135.0;
    constexpr double b53 = 6656.0 / 12825.0;
    constexpr double b54 = 28561.0 / 56430.0;
    constexpr double b55 = -9.0 / 50.0;
    constexpr double b56 = 2.0 / 55.0;
}

RK45Integrator::RK45Integrator(double tol)
    : tolerance(tol)
{
}

double RK45Integrator::integrate(
    PhysicsBlock& state,
    const DerivativeFn& deriv,
    double t0,
    double dt)
{
    double t = t0;
    double remaining = dt;
    double h = dt;

    acceptedStepCount = 0;
    rejectedStepCount = 0;

    PhysicsBlock k1 = state;
    PhysicsBlock k2 = state;
    PhysicsBlock k3 = state;
    PhysicsBlock k4 = state;
    PhysicsBlock k5 = state;
    PhysicsBlock k6 = state;
    PhysicsBlock stage = state;
    PhysicsBlock acc5 = state;
    PhysicsBlock acc4 = state;

    double errMax = 0.0;

    while (remaining > 1e-12)
    {
        if (h > remaining) h = remaining;

        bool accepted = false;
        int attempts = 0;
        while (!accepted && attempts < 200)
        {
            errMax = 0.0;
            deriv(state, t, k1);

            stage = state; applyStateUpdate(stage, k1, h * a21); deriv(stage, t + c2 * h, k2);
            stage = state; applyStateUpdate(stage, k1, h * a31); applyStateUpdate(stage, k2, h * a32); deriv(stage, t + c3 * h, k3);
            stage = state; applyStateUpdate(stage, k1, h * a41); applyStateUpdate(stage, k2, h * a42); applyStateUpdate(stage, k3, h * a43); deriv(stage, t + c4 * h, k4);
            stage = state; applyStateUpdate(stage, k1, h * a51); applyStateUpdate(stage, k2, h * a52); applyStateUpdate(stage, k3, h * a53); applyStateUpdate(stage, k4, h * a54); deriv(stage, t + c5 * h, k5);
            stage = state; applyStateUpdate(stage, k1, h * a61); applyStateUpdate(stage, k2, h * a62); applyStateUpdate(stage, k3, h * a63); applyStateUpdate(stage, k4, h * a64); applyStateUpdate(stage, k5, h * a65); deriv(stage, t + c6 * h, k6);

            // Weighted derivatives for 4th and 5th order solutions
            const std::size_t n = state.size;
            for (std::size_t i = 0; i < n; ++i)
            {
                if (!state.active[i]) continue;

                acc4.px[i] = b41 * k1.px[i] + b43 * k3.px[i] + b44 * k4.px[i] + b45 * k5.px[i];
                acc5.px[i] = b51 * k1.px[i] + b53 * k3.px[i] + b54 * k4.px[i] + b55 * k5.px[i] + b56 * k6.px[i];
                acc4.py[i] = b41 * k1.py[i] + b43 * k3.py[i] + b44 * k4.py[i] + b45 * k5.py[i];
                acc5.py[i] = b51 * k1.py[i] + b53 * k3.py[i] + b54 * k4.py[i] + b55 * k5.py[i] + b56 * k6.py[i];
                acc4.pz[i] = b41 * k1.pz[i] + b43 * k3.pz[i] + b44 * k4.pz[i] + b45 * k5.pz[i];
                acc5.pz[i] = b51 * k1.pz[i] + b53 * k3.pz[i] + b54 * k4.pz[i] + b55 * k5.pz[i] + b56 * k6.pz[i];

                acc4.vx[i] = b41 * k1.vx[i] + b43 * k3.vx[i] + b44 * k4.vx[i] + b45 * k5.vx[i];
                acc5.vx[i] = b51 * k1.vx[i] + b53 * k3.vx[i] + b54 * k4.vx[i] + b55 * k5.vx[i] + b56 * k6.vx[i];
                acc4.vy[i] = b41 * k1.vy[i] + b43 * k3.vy[i] + b44 * k4.vy[i] + b45 * k5.vy[i];
                acc5.vy[i] = b51 * k1.vy[i] + b53 * k3.vy[i] + b54 * k4.vy[i] + b55 * k5.vy[i] + b56 * k6.vy[i];
                acc4.vz[i] = b41 * k1.vz[i] + b43 * k3.vz[i] + b44 * k4.vz[i] + b45 * k5.vz[i];
                acc5.vz[i] = b51 * k1.vz[i] + b53 * k3.vz[i] + b54 * k4.vz[i] + b55 * k5.vz[i] + b56 * k6.vz[i];

                acc4.wx[i] = b41 * k1.wx[i] + b43 * k3.wx[i] + b44 * k4.wx[i] + b45 * k5.wx[i];
                acc5.wx[i] = b51 * k1.wx[i] + b53 * k3.wx[i] + b54 * k4.wx[i] + b55 * k5.wx[i] + b56 * k6.wx[i];
                acc4.wy[i] = b41 * k1.wy[i] + b43 * k3.wy[i] + b44 * k4.wy[i] + b45 * k5.wy[i];
                acc5.wy[i] = b51 * k1.wy[i] + b53 * k3.wy[i] + b54 * k4.wy[i] + b55 * k5.wy[i] + b56 * k6.wy[i];
                acc4.wz[i] = b41 * k1.wz[i] + b43 * k3.wz[i] + b44 * k4.wz[i] + b45 * k5.wz[i];
                acc5.wz[i] = b51 * k1.wz[i] + b53 * k3.wz[i] + b54 * k4.wz[i] + b55 * k5.wz[i] + b56 * k6.wz[i];

                acc4.qw[i] = b41 * k1.qw[i] + b43 * k3.qw[i] + b44 * k4.qw[i] + b45 * k5.qw[i];
                acc5.qw[i] = b51 * k1.qw[i] + b53 * k3.qw[i] + b54 * k4.qw[i] + b55 * k5.qw[i] + b56 * k6.qw[i];
                acc4.qx[i] = b41 * k1.qx[i] + b43 * k3.qx[i] + b44 * k4.qx[i] + b45 * k5.qx[i];
                acc5.qx[i] = b51 * k1.qx[i] + b53 * k3.qx[i] + b54 * k4.qx[i] + b55 * k5.qx[i] + b56 * k6.qx[i];
                acc4.qy[i] = b41 * k1.qy[i] + b43 * k3.qy[i] + b44 * k4.qy[i] + b45 * k5.qy[i];
                acc5.qy[i] = b51 * k1.qy[i] + b53 * k3.qy[i] + b54 * k4.qy[i] + b55 * k5.qy[i] + b56 * k6.qy[i];
                acc4.qz[i] = b41 * k1.qz[i] + b43 * k3.qz[i] + b44 * k4.qz[i] + b45 * k5.qz[i];
                acc5.qz[i] = b51 * k1.qz[i] + b53 * k3.qz[i] + b54 * k4.qz[i] + b55 * k5.qz[i] + b56 * k6.qz[i];

                acc4.mass[i]    = b41 * k1.mass[i] + b43 * k3.mass[i] + b44 * k4.mass[i] + b45 * k5.mass[i];
                acc5.mass[i]    = b51 * k1.mass[i] + b53 * k3.mass[i] + b54 * k4.mass[i] + b55 * k5.mass[i] + b56 * k6.mass[i];
                acc4.finPitch[i] = b41 * k1.finPitch[i] + b43 * k3.finPitch[i] + b44 * k4.finPitch[i] + b45 * k5.finPitch[i];
                acc5.finPitch[i] = b51 * k1.finPitch[i] + b53 * k3.finPitch[i] + b54 * k4.finPitch[i] + b55 * k5.finPitch[i] + b56 * k6.finPitch[i];
                acc4.finYaw[i]   = b41 * k1.finYaw[i] + b43 * k3.finYaw[i] + b44 * k4.finYaw[i] + b45 * k5.finYaw[i];
                acc5.finYaw[i]   = b51 * k1.finYaw[i] + b53 * k3.finYaw[i] + b54 * k4.finYaw[i] + b55 * k5.finYaw[i] + b56 * k6.finYaw[i];
                acc4.finRoll[i]  = b41 * k1.finRoll[i] + b43 * k3.finRoll[i] + b44 * k4.finRoll[i] + b45 * k5.finRoll[i];
                acc5.finRoll[i]  = b51 * k1.finRoll[i] + b53 * k3.finRoll[i] + b54 * k4.finRoll[i] + b55 * k5.finRoll[i] + b56 * k6.finRoll[i];

                // Error estimate: |h*(y4 - y5)| relative to each component's
                // state scale. Include every integrated state group so the
                // controller cannot accept a step that is accurate in
                // translation but poor in attitude or actuator dynamics.
                const double scale = 1.0 + std::abs(state.px[i]);
                errMax = std::max(errMax, std::abs(h * (acc4.px[i] - acc5.px[i])) / scale);
                errMax = std::max(errMax, std::abs(h * (acc4.py[i] - acc5.py[i])) / (1.0 + std::abs(state.py[i])));
                errMax = std::max(errMax, std::abs(h * (acc4.pz[i] - acc5.pz[i])) / (1.0 + std::abs(state.pz[i])));
                errMax = std::max(errMax, std::abs(h * (acc4.vx[i] - acc5.vx[i])) / (1.0 + std::abs(state.vx[i])));
                errMax = std::max(errMax, std::abs(h * (acc4.vy[i] - acc5.vy[i])) / (1.0 + std::abs(state.vy[i])));
                errMax = std::max(errMax, std::abs(h * (acc4.vz[i] - acc5.vz[i])) / (1.0 + std::abs(state.vz[i])));
                errMax = std::max(errMax, std::abs(h * (acc4.wx[i] - acc5.wx[i])) / (1.0 + std::abs(state.wx[i])));
                errMax = std::max(errMax, std::abs(h * (acc4.wy[i] - acc5.wy[i])) / (1.0 + std::abs(state.wy[i])));
                errMax = std::max(errMax, std::abs(h * (acc4.wz[i] - acc5.wz[i])) / (1.0 + std::abs(state.wz[i])));
                errMax = std::max(errMax, std::abs(h * (acc4.qw[i] - acc5.qw[i])) / (1.0 + std::abs(state.qw[i])));
                errMax = std::max(errMax, std::abs(h * (acc4.qx[i] - acc5.qx[i])) / (1.0 + std::abs(state.qx[i])));
                errMax = std::max(errMax, std::abs(h * (acc4.qy[i] - acc5.qy[i])) / (1.0 + std::abs(state.qy[i])));
                errMax = std::max(errMax, std::abs(h * (acc4.qz[i] - acc5.qz[i])) / (1.0 + std::abs(state.qz[i])));
                errMax = std::max(errMax, std::abs(h * (acc4.mass[i] - acc5.mass[i])) / (1.0 + std::abs(state.mass[i])));
                errMax = std::max(errMax, std::abs(h * (acc4.finPitch[i] - acc5.finPitch[i])) / (1.0 + 0.43));
                errMax = std::max(errMax, std::abs(h * (acc4.finYaw[i] - acc5.finYaw[i])) / (1.0 + 0.43));
                errMax = std::max(errMax, std::abs(h * (acc4.finRoll[i] - acc5.finRoll[i])) / (1.0 + 0.43));
            }

            if (errMax <= tolerance || h <= 1e-6)
            {
                accepted = true;
                ++acceptedStepCount;
            }
            else
            {
                h = std::max(1e-6, h * 0.5);
                ++rejectedStepCount;
                ++attempts;
            }
        }

        // Accept 5th-order solution
        applyStateUpdate(state, acc5, h);
        t += h;
        remaining -= h;

        // Adaptive step growth (bounded)
        if (errMax > 0.0)
        {
            const double factor = 0.9 * std::pow(tolerance / errMax, 0.2);
            // Keep adaptation conservative: a bad stage is shrunk quickly,
            // while a smooth region grows by at most 4x per accepted step.
            h = std::clamp(h * factor, 1e-6, std::max(1e-6, 4.0 * h));
            h = std::min(h, dt);
        }
    }

    return dt;
}

} // namespace StrikeEngine::Kernel
