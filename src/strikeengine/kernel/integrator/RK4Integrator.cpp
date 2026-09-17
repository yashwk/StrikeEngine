#include <strikeengine/kernel/integrator/RK4Integrator.hpp>
#include <vector>

namespace StrikeEngine::Kernel
{

void RK4Integrator::ensureCapacity(const PhysicsBlock& state)
{
    // Derivative buffers are fully overwritten by the derivative callback
    // before they are read, so they only need the right size; only the stage
    // is passed as a state and therefore needs the per-entity configuration.
    k1.ensureSize(state.size);
    k2.ensureSize(state.size);
    k3.ensureSize(state.size);
    k4.ensureSize(state.size);
    acc.ensureSize(state.size);
    stage = state;
}

double RK4Integrator::integrate(
    PhysicsBlock& state,
    const DerivativeFn& deriv,
    double t,
    double dt)
{
    const double h = dt;
    const double h2 = 0.5 * h;

    // Stage states and derivative buffers: reused member scratch (copy
    // assignment recycles capacity; no allocation in steady state).
    ensureCapacity(state);

    deriv(state, t, k1);

    // k2 = f(t + h/2, x + h/2*k1)
    copyIntegratedState(stage, state);
    applyStateUpdate(stage, k1, h2);
    deriv(stage, t + h2, k2);

    // k3 = f(t + h/2, x + h/2*k2)
    copyIntegratedState(stage, state);
    applyStateUpdate(stage, k2, h2);
    deriv(stage, t + h2, k3);

    // k4 = f(t + h, x + h*k3)
    copyIntegratedState(stage, state);
    applyStateUpdate(stage, k3, h);
    deriv(stage, t + h, k4);

    // x1 = x0 + h/6 (k1 + 2 k2 + 2 k3 + k4)
    const std::size_t n = state.size;
    for (std::size_t i = 0; i < n; ++i)
    {
        if (!state.active[i]) continue;
        acc.px[i] = (k1.px[i] + 2.0 * k2.px[i] + 2.0 * k3.px[i] + k4.px[i]) / 6.0;
        acc.py[i] = (k1.py[i] + 2.0 * k2.py[i] + 2.0 * k3.py[i] + k4.py[i]) / 6.0;
        acc.pz[i] = (k1.pz[i] + 2.0 * k2.pz[i] + 2.0 * k3.pz[i] + k4.pz[i]) / 6.0;

        acc.vx[i] = (k1.vx[i] + 2.0 * k2.vx[i] + 2.0 * k3.vx[i] + k4.vx[i]) / 6.0;
        acc.vy[i] = (k1.vy[i] + 2.0 * k2.vy[i] + 2.0 * k3.vy[i] + k4.vy[i]) / 6.0;
        acc.vz[i] = (k1.vz[i] + 2.0 * k2.vz[i] + 2.0 * k3.vz[i] + k4.vz[i]) / 6.0;

        acc.wx[i] = (k1.wx[i] + 2.0 * k2.wx[i] + 2.0 * k3.wx[i] + k4.wx[i]) / 6.0;
        acc.wy[i] = (k1.wy[i] + 2.0 * k2.wy[i] + 2.0 * k3.wy[i] + k4.wy[i]) / 6.0;
        acc.wz[i] = (k1.wz[i] + 2.0 * k2.wz[i] + 2.0 * k3.wz[i] + k4.wz[i]) / 6.0;

        acc.qw[i] = (k1.qw[i] + 2.0 * k2.qw[i] + 2.0 * k3.qw[i] + k4.qw[i]) / 6.0;
        acc.qx[i] = (k1.qx[i] + 2.0 * k2.qx[i] + 2.0 * k3.qx[i] + k4.qx[i]) / 6.0;
        acc.qy[i] = (k1.qy[i] + 2.0 * k2.qy[i] + 2.0 * k3.qy[i] + k4.qy[i]) / 6.0;
        acc.qz[i] = (k1.qz[i] + 2.0 * k2.qz[i] + 2.0 * k3.qz[i] + k4.qz[i]) / 6.0;

        acc.mass[i]       = (k1.mass[i] + 2.0 * k2.mass[i] + 2.0 * k3.mass[i] + k4.mass[i]) / 6.0;
        acc.finPitch[i]   = (k1.finPitch[i] + 2.0 * k2.finPitch[i] + 2.0 * k3.finPitch[i] + k4.finPitch[i]) / 6.0;
        acc.finYaw[i]     = (k1.finYaw[i] + 2.0 * k2.finYaw[i] + 2.0 * k3.finYaw[i] + k4.finYaw[i]) / 6.0;
        acc.finRoll[i]    = (k1.finRoll[i] + 2.0 * k2.finRoll[i] + 2.0 * k3.finRoll[i] + k4.finRoll[i]) / 6.0;
        if (i < acc.gimbalPitch.size() && i < acc.gimbalYaw.size()) {
            acc.gimbalPitch[i] = (k1.gimbalPitch[i] + 2.0 * k2.gimbalPitch[i] +
                                  2.0 * k3.gimbalPitch[i] + k4.gimbalPitch[i]) / 6.0;
            acc.gimbalYaw[i] = (k1.gimbalYaw[i] + 2.0 * k2.gimbalYaw[i] +
                                2.0 * k3.gimbalYaw[i] + k4.gimbalYaw[i]) / 6.0;
        }
    }

    applyStateUpdate(state, acc, h);
    return h;
}

} // namespace StrikeEngine::Kernel
