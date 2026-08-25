#include <strikeengine/kernel/integrator/SymplecticIntegrator.hpp>

namespace StrikeEngine::Kernel
{

double SymplecticIntegrator::integrate(
    PhysicsBlock& state,
    const DerivativeFn& deriv,
    double t,
    double dt)
{
    const double h = dt;
    const double h2 = 0.5 * h;

    // Kick-drift-kick (velocity Verlet) with force re-evaluation at midpoint.
    PhysicsBlock a0 = state;   // accelerations at t
    deriv(state, t, a0);

    // Half kick: v += 0.5 h * a(t)
    PhysicsBlock half = state;
    const std::size_t n = state.size;
    for (std::size_t i = 0; i < n; ++i)
    {
        if (!state.active[i]) continue;
        half.vx[i] += h2 * a0.vx[i];
        half.vy[i] += h2 * a0.vy[i];
        half.vz[i] += h2 * a0.vz[i];
        half.wx[i] += h2 * a0.wx[i];
        half.wy[i] += h2 * a0.wy[i];
        half.wz[i] += h2 * a0.wz[i];
    }

    // Drift: x += h * v_mid  (position/rotation/attitude integrate from mid-state)
    PhysicsBlock dHalf = state;
    deriv(half, t + h2, dHalf);
    applyStateUpdate(state, dHalf, h);

    // Second half kick: v += 0.5 h * a(t + h)
    PhysicsBlock a1 = state;
    deriv(state, t + h, a1);
    for (std::size_t i = 0; i < n; ++i)
    {
        if (!state.active[i]) continue;
        state.vx[i] += h2 * a1.vx[i];
        state.vy[i] += h2 * a1.vy[i];
        state.vz[i] += h2 * a1.vz[i];
        state.wx[i] += h2 * a1.wx[i];
        state.wy[i] += h2 * a1.wy[i];
        state.wz[i] += h2 * a1.wz[i];
    }

    return h;
}

} // namespace StrikeEngine::Kernel
