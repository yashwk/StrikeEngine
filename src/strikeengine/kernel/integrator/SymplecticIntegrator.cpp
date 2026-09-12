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
    const std::size_t n = state.size;

    // Kick-drift-kick (velocity Verlet) with force re-evaluation at the
    // half-kicked midpoint. The drift must advance position/quaternion/mass/
    // fins ONLY: applyStateUpdate would also add h*a_mid to the velocities,
    // which stacked with the second half-kick yields 1.5*h*a per step instead
    // of h*a (verified against constant-gravity free fall). The velocity
    // derivative entries are therefore zeroed before the drift.
    PhysicsBlock a0 = state;   // accelerations at t
    deriv(state, t, a0);

    // Half kick: v += 0.5 h * a(t)  (in place, so the drift's d.px = v_half)
    for (std::size_t i = 0; i < n; ++i)
    {
        if (!state.active[i]) continue;
        state.vx[i] += h2 * a0.vx[i];
        state.vy[i] += h2 * a0.vy[i];
        state.vz[i] += h2 * a0.vz[i];
        state.wx[i] += h2 * a0.wx[i];
        state.wy[i] += h2 * a0.wy[i];
        state.wz[i] += h2 * a0.wz[i];
    }

    // Drift: x += h * v_half; q, mass and fins advance with midpoint
    // derivatives; velocity/angular-rate entries of the derivative are zeroed
    // so the drift touches no rate state.
    PhysicsBlock dHalf = state;
    deriv(state, t + h2, dHalf);
    for (std::size_t i = 0; i < n; ++i)
    {
        dHalf.vx[i] = 0.0;
        dHalf.vy[i] = 0.0;
        dHalf.vz[i] = 0.0;
        dHalf.wx[i] = 0.0;
        dHalf.wy[i] = 0.0;
        dHalf.wz[i] = 0.0;
    }
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
