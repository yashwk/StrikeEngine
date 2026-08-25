#include <strikeengine/kernel/integrator/EulerIntegrator.hpp>

namespace StrikeEngine::Kernel
{

double EulerIntegrator::integrate(
    PhysicsBlock& state,
    const DerivativeFn& deriv,
    double t,
    double dt)
{
    PhysicsBlock d = state;
    deriv(state, t, d);
    applyStateUpdate(state, d, dt);
    return dt;
}

} // namespace StrikeEngine::Kernel
