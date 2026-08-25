#pragma once

#include <strikeengine/kernel/integrator/Integrator.hpp>

namespace StrikeEngine::Kernel
{
	/**
	 * @brief Velocity Verlet (Symplectic) integrator with force
	 * re-evaluation at the midpoint (kick-drift-kick).
	 *
	 * Energy preserving for long-duration simulations.
	 */
	class SymplecticIntegrator final : public Integrator
	{
	public:
		double integrate(
			PhysicsBlock& state,
			const DerivativeFn& deriv,
			double t,
			double dt) override;
		bool isAdaptive() const override { return false; }
	};
} // namespace StrikeEngine::Kernel
