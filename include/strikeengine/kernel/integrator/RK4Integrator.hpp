#pragma once

#include <strikeengine/kernel/integrator/Integrator.hpp>

namespace StrikeEngine::Kernel
{
	/**
	 * @brief Classical 4th-order Runge-Kutta with force re-evaluation at
	 * every stage (true RK4 — not constant-acceleration stepping).
	 */
	class RK4Integrator final : public Integrator
	{
	public:
		double integrate(
			PhysicsBlock& state,
			const DerivativeFn& deriv,
			double t,
			double dt) override;
		bool isAdaptive() const override { return false; }

	private:
		// Stage/accumulator scratch reused across calls: no per-step heap
		// allocation. Each kernel owns one integrator instance, so the
		// buffers are never shared across concurrent runs.
		void ensureCapacity(const PhysicsBlock& state);
		PhysicsBlock k1, k2, k3, k4, stage, acc;
	};
} // namespace StrikeEngine::Kernel
