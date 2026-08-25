#pragma once

#include <strikeengine/kernel/integrator/Integrator.hpp>

namespace StrikeEngine::Kernel
{
	/**
	 * @brief Adaptive 4th/5th-order Runge-Kutta (Fehlberg pair) with
	 * per-step error control and step halving on rejection.
	 */
	class RK45Integrator final : public Integrator
	{
	public:
		RK45Integrator(double tolerance = 1e-4);
		double integrate(
			PhysicsBlock& state,
			const DerivativeFn& deriv,
			double t,
			double dt) override;
		bool isAdaptive() const override { return true; }
	private:
		double tolerance;
	};
} // namespace StrikeEngine::Kernel
