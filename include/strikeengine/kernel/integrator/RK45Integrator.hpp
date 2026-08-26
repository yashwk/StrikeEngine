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

		// Diagnostics for validation and runtime telemetry.
		std::size_t acceptedSteps() const { return acceptedStepCount; }
		std::size_t rejectedSteps() const { return rejectedStepCount; }
	private:
		double tolerance;
		std::size_t acceptedStepCount = 0;
		std::size_t rejectedStepCount = 0;
	};
} // namespace StrikeEngine::Kernel
