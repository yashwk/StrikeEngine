#pragma once

#include <strikeengine/kernel/integrator/Integrator.hpp>

namespace StrikeEngine::Kernel
{
	/**
	 * @brief Adaptive 5th/4th-order Runge-Kutta (Dormand-Prince pair).
	 *
	 * Subdivides the requested step internally until the estimated local error
	 * per substep is within @p tolerance; it never returns a shorter step than
	 * it was asked for, so it cannot make the engine's outer step smaller.
	 *
	 * Dormand-Prince is preferred to the older Fehlberg pair for two reasons:
	 * the embedded error estimate is smaller, and the method is FSAL (first
	 * same as last), so the derivative at the end of a step is reused as the
	 * first stage of the next, saving one of six derivative evaluations.
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

		// Stage/accumulator scratch reused across calls and rejection
		// attempts: no per-step heap allocation. Each kernel owns one
		// integrator instance, so the buffers are never shared across runs.
		// accErr holds the embedded error estimate (the accepted solution is
		// `stage` itself, which is built from the same 5th-order weights).
		void ensureCapacity(const PhysicsBlock& state);
		PhysicsBlock k1, k2, k3, k4, k5, k6, k7, stage, accErr;
	};
} // namespace StrikeEngine::Kernel
