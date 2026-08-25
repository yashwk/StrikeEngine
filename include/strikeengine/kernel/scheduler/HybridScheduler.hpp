#pragma once

#include <memory>
#include <strikeengine/kernel/integrator/Integrator.hpp>
#include <strikeengine/kernel/data/PhysicsBlock.hpp>

namespace StrikeEngine::Kernel
{

	/**
	 * @brief Substep scheduler for adaptive integrators.
	 *
	 * Feeds the global timestep to the wrapped adaptive integrator in slices;
	 * the integrator consumes each slice (possibly rejecting/retrying
	 * internally) and the scheduler accumulates until the global dt is spent.
	 */
	class HybridScheduler
	{
	public:
		explicit HybridScheduler(Integrator& integ);

		void executeStep(
			PhysicsBlock& physics,
			const Integrator::DerivativeFn& deriv,
			double currentTime,
			double globalDt);

	private:
		Integrator& integrator;
	};

} // namespace StrikeEngine::Kernel
