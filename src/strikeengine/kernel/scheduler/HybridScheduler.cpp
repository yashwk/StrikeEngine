#include <strikeengine/kernel/scheduler/HybridScheduler.hpp>
#include <strikeengine/kernel/integrator/Integrator.hpp>

namespace StrikeEngine::Kernel
{

	HybridScheduler::HybridScheduler(Integrator& integ)
		: integrator(integ)
	{
	}

	void HybridScheduler::executeStep(
		PhysicsBlock& physics,
		double globalDt)
	{
		double accumulated = 0.0;
		double remaining = globalDt;

		while (remaining > 0.0)
		{
			double proposedDt = remaining;

			double actualDt = integrator.integrate(physics, proposedDt);

			// Clamp safety
			if (actualDt <= 0.0)
				actualDt = remaining;

			if (actualDt > remaining)
				actualDt = remaining;

			accumulated += actualDt;
			remaining = globalDt - accumulated;

			// Prevent infinite loops due to tiny tolerances
			if (remaining < 1e-12)
				break;
		}
	}

} // namespace StrikeEngine::Kernel