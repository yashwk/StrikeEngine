#pragma once

#include <cstddef>

namespace StrikeEngine::Kernel
{

	class Integrator;
	struct PhysicsBlock;

	/**
	 * @brief Hybrid adaptive scheduler.
	 *
	 * Keeps global fixed timestep but allows integrator
	 * to subdivide dt internally.
	 */
	class HybridScheduler
	{
	public:
		HybridScheduler(Integrator& integrator);

		void executeStep(
			PhysicsBlock& physics,
			double globalDt);

	private:
		Integrator& integrator;
	};

} // namespace StrikeEngine::Kernel