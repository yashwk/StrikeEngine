#pragma once

#include <cstddef>

namespace StrikeEngine::Kernel
{

	struct PhysicsBlock;

	/**
	 * @brief Base integrator interface.
	 *
	 * Returns actual dt used (for adaptive integrators).
	 */
	class Integrator
	{
	public:
		virtual ~Integrator() = default;

		/**
		 * @brief Integrate physics state.
		 * @param physics Physics state (accelerations already computed).
		 * @param dt Desired timestep.
		 * @return Actual timestep applied.
		 */
		virtual double integrate(
			PhysicsBlock& physics,
			double dt) = 0;

		/**
		 * @brief Whether integrator is adaptive.
		 */
		virtual bool isAdaptive() const = 0;
	};

} // namespace StrikeEngine::Kernel