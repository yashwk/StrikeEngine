#pragma once

#include "Integrator.hpp"

namespace StrikeEngine::Kernel
{

	/**
	 * @brief Velocity Verlet (Symplectic) integrator.
	 *
	 * Energy preserving for long-duration simulations.
	 */
	class SymplecticIntegrator final : public Integrator
	{
	public:
		double integrate(
			PhysicsBlock& physics,
			double dt) override;

		bool isAdaptive() const override { return false; }
	};

} // namespace StrikeEngine::Kernel