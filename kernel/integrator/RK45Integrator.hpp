#pragma once

#include "Integrator.hpp"

namespace StrikeEngine::Kernel
{

	class RK45Integrator final : public Integrator
	{
	public:
		RK45Integrator(double tolerance = 1e-6);

		double integrate(
			PhysicsBlock& physics,
			double dt) override;

		bool isAdaptive() const override { return true; }

	private:
		double tolerance;
	};

} // namespace StrikeEngine::Kernel