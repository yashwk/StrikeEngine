#pragma once

#include <strikeengine/kernel/integrator/Integrator.hpp>

namespace StrikeEngine::Kernel
{

	class RK4Integrator final : public Integrator
	{
	public:
		double integrate(
			PhysicsBlock& physics,
			double dt) override;

		bool isAdaptive() const override { return false; }
	};

} // namespace StrikeEngine::Kernel