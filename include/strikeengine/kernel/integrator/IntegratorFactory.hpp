#pragma once

#include <memory>
#include <strikeengine/kernel/integrator/EulerIntegrator.hpp>
#include <strikeengine/kernel/integrator/RK4Integrator.hpp>
#include <strikeengine/kernel/integrator/SymplecticIntegrator.hpp>
#include <strikeengine/kernel/integrator/RK45Integrator.hpp>

namespace StrikeEngine::Kernel
{

	enum class IntegratorType
	{
		Euler,
		RK4,
		Symplectic,
		RK45
	};

	class IntegratorFactory
	{
	public:
		static std::unique_ptr<Integrator> create(
			IntegratorType type)
		{
			switch (type)
			{
			case IntegratorType::Euler:
				return std::make_unique<EulerIntegrator>();

			case IntegratorType::Symplectic:
				return std::make_unique<SymplecticIntegrator>();

			case IntegratorType::RK45:
				return std::make_unique<RK45Integrator>();

			case IntegratorType::RK4:
			default:
				return std::make_unique<RK4Integrator>();
			}
		}
	};

} // namespace StrikeEngine::Kernel