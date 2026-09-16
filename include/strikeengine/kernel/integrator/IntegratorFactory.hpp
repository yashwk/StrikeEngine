#pragma once

#include <memory>
#include <strikeengine/kernel/integrator/RK4Integrator.hpp>
#include <strikeengine/kernel/integrator/RK45Integrator.hpp>

namespace StrikeEngine::Kernel
{

	/**
	 * @brief Selects the truth integrator used by the CPU backend.
	 *
	 * RK4 is the default. RK45 is an adaptive-step scheme that subdivides
	 * within the requested step (it never returns a shorter step than asked
	 * for); see RK45Integrator.
	 *
	 * Euler and velocity-Verlet (symplectic) were removed: Euler is
	 * first-order and unusable for the stiff roll mode, and Verlet is only
	 * symplectic for a separable Hamiltonian, which a velocity- and
	 * rate-dependent aerodynamic force does not have. It cost 3 derivative
	 * evaluations for O(h^2) where RK4 costs 4 for O(h^4).
	 */
	enum class IntegratorType
	{
		RK4,
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
			case IntegratorType::RK45:
				return std::make_unique<RK45Integrator>();

			case IntegratorType::RK4:
			default:
				return std::make_unique<RK4Integrator>();
			}
		}
	};

} // namespace StrikeEngine::Kernel