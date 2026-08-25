#include <strikeengine/kernel/integrator/SymplecticIntegrator.hpp>
#include <cmath>
#include <vector>
#include <strikeengine/kernel/data/PhysicsBlock.hpp>

namespace StrikeEngine::Kernel
{

	double SymplecticIntegrator::integrate(
		PhysicsBlock& physics,
		double dt)
	{
		const std::size_t n = physics.size;

		for (std::size_t i = 0; i < n; ++i)
		{
			if (!physics.active[i])
				continue;

			// Half velocity update
			physics.vx[i] += 0.5 * physics.ax[i] * dt;
			physics.vy[i] += 0.5 * physics.ay[i] * dt;
			physics.vz[i] += 0.5 * physics.az[i] * dt;

			// Position update
			physics.px[i] += physics.vx[i] * dt;
			physics.py[i] += physics.vy[i] * dt;
			physics.pz[i] += physics.vz[i] * dt;

			// Second half velocity update
			physics.vx[i] += 0.5 * physics.ax[i] * dt;
			physics.vy[i] += 0.5 * physics.ay[i] * dt;
			physics.vz[i] += 0.5 * physics.az[i] * dt;
		}

		return dt;
	}

} // namespace StrikeEngine::Kernel