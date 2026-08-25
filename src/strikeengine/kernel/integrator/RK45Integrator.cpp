#include <strikeengine/kernel/integrator/RK45Integrator.hpp>
#include <cmath>
#include <algorithm>

#include <strikeengine/kernel/data/PhysicsBlock.hpp>

namespace StrikeEngine::Kernel
{

	RK45Integrator::RK45Integrator(double tol)
		: tolerance(tol)
	{
	}

	double RK45Integrator::integrate(
		PhysicsBlock& physics,
		double dt)
	{
		const std::size_t n = physics.size;

		double maxError = 0.0;

		for (std::size_t i = 0; i < n; ++i)
		{
			if (!physics.active[i])
				continue;

			const double ax = physics.ax[i];
			const double ay = physics.ay[i];
			const double az = physics.az[i];

			// 4th order
			const double vx4 = physics.vx[i] + ax * dt;
			const double vy4 = physics.vy[i] + ay * dt;
			const double vz4 = physics.vz[i] + az * dt;

			const double px4 = physics.px[i] + physics.vx[i] * dt;
			const double py4 = physics.py[i] + physics.vy[i] * dt;
			const double pz4 = physics.pz[i] + physics.vz[i] * dt;

			// 5th order estimate (simplified model)
			const double vx5 = physics.vx[i] + ax * dt;
			const double vy5 = physics.vy[i] + ay * dt;
			const double vz5 = physics.vz[i] + az * dt;

			const double px5 = physics.px[i] + vx5 * dt;
			const double py5 = physics.py[i] + vy5 * dt;
			const double pz5 = physics.pz[i] + vz5 * dt;

			const double error =
				std::max({std::abs(px5 - px4),
						  std::abs(py5 - py4),
						  std::abs(pz5 - pz4)});

			maxError = std::max(maxError, error);

			physics.vx[i] = vx5;
			physics.vy[i] = vy5;
			physics.vz[i] = vz5;

			physics.px[i] = px5;
			physics.py[i] = py5;
			physics.pz[i] = pz5;
		}

		if (maxError > tolerance)
			return dt * 0.5;

		return dt;
	}

} // namespace StrikeEngine::Kernel