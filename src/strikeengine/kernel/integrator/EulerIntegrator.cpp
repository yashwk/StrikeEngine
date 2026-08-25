#include <strikeengine/kernel/integrator/EulerIntegrator.hpp>
#include <cmath>
#include <vector>

#include <strikeengine/kernel/data/PhysicsBlock.hpp>

namespace StrikeEngine::Kernel
{

	double EulerIntegrator::integrate(
		PhysicsBlock& physics,
		double dt)
	{
		const std::size_t n = physics.size;
		const double halfDt = 0.5 * dt;

		for (std::size_t i = 0; i < n; ++i)
		{
			if (!physics.active[i])
				continue;

			// --- Translational ---
			physics.vx[i] += physics.ax[i] * dt;
			physics.vy[i] += physics.ay[i] * dt;
			physics.vz[i] += physics.az[i] * dt;

			physics.px[i] += physics.vx[i] * dt;
			physics.py[i] += physics.vy[i] * dt;
			physics.pz[i] += physics.vz[i] * dt;

			// --- Rotational ---
			const double wx = physics.wx[i];
			const double wy = physics.wy[i];
			const double wz = physics.wz[i];

			double qw = physics.qw[i];
			double qx = physics.qx[i];
			double qy = physics.qy[i];
			double qz = physics.qz[i];

			const double dq_w = -halfDt * (wx*qx + wy*qy + wz*qz);
			const double dq_x =  halfDt * (wx*qw + wy*qz - wz*qy);
			const double dq_y =  halfDt * (-wx*qz + wy*qw + wz*qx);
			const double dq_z =  halfDt * (wx*qy - wy*qx + wz*qw);

			qw += dq_w;
			qx += dq_x;
			qy += dq_y;
			qz += dq_z;

			const double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);

			if (norm > 0.0)
			{
				physics.qw[i] = qw / norm;
				physics.qx[i] = qx / norm;
				physics.qy[i] = qy / norm;
				physics.qz[i] = qz / norm;
			}
		}

		return dt;
	}

} // namespace StrikeEngine::Kernel