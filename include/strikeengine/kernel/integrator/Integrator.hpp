#pragma once

#include <cstddef>
#include <cmath>
#include <functional>
#include <algorithm>

#include <strikeengine/kernel/data/PhysicsBlock.hpp>

namespace StrikeEngine::Kernel
{

	/**
	 * @brief Applies x1 = x0 + scale * d for INTEGRATED state fields only.
	 *
	 * The PhysicsBlock also carries derived caches (ax/ay/az, alphax..) and
	 * constants (Ixx.., referenceArea, ...). Only the true ODE state is
	 * advanced here:
	 *   px,py,pz , vx,vy,vz , qw,qx,qy,qz , wx,wy,wz , mass , finPitch/finYaw/finRoll,
	 *   gimbalPitch/gimbalYaw
	 * The attitude is advanced by the exponential map of the body rate rather
	 * than the linear quaternion derivative; mass is floored at the stage-aware
	 * floor and the servo deflections are clamped to their limits.
	 */
	inline void applyStateUpdate(PhysicsBlock& state, const PhysicsBlock& d, double scale)
	{
		const std::size_t n = state.size;
		for (std::size_t i = 0; i < n; ++i)
		{
			if (!state.active[i])
				continue;

			state.px[i] += scale * d.px[i];
			state.py[i] += scale * d.py[i];
			state.pz[i] += scale * d.pz[i];

			state.vx[i] += scale * d.vx[i];
			state.vy[i] += scale * d.vy[i];
			state.vz[i] += scale * d.vz[i];

			state.wx[i] += scale * d.wx[i];
			state.wy[i] += scale * d.wy[i];
			state.wz[i] += scale * d.wz[i];

			// Attitude: apply the exponential map of the body rate, whose exact
			// solution over the interval is q * exp(w * scale / 2). This stays
			// on the unit sphere by construction, unlike adding a linear
			// derivative and renormalizing, whose projection bounds the
			// achievable attitude order. exp(w a) = (cos a, sin(a) * w_hat).
			{
				const double wx = state.wx[i];
				const double wy = state.wy[i];
				const double wz = state.wz[i];
				const double wmag = std::sqrt(wx * wx + wy * wy + wz * wz);
				const double half = 0.5 * wmag * scale;
				double iw, ix, iy, iz;
				if (half > 1e-12) {
					// sin(half)/wmag is the sin(angle/2) over the axis length;
					// well conditioned for any wmag above the threshold.
					const double s = std::sin(half) / wmag;
					iw = std::cos(half); ix = s * wx; iy = s * wy; iz = s * wz;
				} else {
					// Second-order limit as the interval's angle vanishes.
					iw = 1.0;
					ix = 0.5 * scale * wx;
					iy = 0.5 * scale * wy;
					iz = 0.5 * scale * wz;
				}
				const double qw = state.qw[i], qx = state.qx[i];
				const double qy = state.qy[i], qz = state.qz[i];
				state.qw[i] = qw * iw - qx * ix - qy * iy - qz * iz;
				state.qx[i] = qw * ix + qx * iw + qy * iz - qz * iy;
				state.qy[i] = qw * iy - qx * iz + qy * iw + qz * ix;
				state.qz[i] = qw * iz + qx * iy - qy * ix + qz * iw;
			}

			// Renormalize: the product of two unit quaternions is unit, so this
			// corrects floating-point round-off only.
			const double qn = std::sqrt(state.qw[i] * state.qw[i] + state.qx[i] * state.qx[i] +
			                            state.qy[i] * state.qy[i] + state.qz[i] * state.qz[i]);
			const double qinv = (qn > 1e-12) ? 1.0 / qn : 0.0;
			state.qw[i] *= qinv;
			state.qx[i] *= qinv;
			state.qy[i] *= qinv;
			state.qz[i] *= qinv;

			// Mass: burns down to the stage-aware floor (structural dry mass or
			// the current stage's propellant-exhaustion floor), never below.
			const double floor = std::max(state.massDry[i],
			                              (i < state.stageMinMass.size()) ? state.stageMinMass[i] : 0.0);
			state.mass[i] += scale * d.mass[i];
			if (state.mass[i] < floor)
				state.mass[i] = floor;

			// Servo deflections clamped to the per-entity physical limit
			const double defl = (i < state.maxDeflectionRad.size()) ? state.maxDeflectionRad[i] : 0.43;
			state.finPitch[i] = std::clamp(state.finPitch[i] + scale * d.finPitch[i], -defl, defl);
			state.finYaw[i]   = std::clamp(state.finYaw[i]   + scale * d.finYaw[i],   -defl, defl);
			state.finRoll[i]  = std::clamp(state.finRoll[i]  + scale * d.finRoll[i],  -defl, defl);

			if (i < state.gimbalPitch.size() && i < state.gimbalYaw.size() &&
				i < d.gimbalPitch.size() && i < d.gimbalYaw.size()) {
				const double pitchLimit = (i < state.maxGimbalPitchRad.size())
					? state.maxGimbalPitchRad[i] : 0.0;
				const double yawLimit = (i < state.maxGimbalYawRad.size())
					? state.maxGimbalYawRad[i] : 0.0;
				state.gimbalPitch[i] = std::clamp(state.gimbalPitch[i] + scale * d.gimbalPitch[i],
				                                 -pitchLimit, pitchLimit);
				state.gimbalYaw[i] = std::clamp(state.gimbalYaw[i] + scale * d.gimbalYaw[i],
				                               -yawLimit, yawLimit);
			}
		}
	}

	/**
	 * @brief Base integrator interface (derivative-callback form).
	 *
	 * The physics backend supplies a pure derivative function
	 *   deriv(state, t) -> state_dot
	 * that re-evaluates ALL forces/moments at the given state:
	 * required for 6-DOF coupling, servo lag and mass flow to integrate
	 * stably). Integrators advance only the integrated state fields via
	 * applyStateUpdate.
	 *
	 * @param state State to advance (in/out).
	 * @param deriv Derivative evaluator: fills d with d(state)/dt.
	 * @param t     Current simulation time (s).
	 * @param dt    Desired timestep (s).
	 * @return Actual timestep applied (for adaptive integrators).
	 */
	class Integrator
	{
	public:
		using DerivativeFn = std::function<void(const PhysicsBlock& state, double t, PhysicsBlock& d)>;

		virtual ~Integrator() = default;

		virtual double integrate(
			PhysicsBlock& state,
			const DerivativeFn& deriv,
			double t,
			double dt) = 0;

		virtual bool isAdaptive() const = 0;
	};

} // namespace StrikeEngine::Kernel
