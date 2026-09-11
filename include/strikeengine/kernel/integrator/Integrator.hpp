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
	 * The quaternion is renormalized and mass is floored at massDry.
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

			state.qw[i] += scale * d.qw[i];
			state.qx[i] += scale * d.qx[i];
			state.qy[i] += scale * d.qy[i];
			state.qz[i] += scale * d.qz[i];

			// Renormalize quaternion (exact unit norm)
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
