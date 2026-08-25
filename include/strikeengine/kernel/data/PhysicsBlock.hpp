#pragma once
#include <vector>
#include <cstddef>

namespace StrikeEngine::Kernel {

/**
 * SoA truth-state block for all entities.
 *
 * Frame conventions (W1/W2 spine):
 *  - WORLD frame: flat earth, Z UP, ground at pz == 0. ax/ay/az are world
 *    accelerations (sensors rotate them to body for specific force).
 *  - BODY frame: aerospace NED-ish (X forward, Y right, Z down). wx/wy/wz
 *    are BODY angular rates; q = body->world quaternion.
 *  - finPitch/finYaw/finRoll are the ACHIEVED servo deflections (rad),
 *    integrated with first-order lag toward the commanded values.
 *  - mass is total mass (dry + fuel). It integrates down to massDry via the
 *    propulsion mass flow; never below.
 */
struct PhysicsBlock {
	// Translational truth (world)
	std::vector<double> px, py, pz;
	std::vector<double> vx, vy, vz;
	std::vector<double> ax, ay, az;        // world accelerations (cache, refreshed post-step)

	// Rotational truth
	std::vector<double> qw, qx, qy, qz;    // body->world unit quaternion
	std::vector<double> wx, wy, wz;        // BODY angular rates (rad/s)
	std::vector<double> alphax, alphay, alphaz; // BODY angular accel (cache)

	// Mass properties
	std::vector<double> Ixx, Iyy, Izz;     // principal moments of inertia (body)
	std::vector<double> mass;              // total mass (kg)
	std::vector<double> massDry;           // dry mass (kg); fuel = mass - massDry

	// Per-entity truth-model parameters (W1)
	std::vector<double> referenceArea;
	std::vector<double> referenceLength;
	std::vector<double> cd;
	std::vector<double> clAlpha;           // lift slope 1/rad
	std::vector<double> clFin;             // fin lift 1/rad (deflection)
	std::vector<int>    propulsionId;      // index into backend propulsion pool; -1 = none
	std::vector<double> ignitionTime;      // s (thrust curve evaluated at t - ignitionTime)

	// Actuator truth (achieved deflections, rad)
	std::vector<double> finPitch, finYaw, finRoll;

	std::vector<bool> active;

	size_t size = 0;
};

} // namespace StrikeEngine::Kernel
