#pragma once
#include <vector>
#include <cstddef>

namespace StrikeEngine::Kernel {

/**
 * SoA truth-state block for all entities.
 *
 * Frame conventions (W1/W2 spine):
 *  - WORLD frame: local flat-earth ENU-style coordinates by default, with
 *    absolute ECEF position/velocity available when
 *    EnvironmentConfig::earth.useEcefTruth is enabled. ax/ay/az remain in
 *    the selected world frame (sensors rotate them to body).
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
	std::vector<double> clMax;             // max |CL| (stall / control limit)
	std::vector<int>    propulsionId;      // index into backend propulsion pool; -1 = none
	std::vector<double> ignitionTime;      // s (thrust curve evaluated at t - ignitionTime)
	std::vector<int>    stageIndex;        // active stage; -1 = coasting/finished
	std::vector<int>    stageCount;        // number of registered stages
	std::vector<double> stageMinMass;      // minimum mass for the current active stage; below
	                                       // it the stage's propellant is exhausted. For a
	                                       // coasting entity or uncapped stage it equals/falls
	                                       // below massDry; for a capped stage it is
	                                       // massDry + (propellant reserved for later stages).

	// Actuator truth (achieved deflections, rad)
	std::vector<double> finPitch, finYaw, finRoll;

	// Per-entity actuator/config parameters (W1)
	std::vector<double> maxDeflectionRad;      // achieved fin clamp (rad)
	std::vector<double> servoTimeConstantSec;  // first-order actuator lag (s)
	std::vector<double> maxServoRateRadPerSec; // servo rate limit (rad/s)

	std::vector<bool> active;

	// Physical truth mirror (NOT integrated state): lets the backend read
	// failure flags without a signature change. Canonical flags live in
	// EntityStatusBlock; these are kept in sync by the kernel.
	std::vector<bool> motorFailed;
	std::vector<bool> actuatorFailed;

	size_t size = 0;
};

} // namespace StrikeEngine::Kernel
