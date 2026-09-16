#pragma once
#include <strikeengine/kernel/data/BlockGrowth.hpp>
#include <vector>
#include <cstddef>
#include <memory>
#include <strikeengine/models/physics/aerodynamics/CoefficientTable.hpp>
#include <strikeengine/models/physics/aerodynamics/FinsModel.hpp>

namespace StrikeEngine::Models { struct AirframeParams; }  // aircraft airframe (AirframeModel.hpp)

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

	// Ambient truth mirrors (cache, refreshed in the same post-step pass as
	// ax/ay/az): the air-relative Mach / dynamic pressure / air density /
	// local speed of sound that the aerodynamics actually integrated with at
	// the end of the step. Read-only for consumers (sensors, telemetry,
	// dashboards) — never treated as integrated state.
	std::vector<double> mach;              // air-relative Mach (V_airspeed / a)
	std::vector<double> dynamicPressure;   // q = 0.5 * rho * V_airspeed^2 (Pa)
	std::vector<double> airDensity;        // kg/m^3 (truth atmosphere at entity)
	std::vector<double> localSpeedOfSound; // m/s (truth atmosphere at entity)

	// Mass properties
	std::vector<double> Ixx, Iyy, Izz;     // principal moments of inertia (body)
	std::vector<double> Ixy, Ixz, Iyz;     // products of inertia (body)
	std::vector<double> mass;              // total mass (kg)
	std::vector<double> massDry;           // dry mass (kg); fuel = mass - massDry

	// Per-entity truth-model parameters (W1)
	std::vector<double> referenceArea;
	std::vector<double> referenceLength;
	std::vector<double> cd;
	std::vector<double> clAlpha;           // lift slope 1/rad
	std::vector<double> clFin;             // fin lift 1/rad (deflection)
	std::vector<double> clMax;             // max |CL| (stall / control limit)
	std::vector<bool>   tailControl;       // abstract-fin surface type (false = canard)
	std::vector<std::shared_ptr<const Models::AeroTables>> aeroTables;  // data-driven cd/cl tables; nullptr = flat coefficients
	std::vector<std::shared_ptr<const Models::FinsGeometry>> fins;     // geometric fins (primary / legacy); nullptr = abstract fins
	std::vector<std::vector<std::shared_ptr<const Models::FinsGeometry>>> finSets; // all geometric fin sets
	std::vector<std::shared_ptr<const Models::AirframeParams>> airframe; // aircraft wing-body-tail airframe (nullptr = missile)
	std::vector<int>    propulsionId;      // index into backend propulsion pool; -1 = none
	std::vector<double> ignitionTime;      // s (thrust curve evaluated at t - ignitionTime)
	std::vector<int>    stageIndex;        // active stage; -1 = coasting/finished
	std::vector<int>    stageCount;        // number of registered stages
	std::vector<double> stageMinMass;      // minimum mass for the current active stage; below
	                                       // it the stage's propellant is exhausted. For a
	                                       // coasting entity or uncapped stage it equals/falls
	                                       // below massDry; for a capped stage it is
	                                       // massDry + (propellant reserved for later stages).

	// Propulsion truth (achieved TVC gimbal angles, rad)
	std::vector<double> gimbalPitch, gimbalYaw;
	std::vector<double> maxGimbalPitchRad, maxGimbalYawRad;
	std::vector<double> gimbalTimeConstantSec, maxGimbalRateRadPerSec;
	std::vector<double> enginePositionX, enginePositionY, enginePositionZ;

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
	std::vector<bool> engineFailed;
	std::vector<bool> tankFailed;
	std::vector<bool> actuatorFailed;

	size_t size = 0;

	/**
	 * @brief Grows every vector to at least @p n entries.
	 *
	 * New slots receive the documented defaults; existing entries are
	 * preserved (the call is grow-only and never shrinks an array). Callers
	 * must invoke this whenever an entity slot is added so the block stays
	 * self-consistent. Every consumer indexes with a `i < vector.size()`
	 * guard, so a vector that is not grown here silently disables its
	 * subsystem for the newest entity.
	 */
	void ensureSize(std::size_t n) {
		growTo(px, n, 0.0); growTo(py, n, 0.0); growTo(pz, n, 0.0);
		growTo(vx, n, 0.0); growTo(vy, n, 0.0); growTo(vz, n, 0.0);
		growTo(ax, n, 0.0); growTo(ay, n, 0.0); growTo(az, n, 0.0);
		growTo(qw, n, 1.0); growTo(qx, n, 0.0); growTo(qy, n, 0.0); growTo(qz, n, 0.0);
		growTo(wx, n, 0.0); growTo(wy, n, 0.0); growTo(wz, n, 0.0);
		growTo(alphax, n, 0.0); growTo(alphay, n, 0.0); growTo(alphaz, n, 0.0);
		growTo(mach, n, 0.0);
		growTo(dynamicPressure, n, 0.0);
		growTo(airDensity, n, 0.0);
		growTo(localSpeedOfSound, n, 0.0);
		growTo(Ixx, n, 1.0); growTo(Iyy, n, 10.0); growTo(Izz, n, 10.0);
		growTo(Ixy, n, 0.0); growTo(Ixz, n, 0.0); growTo(Iyz, n, 0.0);
		growTo(mass, n, 1.0);
		growTo(massDry, n, 1.0);
		growTo(referenceArea, n, 0.1);
		growTo(referenceLength, n, 1.0);
		growTo(cd, n, 0.3);
		growTo(clAlpha, n, 0.0);
		growTo(clFin, n, 0.0);
		growTo(clMax, n, 2.0);
		growTo(tailControl, n, false);
		growTo(aeroTables, n, nullptr);
		growTo(fins, n, nullptr);
		growTo(finSets, n);
		growTo(airframe, n, nullptr);
		growTo(propulsionId, n, -1);
		growTo(ignitionTime, n, 0.0);
		growTo(stageIndex, n, -1);
		growTo(stageCount, n, 0);
		growTo(stageMinMass, n, 0.0);
		growTo(gimbalPitch, n, 0.0); growTo(gimbalYaw, n, 0.0);
		growTo(maxGimbalPitchRad, n, 0.0); growTo(maxGimbalYawRad, n, 0.0);
		growTo(gimbalTimeConstantSec, n, 0.02);
		growTo(maxGimbalRateRadPerSec, n, 0.0);
		growTo(enginePositionX, n, 0.0);
		growTo(enginePositionY, n, 0.0);
		growTo(enginePositionZ, n, 0.0);
		growTo(finPitch, n, 0.0); growTo(finYaw, n, 0.0); growTo(finRoll, n, 0.0);
		growTo(maxDeflectionRad, n, 0.43);
		growTo(servoTimeConstantSec, n, 0.02);
		growTo(maxServoRateRadPerSec, n, 5.24);
		growTo(active, n, true);
		growTo(motorFailed, n, false);
		growTo(engineFailed, n, false);
		growTo(tankFailed, n, false);
		growTo(actuatorFailed, n, false);
		if (n > size) size = n;
	}
};

} // namespace StrikeEngine::Kernel
