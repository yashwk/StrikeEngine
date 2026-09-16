#pragma once
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
	 * @brief Grows every vector to @p n entries.
	 *
	 * New slots receive the documented defaults; existing entries are
	 * preserved. Callers must invoke this whenever an entity slot is added so
	 * the block stays self-consistent. Every consumer indexes with a
	 * `i < vector.size()` guard, so a vector that is not grown here silently
	 * disables its subsystem for the newest entity.
	 */
	void ensureSize(std::size_t n) {
		px.resize(n, 0.0); py.resize(n, 0.0); pz.resize(n, 0.0);
		vx.resize(n, 0.0); vy.resize(n, 0.0); vz.resize(n, 0.0);
		ax.resize(n, 0.0); ay.resize(n, 0.0); az.resize(n, 0.0);
		qw.resize(n, 1.0); qx.resize(n, 0.0); qy.resize(n, 0.0); qz.resize(n, 0.0);
		wx.resize(n, 0.0); wy.resize(n, 0.0); wz.resize(n, 0.0);
		alphax.resize(n, 0.0); alphay.resize(n, 0.0); alphaz.resize(n, 0.0);
		mach.resize(n, 0.0);
		dynamicPressure.resize(n, 0.0);
		airDensity.resize(n, 0.0);
		localSpeedOfSound.resize(n, 0.0);
		Ixx.resize(n, 1.0); Iyy.resize(n, 10.0); Izz.resize(n, 10.0);
		Ixy.resize(n, 0.0); Ixz.resize(n, 0.0); Iyz.resize(n, 0.0);
		mass.resize(n, 1.0);
		massDry.resize(n, 1.0);
		referenceArea.resize(n, 0.1);
		referenceLength.resize(n, 1.0);
		cd.resize(n, 0.3);
		clAlpha.resize(n, 0.0);
		clFin.resize(n, 0.0);
		clMax.resize(n, 2.0);
		tailControl.resize(n, false);
		aeroTables.resize(n, nullptr);
		fins.resize(n, nullptr);
		finSets.resize(n);
		airframe.resize(n, nullptr);
		propulsionId.resize(n, -1);
		ignitionTime.resize(n, 0.0);
		stageIndex.resize(n, -1);
		stageCount.resize(n, 0);
		stageMinMass.resize(n, 0.0);
		gimbalPitch.resize(n, 0.0); gimbalYaw.resize(n, 0.0);
		maxGimbalPitchRad.resize(n, 0.0); maxGimbalYawRad.resize(n, 0.0);
		gimbalTimeConstantSec.resize(n, 0.02);
		maxGimbalRateRadPerSec.resize(n, 0.0);
		enginePositionX.resize(n, 0.0);
		enginePositionY.resize(n, 0.0);
		enginePositionZ.resize(n, 0.0);
		finPitch.resize(n, 0.0); finYaw.resize(n, 0.0); finRoll.resize(n, 0.0);
		maxDeflectionRad.resize(n, 0.43);
		servoTimeConstantSec.resize(n, 0.02);
		maxServoRateRadPerSec.resize(n, 5.24);
		active.resize(n, true);
		motorFailed.resize(n, false);
		engineFailed.resize(n, false);
		tankFailed.resize(n, false);
		actuatorFailed.resize(n, false);
		size = n;
	}
};

} // namespace StrikeEngine::Kernel
