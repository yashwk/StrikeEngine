#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <string>
#include <strikeengine/kernel/data/GuidanceBlock.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>
#include <strikeengine/kernel/config/VehicleInitState.hpp>

namespace StrikeEngine::Kernel {

    class SimulationKernel;  // loadInto body lives in ConfigSerialization.cpp

    // Represents an initial state and configuration for an entity in a scenario
    struct ScenarioEntityConfig {
        VehicleInitState initState;
        VehicleConfig vehicleConfig;

        // Display name / callsign and operational role (e.g. "interceptor", "target", "escort")
        std::string name = "";
        std::string role = "";

        // Path to a design file (JSON). During scenario deserialization a
        // non-empty designRef is resolved via loadDesignPhysics and OVERRIDES
        // any inline vehicleConfig. loadInto does not resolve it.
        std::string designRef = "";

        GuidanceMode initialGuidanceMode = GuidanceMode::None;
        
        double initialTargetX = 0.0;
        double initialTargetY = 0.0;
        double initialTargetZ = 0.0;

        // Target velocity. ProNav needs a correct closing velocity against a
        // moving target (LOS-rate guidance derives omega from relative
        // velocity); defaults keep legacy scenarios stationary-aimpoint.
        double initialTargetVx = 0.0;
        double initialTargetVy = 0.0;
        double initialTargetVz = 0.0;
        double initialMaxAccel = 0.0;

        // Target acceleration for APN feed-forward (augmented PN). Used only
        // when initialTargetAccelAvailable is true; see SimulationCommand.
        double initialTargetAccelX = 0.0;
        double initialTargetAccelY = 0.0;
        double initialTargetAccelZ = 0.0;
        bool   initialTargetAccelAvailable = false;

        // Optional target identity for the persistent track; -1 unknown
        // (the seeker supplies identity once it locks).
        std::int64_t initialTargetId = -1;

        // Data-driven rail launch. When enabled, loadInto does NOT create
        // the entity; the kernel spawns it in-flight when the parent's
        // seeker has held lock for lockHoldSec and the parent->target slant
        // range is inside rangeGateM (0 = no gate, targetIndex -1 = any
        // lock). Spawn geometry models carriage release: parent position
        // shifted dropM along local up, parent velocity minus pushMps along
        // local up, parent attitude, zero body rates. This replaces
        // app-side hardcoded launch directors: the scenario file fully
        // declares the mission.
        struct LaunchSpec {
            bool enabled = false;
            std::size_t parentIndex = 0;
            std::int64_t targetIndex = -1;
            double dropM = 0.0;
            double pushMps = 0.0;
            double lockHoldSec = 0.0;
            double rangeGateM = 0.0;
            // Separation flyout phase (0 = off): after spawn, hold a
            // straight-ahead Waypoint (aim = spawn position + spawn
            // velocity direction * flyoutAheadM + local up * flyoutClimbM)
            // for flyoutSec, then switch to the entity's initial guidance
            // command seeded from the parent's relayed datalink track. This
            // is the booster-separation phase of a rail launch: the round
            // flies clean while the motor lights and the datalink solution
            // refines before homing starts.
            double flyoutSec = 0.0;
            double flyoutAheadM = 6000.0;
            double flyoutClimbM = 0.0;
            // Launch attitude. 0 = inherit the parent's attitude (air rail
            // launch, the legacy behaviour). > 0 = cold-launch attitude
            // pitched that many degrees above the horizon toward the target
            // (ground batteries: the round leaves the tube nose-up and the
            // midcourse loft takes it from there). pushMps is applied along
            // that launch axis instead of straight down.
            double launchElevationDeg = 0.0;
            // Cold-launch clearance axis (0 = the ejection axis is the launch
            // axis, no pitch-over). A vertical launch (90) pops the round out
            // of the canister and the gas thrusters then reorient it to
            // launchElevationDeg (the loft axis) before the main motor lights.
            // The kernel applies that reorientation at the first stage's
            // ignition delay, rotating the body and the ejection velocity
            // together -- no fin can pitch a cold round over at 40 m/s, which
            // is exactly why real vertical launchers carry thrusters.
            double ejectElevationDeg = 0.0;
            // Target reaction (0 = the target is unaware). The launch warns
            // the target and it runs: the kernel queues the target a level
            // Cruise course toward a point this far away on the reciprocal
            // bearing from the launcher, so a long-range round has to chase a
            // fleeing target instead of a cooperative one. The target's own
            // cruise law keeps its altitude; the escape is purely a heading
            // change at whatever thrust the airframe has.
            double targetEvadeDistanceM = 0.0;
        };
        LaunchSpec launch;
    };

    struct ScenarioConfig {
        std::string name;
        std::string description;

        EnvironmentConfig environment;
        
        std::vector<ScenarioEntityConfig> entities;

        // Entity selected by study wrappers for per-scenario summaries and
        // sweep/Monte Carlo rows. Zero preserves legacy behavior.
        std::size_t primaryEntityIndex = 0;

        // THE global seed for this scenario. loadInto copies it into the
        // kernel, which fans it out to every stochastic system inside
        // (sensor stream, nav alignment stream, split warhead stream). One
        // seed per scenario, no per-system seeding from outside.
        std::uint32_t randomSeed = 0xDEADBEEFu;

        // Serialize/deserialize this scenario to/from a JSON file.
        // save() returns false if the file cannot be opened; load() throws
        // std::runtime_error if the file is missing or the JSON is malformed.
        bool save(const std::string& path) const;
        static ScenarioConfig load(const std::string& path);

        // Apply this scenario to the given kernel
        void loadInto(SimulationKernel& kernel) const;
    };

} // namespace StrikeEngine::Kernel
