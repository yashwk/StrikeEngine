#pragma once

#include <string>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>
#include <strikeengine/kernel/config/AeroConfig.hpp>
#include <strikeengine/kernel/config/PropulsionConfig.hpp>
#include <strikeengine/kernel/config/SensorConfig.hpp>
#include <strikeengine/kernel/config/GuidanceAutopilotConfig.hpp>
#include <strikeengine/kernel/config/WarheadConfig.hpp>
#include <strikeengine/kernel/config/SeekerConfig.hpp>

namespace StrikeEngine::Kernel {

    /**
     * @brief Per-vehicle truth-model configuration (flattened simulation view).
     *
     * Composition of the vehicle's subsystem configurations: aero, propulsion
     * (single-stage for now; multi-stage ignition/separation is a later
     * increment), seeker, sensors, guidance/autopilot gains, and warhead.
     * Structural summary fields (initialMass/massDry/inertias) are the
     * designer output from geometry.
     *
     * Passed to SimulationKernel::createVehicle(init, config). Lookups of
     * aero/motor/seek/sensor profiles from a profile-id database are a future
     * layer on top of this flat struct.
     */
    struct VehicleConfig {
        EntityType type = EntityType::Missile;

        // Structural summary (designer output from geometry)
        double initialMass = -1.0;   // kg; <0 => launch mass = init.mass
        double massDry     = -1.0;   // kg; <0 => dry mass == launch mass (no fuel)
        double Ixx = 1.0, Iyy = 10.0, Izz = 10.0;

        AeroConfig       aero;
        PropulsionConfig propulsion;
        SeekerConfig     seeker;
        SensorConfig     sensor;
        GuidanceAutopilotConfig guidanceAutopilot;
        WarheadConfig    warhead;

        std::string rcsProfileId = "";
        std::string irProfileId  = "";
        double emitterEirpW = 0.0;
    };

} // namespace StrikeEngine::Kernel
