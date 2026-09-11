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
     * (multi-stage: ignition and separation of spent stages), seeker,
     * sensors, guidance/autopilot gains, and warhead. Structural summary
     * fields (initialMass/massDry/inertias) are the designer output from
     * geometry.
     *
     * Passed to SimulationKernel::createVehicle(init, config). Non-empty
     * aeroProfileId / motorProfileId / seekerProfileId / sensorProfileId
     * reference profile files whose parsed config replaces the corresponding
     * inline sub-config (see include/strikeengine/kernel/profiles/).
     */
    struct VehicleConfig {
        EntityType type = EntityType::Missile;

        // Structural summary (designer output from geometry)
        double initialMass = -1.0;   // kg; <0 => launch mass = init.mass
        double massDry     = -1.0;   // kg; <0 => dry mass == launch mass (no fuel)
        double Ixx = 1.0, Iyy = 10.0, Izz = 10.0;
        double Ixy = 0.0, Ixz = 0.0, Iyz = 0.0;

        AeroConfig       aero;
        PropulsionConfig propulsion;
        SeekerConfig     seeker;
        SensorConfig     sensor;
        GuidanceAutopilotConfig guidanceAutopilot;
        WarheadConfig    warhead;

        std::string rcsProfileId = "";
        std::string irProfileId  = "";
        std::string aeroProfileId = "";
        std::string motorProfileId = "";
        std::string seekerProfileId = "";
        std::string sensorProfileId = "";
        std::string guidanceProfileId = "";
        std::string warheadProfileId = "";
        double emitterEirpW = 0.0;
        // Target-side RF jammer EIRP (W); degrades RF/SARH seeker SNR when
        // > 0 (assumed co-located with the target). 0 = no jammer.
        double jammerEirpW = 0.0;
    };

} // namespace StrikeEngine::Kernel
