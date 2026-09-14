#pragma once
#include <vector>
#include <cstdint>
#include <string>

namespace StrikeEngine::Kernel {

    enum class EntityType : uint8_t {
        Missile,
        Aircraft,
        SurfaceTarget,
        RadarSite,
        Chaff,  // RF decoy: carries an rcsProfileId
        Flare   // IR decoy: carries an irProfileId
    };

    enum class Allegiance : uint8_t {
        Friendly,
        Hostile,
        Neutral
    };

    enum class FailureMode : uint8_t {
        None, MotorFailure, EngineFailure, TankFailure, ActuatorFailure,
        SensorFailure, StructuralFailure, CommunicationFailure
    };

    struct EntityStatusBlock {
        std::vector<EntityType> type;
        std::vector<Allegiance> allegiance;
        std::vector<double> health;
        std::vector<bool> isAlive;

        // Deterministic failure flags (set/cleared by the kernel; NOT part of
        // numerical integration).
        std::vector<bool> motorFailed;
        std::vector<bool> engineFailed;
        std::vector<bool> tankFailed;
        std::vector<bool> actuatorFailed;
        std::vector<bool> sensorFailed;
        std::vector<bool> commsFailed;
        // Structural failure is represented by isAlive=false + health=0.

        // Target Signature Metadata
        std::vector<std::string> name;
        std::vector<std::string> role;
        std::vector<std::string> rcsProfileId;
        std::vector<std::string> irProfileId;
        // Target-side effective radiated power (W); used by PassiveRF seekers.
        // 0 = no emitter.
        std::vector<double> emitterEirpW;
        // Target-side RF jammer EIRP (W); degrades RF/SARH seeker SNR when > 0.
        std::vector<double> jammerEirpW;

        std::size_t size = 0;
    };

} // namespace StrikeEngine::Kernel
