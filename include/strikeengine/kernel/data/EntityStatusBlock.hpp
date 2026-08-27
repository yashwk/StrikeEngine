#pragma once
#include <vector>
#include <cstdint>
#include <string>

namespace StrikeEngine::Kernel {

    enum class EntityType : uint8_t {
        Missile,
        Aircraft,
        SurfaceTarget,
        RadarSite
    };

    enum class Allegiance : uint8_t {
        Friendly,
        Hostile,
        Neutral
    };

    enum class FailureMode : uint8_t { None, MotorFailure, ActuatorFailure, SensorFailure, StructuralFailure, CommunicationFailure };

    struct EntityStatusBlock {
        std::vector<EntityType> type;
        std::vector<Allegiance> allegiance;
        std::vector<double> health;
        std::vector<bool> isAlive;

        // Deterministic failure flags (set/cleared by the kernel; NOT part of
        // numerical integration).
        std::vector<bool> motorFailed;
        std::vector<bool> actuatorFailed;
        std::vector<bool> sensorFailed;
        std::vector<bool> commsFailed;
        // Structural failure is represented by isAlive=false + health=0.

        // Target Signature Metadata
        std::vector<std::string> rcsProfileId;
        std::vector<std::string> irProfileId;

        std::size_t size = 0;
    };

} // namespace StrikeEngine::Kernel
