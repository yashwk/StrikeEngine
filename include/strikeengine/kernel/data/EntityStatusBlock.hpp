#pragma once
#include <strikeengine/kernel/data/BlockGrowth.hpp>
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

        /**
         * @brief Grows every vector to @p n entries; see
         *        PhysicsBlock::ensureSize.
         */
        void ensureSize(std::size_t n) {
            growTo(type, n, EntityType::Missile);
            growTo(allegiance, n, Allegiance::Friendly);
            growTo(health, n, 100.0);
            growTo(isAlive, n, true);
            growTo(motorFailed, n, false);
            growTo(engineFailed, n, false);
            growTo(tankFailed, n, false);
            growTo(actuatorFailed, n, false);
            growTo(sensorFailed, n, false);
            growTo(commsFailed, n, false);
            growTo(name, n);
            growTo(role, n);
            growTo(rcsProfileId, n);
            growTo(irProfileId, n);
            growTo(emitterEirpW, n, 0.0);
            growTo(jammerEirpW, n, 0.0);
            if (n > size) size = n;
        }
    };

} // namespace StrikeEngine::Kernel
