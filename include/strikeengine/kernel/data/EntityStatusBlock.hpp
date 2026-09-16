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

        /**
         * @brief Grows every vector to @p n entries; see
         *        PhysicsBlock::ensureSize.
         */
        void ensureSize(std::size_t n) {
            type.resize(n, EntityType::Missile);
            allegiance.resize(n, Allegiance::Friendly);
            health.resize(n, 100.0);
            isAlive.resize(n, true);
            motorFailed.resize(n, false);
            engineFailed.resize(n, false);
            tankFailed.resize(n, false);
            actuatorFailed.resize(n, false);
            sensorFailed.resize(n, false);
            commsFailed.resize(n, false);
            name.resize(n);
            role.resize(n);
            rcsProfileId.resize(n);
            irProfileId.resize(n);
            emitterEirpW.resize(n, 0.0);
            jammerEirpW.resize(n, 0.0);
            size = n;
        }
    };

} // namespace StrikeEngine::Kernel
