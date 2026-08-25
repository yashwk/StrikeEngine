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

    struct EntityStatusBlock {
        std::vector<EntityType> type;
        std::vector<Allegiance> allegiance;
        std::vector<double> health;
        std::vector<bool> isAlive;

        // Target Signature Metadata
        std::vector<std::string> rcsProfileId;
        std::vector<std::string> irProfileId;

        std::size_t size = 0;
    };

} // namespace StrikeEngine::Kernel
