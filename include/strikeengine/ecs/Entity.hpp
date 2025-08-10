#pragma once

#include <cstdint>
#include <functional>

namespace StrikeEngine {
    /**
     * @brief A type-safe handle for an entity in the simulation.
     */
    struct Entity {
        uint32_t id;

        bool operator==(const Entity& other) const { return id == other.id; }
        bool operator!=(const Entity& other) const { return id != other.id; }
        bool operator<(const Entity& other) const { return id < other.id; }

        explicit operator uint32_t() const { return id; }
    };

    constexpr Entity NULL_ENTITY = { static_cast<uint32_t>(-1) };

} // namespace StrikeEngine

template<>
struct std::hash<StrikeEngine::Entity> {
    std::size_t operator()(const StrikeEngine::Entity& entity) const noexcept {
        return hash<uint32_t>()(entity.id);
    }
};
