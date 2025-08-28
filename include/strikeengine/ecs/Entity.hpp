#pragma once
#include <cstdint>
#include <functional>
#include <limits>

namespace StrikeEngine {
    class Entity {
    public:
        // Use a 64-bit integer to store both the index and the version.
        using IDType = uint64_t;

        constexpr Entity() : _id(std::numeric_limits<IDType>::max())
        {
        }

        explicit Entity(IDType id) : _id(id)
        {
        }

        Entity(uint32_t index, uint32_t version) : _id((static_cast<IDType>(version) << 32) | index)
        {
        }

        // Get the index part of the ID (the lower 32 bits).
        [[nodiscard]] uint32_t index() const
        {
            return static_cast<uint32_t>(_id & 0xFFFFFFFF);
        }

        // Get the version part of the ID (the upper 32 bits).
        [[nodiscard]] uint32_t version() const
        {
            return static_cast<uint32_t>(_id >> 32);
        }

        // Overload operators for easy comparison and use in maps.
        bool operator==(const Entity& other) const { return _id == other._id; }
        bool operator!=(const Entity& other) const { return _id != other._id; }
        bool operator<(const Entity& other) const { return _id < other._id; }

        // Allow casting to the underlying ID type.
        explicit operator IDType() const { return _id; }

    private:
        IDType _id;
    };

    constexpr Entity NULL_ENTITY = {};
} // namespace StrikeEngine

template <>
struct std::hash<StrikeEngine::Entity> {
    size_t operator()(const StrikeEngine::Entity& entity) const noexcept
    {
        return hash<StrikeEngine::Entity::IDType>()(static_cast<StrikeEngine::Entity::IDType>(entity));
    }
};
