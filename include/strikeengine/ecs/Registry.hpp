#pragma once

#include "Component.hpp"
#include "Entity.hpp"
#include <vector>
#include <unordered_map>
#include <memory>
#include <typeindex>
#include <stdexcept>
#include <algorithm>
#include <cassert>
#include <ranges>
#include <tuple>

namespace StrikeEngine {

    class IComponentPool {
    public:
        virtual ~IComponentPool() = default;
        virtual void removeFor(Entity entity) = 0;
    };

    template<typename T>
    class ComponentPool final : public IComponentPool {
    public:
        T& add(const Entity entity, T component) {
            assert(!_entityToIndexMap.contains(entity) && "Component already exists on entity.");
            const size_t newIndex = _components.size();
            _entityToIndexMap[entity] = newIndex;
            _indexToEntityMap[newIndex] = entity;
            _components.push_back(std::move(component));
            return _components.back();
        }

        void removeFor(const Entity entity) override {
            if (!_entityToIndexMap.contains(entity)) return;

            size_t indexOfRemoved = _entityToIndexMap.at(entity);
            size_t indexOfLast = _components.size() - 1;
            const Entity entityOfLast = _indexToEntityMap.at(indexOfLast);

            _components[indexOfRemoved] = std::move(_components[indexOfLast]);
            _entityToIndexMap[entityOfLast] = indexOfRemoved;
            _indexToEntityMap[indexOfRemoved] = entityOfLast;

            _entityToIndexMap.erase(entity);
            _indexToEntityMap.erase(indexOfLast);
            _components.pop_back();
        }

        T& get(const Entity entity) {
            assert(_entityToIndexMap.contains(entity) && "Component does not exist on entity.");
            return _components.at(_entityToIndexMap.at(entity));
        }

        const T& get(const Entity entity) const {
             assert(_entityToIndexMap.contains(entity) && "Component does not exist on entity.");
            return _components.at(_entityToIndexMap.at(entity));
        }

        [[nodiscard]] bool has(const Entity entity) const {
            return _entityToIndexMap.contains(entity);
        }

    private:
        std::vector<T> _components;
        std::unordered_map<Entity, size_t> _entityToIndexMap{};
        std::unordered_map<size_t, Entity> _indexToEntityMap{};
    };

    class Registry {
    public:
        Entity createEntity() {
            const Entity newEntity{_nextEntityId++};
            _entities.push_back(newEntity);
            return newEntity;
        }

        void destroyEntity(const Entity entity) {
            for (const auto &pool: _componentPools | std::views::values) {
                pool->removeFor(entity);
            }
            std::erase(_entities, entity);
        }

        template<typename T, typename... Args>
        T& add(Entity entity, Args&&... args) {
            static_assert(std::is_base_of_v<Component, T>, "T must inherit from Component");
            return getPool<T>().add(entity, T{std::forward<Args>(args)...});
        }

        template<typename T>
        void remove(Entity entity) {
            static_assert(std::is_base_of_v<Component, T>, "T must inherit from Component");
            getPool<T>().removeFor(entity);
        }

        template<typename T>
        T& get(Entity entity) {
            static_assert(std::is_base_of_v<Component, T>, "T must inherit from Component");
            return getPool<T>().get(entity);
        }

        template<typename T>
        const T& get(Entity entity) const {
            static_assert(std::is_base_of_v<Component, T>, "T must inherit from Component");
            return getPool<T>().get(entity);
        }

        template<typename T>
        [[nodiscard]] bool has(Entity entity) const {
            static_assert(std::is_base_of_v<Component, T>, "T must inherit from Component");
            const auto it = _componentPools.find(std::type_index(typeid(T)));
            if (it == _componentPools.end()) return false;

            const auto* pool = static_cast<const ComponentPool<T>*>(it->second.get());
            return pool->has(entity);
        }

        template<typename... ComponentTypes>
        auto view() {
            std::vector<std::tuple<Entity, ComponentTypes&...>> result;
            for (const auto entity : _entities) {
                if ((has<ComponentTypes>(entity) && ...)) {
                    result.emplace_back(entity, get<ComponentTypes>(entity)...);
                }
            }
            return result;
        }

    private:
        template<typename T>
        ComponentPool<T>& getPool() {
            const auto typeId = std::type_index(typeid(T));
            if (!_componentPools.contains(typeId)) {
                _componentPools[typeId] = std::make_unique<ComponentPool<T>>();
            }
            return *static_cast<ComponentPool<T>*>(_componentPools.at(typeId).get());
        }

        template<typename T>
        const ComponentPool<T>& getPool() const {
            const auto typeId = std::type_index(typeid(T));
            assert(_componentPools.contains(typeId) && "Requesting non-existent component pool from const registry.");
            return *static_cast<const ComponentPool<T>*>(_componentPools.at(typeId).get());
        }

        uint32_t _nextEntityId = 0;
        std::vector<Entity> _entities;
        std::unordered_map<std::type_index, std::unique_ptr<IComponentPool>> _componentPools;
    };

} // namespace StrikeEngine
