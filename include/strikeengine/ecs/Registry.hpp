#pragma once

#include "Entity.hpp"
#include <utility>
#include <vector>
#include <unordered_map>
#include <memory>
#include <typeinfo>
#include <stdexcept>
#include <iostream>
#include <deque>
#include <ranges>

namespace StrikeEngine {

    // --- Component Pool (Interface) ---
    class IComponentPool {
    public:
        virtual ~IComponentPool() = default;
        virtual void onEntityDestroyed(Entity entity) = 0;
    };

    // --- Component Pool (Implementation) ---
    template<typename T>
    class ComponentPool final : public IComponentPool {
    public:
        T& add(Entity entity, T component) {
            if (_entityToIndexMap.contains(entity)) {
                _components[_entityToIndexMap[entity]] = component;
                return _components[_entityToIndexMap[entity]];
            }
            size_t newIndex = _components.size();
            _entityToIndexMap[entity] = newIndex;
            _indexToEntityMap[newIndex] = entity;
            _components.push_back(component);
            return _components.back();
        }

        T& get(Entity entity) {
            if (!_entityToIndexMap.contains(entity)) {
                throw std::runtime_error("Component not found for entity.");
            }
            return _components[_entityToIndexMap[entity]];
        }

        [[nodiscard]] bool has(Entity entity) const {
            return _entityToIndexMap.contains(entity);
        }

        void onEntityDestroyed(Entity entity) override {
            if (!_entityToIndexMap.contains(entity)) {
                return;
            }
            // Efficiently remove a component by swapping with the last element
            size_t indexOfRemoved = _entityToIndexMap[entity];
            size_t indexOfLast = _components.size() - 1;
            Entity entityOfLast = _indexToEntityMap[indexOfLast];

            _components[indexOfRemoved] = _components[indexOfLast];
            _entityToIndexMap[entityOfLast] = indexOfRemoved;
            _indexToEntityMap[indexOfRemoved] = entityOfLast;

            _entityToIndexMap.erase(entity);
            _indexToEntityMap.erase(indexOfLast);
            _components.pop_back();
        }

        [[nodiscard]] std::vector<Entity> getEntities() const {
            std::vector<Entity> entities;
            entities.reserve(_entityToIndexMap.size());
            for(const auto& entity : _entityToIndexMap | std::views::keys) {
                entities.push_back(entity);
            }
            return entities;
        }

    private:
        std::vector<T> _components;
        std::unordered_map<Entity, size_t> _entityToIndexMap;
        std::unordered_map<size_t, Entity> _indexToEntityMap;
    };


    // --- Registry ---
    class Registry {
    public:
        Entity create() {
            uint32_t index;
            if (!_freeList.empty()) {
                index = _freeList.front();
                _freeList.pop_front();
            } else {
                index = _nextEntityIndex++;
                if (index >= _entityVersions.size()) {
                    _entityVersions.resize(index + 1, 1);
                }
            }
            return {index, _entityVersions[index]};
        }

        void destroy(Entity entity) {
            uint32_t index = entity.index();
            if (index >= _entityVersions.size() || _entityVersions[index] != entity.version()) {
                return; // Entity is already invalid
            }
            _entityVersions[index]++; // Invalidate all existing handles
            _freeList.push_back(index);

            // Notify all component pools to remove their data for this entity
            for (const auto& pool : _componentPools | std::views::values) {
                pool->onEntityDestroyed(entity);
            }
        }

        [[nodiscard]] bool isAlive(Entity entity) const {
            uint32_t index = entity.index();
            return index < _entityVersions.size() && _entityVersions[index] == entity.version();
        }

        template<typename T, typename... Args>
        T& add(Entity entity, Args&&... args) {
            if (!isAlive(entity)) {
                throw std::runtime_error("Cannot add component to a dead entity.");
            }
            return getComponentPool<T>()->add(entity, T{std::forward<Args>(args)...});
        }

        template<typename T>
        T& get(Entity entity) {
            if (!isAlive(entity)) {
                throw std::runtime_error("Cannot get component from a dead entity.");
            }
            return getComponentPool<T>()->get(entity);
        }

        template<typename T>
        bool has(Entity entity) {
            if (!isAlive(entity)) {
                return false;
            }
            const char* typeName = typeid(T).name();
            if (!_componentPools.contains(typeName)) {
                return false;
            }
            return getComponentPool<T>()->has(entity);
        }

        template<typename... Components>
        class View {
        public:
            explicit View(Registry& registry) : _registry(registry) {
                _entities = findEntitiesWithComponents();
            }

            struct Iterator {
                Iterator(Registry& registry, std::vector<Entity>::const_iterator it)
                    : _registry(registry), _it(std::move(it)) {}
                Entity operator*() const { return *_it; }
                Iterator& operator++() { ++_it; return *this; }
                bool operator!=(const Iterator& other) const { return _it != other._it; }
            private:
                Registry& _registry;
                std::vector<Entity>::const_iterator _it;
            };

            Iterator begin() { return Iterator(_registry, _entities.begin()); }
            Iterator end() { return Iterator(_registry, _entities.end()); }

            template<typename T>
            T& get(Entity entity) { return _registry.get<T>(entity); }
        private:
            Registry& _registry;
            std::vector<Entity> _entities;

            auto findEntitiesWithComponents() {
                std::vector<Entity> result;
                if constexpr (sizeof...(Components) > 0) {
                    // --- FIX: Changed auto& to auto ---
                    auto pool = _registry.getComponentPool<std::tuple_element_t<0, std::tuple<Components...>>>();
                    auto initialEntities = pool->getEntities();
                    for (Entity entity : initialEntities) {
                        if ((_registry.has<Components>(entity) && ...)) {
                            result.push_back(entity);
                        }
                    }
                }
                return result;
            }
        };

        template<typename... Components>
        View<Components...> view() {
            return View<Components...>(*this);
        }

    private:
        template<typename T>
        std::shared_ptr<ComponentPool<T>> getComponentPool() {
            const char* typeName = typeid(T).name();
            if (!_componentPools.contains(typeName)) {
                _componentPools[typeName] = std::make_shared<ComponentPool<T>>();
            }
            return std::static_pointer_cast<ComponentPool<T>>(_componentPools[typeName]);
        }

        uint32_t _nextEntityIndex = 0;
        std::deque<uint32_t> _freeList;
        std::vector<uint32_t> _entityVersions;
        std::unordered_map<const char*, std::shared_ptr<IComponentPool>> _componentPools;
    };
}
