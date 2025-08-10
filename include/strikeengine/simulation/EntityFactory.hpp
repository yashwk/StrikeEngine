#pragma once

#include "strikeengine/ecs/Entity.hpp"
#include <string>

namespace StrikeEngine {

    class Registry; // Forward-declaration

    /**
     * @brief Responsible for creating entities and attaching components from data profiles.
     *
     * This class reads entity definitions from JSON files, creates a new entity
     * in the registry, and initializes all of its components based on the data
     * in the profile. This allows for a fully data-driven entity creation pipeline.
     */
    class EntityFactory {
    public:
        /**
         * @brief Constructs the factory with a reference to the ECS registry.
         * @param registry The registry where new entities will be created.
         */
        explicit EntityFactory(Registry& registry);

        /**
         * @brief Creates a single entity from a JSON profile file.
         * @param profilePath The full path to the entity's JSON profile.
         * @return The Entity handle of the newly created entity.
         * @throws std::runtime_error if the file cannot be opened or is malformed.
         */
        Entity createFromProfile(const std::string& profilePath);

    private:
        Registry& _registry;
    };

} // namespace StrikeEngine
