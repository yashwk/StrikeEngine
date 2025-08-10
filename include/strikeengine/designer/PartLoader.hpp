#pragma once

#include <string>
#include <vector>
#include <map>
#include "nlohmann/json.hpp"

namespace StrikeEngine {

    /**
     * @brief A data structure holding the properties of a defined material.
     */
    struct MaterialDefinition {
        std::string id;
        std::string name;
        double density_kg_m3 = 0.0;
    };

    /**
     * @brief A data structure holding the definition of a reusable part from the library.
     */
    struct PartDefinition {
        std::string part_id;
        std::string name;
        std::string type; // e.g., "NoseConePart", "CylinderPart"
        std::string material_id;
        nlohmann::json geometry; // Store geometry data as a flexible JSON object
    };

    /**
     * @brief Manages the loading and accessing of the Part and Material libraries.
     *
     * This class reads JSON definition files for materials and parts, providing a
     * central, queryable database for the StrikeDesigner application.
     */
    class PartLoader {
    public:
        PartLoader() = default;

        /**
         * @brief Loads the material library from a specified JSON file.
         * @param filepath Path to the materials.json file.
         */
        void loadMaterialLibrary(const std::string& filepath);

        /**
         * @brief Loads a part library from a specified JSON file.
         * @param filepath Path to a part definition file (e.g., nose_cones.json).
         */
        void loadPartLibrary(const std::string& filepath);

        /**
         * @brief Retrieves a material definition by its unique ID.
         * @param material_id The ID of the material (e.g., "aluminum_7075_t6").
         * @return A const reference to the MaterialDefinition.
         * @throws std::out_of_range if the ID is not found.
         */
        [[nodiscard]] const MaterialDefinition& getMaterial(const std::string& material_id) const;

        /**
         * @brief Retrieves a part definition by its unique ID.
         * @param part_id The ID of the part (e.g., "nc_ogive_50mm").
         * @return A const reference to the PartDefinition.
         * @throws std::out_of_range if the ID is not found.
         */
        [[nodiscard]] const PartDefinition& getPart(const std::string& part_id) const;

        /**
         * @brief Gets all loaded part definitions.
         * @return A const reference to the map of all parts.
         */
        [[nodiscard]] const std::map<std::string, PartDefinition>& getAllParts() const;

    private:
        std::map<std::string, MaterialDefinition> _materials;
        std::map<std::string, PartDefinition> _parts;
    };

} // namespace StrikeEngine
