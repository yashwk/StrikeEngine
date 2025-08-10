#include "strikeengine/designer/PartLoader.hpp"
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

namespace StrikeEngine {

    using json = nlohmann::json;

    // Helper functions for JSON deserialization
    void from_json(const json& j, MaterialDefinition& m) {
        j.at("id").get_to(m.id);
        j.at("name").get_to(m.name);
        j.at("density_kg_m3").get_to(m.density_kg_m3);
    }

    void from_json(const json& j, PartDefinition& p) {
        j.at("part_id").get_to(p.part_id);
        j.at("name").get_to(p.name);
        j.at("type").get_to(p.type);
        j.at("material_id").get_to(p.material_id);
        j.at("geometry").get_to(p.geometry);
    }

    void PartLoader::loadMaterialLibrary(const std::string& filepath) {
        std::ifstream f(filepath);
        if (!f.is_open()) {
            throw std::runtime_error("PartLoader: Could not open material library file: " + filepath);
        }

        json data = json::parse(f);
        auto materials = data.at("materials").get<std::vector<MaterialDefinition>>();
        for (const auto& mat : materials) {
            _materials[mat.id] = mat;
        }
    }

    void PartLoader::loadPartLibrary(const std::string& filepath) {
        std::ifstream f(filepath);
        if (!f.is_open()) {
            throw std::runtime_error("PartLoader: Could not open part library file: " + filepath);
        }

        json data = json::parse(f);
        auto parts = data.at("parts").get<std::vector<PartDefinition>>();
        for (const auto& part : parts) {
            _parts[part.part_id] = part;
        }
    }

    const MaterialDefinition& PartLoader::getMaterial(const std::string& material_id) const {
        return _materials.at(material_id);
    }

    const PartDefinition& PartLoader::getPart(const std::string& part_id) const {
        return _parts.at(part_id);
    }

    const std::map<std::string, PartDefinition>& PartLoader::getAllParts() const {
        return _parts;
    }

} // namespace StrikeEngine
