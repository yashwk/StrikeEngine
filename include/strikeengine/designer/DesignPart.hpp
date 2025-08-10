#pragma once

#include <string>
#include <vector>
#include <memory>
#include <glm/glm.hpp>

namespace StrikeEngine {

    /**
     * @brief The abstract base class for all physical components in the StrikeDesigner.
     *
     * This class defines the common interface and properties for any part that can
     * be added to a vehicle's Digital Mockup, such as mass, dimensions, and its
     * position relative to its parent. It supports a hierarchical structure.
     */
    class DesignPart {
    public:
        virtual ~DesignPart() = default;

        // --- Core Properties ---
        void setName(const std::string& name) { _name = name; }
        [[nodiscard]] const std::string& getName() const { return _name; }

        void setMass(double mass_kg) { _mass_kg = mass_kg; }
        [[nodiscard]] double getMass() const { return _mass_kg; }

        void setRelativePosition(const glm::dvec3& position) { _relative_position = position; }
        [[nodiscard]] const glm::dvec3& getRelativePosition() const { return _relative_position; }

        // --- Hierarchy Management ---
        void addChild(std::unique_ptr<DesignPart> child) {
            _children.push_back(std::move(child));
        }
        [[nodiscard]] const std::vector<std::unique_ptr<DesignPart>>& getChildren() const {
            return _children;
        }

        // --- Abstract Methods ---
        [[nodiscard]] virtual glm::dvec3 getDimensions() const = 0;
        [[nodiscard]] virtual glm::dmat3 calculateInertiaTensor() const = 0;

    protected:
        std::string _name;
        double _mass_kg = 0.0;
        glm::dvec3 _relative_position{0.0}; // Position relative to parent

        std::vector<std::unique_ptr<DesignPart>> _children;
    };

} // namespace StrikeEngine