#pragma once

#include "strikeengine/designer/DesignPart.hpp"
#include <memory>
#include <string>

namespace StrikeEngine {

    /**
     * @brief Represents the complete Digital Mockup (DMU) of a vehicle.
     *
     * This class is the top-level container for a vehicle design. It holds the
     * root of the part hierarchy and provides methods for managing and analyzing
     * the entire assembly.
     */
    class VehicleModel {
    public:
        VehicleModel() = default;

        void setName(const std::string& name) { _name = name; }
        [[nodiscard]] const std::string& getName() const { return _name; }

        /**
         * @brief Sets the root part of the vehicle design (e.g., the first stage).
         * @param root_part A unique_ptr to the root DesignPart.
         */
        void setRootPart(std::unique_ptr<DesignPart> root_part) {
            _root_part = std::move(root_part);
        }

        /**
         * @brief Gets a pointer to the root part of the design.
         * @return A const pointer to the root DesignPart, or nullptr if none is set.
         */
        [[nodiscard]] const DesignPart* getRootPart() const {
            return _root_part.get();
        }

    private:
        std::string _name;
        std::unique_ptr<DesignPart> _root_part;
    };

} // namespace StrikeEngine
