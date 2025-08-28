#pragma once

#include "strikeengine/designer/VehicleModel.hpp"
#include <glm/glm.hpp>

namespace StrikeEngine {

    /**
     * @brief A data structure to hold the calculated mass properties of an entire vehicle assembly.
     */
    struct MassProperties {
        double total_mass_kg = 0.0;
        glm::dvec3 center_of_mass{0.0}; // Relative to the root part's origin
        glm::dmat3 inertia_tensor{1.0}; // About the vehicle's center of mass
    };

    /**
     * @brief A tool to analyze a VehicleModel and compute its overall mass properties.
     *
     * This class traverses the hierarchical assembly of a vehicle's Digital Mockup (DMU)
     * to calculate the total mass, center of mass location, and the
     * complete moment of inertia tensor for the entire vehicle.
     */
    class MassPropertiesCalculator {
    public:
        MassPropertiesCalculator() = default;

        /**
         * @brief Calculates the mass properties for a given vehicle model.
         * @param model The VehicleModel to analyze.
         * @return A MassProperties struct containing the results.
         */
        [[nodiscard]] MassProperties calculate(const VehicleModel& model) const;

    private:
        // A recursive helper function to traverse the part hierarchy.
        static void processPart(const DesignPart* part, MassProperties& props);
    };

} // namespace StrikeEngine
