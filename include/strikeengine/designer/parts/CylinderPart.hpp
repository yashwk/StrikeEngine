#pragma once

#include "strikeengine/designer/DesignPart.hpp"

namespace StrikeEngine {

    /**
     * @brief A concrete DesignPart representing a simple cylinder.
     *
     * Used for body tubes, couplers, and solid rocket motor casings.
     */
    class CylinderPart final : public DesignPart {
    public:
        CylinderPart(double length, double radius)
            : _length(length), _radius(radius) {}

        // --- Property Accessors ---
        [[nodiscard]] double getLength() const { return _length; }
        [[nodiscard]] double getRadius() const { return _radius; }

        // --- Overridden Methods ---
        [[nodiscard]] glm::dvec3 getDimensions() const override {
            return {_length, _radius * 2.0, _radius * 2.0};
        }

        [[nodiscard]] glm::dmat3 calculateInertiaTensor() const override {
            // Standard formula for a solid cylinder's moment of inertia.
            // Assumes uniform mass distribution.
            const double m = getMass();
            const double r2 = _radius * _radius;
            const double l2 = _length * _length;

            const double i_x = 0.5 * m * r2; // About the longitudinal axis
            const double i_y_z = (1.0 / 12.0) * m * (3.0 * r2 + l2);

            return {
                    {i_x, 0.0, 0.0},
                    {0.0, i_y_z, 0.0},
                    {0.0, 0.0, i_y_z}
            };
        }

    private:
        double _length;
        double _radius;
    };

} // namespace StrikeEngine
