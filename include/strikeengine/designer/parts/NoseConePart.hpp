#pragma once

#include "strikeengine/designer/DesignPart.hpp"

namespace StrikeEngine {

    /**
     * @brief An enum to define the geometric shape of the nose cone.
     */
    enum class NoseConeShape {
        CONICAL,
        OGIVE,
        PARABOLIC
    };

    /**
     * @brief A concrete DesignPart representing a nose cone.
     */
    class NoseConePart final : public DesignPart {
    public:
        NoseConePart(double length, double base_radius, NoseConeShape shape)
            : _length(length), _base_radius(base_radius), _shape(shape) {}

        // --- Property Accessors ---
        [[nodiscard]] double getLength() const { return _length; }
        [[nodiscard]] double getBaseRadius() const { return _base_radius; }
        [[nodiscard]] NoseConeShape getShape() const { return _shape; }

        // --- Overridden Methods ---
        [[nodiscard]] glm::dvec3 getDimensions() const override {
            return {_length, _base_radius * 2.0, _base_radius * 2.0};
        }

        [[nodiscard]] glm::dmat3 calculateInertiaTensor() const override {
            // Formula for a solid cone's moment of inertia (as a simplification for all shapes).
            // A more advanced implementation would have different formulas for each shape.
            const double m = getMass();
            const double r2 = _base_radius * _base_radius;
            const double l2 = _length * _length;

            const double i_x = (3.0 / 10.0) * m * r2; // About the longitudinal axis
            const double i_y_z = (3.0 / 20.0) * m * (r2 + 4.0 * l2);

            return {
                    {i_x, 0.0, 0.0},
                    {0.0, i_y_z, 0.0},
                    {0.0, 0.0, i_y_z}
            };
        }

    private:
        double _length;
        double _base_radius;
        NoseConeShape _shape;
    };

} // namespace StrikeEngine
