#include "strikeengine/designer/analysis/MassPropertiesCalculator.hpp"

namespace StrikeEngine {

    MassProperties MassPropertiesCalculator::calculate(const VehicleModel& model) const {
        MassProperties aggregate_properties;
        const DesignPart* root_part = model.getRootPart();

        if (root_part) {
            // Start the recursive calculation from the root part.
            processPart(root_part, aggregate_properties);

            // After summing all parts, the inertia tensor is still about the origin.
            // We must now translate it to be about the final, calculated center of mass.
            const glm::dvec3& com = aggregate_properties.center_of_mass;
            const double m_total = aggregate_properties.total_mass_kg;

            // Parallel Axis Theorem: I_com = I_origin - M * [(r.r)E - r*r^T]
            // where r is the vector from the origin to the CoM.
            const double rx2 = com.x * com.x;
            const double ry2 = com.y * com.y;
            const double rz2 = com.z * com.z;
            const double rxy = com.x * com.y;
            const double rxz = com.x * com.z;
            const double ryz = com.y * com.z;

            glm::dmat3 com_offset_tensor(
                ry2 + rz2, -rxy, -rxz,
                -rxy, rx2 + rz2, -ryz,
                -rxz, -ryz, rx2 + ry2
            );

            aggregate_properties.inertia_tensor -= m_total * com_offset_tensor;
        }

        return aggregate_properties;
    }

    void MassPropertiesCalculator::processPart(const DesignPart* part, MassProperties& props) const {
        const double part_mass = part->getMass();
        const glm::dvec3& part_pos = part->getRelativePosition();

        // --- 1. Update Total Mass and Center of Mass ---
        // The new center of mass is the weighted average of the old CoM and the new part's position.
        props.center_of_mass = (props.center_of_mass * props.total_mass_kg + part_pos * part_mass) / (props.total_mass_kg + part_mass);
        props.total_mass_kg += part_mass;

        // --- 2. Update Inertia Tensor ---
        // Get the part's inertia tensor about its own center of mass.
        glm::dmat3 part_inertia_local = part->calculateInertiaTensor();

        // Use the Parallel Axis Theorem to translate the part's local inertia
        // to be relative to the assembly's origin.
        // I_origin = I_com + M * [(r.r)E - r*r^T]
        // where r is the vector from the origin to the part's CoM.
        const double rx2 = part_pos.x * part_pos.x;
        const double ry2 = part_pos.y * part_pos.y;
        const double rz2 = part_pos.z * part_pos.z;
        const double rxy = part_pos.x * part_pos.y;
        const double rxz = part_pos.x * part_pos.z;
        const double ryz = part_pos.y * part_pos.z;

        glm::dmat3 offset_tensor(
            ry2 + rz2, -rxy, -rxz,
            -rxy, rx2 + rz2, -ryz,
            -rxz, -ryz, rx2 + ry2
        );

        glm::dmat3 part_inertia_origin = part_inertia_local + part_mass * offset_tensor;

        // Add this part's contribution to the total inertia tensor.
        props.inertia_tensor += part_inertia_origin;

        // --- 3. Recurse for all children ---
        for (const auto& child : part->getChildren()) {
            processPart(child.get(), props);
        }
    }

} // namespace StrikeEngine
