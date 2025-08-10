#include "strikeengine/systems/physics/GravitySystem.hpp"
#include "strikeengine/ecs/Registry.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/physics/MassComponent.hpp"
#include "strikeengine/components/physics/ForceAccumulatorComponent.hpp"

namespace StrikeEngine {
    // Physics Constants (WGS 84 standard)
    constexpr double GRAVITATIONAL_CONSTANT = 6.67430e-11; /// m^3 kg^-1 s^-2
    constexpr double EARTH_MASS_KG = 5.97219e24; /// kg

    void GravitySystem::update(Registry &registry, double dt) {
        // Get a view of all entities that have the components we need.
        auto view = registry.view<TransformComponent, MassComponent, ForceAccumulatorComponent>();
        for (auto [entity, transform, mass, accumulator]: view) {
            // Calculate the distance from the center of the Earth.
            double distance_from_center = glm::length(transform.position);

            // Avoid division by zero if an object is at the exact center of the Earth.
            if (distance_from_center < 1.0) {
                continue;
            }

            // Calculate the size of the gravitational force using Newton's law.
            // F = G * (m1 * m2) / r^2
            double force_magnitude = (GRAVITATIONAL_CONSTANT * EARTH_MASS_KG * mass.currentMass_kg) /
                                     (distance_from_center * distance_from_center);

            // Determine the direction of the force (towards the Earth's center at 0,0,0).
            glm::dvec3 force_direction = -glm::normalize(transform.position);

            // Calculate the final force vector.
            glm::dvec3 gravity_force = force_direction * force_magnitude;

            // Add the calculated force to the entity's force accumulator.
            accumulator.totalForce += gravity_force;
        }
    }
} // namespace StrikeEngine
