#include "strikeengine/systems/physics/IntegrationSystem.hpp"
#include "strikeengine/ecs/Registry.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/physics/VelocityComponent.hpp"
#include "strikeengine/components/physics/MassComponent.hpp"
#include "strikeengine/components/physics/InertiaComponent.hpp"
#include "strikeengine/components/physics/ForceAccumulatorComponent.hpp"
#include <glm/gtc/quaternion.hpp>

namespace StrikeEngine {

    void IntegrationSystem::update(Registry& registry, double dt) {
        auto view = registry.view<TransformComponent, VelocityComponent, MassComponent, InertiaComponent, ForceAccumulatorComponent>();

        for (auto [entity, transform, velocity, mass, inertia, accumulator] : view) {
            if (mass.inverseMass <= 0.0) { // Skip objects with infinite mass
                accumulator.clear();
                continue;
            }

            // --- 1. Calculate Acceleration from Accumulated Forces ---
            const glm::dvec3 linear_acceleration = accumulator.totalForce * mass.inverseMass;

            const glm::dvec3 angular_velocity_world = transform.orientation * velocity.angular;
            const glm::dvec3 angular_acceleration_world = inertia.inverseInertiaTensor *
                (accumulator.totalTorque - glm::cross(angular_velocity_world, inertia.inertiaTensor * angular_velocity_world));

            // --- 2. Integrate using Semi-Implicit Euler Method ---
            // This method is more stable than standard Euler because it uses the *new*
            // velocity to update the position.

            // First, update the velocities over the time step.
            velocity.linear += linear_acceleration * dt;
            velocity.angular += glm::inverse(transform.orientation) * angular_acceleration_world * dt; // Convert world accel to body space for storage

            // Second, update the position and orientation using the *newly calculated* velocities.
            transform.position += velocity.linear * dt;

            // Update orientation based on the new angular velocity.
            // Create a quaternion representing the rotation over the time step.
            glm::dvec3 rotation_axis = velocity.angular;
            double angle = glm::length(rotation_axis) * dt;
            if (angle > 0.0) {
                rotation_axis = glm::normalize(rotation_axis);
                glm::dquat rotation_delta = glm::angleAxis(angle, rotation_axis);
                transform.orientation = glm::normalize(rotation_delta * transform.orientation);
            }

            // --- Clear the Force Accumulator ---
            accumulator.clear();
        }
    }

} // namespace StrikeEngine
