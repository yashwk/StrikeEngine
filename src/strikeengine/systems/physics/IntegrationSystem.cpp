#include "strikeengine/systems/physics/IntegrationSystem.hpp"
#include "strikeengine/ecs/Registry.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/physics/VelocityComponent.hpp"
#include "strikeengine/components/physics/MassComponent.hpp"
#include "strikeengine/components/physics/InertiaComponent.hpp"
#include "strikeengine/components/physics/ForceAccumulatorComponent.hpp"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

namespace StrikeEngine {

    struct StateDerivative {
        glm::dvec3 velocity;        // dx/dt
        glm::dvec3 acceleration;    // dv/dt
    };

    // This helper function evaluates the forces acting on the body at a given state
    // and computes the resulting derivatives (accelerations).
    StateDerivative evaluate(const Registry& registry, const TransformComponent& transform, const VelocityComponent& velocity, const MassComponent& mass, const InertiaComponent& inertia, const ForceAccumulatorComponent& accumulator) {
        StateDerivative output{};
        output.velocity = velocity.linear;
        output.acceleration = accumulator.totalForce * mass.inverseMass;

        // Note: For a true RK4 on rotation, we would also calculate angular acceleration here
        // and return it. For simplicity and stability, we will integrate rotation separately
        // after the main RK4 position/velocity update.
        return output;
    }

    void IntegrationSystem::update(Registry& registry, double dt) {
        auto view = registry.view<TransformComponent, VelocityComponent, MassComponent, InertiaComponent, ForceAccumulatorComponent>();

        for (auto [entity, transform, velocity, mass, inertia, accumulator] : view) {
            if (mass.inverseMass <= 0.0) { // Skip static or immovable objects
                accumulator.clear();
                continue;
            }

            // --- RK4 Integration for Linear Motion (Position and Velocity) ---

            // k1: Evaluate at the beginning of the step (t)
            StateDerivative k1 = evaluate(registry, transform, velocity, mass, inertia, accumulator);

            // k2: Evaluate at the midpoint of the step (t + dt/2)
            VelocityComponent mid_vel_k2 = velocity;
            mid_vel_k2.linear += k1.acceleration * (dt * 0.5);
            StateDerivative k2 = evaluate(registry, transform, mid_vel_k2, mass, inertia, accumulator);

            // k3: Evaluate again at the midpoint (t + dt/2), but using k2's velocity
            VelocityComponent mid_vel_k3 = velocity;
            mid_vel_k3.linear += k2.acceleration * (dt * 0.5);
            StateDerivative k3 = evaluate(registry, transform, mid_vel_k3, mass, inertia, accumulator);

            // k4: Evaluate at the end of the step (t + dt)
            VelocityComponent end_vel_k4 = velocity;
            end_vel_k4.linear += k3.acceleration * dt;
            StateDerivative k4 = evaluate(registry, transform, end_vel_k4, mass, inertia, accumulator);

            // --- Update Final Linear State ---
            // Combine the derivatives using the RK4 weighted average to get the final position and velocity.
            transform.position += (dt / 6.0) * (k1.velocity + 2.0 * k2.velocity + 2.0 * k3.velocity + k4.velocity);
            velocity.linear += (dt / 6.0) * (k1.acceleration + 2.0 * k2.acceleration + 2.0 * k3.acceleration + k4.acceleration);


            // --- Integrate Rotational Motion ---
            // Rotational updates are kept separate for stability and clarity. We use the same
            // gyroscopic precession formula as the previous implementation.
            const glm::dvec3 angular_velocity_world = transform.orientation * velocity.angular;
            const glm::dvec3 angular_acceleration_world = inertia.inverseInertiaTensor *
                (accumulator.totalTorque - glm::cross(angular_velocity_world, inertia.inertiaTensor * angular_velocity_world));

            // Update angular velocity (convert world-space acceleration to body-space for storage)
            velocity.angular += glm::inverse(transform.orientation) * angular_acceleration_world * dt;

            // Update orientation using the new angular velocity
            glm::dvec3 rotation_axis = velocity.angular;
            double angle = glm::length(rotation_axis) * dt;
            if (angle > 0.0) {
                rotation_axis = glm::normalize(rotation_axis);
                glm::dquat rotation_delta = glm::angleAxis(angle, rotation_axis);
                transform.orientation = glm::normalize(rotation_delta * transform.orientation);
            }

            // --- Clear the Force Accumulator for the next frame ---
            accumulator.clear();
        }
    }

} // namespace StrikeEngine
