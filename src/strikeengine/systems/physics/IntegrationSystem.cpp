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

    // Rigid body derivative
    struct RigidBodyDerivative {
        glm::dvec3 dposition;      // dr/dt
        glm::dvec3 dvelocity;      // dv/dt
        glm::dvec3 domega;         // dω/dt (body frame)
        glm::dquat dquaternion;    // dq/dt
    };

    // Evaluate derivatives at state
    static RigidBodyDerivative evaluate(
        const TransformComponent& transform,
        const VelocityComponent& velocity,
        const MassComponent& mass,
        const InertiaComponent& inertia,
        const ForceAccumulatorComponent& accumulator)
    {
        RigidBodyDerivative output{};

        // Translation
        output.dposition = velocity.getLinear();
        output.dvelocity = accumulator.getTotalForce() * mass.inverseMass();

        // Rotation (BODY FRAME)
        const glm::dvec3 omega = velocity.getAngular();
        const glm::dvec3 torque_body = glm::inverse(transform.orientation) * accumulator.getTotalTorque();
        const glm::dvec3 Iomega = inertia.getInertiaTensor() * omega;

        output.domega =inertia.getInverseInertiaTensor() * (torque_body - glm::cross(omega, Iomega));

        // Quaternion derivative
        const glm::dquat omega_q(0.0, omega.x, omega.y, omega.z);
        output.dquaternion = 0.5 * transform.orientation * omega_q;

        return output;
    }

    // Integration system update
    void IntegrationSystem::update(Registry& registry, double dt)
    {
        auto view = registry.view<
            TransformComponent,
            VelocityComponent,
            MassComponent,
            InertiaComponent,
            ForceAccumulatorComponent>();

        for (auto entity : view)
        {
            auto& transform   = view.get<TransformComponent>(entity);
            auto& velocity    = view.get<VelocityComponent>(entity);
            auto& mass        = view.get<MassComponent>(entity);
            auto& inertia     = view.get<InertiaComponent>(entity);
            auto& accumulator = view.get<ForceAccumulatorComponent>(entity);

            if (mass.inverseMass() <= 0.0)
            {
                accumulator.clear();
                continue;
            }

            // initial state
            TransformComponent t0 = transform;
            VelocityComponent  v0 = velocity;

            // RK4 Time Integration

            // RK4 Step 1
            RigidBodyDerivative k1 = evaluate(transform, velocity, mass, inertia, accumulator);

            // RK4 Step 2
            transform.position    = t0.position + 0.5 * dt * k1.dposition;
            transform.orientation = glm::normalize(t0.orientation + 0.5 * dt * k1.dquaternion);

            velocity.setLinear(v0.getLinear() + 0.5 * dt * k1.dvelocity);
            velocity.setAngular(v0.getAngular() + 0.5 * dt * k1.domega);

            RigidBodyDerivative k2 = evaluate(transform, velocity, mass, inertia, accumulator);

            // RK4 Step 3
            transform.position    = t0.position + 0.5 * dt * k2.dposition;
            transform.orientation = glm::normalize(t0.orientation + 0.5 * dt * k2.dquaternion);
            velocity.setLinear(v0.getLinear() + 0.5 * dt * k2.dvelocity);
            velocity.setAngular(v0.getAngular() + 0.5 * dt * k2.domega);

            RigidBodyDerivative k3 = evaluate(transform, velocity, mass, inertia, accumulator);

            // RK4 Step 4
            transform.position    = t0.position + dt * k3.dposition;
            transform.orientation = glm::normalize(t0.orientation + dt * k3.dquaternion);
            velocity.setLinear(v0.getLinear() + dt * k3.dvelocity);
            velocity.setAngular(v0.getAngular() + dt * k3.domega);

            RigidBodyDerivative k4 = evaluate(transform, velocity, mass, inertia, accumulator);

            // -----------------------------
            // Final RK4 combine
            // -----------------------------
            transform.position =
                t0.position +
                (dt / 6.0) *
                (k1.dposition + 2.0 * k2.dposition + 2.0 * k3.dposition + k4.dposition);

            velocity.setLinear(
                v0.getLinear() +
                (dt / 6.0) *
                (k1.dvelocity + 2.0 * k2.dvelocity + 2.0 * k3.dvelocity + k4.dvelocity)
            );

            velocity.setAngular(
                v0.getAngular() +
                (dt / 6.0) *
                (k1.domega + 2.0 * k2.domega + 2.0 * k3.domega + k4.domega)
            );

            transform.orientation =
                glm::normalize(
                    t0.orientation +
                    (dt / 6.0) *
                    (k1.dquaternion + 2.0 * k2.dquaternion + 2.0 * k3.dquaternion + k4.dquaternion)
                );

            // Clear the forces
            accumulator.clear();
        }
    }

} // namespace StrikeEngine
