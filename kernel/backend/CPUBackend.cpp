#include "CPUBackend.hpp"
#include "../data/PhysicsBlock.hpp"
#include "../data/ControlBlock.hpp"
#include <cmath>

namespace StrikeEngine::Kernel
{

    CPUBackend::CPUBackend(
        std::unique_ptr<Integrator> integ,
        std::shared_ptr<Models::AtmosphereModel> atmos,
        std::shared_ptr<Models::AeroModel> aeroModel,
        std::shared_ptr<Models::PropulsionModel> propModel
    )
        : integrator(std::move(integ)),
          atmosphere(std::move(atmos)),
          aero(std::move(aeroModel)),
          propulsion(std::move(propModel))
    {
        scheduler = std::make_unique<HybridScheduler>(*integrator);
    }

    void CPUBackend::initialize(
        PhysicsBlock&,
        ControlBlock&)
    {
    }

    void CPUBackend::shutdown()
    {
    }

    void CPUBackend::step(
        PhysicsBlock& physics,
        ControlBlock& control,
        double currentTime,
        double dt)
    {
        computeForces(physics, control, currentTime, dt);

        if (integrator->isAdaptive())
        {
            scheduler->executeStep(physics, dt);
        }
        else
        {
            integrator->integrate(physics, dt);
        }
    }

    void CPUBackend::computeForces(
        PhysicsBlock& physics,
        ControlBlock& control,
        double currentTime,
        double dt)
    {
        const std::size_t n = physics.size;

        constexpr double gravity = -9.80665; // simplistic flat earth gravity

        for (std::size_t i = 0; i < n; ++i)
        {
            if (!physics.active[i])
                continue;

            physics.ax[i] = 0.0;
            physics.ay[i] = 0.0;
            physics.az[i] = 0.0;

            // 1. Atmosphere
            double altitude = physics.pz[i]; // assuming Z is up for flat earth
            auto atmProps = atmosphere->evaluate(altitude);

            // 2. Aerodynamics (6-DOF Wrench)
            auto aeroWrench = aero->computeWrench(
                physics.vx[i], physics.vy[i], physics.vz[i],
                physics.qx[i], physics.qy[i], physics.qz[i], physics.qw[i],
                physics.wx[i], physics.wy[i], physics.wz[i],
                control.pitchCommand[i], control.yawCommand[i], control.rollCommand[i],
                atmProps.density, atmProps.speedOfSound, 0.1 // 0.1m^2 reference area for MVP
            );

            // 3. Propulsion
            auto propState = propulsion->evaluate(
                currentTime,
                physics.qx[i], physics.qy[i], physics.qz[i], physics.qw[i],
                atmProps.pressure
            );

            // 4. Sum Forces & Update Mass
            if (propState.massFlowRate_kg_s > 0.0) {
                physics.mass[i] -= propState.massFlowRate_kg_s * dt;
                if (physics.mass[i] < 1.0) physics.mass[i] = 1.0; // Prevent zero mass
            }

            const double invMass = 1.0 / physics.mass[i];

            double fx = aeroWrench.force_x + propState.thrust_x;
            double fy = aeroWrench.force_y + propState.thrust_y;
            double fz = aeroWrench.force_z + propState.thrust_z;

            physics.ax[i] = fx * invMass;
            physics.ay[i] = fy * invMass;
            physics.az[i] = fz * invMass + gravity;

            // 5. Angular Acceleration (alpha = Torque / I)
            // Simplified uncoupled rotational dynamics for MVP (I is diagonal and in world frame approximation for stability)
            // A true 6-DOF solves Euler's equations in the body frame: I*w_dot + w x (I*w) = Torque
            // For MVP, we will simplify: alpha = Torque_world / I_world_approx (just using body inertias directly as a crude placeholder until full tensor rotation is implemented).
            physics.alphax[i] = aeroWrench.torque_x / physics.Ixx[i];
            physics.alphay[i] = aeroWrench.torque_y / physics.Iyy[i];
            physics.alphaz[i] = aeroWrench.torque_z / physics.Izz[i];
        }
    }

} // namespace StrikeEngine::Kernel