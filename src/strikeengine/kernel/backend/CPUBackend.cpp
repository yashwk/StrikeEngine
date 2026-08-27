#include <strikeengine/kernel/backend/CPUBackend.hpp>
#include <strikeengine/kernel/data/PhysicsBlock.hpp>
#include <strikeengine/kernel/data/ControlBlock.hpp>
#include <strikeengine/kernel/math/Quaternion.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>
#include <strikeengine/models/physics/earth/EarthFrames.hpp>
#include <strikeengine/models/physics/earth/EarthFixedPropagator.hpp>
#include <array>
#include <cmath>

namespace StrikeEngine::Kernel
{

    CPUBackend::CPUBackend(
        std::unique_ptr<Integrator> integ,
        std::shared_ptr<Models::AtmosphereModel> atmos,
        std::shared_ptr<Models::AeroModel> aeroModel,
        EnvironmentConfig environmentConfig
    )
        : integrator(std::move(integ)),
          atmosphere(std::move(atmos)),
          aero(std::move(aeroModel)),
          environment(std::move(environmentConfig))
    {
        scheduler = std::make_unique<HybridScheduler>(*integrator);
    }

    int CPUBackend::registerPropulsion(std::shared_ptr<const Models::PropulsionModel> model)
    {
        propulsionPool.push_back(std::move(model));
        return static_cast<int>(propulsionPool.size()) - 1;
    }

    void CPUBackend::setEnvironment(const EnvironmentConfig& environmentConfig)
    {
        environment = environmentConfig;
    }

    void CPUBackend::initialize(
        PhysicsBlock&,
        ControlBlock&)
    {
    }

    void CPUBackend::shutdown()
    {
    }

    void CPUBackend::ensureDerivCapacity(const PhysicsBlock& state)
    {
        if (derivBuffer.size == state.size)
            return;

        const std::size_t n = state.size;
        derivBuffer.px.assign(n, 0.0); derivBuffer.py.assign(n, 0.0); derivBuffer.pz.assign(n, 0.0);
        derivBuffer.vx.assign(n, 0.0); derivBuffer.vy.assign(n, 0.0); derivBuffer.vz.assign(n, 0.0);
        derivBuffer.ax.assign(n, 0.0); derivBuffer.ay.assign(n, 0.0); derivBuffer.az.assign(n, 0.0);
        derivBuffer.qw.assign(n, 0.0); derivBuffer.qx.assign(n, 0.0); derivBuffer.qy.assign(n, 0.0); derivBuffer.qz.assign(n, 0.0);
        derivBuffer.wx.assign(n, 0.0); derivBuffer.wy.assign(n, 0.0); derivBuffer.wz.assign(n, 0.0);
        derivBuffer.alphax.assign(n, 0.0); derivBuffer.alphay.assign(n, 0.0); derivBuffer.alphaz.assign(n, 0.0);
        derivBuffer.Ixx.assign(n, 0.0); derivBuffer.Iyy.assign(n, 0.0); derivBuffer.Izz.assign(n, 0.0);
        derivBuffer.mass.assign(n, 0.0);
        derivBuffer.massDry.assign(n, 0.0);
        derivBuffer.referenceArea.assign(n, 0.0);
        derivBuffer.referenceLength.assign(n, 0.0);
        derivBuffer.cd.assign(n, 0.0);
        derivBuffer.clAlpha.assign(n, 0.0);
        derivBuffer.clFin.assign(n, 0.0);
        derivBuffer.clMax.assign(n, 0.0);
        derivBuffer.propulsionId.assign(n, -1);
        derivBuffer.ignitionTime.assign(n, 0.0);
        derivBuffer.finPitch.assign(n, 0.0); derivBuffer.finYaw.assign(n, 0.0); derivBuffer.finRoll.assign(n, 0.0);
        derivBuffer.active.assign(n, false);
        derivBuffer.size = n;
    }

    void CPUBackend::evaluateDerivative(
        const PhysicsBlock& s,
        const ControlBlock& c,
        double t,
        PhysicsBlock& d)
    {
        const std::size_t n = s.size;
        constexpr double servoTimeConstant = 0.02; // first-order actuator lag (s)

        for (std::size_t i = 0; i < n; ++i)
        {
            d.px[i] = 0.0; d.py[i] = 0.0; d.pz[i] = 0.0;
            d.vx[i] = 0.0; d.vy[i] = 0.0; d.vz[i] = 0.0;
            d.ax[i] = 0.0; d.ay[i] = 0.0; d.az[i] = 0.0;
            d.qw[i] = 0.0; d.qx[i] = 0.0; d.qy[i] = 0.0; d.qz[i] = 0.0;
            d.wx[i] = 0.0; d.wy[i] = 0.0; d.wz[i] = 0.0;
            d.alphax[i] = 0.0; d.alphay[i] = 0.0; d.alphaz[i] = 0.0;
            d.mass[i] = 0.0;
            d.finPitch[i] = 0.0; d.finYaw[i] = 0.0; d.finRoll[i] = 0.0;

            if (!s.active[i])
                continue;

            // Position derivative = velocity (world)
            d.px[i] = s.vx[i];
            d.py[i] = s.vy[i];
            d.pz[i] = s.vz[i];

            // 1. Atmosphere at altitude. Local mode uses Z above datum;
            // ECEF mode resolves the absolute position through WGS84.
            const bool ecefTruth = environment.earth.useEcefTruth;
            const Models::EcefCoordinate ecefPosition{
                s.px[i], s.py[i], s.pz[i]};
            Models::GeodeticCoordinate earthPosition{};
            if (ecefTruth) {
                earthPosition = Models::ecefToGeodetic(ecefPosition);
            }
            const double altitude = ecefTruth ? earthPosition.altitudeM : s.pz[i];
            const auto atm = atmosphere->evaluate(altitude);

            // 2. Body-frame air-relative velocity. Wind is the world-frame
            // air-mass velocity, so it is removed before evaluating aero.
            const auto wind = environment.windVelocity
                ? environment.windVelocity(s.px[i], s.py[i], s.pz[i], t)
                : std::array<double, 3>{0.0, 0.0, 0.0};
            double u, v, w;
            quatRotateToBody(s.qw[i], s.qx[i], s.qy[i], s.qz[i],
                             s.vx[i] - wind[0], s.vy[i] - wind[1],
                             s.vz[i] - wind[2], u, v, w);

            // 3. Aerodynamics: body-frame forces and moments (W2/W3 spine)
            Models::AeroParams params;
            params.referenceArea   = s.referenceArea[i];
            params.referenceLength = s.referenceLength[i];
            params.cd              = s.cd[i];
            params.clAlpha         = s.clAlpha[i];
            params.clFin           = s.clFin[i];
            params.clMax           = s.clMax[i];

            const Models::AeroWrench aeroWrench = aero->computeWrench(
                u, v, w,
                s.wx[i], s.wy[i], s.wz[i],          // achieved BODY rates
                s.finPitch[i], s.finYaw[i], s.finRoll[i],  // achieved deflections
                atm.density, atm.speedOfSound,
                params);

            // 4. Propulsion (per-entity motor, fuel-limited)
            double thrustBodyX = 0.0;
            double massFlow = 0.0;
            const int pid = s.propulsionId[i];
            const double fuel = s.mass[i] - s.massDry[i];
            if (pid >= 0 && fuel > 1e-9 && pid < static_cast<int>(propulsionPool.size()))
            {
                const auto prop = propulsionPool[static_cast<std::size_t>(pid)]->evaluate(
                    t - s.ignitionTime[i], atm.pressure);
                thrustBodyX = prop.thrustBodyX;
                massFlow   = prop.massFlowRate_kg_s;
                constexpr double kFuelDepletionGuardWindowSec = 0.01; // depletion guard window (s); mass floor is enforced by applyStateUpdate
                if (massFlow * kFuelDepletionGuardWindowSec > fuel)  // never burn more fuel than remains
                    massFlow = fuel / kFuelDepletionGuardWindowSec;
            }

            // 5. Total body force -> world acceleration
            const double invMass = 1.0 / s.mass[i];
            const double fbx = aeroWrench.force_x + thrustBodyX;
            const double fby = aeroWrench.force_y;
            const double fbz = aeroWrench.force_z;

            double afx, afy, afz;
            quatRotateToWorld(s.qw[i], s.qx[i], s.qy[i], s.qz[i], fbx, fby, fbz, afx, afy, afz);

            std::array<double, 3> gravity{0.0, 0.0, -9.80665};
            std::array<double, 3> earthFrameAcceleration{0.0, 0.0, 0.0};
            if (ecefTruth) {
                // ECEF truth defaults to radial spherical gravity unless the
                // caller explicitly selects WGS84 normal gravity.
                if (environment.earth.useWgs84Gravity) {
                    gravity = Models::EarthFrames::ecefNormalGravityAcceleration(
                        earthPosition);
                } else {
                    gravity = Models::EarthFrames::toVector(
                        Models::sphericalGravityAccelerationEcef(ecefPosition));
                }
                const auto rotating = Models::EarthFixed::acceleration(
                    ecefPosition,
                    {s.vx[i], s.vy[i], s.vz[i]},
                    {},
                    {false,
                     environment.earth.includeCoriolis,
                     environment.earth.includeCentrifugal});
                earthFrameAcceleration = Models::EarthFrames::toVector(rotating);
            } else if (environment.earth.useWgs84Gravity) {
                gravity[2] = -Models::normalGravity(
                    environment.earth.referenceLatitudeRad, altitude);
            } else if (environment.earth.useSphericalGravity) {
                const Models::GeodeticCoordinate reference{
                    environment.earth.referenceLatitudeRad,
                    environment.earth.referenceLongitudeRad,
                    0.0};
                const auto position = Models::EarthFrames::enuToGeodetic(
                    {s.px[i], s.py[i], s.pz[i]}, reference);
                gravity = Models::EarthFrames::localSphericalGravityAcceleration(
                    position);
            }
            std::array<double, 3> coriolis{0.0, 0.0, 0.0};
            if (!ecefTruth && environment.earth.includeCoriolis) {
                coriolis = Models::localCoriolisAcceleration(
                    environment.earth.referenceLatitudeRad,
                    {s.vx[i], s.vy[i], s.vz[i]});
            }
            if (!ecefTruth && environment.earth.includeCentrifugal) {
                const auto centrifugal = Models::EarthFrames::localCentrifugalAcceleration({
                    environment.earth.referenceLatitudeRad,
                    environment.earth.referenceLongitudeRad,
                    altitude});
                coriolis[0] += centrifugal[0];
                coriolis[1] += centrifugal[1];
                coriolis[2] += centrifugal[2];
            }
            if (!ecefTruth && environment.earth.includeTransportRate) {
                const Models::GeodeticCoordinate reference{
                    environment.earth.referenceLatitudeRad,
                    environment.earth.referenceLongitudeRad,
                    0.0};
                const auto position = Models::EarthFrames::enuToGeodetic(
                    {s.px[i], s.py[i], s.pz[i]}, reference);
                const auto transport = Models::localTransportAcceleration(
                    position, {s.vx[i], s.vy[i], s.vz[i]});
                coriolis[0] += transport[0];
                coriolis[1] += transport[1];
                coriolis[2] += transport[2];
            }
            coriolis[0] += earthFrameAcceleration[0];
            coriolis[1] += earthFrameAcceleration[1];
            coriolis[2] += earthFrameAcceleration[2];

            const double axWorld = afx * invMass + gravity[0] + coriolis[0];
            const double ayWorld = afy * invMass + gravity[1] + coriolis[1];
            const double azWorld = afz * invMass + gravity[2] + coriolis[2];

            d.vx[i] = axWorld;
            d.vy[i] = ayWorld;
            d.vz[i] = azWorld;
            d.ax[i] = axWorld;   // cache (world) for sensors/readouts
            d.ay[i] = ayWorld;
            d.az[i] = azWorld;

            // 6. Body-frame rotational dynamics (W2): I*w_dot + w x (I w) = tau
            const double Ixx = s.Ixx[i], Iyy = s.Iyy[i], Izz = s.Izz[i];
            const double wx = s.wx[i], wy = s.wy[i], wz = s.wz[i];
            const double alphaX = (aeroWrench.torque_x - (Izz - Iyy) * wy * wz) / Ixx;
            const double alphaY = (aeroWrench.torque_y - (Ixx - Izz) * wz * wx) / Iyy;
            const double alphaZ = (aeroWrench.torque_z - (Iyy - Ixx) * wx * wy) / Izz;

            d.wx[i] = alphaX;
            d.wy[i] = alphaY;
            d.wz[i] = alphaZ;
            d.alphax[i] = alphaX;  // cache (body)
            d.alphay[i] = alphaY;
            d.alphaz[i] = alphaZ;

            // 7. Quaternion derivative: q_dot = 0.5 * q (x) (0, w)
            d.qw[i] = -0.5 * (s.qx[i] * wx + s.qy[i] * wy + s.qz[i] * wz);
            d.qx[i] =  0.5 * (s.qw[i] * wx + s.qy[i] * wz - s.qz[i] * wy);
            d.qy[i] =  0.5 * (s.qw[i] * wy - s.qx[i] * wz + s.qz[i] * wx);
            d.qz[i] =  0.5 * (s.qw[i] * wz + s.qx[i] * wy - s.qy[i] * wx);

            // 8. Mass flow (fuel-limited; clamped at massDry by integrator)
            d.mass[i] = -massFlow;

            // 9. Actuator dynamics: first-order lag toward commanded
            // deflection with a physical rate limit (~300 deg/s).
            constexpr double maxServoRate = 5.24;  // rad/s
            auto servo = [maxServoRate](double cmd, double fin, double tau) {
                const double target = std::clamp((cmd - fin) / tau, -maxServoRate, maxServoRate);
                return target;
            };
            d.finPitch[i] = servo(c.pitchCommand[i], s.finPitch[i], servoTimeConstant);
            d.finYaw[i]   = servo(c.yawCommand[i],   s.finYaw[i],   servoTimeConstant);
            d.finRoll[i]  = servo(c.rollCommand[i],  s.finRoll[i],  servoTimeConstant);
        }
    }

    void CPUBackend::step(
        PhysicsBlock& physics,
        ControlBlock& control,
        double currentTime,
        double dt)
    {
        ensureDerivCapacity(physics);

        // Derivative callback: pure function of (state, time) with fixed
        // zero-order-held control commands for this step.
        const Integrator::DerivativeFn derivFn =
            [this, &control](const PhysicsBlock& state, double t, PhysicsBlock& d)
            {
                evaluateDerivative(state, control, t, d);
            };

        if (integrator->isAdaptive())
        {
            scheduler->executeStep(physics, derivFn, currentTime, dt);
        }
        else
        {
            integrator->integrate(physics, derivFn, currentTime, dt);
        }

        // Refresh acceleration caches at the final state so sensors and
        // readouts see the true accelerations at the end of the step.
        evaluateDerivative(physics, control, currentTime + dt, derivBuffer);
        const std::size_t n = physics.size;
        for (std::size_t i = 0; i < n; ++i)
        {
            if (!physics.active[i])
                continue;
            physics.ax[i] = derivBuffer.ax[i];
            physics.ay[i] = derivBuffer.ay[i];
            physics.az[i] = derivBuffer.az[i];
            physics.alphax[i] = derivBuffer.alphax[i];
            physics.alphay[i] = derivBuffer.alphay[i];
            physics.alphaz[i] = derivBuffer.alphaz[i];
        }
    }

} // namespace StrikeEngine::Kernel
