#include <strikeengine/kernel/backend/CPUBackend.hpp>
#include <strikeengine/kernel/data/PhysicsBlock.hpp>
#include <strikeengine/kernel/data/ControlBlock.hpp>
#include <strikeengine/kernel/math/Quaternion.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>
#include <strikeengine/models/physics/earth/EarthFrames.hpp>
#include <strikeengine/models/physics/earth/EarthFixedPropagator.hpp>
#include <strikeengine/models/physics/aerodynamics/AirframeModel.hpp>
#include <array>
#include <algorithm>
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
        // Recycle a released slot when one is available so repeated
        // createVehicle/removeVehicle cycles do not grow the pool.
        if (!propulsionFreeList.empty()) {
            const int slot = propulsionFreeList.back();
            propulsionFreeList.pop_back();
            propulsionPool[static_cast<std::size_t>(slot)] = std::move(model);
            return slot;
        }
        propulsionPool.push_back(std::move(model));
        return static_cast<int>(propulsionPool.size()) - 1;
    }

    void CPUBackend::releasePropulsion(int poolId)
    {
        // Only live registrations are recyclable; a stale or double release
        // (slot already on the free list, or out of range) is ignored.
        if (poolId < 0 || poolId >= static_cast<int>(propulsionPool.size())) {
            return;
        }
        const auto it = std::find(propulsionFreeList.begin(), propulsionFreeList.end(), poolId);
        if (it != propulsionFreeList.end()) {
            return;
        }
        propulsionPool[static_cast<std::size_t>(poolId)] = nullptr;
        propulsionFreeList.push_back(poolId);
    }

    void CPUBackend::reset()
    {
        // Every createVehicle registers fresh propulsion models, so without
        // this the pool (and propulsionId space) grows across reset->create
        // cycles. Integrator/scheduler hold no per-run state.
        propulsionPool.clear();
        propulsionFreeList.clear();
    }

    void CPUBackend::setEnvironment(const EnvironmentConfig& environmentConfig)
    {
        environment = environmentConfig;
    }

    void CPUBackend::setThreadCount(std::size_t threads)
    {
        threadPool.resize(threads);
    }

    std::size_t CPUBackend::threadCount() const
    {
        return threadPool.size();
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
        derivBuffer.mach.assign(n, 0.0);
        derivBuffer.dynamicPressure.assign(n, 0.0);
        derivBuffer.airDensity.assign(n, 0.0);
        derivBuffer.localSpeedOfSound.assign(n, 0.0);
        derivBuffer.mass.assign(n, 0.0);
        derivBuffer.gimbalPitch.assign(n, 0.0); derivBuffer.gimbalYaw.assign(n, 0.0);
        derivBuffer.finPitch.assign(n, 0.0); derivBuffer.finYaw.assign(n, 0.0); derivBuffer.finRoll.assign(n, 0.0);
        derivBuffer.size = n;
    }

    void CPUBackend::evaluateDerivativeChunk(
        const PhysicsBlock& s,
        const ControlBlock& c,
        double t,
        PhysicsBlock& d,
        std::size_t start,
        std::size_t end)
    {
        for (std::size_t i = start; i < end; ++i)
        {
            d.px[i] = 0.0; d.py[i] = 0.0; d.pz[i] = 0.0;
            d.vx[i] = 0.0; d.vy[i] = 0.0; d.vz[i] = 0.0;
            d.ax[i] = 0.0; d.ay[i] = 0.0; d.az[i] = 0.0;
            d.qw[i] = 0.0; d.qx[i] = 0.0; d.qy[i] = 0.0; d.qz[i] = 0.0;
            d.wx[i] = 0.0; d.wy[i] = 0.0; d.wz[i] = 0.0;
            d.alphax[i] = 0.0; d.alphay[i] = 0.0; d.alphaz[i] = 0.0;
            d.mach[i] = 0.0; d.dynamicPressure[i] = 0.0;
            d.airDensity[i] = 0.0; d.localSpeedOfSound[i] = 0.0;
            d.mass[i] = 0.0;
            d.gimbalPitch[i] = 0.0; d.gimbalYaw[i] = 0.0;
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

            // 3. Aerodynamics: body-frame forces and moments.
            Models::AeroParams params;
            params.referenceArea   = s.referenceArea[i];
            params.referenceLength = s.referenceLength[i];
            params.cd              = s.cd[i];
            params.clAlpha         = s.clAlpha[i];
            params.clFin           = s.clFin[i];
            params.clMax           = s.clMax[i];
            params.tailControl     = s.tailControl[i];
            params.tables          = s.aeroTables[i];
            params.fins            = s.fins[i];
            // Rotational inertia, so the aero model can bound its moment as an
            // angular acceleration rather than against dynamic pressure.
            if (i < s.Ixx.size()) params.inertiaX = s.Ixx[i];
            if (i < s.Iyy.size()) params.inertiaY = s.Iyy[i];
            if (i < s.Izz.size()) params.inertiaZ = s.Izz[i];
            if (s.finSets.size() > i) {
                params.finSets     = s.finSets[i];
            }
            if (s.airframe.size() > i) {
                params.airframe    = s.airframe[i];
            }

            const Models::AeroWrench aeroWrench = aero->computeWrench(
                u, v, w,
                s.wx[i], s.wy[i], s.wz[i],          // achieved BODY rates
                s.finPitch[i], s.finYaw[i], s.finRoll[i],  // achieved deflections
                atm.density, atm.speedOfSound,
                params);

            // Ambient truth mirrors: the same airspeed / atmosphere the aero
            // model above consumed, exposed to consumers via the post-step
            // refresh in step(). Airspeed is wind-relative (matches the
            // force computation), so Mach and q are the true flight values.
            const double vAirMag = std::sqrt(u * u + v * v + w * w);
            d.mach[i] = (atm.speedOfSound > 0.0) ? vAirMag / atm.speedOfSound : 0.0;
            d.dynamicPressure[i] = 0.5 * atm.density * vAirMag * vAirMag;
            d.airDensity[i] = atm.density;
            d.localSpeedOfSound[i] = atm.speedOfSound;

            // 4. Propulsion (per-entity motor, fuel-limited)
            double thrustBodyX = 0.0;
            double thrustBodyY = 0.0;
            double thrustBodyZ = 0.0;
            double massFlow = 0.0;
            const int pid = s.propulsionId[i];
            const double fuel = s.mass[i] - std::max(s.massDry[i], s.stageMinMass[i]);
            if (pid >= 0 && fuel > 1e-9 && pid < static_cast<int>(propulsionPool.size()))
            {
                const auto prop = propulsionPool[static_cast<std::size_t>(pid)]->evaluate(
                    t - s.ignitionTime[i], atm.pressure,
                    s.gimbalPitch[i], s.gimbalYaw[i]);
                thrustBodyX = prop.thrustBodyX;
                thrustBodyY = prop.thrustBodyY;
                thrustBodyZ = prop.thrustBodyZ;
                massFlow   = prop.massFlowRate_kg_s;
                // Never burn more fuel than remains: cap the flow so the
                // remaining fuel lasts the current step window (thrust scales
                // with the capped flow so T = mdot*Isp*g0 holds — no
                // free-thrust tail on the last grams of propellant).
                const double guardWindowSec = std::max(currentStepDt, 1e-6);
                if (massFlow * guardWindowSec > fuel) {  // never burn more fuel than remains
                    const double cappedMassFlow = fuel / guardWindowSec;
                    // Scale thrust with the capped flow so T = mdot*Isp*g0
                    // holds at every instant: no free-thrust tail on the last
                    // few grams of propellant (full thrust on ~0 flow).
                    thrustBodyX *= cappedMassFlow / massFlow;
                    thrustBodyY *= cappedMassFlow / massFlow;
                    thrustBodyZ *= cappedMassFlow / massFlow;
                    massFlow = cappedMassFlow;
                }
            }

            // Motor/engine failure disables the engine. Tank failure disables
            // feed as a deterministic no-leak failure mode: residual fuel is
            // retained, but it cannot reach the engine.
            if ((i < s.motorFailed.size() && s.motorFailed[i]) ||
                (i < s.engineFailed.size() && s.engineFailed[i]) ||
                (i < s.tankFailed.size() && s.tankFailed[i])) {
                thrustBodyX = 0.0;
                thrustBodyY = 0.0;
                thrustBodyZ = 0.0;
                massFlow = 0.0;
            }

            // 5. Total body force -> world acceleration
            const double invMass = 1.0 / s.mass[i];
            const double fbx = aeroWrench.force_x + thrustBodyX;
            const double fby = aeroWrench.force_y + thrustBodyY;
            const double fbz = aeroWrench.force_z + thrustBodyZ;

            double afx, afy, afz;
            quatRotateToWorld(s.qw[i], s.qx[i], s.qy[i], s.qz[i], fbx, fby, fbz, afx, afy, afz);

            std::array<double, 3> gravity{0.0, 0.0, -9.80665};
            std::array<double, 3> earthFrameAcceleration{0.0, 0.0, 0.0};
            if (ecefTruth) {
                // ECEF truth defaults to radial spherical gravity unless the
                // caller explicitly selects WGS84 normal or J2 gravity.
                if (environment.earth.includeJ2Gravity) {
                    gravity = Models::EarthFrames::toVector(
                        Models::j2GravityAccelerationEcef(ecefPosition));
                } else if (environment.earth.useWgs84Gravity) {
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
            } else if (environment.earth.includeJ2Gravity) {
                const Models::GeodeticCoordinate reference{
                    environment.earth.referenceLatitudeRad,
                    environment.earth.referenceLongitudeRad,
                    0.0};
                const auto position = Models::EarthFrames::enuToGeodetic(
                    {s.px[i], s.py[i], s.pz[i]}, reference);
                gravity = Models::EarthFrames::localJ2GravityAcceleration(position);
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
            const double Ixy = s.Ixy[i], Ixz = s.Ixz[i], Iyz = s.Iyz[i];
            const double wx = s.wx[i], wy = s.wy[i], wz = s.wz[i];
            const double propulsionTorqueX = s.enginePositionY[i] * thrustBodyZ -
                s.enginePositionZ[i] * thrustBodyY;
            const double propulsionTorqueY = s.enginePositionZ[i] * thrustBodyX -
                s.enginePositionX[i] * thrustBodyZ;
            const double propulsionTorqueZ = s.enginePositionX[i] * thrustBodyY -
                s.enginePositionY[i] * thrustBodyX;

            const double totalMx = aeroWrench.torque_x + propulsionTorqueX;
            const double totalMy = aeroWrench.torque_y + propulsionTorqueY;
            const double totalMz = aeroWrench.torque_z + propulsionTorqueZ;

            double alphaX = 0.0, alphaY = 0.0, alphaZ = 0.0;
            if (Ixy == 0.0 && Ixz == 0.0 && Iyz == 0.0) {
                // Diagonal fast path (zero off-diagonal cross-coupling)
                alphaX = (totalMx - (Izz - Iyy) * wy * wz) / Ixx;
                alphaY = (totalMy - (Ixx - Izz) * wz * wx) / Iyy;
                alphaZ = (totalMz - (Iyy - Ixx) * wx * wy) / Izz;
            } else {
                // Full 3x3 symmetric inertia tensor:
                // [  Ixx  -Ixy  -Ixz ]
                // [ -Ixy   Iyy  -Iyz ]
                // [ -Ixz  -Iyz   Izz ]
                const double M00 = Ixx,  M01 = -Ixy, M02 = -Ixz;
                const double             M11 =  Iyy, M12 = -Iyz;
                const double                         M22 =  Izz;

                // Angular momentum H = I * w
                const double Hx = M00 * wx + M01 * wy + M02 * wz;
                const double Hy = M01 * wx + M11 * wy + M12 * wz;
                const double Hz = M02 * wx + M12 * wy + M22 * wz;

                // Gyroscopic cross-coupling torque: w x H
                const double gyroX = wy * Hz - wz * Hy;
                const double gyroY = wz * Hx - wx * Hz;
                const double gyroZ = wx * Hy - wy * Hx;

                const double tauEffX = totalMx - gyroX;
                const double tauEffY = totalMy - gyroY;
                const double tauEffZ = totalMz - gyroZ;

                // Analytical cofactor matrix for symmetric 3x3
                const double c00 = M11 * M22 - M12 * M12;
                const double c01 = M02 * M12 - M01 * M22;
                const double c02 = M01 * M12 - M02 * M11;
                const double c11 = M00 * M22 - M02 * M02;
                const double c12 = M01 * M02 - M00 * M12;
                const double c22 = M00 * M11 - M01 * M01;

                const double det = M00 * c00 + M01 * c01 + M02 * c02;
                if (std::abs(det) > 1e-12) {
                    const double invDet = 1.0 / det;
                    const double inv00 = c00 * invDet;
                    const double inv01 = c01 * invDet;
                    const double inv02 = c02 * invDet;
                    const double inv11 = c11 * invDet;
                    const double inv12 = c12 * invDet;
                    const double inv22 = c22 * invDet;

                    alphaX = inv00 * tauEffX + inv01 * tauEffY + inv02 * tauEffZ;
                    alphaY = inv01 * tauEffX + inv11 * tauEffY + inv12 * tauEffZ;
                    alphaZ = inv02 * tauEffX + inv12 * tauEffY + inv22 * tauEffZ;
                } else {
                    alphaX = (totalMx - (Izz - Iyy) * wy * wz) / Ixx;
                    alphaY = (totalMy - (Ixx - Izz) * wz * wx) / Iyy;
                    alphaZ = (totalMz - (Iyy - Ixx) * wx * wy) / Izz;
                }
            }

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

            // TVC actuator dynamics. Commands are clamped to the configured
            // gimbal envelope; achieved angles are integrated and clamped by
            // applyStateUpdate, just like fin servos.
            auto gimbalServo = [](double command, double achieved, double limit,
                                  double tau, double maxRate) {
                if (limit <= 0.0) return 0.0;
                const double target = std::clamp(command, -limit, limit);
                const double effectiveTau = tau > 0.0 ? tau : 1e-6;
                double rate = (target - achieved) / effectiveTau;
                if (maxRate > 0.0) rate = std::clamp(rate, -maxRate, maxRate);
                return rate;
            };
            if ((i < s.motorFailed.size() && s.motorFailed[i]) ||
                (i < s.engineFailed.size() && s.engineFailed[i]) ||
                (i < s.tankFailed.size() && s.tankFailed[i])) {
                d.gimbalPitch[i] = 0.0;
                d.gimbalYaw[i] = 0.0;
            } else {
                d.gimbalPitch[i] = gimbalServo(
                    c.thrustVectorPitchCommand[i], s.gimbalPitch[i],
                    s.maxGimbalPitchRad[i], s.gimbalTimeConstantSec[i],
                    s.maxGimbalRateRadPerSec[i]);
                d.gimbalYaw[i] = gimbalServo(
                    c.thrustVectorYawCommand[i], s.gimbalYaw[i],
                    s.maxGimbalYawRad[i], s.gimbalTimeConstantSec[i],
                    s.maxGimbalRateRadPerSec[i]);
            }

            // 9. Actuator dynamics: first-order lag toward commanded
            // deflection with a per-entity physical rate limit.
            const double servoTimeConstant = s.servoTimeConstantSec[i];
            const double maxServoRate = s.maxServoRateRadPerSec[i];
            auto servo = [maxServoRate](double cmd, double fin, double tau) {
                const double target = std::clamp((cmd - fin) / tau, -maxServoRate, maxServoRate);
                return target;
            };
            if (i < s.actuatorFailed.size() && s.actuatorFailed[i]) {
                // Actuator failure: achieved fins freeze (no servo motion).
                d.finPitch[i] = 0.0;
                d.finYaw[i] = 0.0;
                d.finRoll[i] = 0.0;
            } else {
                d.finPitch[i] = servo(c.pitchCommand[i], s.finPitch[i], servoTimeConstant);
                d.finYaw[i]   = servo(c.yawCommand[i],   s.finYaw[i],   servoTimeConstant);
                d.finRoll[i]  = servo(c.rollCommand[i],  s.finRoll[i],  servoTimeConstant);
            }
        }
    }

    void CPUBackend::evaluateDerivative(
        const PhysicsBlock& s,
        const ControlBlock& c,
        double t,
        PhysicsBlock& d)
    {
        threadPool.parallelFor(s.size, [this, &s, &c, t, &d](std::size_t start, std::size_t end) {
            evaluateDerivativeChunk(s, c, t, d, start, end);
        });
    }

    void CPUBackend::step(
        PhysicsBlock& physics,
        ControlBlock& control,
        double currentTime,
        double dt)
    {
        ensureDerivCapacity(physics);
        currentStepDt = (dt > 0.0) ? dt : 0.01;

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
        threadPool.parallelFor(n, [this, &physics](std::size_t start, std::size_t end) {
            for (std::size_t i = start; i < end; ++i)
            {
                if (!physics.active[i])
                    continue;
                physics.ax[i] = derivBuffer.ax[i];
                physics.ay[i] = derivBuffer.ay[i];
                physics.az[i] = derivBuffer.az[i];
                physics.alphax[i] = derivBuffer.alphax[i];
                physics.alphay[i] = derivBuffer.alphay[i];
                physics.alphaz[i] = derivBuffer.alphaz[i];
                physics.mach[i] = derivBuffer.mach[i];
                physics.dynamicPressure[i] = derivBuffer.dynamicPressure[i];
                physics.airDensity[i] = derivBuffer.airDensity[i];
                physics.localSpeedOfSound[i] = derivBuffer.localSpeedOfSound[i];
            }
        });
    }

} // namespace StrikeEngine::Kernel
