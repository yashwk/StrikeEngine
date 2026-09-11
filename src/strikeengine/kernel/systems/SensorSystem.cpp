#include <strikeengine/kernel/systems/SensorSystem.hpp>
#include <strikeengine/kernel/math/Quaternion.hpp>
#include <strikeengine/models/physics/earth/EarthFrames.hpp>
#include <strikeengine/models/physics/earth/MagneticModel.hpp>
#include <cmath>
#include <chrono>
#include <array>
#include <deque>

namespace StrikeEngine::Kernel {

    SensorSystem::SensorSystem() {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        rng = std::mt19937(seed);
    }

    void SensorSystem::setSeed(std::uint32_t seed) {
        // Deterministic stream for reproducible validation runs. Note: the
        // streaming-bias state is a pure function of the RNG, so re-seeding
        // the same scenario reproduces the same biases as well.
        rng.seed(seed);
    }

    double SensorSystem::nextUniform01() {
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        return uniform(rng);
    }

    void SensorSystem::reset() {
        trueAccelBiasX.clear();
        trueAccelBiasY.clear();
        trueAccelBiasZ.clear();
        trueGyroBiasX.clear();
        trueGyroBiasY.clear();
        trueGyroBiasZ.clear();
        lastGpsUpdateTime.clear();
        lastBaroUpdateTime.clear();
        lastMagUpdateTime.clear();
        trueBaroBias.clear();
        gpsLatencyQueue.clear();
    }

    void SensorSystem::ensureCapacity(std::size_t size) {
        if (trueAccelBiasX.size() < size) {
            trueAccelBiasX.resize(size, 0.0);
            trueAccelBiasY.resize(size, 0.0);
            trueAccelBiasZ.resize(size, 0.0);
            trueGyroBiasX.resize(size, 0.0);
            trueGyroBiasY.resize(size, 0.0);
            trueGyroBiasZ.resize(size, 0.0);
        }
        if (lastGpsUpdateTime.size() < size) {
            lastGpsUpdateTime.resize(size, 0.0);
        }
        if (lastBaroUpdateTime.size() < size) {
            lastBaroUpdateTime.resize(size, 0.0);
            trueBaroBias.resize(size, 0.0);
        }
        if (lastMagUpdateTime.size() < size) {
            lastMagUpdateTime.resize(size, 0.0);
        }
        if (gpsLatencyQueue.size() < size) {
            gpsLatencyQueue.resize(size);
        }
    }

    void SensorSystem::update(
        const PhysicsBlock& physics,
        SensorBlock& sensors,
        const EntityStatusBlock& status,
        double currentTime,
        double dt,
        const EnvironmentConfig& environment)
    {
        std::size_t size = physics.size;
        sensors.size = size;
        ensureCapacity(size);

        // Resize sensor arrays if needed (handled in SimulationKernel usually, but let's be safe)
        if (sensors.accelX.size() < size) {
            sensors.accelX.resize(size); sensors.accelY.resize(size); sensors.accelZ.resize(size);
            sensors.gyroX.resize(size); sensors.gyroY.resize(size); sensors.gyroZ.resize(size);
            sensors.gpsPosX.resize(size); sensors.gpsPosY.resize(size); sensors.gpsPosZ.resize(size);
            sensors.gpsVelX.resize(size); sensors.gpsVelY.resize(size); sensors.gpsVelZ.resize(size);
            sensors.gpsUpdated.resize(size);
            sensors.imuLeverArmX.resize(size); sensors.imuLeverArmY.resize(size); sensors.imuLeverArmZ.resize(size);
            sensors.gpsInnovationGateSigma.resize(size, 5.0);
            sensors.imuEnabled.resize(size, true);
            sensors.gpsEnabled.resize(size, true);
            sensors.gpsUpdateRateHz.resize(size, 1.0);
            sensors.baroAlt.resize(size, 0.0);
            sensors.baroUpdated.resize(size, false);
            sensors.magX.resize(size, 0.0); sensors.magY.resize(size, 0.0); sensors.magZ.resize(size, 0.0);
            sensors.magUpdated.resize(size, false);
            sensors.baroEnabled.resize(size, false);
            sensors.baroNoiseStdDev.resize(size, 1.0);
            sensors.baroBiasStdDev.resize(size, 0.0);
            sensors.baroUpdateRateHz.resize(size, 1.0);
            sensors.magEnabled.resize(size, false);
            sensors.magNoiseStdDev.resize(size, 50e-9);
            sensors.magUpdateRateHz.resize(size, 10.0);
            sensors.magDisturbanceGateRel.resize(size, 0.25);
            sensors.gpsLatencySec.resize(size, 0.0);
            sensors.gpsLeverArmX.resize(size, 0.0);
            sensors.gpsLeverArmY.resize(size, 0.0);
            sensors.gpsLeverArmZ.resize(size, 0.0);
            sensors.gpsFixConsistencyEnabled.resize(size, false);
            sensors.insConingCompensationEnabled.resize(size, false);
            sensors.insAdaptiveQEnabled.resize(size, false);
            sensors.insAdaptiveQGain.resize(size, 1.0);
            sensors.initialAttitudeErrorDeg.resize(size, 0.0);
            sensors.initialPositionErrorM.resize(size, 0.0);
            sensors.initialVelocityErrorMps.resize(size, 0.0);
        }

        std::normal_distribution<double> stdNorm(0.0, 1.0);

        for (std::size_t i = 0; i < size; ++i) {
            if (!physics.active[i]) continue;

            // Sensor failure: measurements stop updating entirely (no new
            // IMU/GPS noise is generated; GPS simply never reports fresh data).
            if (i < status.sensorFailed.size() && status.sensorFailed[i]) {
                sensors.gpsUpdated[i] = false;
                if (i < sensors.baroUpdated.size()) sensors.baroUpdated[i] = false;
                if (i < sensors.magUpdated.size()) sensors.magUpdated[i] = false;
                continue;
            }

            // Per-entity GPS refresh scheduling: a sample is produced only when
            // the GPS device is enabled and the per-entity interval has elapsed.
            bool updateGps = false;
            if (sensors.gpsEnabled[i]) {
                const double period = 1.0 / sensors.gpsUpdateRateHz[i];
                if (currentTime - lastGpsUpdateTime[i] >= period) {
                    updateGps = true;
                    lastGpsUpdateTime[i] = currentTime;
                }
            }

            // 1. IMU Specific Force (world-frame acceleration minus gravity).
            std::array<double, 3> gravity{0.0, 0.0, -9.80665};
            if (environment.earth.useEcefTruth) {
                const auto position = Models::ecefToGeodetic({
                    physics.px[i], physics.py[i], physics.pz[i]});
                if (environment.earth.includeJ2Gravity) {
                    gravity = Models::EarthFrames::toVector(
                        Models::j2GravityAccelerationEcef({
                            physics.px[i], physics.py[i], physics.pz[i]}));
                } else if (environment.earth.useWgs84Gravity) {
                    gravity = Models::EarthFrames::ecefNormalGravityAcceleration(
                        position);
                } else {
                    gravity = Models::EarthFrames::toVector(
                        Models::sphericalGravityAccelerationEcef(
                            {physics.px[i], physics.py[i], physics.pz[i]}));
                }
            } else if (environment.earth.includeJ2Gravity) {
                const Models::GeodeticCoordinate reference{
                    environment.earth.referenceLatitudeRad,
                    environment.earth.referenceLongitudeRad,
                    0.0};
                gravity = Models::EarthFrames::localJ2GravityAcceleration(
                    Models::EarthFrames::enuToGeodetic(
                        {physics.px[i], physics.py[i], physics.pz[i]}, reference));
            } else if (environment.earth.useWgs84Gravity) {
                gravity[2] = -Models::normalGravity(
                    environment.earth.referenceLatitudeRad, physics.pz[i]);
            } else if (environment.earth.useSphericalGravity) {
                const Models::GeodeticCoordinate reference{
                    environment.earth.referenceLatitudeRad,
                    environment.earth.referenceLongitudeRad,
                    0.0};
                gravity = Models::EarthFrames::localSphericalGravityAcceleration(
                    Models::EarthFrames::enuToGeodetic(
                        {physics.px[i], physics.py[i], physics.pz[i]}, reference));
            }
            const double fx = physics.ax[i] - gravity[0];
            const double fy = physics.ay[i] - gravity[1];
            const double fz = physics.az[i] - gravity[2];

            // Rotate Specific Force to Body Frame. Uses the shared
            // Quaternion.hpp rotation (v' = q^-1 v q). The historical local
            // helper had a sign error on the cross-product terms (it applied
            // q^-1 twice), which inverted the measured specific force for any
            // non-identity attitude and made the autopilot's lateral channels
            // positive-feedback (growing oscillation -> tumble).
            double bfx, bfy, bfz;
            quatRotateToBody(physics.qw[i], physics.qx[i], physics.qy[i], physics.qz[i],
                             fx, fy, fz, bfx, bfy, bfz);

            // True Gyro is already body frame (assumed in PhysicsBlock wx,wy,wz)
            // Wait, PhysicsBlock wx, wy, wz are body frame angular rates? Yes.
            double bwx = physics.wx[i];
            double bwy = physics.wy[i];
            double bwz = physics.wz[i];

            // Earth-rate gyro modeling (ECEF truth, opt-in): a physical gyro
            // measures the INERTIAL body rate, which includes the earth
            // rotation vector resolved into the body frame,
            //     omega_ie^b = C_e^b * (0, 0, Omega_ie)
            // via the body->ECEF attitude quaternion.
            if (environment.earth.useEcefTruth && environment.earth.includeEarthRateGyro) {
                double ewx, ewy, ewz;
                quatRotateToBody(physics.qw[i], physics.qx[i], physics.qy[i], physics.qz[i],
                                 0.0, 0.0, Models::EarthModel::earthRotationRateRadPerSec,
                                 ewx, ewy, ewz);
                bwx += ewx;
                bwy += ewy;
                bwz += ewz;
            }

            // IMU lever-arm correction: the IMU is mounted at a fixed
            // body-frame offset l from the centre of mass, so it senses the
            // CM specific force plus the rigid-body terms
            //     f_imu = f_cm + alpha x l + omega x (omega x l)
            // where alpha is the body angular acceleration and omega the body
            // angular rate. Zero lever arm yields a zero correction.
            const double lx = sensors.imuLeverArmX[i];
            const double ly = sensors.imuLeverArmY[i];
            const double lz = sensors.imuLeverArmZ[i];
            if (lx != 0.0 || ly != 0.0 || lz != 0.0) {
                // term_a = alpha x l
                const double termAx = physics.alphay[i] * lz - physics.alphaz[i] * ly;
                const double termAy = physics.alphaz[i] * lx - physics.alphax[i] * lz;
                const double termAz = physics.alphax[i] * ly - physics.alphay[i] * lx;
                // c = omega x l; term_c = omega x (omega x l)
                const double cx = bwy * lz - bwz * ly;
                const double cy = bwz * lx - bwx * lz;
                const double cz = bwx * ly - bwy * lx;
                const double termCx = bwy * cz - bwz * cy;
                const double termCy = bwz * cx - bwx * cz;
                const double termCz = bwx * cy - bwy * cx;
                bfx += termAx + termCx;
                bfy += termAy + termCy;
                bfz += termAz + termCz;
            }

            // Random walk biases + measurement output. A disabled IMU freezes
            // its last measurements and stops its streaming-bias drift.
            if (sensors.imuEnabled[i]) {
                // Random walk biases (slow drift) - very simple model
                trueAccelBiasX[i] += stdNorm(rng) * sensors.accelBiasStdDev[i] * dt;
                trueAccelBiasY[i] += stdNorm(rng) * sensors.accelBiasStdDev[i] * dt;
                trueAccelBiasZ[i] += stdNorm(rng) * sensors.accelBiasStdDev[i] * dt;
                trueGyroBiasX[i]  += stdNorm(rng) * sensors.gyroBiasStdDev[i] * dt;
                trueGyroBiasY[i]  += stdNorm(rng) * sensors.gyroBiasStdDev[i] * dt;
                trueGyroBiasZ[i]  += stdNorm(rng) * sensors.gyroBiasStdDev[i] * dt;

                // Add noise and bias
                sensors.accelX[i] = bfx + trueAccelBiasX[i] + stdNorm(rng) * sensors.accelNoiseStdDev[i];
                sensors.accelY[i] = bfy + trueAccelBiasY[i] + stdNorm(rng) * sensors.accelNoiseStdDev[i];
                sensors.accelZ[i] = bfz + trueAccelBiasZ[i] + stdNorm(rng) * sensors.accelNoiseStdDev[i];

                sensors.gyroX[i] = bwx + trueGyroBiasX[i] + stdNorm(rng) * sensors.gyroNoiseStdDev[i];
                sensors.gyroY[i] = bwy + trueGyroBiasY[i] + stdNorm(rng) * sensors.gyroNoiseStdDev[i];
                sensors.gyroZ[i] = bwz + trueGyroBiasZ[i] + stdNorm(rng) * sensors.gyroNoiseStdDev[i];
            }

            // 2. GPS Update (antenna position = CM + body-frame lever arm).
            // Lever arm is rotated by the TRUE attitude (same convention as
            // the IMU lever arm); a zero lever reproduces the legacy CM fix.
            double antX = physics.px[i], antY = physics.py[i], antZ = physics.pz[i];
            {
                const double lx = sensors.gpsLeverArmX[i];
                const double ly = sensors.gpsLeverArmY[i];
                const double lz = sensors.gpsLeverArmZ[i];
                if (lx != 0.0 || ly != 0.0 || lz != 0.0) {
                    double wox, woy, woz;
                    quatRotateToWorld(physics.qw[i], physics.qx[i], physics.qy[i], physics.qz[i],
                                      lx, ly, lz, wox, woy, woz);
                    antX += wox; antY += woy; antZ += woz;
                }
            }
            const double latency = std::max(0.0, sensors.gpsLatencySec[i]);
            sensors.gpsUpdated[i] = false;
            if (updateGps) {
                // Noise is drawn at SAMPLE time (stream order identical to
                // legacy when latency is 0); delivery may be delayed below.
                DelayedGpsSample sample;
                sample.timeSec = currentTime;
                sample.px = antX + stdNorm(rng) * sensors.gpsPosNoiseStdDev[i];
                sample.py = antY + stdNorm(rng) * sensors.gpsPosNoiseStdDev[i];
                sample.pz = antZ + stdNorm(rng) * sensors.gpsPosNoiseStdDev[i];
                sample.vx = physics.vx[i] + stdNorm(rng) * sensors.gpsVelNoiseStdDev[i];
                sample.vy = physics.vy[i] + stdNorm(rng) * sensors.gpsVelNoiseStdDev[i];
                sample.vz = physics.vz[i] + stdNorm(rng) * sensors.gpsVelNoiseStdDev[i];
                if (latency <= 0.0) {
                    sensors.gpsPosX[i] = sample.px; sensors.gpsPosY[i] = sample.py; sensors.gpsPosZ[i] = sample.pz;
                    sensors.gpsVelX[i] = sample.vx; sensors.gpsVelY[i] = sample.vy; sensors.gpsVelZ[i] = sample.vz;
                    sensors.gpsUpdated[i] = true;
                } else {
                    gpsLatencyQueue[i].push_back(sample);
                }
            }
            if (latency > 0.0 && !gpsLatencyQueue[i].empty()) {
                // Drop superseded samples; publish the newest due one.
                while (gpsLatencyQueue[i].size() > 1 &&
                       currentTime - gpsLatencyQueue[i][1].timeSec >= latency - 1e-12) {
                    gpsLatencyQueue[i].pop_front();
                }
                const DelayedGpsSample& due = gpsLatencyQueue[i].front();
                if (currentTime - due.timeSec >= latency - 1e-12) {
                    sensors.gpsPosX[i] = due.px; sensors.gpsPosY[i] = due.py; sensors.gpsPosZ[i] = due.pz;
                    sensors.gpsVelX[i] = due.vx; sensors.gpsVelY[i] = due.vy; sensors.gpsVelZ[i] = due.vz;
                    sensors.gpsUpdated[i] = true;
                    gpsLatencyQueue[i].pop_front();
                }
            }

            // 3. Barometer altitude (opt-in). Truth altitude is geodetic
            // under ECEF truth, datum-relative Z otherwise; bias random-walks
            // like the IMU biases. No draws happen while disabled.
            sensors.baroUpdated[i] = false;
            if (sensors.baroEnabled[i]) {
                const double baroPeriod = 1.0 / std::max(sensors.baroUpdateRateHz[i], 1e-9);
                if (currentTime - lastBaroUpdateTime[i] >= baroPeriod) {
                    lastBaroUpdateTime[i] = currentTime;
                    double truthAlt = physics.pz[i];
                    if (environment.earth.useEcefTruth) {
                        truthAlt = Models::ecefToGeodetic({
                            physics.px[i], physics.py[i], physics.pz[i]}).altitudeM;
                    }
                    trueBaroBias[i] += stdNorm(rng) * sensors.baroBiasStdDev[i] * dt;
                    sensors.baroAlt[i] = truthAlt + trueBaroBias[i] +
                        stdNorm(rng) * sensors.baroNoiseStdDev[i];
                    sensors.baroUpdated[i] = true;
                }
            }

            // 4. Magnetometer (opt-in). Tilted-dipole world field at the true
            // position, rotated to the body frame by the TRUE attitude, plus
            // white noise per axis. No draws happen while disabled.
            sensors.magUpdated[i] = false;
            if (sensors.magEnabled[i]) {
                const double magPeriod = 1.0 / std::max(sensors.magUpdateRateHz[i], 1e-9);
                if (currentTime - lastMagUpdateTime[i] >= magPeriod) {
                    lastMagUpdateTime[i] = currentTime;
                    Models::EcefCoordinate ecefPos{
                        physics.px[i], physics.py[i], physics.pz[i]};
                    if (!environment.earth.useEcefTruth) {
                        const Models::GeodeticCoordinate reference{
                            environment.earth.referenceLatitudeRad,
                            environment.earth.referenceLongitudeRad,
                            0.0};
                        ecefPos = Models::geodeticToEcef(
                            Models::EarthFrames::enuToGeodetic(
                                {physics.px[i], physics.py[i], physics.pz[i]}, reference));
                    }
                    const auto field = Models::dipoleMagneticFieldEcef(
                        ecefPos.x, ecefPos.y, ecefPos.z);
                    double bbx, bby, bbz;
                    quatRotateToBody(physics.qw[i], physics.qx[i], physics.qy[i], physics.qz[i],
                                     field[0], field[1], field[2], bbx, bby, bbz);
                    sensors.magX[i] = bbx + stdNorm(rng) * sensors.magNoiseStdDev[i];
                    sensors.magY[i] = bby + stdNorm(rng) * sensors.magNoiseStdDev[i];
                    sensors.magZ[i] = bbz + stdNorm(rng) * sensors.magNoiseStdDev[i];
                    sensors.magUpdated[i] = true;
                }
            }
        }
    }

} // namespace StrikeEngine::Kernel
