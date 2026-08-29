#include <strikeengine/kernel/systems/SensorSystem.hpp>
#include <strikeengine/kernel/math/Quaternion.hpp>
#include <strikeengine/models/physics/earth/EarthFrames.hpp>
#include <cmath>
#include <chrono>
#include <array>

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
            sensors.imuEnabled.resize(size, true);
            sensors.gpsEnabled.resize(size, true);
            sensors.gpsUpdateRateHz.resize(size, 1.0);
        }

        std::normal_distribution<double> stdNorm(0.0, 1.0);

        for (std::size_t i = 0; i < size; ++i) {
            if (!physics.active[i]) continue;

            // Sensor failure: measurements stop updating entirely (no new
            // IMU/GPS noise is generated; GPS simply never reports fresh data).
            if (i < status.sensorFailed.size() && status.sensorFailed[i]) {
                sensors.gpsUpdated[i] = false;
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

            // IMU lever-arm correction (MVP): the IMU is mounted at a fixed
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

            // 2. GPS Update
            sensors.gpsUpdated[i] = updateGps;
            if (updateGps) {
                sensors.gpsPosX[i] = physics.px[i] + stdNorm(rng) * sensors.gpsPosNoiseStdDev[i];
                sensors.gpsPosY[i] = physics.py[i] + stdNorm(rng) * sensors.gpsPosNoiseStdDev[i];
                sensors.gpsPosZ[i] = physics.pz[i] + stdNorm(rng) * sensors.gpsPosNoiseStdDev[i];

                sensors.gpsVelX[i] = physics.vx[i] + stdNorm(rng) * sensors.gpsVelNoiseStdDev[i];
                sensors.gpsVelY[i] = physics.vy[i] + stdNorm(rng) * sensors.gpsVelNoiseStdDev[i];
                sensors.gpsVelZ[i] = physics.vz[i] + stdNorm(rng) * sensors.gpsVelNoiseStdDev[i];
            }
        }
    }

} // namespace StrikeEngine::Kernel
