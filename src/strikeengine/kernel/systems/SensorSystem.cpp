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

    void SensorSystem::ensureCapacity(std::size_t size) {
        if (trueAccelBiasX.size() < size) {
            trueAccelBiasX.resize(size, 0.0);
            trueAccelBiasY.resize(size, 0.0);
            trueAccelBiasZ.resize(size, 0.0);
            trueGyroBiasX.resize(size, 0.0);
            trueGyroBiasY.resize(size, 0.0);
            trueGyroBiasZ.resize(size, 0.0);
        }
    }

    void SensorSystem::update(
        const PhysicsBlock& physics,
        SensorBlock& sensors,
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
        }

        bool updateGps = false;
        if (currentTime - lastGpsUpdateTime >= 1.0 / gpsUpdateRate) {
            updateGps = true;
            lastGpsUpdateTime = currentTime;
        }

        std::normal_distribution<double> stdNorm(0.0, 1.0);

        for (std::size_t i = 0; i < size; ++i) {
            if (!physics.active[i]) continue;

            // 1. IMU Specific Force (world-frame acceleration minus gravity).
            std::array<double, 3> gravity{0.0, 0.0, -9.80665};
            if (environment.earth.useEcefTruth) {
                const auto position = Models::ecefToGeodetic({
                    physics.px[i], physics.py[i], physics.pz[i]});
                if (environment.earth.useWgs84Gravity) {
                    gravity = Models::EarthFrames::ecefNormalGravityAcceleration(
                        position);
                } else {
                    gravity = Models::EarthFrames::toVector(
                        Models::sphericalGravityAccelerationEcef(
                            {physics.px[i], physics.py[i], physics.pz[i]}));
                }
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
