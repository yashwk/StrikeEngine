#include "SensorSystem.hpp"
#include <cmath>
#include <chrono>

namespace StrikeEngine::Kernel {

    SensorSystem::SensorSystem() {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        rng = std::mt19937(seed);
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

    // Helper: Rotate vector by inverse quaternion (World to Body)
    void rotateWorldToBody(double qw, double qx, double qy, double qz,
                           double vx, double vy, double vz,
                           double& bx, double& by, double& bz) {
        // Inverse of unit quaternion (qw, qx, qy, qz) is (qw, -qx, -qy, -qz)
        // q_inv * v * q
        double ix =  qw * vx - (-qy) * vz + (-qz) * vy;
        double iy =  qw * vy - (-qz) * vx + (-qx) * vz;
        double iz =  qw * vz - (-qx) * vy + (-qy) * vx;
        double iw = -(-qx) * vx - (-qy) * vy - (-qz) * vz;

        bx = ix * qw + iw * (-qx) + iy * (-qz) - iz * (-qy);
        by = iy * qw + iw * (-qy) + iz * (-qx) - ix * (-qz);
        bz = iz * qw + iw * (-qz) + ix * (-qy) - iy * (-qx);
    }

    void SensorSystem::update(
        const PhysicsBlock& physics,
        SensorBlock& sensors,
        double currentTime,
        double dt)
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

            // 1. IMU Specific Force (World frame a - g)
            double fx = physics.ax[i];
            double fy = physics.ay[i];
            double fz = physics.az[i] + 9.80665; // Gravity points down (-Z), so specific force +g in Z

            // Rotate Specific Force to Body Frame
            double bfx, bfy, bfz;
            rotateWorldToBody(physics.qw[i], physics.qx[i], physics.qy[i], physics.qz[i],
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
