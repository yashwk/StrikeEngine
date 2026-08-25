#include <strikeengine/kernel/systems/NavigationSystem.hpp>
#include <cmath>
#include <iostream>

namespace StrikeEngine::Kernel {

    void NavigationSystem::ensureCapacity(std::size_t size, NavigationBlock& nav) {
        if (nav.estPx.size() < size) {
            nav.estPx.resize(size, 0.0); nav.estPy.resize(size, 0.0); nav.estPz.resize(size, 0.0);
            nav.estVx.resize(size, 0.0); nav.estVy.resize(size, 0.0); nav.estVz.resize(size, 0.0);
            nav.estQx.resize(size, 0.0); nav.estQy.resize(size, 0.0); nav.estQz.resize(size, 0.0); nav.estQw.resize(size, 1.0);
            nav.estWx.resize(size, 0.0); nav.estWy.resize(size, 0.0); nav.estWz.resize(size, 0.0);
            nav.estAccelBiasX.resize(size, 0.0); nav.estAccelBiasY.resize(size, 0.0); nav.estAccelBiasZ.resize(size, 0.0);
            nav.estGyroBiasX.resize(size, 0.0); nav.estGyroBiasY.resize(size, 0.0); nav.estGyroBiasZ.resize(size, 0.0);
            nav.covarianceDiag.resize(size, std::array<double, 15>{10.0, 10.0, 10.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
            nav.isAligned.resize(size, false);
        }
    }

    void rotateBodyToWorld(double qw, double qx, double qy, double qz,
                           double bx, double by, double bz,
                           double& wx, double& wy, double& wz) {
        // q * v * q_inv
        double ix =  qw * bx + qy * bz - qz * by;
        double iy =  qw * by + qz * bx - qx * bz;
        double iz =  qw * bz + qx * by - qy * bx;
        double iw = -qx * bx - qy * by - qz * bz;

        wx = ix * qw + iw * (-qx) + iy * (-qz) - iz * (-qy);
        wy = iy * qw + iw * (-qy) + iz * (-qx) - ix * (-qz);
        wz = iz * qw + iw * (-qz) + ix * (-qy) - iy * (-qx);
    }

    void NavigationSystem::strapdownINS(std::size_t id, const SensorBlock& sensors, NavigationBlock& nav, double dt) {
        // 1. Compensate IMU measurements
        double fx = sensors.accelX[id] - nav.estAccelBiasX[id];
        double fy = sensors.accelY[id] - nav.estAccelBiasY[id];
        double fz = sensors.accelZ[id] - nav.estAccelBiasZ[id];

        double wx = sensors.gyroX[id] - nav.estGyroBiasX[id];
        double wy = sensors.gyroY[id] - nav.estGyroBiasY[id];
        double wz = sensors.gyroZ[id] - nav.estGyroBiasZ[id];

        nav.estWx[id] = wx;
        nav.estWy[id] = wy;
        nav.estWz[id] = wz;

        // 2. Rotate specific force to World frame
        double wfx, wfy, wfz;
        rotateBodyToWorld(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id], fx, fy, fz, wfx, wfy, wfz);

        // 3. Add gravity (-Z)
        double ax = wfx;
        double ay = wfy;
        double az = wfz - 9.80665;

        // 4. Integrate velocity
        nav.estVx[id] += ax * dt;
        nav.estVy[id] += ay * dt;
        nav.estVz[id] += az * dt;

        // 5. Integrate position
        nav.estPx[id] += nav.estVx[id] * dt;
        nav.estPy[id] += nav.estVy[id] * dt;
        nav.estPz[id] += nav.estVz[id] * dt;

        // 6. Integrate attitude (Quaternion propagation: q = q + 0.5 * q * w * dt)
        double qw = nav.estQw[id];
        double qx = nav.estQx[id];
        double qy = nav.estQy[id];
        double qz = nav.estQz[id];

        double dqx =  qw * wx + qy * wz - qz * wy;
        double dqy =  qw * wy + qz * wx - qx * wz;
        double dqz =  qw * wz + qx * wy - qy * wx;
        double dqw = -qx * wx - qy * wy - qz * wz;

        qw += 0.5 * dqw * dt;
        qx += 0.5 * dqx * dt;
        qy += 0.5 * dqy * dt;
        qz += 0.5 * dqz * dt;

        // Normalize
        double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (norm > 0) {
            nav.estQw[id] = qw / norm;
            nav.estQx[id] = qx / norm;
            nav.estQy[id] = qy / norm;
            nav.estQz[id] = qz / norm;
        }
        
        // Very simplified Covariance propagation (Random walk growth)
        for (int i=0; i<15; ++i) {
            nav.covarianceDiag[id][i] += 0.001 * dt; // Arbitrary process noise for MVP
        }
    }

    void NavigationSystem::ekfUpdate(std::size_t id, const SensorBlock& sensors, NavigationBlock& nav) {
        // Simplified loosely-coupled GPS update (Direct position/velocity observation)
        // Innovation (Measurement - Estimate)
        double dzP_x = sensors.gpsPosX[id] - nav.estPx[id];
        double dzP_y = sensors.gpsPosY[id] - nav.estPy[id];
        double dzP_z = sensors.gpsPosZ[id] - nav.estPz[id];
        
        double dzV_x = sensors.gpsVelX[id] - nav.estVx[id];
        double dzV_y = sensors.gpsVelY[id] - nav.estVy[id];
        double dzV_z = sensors.gpsVelZ[id] - nav.estVz[id];

        // Measurement noise covariance (R)
        double rPos = sensors.gpsPosNoiseStdDev[id] * sensors.gpsPosNoiseStdDev[id];
        double rVel = sensors.gpsVelNoiseStdDev[id] * sensors.gpsVelNoiseStdDev[id];

        // Compute Kalman Gain and update state (Simplified Scalar Update per diagonal element)
        // K = P / (P + R)
        auto updateState = [](double& state, double& pDiag, double z, double r) {
            double k = pDiag / (pDiag + r);
            state += k * z;
            pDiag = (1.0 - k) * pDiag;
        };

        updateState(nav.estPx[id], nav.covarianceDiag[id][0], dzP_x, rPos);
        updateState(nav.estPy[id], nav.covarianceDiag[id][1], dzP_y, rPos);
        updateState(nav.estPz[id], nav.covarianceDiag[id][2], dzP_z, rPos);

        updateState(nav.estVx[id], nav.covarianceDiag[id][3], dzV_x, rVel);
        updateState(nav.estVy[id], nav.covarianceDiag[id][4], dzV_y, rVel);
        updateState(nav.estVz[id], nav.covarianceDiag[id][5], dzV_z, rVel);

        // Attitude and biases would normally be updated via cross-correlation in the off-diagonal P matrix.
        // For a diagonal-only MVP, we apply a tiny fixed gain to couple velocity error to attitude and accel bias.
        double accelBiasGain = 0.001;
        nav.estAccelBiasX[id] -= accelBiasGain * dzV_x;
        nav.estAccelBiasY[id] -= accelBiasGain * dzV_y;
        nav.estAccelBiasZ[id] -= accelBiasGain * dzV_z;
    }

    void NavigationSystem::update(const SensorBlock& sensors, NavigationBlock& nav, double dt) {
        nav.size = sensors.size;
        ensureCapacity(nav.size, nav);

        for (std::size_t i = 0; i < nav.size; ++i) {
            // Initial alignment (Perfect initialization for MVP, later we can add initial uncertainty)
            if (!nav.isAligned[i]) {
                nav.estPx[i] = sensors.gpsPosX[i];
                nav.estPy[i] = sensors.gpsPosY[i];
                nav.estPz[i] = sensors.gpsPosZ[i];
                nav.estVx[i] = sensors.gpsVelX[i];
                nav.estVy[i] = sensors.gpsVelY[i];
                nav.estVz[i] = sensors.gpsVelZ[i];
                nav.estWx[i] = 0.0;
                nav.estWy[i] = 0.0;
                nav.estWz[i] = 0.0;
                nav.isAligned[i] = true;
                continue; // Skip first tick integration
            }

            // High rate strapdown integration
            strapdownINS(i, sensors, nav, dt);

            // Low rate EKF fusion
            if (sensors.gpsUpdated[i]) {
                ekfUpdate(i, sensors, nav);
            }
        }
    }

} // namespace StrikeEngine::Kernel
