#include <strikeengine/kernel/systems/NavigationSystem.hpp>
#include <algorithm>
#include <cmath>
#include <array>

namespace {

constexpr std::size_t kErrorStateSize = 15;
constexpr double kMinCovariance = 1e-12;
constexpr double kMaxCovariance = 1e6;
using Covariance = std::array<double, kErrorStateSize * kErrorStateSize>;

constexpr std::size_t covarianceIndex(std::size_t row, std::size_t column)
{
    return row * kErrorStateSize + column;
}

std::array<double, kErrorStateSize> initialCovarianceDiag()
{
    return {10.0, 10.0, 10.0, 1.0, 1.0, 1.0,
            0.1, 0.1, 0.1, 0.01, 0.01, 0.01,
            0.01, 0.01, 0.01};
}

void initializeCovariance(Covariance& covariance,
                          const std::array<double, kErrorStateSize>& diagonal)
{
    covariance.fill(0.0);
    for (std::size_t i = 0; i < kErrorStateSize; ++i) {
        covariance[covarianceIndex(i, i)] = diagonal[i];
    }
}

void boundCovariance(Covariance& covariance,
                     std::array<double, kErrorStateSize>& diagonal)
{
    for (std::size_t i = 0; i < kErrorStateSize; ++i) {
        double& variance = covariance[covarianceIndex(i, i)];
        variance = std::clamp(variance, kMinCovariance, kMaxCovariance);
    }

    for (std::size_t i = 0; i < kErrorStateSize; ++i) {
        for (std::size_t j = i + 1; j < kErrorStateSize; ++j) {
            double& upper = covariance[covarianceIndex(i, j)];
            double& lower = covariance[covarianceIndex(j, i)];
            const double limit = std::sqrt(
                covariance[covarianceIndex(i, i)] * covariance[covarianceIndex(j, j)]);
            const double value = std::clamp(0.5 * (upper + lower), -limit, limit);
            upper = value;
            lower = value;
        }
    }

    for (std::size_t i = 0; i < kErrorStateSize; ++i) {
        diagonal[i] = covariance[covarianceIndex(i, i)];
    }
}

void applyAttitudeError(StrikeEngine::Kernel::NavigationBlock& nav,
                        std::size_t id,
                        double dx,
                        double dy,
                        double dz)
{
    // Small attitude error is expressed in body axes. Right-multiply the
    // body->world quaternion by the corresponding small-angle quaternion.
    const double qw = nav.estQw[id];
    const double qx = nav.estQx[id];
    const double qy = nav.estQy[id];
    const double qz = nav.estQz[id];
    const double hX = 0.5 * dx;
    const double hY = 0.5 * dy;
    const double hZ = 0.5 * dz;

    nav.estQw[id] = qw - qx * hX - qy * hY - qz * hZ;
    nav.estQx[id] = qx + qw * hX + qy * hZ - qz * hY;
    nav.estQy[id] = qy + qw * hY - qx * hZ + qz * hX;
    nav.estQz[id] = qz + qw * hZ + qx * hY - qy * hX;

    const double norm = std::sqrt(
        nav.estQw[id] * nav.estQw[id] + nav.estQx[id] * nav.estQx[id] +
        nav.estQy[id] * nav.estQy[id] + nav.estQz[id] * nav.estQz[id]);
    if (norm > 1e-12) {
        nav.estQw[id] /= norm;
        nav.estQx[id] /= norm;
        nav.estQy[id] /= norm;
        nav.estQz[id] /= norm;
    }
}

} // namespace

namespace StrikeEngine::Kernel {

    void NavigationSystem::ensureCapacity(std::size_t size, NavigationBlock& nav) {
        const auto defaultDiag = initialCovarianceDiag();
        if (nav.estPx.size() < size) {
            nav.estPx.resize(size, 0.0); nav.estPy.resize(size, 0.0); nav.estPz.resize(size, 0.0);
            nav.estVx.resize(size, 0.0); nav.estVy.resize(size, 0.0); nav.estVz.resize(size, 0.0);
            nav.estQx.resize(size, 0.0); nav.estQy.resize(size, 0.0); nav.estQz.resize(size, 0.0); nav.estQw.resize(size, 1.0);
            nav.estWx.resize(size, 0.0); nav.estWy.resize(size, 0.0); nav.estWz.resize(size, 0.0);
            nav.estAccelBiasX.resize(size, 0.0); nav.estAccelBiasY.resize(size, 0.0); nav.estAccelBiasZ.resize(size, 0.0);
            nav.estGyroBiasX.resize(size, 0.0); nav.estGyroBiasY.resize(size, 0.0); nav.estGyroBiasZ.resize(size, 0.0);
            nav.covarianceDiag.resize(size, defaultDiag);
            nav.isAligned.resize(size, false);
        }

        const std::size_t oldCovarianceSize = nav.covarianceFull.size();
        nav.covarianceFull.resize(size);
        for (std::size_t i = oldCovarianceSize; i < size; ++i) {
            const auto& diagonal = i < nav.covarianceDiag.size()
                ? nav.covarianceDiag[i] : defaultDiag;
            initializeCovariance(nav.covarianceFull[i], diagonal);
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
        
        auto& covariance = nav.covarianceFull[id];
        Covariance transition{};
        transition.fill(0.0);
        for (std::size_t i = 0; i < kErrorStateSize; ++i) {
            transition[covarianceIndex(i, i)] = 1.0;
        }
        for (std::size_t axis = 0; axis < 3; ++axis) {
            transition[covarianceIndex(axis, axis + 3)] = dt;
            transition[covarianceIndex(axis + 6, axis + 12)] = -dt;
        }

        // Linearized velocity sensitivity to attitude and accelerometer bias.
        const double skewBody[3][3] = {
            {0.0, -fz, fy},
            {fz, 0.0, -fx},
            {-fy, fx, 0.0}};
        double rotation[3][3]{};
        rotateBodyToWorld(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                          1.0, 0.0, 0.0, rotation[0][0], rotation[1][0], rotation[2][0]);
        rotateBodyToWorld(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                          0.0, 1.0, 0.0, rotation[0][1], rotation[1][1], rotation[2][1]);
        rotateBodyToWorld(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                          0.0, 0.0, 1.0, rotation[0][2], rotation[1][2], rotation[2][2]);
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                double attitudeSensitivity = 0.0;
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    attitudeSensitivity += rotation[row][axis] * skewBody[axis][column];
                }
                transition[covarianceIndex(row + 3, column + 6)] =
                    -attitudeSensitivity * dt;
                transition[covarianceIndex(row + 3, column + 9)] =
                    -rotation[row][column] * dt;
            }
        }

        Covariance temp{};
        temp.fill(0.0);
        for (std::size_t row = 0; row < kErrorStateSize; ++row) {
            for (std::size_t column = 0; column < kErrorStateSize; ++column) {
                double value = 0.0;
                for (std::size_t k = 0; k < kErrorStateSize; ++k) {
                    value += transition[covarianceIndex(row, k)] *
                             covariance[covarianceIndex(k, column)];
                }
                temp[covarianceIndex(row, column)] = value;
            }
        }
        Covariance propagated{};
        propagated.fill(0.0);
        for (std::size_t row = 0; row < kErrorStateSize; ++row) {
            for (std::size_t column = 0; column < kErrorStateSize; ++column) {
                double value = 0.0;
                for (std::size_t k = 0; k < kErrorStateSize; ++k) {
                    value += temp[covarianceIndex(row, k)] *
                             transition[covarianceIndex(column, k)];
                }
                propagated[covarianceIndex(row, column)] = value;
            }
        }

        const double accelNoise = std::max(0.0, sensors.accelNoiseStdDev[id]);
        const double gyroNoise = std::max(0.0, sensors.gyroNoiseStdDev[id]);
        const double accelBiasNoise = std::max(0.0, sensors.accelBiasStdDev[id]);
        const double gyroBiasNoise = std::max(0.0, sensors.gyroBiasStdDev[id]);
        const double processNoise[kErrorStateSize] = {
            0.25 * accelNoise * accelNoise * dt * dt * dt,
            0.25 * accelNoise * accelNoise * dt * dt * dt,
            0.25 * accelNoise * accelNoise * dt * dt * dt,
            accelNoise * accelNoise * dt,
            accelNoise * accelNoise * dt,
            accelNoise * accelNoise * dt,
            gyroNoise * gyroNoise * dt,
            gyroNoise * gyroNoise * dt,
            gyroNoise * gyroNoise * dt,
            accelBiasNoise * accelBiasNoise * dt,
            accelBiasNoise * accelBiasNoise * dt,
            accelBiasNoise * accelBiasNoise * dt,
            gyroBiasNoise * gyroBiasNoise * dt,
            gyroBiasNoise * gyroBiasNoise * dt,
            gyroBiasNoise * gyroBiasNoise * dt};
        for (std::size_t i = 0; i < kErrorStateSize; ++i) {
            propagated[covarianceIndex(i, i)] += processNoise[i];
        }
        covariance = propagated;
        boundCovariance(covariance, nav.covarianceDiag[id]);
    }

    void NavigationSystem::ekfUpdate(std::size_t id, const SensorBlock& sensors, NavigationBlock& nav) {
        // Sequential scalar GPS position/velocity updates. The full covariance
        // couples the observed translational states to attitude and IMU bias
        // corrections instead of applying independent diagonal gains.
        double rPos = sensors.gpsPosNoiseStdDev[id] * sensors.gpsPosNoiseStdDev[id];
        double rVel = sensors.gpsVelNoiseStdDev[id] * sensors.gpsVelNoiseStdDev[id];

        auto& covariance = nav.covarianceFull[id];
        std::array<double, kErrorStateSize> correction{};
        const std::array<double, kErrorStateSize> baseState = {
            nav.estPx[id], nav.estPy[id], nav.estPz[id],
            nav.estVx[id], nav.estVy[id], nav.estVz[id],
            0.0, 0.0, 0.0,
            nav.estAccelBiasX[id], nav.estAccelBiasY[id], nav.estAccelBiasZ[id],
            nav.estGyroBiasX[id], nav.estGyroBiasY[id], nav.estGyroBiasZ[id]};

        auto updateScalar = [&](std::size_t measurementIndex,
                                double measurement,
                                double variance) {
            variance = std::max(variance, kMinCovariance);
            const double innovation = measurement -
                (baseState[measurementIndex] + correction[measurementIndex]);
            const double innovationVariance = covariance[covarianceIndex(
                measurementIndex, measurementIndex)] + variance;
            if (innovationVariance <= kMinCovariance) return;

            const Covariance prior = covariance;
            for (std::size_t row = 0; row < kErrorStateSize; ++row) {
                correction[row] += prior[covarianceIndex(row, measurementIndex)] /
                    innovationVariance * innovation;
            }
            for (std::size_t row = 0; row < kErrorStateSize; ++row) {
                for (std::size_t column = 0; column < kErrorStateSize; ++column) {
                    covariance[covarianceIndex(row, column)] =
                        prior[covarianceIndex(row, column)] -
                        prior[covarianceIndex(row, measurementIndex)] *
                        prior[covarianceIndex(measurementIndex, column)] /
                        innovationVariance;
                }
            }
            boundCovariance(covariance, nav.covarianceDiag[id]);
        };

        updateScalar(0, sensors.gpsPosX[id], rPos);
        updateScalar(1, sensors.gpsPosY[id], rPos);
        updateScalar(2, sensors.gpsPosZ[id], rPos);
        updateScalar(3, sensors.gpsVelX[id], rVel);
        updateScalar(4, sensors.gpsVelY[id], rVel);
        updateScalar(5, sensors.gpsVelZ[id], rVel);

        nav.estPx[id] += correction[0];
        nav.estPy[id] += correction[1];
        nav.estPz[id] += correction[2];
        nav.estVx[id] += correction[3];
        nav.estVy[id] += correction[4];
        nav.estVz[id] += correction[5];
        applyAttitudeError(nav, id, correction[6], correction[7], correction[8]);
        nav.estAccelBiasX[id] += correction[9];
        nav.estAccelBiasY[id] += correction[10];
        nav.estAccelBiasZ[id] += correction[11];
        nav.estGyroBiasX[id] += correction[12];
        nav.estGyroBiasY[id] += correction[13];
        nav.estGyroBiasZ[id] += correction[14];
    }

    void NavigationSystem::update(const SensorBlock& sensors, const PhysicsBlock& physics, NavigationBlock& nav, double dt) {
        nav.size = sensors.size;
        ensureCapacity(nav.size, nav);

        for (std::size_t i = 0; i < nav.size; ++i) {
            // Initial alignment ("perfect initialization"): state is taken
            // from truth at launch — GPS may not have produced its first
            // sample yet, and attitude/rates live in the physics block.
            // (Initial uncertainty can be added later for realism.)
            if (!nav.isAligned[i]) {
                nav.estPx[i] = physics.px[i];
                nav.estPy[i] = physics.py[i];
                nav.estPz[i] = physics.pz[i];
                nav.estVx[i] = physics.vx[i];
                nav.estVy[i] = physics.vy[i];
                nav.estVz[i] = physics.vz[i];
                nav.estQw[i] = physics.qw[i];
                nav.estQx[i] = physics.qx[i];
                nav.estQy[i] = physics.qy[i];
                nav.estQz[i] = physics.qz[i];
                nav.estWx[i] = physics.wx[i];
                nav.estWy[i] = physics.wy[i];
                nav.estWz[i] = physics.wz[i];
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
