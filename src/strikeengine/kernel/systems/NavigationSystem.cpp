#include <strikeengine/kernel/systems/NavigationSystem.hpp>
#include <strikeengine/kernel/math/Quaternion.hpp>
#include <strikeengine/models/physics/earth/EarthFrames.hpp>
#include <strikeengine/models/physics/earth/EarthFixedPropagator.hpp>
#include <algorithm>
#include <cmath>
#include <array>

namespace {

constexpr std::size_t kErrorStateSize = 15;
constexpr double kMinCovariance = 1e-12;
constexpr double kMaxCovariance = 1e6;
using Covariance = std::array<double, kErrorStateSize * kErrorStateSize>;

// Single-interval sculling/rotation compensation coefficient. For a constant
// body rate omega and specific force f over one interval, the exact world
// velocity increment is C(tn-1)[f dt + k (omega x f) dt^2] with k = 1/2. The
// two-interval Bortz coning/sculling corrections (coefficient 2/3 over the
// previous and current increments) were validated numerically against a
// fine-step reference integrator (see coning_sculling_test): they do not
// improve the point-sampled per-step scheme used here (the optimal coefficient
// is frequency-dependent), so only the single-interval term is shipped.
constexpr double kScullingCoefficient = 0.5;

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

    void NavigationSystem::strapdownINS(
        std::size_t id,
        const SensorBlock& sensors,
        NavigationBlock& nav,
        double dt,
        const EnvironmentConfig& environment) {
        // 1. Compensate IMU measurements
        double fx = sensors.accelX[id] - nav.estAccelBiasX[id];
        double fy = sensors.accelY[id] - nav.estAccelBiasY[id];
        double fz = sensors.accelZ[id] - nav.estAccelBiasZ[id];

        double wx = sensors.gyroX[id] - nav.estGyroBiasX[id];
        double wy = sensors.gyroY[id] - nav.estGyroBiasY[id];
        double wz = sensors.gyroZ[id] - nav.estGyroBiasZ[id];

        // Earth-rate compensation (ECEF truth, opt-in): with the gyro modeled
        // as measuring the inertial body rate, subtract the earth-rotation
        // vector resolved into the body frame via the ESTIMATED attitude so
        // the INS propagates the earth-fixed body rate without spurious drift.
        if (environment.earth.useEcefTruth && environment.earth.includeEarthRateGyro) {
            double ewx, ewy, ewz;
            quatRotateToBody(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                             0.0, 0.0, Models::EarthModel::earthRotationRateRadPerSec,
                             ewx, ewy, ewz);
            wx -= ewx;
            wy -= ewy;
            wz -= ewz;
        }

        nav.estWx[id] = wx;
        nav.estWy[id] = wy;
        nav.estWz[id] = wz;

        // 1b. Sculling / velocity rotation compensation: the body rotates
        // while the body-frame specific force is measured, so the exact world
        // increment over the interval is
        //     dv^n = C(tn-1) [ f dt + k (omega x f) dt^2 ] , k = 1/2
        // for a constant rate and force. Apply the single-interval term in
        // the body frame before the rotation to world.
        const double sculX = kScullingCoefficient * (wy * fz - wz * fy) * dt;
        const double sculY = kScullingCoefficient * (wz * fx - wx * fz) * dt;
        const double sculZ = kScullingCoefficient * (wx * fy - wy * fx) * dt;
        fx += sculX;
        fy += sculY;
        fz += sculZ;

        // 2. Rotate specific force to World frame
        double wfx, wfy, wfz;
        rotateBodyToWorld(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id], fx, fy, fz, wfx, wfy, wfz);

        std::array<double, 3> gravity{0.0, 0.0, -9.80665};
        std::array<double, 3> earthAcceleration{0.0, 0.0, 0.0};
        if (environment.earth.useEcefTruth) {
            const Models::EcefCoordinate position{
                nav.estPx[id], nav.estPy[id], nav.estPz[id]};
            const auto geodetic = Models::ecefToGeodetic(position);
            if (environment.earth.includeJ2Gravity) {
                gravity = Models::EarthFrames::toVector(
                    Models::j2GravityAccelerationEcef(position));
            } else if (environment.earth.useWgs84Gravity) {
                gravity = Models::EarthFrames::ecefNormalGravityAcceleration(geodetic);
            } else {
                gravity = Models::EarthFrames::toVector(
                    Models::sphericalGravityAccelerationEcef(position));
            }
            earthAcceleration = Models::EarthFrames::toVector(
                Models::EarthFixed::acceleration(
                    position,
                    {nav.estVx[id], nav.estVy[id], nav.estVz[id]},
                    {},
                    {false,
                     environment.earth.includeCoriolis,
                     environment.earth.includeCentrifugal}));
        } else if (environment.earth.includeJ2Gravity) {
            const Models::GeodeticCoordinate reference{
                environment.earth.referenceLatitudeRad,
                environment.earth.referenceLongitudeRad,
                0.0};
            gravity = Models::EarthFrames::localJ2GravityAcceleration(
                Models::EarthFrames::enuToGeodetic(
                    {nav.estPx[id], nav.estPy[id], nav.estPz[id]}, reference));
        } else if (environment.earth.useWgs84Gravity) {
            gravity[2] = -Models::normalGravity(
                environment.earth.referenceLatitudeRad, nav.estPz[id]);
        } else if (environment.earth.useSphericalGravity) {
            const Models::GeodeticCoordinate reference{
                environment.earth.referenceLatitudeRad,
                environment.earth.referenceLongitudeRad,
                0.0};
            gravity = Models::EarthFrames::localSphericalGravityAcceleration(
                Models::EarthFrames::enuToGeodetic(
                    {nav.estPx[id], nav.estPy[id], nav.estPz[id]}, reference));
        }
        if (!environment.earth.useEcefTruth && environment.earth.includeCoriolis) {
            earthAcceleration = Models::localCoriolisAcceleration(
                environment.earth.referenceLatitudeRad,
                {nav.estVx[id], nav.estVy[id], nav.estVz[id]});
        }

        // 3. Add frame gravity and rotating-earth terms.
        const double ax = wfx + gravity[0] + earthAcceleration[0];
        const double ay = wfy + gravity[1] + earthAcceleration[1];
        const double az = wfz + gravity[2] + earthAcceleration[2];

        // 4. Integrate velocity
        nav.estVx[id] += ax * dt;
        nav.estVy[id] += ay * dt;
        nav.estVz[id] += az * dt;

        // 5. Integrate position
        nav.estPx[id] += nav.estVx[id] * dt;
        nav.estPy[id] += nav.estVy[id] * dt;
        nav.estPz[id] += nav.estVz[id] * dt;

        // 6. Integrate attitude with a rotation-vector update. The current
        // angular increment (compensated body rate times dt) forms the
        // rotation vector phi; the corresponding delta quaternion is
        // right-multiplied onto the body->world attitude, matching the
        // q_dot = 0.5 q (x) (0, omega) convention used by the truth model.
        // Note: the textbook two-interval coning cross-term (2/3)(prev x cur)
        // was evaluated against a fine-step reference and does not improve
        // the point-sampled per-step scheme (its optimum coefficient is
        // frequency-dependent), so it is intentionally omitted here.
        const double phiX = wx * dt;
        const double phiY = wy * dt;
        const double phiZ = wz * dt;

        const double phiMag = std::sqrt(phiX * phiX + phiY * phiY + phiZ * phiZ);
        double dqw, dqx, dqy, dqz;
        if (phiMag > 1e-12) {
            const double half = 0.5 * phiMag;
            const double scale = std::sin(half) / phiMag;
            dqw = std::cos(half);
            dqx = phiX * scale;
            dqy = phiY * scale;
            dqz = phiZ * scale;
        } else {
            dqw = 1.0;
            dqx = dqy = dqz = 0.0;
        }

        const double qw = nav.estQw[id];
        const double qx = nav.estQx[id];
        const double qy = nav.estQy[id];
        const double qz = nav.estQz[id];
        nav.estQw[id] = qw * dqw - qx * dqx - qy * dqy - qz * dqz;
        nav.estQx[id] = qw * dqx + qx * dqw + qy * dqz - qz * dqy;
        nav.estQy[id] = qw * dqy - qx * dqz + qy * dqw + qz * dqx;
        nav.estQz[id] = qw * dqz + qx * dqy - qy * dqx + qz * dqw;

        // Normalize
        double norm = std::sqrt(nav.estQw[id]*nav.estQw[id] + nav.estQx[id]*nav.estQx[id] +
                                nav.estQy[id]*nav.estQy[id] + nav.estQz[id]*nav.estQz[id]);
        if (norm > 0) {
            nav.estQw[id] /= norm;
            nav.estQx[id] /= norm;
            nav.estQy[id] /= norm;
            nav.estQz[id] /= norm;
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

    void NavigationSystem::update(
        const SensorBlock& sensors,
        const PhysicsBlock& physics,
        NavigationBlock& nav,
        double dt,
        const EnvironmentConfig& environment) {
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
            strapdownINS(i, sensors, nav, dt, environment);

            // Low rate EKF fusion
            if (sensors.gpsUpdated[i]) {
                ekfUpdate(i, sensors, nav);
            }
        }
    }

} // namespace StrikeEngine::Kernel
