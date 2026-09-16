#include <strikeengine/kernel/systems/NavigationSystem.hpp>
#include <strikeengine/kernel/math/Quaternion.hpp>
#include <strikeengine/models/physics/earth/EarthFrames.hpp>
#include <strikeengine/models/physics/earth/EarthFixedPropagator.hpp>
#include <strikeengine/models/physics/earth/MagneticModel.hpp>
#include <algorithm>
#include <cmath>
#include <array>
#include <random>

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
    // Tight initial uncertainty: the INS is aligned from truth (perfect
    // initialization), and the attitude/heading correction must not be
    // over-applied at each GPS fix (a 0.1 (std 18 deg) attitude variance drove
    // the EKF to over-correct and diverge). POS=10 m, VEL=1 m/s,
    // ATTITUDE=1e-4 (std ~0.6 deg), a-bias=0.01, g-bias=4e-5 (std ~0.4 deg/s).
    return {10.0, 10.0, 10.0, 1.0, 1.0, 1.0,
            1e-4, 1e-4, 1e-4, 0.01, 0.01, 0.01,
            4e-5, 4e-5, 4e-5};
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
                        double dz){
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

// Defensive SensorBlock reads: direct unit tests hand-build blocks without
// the newer config arrays, so every post-legacy read defaults instead of
// running off the end. Kernel-driven runs always wire full arrays.
static bool sensFlag(const std::vector<bool>& v, std::size_t i)
{
    return i < v.size() && v[i];
}

static double sensVal(const std::vector<double>& v, std::size_t i, double def)
{
    return i < v.size() ? v[i] : def;
}

static int sensInt(const std::vector<int>& v, std::size_t i, int def)
{
    return i < v.size() ? v[i] : def;
}

// --- World-frame gravity + rotating-earth acceleration --------------------
// Factored out of the INS mechanization so the gravity gradient and the
// earth-rotation Jacobian are derived from exactly the model being integrated.
std::array<double, 3> worldGravity(
    const StrikeEngine::Kernel::EnvironmentConfig& environment,
    double px, double py, double pz)
{
    using namespace StrikeEngine::Models;
    std::array<double, 3> gravity{0.0, 0.0, -9.80665};
    if (environment.earth.useEcefTruth) {
        const EcefCoordinate position{px, py, pz};
        const auto geodetic = ecefToGeodetic(position);
        if (environment.earth.includeJ2Gravity) {
            gravity = EarthFrames::toVector(j2GravityAccelerationEcef(position));
        } else if (environment.earth.useWgs84Gravity) {
            gravity = EarthFrames::ecefNormalGravityAcceleration(geodetic);
        } else {
            gravity = EarthFrames::toVector(sphericalGravityAccelerationEcef(position));
        }
    } else if (environment.earth.includeJ2Gravity) {
        const GeodeticCoordinate reference{
            environment.earth.referenceLatitudeRad,
            environment.earth.referenceLongitudeRad, 0.0};
        gravity = EarthFrames::localJ2GravityAcceleration(
            EarthFrames::enuToGeodetic({px, py, pz}, reference));
    } else if (environment.earth.useWgs84Gravity) {
        gravity[2] = -normalGravity(environment.earth.referenceLatitudeRad, pz);
    } else if (environment.earth.useSphericalGravity) {
        const GeodeticCoordinate reference{
            environment.earth.referenceLatitudeRad,
            environment.earth.referenceLongitudeRad, 0.0};
        gravity = EarthFrames::localSphericalGravityAcceleration(
            EarthFrames::enuToGeodetic({px, py, pz}, reference));
    }
    return gravity;
}

std::array<double, 3> worldEarthAcceleration(
    const StrikeEngine::Kernel::EnvironmentConfig& environment,
    double px, double py, double pz, double vx, double vy, double vz)
{
    using namespace StrikeEngine::Models;
    std::array<double, 3> acceleration{0.0, 0.0, 0.0};
    if (environment.earth.useEcefTruth) {
        acceleration = EarthFrames::toVector(EarthFixed::acceleration(
            EcefCoordinate{px, py, pz}, EcefCoordinate{vx, vy, vz}, {},
            EcefDynamicsOptions{false, environment.earth.includeCoriolis,
                                environment.earth.includeCentrifugal, false}));
    } else if (environment.earth.includeCoriolis) {
        acceleration = localCoriolisAcceleration(
            environment.earth.referenceLatitudeRad, {vx, vy, vz});
    }
    return acceleration;
}

// Dominant gravity gradient dg/dp expressed in the mechanization frame. The
// spherical radial form is used for every position-dependent gravity model
// (it is exact for point-mass gravity and captures the vertical/Schuler
// stiffening for J2 and normal gravity). Flat constant gravity returns zero.
void worldGravityGradient(
    const StrikeEngine::Kernel::EnvironmentConfig& environment,
    double px, double py, double pz, double G[3][3])
{
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) G[r][c] = 0.0;

    const bool positionDependent = environment.earth.useEcefTruth ||
        environment.earth.useSphericalGravity ||
        environment.earth.useWgs84Gravity ||
        environment.earth.includeJ2Gravity;
    if (!positionDependent) return;

    const auto g = worldGravity(environment, px, py, pz);
    const double gMag = std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    if (!(gMag > 0.0)) return;

    if (environment.earth.useEcefTruth) {
        const double r = std::sqrt(px * px + py * py + pz * pz);
        if (r < 1.0) return;
        const double u[3] = {px / r, py / r, pz / r};
        const double radial = 2.0 * gMag / r;   // along rhat (up)
        const double tangential = gMag / r;     // across rhat
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                const double delta = (i == j) ? 1.0 : 0.0;
                G[i][j] = radial * u[i] * u[j] -
                          tangential * (delta - u[i] * u[j]);
            }
        }
    } else {
        // Local ENU: only the vertical channel is position dependent.
        constexpr double kEarthRadiusM = 6371000.0;
        G[2][2] = 2.0 * gMag / kEarthRadiusM;
    }
}

// Jacobians of the rotating-earth acceleration: d(ea)/d(v) (Coriolis) and
// d(ea)/d(p) (centrifugal). Zero unless the corresponding term is active.
void worldEarthAccelerationJacobian(
    const StrikeEngine::Kernel::EnvironmentConfig& environment,
    double dAdv[3][3], double dAdp[3][3])
{
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) { dAdv[r][c] = 0.0; dAdp[r][c] = 0.0; }

    const double omega = StrikeEngine::Models::EarthModel::earthRotationRateRadPerSec;
    if (environment.earth.useEcefTruth) {
        if (environment.earth.includeCoriolis) {
            // ea_co = -2 (Omega x v) with Omega = (0,0,w):
            //   d(ea)/d(v) = [[0, 2w, 0], [-2w, 0, 0], [0, 0, 0]]
            dAdv[0][1] = 2.0 * omega;
            dAdv[1][0] = -2.0 * omega;
        }
        if (environment.earth.includeCentrifugal) {
            // ea_cf = |w|^2 r - (Omega.r) Omega; d/dr = |w|^2 I - Omega Omega^T
            const double omega2 = omega * omega;
            dAdp[0][0] = omega2;
            dAdp[1][1] = omega2;
            dAdp[2][2] = 0.0;
        }
    } else if (environment.earth.includeCoriolis) {
        const double lat = environment.earth.referenceLatitudeRad;
        const double omegaN = omega * std::cos(lat);
        const double omegaU = omega * std::sin(lat);
        // ea = [2(wU vy - wN vz), -2 wU vx, 2 wN vx]
        dAdv[0][1] = 2.0 * omegaU;
        dAdv[0][2] = -2.0 * omegaN;
        dAdv[1][0] = -2.0 * omegaU;
        dAdv[2][0] = 2.0 * omegaN;
    }
}

// Inverse standard-normal CDF (Acklam's rational approximation) and the
// Wilson-Hilferty chi-square quantile, used only to derive a gate threshold
// when the caller leaves it at <= 0.
double inverseNormalCdf(double p)
{
    static const double a[6] = {-3.969683028665376e+01, 2.209460984245205e+02,
        -2.759285104469687e+02, 1.383577518672690e+02, -3.066479806614716e+01,
        2.506628277459239e+00};
    static const double b[5] = {-5.447609879822406e+01, 1.615858368580409e+02,
        -1.556989798598866e+02, 6.680131188771972e+01, -1.328068155288572e+01};
    static const double c[6] = {-7.784894002430293e-03, -3.223964580411365e-01,
        -2.400758277161838e+00, -2.549732539343734e+00, 4.374664141464968e+00,
        2.938163982698783e+00};
    static const double d[4] = {7.784695709041462e-03, 3.224671290700398e-01,
        2.445134137142996e+00, 3.754408661907416e+00};
    constexpr double pLow = 0.02425;
    constexpr double pHigh = 1.0 - pLow;
    if (p <= 0.0) return -1e300;
    if (p >= 1.0) return 1e300;
    if (p < pLow) {
        const double q = std::sqrt(-2.0 * std::log(p));
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
               ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
    }
    if (p <= pHigh) {
        const double q = p - 0.5;
        const double r = q * q;
        return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q /
               (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0);
    }
    const double q = std::sqrt(-2.0 * std::log(1.0 - p));
    return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
            ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
}

double chiSquareQuantile(double confidence, int dof)
{
    if (dof <= 0) return 0.0;
    const double z = inverseNormalCdf(confidence);
    const double t = 1.0 - 2.0 / (9.0 * dof) + z * std::sqrt(2.0 / (9.0 * dof));
    return dof * t * t * t;
}

// 6x6 Gauss-Jordan inverse with partial pivoting; false if singular.
bool invert6(const double in[6][6], double out[6][6])
{
    double aug[6][12];
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            aug[r][c] = in[r][c];
            aug[r][c + 6] = (r == c) ? 1.0 : 0.0;
        }
    }
    for (int col = 0; col < 6; ++col) {
        int pivot = col;
        double best = std::abs(aug[col][col]);
        for (int r = col + 1; r < 6; ++r) {
            const double v = std::abs(aug[r][col]);
            if (v > best) { best = v; pivot = r; }
        }
        if (best < 1e-18) return false;
        if (pivot != col) {
            for (int c = 0; c < 12; ++c) std::swap(aug[col][c], aug[pivot][c]);
        }
        const double diag = aug[col][col];
        for (int c = 0; c < 12; ++c) aug[col][c] /= diag;
        for (int r = 0; r < 6; ++r) {
            if (r == col) continue;
            const double factor = aug[r][col];
            if (factor == 0.0) continue;
            for (int c = 0; c < 12; ++c) aug[r][c] -= factor * aug[col][c];
        }
    }
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) out[r][c] = aug[r][c + 6];
    return true;
}

} // namespace

namespace StrikeEngine::Kernel {

    void NavigationSystem::setSeed(std::uint32_t seed) {
        // Alignment-error draws only (see initialAttitudeErrorDeg and
        // friends). The stream is untouched when all sigmas are zero, so
        // legacy runs are bit-identical with or without this call.
        alignRng.seed(seed ^ 0x51AB1Eu);
    }

    void NavigationSystem::resetEntity(NavigationBlock& nav, std::size_t id) {
        ensureCapacity(id + 1, nav);
        const auto defaultDiag = initialCovarianceDiag();
        initializeCovariance(nav.covarianceFull[id], defaultDiag);
        nav.covarianceDiag[id] = defaultDiag;
        nav.isAligned[id] = false;
        nav.estAccelBiasX[id] = nav.estAccelBiasY[id] = nav.estAccelBiasZ[id] = 0.0;
        nav.estGyroBiasX[id] = nav.estGyroBiasY[id] = nav.estGyroBiasZ[id] = 0.0;
        nav.prevDThetaX[id] = nav.prevDThetaY[id] = nav.prevDThetaZ[id] = 0.0;
        nav.prevDVelX[id] = nav.prevDVelY[id] = nav.prevDVelZ[id] = 0.0;
        nav.lastGpsUpdateRejected[id] = false;
        nav.lastGpsMaxInnovationSigma[id] = 0.0;
        nav.lastBaroRejected[id] = false;
        nav.lastMagRejected[id] = false;
    }

    void NavigationSystem::ensureCapacity(std::size_t size, NavigationBlock& nav) {
        const auto defaultDiag = initialCovarianceDiag();
        if (nav.estPx.size() < size) {
            nav.estPx.resize(size, 0.0); nav.estPy.resize(size, 0.0); nav.estPz.resize(size, 0.0);
            nav.estVx.resize(size, 0.0); nav.estVy.resize(size, 0.0); nav.estVz.resize(size, 0.0);
            nav.estAx.resize(size, 0.0); nav.estAy.resize(size, 0.0); nav.estAz.resize(size, 0.0);
            nav.estQx.resize(size, 0.0); nav.estQy.resize(size, 0.0); nav.estQz.resize(size, 0.0); nav.estQw.resize(size, 1.0);
            nav.estWx.resize(size, 0.0); nav.estWy.resize(size, 0.0); nav.estWz.resize(size, 0.0);
            nav.estAccelBiasX.resize(size, 0.0); nav.estAccelBiasY.resize(size, 0.0); nav.estAccelBiasZ.resize(size, 0.0);
            nav.estGyroBiasX.resize(size, 0.0); nav.estGyroBiasY.resize(size, 0.0); nav.estGyroBiasZ.resize(size, 0.0);
            nav.covarianceDiag.resize(size, defaultDiag);
            nav.isAligned.resize(size, false);
        }
        nav.lastGpsUpdateRejected.resize(size, false);
        nav.lastGpsMaxInnovationSigma.resize(size, 0.0);
        nav.lastBaroRejected.resize(size, false);
        nav.lastMagRejected.resize(size, false);
        if (nav.prevDThetaX.size() < size) {
            nav.prevDThetaX.resize(size, 0.0); nav.prevDThetaY.resize(size, 0.0); nav.prevDThetaZ.resize(size, 0.0);
            nav.prevDVelX.resize(size, 0.0); nav.prevDVelY.resize(size, 0.0); nav.prevDVelZ.resize(size, 0.0);
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

        // 1b. Rotation/sculling compensation. Legacy path: the exact
        // single-interval term dv = C(tn-1)[f dt + k (omega x f) dt^2],
        // k = 1/2, for constant rate and force. Two-sample path (opt-in):
        // Titterton & Weston dual-sample (N=2) coning 9/20 (dTh0 x dTh1)
        // on the attitude increment and sculling 9/20
        // (dTh0 x dv1 + dv0 x dTh1) on the velocity increment. The two paths
        // are mutually exclusive; the legacy math below is byte-untouched
        // when the option is off.
        const double dth1x = wx * dt, dth1y = wy * dt, dth1z = wz * dt;
        const double dv1x = fx * dt, dv1y = fy * dt, dv1z = fz * dt;
        double conX = 0.0, conY = 0.0, conZ = 0.0; // attitude-increment add (rad)
        const bool twoSample = sensFlag(sensors.insConingCompensationEnabled, id);
        if (twoSample) {
            constexpr double kTwoSample = 9.0 / 20.0;
            const double ptx = nav.prevDThetaX[id], pty = nav.prevDThetaY[id], ptz = nav.prevDThetaZ[id];
            const double pvx = nav.prevDVelX[id], pvy = nav.prevDVelY[id], pvz = nav.prevDVelZ[id];
            conX = kTwoSample * (pty * dth1z - ptz * dth1y);
            conY = kTwoSample * (ptz * dth1x - ptx * dth1z);
            conZ = kTwoSample * (ptx * dth1y - pty * dth1x);
            // Sculling velocity increment dVs = k[(dTh0 x dv1) + (dv0 x dTh1)];
            // folded back into specific force (divide by dt).
            fx += (kTwoSample / dt) * ((pty * dv1z - ptz * dv1y) + (pvy * dth1z - pvz * dth1y));
            fy += (kTwoSample / dt) * ((ptz * dv1x - ptx * dv1z) + (pvz * dth1x - pvx * dth1z));
            fz += (kTwoSample / dt) * ((ptx * dv1y - pty * dv1x) + (pvx * dth1y - pvy * dth1x));
        } else {
            const double sculX = kScullingCoefficient * (wy * fz - wz * fy) * dt;
            const double sculY = kScullingCoefficient * (wz * fx - wx * fz) * dt;
            const double sculZ = kScullingCoefficient * (wx * fy - wy * fx) * dt;
            fx += sculX;
            fy += sculY;
            fz += sculZ;
        }
        nav.prevDThetaX[id] = dth1x; nav.prevDThetaY[id] = dth1y; nav.prevDThetaZ[id] = dth1z;
        nav.prevDVelX[id] = dv1x; nav.prevDVelY[id] = dv1y; nav.prevDVelZ[id] = dv1z;

        // 2. Rotate specific force to World frame
        double wfx, wfy, wfz;
        rotateBodyToWorld(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id], fx, fy, fz, wfx, wfy, wfz);

        const std::array<double, 3> gravity = worldGravity(
            environment, nav.estPx[id], nav.estPy[id], nav.estPz[id]);
        const std::array<double, 3> earthAcceleration = worldEarthAcceleration(
            environment, nav.estPx[id], nav.estPy[id], nav.estPz[id],
            nav.estVx[id], nav.estVy[id], nav.estVz[id]);

        // 3. Add frame gravity and rotating-earth terms.
        const double ax = wfx + gravity[0] + earthAcceleration[0];
        const double ay = wfy + gravity[1] + earthAcceleration[1];
        const double az = wfz + gravity[2] + earthAcceleration[2];
        nav.estAx[id] = ax;
        nav.estAy[id] = ay;
        nav.estAz[id] = az;

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
        const double phiX = wx * dt + conX;
        const double phiY = wy * dt + conY;
        const double phiZ = wz * dt + conZ;

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

        // Position-dependent gravity (Schuler/vertical) and rotating-earth
        // Jacobian blocks. Opt-in: flat constant-gravity legacy runs keep the
        // simplified error model. See worldGravityGradient.
        if (sensFlag(sensors.insGravityGradientEnabled, id)) {
            double G[3][3];
            worldGravityGradient(environment, nav.estPx[id], nav.estPy[id],
                                 nav.estPz[id], G);
            for (std::size_t row = 0; row < 3; ++row) {
                for (std::size_t column = 0; column < 3; ++column) {
                    transition[covarianceIndex(row + 3, column)] +=
                        G[row][column] * dt;
                }
            }
        }
        if (sensFlag(sensors.insEarthRotationCouplingEnabled, id)) {
            double dAdv[3][3], dAdp[3][3];
            worldEarthAccelerationJacobian(environment, dAdv, dAdp);
            for (std::size_t row = 0; row < 3; ++row) {
                for (std::size_t column = 0; column < 3; ++column) {
                    transition[covarianceIndex(row + 3, column + 3)] +=
                        dAdv[row][column] * dt;
                    transition[covarianceIndex(row + 3, column)] +=
                        dAdp[row][column] * dt;
                }
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
        // Adaptive Q (opt-in): scale the IMU white-noise-driven diagonals
        // with measured dynamics so the covariance stays honest under
        // high-g/high-rate flight (spec-sheet Q understates vibration and
        // coning residuals there, which silently tightens gates). Disabled =
        // the array above, untouched.
        double adaptiveFactor = 1.0;
        if (sensFlag(sensors.insAdaptiveQEnabled, id)) {
            const double wMag = std::sqrt(wx * wx + wy * wy + wz * wz);
            const double fMag = std::sqrt(fx * fx + fy * fy + fz * fz);
            const double dyn = std::min(10.0, (wMag / 0.5) * (fMag / 20.0));
            adaptiveFactor = 1.0 + std::max(0.0, sensVal(sensors.insAdaptiveQGain, id, 1.0)) * dyn;
        }
        for (std::size_t i = 0; i < kErrorStateSize; ++i) {
            propagated[covarianceIndex(i, i)] +=
                processNoise[i] * (i < 9 ? adaptiveFactor : 1.0);
        }
        covariance = propagated;
        boundCovariance(covariance, nav.covarianceDiag[id]);
    }

    void NavigationSystem::ekfUpdate(std::size_t id, const SensorBlock& sensors,
                                   NavigationBlock& nav, const EnvironmentConfig& environment) {
        // Sequential scalar GPS position/velocity updates. The full covariance
        // couples the observed translational states to attitude and IMU bias
        // corrections instead of applying independent diagonal gains.
        nav.lastGpsUpdateRejected[id] = false;
        nav.lastGpsMaxInnovationSigma[id] = 0.0;
        nav.lastBaroRejected[id] = false;
        nav.lastMagRejected[id] = false;
        const double innovationGate = id < sensors.gpsInnovationGateSigma.size()
            ? sensors.gpsInnovationGateSigma[id] : 0.0;
        double rPos = sensVal(sensors.gpsPosNoiseStdDev, id, 5.0);
        rPos *= rPos;
        double rVel = sensVal(sensors.gpsVelNoiseStdDev, id, 0.5);
        rVel *= rVel;

        auto& covariance = nav.covarianceFull[id];
        std::array<double, kErrorStateSize> correction{};
        // Magnetometer attitude corrections accumulate separately: GPS yaw is
        // damped 10x (weakly observable), but heading is the magnetometer's
        // whole job, so its yaw applies undamped at the end.
        std::array<double, kErrorStateSize> magCorrection{};
        const std::array<double, kErrorStateSize> baseState = {
            nav.estPx[id], nav.estPy[id], nav.estPz[id],
            nav.estVx[id], nav.estVy[id], nav.estVz[id],
            0.0, 0.0, 0.0,
            nav.estAccelBiasX[id], nav.estAccelBiasY[id], nav.estAccelBiasZ[id],
            nav.estGyroBiasX[id], nav.estGyroBiasY[id], nav.estGyroBiasZ[id]};

        // Generalized scalar row update: z ~= H.x + noise(R). Returns the
        // normalized innovation sigma, or -1 when skipped (non-finite input
        // or degenerate variance — same silent-skip contract as legacy).
        // Gate rejects set the caller's flag (GPS rows keep the legacy
        // lastGpsUpdateRejected semantics exactly).
        // vector<bool> element proxies cannot bind to bool&, so rejection
        // flows through locals published to the nav block at the end.
        bool gpsRej = false, baroRej = false, magRej = false;
        // Baro/mag rejection used to share the GPS gate, so disabling GPS
        // gating silently disabled theirs too. Each now has its own gate;
        // negative follows the GPS gate (legacy).
        const double baroGateCfg = sensVal(sensors.baroInnovationGateSigma, id, -1.0);
        const double magGateCfg = sensVal(sensors.magInnovationGateSigma, id, -1.0);
        const double baroGate = baroGateCfg >= 0.0 ? baroGateCfg : innovationGate;
        const double magGate = magGateCfg >= 0.0 ? magGateCfg : innovationGate;
        auto updateRow = [&](const double (&H)[kErrorStateSize], double z,
                             double variance, bool* rejectedFlag,
                             std::array<double, kErrorStateSize>& acc,
                             bool freezeAttitude = false,
                             double gateOverride = -1.0) -> double {
            if (!std::isfinite(z)) {
                if (rejectedFlag) *rejectedFlag = true;
                return -1.0;
            }
            double pred = 0.0;
            for (std::size_t k = 0; k < kErrorStateSize; ++k) {
                pred += H[k] * (baseState[k] + correction[k] + magCorrection[k]);
            }
            const double innovation = z - pred;
            double hph = 0.0;
            for (std::size_t r = 0; r < kErrorStateSize; ++r) {
                for (std::size_t c = 0; c < kErrorStateSize; ++c) {
                    hph += H[r] * H[c] * covariance[covarianceIndex(r, c)];
                }
            }
            variance = std::max(variance, kMinCovariance);
            const double innovationVariance = hph + variance;
            if (innovationVariance <= kMinCovariance) return -1.0;
            const double innovationSigma = std::abs(innovation) /
                std::sqrt(innovationVariance);
            const double gate = gateOverride >= 0.0 ? gateOverride : innovationGate;
            if (gate > 0.0 &&
                (!std::isfinite(innovationSigma) || innovationSigma > gate)) {
                if (rejectedFlag) *rejectedFlag = true;
                // Report the sigma even for gate rejects: the max-innovation
                // diagnostic must reflect rejected fixes (legacy contract).
                return std::isfinite(innovationSigma) ? innovationSigma : -1.0;
            }

            const Covariance prior = covariance;
            double ph[kErrorStateSize];
            for (std::size_t row = 0; row < kErrorStateSize; ++row) {
                ph[row] = 0.0;
                for (std::size_t k = 0; k < kErrorStateSize; ++k) {
                    ph[row] += prior[covarianceIndex(row, k)] * H[k];
                }
            }
            if (freezeAttitude) {
                // Baro rows must not steer attitude or gyro bias (see call
                // site): the tilt-vertical coupling turns baro noise into a
                // pitch/roll random walk through gravity misprojection, both
                // directly and via the gyro-bias states (bias error
                // integrates into attitude permanently — the worse channel).
                // A barometer physically observes neither orientation nor
                // rotation rate. Pos/vel/accel-bias corrections (the reason
                // baro exists) still apply.
                ph[6] = 0.0; ph[7] = 0.0; ph[8] = 0.0;
                ph[12] = 0.0; ph[13] = 0.0; ph[14] = 0.0;
            }
            for (std::size_t row = 0; row < kErrorStateSize; ++row) {
                acc[row] += ph[row] / innovationVariance * innovation;
            }
            for (std::size_t row = 0; row < kErrorStateSize; ++row) {
                for (std::size_t column = 0; column < kErrorStateSize; ++column) {
                    covariance[covarianceIndex(row, column)] =
                        prior[covarianceIndex(row, column)] -
                        ph[row] * ph[column] / innovationVariance;
                }
            }
            boundCovariance(covariance, nav.covarianceDiag[id]);
            return innovationSigma;
        };

        auto updateScalar = [&](std::size_t measurementIndex,
                                double measurement,
                                double variance) {
            double H[kErrorStateSize] = {};
            H[measurementIndex] = 1.0;
            const double sigma = updateRow(H, measurement, variance,
                                           &gpsRej, correction);
            if (sigma >= 0.0) {
                nav.lastGpsMaxInnovationSigma[id] = std::max(
                    nav.lastGpsMaxInnovationSigma[id], sigma);
            }
        };

        // GPS antenna lever-arm compensation: the sensor reports the antenna
        // (CM + R.l position, CM + R.(w x l) velocity). Refer the fix to the
        // CM before fusing so the lever arm is not silently absorbed into
        // attitude/velocity. Zero lever arm (the default) is a no-op.
        double gpsPos[3] = {sensors.gpsPosX[id], sensors.gpsPosY[id], sensors.gpsPosZ[id]};
        double gpsVel[3] = {sensors.gpsVelX[id], sensors.gpsVelY[id], sensors.gpsVelZ[id]};
        if (id < sensors.gpsLeverArmX.size() &&
            sensFlag(sensors.gpsLeverArmCompensationEnabled, id)) {
            const double lx = sensors.gpsLeverArmX[id];
            const double ly = sensors.gpsLeverArmY[id];
            const double lz = sensors.gpsLeverArmZ[id];
            if (lx != 0.0 || ly != 0.0 || lz != 0.0) {
                double wl[3];
                rotateBodyToWorld(nav.estQw[id], nav.estQx[id], nav.estQy[id],
                                  nav.estQz[id], lx, ly, lz, wl[0], wl[1], wl[2]);
                gpsPos[0] -= wl[0]; gpsPos[1] -= wl[1]; gpsPos[2] -= wl[2];
                const double ox = nav.estWy[id] * lz - nav.estWz[id] * ly;
                const double oy = nav.estWz[id] * lx - nav.estWx[id] * lz;
                const double oz = nav.estWx[id] * ly - nav.estWy[id] * lx;
                double wo[3];
                rotateBodyToWorld(nav.estQw[id], nav.estQx[id], nav.estQy[id],
                                  nav.estQz[id], ox, oy, oz, wo[0], wo[1], wo[2]);
                gpsVel[0] -= wo[0]; gpsVel[1] -= wo[1]; gpsVel[2] -= wo[2];
            }
        }

        // Whole-fix consistency threshold: explicit when > 0, else derived
        // from the configured dof/confidence (Wilson-Hilferty chi-square).
        auto consistencyThreshold = [&]() {
            const double t = sensVal(sensors.gpsFixConsistencyThreshold, id, 16.81);
            if (t > 0.0) return t;
            return chiSquareQuantile(
                sensVal(sensors.gpsFixConsistencyConfidence, id, 0.99),
                sensInt(sensors.gpsFixConsistencyDof, id, 6));
        };

        bool skipGpsFix = false;
        if (sensors.gpsUpdated[id]) {
            if (sensFlag(sensors.gpsBatchUpdateEnabled, id)) {
                // Batch 6-vector update using the full joint HPH'+R, with the
                // joint NIS gate. Correct cross-axis correlation, at the cost
                // of a 6x6 solve.
                double S[6][6];
                double Y[6];
                for (std::size_t a = 0; a < 6; ++a) {
                    Y[a] = (a < 3 ? gpsPos[a] : gpsVel[a - 3]) - baseState[a];
                }
                for (std::size_t r = 0; r < 6; ++r) {
                    for (std::size_t c = 0; c < 6; ++c) {
                        S[r][c] = covariance[covarianceIndex(r, c)] +
                                  ((r == c) ? (r < 3 ? rPos : rVel) : 0.0);
                    }
                }
                double Sinv[6][6];
                if (!invert6(S, Sinv)) {
                    gpsRej = true;
                    skipGpsFix = true;
                } else {
                    double nis = 0.0;
                    for (std::size_t r = 0; r < 6; ++r)
                        for (std::size_t c = 0; c < 6; ++c)
                            nis += Y[r] * Sinv[r][c] * Y[c];
                    nav.lastGpsMaxInnovationSigma[id] = std::sqrt(std::max(0.0, nis));
                    if (sensFlag(sensors.gpsFixConsistencyEnabled, id) &&
                        nis > consistencyThreshold()) {
                        gpsRej = true;
                        skipGpsFix = true;
                    } else {
                        // K = P H' S^-1 ; x += K Y ; P -= K (H P).
                        double K[15][6];
                        for (std::size_t row = 0; row < kErrorStateSize; ++row) {
                            for (std::size_t j = 0; j < 6; ++j) {
                                double value = 0.0;
                                for (std::size_t i = 0; i < 6; ++i) {
                                    value += covariance[covarianceIndex(row, i)] * Sinv[i][j];
                                }
                                K[row][j] = value;
                            }
                        }
                        const Covariance prior = covariance;
                        for (std::size_t row = 0; row < kErrorStateSize; ++row) {
                            for (std::size_t j = 0; j < 6; ++j) {
                                correction[row] += K[row][j] * Y[j];
                            }
                        }
                        for (std::size_t row = 0; row < kErrorStateSize; ++row) {
                            for (std::size_t col = 0; col < kErrorStateSize; ++col) {
                                double value = 0.0;
                                for (std::size_t j = 0; j < 6; ++j) {
                                    value += K[row][j] * prior[covarianceIndex(j, col)];
                                }
                                covariance[covarianceIndex(row, col)] =
                                    prior[covarianceIndex(row, col)] - value;
                            }
                        }
                        boundCovariance(covariance, nav.covarianceDiag[id]);
                    }
                }
            } else {
                // Legacy path: whole-fix diagonal pre-gate, then sequential
                // scalar position/velocity updates.
                if (sensFlag(sensors.gpsFixConsistencyEnabled, id)) {
                    const int dof = std::clamp(
                        sensInt(sensors.gpsFixConsistencyDof, id, 6), 1, 6);
                    double nis = 0.0;
                    bool fixUsable = true;
                    for (int axis = 0; axis < dof; ++axis) {
                        const double meas = (axis < 3) ? gpsPos[axis] : gpsVel[axis - 3];
                        if (!std::isfinite(meas)) { fixUsable = false; break; }
                        const double var = std::max(
                            (axis < 3 ? rPos : rVel), kMinCovariance);
                        const double innov = meas - baseState[axis];
                        const double s = covariance[covarianceIndex(axis, axis)] + var;
                        if (s <= kMinCovariance) { fixUsable = false; break; }
                        nis += (innov * innov) / s;
                    }
                    if (!fixUsable || nis > consistencyThreshold()) {
                        gpsRej = true;
                        skipGpsFix = true;
                    }
                }
                if (!skipGpsFix) {
                    updateScalar(0, gpsPos[0], rPos);
                    updateScalar(1, gpsPos[1], rPos);
                    updateScalar(2, gpsPos[2], rPos);
                    updateScalar(3, gpsVel[0], rVel);
                    updateScalar(4, gpsVel[1], rVel);
                    updateScalar(5, gpsVel[2], rVel);
                }
            }
        }

        // Barometer altitude (opt-in): linearized about the geodetic up at
        // the estimate (ECEF) or the Z row (local). Bounds the vertical
        // channel between GPS fixes.
        if (sensFlag(sensors.baroUpdated, id)) {
            double H[kErrorStateSize] = {};
            double predAlt = 0.0;
            if (environment.earth.useEcefTruth) {
                const auto geo = Models::ecefToGeodetic(
                    {nav.estPx[id], nav.estPy[id], nav.estPz[id]});
                const double cLat = std::cos(geo.latitudeRad);
                H[0] = cLat * std::cos(geo.longitudeRad);
                H[1] = cLat * std::sin(geo.longitudeRad);
                H[2] = std::sin(geo.latitudeRad);
                predAlt = geo.altitudeM;
            } else {
                H[2] = 1.0;
                predAlt = nav.estPz[id];
            }
            // updateRow predicts H.(base+correction); fold the nonlinear
            // prediction in by shifting the measurement so H.x linearizes
            // about the current estimate (standard EKF relinearization).
            const double linShift = predAlt -
                (H[0] * nav.estPx[id] + H[1] * nav.estPy[id] + H[2] * nav.estPz[id]);
            const double rb = sensVal(sensors.baroNoiseStdDev, id, 1.0);
            const double rBaro = rb * rb;
            const bool freezeAttitude =
                !sensFlag(sensors.baroAttitudeCorrectionEnabled, id);
            updateRow(H, sensVal(sensors.baroAlt, id, 0.0) - linShift, rBaro,
                      &baroRej, correction, freezeAttitude, baroGate);
        }

        // Magnetometer heading aid (opt-in): body-frame residual against the
        // shared dipole reference observes the attitude error states through
        // H = R(q_est) . skew(B_world). Magnitude pre-gate rejects
        // disturbed fields (motors, structures) before they touch the filter.
        if (sensFlag(sensors.magUpdated, id)) {
            Models::EcefCoordinate ecefEst{
                nav.estPx[id], nav.estPy[id], nav.estPz[id]};
            if (!environment.earth.useEcefTruth) {
                const Models::GeodeticCoordinate reference{
                    environment.earth.referenceLatitudeRad,
                    environment.earth.referenceLongitudeRad,
                    0.0};
                ecefEst = Models::geodeticToEcef(
                    Models::EarthFrames::enuToGeodetic(
                        {nav.estPx[id], nav.estPy[id], nav.estPz[id]}, reference));
            }
            const auto field = Models::dipoleMagneticFieldEcef(
                ecefEst.x, ecefEst.y, ecefEst.z);
            const double bMag = std::sqrt(
                field[0] * field[0] + field[1] * field[1] + field[2] * field[2]);
            double mmx = sensVal(sensors.magX, id, 0.0), mmy = sensVal(sensors.magY, id, 0.0), mmz = sensVal(sensors.magZ, id, 0.0);
            const double mMag = std::sqrt(mmx * mmx + mmy * mmy + mmz * mmz);
            const double gate = std::max(0.0, sensVal(sensors.magDisturbanceGateRel, id, 0.25));
            if (!(bMag > 1e-12) || std::abs(mMag - bMag) / bMag > gate) {
                magRej = true;
            } else {
                // Body->world rotation from the estimate (columns = body axes).
                double R[3][3]{};
                rotateBodyToWorld(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                                  1.0, 0.0, 0.0, R[0][0], R[1][0], R[2][0]);
                rotateBodyToWorld(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                                  0.0, 1.0, 0.0, R[0][1], R[1][1], R[2][1]);
                rotateBodyToWorld(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                                  0.0, 0.0, 1.0, R[0][2], R[1][2], R[2][2]);
                // skew(B): (skew . v) = B x v. Sign: with the body-frame
                // error convention R_true = R(q_est)(I + [dth x]) (see
                // applyAttitudeError's right-multiplication), meas - pred =
                // -R.skew(B).dth, hence the negation below. The positive
                // variant is unstable positive feedback (verified: a 10 deg
                // error diverges instead of converging).
                const double S[3][3] = {
                    {0.0, -field[2], field[1]},
                    {field[2], 0.0, -field[0]},
                    {-field[1], field[0], 0.0}};
                const double rm = sensVal(sensors.magNoiseStdDev, id, 50e-9);
                const double rMag = rm * rm;
                const double mvec[3] = {mmx, mmy, mmz};
                for (std::size_t r = 0; r < 3; ++r) {
                    double H[kErrorStateSize] = {};
                    for (std::size_t c = 0; c < 3; ++c) {
                        H[6 + c] = -(R[r][0] * S[0][c] + R[r][1] * S[1][c] + R[r][2] * S[2][c]);
                    }
                    // Predicted body component from the estimate (nonlinear
                    // part); the row linearizes the error about it.
                    const double pred = R[r][0] * field[0] + R[r][1] * field[1] + R[r][2] * field[2];
                    updateRow(H, mvec[r] - pred, rMag, &magRej, magCorrection, false, magGate);
                }
            }
        }

        nav.lastGpsUpdateRejected[id] = gpsRej;
        nav.lastBaroRejected[id] = baroRej;
        nav.lastMagRejected[id] = magRej;
        nav.estPx[id] += correction[0] + magCorrection[0];
        nav.estPy[id] += correction[1] + magCorrection[1];
        nav.estPz[id] += correction[2] + magCorrection[2];
        nav.estVx[id] += correction[3] + magCorrection[3];
        nav.estVy[id] += correction[4] + magCorrection[4];
        nav.estVz[id] += correction[5] + magCorrection[5];
        // The attitude (especially yaw) is only weakly observable from GPS
        // position/velocity, so the full yaw correction over-corrects and drives
        // the attitude to diverge at each GPS fix (observed stair-step growth to
        // >100 deg on a turning aircraft). Keep the full, well-observable
        // roll/pitch correction but heavily damp the weakly-observable yaw; the
        // tightly-seeded gyro integration tracks the true heading between fixes.
        // Magnetometer yaw is absolute heading, so it applies undamped.
        const double yawDamping = sensVal(sensors.gpsYawCorrectionDamping, id, 0.1);
        applyAttitudeError(nav, id,
                            correction[6] + magCorrection[6],
                            correction[7] + magCorrection[7],
                            yawDamping * correction[8] + magCorrection[8]);
        nav.estAccelBiasX[id] += correction[9] + magCorrection[9];
        nav.estAccelBiasY[id] += correction[10] + magCorrection[10];
        nav.estAccelBiasZ[id] += correction[11] + magCorrection[11];
        nav.estGyroBiasX[id] += correction[12] + magCorrection[12];
        nav.estGyroBiasY[id] += correction[13] + magCorrection[13];
        nav.estGyroBiasZ[id] += yawDamping * correction[14] + magCorrection[14];

        // The yaw (and hence the yaw gyro-bias) is only weakly observable from
        // GPS position/velocity, so the bias estimate can otherwise run away and
        // corrupt the attitude integration (observed: estGyroBiasZ -> -0.09
        // rad/s). Bound the IMU bias estimates to physically plausible ranges
        // (legacy literals 0.5 m/s^2 and 0.02 rad/s, now configurable).
        const double kMaxAccelBias = sensVal(sensors.maxAccelBiasEstimate, id, 0.5);
        const double kMaxGyroBias = sensVal(sensors.maxGyroBiasEstimate, id, 0.02);
        // A pinned estimate must not look certain: when the clamp fires,
        // open that variance to the bound so later fixes can pull the state
        // back off the rail instead of trusting a stuck value.
        auto clampBias = [&](double& est, double bound, std::size_t diag) {
            const double c = std::clamp(est, -bound, bound);
            if (c != est) {
                est = c;
                double& var = covariance[covarianceIndex(diag, diag)];
                var = std::max(var, bound * bound);
            }
        };
        clampBias(nav.estAccelBiasX[id], kMaxAccelBias, 9);
        clampBias(nav.estAccelBiasY[id], kMaxAccelBias, 10);
        clampBias(nav.estAccelBiasZ[id], kMaxAccelBias, 11);
        clampBias(nav.estGyroBiasX[id], kMaxGyroBias, 12);
        clampBias(nav.estGyroBiasY[id], kMaxGyroBias, 13);
        clampBias(nav.estGyroBiasZ[id], kMaxGyroBias, 14);
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
            // Initial alignment: state is seeded from truth at launch, plus
            // optional Gaussian alignment errors (all zero = the legacy
            // perfect initialization). Rail/ejection launches use small
            // transfer-alignment errors; mid-flight spawns keep zeros.
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
                if (i < physics.ax.size()) {
                    nav.estAx[i] = physics.ax[i];
                    nav.estAy[i] = physics.ay[i];
                    nav.estAz[i] = physics.az[i];
                }
                std::normal_distribution<double> gauss(0.0, 1.0);
                const double attSig = sensVal(sensors.initialAttitudeErrorDeg, i, 0.0) *
                    3.14159265358979323846 / 180.0;
                if (attSig > 0.0) {
                    applyAttitudeError(nav, i,
                                        attSig * gauss(alignRng),
                                        attSig * gauss(alignRng),
                                        attSig * gauss(alignRng));
                }
                const double posSig = sensVal(sensors.initialPositionErrorM, i, 0.0);
                if (posSig > 0.0) {
                    nav.estPx[i] += posSig * gauss(alignRng);
                    nav.estPy[i] += posSig * gauss(alignRng);
                    nav.estPz[i] += posSig * gauss(alignRng);
                }
                const double velSig = sensVal(sensors.initialVelocityErrorMps, i, 0.0);
                if (velSig > 0.0) {
                    nav.estVx[i] += velSig * gauss(alignRng);
                    nav.estVy[i] += velSig * gauss(alignRng);
                    nav.estVz[i] += velSig * gauss(alignRng);
                }
                // Seed the error covariance from the same sigmas that
                // perturbed the state, so the filter does not start more
                // certain than it is (an over-confident prior makes valid
                // early fixes look like outliers). All-zero sigmas keep the
                // tight legacy default and any caller-supplied covariance
                // (see navigation_aiding_test); a recycled entity slot was
                // already reset by resetEntity.
                if (attSig > 0.0 || posSig > 0.0 || velSig > 0.0) {
                    auto diag = initialCovarianceDiag();
                    if (attSig > 0.0) {
                        diag[6] = diag[7] = diag[8] = attSig * attSig;
                    }
                    if (posSig > 0.0) {
                        diag[0] = diag[1] = diag[2] = posSig * posSig;
                    }
                    if (velSig > 0.0) {
                        diag[3] = diag[4] = diag[5] = velSig * velSig;
                    }
                    initializeCovariance(nav.covarianceFull[i], diag);
                    nav.covarianceDiag[i] = diag;
                }
                nav.prevDThetaX[i] = 0.0; nav.prevDThetaY[i] = 0.0; nav.prevDThetaZ[i] = 0.0;
                nav.prevDVelX[i] = 0.0; nav.prevDVelY[i] = 0.0; nav.prevDVelZ[i] = 0.0;
                nav.isAligned[i] = true;
                continue; // Skip first tick integration
            }

            // High rate strapdown integration
            strapdownINS(i, sensors, nav, dt, environment);

            // Low rate EKF fusion
            if (sensFlag(sensors.gpsUpdated, i) || sensFlag(sensors.baroUpdated, i) || sensFlag(sensors.magUpdated, i)) {
                ekfUpdate(i, sensors, nav, environment);
            }
        }
    }

} // namespace StrikeEngine::Kernel
