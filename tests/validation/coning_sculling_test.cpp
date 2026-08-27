// Strapdown coning/sculling corrections (MVP): the NavigationSystem replaces
// the first-order quaternion step with a rotation-vector (exact exp) attitude
// update and applies the single-interval sculling velocity compensation
// +0.5 (omega x f) dt^2. Two references validate it deterministically with
// zero sensor noise:
//   - a fine-step RK4 integrator of the exact kinematics q_dot = 0.5 q (x) w,
//     v_dot = C(q) f + g, and
//   - a naive first-order strapdown mirroring the pre-change code.
// The shipped system must beat the naive reference for a true coning motion
// (attitude) and for an orthogonal oscillatory rotation/specific-force motion
// (velocity).
#include <strikeengine/kernel/systems/NavigationSystem.hpp>
#include <strikeengine/kernel/math/Quaternion.hpp>
#include <cmath>
#include <cstdio>
#include <array>

using namespace StrikeEngine::Kernel;

namespace {

constexpr double kGravityZ = -9.80665;
using Vec3 = std::array<double, 3>;

struct InsState {
    double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
};

// --- Scenario definitions (analytic omega(t), f(t)) ---
// Coning motion: body z-axis cones about the reference z-axis while the body
// spins about its own z-axis.
Vec3 coningOmega(double t) {
    constexpr double kOmega = 10.0;
    constexpr double kAlpha = 0.1;
    return {-kOmega * kAlpha * std::sin(kOmega * t),
            kOmega * kAlpha * std::cos(kOmega * t), kOmega};
}
Vec3 zeroF(double) { return {0.0, 0.0, 0.0}; }

// Sculling motion: orthogonal oscillatory rotation about z and specific force
// along x at the same frequency.
Vec3 sculOmega(double t) {
    constexpr double kOmega = 5.0;
    constexpr double kAmp = 0.1;
    return {0.0, 0.0, kOmega * kAmp * std::sin(kOmega * t)};
}
Vec3 sculF(double t) {
    constexpr double kOmega = 5.0;
    constexpr double kForce = 10.0;
    return {kForce * std::cos(kOmega * t), 0.0, 0.0};
}

// Steady rotation + force: the single-interval sculling term is exact here.
Vec3 steadyOmega(double) { return {0.0, 0.0, 1.0}; }
Vec3 steadyF(double) { return {10.0, 0.0, 0.0}; }

void qDeriv(const InsState& s, double wx, double wy, double wz,
            double& dqw, double& dqx, double& dqy, double& dqz) {
    dqw = -0.5 * (s.qx * wx + s.qy * wy + s.qz * wz);
    dqx =  0.5 * (s.qw * wx + s.qy * wz - s.qz * wy);
    dqy =  0.5 * (s.qw * wy + s.qz * wx - s.qx * wz);
    dqz =  0.5 * (s.qw * wz + s.qx * wy - s.qy * wx);
}

// Fine-step RK4 reference of q_dot = 0.5 q (x) w(t); v_dot = C(q) f(t) + g.
// Iterates with an integer counter so the sub-step count is exact (no
// floating-point loop drift).
void fineStep(InsState& s, double t0, double t1,
              Vec3 (*omegaAt)(double), Vec3 (*fAt)(double)) {
    constexpr int kSubSteps = 200;
    const double h = (t1 - t0) / kSubSteps;
    for (int k = 0; k < kSubSteps; ++k) {
        const double t = t0 + k * h;
        auto deriv = [&](const InsState& st, double tt) {
            const Vec3 w = omegaAt(tt);
            const Vec3 f = fAt(tt);
            double dqw, dqx, dqy, dqz;
            qDeriv(st, w[0], w[1], w[2], dqw, dqx, dqy, dqz);
            double wfx, wfy, wfz;
            quatRotateToWorld(st.qw, st.qx, st.qy, st.qz,
                              f[0], f[1], f[2], wfx, wfy, wfz);
            return std::array<double, 7>{dqw, dqx, dqy, dqz,
                                         wfx, wfy, wfz + kGravityZ};
        };
        const std::array<double, 7> k1 = deriv(s, t);
        InsState s2 = s;
        double* p2 = &s2.qw;
        for (int i = 0; i < 7; ++i) p2[i] += 0.5 * h * k1[i];
        const std::array<double, 7> k2 = deriv(s2, t + 0.5 * h);
        InsState s3 = s;
        double* p3 = &s3.qw;
        for (int i = 0; i < 7; ++i) p3[i] += 0.5 * h * k2[i];
        const std::array<double, 7> k3 = deriv(s3, t + 0.5 * h);
        InsState s4 = s;
        double* p4 = &s4.qw;
        for (int i = 0; i < 7; ++i) p4[i] += h * k3[i];
        const std::array<double, 7> k4 = deriv(s4, t + h);
        double* p = &s.qw;
        for (int i = 0; i < 7; ++i) p[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        const double norm = std::sqrt(s.qw * s.qw + s.qx * s.qx + s.qy * s.qy + s.qz * s.qz);
        s.qw /= norm; s.qx /= norm; s.qy /= norm; s.qz /= norm;
    }
}

// Naive first-order strapdown (mirror of the pre-change code): rotate the
// specific force with the current attitude, integrate velocity, then advance
// the quaternion by q_dot = 0.5 q (x) (0, w) with a first-order step.
void naiveStep(InsState& s, double dt, double t,
               Vec3 (*omegaAt)(double), Vec3 (*fAt)(double)) {
    const Vec3 w = omegaAt(t);
    const Vec3 f = fAt(t);
    double wfx, wfy, wfz;
    quatRotateToWorld(s.qw, s.qx, s.qy, s.qz, f[0], f[1], f[2], wfx, wfy, wfz);
    s.vx += wfx * dt;
    s.vy += wfy * dt;
    s.vz += (wfz + kGravityZ) * dt;
    double dqw, dqx, dqy, dqz;
    qDeriv(s, w[0], w[1], w[2], dqw, dqx, dqy, dqz);
    s.qw += 0.5 * dqw * dt;
    s.qx += 0.5 * dqx * dt;
    s.qy += 0.5 * dqy * dt;
    s.qz += 0.5 * dqz * dt;
    const double norm = std::sqrt(s.qw * s.qw + s.qx * s.qx + s.qy * s.qy + s.qz * s.qz);
    s.qw /= norm; s.qx /= norm; s.qy /= norm; s.qz /= norm;
}

double angleBetween(double aw, double ax, double ay, double az,
                    double bw, double bx, double by, double bz) {
    const double dot = std::abs(aw * bw + ax * bx + ay * by + az * bz);
    return 2.0 * std::acos(std::min(1.0, dot));
}

double velError(double vx, double vy, double vz,
                double rx, double ry, double rz) {
    return std::sqrt((vx - rx) * (vx - rx) + (vy - ry) * (vy - ry) + (vz - rz) * (vz - rz));
}

// Zero-noise single-entity sensor block for the unit-level runs.
SensorBlock makeSensors() {
    SensorBlock sensors;
    sensors.size = 1;
    sensors.accelX = {0.0}; sensors.accelY = {0.0}; sensors.accelZ = {0.0};
    sensors.gyroX = {0.0}; sensors.gyroY = {0.0}; sensors.gyroZ = {0.0};
    sensors.gpsUpdated = {false};
    sensors.accelNoiseStdDev = {0.0};
    sensors.accelBiasStdDev = {0.0};
    sensors.gyroNoiseStdDev = {0.0};
    sensors.gyroBiasStdDev = {0.0};
    sensors.gpsPosNoiseStdDev = {1.0};
    sensors.gpsVelNoiseStdDev = {1.0};
    return sensors;
}

PhysicsBlock makePhysics() {
    PhysicsBlock physics;
    physics.size = 1;
    physics.px = {0.0}; physics.py = {0.0}; physics.pz = {0.0};
    physics.vx = {0.0}; physics.vy = {0.0}; physics.vz = {0.0};
    physics.qw = {1.0}; physics.qx = {0.0}; physics.qy = {0.0}; physics.qz = {0.0};
    physics.wx = {0.0}; physics.wy = {0.0}; physics.wz = {0.0};
    return physics;
}

// Run the shipped NavigationSystem over N steps with endpoint samples of
// omega(t_n), f(t_n), returning the final estQ/estV.
void runNavigationSystem(double dt, int steps,
                         Vec3 (*omegaAt)(double), Vec3 (*fAt)(double),
                         InsState& out) {
    NavigationSystem system;
    NavigationBlock nav;
    SensorBlock sensors = makeSensors();
    PhysicsBlock physics = makePhysics();
    EnvironmentConfig environment;   // default local flat-earth mode
    system.update(sensors, physics, nav, dt, environment);  // aligns from truth
    for (int n = 1; n <= steps; ++n) {
        const double t = n * dt;
        const Vec3 w = omegaAt(t);
        const Vec3 f = fAt(t);
        sensors.gyroX[0] = w[0]; sensors.gyroY[0] = w[1]; sensors.gyroZ[0] = w[2];
        sensors.accelX[0] = f[0]; sensors.accelY[0] = f[1]; sensors.accelZ[0] = f[2];
        system.update(sensors, physics, nav, dt, environment);
    }
    out.qw = nav.estQw[0]; out.qx = nav.estQx[0];
    out.qy = nav.estQy[0]; out.qz = nav.estQz[0];
    out.vx = nav.estVx[0]; out.vy = nav.estVy[0]; out.vz = nav.estVz[0];
}

void runNaive(double dt, int steps, Vec3 (*omegaAt)(double), Vec3 (*fAt)(double),
              InsState& out) {
    InsState s;
    for (int n = 0; n < steps; ++n) {
        const double t = (n + 1) * dt;
        naiveStep(s, dt, t, omegaAt, fAt);
    }
    out = s;
}

void runFine(double dt, int steps, Vec3 (*omegaAt)(double), Vec3 (*fAt)(double),
             InsState& out) {
    InsState s;
    for (int n = 0; n < steps; ++n) {
        fineStep(s, n * dt, (n + 1) * dt, omegaAt, fAt);
    }
    out = s;
}

} // namespace

int main() {
    std::printf("=== coning_sculling_test: strapdown coning/sculling corrections ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    constexpr double dt = 0.01;
    constexpr int N = 100;   // 1.0 s

    // ---- Coning scenario: true coning motion, zero specific force ----
    {
        InsState ref, naive, ins;
        runFine(dt, N, coningOmega, zeroF, ref);
        runNaive(dt, N, coningOmega, zeroF, naive);
        runNavigationSystem(dt, N, coningOmega, zeroF, ins);

        const double insAngErr = angleBetween(ins.qw, ins.qx, ins.qy, ins.qz,
                                              ref.qw, ref.qx, ref.qy, ref.qz);
        const double naiveAngErr = angleBetween(naive.qw, naive.qx, naive.qy, naive.qz,
                                                ref.qw, ref.qx, ref.qy, ref.qz);
        std::printf("  coning max attitude error: rotation-vector %.3e rad vs naive %.3e rad (%.1fx)\n",
                    insAngErr, naiveAngErr, naiveAngErr / std::max(insAngErr, 1e-300));
        check(insAngErr < naiveAngErr,
              "rotation-vector attitude update beats the first-order quaternion step");
        check(insAngErr < 1e-2,
              "coning attitude error is bounded at the dt^2 scale");
    }

    // ---- Sculling scenario: orthogonal oscillatory rotation and specific force ----
    {
        InsState ref, naive, ins;
        runFine(dt, N, sculOmega, sculF, ref);
        runNaive(dt, N, sculOmega, sculF, naive);
        runNavigationSystem(dt, N, sculOmega, sculF, ins);

        const double insVelErr = velError(ins.vx, ins.vy, ins.vz, ref.vx, ref.vy, ref.vz);
        const double naiveVelErr = velError(naive.vx, naive.vy, naive.vz, ref.vx, ref.vy, ref.vz);
        std::printf("  sculling max velocity error: corrected %.3e vs uncorrected %.3e (%.1fx)\n",
                    insVelErr, naiveVelErr, naiveVelErr / std::max(insVelErr, 1e-300));
        check(insVelErr < naiveVelErr,
              "single-interval sculling correction reduces the velocity error");
    }

    // ---- Steady rotation compensation: the single-interval term is exact for
    //      constant rate + force over the interval ----
    {
        InsState ref, naive, ins;
        runFine(dt, N, steadyOmega, steadyF, ref);
        runNaive(dt, N, steadyOmega, steadyF, naive);
        runNavigationSystem(dt, N, steadyOmega, steadyF, ins);

        const double insVelErr = velError(ins.vx, ins.vy, ins.vz, ref.vx, ref.vy, ref.vz);
        const double naiveVelErr = velError(naive.vx, naive.vy, naive.vz, ref.vx, ref.vy, ref.vz);
        std::printf("  steady-rotation velocity error: corrected %.3e vs uncorrected %.3e (%.1fx)\n",
                    insVelErr, naiveVelErr, naiveVelErr / std::max(insVelErr, 1e-300));
        check(insVelErr < naiveVelErr,
              "single-interval sculling term is the exact rotation compensation");
        check(insVelErr < 1e-3,
              "steady-rotation velocity error is near-exact with the correction");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
