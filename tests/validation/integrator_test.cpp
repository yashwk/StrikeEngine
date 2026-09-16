#include <strikeengine/kernel/integrator/RK4Integrator.hpp>
#include <strikeengine/kernel/integrator/RK45Integrator.hpp>
#include <strikeengine/kernel/systems/EventSystem.hpp>
#include <cmath>
#include <cstdio>

using namespace StrikeEngine::Kernel;

namespace {

PhysicsBlock makeState()
{
    PhysicsBlock state;
    state.size = 1;
    state.px = {1.0};
    state.py = {0.0}; state.pz = {10.0};
    state.vx = {0.0}; state.vy = {0.0}; state.vz = {0.0};
    state.ax = {0.0}; state.ay = {0.0}; state.az = {0.0};
    state.qw = {1.0}; state.qx = {0.0}; state.qy = {0.0}; state.qz = {0.0};
    state.wx = {0.0}; state.wy = {0.0}; state.wz = {0.0};
    state.mass = {1.0}; state.massDry = {1.0};
    state.finPitch = {0.0}; state.finYaw = {0.0}; state.finRoll = {0.0};
    state.active = {true};
    return state;
}

void exponentialDerivative(const PhysicsBlock& state, double, PhysicsBlock& d)
{
    d.py[0] = 0.0; d.pz[0] = 0.0;
    d.vx[0] = 0.0; d.vy[0] = 0.0; d.vz[0] = 0.0;
    d.qw[0] = 0.0; d.qx[0] = 0.0; d.qy[0] = 0.0; d.qz[0] = 0.0;
    d.wx[0] = 0.0; d.wy[0] = 0.0; d.wz[0] = 0.0;
    d.mass[0] = 0.0;
    d.finPitch[0] = 0.0; d.finYaw[0] = 0.0; d.finRoll[0] = 0.0;
    d.px[0] = state.px[0];
}

// Pure rotation about body Y at a constant rate: after time h the exact
// attitude is a rotation of |w| * h about that axis. Mirrors what the physics
// backend supplies: q_dot = 0.5 * q (x) (0, w), w_dot = 0.
double gRate = 1.0;
void constantRateDerivative(const PhysicsBlock& state, double, PhysicsBlock& d)
{
    d.px[0] = 0.0; d.py[0] = 0.0; d.pz[0] = 0.0;
    d.vx[0] = 0.0; d.vy[0] = 0.0; d.vz[0] = 0.0;
    d.wx[0] = 0.0; d.wy[0] = 0.0; d.wz[0] = 0.0;
    // w = (0, gRate, 0)  =>  0.5 * q (x) (0, w)
    d.qw[0] = -0.5 * state.qy[0] * gRate;
    d.qx[0] = -0.5 * state.qz[0] * gRate;
    d.qy[0] =  0.5 * state.qw[0] * gRate;
    d.qz[0] =  0.5 * state.qx[0] * gRate;
    d.mass[0] = 0.0;
    d.finPitch[0] = 0.0; d.finYaw[0] = 0.0; d.finRoll[0] = 0.0;
}

}

int main()
{
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    auto rk4State = makeState();
    RK4Integrator rk4;
    rk4.integrate(rk4State, exponentialDerivative, 0.0, 1.0);

    auto rk45State = makeState();
    RK45Integrator rk45(1e-10);
    rk45.integrate(rk45State, exponentialDerivative, 0.0, 1.0);

    const double expected = std::exp(1.0);
    check(std::abs(rk4State.px[0] - expected) < 2e-2,
          "RK4 stage re-evaluation reaches fourth-order exponential solution");
    check(std::abs(rk45State.px[0] - expected) < 1e-8,
          "RK45 accepted solution meets requested tolerance");
    check(rk45.rejectedSteps() > 0 && rk45.acceptedSteps() > 1,
          "RK45 rejects an oversized step and adapts its substeps");

    // Pure rotation must be integrated by the exponential map, which is exact
    // for a constant body rate: after 1 s at 1 rad/s about Y the attitude is a
    // 1 rad rotation. The linear update (q += 0.5*h*q(x)(0,w), then renormalize)
    // lands ~1e-4 away instead, so this pins the change.
    {
        auto s = makeState();
        s.wy[0] = gRate;
        RK4Integrator rk;
        rk.integrate(s, constantRateDerivative, 0.0, 1.0);
        const double expected = std::sin(0.5);
        const double expectedW = std::cos(0.5);
        std::printf("  [info] rotation: q=(%.6f,%.6f,%.6f,%.6f)\n",
                    s.qw[0], s.qx[0], s.qy[0], s.qz[0]);
        check(std::abs(s.qy[0] - expected) < 1e-12 &&
                  std::abs(s.qw[0] - expectedW) < 1e-12 &&
                  std::abs(s.qx[0]) < 1e-12 && std::abs(s.qz[0]) < 1e-12,
              "exponential attitude update is exact for a constant body rate");
        const double norm = std::sqrt(s.qw[0]*s.qw[0] + s.qx[0]*s.qx[0] +
                                      s.qy[0]*s.qy[0] + s.qz[0]*s.qz[0]);
        check(std::abs(norm - 1.0) < 1e-14, "attitude stays unit-norm");

        // Many small steps must not drift: the map composes exactly.
        auto many = makeState();
        many.wy[0] = gRate;
        for (int k = 0; k < 1000; ++k) {
            rk.integrate(many, constantRateDerivative, k * 1e-3, 1e-3);
        }
        const double normMany = std::sqrt(many.qw[0]*many.qw[0] + many.qx[0]*many.qx[0] +
                                          many.qy[0]*many.qy[0] + many.qz[0]*many.qz[0]);
        check(std::abs(normMany - 1.0) < 1e-12 &&
                  std::abs(many.qy[0] - expected) < 1e-9,
              "1000 composed rotation steps keep the exact attitude and unit norm");
    }

    // Observed order of convergence. Integrating dx/dt = x to T with n equal
    // steps and halving h must reduce the error by ~2^p. This pins the scheme
    //'s order rather than just its accuracy at one step size, so a change that
    // silently drops a stage's contribution is caught.
    {
        auto errorAt = [&](int steps, double& error) {
            auto s = makeState();
            RK4Integrator rk;
            const double T = 1.0;
            const double h = T / static_cast<double>(steps);
            for (int k = 0; k < steps; ++k) {
                rk.integrate(s, exponentialDerivative, k * h, h);
            }
            error = std::abs(s.px[0] - std::exp(T));
        };
        double e1 = 0.0, e2 = 0.0, e3 = 0.0;
        errorAt(16, e1);
        errorAt(32, e2);
        errorAt(64, e3);
        const double order1 = (e1 > 0.0 && e2 > 0.0) ? std::log2(e1 / e2) : 0.0;
        const double order2 = (e2 > 0.0 && e3 > 0.0) ? std::log2(e2 / e3) : 0.0;
        std::printf("  [info] RK4 observed order: %.2f then %.2f\n", order1, order2);
        check(order1 > 3.5 && order2 > 3.5,
              "RK4 converges at fourth order under step halving");
    }

    // Tolerances represent an error bound, so tightening the tolerance must
    // reduce the achieved error. An error controller that ignores the tolerance
    // (or saturates) would leave these equal.
    {
        auto errorAtTolerance = [&](double tol, double& error, std::size_t& accepted) {
            auto s = makeState();
            RK45Integrator rk(tol);
            rk.integrate(s, exponentialDerivative, 0.0, 1.0);
            error = std::abs(s.px[0] - std::exp(1.0));
            accepted = rk.acceptedSteps();
        };
        double eLoose = 0.0, eTight = 0.0;
        std::size_t nLoose = 0, nTight = 0;
        errorAtTolerance(1e-4, eLoose, nLoose);
        errorAtTolerance(1e-9, eTight, nTight);
        std::printf("  [info] RK45 tolerance 1e-4 -> err %.2e (%zu steps), "
                    "1e-9 -> err %.2e (%zu steps)\n",
                    eLoose, nLoose, eTight, nTight);
        check(eTight < eLoose,
              "RK45 achieved error falls when the tolerance is tightened");
        // The controller bounds the LOCAL error per substep, so global error
        // accumulates over the substeps and may slightly exceed the tolerance.
        // Assert the honest contract: within an order of magnitude of it.
        check(eTight < 10.0 * 1e-9,
              "RK45 global error stays within an order of magnitude of the tolerance");
    }

    PhysicsBlock impact = makeState();
    impact.pz[0] = -5.0;
    EntityStatusBlock status;
    status.size = 1;
    status.isAlive = {true};
    EventSystem events;
    double timestamp = -1.0;
    events.subscribe([&](const SimulationEvent& event) { timestamp = event.timestamp; });
    events.evaluate(impact, status, 1.0, 0.25, {10.0});
    events.processQueue();

    check(std::abs(timestamp - (1.0 - 0.25 + 0.25 * 10.0 / 15.0)) < 1e-12,
          "ground impact timestamp is interpolated within the step");
    check(impact.pz[0] == 0.0 && !impact.active[0] && !status.isAlive[0],
          "interpolated impact clamps and deactivates the entity");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
