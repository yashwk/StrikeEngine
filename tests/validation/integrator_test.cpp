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
