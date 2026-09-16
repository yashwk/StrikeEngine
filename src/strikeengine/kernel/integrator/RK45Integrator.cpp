#include <strikeengine/kernel/integrator/RK45Integrator.hpp>
#include <cmath>
#include <algorithm>
#include <array>

namespace StrikeEngine::Kernel
{

namespace
{
    // Dormand-Prince 5(4), the RK5(4)7M pair.
    constexpr double c2 = 1.0 / 5.0;
    constexpr double c3 = 3.0 / 10.0;
    constexpr double c4 = 4.0 / 5.0;
    constexpr double c5 = 8.0 / 9.0;
    constexpr double c6 = 1.0;

    constexpr double a21 = 1.0 / 5.0;

    constexpr double a31 = 3.0 / 40.0;
    constexpr double a32 = 9.0 / 40.0;

    constexpr double a41 = 44.0 / 45.0;
    constexpr double a42 = -56.0 / 15.0;
    constexpr double a43 = 32.0 / 9.0;

    constexpr double a51 = 19372.0 / 6561.0;
    constexpr double a52 = -25360.0 / 2187.0;
    constexpr double a53 = 64448.0 / 6561.0;
    constexpr double a54 = -212.0 / 729.0;

    constexpr double a61 = 9017.0 / 3168.0;
    constexpr double a62 = -355.0 / 33.0;
    constexpr double a63 = 46732.0 / 5247.0;
    constexpr double a64 = 49.0 / 176.0;
    constexpr double a65 = -5103.0 / 18656.0;

    // The seventh stage row doubles as the 5th-order solution weights, which is
    // what makes the method FSAL: evaluating it leaves the derivative at the
    // step's end, reusable as the next step's first stage.
    constexpr double a71 = 35.0 / 384.0;
    constexpr double a73 = 500.0 / 1113.0;
    constexpr double a74 = 125.0 / 192.0;
    constexpr double a75 = -2187.0 / 6784.0;
    constexpr double a76 = 11.0 / 84.0;

    // 5th-order weights (== the seventh stage row).
    constexpr double b1 = a71, b3 = a73, b4 = a74, b5 = a75, b6 = a76;

    // Embedded error estimate: 5th-order weights minus the 4th-order weights.
    constexpr double e1 = 71.0 / 57600.0;
    constexpr double e3 = -71.0 / 16695.0;
    constexpr double e4 = 71.0 / 1920.0;
    constexpr double e5 = -17253.0 / 339200.0;
    constexpr double e6 = 22.0 / 525.0;
    constexpr double e7 = -1.0 / 40.0;

    // Step-size controller. 0.9 keeps a margin below the tolerance; the accepted
    // solution is 5th order so the error falls as h^5, hence the 1/5 exponent.
    constexpr double kSafety = 0.9;
    constexpr double kMinShrink = 0.2;
    constexpr double kMaxGrow = 5.0;
    constexpr double kMinStep = 1e-6;

    using Field = std::vector<double> PhysicsBlock::*;
}

RK45Integrator::RK45Integrator(double tol)
    : tolerance(tol)
{
}

void RK45Integrator::ensureCapacity(const PhysicsBlock& state)
{
    // Copy assignment recycles the buffers' capacity once the entity count is
    // stable, so steady-state stepping performs no heap allocation.
    k1 = state; k2 = state; k3 = state; k4 = state;
    k5 = state; k6 = state; k7 = state;
    stage = state; acc5 = state; accErr = state;
}

double RK45Integrator::integrate(
    PhysicsBlock& state,
    const DerivativeFn& deriv,
    double t0,
    double dt)
{
    const std::size_t n = state.size;
    ensureCapacity(state);

    acceptedStepCount = 0;
    rejectedStepCount = 0;

    // acc5 = sum_r weight_r * k_r, applied to one integrated state field. The
    // kernels are the seven stage derivatives; inactive entities keep their
    // state (they are skipped by applyStateUpdate as well).
    auto combine = [&](PhysicsBlock& acc, Field f,
                       const std::array<const PhysicsBlock*, 7>& ks,
                       const std::array<double, 7>& ws) {
        auto& out = acc.*f;
        // Hand-built blocks may omit optional state groups (gimbal angles).
        const auto& first = (*ks[0]).*f;
        if (out.size() < n || first.size() < n) return;
        for (std::size_t i = 0; i < n; ++i) {
            if (!state.active[i]) continue;
            double s = 0.0;
            for (std::size_t r = 0; r < 7; ++r) s += ws[r] * ((*ks[r]).*f)[i];
            out[i] = s;
        }
    };

    // Builds one stage state from the a-coefficients of its row: stage = x +
    // h * sum_j a_ij k_j. Only k_j with a non-zero coefficient are passed.
    auto buildStage = [&](double h,
                          std::initializer_list<std::pair<const PhysicsBlock*, double>> terms,
                          PhysicsBlock& out) {
        out = state;
        for (const auto& [k, a] : terms) {
            if (a != 0.0) applyStateUpdate(out, *k, h * a);
        }
    };

    const std::array<double, 7> w5 = {b1, 0.0, b3, b4, b5, b6, 0.0};
    const std::array<double, 7> we = {e1, 0.0, e3, e4, e5, e6, e7};

    double t = t0;
    double remaining = dt;
    double h = dt;
    bool haveK1 = false;

    while (remaining > 1e-12)
    {
        if (h > remaining) h = remaining;

        bool accepted = false;
        double errMax = 0.0;

        // Rejection must not spin forever: the step is floored at kMinStep, and
        // a step that cannot meet the tolerance there is accepted with its error
        // reported rather than silently retried forever.
        for (int attempts = 0; attempts < 200 && !accepted; ++attempts)
        {
            if (!haveK1) {
                deriv(state, t, k1);
                haveK1 = true;
            }

            buildStage(h, {{&k1, a21}}, stage);
            deriv(stage, t + c2 * h, k2);

            buildStage(h, {{&k1, a31}, {&k2, a32}}, stage);
            deriv(stage, t + c3 * h, k3);

            buildStage(h,
                       {{&k1, a41}, {&k2, a42}, {&k3, a43}}, stage);
            deriv(stage, t + c4 * h, k4);

            buildStage(h,
                       {{&k1, a51}, {&k2, a52}, {&k3, a53}, {&k4, a54}}, stage);
            deriv(stage, t + c5 * h, k5);

            buildStage(h,
                       {{&k1, a61}, {&k2, a62}, {&k3, a63}, {&k4, a64}, {&k5, a65}}, stage);
            deriv(stage, t + c6 * h, k6);

            // Seventh stage == the 5th-order solution, so it is also the state
            // the step should land on.
            const std::array<const PhysicsBlock*, 7> ks =
                {&k1, &k2, &k3, &k4, &k5, &k6, &k7};
            buildStage(h,
                       {{&k1, a71}, {&k3, a73}, {&k4, a74}, {&k5, a75}, {&k6, a76}}, stage);
            deriv(stage, t + h, k7);

            // acc5 (the increment that produces `stage`) and accErr.
            combine(acc5, &PhysicsBlock::px, ks, w5);
            combine(acc5, &PhysicsBlock::py, ks, w5);
            combine(acc5, &PhysicsBlock::pz, ks, w5);
            combine(acc5, &PhysicsBlock::vx, ks, w5);
            combine(acc5, &PhysicsBlock::vy, ks, w5);
            combine(acc5, &PhysicsBlock::vz, ks, w5);
            combine(acc5, &PhysicsBlock::wx, ks, w5);
            combine(acc5, &PhysicsBlock::wy, ks, w5);
            combine(acc5, &PhysicsBlock::wz, ks, w5);
            combine(acc5, &PhysicsBlock::qw, ks, w5);
            combine(acc5, &PhysicsBlock::qx, ks, w5);
            combine(acc5, &PhysicsBlock::qy, ks, w5);
            combine(acc5, &PhysicsBlock::qz, ks, w5);
            combine(acc5, &PhysicsBlock::mass, ks, w5);
            combine(acc5, &PhysicsBlock::finPitch, ks, w5);
            combine(acc5, &PhysicsBlock::finYaw, ks, w5);
            combine(acc5, &PhysicsBlock::finRoll, ks, w5);
            combine(acc5, &PhysicsBlock::gimbalPitch, ks, w5);
            combine(acc5, &PhysicsBlock::gimbalYaw, ks, w5);

            combine(accErr, &PhysicsBlock::px, ks, we);
            combine(accErr, &PhysicsBlock::py, ks, we);
            combine(accErr, &PhysicsBlock::pz, ks, we);
            combine(accErr, &PhysicsBlock::vx, ks, we);
            combine(accErr, &PhysicsBlock::vy, ks, we);
            combine(accErr, &PhysicsBlock::vz, ks, we);
            combine(accErr, &PhysicsBlock::wx, ks, we);
            combine(accErr, &PhysicsBlock::wy, ks, we);
            combine(accErr, &PhysicsBlock::wz, ks, we);
            combine(accErr, &PhysicsBlock::qw, ks, we);
            combine(accErr, &PhysicsBlock::qx, ks, we);
            combine(accErr, &PhysicsBlock::qy, ks, we);
            combine(accErr, &PhysicsBlock::qz, ks, we);
            combine(accErr, &PhysicsBlock::mass, ks, we);
            combine(accErr, &PhysicsBlock::finPitch, ks, we);
            combine(accErr, &PhysicsBlock::finYaw, ks, we);
            combine(accErr, &PhysicsBlock::finRoll, ks, we);
            combine(accErr, &PhysicsBlock::gimbalPitch, ks, we);
            combine(accErr, &PhysicsBlock::gimbalYaw, ks, we);

            // Error norm: each component's estimated error relative to its own
            // state scale, so the controller cannot accept a step that is
            // accurate in translation but poor in attitude or actuator dynamics.
            errMax = 0.0;
            auto errRelative = [&](Field f) {
                const auto& a = accErr.*f;
                const auto& sc = state.*f;
                if (a.size() < n || sc.size() < n) return;
                for (std::size_t i = 0; i < n; ++i) {
                    if (!state.active[i]) continue;
                    errMax = std::max(errMax, std::abs(h * a[i]) / (1.0 + std::abs(sc[i])));
                }
            };
            // Actuator states are bounded (|deflection| <= ~0.43 rad), so a
            // constant scale is the meaningful one: their own magnitude is not
            // a measure of the error being small.
            auto errActuator = [&](Field f) {
                const auto& a = accErr.*f;
                if (a.size() < n) return;
                for (std::size_t i = 0; i < n; ++i) {
                    if (!state.active[i]) continue;
                    errMax = std::max(errMax, std::abs(h * a[i]) / (1.0 + 0.43));
                }
            };
            errRelative(&PhysicsBlock::px);
            errRelative(&PhysicsBlock::py);
            errRelative(&PhysicsBlock::pz);
            errRelative(&PhysicsBlock::vx);
            errRelative(&PhysicsBlock::vy);
            errRelative(&PhysicsBlock::vz);
            errRelative(&PhysicsBlock::wx);
            errRelative(&PhysicsBlock::wy);
            errRelative(&PhysicsBlock::wz);
            errRelative(&PhysicsBlock::qw);
            errRelative(&PhysicsBlock::qx);
            errRelative(&PhysicsBlock::qy);
            errRelative(&PhysicsBlock::qz);
            errRelative(&PhysicsBlock::mass);
            errActuator(&PhysicsBlock::finPitch);
            errActuator(&PhysicsBlock::finYaw);
            errActuator(&PhysicsBlock::finRoll);
            errActuator(&PhysicsBlock::gimbalPitch);
            errActuator(&PhysicsBlock::gimbalYaw);

            if (errMax <= tolerance || h <= kMinStep) {
                accepted = true;
                ++acceptedStepCount;
            } else {
                // Shrink from the error estimate where it is informative; the
                // estimator can under-predict, so never shrink by less than
                // kMinShrink.
                const double factor = std::clamp(
                    kSafety * std::pow(tolerance / errMax, 0.2), kMinShrink, 1.0);
                h = std::max(kMinStep, h * factor);
                ++rejectedStepCount;
                // k1 is f(state) at the unchanged t, so it stays valid across
                // rejections; only the coefficients depend on h.
            }
        }

        // Accept the 5th-order solution. `stage` already holds exactly this
        // state (it was built from the same a7/b weights), so reuse it rather
        // than re-deriving, and keep k7 as the next step's first stage (FSAL).
        state = stage;
        t += h;
        remaining -= h;
        k1 = k7;
        haveK1 = true;

        // Grow for the next step, bounded so a smooth region cannot balloon and
        // a noisy one cannot collapse in a single jump.
        if (errMax > 0.0) {
            const double factor = std::clamp(
                kSafety * std::pow(tolerance / errMax, 0.2), kMinShrink, kMaxGrow);
            h = std::max(kMinStep, h * factor);
        }
        if (h > remaining) h = remaining;
        if (remaining <= 1e-12) break;
    }

    return dt;
}

} // namespace StrikeEngine::Kernel
