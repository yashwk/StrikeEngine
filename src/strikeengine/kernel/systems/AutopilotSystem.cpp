#include <strikeengine/kernel/systems/AutopilotSystem.hpp>
#include <strikeengine/kernel/math/Quaternion.hpp>
#include <strikeengine/models/physics/atmosphere/ISA1976.hpp>
#include <cmath>
#include <algorithm>

namespace StrikeEngine::Kernel {

    AutopilotSystem::AutopilotSystem() = default;

    void AutopilotSystem::update(
        const EntityStatusBlock& status,
        const NavigationBlock& nav,
        const SensorBlock& /*sensor*/,
        const GuidanceBlock& guidance,
        ControlBlock& control,
        double /*dt*/)
    {
        // Resize control block if needed (usually handled in SimulationKernel, but safe to check)
        if (control.pitchCommand.size() < nav.size) {
            control.pitchCommand.resize(nav.size, 0.0);
            control.yawCommand.resize(nav.size, 0.0);
            control.rollCommand.resize(nav.size, 0.0);
            control.thrustCommand.resize(nav.size, 0.0);
            control.pitchSaturated.assign(nav.size, false);
            control.yawSaturated.assign(nav.size, false);
            control.rollSaturated.assign(nav.size, false);
        }
        if (control.gainSchedulingEnabled.size() < nav.size) {
            control.gainSchedulingEnabled.resize(nav.size, false);
            control.refDynamicPressurePa.resize(nav.size, 50000.0);
            control.minDynamicPressurePa.resize(nav.size, 2000.0);
            control.maxDynamicPressurePa.resize(nav.size, 300000.0);
        }

        for (std::size_t i = 0; i < nav.size; ++i) {
            if (!status.isAlive[i]) continue;

            if (guidance.mode[i] == GuidanceMode::None) {
                control.pitchCommand[i] = 0.0;
                control.yawCommand[i] = 0.0;
                control.rollCommand[i] = 0.0;
                continue;
            }

            updateFlightController(i, nav, guidance, control);
        }
    }

    void AutopilotSystem::updateFlightController(
        std::size_t id,
        const NavigationBlock& nav,
        const GuidanceBlock& guidance,
        ControlBlock& control)
    {
        // 1. Commanded acceleration (world, from guidance) -> body frame
        double axCmdB, ayCmdB, azCmdB;
        quatRotateToBody(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                         guidance.commandedAccelX[id],
                         guidance.commandedAccelY[id],
                         guidance.commandedAccelZ[id],
                         axCmdB, ayCmdB, azCmdB);

        // Guidance commands are total world-frame accelerations, while the
        // accelerometer measures specific force (total acceleration minus
        // gravity). Convert the command into the body-frame specific-force
        // demand before mapping it to the fins. The normal-force term is
        // intentionally feed-forward: feeding the fin's own measured force
        // back into this simplified airframe model creates a short-period
        // limit cycle. Rates and AoA below provide the stabilizing feedback.
        double gravityBx, gravityBy, gravityBz;
        quatRotateToBody(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                         0.0, 0.0, -9.80665, gravityBx, gravityBy, gravityBz);
        const double aySpecificCmd = ayCmdB - gravityBy;
        const double azSpecificCmd = azCmdB - gravityBz;

        // 3. Direct acceleration-command controller. The feed-forward term
        //    supplies the requested normal force; body-rate and AoA terms
        //    damp the short-period response before the fins saturate.
        // 4. AoA / sideslip estimates from estimated body-frame velocity
        double ub, vb, wb;
        quatRotateToBody(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                         nav.estVx[id], nav.estVy[id], nav.estVz[id],
                         ub, vb, wb);
        const double alpha = std::atan2(wb, ub);
        const double beta  = std::atan2(vb, ub);

        // Dynamic pressure (q) gain scheduling: scales feed-forward fin command
        // by sqrt(q_ref / q) to prevent max-Q control flutter and high-altitude sluggishness.
        double sqScale = 1.0;
        if (id < control.gainSchedulingEnabled.size() && control.gainSchedulingEnabled[id]) {
            const double vx = nav.estVx[id];
            const double vy = nav.estVy[id];
            const double vz = nav.estVz[id];
            const double speedSq = vx * vx + vy * vy + vz * vz;
            const double alt = std::max(0.0, nav.estPz[id]);

            static const Models::ISA1976 isa;
            const auto atmos = isa.evaluate(alt);
            const double qEst = 0.5 * atmos.density * speedSq;

            const double qRef = (id < control.refDynamicPressurePa.size() && control.refDynamicPressurePa[id] > 0.0)
                ? control.refDynamicPressurePa[id] : 50000.0;
            const double qMin = (id < control.minDynamicPressurePa.size() && control.minDynamicPressurePa[id] > 0.0)
                ? control.minDynamicPressurePa[id] : 2000.0;
            const double qMax = (id < control.maxDynamicPressurePa.size() && control.maxDynamicPressurePa[id] >= qMin)
                ? control.maxDynamicPressurePa[id] : 300000.0;

            const double qClamped = std::clamp(qEst, qMin, qMax);
            sqScale = std::clamp(std::sqrt(qRef / qClamped), 0.2, 5.0);
        }

        // Positive body-Z specific force is down and requires nose-down
        // (negative wy / fin); positive body-Y force requires nose-right
        // (positive wz / fin). The signs below match AeroModel's documented
        // X-forward/Y-right/Z-down convention.
        const double effectiveKAccel = control.kAccelP[id] * sqScale;
        const double pitchFeedForward = std::clamp(
            -effectiveKAccel * azSpecificCmd, -0.35, 0.35);
        const double yawFeedForward = std::clamp(
            effectiveKAccel * aySpecificCmd, -0.35, 0.35);
        const double pitchRateDamping = std::clamp(
            -control.kRateP[id] * nav.estWy[id], -0.20, 0.20);
        const double yawRateDamping = std::clamp(
            -control.kRateP[id] * nav.estWz[id], -0.20, 0.20);
        const double pitchAoaDamping = std::clamp(
            -control.kAlphaP[id] * alpha, -0.15, 0.15);
        const double yawAoaDamping = std::clamp(
            -control.kAlphaP[id] * beta, -0.15, 0.15);

        // Do not create a lateral steering demand from sensor noise when guidance
        // is asking for a straight-plane flight path; keep rate and AoA damping
        // active so the vehicle stays aerodynamically stable and roll/yaw trimmed.
        const double yawFeed = (std::abs(ayCmdB) < 0.5) ? 0.0 : yawFeedForward;
        double pitchDeflection = pitchFeedForward + pitchRateDamping + pitchAoaDamping;
        double yawDeflection   = yawFeed + yawRateDamping + yawAoaDamping;

        // 6. Roll stabilization: wings-level P-D, referenced to the local
        // gravity direction (attitude-independent — works for any initial
        // orientation, unlike a quaternion-identity reference).
        // g_b = gravity direction in body axes: level flight => (0, 0, +1).
        // A right roll by phi gives g_b.y = -sin(phi): rollError = atan2(-g_b.y, g_b.z).
        // In a steep dive g_b.z -> 0 and the reference degenerates; scale the
        // command by the vertical component so wings-level authority fades
        // near-vertical instead of slamming full deflection.
        double gBx, gBy, gBz;
        quatRotateToBody(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                         0.0, 0.0, -1.0, gBx, gBy, gBz);
        const double verticality = std::clamp(gBz, 0.0, 1.0);
        const double rollError = std::atan2(-gBy, std::max(gBz, 0.15));
        double rollDeflection = verticality * (-control.kRollP[id] * rollError - control.kRollD[id] * nav.estWx[id]);

        // 7. Clamp to physical limits (+/- 25 deg = ~0.43 rad, servo authority).
        const double maxDeflection = control.maxDeflectionRad[id];
        const double pitchClamped = std::clamp(pitchDeflection, -maxDeflection, maxDeflection);
        const double yawClamped   = std::clamp(yawDeflection,   -maxDeflection, maxDeflection);
        control.pitchSaturated[id] = std::abs(pitchDeflection) > maxDeflection;
        control.yawSaturated[id]   = std::abs(yawDeflection)   > maxDeflection;
        control.pitchCommand[id] = pitchClamped;
        control.yawCommand[id]   = yawClamped;
        control.rollSaturated[id] = std::abs(rollDeflection) > maxDeflection;
        control.rollCommand[id]  = std::clamp(rollDeflection,  -maxDeflection, maxDeflection);
    }

} // namespace StrikeEngine::Kernel
