#include <strikeengine/kernel/systems/AutopilotSystem.hpp>
#include <strikeengine/kernel/math/Quaternion.hpp>
#include <cmath>
#include <algorithm>

namespace StrikeEngine::Kernel {

    AutopilotSystem::AutopilotSystem() = default;

    void AutopilotSystem::update(
        const EntityStatusBlock& status,
        const NavigationBlock& nav,
        const SensorBlock& sensor,
        const GuidanceBlock& guidance,
        ControlBlock& control,
        double dt)
    {
        // Resize control block if needed (usually handled in SimulationKernel, but safe to check)
        if (control.pitchCommand.size() < nav.size) {
            control.pitchCommand.resize(nav.size, 0.0);
            control.yawCommand.resize(nav.size, 0.0);
            control.rollCommand.resize(nav.size, 0.0);
            control.thrustCommand.resize(nav.size, 0.0);
        }

        if (pitchIntegral.size() < nav.size) {
            pitchIntegral.resize(nav.size, 0.0);
            yawIntegral.resize(nav.size, 0.0);
            wasCommanded.resize(nav.size, false);
            azFiltered.resize(nav.size, 0.0);
            ayFiltered.resize(nav.size, 0.0);
        }

        for (std::size_t i = 0; i < nav.size; ++i) {
            if (!status.isAlive[i]) continue;

            if (guidance.mode[i] == GuidanceMode::None) {
                control.pitchCommand[i] = 0.0;
                control.yawCommand[i] = 0.0;
                control.rollCommand[i] = 0.0;
                pitchIntegral[i] = 0.0;
                yawIntegral[i] = 0.0;
                azFiltered[i] = 0.0;
                ayFiltered[i] = 0.0;
                wasCommanded[i] = false;
                continue;
            }

            updateFlightController(i, nav, sensor, guidance, control, dt);
            wasCommanded[i] = true;
        }
    }

    void AutopilotSystem::updateFlightController(
        std::size_t id,
        const NavigationBlock& nav,
        const SensorBlock& sensor,
        const GuidanceBlock& guidance,
        ControlBlock& control,
        double dt)
    {
        // 1. Commanded acceleration (world, from guidance) -> body frame
        double axCmdB, ayCmdB, azCmdB;
        quatRotateToBody(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                         guidance.commandedAccelX[id],
                         guidance.commandedAccelY[id],
                         guidance.commandedAccelZ[id],
                         axCmdB, ayCmdB, azCmdB);

        // 2. Measured specific force (body frame, from sensors), low-pass
        //    filtered. Without the lag, the fin's own lift feeds straight
        //    back through azMeas in the same tick and couples the outer loop
        //    to the short period (fin/accel limit cycle on stiff airframes).
        azFiltered[id] += (sensor.accelZ[id] - azFiltered[id]) * (dt / kAccelFilTau);
        ayFiltered[id] += (sensor.accelY[id] - ayFiltered[id]) * (dt / kAccelFilTau);
        const double azMeas = azFiltered[id];
        const double ayMeas = ayFiltered[id];

        // 3. Outer loop: accel error -> desired body rate (P + integral).
        //    The integral is provisional here; anti-windup back-calculation
        //    happens after the deflection clamp (step 7) so a saturated fin
        //    cannot wind the integrator up into a permanent slam.
        const double ez = azCmdB - azMeas;
        const double ey = ayCmdB - ayMeas;
        pitchIntegral[id] += ez * dt;
        yawIntegral[id]   += ey * dt;
        pitchIntegral[id] = std::clamp(pitchIntegral[id], -10.0, 10.0);
        yawIntegral[id]   = std::clamp(yawIntegral[id],   -10.0, 10.0);

        // Positive az (down) needs nose-DOWN: negative wy. Positive ay (right)
        // needs nose-RIGHT: positive wz.
        const double pitchRateCmd = -(kAccelP * ez + kAccelI * pitchIntegral[id]);
        const double yawRateCmd   =   (kAccelP * ey + kAccelI * yawIntegral[id]);

        // 4. AoA / sideslip estimates from estimated body-frame velocity
        double ub, vb, wb;
        quatRotateToBody(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                         nav.estVx[id], nav.estVy[id], nav.estVz[id],
                         ub, vb, wb);
        const double alpha = std::atan2(wb, ub);
        const double beta  = std::atan2(vb, ub);

        // 5. Inner loop: desired rate -> deflection, with AoA damping
        double pitchDeflection = kRateP * (pitchRateCmd - nav.estWy[id]) - kAlphaP * alpha;
        double yawDeflection   = kRateP * (yawRateCmd   - nav.estWz[id]) - kAlphaP * beta;

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
        double rollDeflection = verticality * (-kRollP * rollError - kRollD * nav.estWx[id]);

        // 7. Clamp to physical limits (+/- 25 deg = ~0.43 rad, servo authority),
        //    then back-calculate the integrals that would just reach the
        //    clamp (anti-windup): when saturated, the integral is re-set so
        //    the unclamped command equals the limit. Without this, a
        //    sustained guidance demand pins the fins at max forever.
        constexpr double maxDeflection = 0.43;
        const double pitchClamped = std::clamp(pitchDeflection, -maxDeflection, maxDeflection);
        const double yawClamped   = std::clamp(yawDeflection,   -maxDeflection, maxDeflection);
        control.pitchCommand[id] = pitchClamped;
        control.yawCommand[id]   = yawClamped;
        control.rollCommand[id]  = std::clamp(rollDeflection,  -maxDeflection, maxDeflection);

        if (pitchClamped != pitchDeflection) {
            // pitchRateCmd = -(kAccelP*ez + kAccelI*iz); solve for iz at the clamp
            const double rateAtLimit = (pitchClamped + kAlphaP * alpha) / kRateP + nav.estWy[id];
            pitchIntegral[id] = std::clamp(-(rateAtLimit + kAccelP * ez) / kAccelI, -10.0, 10.0);
        }
        if (yawClamped != yawDeflection) {
            const double rateAtLimit = (yawClamped + kAlphaP * beta) / kRateP + nav.estWz[id];
            yawIntegral[id] = std::clamp(-(rateAtLimit + kAccelP * ey) / kAccelI, -10.0, 10.0);
        }
    }

} // namespace StrikeEngine::Kernel
