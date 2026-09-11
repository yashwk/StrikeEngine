#include <strikeengine/kernel/systems/AutopilotSystem.hpp>
#include <strikeengine/kernel/math/Quaternion.hpp>
#include <strikeengine/models/physics/atmosphere/ISA1976.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>
#include <strikeengine/models/physics/earth/EarthFrames.hpp>
#include <cmath>
#include <algorithm>

namespace StrikeEngine::Kernel {

    namespace {

        double valAt(const std::vector<double>& v, std::size_t i, double def)
        {
            return i < v.size() ? v[i] : def;
        }

        bool flagAt(const std::vector<bool>& v, std::size_t i)
        {
            return i < v.size() && v[i];
        }

        // Node gravity for the body-frame specific-force conversion. Mirrors
        // the environment configuration when requested (J2 > WGS84-normal >
        // spherical for ECEF; flat otherwise); legacy keeps normal gravity for
        // every ECEF mode.
        void autopilotGravity(std::size_t id, const NavigationBlock& nav,
                              const ControlBlock& control,
                              const EnvironmentConfig& environment,
                              double& gx, double& gy, double& gz)
        {
            const bool truth = flagAt(control.useTruthGravityModel, id);
            if (environment.earth.useEcefTruth) {
                const Models::EcefCoordinate pos{nav.estPx[id], nav.estPy[id], nav.estPz[id]};
                if (truth) {
                    if (environment.earth.includeJ2Gravity) {
                        const auto v = Models::j2GravityAccelerationEcef(pos);
                        gx = v.x; gy = v.y; gz = v.z;
                        return;
                    }
                    if (environment.earth.useSphericalGravity) {
                        const auto v = Models::sphericalGravityAccelerationEcef(pos);
                        gx = v.x; gy = v.y; gz = v.z;
                        return;
                    }
                }
                const auto geodetic = Models::ecefToGeodetic(pos);
                const auto grav = Models::EarthFrames::ecefNormalGravityAcceleration(geodetic);
                gx = grav[0]; gy = grav[1]; gz = grav[2];
            } else {
                gx = 0.0; gy = 0.0; gz = -9.80665;
            }
        }

    }

    AutopilotSystem::AutopilotSystem() = default;

    void AutopilotSystem::update(
        const EntityStatusBlock& status,
        const NavigationBlock& nav,
        const SensorBlock& sensor,
        const GuidanceBlock& guidance,
        ControlBlock& control,
        double dt,
        const EnvironmentConfig& environment)
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
                // Park the actuator memory and authority state with the zeroed
                // command so re-entering guidance does not resume the lag /
                // rate-limit filters (or the authority margin) from a stale
                // pre-None value.
                if (i < control.pitchCommandPrev.size()) control.pitchCommandPrev[i] = 0.0;
                if (i < control.yawCommandPrev.size()) control.yawCommandPrev[i] = 0.0;
                if (i < control.rollCommandPrev.size()) control.rollCommandPrev[i] = 0.0;
                if (i < control.authorityMargin01.size()) control.authorityMargin01[i] = 1.0;
                if (i < control.pitchSaturated.size()) control.pitchSaturated[i] = false;
                if (i < control.yawSaturated.size()) control.yawSaturated[i] = false;
                if (i < control.rollSaturated.size()) control.rollSaturated[i] = false;
                continue;
            }

            updateFlightController(i, nav, sensor, guidance, control, dt, environment);
        }
    }

    void AutopilotSystem::updateFlightController(
        std::size_t id,
        const NavigationBlock& nav,
        const SensorBlock& sensor,
        const GuidanceBlock& guidance,
        ControlBlock& control,
        double dt,
        const EnvironmentConfig& environment)
    {
        // 1. Commanded acceleration (world, from guidance) -> body frame
        double axCmdB, ayCmdB, azCmdB;
        quatRotateToBody(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                         guidance.commandedAccelX[id],
                         guidance.commandedAccelY[id],
                         guidance.commandedAccelZ[id],
                         axCmdB, ayCmdB, azCmdB);
        (void)axCmdB; // axial demand is not fin-controlled

        // Guidance commands are total world-frame accelerations, while the
        // accelerometer measures specific force (total acceleration minus
        // gravity). Convert the command into the body-frame specific-force
        // demand before mapping it to the fins. The normal-force term is
        // intentionally feed-forward: feeding the fin's own measured force
        // back into this simplified airframe model creates a short-period
        // limit cycle. Rates and AoA below provide the stabilizing feedback.
        double gx, gy, gz;
        autopilotGravity(id, nav, control, environment, gx, gy, gz);
        double gravityBx, gravityBy, gravityBz;
        quatRotateToBody(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                         gx, gy, gz, gravityBx, gravityBy, gravityBz);
        const double aySpecificCmd = ayCmdB - gravityBy;
        const double azSpecificCmd = azCmdB - gravityBz;
        if (id < control.specificForceDemandY.size()) {
            control.specificForceDemandY[id] = aySpecificCmd;
            control.specificForceDemandZ[id] = azSpecificCmd;
        }

        // 2. AoA / sideslip estimates from estimated body-frame velocity
        double ub, vb, wb;
        quatRotateToBody(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                         nav.estVx[id], nav.estVy[id], nav.estVz[id],
                         ub, vb, wb);
        const double alpha = std::atan2(wb, ub);
        const double beta  = std::atan2(vb, ub);

        // 3. Speed, altitude, Mach and dynamic pressure (shared by the
        //    q-schedule and the control-effectiveness schedule).
        const double vx = nav.estVx[id];
        const double vy = nav.estVy[id];
        const double vz = nav.estVz[id];
        const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
        double alt;
        if (environment.earth.useEcefTruth) {
            const auto geo = Models::ecefToGeodetic({nav.estPx[id], nav.estPy[id], nav.estPz[id]});
            alt = std::max(0.0, geo.altitudeM);
        } else {
            alt = std::max(0.0, nav.estPz[id]);
        }
        static const Models::ISA1976 isa;
        const auto atmos = isa.evaluate(alt);
        const double qEst = 0.5 * atmos.density * speed * speed;
        const double soundSpeed = atmos.temperature > 0.0
            ? std::sqrt(1.4 * 287.05287 * atmos.temperature) : 340.29;
        const double mach = soundSpeed > 1.0 ? speed / soundSpeed : 0.0;
        if (id < control.machNumber.size()) control.machNumber[id] = mach;

        double sqScale = 1.0;
        if (flagAt(control.gainSchedulingEnabled, id)) {
            const double qRef = (id < control.refDynamicPressurePa.size() && control.refDynamicPressurePa[id] > 0.0)
                ? control.refDynamicPressurePa[id] : 50000.0;
            const double qMin = (id < control.minDynamicPressurePa.size() && control.minDynamicPressurePa[id] > 0.0)
                ? control.minDynamicPressurePa[id] : 2000.0;
            const double qMax = (id < control.maxDynamicPressurePa.size() && control.maxDynamicPressurePa[id] >= qMin)
                ? control.maxDynamicPressurePa[id] : 300000.0;
            const double qClamped = std::clamp(qEst, qMin, qMax);
            sqScale = std::clamp(std::sqrt(qRef / qClamped), 0.2, 5.0);
        }

        // Control effectiveness schedule (Mach): one factor applied to the
        // feed-forward (and optionally the damping terms).
        double effectiveness = 1.0;
        if (flagAt(control.controlEffectivenessEnabled, id)) {
            const double base = valAt(control.controlEffBase, id, 1.0);
            const double slope = valAt(control.controlEffMachSlope, id, 0.0);
            const double quad = valAt(control.controlEffMachQuad, id, 0.0);
            const double lo = valAt(control.controlEffMin, id, 0.2);
            const double hi = valAt(control.controlEffMax, id, 5.0);
            effectiveness = std::clamp(base + slope * mach + quad * mach * mach, lo, hi);
        }
        if (id < control.controlEffectiveness.size()) control.controlEffectiveness[id] = effectiveness;

        const double effectiveKAccel = control.kAccelP[id] * sqScale * effectiveness;
        if (id < control.effectiveKAccel.size()) control.effectiveKAccel[id] = effectiveKAccel;

        // 4. Damping feedback source: legacy nav rate estimate, optionally the
        //    measured gyro (isolates the inner loop from nav attitude errors).
        double wx = nav.estWx[id], wy = nav.estWy[id], wz = nav.estWz[id];
        if (flagAt(control.useMeasuredRatesEnabled, id) &&
            id < sensor.gyroX.size() && std::isfinite(sensor.gyroX[id]) &&
            std::isfinite(sensor.gyroY[id]) && std::isfinite(sensor.gyroZ[id])) {
            wx = sensor.gyroX[id]; wy = sensor.gyroY[id]; wz = sensor.gyroZ[id];
        }

        const double kRatePitch = valAt(control.kRatePitchP, id, -1.0) >= 0.0
            ? control.kRatePitchP[id] : control.kRateP[id];
        const double kRateYaw = valAt(control.kRateYawP, id, -1.0) >= 0.0
            ? control.kRateYawP[id] : control.kRateP[id];
        const bool scheduleDamping = flagAt(control.scheduleAllTerms, id);
        const double dampScale = scheduleDamping ? sqScale * effectiveness : 1.0;

        // Positive body-Z specific force is down and requires nose-down
        // (negative wy / fin); positive body-Y force requires nose-right
        // (positive wz / fin). The signs below match AeroModel's documented
        // X-forward/Y-right/Z-down convention.
        const double pitchFeedForward = std::clamp(
            -effectiveKAccel * azSpecificCmd, -0.35, 0.35);
        const double yawFeedForward = std::clamp(
            effectiveKAccel * aySpecificCmd, -0.35, 0.35);
        const double pitchAoaDamping = std::clamp(
            -control.kAlphaP[id] * alpha * dampScale, -0.15, 0.15);
        const double yawAoaDamping = std::clamp(
            -control.kAlphaP[id] * beta * dampScale, -0.15, 0.15);
        // Rate damping uses the (optionally split) pitch/yaw gains; both
        // default to the legacy kRateP when unset.
        const double pitchRateDampingOut = std::clamp(
            -kRatePitch * wy * dampScale, -0.20, 0.20);
        const double yawRateDampingOut = std::clamp(
            -kRateYaw * wz * dampScale, -0.20, 0.20);

        // Do not create a lateral steering demand from sensor noise when guidance
        // is asking for a straight-plane flight path; keep rate and AoA damping
        // active so the vehicle stays aerodynamically stable and roll/yaw trimmed.
        // Legacy: a hard 0.5 m/s^2 gate. Optional: a smooth ramp over the
        // configured width (no chattering at the threshold).
        double yawGate = 1.0;
        if (flagAt(control.yawDeadbandSmoothEnabled, id)) {
            const double width = std::max(1e-6, valAt(control.yawDeadbandWidthMps2, id, 0.5));
            yawGate = std::clamp(std::abs(aySpecificCmd) / width, 0.0, 1.0);
        } else if (std::abs(aySpecificCmd) < 0.5) {
            yawGate = 0.0;
        }

        // 5. Integral trim on the specific-force error (measured accelerometer
        //    minus demand), with a clamp and saturation anti-windup.
        double pitchIntegral = id < control.pitchIntegral.size() ? control.pitchIntegral[id] : 0.0;
        double yawIntegral = id < control.yawIntegral.size() ? control.yawIntegral[id] : 0.0;
        if (flagAt(control.integralEnabled, id) && dt > 0.0 &&
            id < sensor.accelY.size()) {
            const double clampI = std::max(0.0, valAt(control.integralClampRad, id, 0.05));
            const double errPitch = azSpecificCmd - sensor.accelZ[id];
            const double errYaw = aySpecificCmd - sensor.accelY[id];
            if (!control.pitchSaturated[id]) {
                pitchIntegral += control.kIntegralPitch[id] * errPitch * dt;
            }
            if (!control.yawSaturated[id]) {
                yawIntegral += control.kIntegralYaw[id] * errYaw * dt;
            }
            pitchIntegral = std::clamp(pitchIntegral, -clampI, clampI);
            yawIntegral = std::clamp(yawIntegral, -clampI, clampI);
        } else {
            pitchIntegral = 0.0;
            yawIntegral = 0.0;
        }
        control.pitchIntegral[id] = pitchIntegral;
        control.yawIntegral[id] = yawIntegral;

        double pitchDeflection = pitchFeedForward + pitchRateDampingOut + pitchAoaDamping + pitchIntegral;
        double yawDeflection   = yawGate * yawFeedForward + yawRateDampingOut + yawAoaDamping + yawIntegral;

        // 6. Roll stabilization: wings-level P-D, referenced to the local
        // gravity direction (attitude-independent). Roll is optionally
        // suppressed while a lateral demand is being served (skid steering) so
        // it does not fight the yaw channel.
        double gBx, gBy, gBz;
        if (environment.earth.useEcefTruth) {
            const Models::EcefCoordinate pos{nav.estPx[id], nav.estPy[id], nav.estPz[id]};
            const auto geodetic = Models::ecefToGeodetic(pos);
            const auto grav = Models::EarthFrames::ecefNormalGravityAcceleration(geodetic);
            const double gmag = std::sqrt(grav[0]*grav[0] + grav[1]*grav[1] + grav[2]*grav[2]);
            const double nx = gmag > 1e-9 ? grav[0]/gmag : 0.0;
            const double ny = gmag > 1e-9 ? grav[1]/gmag : 0.0;
            const double nz = gmag > 1e-9 ? grav[2]/gmag : -1.0;
            quatRotateToBody(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                             nx, ny, nz, gBx, gBy, gBz);
        } else {
            quatRotateToBody(nav.estQw[id], nav.estQx[id], nav.estQy[id], nav.estQz[id],
                             0.0, 0.0, -1.0, gBx, gBy, gBz);
        }
        const double verticality = std::clamp(gBz, 0.0, 1.0);
        const double rollError = std::atan2(-gBy, std::max(gBz, 0.15));
        double rollScale = scheduleDamping ? sqScale * effectiveness : 1.0;
        const double rollSuppress = valAt(control.rollSuppressLateralAccelMps2, id, 0.0);
        if (rollSuppress > 0.0 && std::abs(aySpecificCmd) > rollSuppress) {
            rollScale *= std::clamp(rollSuppress / std::abs(aySpecificCmd), 0.0, 1.0);
        }
        double rollDeflection = verticality * rollScale *
            (-control.kRollP[id] * rollError - control.kRollD[id] * wx);

        // 7. Clamp to physical limits (+/- 25 deg = ~0.43 rad, servo authority).
        //    Publish the demand breakdown and the delivered/demanded margin.
        if (id < control.feedForwardPitch.size()) {
            control.feedForwardPitch[id] = pitchFeedForward;
            control.feedForwardYaw[id] = yawGate * yawFeedForward;
            control.rateDampingPitch[id] = pitchRateDampingOut;
            control.rateDampingYaw[id] = yawRateDampingOut;
            control.aoaDampingPitch[id] = pitchAoaDamping;
            control.aoaDampingYaw[id] = yawAoaDamping;
        }
        const double maxDeflection = control.maxDeflectionRad[id];
        const double pitchClamped = std::clamp(pitchDeflection, -maxDeflection, maxDeflection);
        const double yawClamped   = std::clamp(yawDeflection,   -maxDeflection, maxDeflection);
        control.pitchSaturated[id] = std::abs(pitchDeflection) > maxDeflection;
        control.yawSaturated[id]   = std::abs(yawDeflection)   > maxDeflection;
        if (id < control.authorityMargin01.size()) {
            double margin = 1.0;
            const double pMag = std::abs(pitchDeflection);
            const double yMag = std::abs(yawDeflection);
            if (pMag > maxDeflection) margin = std::min(margin, maxDeflection / pMag);
            if (yMag > maxDeflection) margin = std::min(margin, maxDeflection / yMag);
            control.authorityMargin01[id] = std::clamp(margin, 0.0, 1.0);
        }

        // 8. Actuator model on the command: first-order lag then a rate limit
        //    (legacy commands the ideal deflection). State is per entity.
        double pitchOut = pitchClamped;
        double yawOut = yawClamped;
        const double lagSec = valAt(control.commandLagSec, id, 0.0);
        const double rateLimit = valAt(control.commandRateLimitRadPerSec, id, 0.0);
        if (lagSec > 0.0 || rateLimit > 0.0) {
            const double prevPitch = id < control.pitchCommandPrev.size() ? control.pitchCommandPrev[id] : pitchOut;
            const double prevYaw = id < control.yawCommandPrev.size() ? control.yawCommandPrev[id] : yawOut;
            if (lagSec > 0.0 && dt > 0.0) {
                const double a = 1.0 - std::exp(-dt / lagSec);
                pitchOut = prevPitch + a * (pitchClamped - prevPitch);
                yawOut = prevYaw + a * (yawClamped - prevYaw);
            }
            if (rateLimit > 0.0 && dt > 0.0) {
                const double step = rateLimit * dt;
                pitchOut = prevPitch + std::clamp(pitchOut - prevPitch, -step, step);
                yawOut = prevYaw + std::clamp(yawOut - prevYaw, -step, step);
            }
        }
        if (id < control.pitchCommandPrev.size()) control.pitchCommandPrev[id] = pitchOut;
        if (id < control.yawCommandPrev.size()) control.yawCommandPrev[id] = yawOut;
        control.pitchCommand[id] = pitchOut;
        control.yawCommand[id] = yawOut;
        control.rollSaturated[id] = std::abs(rollDeflection) > maxDeflection;
        const double rollClamped = std::clamp(rollDeflection, -maxDeflection, maxDeflection);
        const double prevRoll = id < control.rollCommandPrev.size() ? control.rollCommandPrev[id] : rollClamped;
        double rollOut = rollClamped;
        if (lagSec > 0.0 && dt > 0.0) {
            const double a = 1.0 - std::exp(-dt / lagSec);
            rollOut = prevRoll + a * (rollClamped - prevRoll);
        }
        if (id < control.rollCommandPrev.size()) control.rollCommandPrev[id] = rollOut;
        control.rollCommand[id] = rollOut;
    }

} // namespace StrikeEngine::Kernel
