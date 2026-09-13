// autopilot_fidelity_test: integral trim, control-effectiveness scheduling,
// authority margin, yaw dead-band smoothing, split rate gains, term
// scheduling, actuator lag/rate limits, measured-rate damping, diagnostics,
// and config round-trips. All opt-in; the legacy law is covered by the
// existing intercept/rocket/aero suites.
#include <strikeengine/kernel/systems/AutopilotSystem.hpp>
#include <strikeengine/kernel/config/ConfigSerialization.hpp>

#include <cmath>
#include <cstdio>

using namespace StrikeEngine::Kernel;

namespace {

int failures = 0;
void check(bool ok, const char* message)
{
    if (ok) std::printf("  [PASS] %s\n", message);
    else { std::printf("  [FAIL] %s\n", message); ++failures; }
}

ControlBlock makeControl()
{
    ControlBlock c;
    c.thrustCommand = {0.0};
    c.pitchCommand = {0.0}; c.yawCommand = {0.0}; c.rollCommand = {0.0};
    c.thrustVectorPitchCommand = {0.0}; c.thrustVectorYawCommand = {0.0};
    c.kAccelP = {0.030}; c.kRateP = {1.0}; c.kAlphaP = {0.2};
    c.kRollP = {0.1}; c.kRollD = {0.05}; c.maxDeflectionRad = {0.43};
    c.gainSchedulingEnabled = {false};
    c.refDynamicPressurePa = {50000.0};
    c.minDynamicPressurePa = {2000.0};
    c.maxDynamicPressurePa = {300000.0};
    c.kRatePitchP = {-1.0}; c.kRateYawP = {-1.0};
    c.scheduleAllTerms = {false};
    c.integralEnabled = {false};
    c.kIntegralPitch = {0.0}; c.kIntegralYaw = {0.0}; c.integralClampRad = {0.05};
    // Legacy fixtures exercise the pre-acceleration-loop paths; the loop is
    // covered by its own section below.
    c.kAccelErrP = {0.0};
    c.threeLoopEnabled = {false};
    c.accelErrPitch = {0.0}; c.accelErrYaw = {0.0};
    c.rateCommandPitch = {0.0}; c.rateCommandYaw = {0.0};
    c.achievedSpecificForceY = {0.0}; c.achievedSpecificForceZ = {0.0};
    c.controlEffectivenessEnabled = {false};
    c.controlEffBase = {1.0}; c.controlEffMachSlope = {0.0}; c.controlEffMachQuad = {0.0};
    c.controlEffMin = {0.2}; c.controlEffMax = {5.0};
    c.yawDeadbandSmoothEnabled = {false}; c.yawDeadbandWidthMps2 = {0.5};
    c.commandLagSec = {0.0}; c.commandRateLimitRadPerSec = {0.0};
    c.useMeasuredRatesEnabled = {false};
    c.rollSuppressLateralAccelMps2 = {0.0};
    c.useTruthGravityModel = {false};
    c.pitchIntegral = {0.0}; c.yawIntegral = {0.0};
    c.pitchCommandPrev = {0.0}; c.yawCommandPrev = {0.0}; c.rollCommandPrev = {0.0};
    c.specificForceDemandY = {0.0}; c.specificForceDemandZ = {0.0};
    c.effectiveKAccel = {0.0}; c.controlEffectiveness = {1.0}; c.machNumber = {0.0};
    c.feedForwardPitch = {0.0}; c.feedForwardYaw = {0.0};
    c.rateDampingPitch = {0.0}; c.rateDampingYaw = {0.0};
    c.aoaDampingPitch = {0.0}; c.aoaDampingYaw = {0.0};
    c.authorityMargin01 = {1.0};
    c.pitchSaturated = {false}; c.yawSaturated = {false}; c.rollSaturated = {false};
    return c;
}

NavigationBlock makeNav()
{
    NavigationBlock nav;
    nav.size = 1;
    nav.estPx = {0.0}; nav.estPy = {0.0}; nav.estPz = {5000.0};
    nav.estVx = {100.0}; nav.estVy = {0.0}; nav.estVz = {0.0};
    nav.estQw = {1.0}; nav.estQx = {0.0}; nav.estQy = {0.0}; nav.estQz = {0.0};
    nav.estWx = {0.0}; nav.estWy = {0.0}; nav.estWz = {0.0};
    return nav;
}

SensorBlock makeSensor()
{
    SensorBlock s;
    s.size = 1;
    s.accelX = {0.0}; s.accelY = {0.0}; s.accelZ = {0.0};
    s.gyroX = {0.0}; s.gyroY = {0.0}; s.gyroZ = {0.0};
    return s;
}

GuidanceBlock makeGuidance(double ay = 10.0)
{
    GuidanceBlock g;
    g.mode = {GuidanceMode::ProportionalNavigation};
    g.commandedAccelX = {0.0};
    g.commandedAccelY = {ay};
    g.commandedAccelZ = {0.0};
    return g;
}

EntityStatusBlock makeStatus()
{
    EntityStatusBlock st;
    st.size = 1;
    st.isAlive = {true};
    return st;
}

void run(const NavigationBlock& nav, const SensorBlock& sensor, const GuidanceBlock& g,
         ControlBlock& c, double dt)
{
    EntityStatusBlock st = makeStatus();
    AutopilotSystem ap;
    EnvironmentConfig env;
    ap.update(st, nav, sensor, g, c, dt, env);
}

} // namespace

int main()
{
    std::printf("=== autopilot_fidelity_test ===\n");
    const double dt = 0.01;

    // ---- 1. Integral trim accumulates on the specific-force error ----
    {
        NavigationBlock nav = makeNav();
        SensorBlock sensor = makeSensor();
        GuidanceBlock g = makeGuidance();
        ControlBlock plain = makeControl();
        run(nav, sensor, g, plain, dt);
        ControlBlock integral = makeControl();
        integral.integralEnabled = {true};
        integral.kIntegralYaw = {5.0};
        integral.integralClampRad = {5.0};
        for (int k = 0; k < 10; ++k) run(nav, sensor, g, integral, dt);
        check(std::abs(integral.yawCommand[0]) > std::abs(plain.yawCommand[0]),
              "integral trim adds deflection toward the specific-force error");
        check(std::abs(integral.yawIntegral[0]) > 0.0,
              "integral state accumulates while the error persists");
    }

    // ---- 1b. Acceleration-error loop (outer loop, default auto gain) ----
    {
        NavigationBlock nav = makeNav();
        SensorBlock sensor = makeSensor();      // measured force 0: lags the demand
        sensor.accelZ = {9.80665};              // Z channel at gravity trim
        GuidanceBlock g = makeGuidance(10.0);
        ControlBlock off = makeControl();       // kAccelErrP = 0 (legacy law)
        for (int k = 0; k < 100; ++k) run(nav, sensor, g, off, dt);
        ControlBlock loop = makeControl();
        loop.kAccelErrP = {-1.0};               // auto = effectiveKAccel
        for (int k = 0; k < 100; ++k) run(nav, sensor, g, loop, dt);
        check(std::abs(loop.yawCommand[0]) > std::abs(off.yawCommand[0]),
              "acceleration loop adds deflection when the measured force trails the demand");
        std::printf("  [info] effK=%.5f errYaw=%.4f errPitch=%.4f yawCmd=%.4f\n",
                    loop.effectiveKAccel[0], loop.accelErrYaw[0], loop.accelErrPitch[0],
                    loop.yawCommand[0]);
        check(loop.accelErrYaw[0] > 0.0 && std::abs(loop.accelErrPitch[0]) < 1e-3,
              "acceleration-loop trim follows the demand sign convention");
        check(loop.kAccelErrP[0] == -1.0,
              "acceleration-loop gain keeps its auto sentinel");
    }

    // ---- 1c. Authority margin tracks measured delivery ----
    {
        NavigationBlock nav = makeNav();
        SensorBlock sensor = makeSensor();
        sensor.accelY = {2.0};                  // 20% of the 10 m/s^2 maneuver demand
        sensor.accelZ = {9.80665};              // Z channel at gravity trim
        GuidanceBlock g = makeGuidance(10.0);
        ControlBlock loop = makeControl();
        loop.kAccelErrP = {-1.0};
        for (int k = 0; k < 100; ++k) run(nav, sensor, g, loop, dt);
        std::printf("  [info] achY=%.4f achZ=%.4f margin=%.4f\n",
                    loop.achievedSpecificForceY[0], loop.achievedSpecificForceZ[0],
                    loop.authorityMargin01[0]);
        check(std::abs(loop.authorityMargin01[0] - 0.2) < 0.05,
              "authority margin reports the measured delivered/demanded ratio");
    }

    // ---- 1d. Three-loop cascade (Jackson Fig. 6) ----
    {
        NavigationBlock nav = makeNav();
        SensorBlock sensor = makeSensor();
        sensor.accelY = {2.0};                  // 20% of the 10 m/s^2 maneuver
        sensor.accelZ = {9.80665};              // Z channel at gravity trim
        GuidanceBlock g = makeGuidance(10.0);
        ControlBlock tl = makeControl();
        tl.threeLoopEnabled = {true};
        tl.kAccelErrP = {4.5};                  // Ka (accel error -> rate command)
        tl.kIntegralPitch = {14.3};             // Ki (rate-loop integral)
        tl.kIntegralYaw = {14.3};
        tl.integralClampRad = {1.0};
        for (int k = 0; k < 100; ++k) run(nav, sensor, g, tl, dt);
        check(tl.rateCommandYaw[0] > 0.0,
              "three-loop commands nose-right rate for a positive Y accel error");
        check(std::abs(tl.rateCommandPitch[0]) < 1e-2,
              "three-loop pitch rate command is zero at Z trim");
        check(tl.yawIntegral[0] > 0.0,
              "three-loop inner integral accumulates on the rate error");
        check(std::abs(tl.accelErrYaw[0]) < 1e-9 && std::abs(tl.accelErrPitch[0]) < 1e-9,
              "three-loop path bypasses the legacy direct-force trim");
    }

    // ---- 2. Control-effectiveness scheduling scales the feed-forward ----
    {
        NavigationBlock nav = makeNav();
        SensorBlock sensor = makeSensor();
        GuidanceBlock g = makeGuidance(5.0);
        ControlBlock plain = makeControl();
        run(nav, sensor, g, plain, dt);
        ControlBlock eff = makeControl();
        eff.controlEffectivenessEnabled = {true};
        eff.controlEffBase = {1.0};
        eff.controlEffMachSlope = {1.0};
        run(nav, sensor, g, eff, dt);
        check(eff.controlEffectiveness[0] > 1.0 && eff.effectiveKAccel[0] > plain.effectiveKAccel[0],
              "Mach effectiveness schedule raises the effective accel gain");
        check(std::abs(eff.yawCommand[0]) > std::abs(plain.yawCommand[0]),
              "scheduled feed-forward produces a larger fin demand");
    }

    // ---- 3. Authority margin reports fin saturation ----
    {
        NavigationBlock nav = makeNav();
        nav.estWy = {1.0}; // drives the rate-damping term into the clamp
        SensorBlock sensor = makeSensor();
        GuidanceBlock g = makeGuidance(10.0);
        ControlBlock c = makeControl();
        run(nav, sensor, g, c, dt);
        check(c.pitchSaturated[0] && c.authorityMargin01[0] < 1.0,
              "saturated fin command reports a delivered/demanded margin below one");
    }

    // ---- 4. Smooth yaw dead-band replaces the hard gate ----
    {
        NavigationBlock nav = makeNav();
        SensorBlock sensor = makeSensor();
        GuidanceBlock g = makeGuidance(0.3); // below the legacy 0.5 gate
        ControlBlock plain = makeControl();
        run(nav, sensor, g, plain, dt);
        ControlBlock smooth = makeControl();
        smooth.yawDeadbandSmoothEnabled = {true};
        smooth.yawDeadbandWidthMps2 = {0.5};
        run(nav, sensor, g, smooth, dt);
        check(std::abs(plain.yawCommand[0]) < 1e-12,
              "legacy dead-band gates the small lateral demand to zero");
        check(std::abs(smooth.yawCommand[0]) > 0.0,
              "smooth dead-band passes a proportional small demand");
    }

    // ---- 5. Split pitch/yaw rate gains ----
    {
        NavigationBlock nav = makeNav();
        nav.estWz = {0.1};
        SensorBlock sensor = makeSensor();
        GuidanceBlock g = makeGuidance(0.0);
        ControlBlock plain = makeControl();
        run(nav, sensor, g, plain, dt);
        ControlBlock split = makeControl();
        split.kRateYawP = {2.0};
        run(nav, sensor, g, split, dt);
        check(std::abs(split.yawCommand[0]) > std::abs(plain.yawCommand[0]),
              "split yaw rate gain changes the yaw damping demand");
    }

    // ---- 6. scheduleAllTerms q-schedules the damping terms ----
    {
        NavigationBlock nav = makeNav();
        nav.estWz = {0.1};
        SensorBlock sensor = makeSensor();
        GuidanceBlock g = makeGuidance(0.0);
        ControlBlock ffOnly = makeControl();
        ffOnly.gainSchedulingEnabled = {true};
        run(nav, sensor, g, ffOnly, dt);
        ControlBlock all = makeControl();
        all.gainSchedulingEnabled = {true};
        all.scheduleAllTerms = {true};
        run(nav, sensor, g, all, dt);
        check(std::abs(all.yawCommand[0]) > std::abs(ffOnly.yawCommand[0]),
              "scheduleAllTerms scales the rate-damping term too");
    }

    // ---- 7. Actuator lag and rate limit on the fin command ----
    {
        NavigationBlock nav = makeNav();
        SensorBlock sensor = makeSensor();
        GuidanceBlock g = makeGuidance(10.0);
        ControlBlock lagged = makeControl();
        lagged.commandLagSec = {0.1};
        run(nav, sensor, g, lagged, dt);
        check(lagged.yawCommand[0] > 0.0 && lagged.yawCommand[0] < 0.3,
              "first-order actuator lag smooths the commanded deflection");

        ControlBlock limited = makeControl();
        limited.commandRateLimitRadPerSec = {0.1};
        run(nav, sensor, g, limited, dt);
        check(std::abs(limited.yawCommand[0] - 0.001) < 1e-12,
              "actuator rate limit bounds the first-step deflection");
    }

    // ---- 8. Measured-rate damping source ----
    {
        NavigationBlock nav = makeNav(); // zero nav rates
        SensorBlock sensor = makeSensor();
        sensor.gyroZ = {0.5};
        GuidanceBlock g = makeGuidance(0.0);
        ControlBlock plain = makeControl();
        run(nav, sensor, g, plain, dt);
        ControlBlock measured = makeControl();
        measured.useMeasuredRatesEnabled = {true};
        run(nav, sensor, g, measured, dt);
        check(std::abs(plain.yawCommand[0]) < 1e-12,
              "nav-rate damping sees zero rate in the test rig");
        check(std::abs(measured.yawCommand[0]) > 0.0,
              "measured gyro rate drives the damping term");
    }

    // ---- 9. Diagnostics are published ----
    {
        NavigationBlock nav = makeNav();
        SensorBlock sensor = makeSensor();
        GuidanceBlock g = makeGuidance(10.0);
        ControlBlock c = makeControl();
        run(nav, sensor, g, c, dt);
        check(std::isfinite(c.specificForceDemandY[0]) &&
              std::isfinite(c.specificForceDemandZ[0]) &&
              std::isfinite(c.machNumber[0]) && c.machNumber[0] > 0.0 &&
              std::isfinite(c.feedForwardYaw[0]),
              "specific-force demand, Mach and the demand breakdown are finite");
    }

    // ---- 10. Actuator memory parks on GuidanceMode::None ----
    {
        NavigationBlock nav = makeNav();
        SensorBlock sensor = makeSensor();
        GuidanceBlock g = makeGuidance(10.0);
        ControlBlock c = makeControl();
        c.commandLagSec = {0.1};
        run(nav, sensor, g, c, dt);
        run(nav, sensor, g, c, dt);
        check(c.pitchCommandPrev[0] != 0.0,
              "lag filter holds nonzero actuator memory while guiding");
        g.mode = {GuidanceMode::None};
        run(nav, sensor, g, c, dt);
        check(c.pitchCommandPrev[0] == 0.0 && c.yawCommandPrev[0] == 0.0 &&
              c.rollCommandPrev[0] == 0.0 && c.authorityMargin01[0] == 1.0 &&
              !c.pitchSaturated[0] && !c.yawSaturated[0],
              "None parks actuator memory, margin and saturation flags");
    }

    // ---- 11. Config round-trip of the new autopilot keys ----
    {
        VehicleConfig cfg;
        cfg.guidanceAutopilot.kRatePitchP = 1.5;
        cfg.guidanceAutopilot.kRateYawP = 2.5;
        cfg.guidanceAutopilot.scheduleAllTerms = true;
        cfg.guidanceAutopilot.autopilotIntegralEnabled = true;
        cfg.guidanceAutopilot.kIntegralPitch = 0.4;
        cfg.guidanceAutopilot.kIntegralYaw = 0.6;
        cfg.guidanceAutopilot.integralClampRad = 0.08;
        cfg.guidanceAutopilot.controlEffectivenessEnabled = true;
        cfg.guidanceAutopilot.controlEffBase = 0.9;
        cfg.guidanceAutopilot.controlEffMachSlope = 0.3;
        cfg.guidanceAutopilot.controlEffMachQuad = -0.02;
        cfg.guidanceAutopilot.controlEffMin = 0.5;
        cfg.guidanceAutopilot.controlEffMax = 3.0;
        cfg.guidanceAutopilot.yawDeadbandSmoothEnabled = true;
        cfg.guidanceAutopilot.yawDeadbandWidthMps2 = 0.8;
        cfg.guidanceAutopilot.commandLagSec = 0.03;
        cfg.guidanceAutopilot.commandRateLimitRadPerSec = 4.0;
        cfg.guidanceAutopilot.useMeasuredRatesEnabled = true;
        cfg.guidanceAutopilot.rollSuppressLateralAccelMps2 = 5.0;
        cfg.guidanceAutopilot.useTruthGravityModel = true;
        cfg.guidanceAutopilot.guidanceAuthorityAwareLimitEnabled = true;
        const VehicleConfig back = deserializeVehicleConfig(serializeVehicleConfig(cfg));
        const auto& a = back.guidanceAutopilot;
        check(a.kRatePitchP == 1.5 && a.kRateYawP == 2.5 && a.scheduleAllTerms &&
              a.autopilotIntegralEnabled && a.kIntegralPitch == 0.4 &&
              a.kIntegralYaw == 0.6 && a.integralClampRad == 0.08,
              "integral/rate-gain keys round-trip");
        check(a.controlEffectivenessEnabled && a.controlEffBase == 0.9 &&
              a.controlEffMachSlope == 0.3 && a.controlEffMachQuad == -0.02 &&
              a.controlEffMin == 0.5 && a.controlEffMax == 3.0,
              "control-effectiveness keys round-trip");
        check(a.yawDeadbandSmoothEnabled && a.yawDeadbandWidthMps2 == 0.8 &&
              a.commandLagSec == 0.03 && a.commandRateLimitRadPerSec == 4.0 &&
              a.useMeasuredRatesEnabled && a.rollSuppressLateralAccelMps2 == 5.0 &&
              a.useTruthGravityModel && a.guidanceAuthorityAwareLimitEnabled,
              "actuator/roll/authority keys round-trip");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
