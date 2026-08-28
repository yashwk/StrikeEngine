// WGS84 rocket-launch MVP with data-driven aero coefficient tables.
//
// Runs the SAME single-stage vertical-launch vehicle as rocket_mvp_test
// (5 m x 0.4 m -> S = 0.1256637 m^2, L = 5 m, 500/350 kg, 60 kN single-stage
// motor with Isp 250/220, WGS84 gravity at 28.5 N, dt = 0.01 s,
// GuidanceMode::None) under BOTH aero models:
//   (1) constant coefficients (cd = 0.25, clAlpha = 2.0) -- the rocket_mvp
//       baseline, and
//   (2) the sa_missile_mk1 cd(M,a)/cl(M,a) tables.
// and prints a side-by-side comparison of the key flight figures.
//
// Assertions:
//   - the table path actually engaged: the table flight differs meaningfully
//     from the constant run (apogee differs by > 500 m);
//   - physical direction: the table's low subsonic drag (cd ~ 0.02-0.08)
//     versus the constant cd = 0.25 gives a HIGHER table apogee, a HIGHER
//     burnout velocity, and a HIGHER max-Q than the constant run. The max-Q
//     direction may seem counterintuitive: it peaks during boost (~5.45 s),
//     where the lower-drag table rocket is ~4% faster and the V^2 term of
//     q = 0.5*rho*V^2 dominates the smaller cd.
//   - flight sanity holds for the table run (apogee in a wide band,
//     burnout V in [600, 870] m/s, lateral drift < 1 m).
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>
#include <strikeengine/models/physics/atmosphere/ISA1976.hpp>
#include <strikeengine/models/physics/aerodynamics/CoefficientTable.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace StrikeEngine::Kernel;
using namespace StrikeEngine::Models;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLatRad = 28.5 * kPi / 180.0;
constexpr double kG0 = 9.80665;

constexpr double kRefArea = kPi * 0.2 * 0.2;
constexpr double kRefLength = 5.0;
constexpr double kMassInitial = 500.0;
constexpr double kMassDry = 350.0;
constexpr double kIxx = 10.0;
constexpr double kIyy = 1046.7;
constexpr double kIzz = 1046.7;

constexpr double kThrust = 60000.0;
constexpr double kPropellant = 150.0;
constexpr double kVacuumIsp = 250.0;
constexpr double kSeaLvlIsp = 220.0;

int failures = 0;
void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

VehicleConfig makeVehicleConfig(bool useTables) {
    VehicleConfig cfg;
    cfg.massDry = kMassDry;
    cfg.Ixx = kIxx;
    cfg.Iyy = kIyy;
    cfg.Izz = kIzz;
    cfg.aero.referenceArea = kRefArea;
    cfg.aero.referenceLength = kRefLength;
    cfg.aero.cd = 0.25;      // slender body (constant run; fallback in table run)
    cfg.aero.clAlpha = 2.0;
    cfg.aero.clFin = 1.5;
    cfg.aero.clMax = 1.8;
    if (useTables) {
        // sa_missile_mk1 coefficient tables.
        cfg.aero.tables.machBreakpoints = {0.0, 0.5, 0.8, 1.2, 2.0, 4.0};
        cfg.aero.tables.aoaBreakpointsRad = {0.0, 0.087, 0.174, 0.261};
        cfg.aero.tables.clTable = {
            {0.0, 0.5, 1.0, 1.2},
            {0.0, 0.55, 1.1, 1.3},
            {0.0, 0.45, 0.9, 1.1},
            {0.0, 0.3, 0.6, 0.8},
            {0.0, 0.2, 0.4, 0.6},
            {0.0, 0.15, 0.3, 0.45}
        };
        cfg.aero.tables.cdTable = {
            {0.02, 0.03, 0.05, 0.08},
            {0.02, 0.035, 0.055, 0.09},
            {0.05, 0.06, 0.08, 0.12},
            {0.08, 0.09, 0.12, 0.18},
            {0.06, 0.07, 0.10, 0.15},
            {0.04, 0.05, 0.08, 0.12}
        };
    }
    StageConfig stage;
    stage.thrustCurve = { {0.0, 60000.0}, {6.0, 60000.0}, {6.1, 0.0}, {100.0, 0.0} };
    stage.vacuumIsp = kVacuumIsp;
    stage.seaLevelIsp = kSeaLvlIsp;
    stage.propellantMassKg = kPropellant;
    stage.dryMassKg = 0.0;
    cfg.propulsion.stages.push_back(stage);
    return cfg;
}

EnvironmentConfig makeEnvironment() {
    EnvironmentConfig env;
    env.earth.useWgs84Gravity = true;
    env.earth.referenceLatitudeRad = kLatRad;
    return env;
}

struct FlightKeyPoints {
    double firstStepAccel = 0.0;
    double floorTime = -1.0;
    double floorAlt = 0.0;
    double floorSpeed = 0.0;
    double cutTime = -1.0;
    double cutAlt = 0.0;
    double cutSpeed = 0.0;
    double cutMass = 0.0;
    double maxQ = 0.0;
    double maxQTime = 0.0;
    double maxQAlt = 0.0;
    double maxQSpeed = 0.0;
    double apogee = 0.0;
    double apogeeTime = 0.0;
    double apogeeSpeed = 0.0;
    double apogeeMass = 0.0;
    double maxLateralSq = 0.0;
    double maxSpeed = 0.0;
};

FlightKeyPoints runFlight(SimulationKernel& kernel, PhysicsId id) {
    auto& phys = kernel.getPhysics();
    ISA1976 atmos;
    constexpr double dt = 0.01;
    constexpr int maxSteps = 20000;

    FlightKeyPoints kp;
    double prevMass = kMassInitial;
    double prevTime = 0.0;
    double prevAz = 0.0;
    double prevAlt = 0.0;
    double prevSpeed = 0.0;
    bool floorFound = false;
    bool cutFound = false;

    for (int step = 1; step <= maxSteps; ++step) {
        kernel.step(dt);
        const double t = step * dt;
        const double m = phys.mass[id];
        const double h = phys.pz[id];
        const double az = phys.az[id];
        const double V = std::sqrt(phys.vx[id] * phys.vx[id] +
                                   phys.vy[id] * phys.vy[id] +
                                   phys.vz[id] * phys.vz[id]);
        const double rho = atmos.evaluate(h).density;
        const double q = 0.5 * rho * V * V;

        if (step == 1) {
            kp.firstStepAccel = az;
        }

        if (q > kp.maxQ) {
            kp.maxQ = q;
            kp.maxQTime = t;
            kp.maxQAlt = h;
            kp.maxQSpeed = V;
        }
        kp.maxSpeed = std::max(kp.maxSpeed, V);
        kp.maxLateralSq = std::max(kp.maxLateralSq,
                                   phys.px[id] * phys.px[id] +
                                   phys.py[id] * phys.py[id]);

        if (!floorFound && m <= kMassDry + 1e-6) {
            const double frac = (kMassDry - prevMass) / (m - prevMass);
            kp.floorTime = prevTime + std::clamp(frac, 0.0, 1.0) * dt;
            kp.floorAlt = h;
            kp.floorSpeed = V;
            floorFound = true;
        }

        if (!cutFound && t > 5.0 && az <= 0.0 && prevAz > 0.0) {
            const double f = std::clamp(prevAz / (prevAz - az), 0.0, 1.0);
            kp.cutTime = prevTime + f * dt;
            kp.cutAlt = prevAlt + f * (h - prevAlt);
            kp.cutSpeed = prevSpeed + f * (V - prevSpeed);
            kp.cutMass = m;
            cutFound = true;
        }

        if (h > kp.apogee) {
            kp.apogee = h;
            kp.apogeeTime = t;
            kp.apogeeSpeed = V;
            kp.apogeeMass = m;
        }

        if (cutFound && t > kp.cutTime + 0.1 && phys.vz[id] <= 0.0)
            break;

        prevMass = m;
        prevTime = t;
        prevAz = az;
        prevAlt = h;
        prevSpeed = V;
    }
    return kp;
}

} // namespace

int main() {
    std::printf("=== rocket_mvp_tables: constant-coefficient vs table aero ===\n");

    auto run = [&](bool useTables) {
        SimulationKernel kernel;
        kernel.setRandomSeed(0xACE5u);
        kernel.setEnvironment(makeEnvironment());

        VehicleInitState rocket{};
        rocket.px = rocket.py = rocket.pz = 0.0;
        rocket.vx = rocket.vy = rocket.vz = 0.0;
        rocket.qw = std::cos(-kPi / 4.0);
        rocket.qx = 0.0;
        rocket.qy = std::sin(-kPi / 4.0);
        rocket.qz = 0.0;
        rocket.wx = rocket.wy = rocket.wz = 0.0;
        rocket.mass = kMassInitial;

        const auto rid = kernel.createVehicle(rocket, makeVehicleConfig(useTables));
        return runFlight(kernel, rid);
    };

    const FlightKeyPoints cst = run(false);
    const FlightKeyPoints tbl = run(true);

    // ---- Comparison table ----
    const double gLat = EarthModel::normalGravity(kLatRad);
    std::printf("\n  %-14s %14s %14s\n", "metric", "constant", "tables");
    std::printf("  %-14s %14.1f %14.1f   m/s^2 (T0 accel)\n",
                "T0 accel", cst.firstStepAccel, tbl.firstStepAccel);
    std::printf("  %-14s %14.0f %14.0f   Pa (max dynamic pressure)\n",
                "max-Q", cst.maxQ, tbl.maxQ);
    std::printf("  %-14s %14.2f %14.2f   s\n", "max-Q time", cst.maxQTime, tbl.maxQTime);
    std::printf("  %-14s %14.2f %14.2f   s (burnout / end of boost)\n",
                "burnout t", cst.cutTime, tbl.cutTime);
    std::printf("  %-14s %14.1f %14.1f   m/s (burnout V)\n",
                "burnout V", cst.cutSpeed, tbl.cutSpeed);
    std::printf("  %-14s %14.1f %14.1f   m (apogee)\n",
                "apogee", cst.apogee, tbl.apogee);
    std::printf("  %-14s %14.4f %14.4f   m (max lateral drift)\n",
                "lat drift",
                std::sqrt(cst.maxLateralSq), std::sqrt(tbl.maxLateralSq));

    const double dApogee = tbl.apogee - cst.apogee;
    std::printf("\n  delta apogee (tables - constant) = %+.1f m\n", dApogee);
    std::printf("  delta max-Q  (tables - constant) = %+.1f Pa\n", tbl.maxQ - cst.maxQ);

    // ---- Assertions ----
    // (1) Table path engaged: the two flights differ meaningfully.
    check(std::fabs(dApogee) > 500.0,
          "table path engaged: apogee differs from constant run by > 500 m");
    // (2) Physical direction, from the measured deltas (delta apogee +7601 m,
    //     delta burnout V +26.8 m/s). The table's low subsonic drag reduces
    //     drag losses, so the table run flies both HIGHER and FASTER at
    //     burnout. max-Q is also HIGHER, not lower: it peaks during boost
    //     (~5.45 s), where the lower-drag rocket's much higher velocity
    //     dominates dynamic pressure (q = 0.5*rho*V^2) despite the smaller cd.
    check(tbl.apogee > cst.apogee,
          "lower table drag raises apogee above the constant run");
    check(tbl.cutSpeed > cst.cutSpeed,
          "lower table drag raises burnout velocity above the constant run");
    check(tbl.maxQ > cst.maxQ,
          "lower table drag raises peak dynamic pressure (higher boost velocity dominates cd)");
    // (3) Flight sanity for the table run.
    check(tbl.apogee >= 15000.0 && tbl.apogee <= 90000.0,
          "table-run apogee in a physically-plausible band");
    check(tbl.cutSpeed >= 600.0 && tbl.cutSpeed <= 870.0,
          "table-run burnout V in [600, 870] m/s");
    check(tbl.maxLateralSq < 1.0,
          "table-run vertical launch stays vertical (lateral drift < 1 m)");
    check(tbl.cutTime >= 5.39 - 0.05 && tbl.cutTime <= 6.13 + 0.05,
          "table-run end-of-boost time within the Isp band [5.39, 6.13] s");

    std::printf("\n%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
