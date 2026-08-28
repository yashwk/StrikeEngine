// WGS84 rocket-launch MVP verification.
//
// Simulates a single-stage sounding rocket launched vertically from the pad
// (local ENU, WGS84 Somigliana normal gravity, ISA-1976 atmosphere,
// constant-coefficient aero, pressure-interpolated Isp, fuel-limited burnout,
// RK4 at dt = 0.01 s, GuidanceMode::None = purely ballistic) and cross-checks
// the simulated physics against hand-computed expectations:
//   - Somigliana normal gravity at the reference latitude (28.5 N)
//   - T0 thrust and mass flow
//   - initial acceleration
//   - fuel-limited burnout time and mass
//   - ideal rocket-equation Delta-v vs simulated burnout velocity
//   - apogee in a plausible band and below the no-drag ballistic bound
//   - max dynamic pressure and ISA-1976 sea-level density
//   - lateral drift: the vertical launch must stay vertical
// plus a second short case launched at 70 degrees elevation that must fly
// downrange with an arcing trajectory below the vertical-launch apogee.
//
// Burnout is reported at two instants:
//   - "floor crossing": the first moment the integrated mass reaches the
//     dry-mass floor (350 kg) and propellant is nominally exhausted
//     (thrust is exactly zero there: T = mdot*Isp*g0 with mdot -> 0);
//   - "thrust cutoff" / end of boost: the instant the motor stops
//     accelerating the vehicle, detected as the vertical acceleration
//     rolling over from positive to negative.
// With a correct engine the end of boost is no later than fuel exhaustion
// (thrust tapers with remaining fuel). The free-thrust-tail bug (full thrust
// on < 1 g of fuel) instead kept az ~ +139 m/s^2 for ~0.08 s AFTER the floor
// was reached, worth ~30 m/s of spurious Delta-v.
//
// Note on mass flow: the propulsion model interpolates Isp linearly between
// sea-level (220 s) and vacuum (250 s) by ambient pressure. At the pad the
// engine therefore flows T/(220*g0) = 27.81 kg/s, rising to the vacuum value
// T/(250*g0) = 24.47 kg/s as the rocket climbs. The hand-computed checks below
// use that documented behavior (the spec's 24.47 kg/s / 6.13 s figures are the
// vacuum-Isp bounds and are printed for reference).
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>
#include <strikeengine/models/physics/atmosphere/ISA1976.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace StrikeEngine::Kernel;
using namespace StrikeEngine::Models;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLatRad = 28.5 * kPi / 180.0;   // reference latitude 28.5 N
constexpr double kG0 = 9.80665;

// --- Vehicle shape: 5 m long, 0.4 m diameter, 500 kg (350 dry + 150 fuel) ---
constexpr double kRefArea = kPi * 0.2 * 0.2;      // pi*(0.2 m)^2 = 0.1256637 m^2
constexpr double kRefLength = 5.0;
constexpr double kMassInitial = 500.0;
constexpr double kMassDry = 350.0;
// Inertia: Ixx = m*r^2/2 = 500*0.2^2/2 = 10 kg*m^2 (roll)
//         Iyy = Izz = m*(L^2/12 + r^2/4) = 500*(25/12 + 0.04/4) = 1046.7 kg*m^2
constexpr double kIxx = 10.0;
constexpr double kIyy = 1046.7;
constexpr double kIzz = 1046.7;

// --- Motor: constant 60 kN, fuel-limited (150 kg), T0 ignition ---
constexpr double kThrust = 60000.0;
constexpr double kPropellant = 150.0;
constexpr double kVacuumIsp = 250.0;
constexpr double kSeaLvlIsp = 220.0;

// Hand-computed mass flows and burnout-time bounds.
constexpr double kFlowVacuum = kThrust / (kVacuumIsp * kG0);   // 24.47 kg/s
constexpr double kFlowSeaLvl = kThrust / (kSeaLvlIsp * kG0);   // 27.81 kg/s
constexpr double kBurnLow = kPropellant / kFlowSeaLvl;         // 5.39 s
constexpr double kBurnHigh = kPropellant / kFlowVacuum;        // 6.13 s

int failures = 0;
void check(bool ok, const char* what)
{
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

VehicleConfig makeVehicleConfig()
{
    VehicleConfig cfg;
    cfg.massDry = kMassDry;
    cfg.Ixx = kIxx;
    cfg.Iyy = kIyy;
    cfg.Izz = kIzz;
    cfg.aero.referenceArea = kRefArea;
    cfg.aero.referenceLength = kRefLength;
    cfg.aero.cd = 0.25;      // slender body
    cfg.aero.clAlpha = 2.0;  // modest lift slope
    cfg.aero.clFin = 1.5;    // fin authority
    cfg.aero.clMax = 1.8;    // stall limit
    StageConfig stage;
    stage.thrustCurve = { {0.0, 60000.0}, {6.0, 60000.0}, {6.1, 0.0}, {100.0, 0.0} };
    stage.vacuumIsp = kVacuumIsp;
    stage.seaLevelIsp = kSeaLvlIsp;
    stage.propellantMassKg = kPropellant;
    stage.dryMassKg = 0.0;
    cfg.propulsion.stages.push_back(stage);
    return cfg;
}

EnvironmentConfig makeEnvironment()
{
    EnvironmentConfig env;
    env.earth.useWgs84Gravity = true;
    env.earth.referenceLatitudeRad = kLatRad;
    return env;
}

struct FlightKeyPoints {
    // measured at the end of the first 0.01 s step
    double firstStepThrust = 0.0;
    double firstStepFlow = 0.0;
    double firstStepAccel = 0.0;
    // fuel-exhaustion floor crossing (mass reaches the dry-mass floor)
    double floorTime = -1.0;
    double floorAlt = 0.0;
    double floorSpeed = 0.0;
    double floorMass = 0.0;
    double floorQ = 0.0;
    // true thrust cutoff (vertical acceleration rolls over to negative)
    double cutTime = -1.0;
    double cutAlt = 0.0;
    double cutSpeed = 0.0;
    double cutMass = 0.0;
    double cutQ = 0.0;
    // max dynamic pressure
    double maxQ = 0.0;
    double maxQTime = 0.0;
    double maxQAlt = 0.0;
    double maxQSpeed = 0.0;
    double maxQMass = 0.0;
    // apogee
    double apogee = 0.0;
    double apogeeTime = 0.0;
    double apogeeSpeed = 0.0;
    double apogeeMass = 0.0;
    double downrangeAtApogee = 0.0;
    double maxLateralSq = 0.0;
    double maxSpeed = 0.0;
};

FlightKeyPoints runFlight(SimulationKernel& kernel, PhysicsId id)
{
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
            kp.firstStepFlow = (kMassInitial - m) / dt;
            kp.firstStepAccel = az;
            const double gLatHere = EarthModel::normalGravity(kLatRad, h);
            kp.firstStepThrust = (az + gLatHere) * m;
        }

        if (q > kp.maxQ) {
            kp.maxQ = q;
            kp.maxQTime = t;
            kp.maxQAlt = h;
            kp.maxQSpeed = V;
            kp.maxQMass = m;
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
            kp.floorMass = m;
            kp.floorQ = q;
            floorFound = true;
        }

        if (!cutFound && t > 5.0 && az <= 0.0 && prevAz > 0.0) {
            // End of boost: interpolate the az zero-crossing between the
            // last accelerating step (az > 0) and this decelerating step
            // (az < 0). In the buggy engine this lagged fuel exhaustion by
            // ~0.08 s (free-thrust tail); with the fix it is no later than
            // it (thrust tapers to zero as fuel runs out).
            const double f = std::clamp(prevAz / (prevAz - az), 0.0, 1.0);
            kp.cutTime = prevTime + f * dt;
            kp.cutAlt = prevAlt + f * (h - prevAlt);
            kp.cutSpeed = prevSpeed + f * (V - prevSpeed);
            kp.cutMass = m;
            kp.cutQ = q;
            cutFound = true;
        }

        if (h > kp.apogee) {
            kp.apogee = h;
            kp.apogeeTime = t;
            kp.apogeeSpeed = V;
            kp.apogeeMass = m;
            kp.downrangeAtApogee = phys.px[id];
        }

        // Stop once the vehicle is clearly descending after the boost phase.
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

int main()
{
    std::printf("=== rocket_mvp: WGS84 single-stage launch cross-checked by hand ===\n");
    std::printf("  vehicle: 5 m x 0.4 m, Aref=%.7f m^2, Lref=%.1f m, m0=%.0f kg, dry=%.0f kg\n",
                kRefArea, kRefLength, kMassInitial, kMassDry);
    std::printf("  inertia: Ixx=%.1f kg*m^2 (m*r^2/2), Iyy=Izz=%.1f kg*m^2 (m*(L^2/12+r^2/4))\n",
                kIxx, kIyy);
    std::printf("  motor: 60 kN constant, Isp vac=%.0f s / sl=%.0f s, propellant=%.0f kg\n",
                kVacuumIsp, kSeaLvlIsp, kPropellant);
    std::printf("  env: WGS84 gravity at %.1f N lat, ISA-1976, dt=0.01 s, RK4, GuidanceMode::None\n\n",
                kLatRad * 180.0 / kPi);

    // ---- 0. WGS84 datum properties --------------------------------------
    std::printf("-- WGS84 datum properties --\n");
    const double gEq = EarthModel::normalGravity(0.0);
    const double gLat = EarthModel::normalGravity(kLatRad);
    const double gPole = EarthModel::normalGravity(kPi / 2.0);
    std::printf("  earth rotation rate omega_e = %.6e rad/s\n",
                EarthModel::earthRotationRateRadPerSec);
    std::printf("  Somigliana normal gravity: equator %.6f, 28.5N %.6f, pole %.6f m/s^2\n",
                gEq, gLat, gPole);
    check(gEq < gLat && gLat < gPole,
          "Somigliana gravity increases monotonically with latitude");

    const GeodeticCoordinate launchGeo{kLatRad, -80.5 * kPi / 180.0, 0.0};
    const auto launchEcef = EarthModel::geodeticToEcef(launchGeo);
    const auto roundTrip = EarthModel::ecefToGeodetic(launchEcef);
    check(std::abs(roundTrip.latitudeRad - launchGeo.latitudeRad) < 1e-11 &&
              std::abs(roundTrip.longitudeRad - launchGeo.longitudeRad) < 1e-11 &&
              std::abs(roundTrip.altitudeM - launchGeo.altitudeM) < 1e-5,
          "geodetic <-> ECEF round trip is exact for the launch point");

    // ---- 1. T0 vertical gravity vs Somigliana (coasting probe) ----------
    std::printf("\n-- T0 gravity check at %.1f N, zero velocity --\n", kLatRad * 180.0 / kPi);
    {
        SimulationKernel probeKernel;
        probeKernel.setEnvironment(makeEnvironment());
        // Probe sits at altitude (a rest-on-pad entity is culled as a ground
        // impact by the event system); one step later it is still far above
        // the terrain, so az is exactly the WGS84 normal gravity.
        constexpr double probeAlt = 1000.0;
        VehicleInitState probe{};
        probe.pz = probeAlt;
        probe.qw = 1.0;
        probe.mass = 100.0;
        VehicleConfig probeCfg;   // coasting: no motor, no aero
        probeCfg.aero.referenceArea = 0.0;
        probeCfg.aero.cd = 0.0;
        probeCfg.aero.clAlpha = 0.0;
        probeCfg.aero.clFin = 0.0;
        probeKernel.createVehicle(probe, probeCfg);
        probeKernel.step(0.01);
        const double gSim = -probeKernel.getPhysics().az[0];
        const double gAtAlt = EarthModel::normalGravity(kLatRad, probeAlt);
        std::printf("  simulated vertical gravity %.6f m/s^2 vs Somigliana at %.0f m %.6f m/s^2 (%.4f%%)\n",
                    gSim, probeAlt, gAtAlt, 100.0 * std::abs(gSim - gAtAlt) / gAtAlt);
        // Tight tolerance: the flat-earth 9.80665 fallback is ~0.18% off from
        // 9.789 at this latitude and must be caught.
        check(std::abs(gSim - gAtAlt) / gAtAlt < 0.001,
              "simulated vertical gravity matches Somigliana normal gravity (<0.1%)");
    }

    // ---- 2. Vertical launch ---------------------------------------------
    std::printf("\n-- Vertical launch simulation --\n");
    SimulationKernel kernel;
    kernel.setRandomSeed(0xACE5u);
    kernel.setEnvironment(makeEnvironment());

    VehicleInitState rocket{};
    rocket.px = rocket.py = rocket.pz = 0.0;
    rocket.vx = rocket.vy = rocket.vz = 0.0;
    // Vertical attitude: body +X (nose) -> world +Z (up), i.e. 90 deg pitch
    // from level. Test-suite body->world quaternion convention: q = [qw,qx,qy,qz].
    rocket.qw = std::cos(-kPi / 4.0);
    rocket.qx = 0.0;
    rocket.qy = std::sin(-kPi / 4.0);
    rocket.qz = 0.0;
    rocket.wx = rocket.wy = rocket.wz = 0.0;
    rocket.mass = kMassInitial;

    const auto rid = kernel.createVehicle(rocket, makeVehicleConfig());
    const FlightKeyPoints kp = runFlight(kernel, rid);

    // Cross-check 1: T0 thrust and mass flow
    std::printf("\n-- Cross-checks (vertical launch) --\n");
    std::printf("  T0 thrust measured %.2f N (expect %.0f N)\n",
                kp.firstStepThrust, kThrust);
    check(std::abs(kp.firstStepThrust - kThrust) / kThrust < 0.01,
          "T0 thrust = 60000 N within 1%");
    std::printf("  T0 mass flow measured %.4f kg/s (sea-level Isp model %.4f, vacuum-Isp ref %.4f)\n",
                kp.firstStepFlow, kFlowSeaLvl, kFlowVacuum);
    check(std::abs(kp.firstStepFlow - kFlowSeaLvl) / kFlowSeaLvl < 0.01,
          "T0 mass flow matches pressure-interpolated Isp (220 s at sea level) within 1%");

    // Cross-check 2: initial acceleration
    const double aPred = kThrust / kMassInitial - gLat;   // 110.2 m/s^2
    std::printf("  initial acceleration measured %.3f m/s^2 (hand-computed %.3f m/s^2)\n",
                kp.firstStepAccel, aPred);
    check(std::abs(kp.firstStepAccel - aPred) / aPred < 0.05,
          "initial vertical acceleration ~ T/m - g_lat within 5%");

    // Cross-check 3: burnout time and mass (end of boost / thrust cutoff)
    std::printf("  floor crossing: t=%.3f s, h=%.0f m, V=%.1f m/s, m=%.2f kg (thrust exactly zero)\n",
                kp.floorTime, kp.floorAlt, kp.floorSpeed, kp.floorMass);
    std::printf("  thrust cutoff : t=%.3f s, h=%.0f m, V=%.1f m/s, m=%.2f kg (accel rolls over)\n",
                kp.cutTime, kp.cutAlt, kp.cutSpeed, kp.cutMass);
    std::printf("  cutoff bounds: sea-level-Isp lower %.3f s, vacuum-Isp upper %.3f s\n",
                kBurnLow, kBurnHigh);
    check(kp.cutTime >= kBurnLow - 0.05 && kp.cutTime <= kBurnHigh + 0.05,
          "end-of-boost time within the pressure-interpolated-Isp band [5.39, 6.13] s");
    check(std::abs(kp.cutMass - kMassDry) < 1.0,
          "mass at end of boost ~ 350 kg within 1 kg");
    check(kp.cutTime - kp.floorTime < 0.02,
          "no free-thrust tail: boost ends by (not after) fuel exhaustion");

    // Cross-check 4: burnout velocity vs ideal rocket equation
    const double dvIdeal = kVacuumIsp * kG0 * std::log(kMassInitial / kMassDry);
    const double loss = dvIdeal - kp.cutSpeed;
    std::printf("  ideal Delta-v (vacuum Isp) %.1f m/s; simulated cutoff V %.1f m/s\n",
                dvIdeal, kp.cutSpeed);
    std::printf("  implied gravity+drag+Isp loss %.1f m/s (gravity loss ~ g*t_burn ~ %.1f m/s)\n",
                loss, gLat * kp.cutTime);
    check(kp.cutSpeed >= 600.0 && kp.cutSpeed <= 870.0,
          "cutoff velocity in the physically-sound band [600, 870] m/s");

    // Cross-check 5: apogee in a plausible band AND below the no-drag bound
    const double noDragBound = kp.cutSpeed * kp.cutSpeed / (2.0 * gLat) + kp.cutAlt;
    std::printf("  apogee h=%.1f m (%.2f km) at t=%.2f s\n",
                kp.apogee, kp.apogee / 1000.0, kp.apogeeTime);
    std::printf("  no-drag ballistic bound V^2/(2g)+h_cut = %.1f km\n", noDragBound / 1000.0);
    check(kp.apogee >= 15000.0 && kp.apogee <= 45000.0,
          "vertical-launch apogee in plausible band [15, 45] km");
    check(kp.apogee < noDragBound,
          "apogee below the no-drag ballistic bound (drag binds the trajectory)");

    // Lateral drift: a symmetric vertical launch must stay vertical.
    std::printf("  max lateral drift sqrt(px^2+py^2) = %.3f m\n",
                std::sqrt(kp.maxLateralSq));
    check(kp.maxLateralSq < 1.0,
          "vertical launch stays vertical (lateral drift < 1 m through apogee)");

    // Cross-check 6: atmosphere
    const ISA1976 atmos;
    const double rho0 = atmos.evaluate(0.0).density;
    std::printf("  ISA-1976 sea-level density %.4f kg/m^3 (reference 1.225)\n", rho0);
    check(std::abs(rho0 - 1.225) / 1.225 < 0.01,
          "ISA-1976 sea-level density ~ 1.225 kg/m^3 within 1%");
    std::printf("  max dynamic pressure q=%.0f Pa at t=%.2f s, h=%.0f m, V=%.1f m/s\n",
                kp.maxQ, kp.maxQTime, kp.maxQAlt, kp.maxQSpeed);
    check(kp.maxQ > 100000.0 && kp.maxQ < 400000.0,
          "max dynamic pressure in physically-plausible band (~250 kPa at ~2 km)");

    check(kp.maxSpeed < 2000.0, "no numeric blowup during boost + coast");

    // Flight summary table (chronological)
    std::printf("\n-- Flight summary (vertical launch) --\n");
    std::printf("  %-9s %-9s %-11s %-9s %-10s %-11s\n",
                "phase", "time(s)", "alt(m)", "V(m/s)", "mass(kg)", "q(Pa)");
    std::printf("  %-9s %-9.2f %-11.1f %-9.1f %-10.1f %-11.1f\n",
                "T0", 0.0, 0.0, 0.0, kMassInitial, 0.0);
    std::printf("  %-9s %-9.2f %-11.1f %-9.1f %-10.1f %-11.0f\n",
                "burnout", kp.cutTime, kp.cutAlt, kp.cutSpeed, kp.cutMass, kp.cutQ);
    std::printf("  %-9s %-9.2f %-11.1f %-9.1f %-10.1f %-11.0f\n",
                "max-Q", kp.maxQTime, kp.maxQAlt, kp.maxQSpeed, kp.maxQMass, kp.maxQ);
    std::printf("  %-9s %-9.2f %-11.1f %-9.1f %-10.1f %-11.1f\n",
                "apogee", kp.apogeeTime, kp.apogee, kp.apogeeSpeed,
                kp.apogeeMass, 0.0);
    std::printf("  (max-Q is coincident with burnout: velocity peaks as thrust\n");
    std::printf("   tapers, then q decays on the coast)\n");

    // ---- 3. 70-degree elevation launch ----------------------------------
    std::printf("\n-- 70-degree elevation launch --\n");
    SimulationKernel kernel70;
    kernel70.setRandomSeed(0xACE6u);
    kernel70.setEnvironment(makeEnvironment());

    VehicleInitState r70{};
    r70.px = r70.py = r70.pz = 0.0;
    r70.vx = r70.vy = r70.vz = 0.0;
    // Nose at 70 deg elevation from horizontal (toward +X/east): rotation
    // about the world Y axis by -70 deg.
    r70.qw = std::cos(70.0 * kPi / 180.0 / 2.0);
    r70.qx = 0.0;
    r70.qy = -std::sin(70.0 * kPi / 180.0 / 2.0);
    r70.qz = 0.0;
    r70.wx = r70.wy = r70.wz = 0.0;
    r70.mass = kMassInitial;

    const auto r70id = kernel70.createVehicle(r70, makeVehicleConfig());
    const FlightKeyPoints kp70 = runFlight(kernel70, r70id);

    std::printf("  downrange at apogee %.1f m, apogee %.1f m (vertical apogee %.1f m)\n",
                kp70.downrangeAtApogee, kp70.apogee, kp.apogee);
    check(kp70.downrangeAtApogee > 1000.0,
          "70-deg launch flies downrange (arcing trajectory)");
    check(kp70.apogee > 5000.0 && kp70.apogee < kp.apogee,
          "70-deg apogee is a real arc below the vertical-launch apogee");

    std::printf("\n%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
