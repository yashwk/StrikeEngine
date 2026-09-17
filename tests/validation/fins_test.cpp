// Geometric fin model verification (RocketPy trapezoidal / elliptical /
// free-form port). Cross-checks the FinsGeometry builder against hand-computed
// RocketPy values, the Mach behavior of the lift slope, the stability/roll sign
// conventions, JSON round-trip, and a ballistic flight on the rocket_mvp
// vehicle with tail fins.
#include <strikeengine/models/physics/aerodynamics/FinsModel.hpp>
#include <strikeengine/models/physics/aerodynamics/AeroModel.hpp>
#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/VehicleConfig.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <strikeengine/kernel/config/ConfigSerialization.hpp>
#include <strikeengine/models/physics/atmosphere/ISA1976.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
#include <array>

using namespace StrikeEngine::Models;
using namespace StrikeEngine::Kernel;

namespace {

int failures = 0;
void check(bool ok, const char* what)
{
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

constexpr double kPi = 3.14159265358979323846;
const double kRefAreaRadius01 = kPi * 0.1 * 0.1;  // radius 0.1 m reference area

} // namespace

static void geometryChecks()
{
    std::printf("-- geometry (radius 0.1 m reference area) --\n");

    // --- Trapezoidal reference fin ---
    // n=4, root 0.5, tip 0.35, span 0.25, sweep 0.15.
    {
        std::string err;
        auto g = buildFinsGeometry(FinShape::Trapezoidal, 4,
            0.5, 0.35, 0.25, 0.15, 0.0, 0.0, {}, kRefAreaRadius01, &err);
        check(g != nullptr, "trapezoidal fin builds");
        if (!g) return;

        const double root = 0.5, tip = 0.35, span = 0.25, sweep = 0.15, r = 0.1;
        const double Yr = root + tip;
        const double Af = Yr * span / 2.0;
        const double AR = 2.0 * span * span / Af;
        const double gammaC = std::atan((sweep + 0.5 * tip - 0.5 * root) / span);
        const double Yma = (span / 3.0) * (root + 2.0 * tip) / Yr;
        const double cpz = (sweep / 3.0) * ((root + 2.0 * tip) / Yr)
                         + (1.0 / 6.0) * (Yr - root * tip / Yr);
        const double tau = (span + r) / r;
        const double lift = 1.0 + 1.0 / tau;

        check(std::abs(g->Af - Af) < 1e-12, "trapezoidal Af");
        check(std::abs(g->AR - AR) < 1e-12, "trapezoidal AR == 2 span^2 / Af");
        check(std::abs(g->gammaC - gammaC) < 1e-12, "trapezoidal gamma_c");
        check(std::abs(g->Yma - Yma) < 1e-12, "trapezoidal Yma");
        check(std::abs(g->cpz - cpz) < 1e-12, "trapezoidal cpz");
        check(std::abs(g->liftInterferenceFactor - lift) < 1e-12, "trapezoidal 1+1/tau");
        check(std::abs(g->finNumCorrection - 2.0) < 1e-12, "4 fins -> correction 2.0");
        check(g->cpLeverArmM == 0.0 - cpz, "cp lever arm = position - cpz");
    }

    // --- Elliptical ---
    {
        auto g = buildFinsGeometry(FinShape::Elliptical, 4,
            0.5, 0.0, 0.25, 0.0, 0.0, 0.0, {}, kRefAreaRadius01, nullptr);
        check(g != nullptr, "elliptical fin builds");
        if (!g) return;
        const double root = 0.5, span = 0.25;
        const double Af = kPi * root * span / 4.0;
        const double Yma = span / (3.0 * kPi) * std::sqrt(9.0 * kPi * kPi - 64.0);
        check(std::abs(g->Af - Af) < 1e-12, "elliptical Af = pi*root*span/4");
        check(std::abs(g->cpz - 0.288 * root) < 1e-12, "elliptical cpz = 0.288*root");
        check(std::abs(g->gammaC) < 1e-15, "elliptical gamma_c = 0");
        check(std::abs(g->Yma - Yma) < 1e-12, "elliptical Yma");
    }

    // --- Free-form rectangular ---
    {
        std::vector<std::array<double, 2>> pts = { {0.0, 0.0}, {0.0, 0.2},
                                                   {0.5, 0.2}, {0.5, 0.0} };
        auto g = buildFinsGeometry(FinShape::FreeForm, 3,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, pts, kRefAreaRadius01, nullptr);
        check(g != nullptr, "free-form fin builds");
        if (!g) return;
        const double root = 0.5, span = 0.2, Af = 0.1;
        check(std::abs(g->rootChord - root) < 1e-12, "freeform root chord = 0.5");
        check(std::abs(g->span - span) < 1e-12, "freeform span = 0.2");
        check(std::abs(g->Af - Af) < 1e-12, "freeform Af = 0.1");
        check(std::abs(g->AR - 2.0 * span * span / Af) < 1e-12, "freeform AR");
        check(std::abs(g->cpz - 0.125) < 1e-6, "freeform rectangular cpz ~ 0.125");
    }
}

static void machAndSignChecks()
{
    std::printf("\n-- Mach behavior + sign conventions --\n");

    auto trapezoid = buildFinsGeometry(FinShape::Trapezoidal, 4,
        0.5, 0.35, 0.25, 0.15, 0.0, 0.0, {}, kRefAreaRadius01, nullptr);
    if (!trapezoid) return;
    auto tail = buildFinsGeometry(FinShape::Trapezoidal, 4,
        0.5, 0.35, 0.25, 0.15, -1.5, 0.0, {}, kRefAreaRadius01, nullptr);
    auto canted = buildFinsGeometry(FinShape::Trapezoidal, 4,
        0.5, 0.35, 0.25, 0.15, 0.0, 1.0, {}, kRefAreaRadius01, nullptr);

    check(trapezoid->clAlpha(0.5) > trapezoid->clAlpha(0.0),
          "clAlpha rises subsonically with Mach");
    check(trapezoid->clAlpha(2.0) < trapezoid->clAlpha(0.5),
          "clAlpha falls supersonically with Mach");
    check(trapezoid->clAlpha(0.0) > 0.0, "clAlpha positive");

    // Stability: tail fins (negative lever arm) give a restoring pitch moment
    // for positive alpha (nose-down => negative torque_y).
    if (tail && canted) {
        AeroParams p;
        p.referenceArea = kRefAreaRadius01;
        p.referenceLength = 0.2;
        p.cd = 0.0; p.clAlpha = 0.0; p.clFin = 0.0; p.clMax = 2.0;
        p.fins = tail;
        BasicAeroModel m;
        // alpha ~ 0.05 rad: u=100, w=5
        auto w = m.computeWrench(100.0, 0.0, 5.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 1.225, 340.0, p);
        check(w.torque_y < 0.0, "tail fins restore: +alpha -> nose-down moment");

        check(canted->rollForcingPerRad(0.5) > 0.0, "positive cant -> positive roll forcing");
        check(canted->rollDampingCoeff(0.5) > 0.0, "roll damping coefficient positive");
    }
}

static void serializationChecks()
{
    std::printf("\n-- serialization round-trip --\n");
    {
        VehicleConfig cfg;
        cfg.aero.referenceArea = 0.1256637;
        cfg.aero.fins = FinsConfig{};
        cfg.aero.fins.shape = FinShape::Trapezoidal;
        cfg.aero.fins.count = 4;
        cfg.aero.fins.rootChordM = 0.5;
        cfg.aero.fins.tipChordM = 0.35;
        cfg.aero.fins.spanM = 0.25;
        cfg.aero.fins.sweepLengthM = 0.15;
        cfg.aero.fins.positionM = -1.5;
        cfg.aero.fins.cantAngleDeg = 1.0;

        const std::string s = serializeVehicleConfig(cfg);
        VehicleConfig rt = deserializeVehicleConfig(s);
        check(rt.aero.fins.enabled(), "trapezoidal fins round-trip enabled");
        check(rt.aero.fins.shape == FinShape::Trapezoidal, "shape preserved");
        check(rt.aero.fins.count == 4 && std::abs(rt.aero.fins.rootChordM - 0.5) < 1e-12,
              "trapezoidal dimensions preserved");
        check(std::abs(rt.aero.fins.positionM + 1.5) < 1e-12 &&
              std::abs(rt.aero.fins.cantAngleDeg - 1.0) < 1e-12,
              "position + cant preserved");
    }

    {
        VehicleConfig cfg;
        cfg.aero.fins.shape = FinShape::FreeForm;
        cfg.aero.fins.count = 3;
        cfg.aero.fins.shapePoints = { {0.0, 0.0}, {0.0, 0.2}, {0.5, 0.2}, {0.5, 0.0} };
        const std::string s = serializeVehicleConfig(cfg);
        VehicleConfig rt = deserializeVehicleConfig(s);
        check(rt.aero.fins.shape == FinShape::FreeForm &&
              rt.aero.fins.shapePoints.size() == 4,
              "free-form shape_points round-trip");
    }

    {
        VehicleConfig cfg;
        const std::string s = serializeVehicleConfig(cfg);
        VehicleConfig rt = deserializeVehicleConfig(s);
        check(!rt.aero.fins.enabled(), "absent fins stay disabled");
    }
}

static void validationChecks()
{
    std::printf("\n-- validation --\n");
    std::string err;
    check(buildFinsGeometry(FinShape::Trapezoidal, 2, 0.5, 0.35, 0.25, 0.15,
                            0.0, 0.0, {}, kRefAreaRadius01, &err) == nullptr,
          "count < 3 rejected");
    check(buildFinsGeometry(FinShape::FreeForm, 3, 0, 0, 0, 0, 0, 0,
                            { {0.0, 0.0}, {0.5, 0.1} }, kRefAreaRadius01, &err) == nullptr,
          "free-form with 2 points rejected");
}

// Minimal ballistic flight on the rocket_mvp vehicle (5 m x 0.4 m, 500 kg,
// 60 kN motor) with a tail fin set; asserts the vertical launch stays
// vertical through apogee.
static double runFinFlight(const FinsConfig& fins, double& apogee, double& driftSq)
{
    SimulationKernel kernel;
    kernel.setRandomSeed(0xF1A5u);
    EnvironmentConfig env;
    env.earth.useWgs84Gravity = true;
    env.earth.referenceLatitudeRad = 28.5 * kPi / 180.0;
    kernel.setEnvironment(env);

    VehicleConfig cfg;
    cfg.massDry = 350.0;
    cfg.Ixx = 10.0; cfg.Iyy = 1046.7; cfg.Izz = 1046.7;
    cfg.aero.referenceArea = kPi * 0.2 * 0.2;
    cfg.aero.referenceLength = 5.0;
    cfg.aero.cd = 0.25;
    cfg.aero.clAlpha = 2.0;
    cfg.aero.clFin = 1.5;
    cfg.aero.clMax = 1.8;
    cfg.aero.fins = fins;
    StageConfig stage;
    stage.thrustCurve = { {0.0, 60000.0}, {6.0, 60000.0}, {6.1, 0.0}, {100.0, 0.0} };
    stage.vacuumIsp = 250.0;
    stage.seaLevelIsp = 220.0;
    stage.propellantMassKg = 150.0;
    stage.dryMassKg = 0.0;
    cfg.propulsion.stages.push_back(stage);

    VehicleInitState r{};
    r.qw = std::cos(-kPi / 4.0);
    r.qy = std::sin(-kPi / 4.0);
    r.mass = 500.0;
    const auto id = kernel.createVehicle(r, cfg);

    auto& phys = kernel.getPhysics();
    constexpr double dt = 0.01;
    apogee = 0.0;
    driftSq = 0.0;
    for (int step = 1; step <= 20000; ++step) {
        kernel.step(dt);
        const double h = phys.pz[id];
        const double d2 = phys.px[id] * phys.px[id] + phys.py[id] * phys.py[id];
        driftSq = std::max(driftSq, d2);
        apogee = std::max(apogee, h);
        if (step * dt > 7.0 && phys.vz[id] <= 0.0) break;
    }
    return 0.0;
}

static void flightChecks()
{
    std::printf("\n-- ballistic flight w/ tail fins --\n");
    FinsConfig trap;
    trap.shape = FinShape::Trapezoidal;
    trap.count = 4;
    trap.rootChordM = 0.5; trap.tipChordM = 0.35; trap.spanM = 0.25;
    trap.sweepLengthM = 0.15; trap.positionM = -1.5;

    double apogee = 0.0, driftSq = 0.0;
    runFinFlight(trap, apogee, driftSq);
    std::printf("  trapezoidal tails: apogee %.1f km, drift %.3f m\n",
                apogee / 1000.0, std::sqrt(driftSq));
    check(apogee >= 15000.0 && apogee <= 45000.0, "trapezoidal apogee in band [15,45] km");
    check(driftSq < 4.0, "trapezoidal vertical launch stays vertical (<2 m)");

    FinsConfig ellip;
    ellip.shape = FinShape::Elliptical;
    ellip.count = 4;
    ellip.rootChordM = 0.5; ellip.spanM = 0.25; ellip.positionM = -1.5;
    double apogeeE = 0.0, driftSqE = 0.0;
    runFinFlight(ellip, apogeeE, driftSqE);
    std::printf("  elliptical tails : apogee %.1f km, drift %.3f m\n",
                apogeeE / 1000.0, std::sqrt(driftSqE));
    check(apogeeE >= 15000.0 && apogeeE <= 45000.0, "elliptical apogee in band [15,45] km");
    check(driftSqE < 4.0, "elliptical vertical launch stays vertical (<2 m)");
}

static void controlPolarityChecks()
{
    std::printf("\n-- control polarity (tail fins) --\n");
    // The guidance loop is tuned to the documented+abstract convention: a
    // positive pitch deflection command produces a nose-UP moment (+torque_y)
    // and a positive yaw command produces a nose-RIGHT moment (+torque_z).
    // The geometric-fin control terms must reproduce exactly that response so
    // the closed loop has the same polarity with or without fins (a previous
    // inversion here put aft-fin vehicles into positive feedback -> hard-over
    // dive). Stability keeps its restoring sign (alpha>0 -> nose-DOWN).
    auto tail = buildFinsGeometry(FinShape::Trapezoidal, 4,
        0.5, 0.35, 0.25, 0.15, -1.5, 0.0, {}, kRefAreaRadius01, nullptr);
    if (!tail) return;
    AeroParams p;
    p.referenceArea = kRefAreaRadius01;
    p.referenceLength = 0.2;
    p.cd = 0.0; p.clAlpha = 0.0; p.clFin = 0.0; p.clMax = 2.0;
    p.fins = tail;
    BasicAeroModel m;
    const double V = 100.0;

    {   // zero alpha/beta, +pitch command -> nose-UP
        auto w = m.computeWrench(V, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.05, 0.0, 0.0, 1.225, 340.0, p);
        check(w.torque_y > 0.0, "+finPitch -> nose-UP (+torque_y)");
        auto w2 = m.computeWrench(V, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  -0.05, 0.0, 0.0, 1.225, 340.0, p);
        check(w2.torque_y < 0.0, "-finPitch -> nose-DOWN (-torque_y)");
    }
    {   // zero alpha/beta, +yaw command -> nose-RIGHT
        auto w = m.computeWrench(V, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.05, 0.0, 1.225, 340.0, p);
        check(w.torque_z > 0.0, "+finYaw -> nose-RIGHT (+torque_z)");
        auto w2 = m.computeWrench(V, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, -0.05, 0.0, 1.225, 340.0, p);
        check(w2.torque_z < 0.0, "-finYaw -> nose-LEFT (-torque_z)");
    }
    {   // stability restored beyond the control check: +alpha, no command
        auto w = m.computeWrench(V, 0.0, 5.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 1.225, 340.0, p);
        check(w.torque_y < 0.0, "stability: +alpha (nose up) -> nose-DOWN torque");
        auto w2 = m.computeWrench(V, 4.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 1.225, 340.0, p);
        check(w2.torque_z > 0.0, "stability: +beta (wind from right) -> nose-RIGHT restoring torque");
    }
    {   // a nose-up command at small positive alpha still commands nose-up
        auto w = m.computeWrench(V, 0.0, 2.0, 0.0, 0.0, 0.0,
                                 0.08, 0.0, 0.0, 1.225, 340.0, p);
        check(w.torque_y > 0.0, "nose-up command dominates the small restoring term");
    }
}

static void multiFinSetChecks()
{
    std::printf("\n-- multi-fin sets (canards + tail fins) --\n");
    // Build forward steerable canards (position +1.0 m, smaller span)
    auto canards = buildFinsGeometry(FinShape::Trapezoidal, 4,
        0.3, 0.15, 0.15, 0.10, +1.0, 0.0, {}, kRefAreaRadius01, nullptr, true);
    check(canards != nullptr, "canards build");
    check(canards->steerable == true, "canards are steerable");

    // Build aft stabilizing tail fins (position -1.5 m, larger span, fixed/passive)
    auto tails = buildFinsGeometry(FinShape::Trapezoidal, 4,
        0.5, 0.35, 0.25, 0.15, -1.5, 0.0, {}, kRefAreaRadius01, nullptr, false);
    check(tails != nullptr, "tails build");
    check(tails->steerable == false, "tails are non-steerable (passive)");

    AeroParams p;
    p.referenceArea = kRefAreaRadius01;
    p.referenceLength = 0.2;
    p.cd = 0.0; p.clAlpha = 0.0; p.clFin = 0.0; p.clMax = 2.0;
    p.finSets = {canards, tails};

    BasicAeroModel m;
    const double V = 100.0;

    // 1. Overall stability check: with alpha > 0 and no control commands,
    // the larger aft tail fins should dominate the forward canards, producing net restoring nose-DOWN moment.
    auto wStab = m.computeWrench(V, 0.0, 5.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 1.225, 340.0, p);
    check(wStab.torque_y < 0.0, "multi-fin net stability: tail dominates canard -> nose-DOWN restoring moment");

    // 2. Control check: steerable canards should respond to positive pitch command -> nose-UP moment (+torque_y)
    // while non-steerable tails do not add anti-pitch control deflection.
    auto wCtrl = m.computeWrench(V, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.05, 0.0, 0.0, 1.225, 340.0, p);
    check(wCtrl.torque_y > 0.0, "canard pitch command -> nose-UP moment");

    // 3. Serialization check: multi-fin sets round-trip through JSON
    VehicleConfig vcfg;
    FinsConfig fCanard;
    fCanard.shape = FinShape::Trapezoidal;
    fCanard.count = 4;
    fCanard.positionM = 1.0;
    fCanard.rootChordM = 0.3;
    fCanard.tipChordM = 0.15;
    fCanard.spanM = 0.15;
    fCanard.steerable = true;

    FinsConfig fTail;
    fTail.shape = FinShape::Trapezoidal;
    fTail.count = 4;
    fTail.positionM = -1.5;
    fTail.rootChordM = 0.5;
    fTail.tipChordM = 0.35;
    fTail.spanM = 0.25;
    fTail.steerable = false;

    vcfg.aero.finSets = {fCanard, fTail};

    const std::string s = serializeVehicleConfig(vcfg);
    VehicleConfig rt = deserializeVehicleConfig(s);
    check(rt.aero.finSets.size() == 2, "deserialized finSets has 2 entries");
    check(rt.aero.finSets[0].positionM == 1.0 && rt.aero.finSets[0].steerable == true, "deserialized canard matches");
    check(rt.aero.finSets[1].positionM == -1.5 && rt.aero.finSets[1].steerable == false, "deserialized tail matches");
    check(rt.aero.allFinSets().size() == 2, "allFinSets() returns both sets");
}

static void planformChecks()
{
    std::printf("\n-- polygon planform engine --\n");
    const double r = 0.1;
    const double refArea = kPi * r * r;

    // The polygon metrics must reproduce the closed forms for the shapes that
    // have them, and must be independent of the caller's y origin.
    {
        std::string err;
        auto rect = [&](double y0) {
            return buildFinsGeometry(FinShape::FreeForm, 4, 0.0, 0.0, 0.0, -1.0,
                                     0.0, 0.0,
                                     {{0.0, y0}, {0.0, y0 + 0.2},
                                      {0.5, y0 + 0.2}, {0.5, y0}},
                                     refArea, &err);
        };
        auto a = rect(0.0);
        auto b = rect(0.2);
        auto c = rect(1.0);
        check(a && b && c, "free-form fin builds at every y origin");
        if (a && b && c) {
            const bool same =
                std::abs(a->Af - b->Af) < 1e-12 && std::abs(b->Af - c->Af) < 1e-12 &&
                std::abs(a->cpz - b->cpz) < 1e-12 && std::abs(b->cpz - c->cpz) < 1e-12 &&
                std::abs(a->Yma - b->Yma) < 1e-12 && std::abs(b->Yma - c->Yma) < 1e-12 &&
                std::abs(a->span - c->span) < 1e-12;
            check(same, "free-form metrics are invariant under a y-origin shift");
            check(std::abs(a->Af - 0.1) < 1e-12, "free-form rectangle area = 0.1");
            check(std::abs(a->cpz - 0.125) < 1e-12,
                  "free-form rectangle cpz = quarter-MAC = 0.125");
            check(std::abs(a->Yma - 0.1) < 1e-12, "free-form rectangle Yma = 0.1");
        }
    }

    // Delta preset: right triangle, area = root*span/2, quarter-MAC CP.
    {
        std::string err;
        auto g = buildFinsGeometry(FinShape::Delta, 4, 0.5, 0.0, 0.4, -1.0,
                                   0.0, 0.0, {}, refArea, &err);
        check(g != nullptr, "delta fin builds");
        if (g) {
            check(std::abs(g->Af - 0.1) < 1e-12, "delta area = root*span/2 = 0.1");
            check(g->Yma > 0.0, "delta Yma is positive");
            check(g->cpz > 0.0 && g->cpz < 0.5, "delta cpz is inside the root chord");
        }
    }

    // Cranked (double-delta) preset: six-vertex polygon, larger area than the
    // straight taper it would otherwise be.
    {
        std::string err;
        auto g = buildFinsGeometry(FinShape::Cranked, 4, 0.5, 0.15, 0.4, 0.2,
                                   0.0, 0.0, {}, refArea, &err, true,
                                   FinAirfoil::FlatPlate, 0.0, 0.5, 0.0, 0.0,
                                   0.4, 0.6);
        check(g != nullptr, "cranked fin builds");
        if (g) {
            check(g->Af > 0.1 && g->Af < 0.25, "cranked area lies between root and taper");
            check(g->Yma > 0.0 && g->cpz > 0.0, "cranked metrics are positive");
        }
        std::string bad;
        auto rejected = buildFinsGeometry(FinShape::Cranked, 4, 0.5, 0.15, 0.4, 0.2,
                                          0.0, 0.0, {}, refArea, &bad, true,
                                          FinAirfoil::FlatPlate, 0.0, 0.5, 0.0, 0.0,
                                          1.5, 0.6);
        check(rejected == nullptr, "cranked rejects a crank fraction outside (0,1)");
    }

    // Airfoil section parameters are carried through and validated.
    {
        std::string err;
        auto g = buildFinsGeometry(FinShape::Trapezoidal, 4, 0.5, 0.35, 0.25, 0.15,
                                   0.0, 0.0, {}, refArea, &err, true,
                                   FinAirfoil::DoubleWedge, 0.05, 0.4, 0.001, 0.002);
        check(g != nullptr, "fin with an airfoil section builds");
        if (g) {
            check(g->airfoil == FinAirfoil::DoubleWedge, "airfoil type is preserved");
            check(std::abs(g->thicknessRatio - 0.05) < 1e-12, "thickness ratio preserved");
            check(std::abs(g->maxThicknessLocation - 0.4) < 1e-12,
                  "max thickness location preserved");
            check(std::abs(g->leadingEdgeRadius - 0.001) < 1e-12 &&
                      std::abs(g->trailingEdgeThickness - 0.002) < 1e-12,
                  "edge radii preserved");
        }
        std::string bad;
        auto rejected = buildFinsGeometry(FinShape::Trapezoidal, 4, 0.5, 0.35, 0.25, 0.15,
                                          0.0, 0.0, {}, refArea, &bad, true,
                                          FinAirfoil::DoubleWedge, 0.9);
        check(rejected == nullptr, "thickness ratio above 0.5 is rejected");
    }

    // A flat-plate fin with zero thickness must equal the legacy geometry: the
    // airfoil fields default to inert.
    {
        std::string err;
        auto g = buildFinsGeometry(FinShape::Trapezoidal, 4, 0.5, 0.35, 0.25, 0.15,
                                   0.0, 0.0, {}, refArea, &err);
        check(g && g->airfoil == FinAirfoil::FlatPlate &&
                  g->thicknessRatio == 0.0,
              "default fin is a flat plate with zero thickness");
    }
}

static void dragModelChecks()
{
    std::printf("\n-- fin drag model --\n");
    const double r = 0.1;
    const double refArea = kPi * r * r;

    auto fin = [&](FinAirfoil af, double tc, double land = 0.33, double tte = 0.0) {
        std::string err;
        return buildFinsGeometry(FinShape::Trapezoidal, 4, 0.75, 0.30, 0.42, -1.0,
                                 0.0, 0.0, {}, refArea, &err, true,
                                 af, tc, 0.5, 0.0, tte, 0.5, 0.5, land);
    };

    // Ackeret: double wedge and biconvex must match their closed forms exactly
    // once referenced to the planform area.
    {
        auto dw = fin(FinAirfoil::DoubleWedge, 0.05);
        auto bx = fin(FinAirfoil::Biconvex, 0.05);
        const double ratio = dw->areaRatio();
        bool ok = true;
        for (double M : {1.5, 2.0, 3.0, 4.0}) {
            const double s = std::sqrt(M * M - 1.0);
            const double t = 0.05;
            ok = ok && std::abs(dw->waveDragCoefficient(M) - ratio * 4.0 * t * t / s) < 1e-12;
            ok = ok && std::abs(bx->waveDragCoefficient(M)
                                - ratio * (16.0 / 3.0) * t * t / s) < 1e-12;
        }
        check(ok, "wave drag matches Ackeret 4(t/c)^2/sqrt(M^2-1) for wedge and biconvex");
    }

    // Hexagonal carries a flat land, which raises its wave drag over a wedge by
    // 1/(1-f) for the same thickness ratio.
    {
        auto hx = fin(FinAirfoil::Hexagonal, 0.05, 0.33);
        auto dw = fin(FinAirfoil::DoubleWedge, 0.05);
        const double expected = dw->waveDragCoefficient(2.0) / (1.0 - 0.33);
        check(std::abs(hx->waveDragCoefficient(2.0) - expected) < 1e-9,
              "hexagonal wave drag = double wedge / (1 - land fraction)");
    }

    // Zero thickness, and a flat plate, must produce no wave drag at any Mach.
    {
        auto flat = fin(FinAirfoil::FlatPlate, 0.05);
        auto zero = fin(FinAirfoil::DoubleWedge, 0.0);
        double maxFlat = 0.0, maxZero = 0.0;
        for (double M = 0.1; M <= 6.0; M += 0.1) {
            maxFlat = std::max(maxFlat, flat->waveDragCoefficient(M));
            maxZero = std::max(maxZero, zero->waveDragCoefficient(M));
        }
        check(maxFlat == 0.0, "flat plate has zero wave drag");
        check(maxZero == 0.0, "zero thickness has zero wave drag");
    }

    // Wave drag must rise monotonically through the transonic blend, starting
    // and ending on the correct values.
    {
        auto dw = fin(FinAirfoil::DoubleWedge, 0.05);
        check(dw->waveDragCoefficient(0.8) == 0.0,
              "wave drag is zero at the divergence Mach");
        bool monotonic = true;
        double prev = 0.0;
        for (double M = 0.80; M <= 1.20 + 1e-9; M += 0.01) {
            const double cur = dw->waveDragCoefficient(M);
            if (cur < prev - 1e-12) monotonic = false;
            prev = cur;
        }
        check(monotonic, "wave drag rises monotonically through the transonic blend");
        const double s = std::sqrt(1.2 * 1.2 - 1.0);
        check(std::abs(dw->waveDragCoefficient(1.2)
                       - dw->areaRatio() * 4.0 * 0.05 * 0.05 / s) < 1e-12,
              "blend joins the supersonic branch at M=1.2");
    }

    // Thicker fins must create more wave drag; a thicker section also raises
    // skin friction through the form factor.
    {
        auto thin = fin(FinAirfoil::DoubleWedge, 0.03);
        auto thick = fin(FinAirfoil::DoubleWedge, 0.08);
        check(thick->waveDragCoefficient(3.0) > thin->waveDragCoefficient(3.0),
              "wave drag grows with thickness ratio");
        check(thick->skinFrictionDragCoefficient(3.0, 0.4, 1000.0, 250.0)
                  > thin->skinFrictionDragCoefficient(3.0, 0.4, 1000.0, 250.0),
              "skin friction form factor grows with thickness ratio");
    }

    // Skin friction must fall as Reynolds number rises.
    {
        auto f = fin(FinAirfoil::DoubleWedge, 0.05);
        const double lowRe = f->skinFrictionDragCoefficient(1.0, 0.4, 100.0, 250.0);
        const double highRe = f->skinFrictionDragCoefficient(1.0, 0.4, 1000.0, 250.0);
        check(lowRe > highRe, "skin friction falls with increasing Reynolds number");
        check(f->skinFrictionDragCoefficient(1.0, 0.0, 1000.0, 250.0) == 0.0,
              "skin friction is zero without density");
    }

    // Total is the sum of the parts and the parts are non-negative.
    {
        auto f = fin(FinAirfoil::Hexagonal, 0.06, 0.3, 0.001);
        const auto d = f->dragC(2.5, 0.4, 900.0, 250.0);
        check(std::abs(d.total - (d.wave + d.skinFriction + d.trailingEdge)) < 1e-15,
              "total fin drag is the sum of its terms");
        check(d.wave > 0.0 && d.skinFriction > 0.0 && d.trailingEdge > 0.0,
              "all three drag terms contribute for a thick section");
    }

    // Zero trailing edge thickness removes the base term only.
    {
        auto f = fin(FinAirfoil::DoubleWedge, 0.05, 0.33, 0.0);
        check(f->trailingEdgeDragCoefficient(2.0) == 0.0,
              "no trailing edge thickness means no base drag");
    }
}

int main()
{
    std::printf("=== fins: RocketPy geometric fin model (trapezoidal/elliptical/free-form) ===\n");
    geometryChecks();
    machAndSignChecks();
    controlPolarityChecks();
    multiFinSetChecks();
    serializationChecks();
    validationChecks();
    planformChecks();
    dragModelChecks();
    flightChecks();
    std::printf("\n%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
