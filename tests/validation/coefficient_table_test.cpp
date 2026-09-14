// Unit tests for the data-driven aero coefficient table machinery:
//   - interpolateCoefficient is exact at every breakpoint
//   - bilinear interior values on a linear grid (e.g. midpoints average)
//   - clamps below/above the grid bounds (no off-by-one at the first breakpoint)
//   - AeroTables::isValid rejects malformed grids and accepts valid ones
//   - optional Cm/Cy/Cn/Cl tables interpolate into the aerodynamic wrench
#include <strikeengine/models/physics/aerodynamics/CoefficientTable.hpp>
#include <strikeengine/models/physics/aerodynamics/AeroModel.hpp>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using StrikeEngine::Models::AeroTables;
using StrikeEngine::Models::interpolateCoefficient;

static int failures = 0;
static void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}
static void checkClose(double a, double b, double tol, const char* what) {
    check(std::fabs(a - b) <= tol, what);
}

int main() {
    std::printf("=== coefficient_table: bilinear interpolation + grid validation ===\n");

    // A linear 2x2 grid: f(x,y) = 10x + 100y over x in [0,1], y in [0,1].
    const std::vector<double> xs{0.0, 1.0};
    const std::vector<double> ys{0.0, 1.0};
    const std::vector<std::vector<double>> table{
        {0.0, 100.0},   // y=0, y=1 at x=0
        {10.0, 110.0}   // y=0, y=1 at x=1
    };

    // Exact at breakpoints.
    checkClose(interpolateCoefficient(0.0, 0.0, xs, ys, table), 0.0, 1e-12,
               "exact at bottom-left breakpoint");
    checkClose(interpolateCoefficient(1.0, 1.0, xs, ys, table), 110.0, 1e-12,
               "exact at top-right breakpoint");
    checkClose(interpolateCoefficient(0.0, 1.0, xs, ys, table), 100.0, 1e-12,
               "exact at top-left breakpoint");
    checkClose(interpolateCoefficient(1.0, 0.0, xs, ys, table), 10.0, 1e-12,
               "exact at bottom-right breakpoint");

    // Interior: bilinear is linear so midpoints average.
    checkClose(interpolateCoefficient(0.5, 0.5, xs, ys, table), 55.0, 1e-12,
               "bilinear interior midpoint (average of corners)");
    checkClose(interpolateCoefficient(0.5, 0.0, xs, ys, table), 5.0, 1e-12,
               "edge midpoint along x");
    checkClose(interpolateCoefficient(0.0, 0.5, xs, ys, table), 50.0, 1e-12,
               "edge midpoint along y");
    checkClose(interpolateCoefficient(0.25, 0.75, xs, ys, table),
               0.25 * 10.0 + 0.75 * 100.0 + 0.25 * 0.75 * 0.0, 1e-12,
               "arbitrary interior point matches the bilinear formula");

    // Clamping below/above the grid bounds.
    checkClose(interpolateCoefficient(-5.0, -5.0, xs, ys, table), 0.0, 1e-12,
               "clamps below both axes to bottom-left");
    checkClose(interpolateCoefficient(-2.0, 0.5, xs, ys, table), 50.0, 1e-12,
               "clamps below x to first column, interpolates y");
    checkClose(interpolateCoefficient(3.0, 4.0, xs, ys, table), 110.0, 1e-12,
               "clamps above both axes to top-right");
    checkClose(interpolateCoefficient(0.5, 7.0, xs, ys, table), 105.0, 1e-12,
               "clamps above y to last row, interpolates x");

    // A query exactly on the FIRST breakpoint (not clamped, in-bin) must
    // reproduce the first-column value (regression for the RCS j==0->j=1 bug).
    checkClose(interpolateCoefficient(0.0, 0.25, xs, ys, table), 25.0, 1e-12,
               "first-breakpoint query (x=0) interpolates in the first column");
    checkClose(interpolateCoefficient(0.25, 0.0, xs, ys, table), 2.5, 1e-12,
               "first-breakpoint query (y=0) interpolates in the first row");

    // ---- AeroTables::isValid ----
    auto validTables = [] {
        AeroTables t;
        t.machBreakpoints = {0.0, 0.5, 1.0};
        t.aoaBreakpointsRad = {0.0, 0.1, 0.2};
        t.clTable = std::vector<std::vector<double>>(3, std::vector<double>(3, 0.0));
        t.cdTable = std::vector<std::vector<double>>(3, std::vector<double>(3, 0.0));
        return t;
    };
    {
        AeroTables t = validTables();
        std::string err;
        check(t.isValid(&err), "valid grid is accepted");
    }
    {
        AeroTables t = validTables();
        t.machBreakpoints = {0.0, 0.5, 0.5};  // not strictly ascending
        std::string err;
        check(!t.isValid(&err), "non-ascending mach breakpoints rejected");
    }
    {
        AeroTables t = validTables();
        t.aoaBreakpointsRad = {0.0, 0.2, 0.1};  // descending
        std::string err;
        check(!t.isValid(&err), "non-ascending aoa breakpoints rejected");
    }
    {
        AeroTables t = validTables();
        t.clTable = std::vector<std::vector<double>>(2, std::vector<double>(3, 0.0));
        std::string err;
        check(!t.isValid(&err), "cl_table row count mismatch rejected");
    }
    {
        AeroTables t = validTables();
        t.cdTable = std::vector<std::vector<double>>(3, std::vector<double>(2, 0.0));
        std::string err;
        check(!t.isValid(&err), "cd_table column count mismatch rejected");
    }
    {
        AeroTables t = validTables();
        t.machBreakpoints = {};
        std::string err;
        check(!t.isValid(&err), "empty grid rejected");
    }

    // Optional moment/lateral tables use the same Mach axis, with alpha for
    // Cm and beta for Cy/Cn/Cl. Their dimensions and runtime signs are part
    // of the aerodynamic data contract.
    {
        AeroTables t = validTables();
        t.aoaBreakpointsRad = {-1.0, 1.0};
        t.clTable = std::vector<std::vector<double>>(3, std::vector<double>(2, 0.0));
        t.cdTable = std::vector<std::vector<double>>(3, std::vector<double>(2, 0.0));
        t.betaBreakpointsRad = {-1.0, 1.0};
        t.cmTable = {{0.2, 0.2}, {0.2, 0.2}, {0.2, 0.2}};
        t.cyTable = {{-0.6, 0.0}, {-0.6, 0.0}, {-0.6, 0.0}};
        t.cnTable = {{0.4, 0.4}, {0.4, 0.4}, {0.4, 0.4}};
        t.clRollTable = {{0.1, 0.1}, {0.1, 0.1}, {0.1, 0.1}};
        std::string err;
        check(t.isValid(&err), "optional moment and lateral tables are accepted");

        StrikeEngine::Models::BasicAeroModel model;
        StrikeEngine::Models::AeroParams p;
        p.referenceArea = 1.0;
        p.referenceLength = 1.0;
        p.cd = 0.0;
        p.clAlpha = 0.0;
        p.clFin = 0.0;
        p.clMax = 10.0;
        p.tables = std::make_shared<const AeroTables>(t);

        // V=10 m/s and a=10 m/s make Mach=1 at the supplied sound speed;
        // q=50 Pa. At alpha=beta=0, Cy is the average of its four corners
        // (-0.3), while the other tables are constant.
        const auto wrench = model.computeWrench(
            10.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            1.0, 10.0, p);
        checkClose(wrench.force_y, -15.0, 1e-12,
                   "Cy table produces the expected body-Y side force");
        checkClose(wrench.torque_x, 5.0, 1e-12,
                   "Cl roll table produces the expected body-X moment");
        checkClose(wrench.torque_y, 10.0, 1e-12,
                   "Cm table produces the expected body-Y moment");
        checkClose(wrench.torque_z, 20.0, 1e-12,
                   "Cn table produces the expected body-Z moment");
    }
    {
        AeroTables t = validTables();
        t.betaBreakpointsRad = {-1.0, 1.0};
        t.cyTable = {{0.0, 0.0}, {0.0, 0.0}};
        t.cmTable = {{0.0, 0.0}}; // wrong Mach row count
        std::string err;
        check(!t.isValid(&err), "optional moment table row mismatch rejected");
    }
    {
        AeroTables t = validTables();
        t.cyTable = {{0.0, 0.0}, {0.0, 0.0}};
        std::string err;
        check(!t.isValid(&err), "lateral table without beta breakpoints rejected");
    }

    // One-sided exported grids (positive angles only, as the designer writes
    // symmetric airframes) must mirror odd coefficients so the vehicle can
    // push over and yaw both ways. Regression: a negative angle clamped to the
    // zero-lift column, leaving a table+fins missile with no body sideslip
    // force in one direction.
    {
        AeroTables t;
        t.machBreakpoints = {0.5, 1.0, 2.0};
        t.aoaBreakpointsRad = {0.0, 0.1, 0.2};
        t.clTable = {{0.0, 0.7, 1.4}, {0.0, 0.7, 1.4}, {0.0, 0.6, 1.2}};
        t.cdTable = std::vector<std::vector<double>>(3, std::vector<double>(3, 0.02));
        std::string err;
        check(t.isValid(&err), "one-sided grid accepted");

        StrikeEngine::Models::AeroParams p;
        p.referenceArea = 0.05;
        p.referenceLength = 1.0;
        p.clAlpha = 0.0;
        p.clFin = 0.0;
        p.clMax = 10.0;
        p.tables = std::make_shared<const AeroTables>(t);
        p.fins = StrikeEngine::Models::buildFinsGeometry(
            StrikeEngine::Models::FinShape::Trapezoidal, 4,
            0.5, 0.35, 0.25, 0.15, -1.6, 0.0, {}, 0.05, &err);
        check(p.fins != nullptr, "test fin set builds");

        StrikeEngine::Models::BasicAeroModel model;
        const double V = 100.0;  // Mach 1 at sound speed 100, q = 5 kPa
        auto atBeta = [&](double b) {
            return model.computeWrench(V, V * std::tan(b), 0.0, 0, 0, 0,
                                       0, 0, 0, 1.0, 100.0, p);
        };
        auto atAlpha = [&](double a) {
            return model.computeWrench(V, 0.0, V * std::tan(a), 0, 0, 0,
                                       0, 0, 0, 1.0, 100.0, p);
        };
        const auto yPlus = atBeta(0.05), yMinus = atBeta(-0.05);
        check(yPlus.force_y < -1.0 && yMinus.force_y > 1.0,
              "body sideslip force opposes beta in both directions");
        checkClose(yPlus.force_y + yMinus.force_y, 0.0,
                   1e-9 * std::abs(yPlus.force_y),
                   "side force antisymmetric in beta");
        check(yPlus.torque_z > 0.0 && yMinus.torque_z < 0.0,
              "yaw moment restores in both directions");
        const auto aPlus = atAlpha(0.05), aMinus = atAlpha(-0.05);
        checkClose(aPlus.force_z + aMinus.force_z, 0.0,
                   1e-9 * std::abs(aPlus.force_z),
                   "pitch lift antisymmetric in alpha");
        check(std::abs(aMinus.force_z) > 1.0, "negative alpha produces lift");

        // Same mirror requirement on the finless abstract path.
        p.fins = nullptr;
        const auto nPlus = atBeta(0.05), nMinus = atBeta(-0.05);
        check(nPlus.force_y < -1.0 && nMinus.force_y > 1.0,
              "finless sideslip force opposes beta in both directions");
        checkClose(nPlus.force_y + nMinus.force_y, 0.0,
                   1e-9 * std::abs(nPlus.force_y),
                   "finless side force antisymmetric in beta");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
