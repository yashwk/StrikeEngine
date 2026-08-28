// Unit tests for the data-driven aero coefficient table machinery:
//   - interpolateCoefficient is exact at every breakpoint
//   - bilinear interior values on a linear grid (e.g. midpoints average)
//   - clamps below/above the grid bounds (no off-by-one at the first breakpoint)
//   - AeroTables::isValid rejects malformed grids and accepts valid ones
#include <strikeengine/models/physics/aerodynamics/CoefficientTable.hpp>

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

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
