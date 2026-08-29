#pragma once
#include <vector>
#include <string>
#include <cstddef>
#include <algorithm>
#include <cmath>

namespace StrikeEngine::Models {

    /**
     * @brief Data-driven aerodynamic coefficient tables over a Mach x AoA grid.
     *
     * This mirrors the approach taken by RocketPy / OpenRocket: the
     * Mach/angle-dependent aerodynamic coefficients are reduced to lookup
     * tables that are interpolated (bilinearly) at runtime. The lift/drag and
     * pitch-moment tables use the Mach x AoA grid. The lateral-force, yawing-
     * moment, and rolling-moment tables use the Mach x beta grid.
     *
     * The cd/cl tables are the required base pair for a populated AeroTables
     * object. The moment and lateral tables are optional so existing cd/cl
     * profiles remain valid and keep their scalar moment/side-force fallback.
     *
     * clTable/cdTable are indexed [mach][aoa] and must have dimensions
     * [machBreakpoints.size()][aoaBreakpointsRad.size()].
     */
    struct AeroTables {
        std::vector<double> machBreakpoints;        // strictly ascending
        std::vector<double> aoaBreakpointsRad;      // strictly ascending
        std::vector<std::vector<double>> clTable;   // [mach][aoa]
        std::vector<std::vector<double>> cdTable;   // [mach][aoa]

        // Optional static pitch moment coefficient Cm(M, alpha).
        std::vector<std::vector<double>> cmTable;   // [mach][aoa]

        // Optional lateral coefficients on the Mach x beta grid. Cy is the
        // body +Y force coefficient, Cn is the body +Z yawing-moment
        // coefficient, and Cl is the body +X rolling-moment coefficient.
        std::vector<double> betaBreakpointsRad;     // strictly ascending
        std::vector<std::vector<double>> cyTable;   // [mach][beta]
        std::vector<std::vector<double>> cnTable;   // [mach][beta]
        std::vector<std::vector<double>> clRollTable; // [mach][beta]

        bool empty() const {
            return machBreakpoints.empty() || aoaBreakpointsRad.empty();
        }

        bool hasCmTable() const { return !cmTable.empty(); }
        bool hasCyTable() const { return !cyTable.empty(); }
        bool hasCnTable() const { return !cnTable.empty(); }
        bool hasClRollTable() const { return !clRollTable.empty(); }

        /**
         * @brief Validates the grid structure.
         * @return true when the breakpoints are strictly ascending and the
         *         tables are non-empty and dimensioned [mach][aoa].
         */
        bool isValid(std::string* error = nullptr) const {
            auto fail = [&](const char* msg) {
                if (error) *error = msg;
                return false;
            };

            if (machBreakpoints.empty()) return fail("mach_breakpoints is empty");
            if (aoaBreakpointsRad.empty()) return fail("aoa_breakpoints_rad is empty");
            if (machBreakpoints.size() < 2 || aoaBreakpointsRad.size() < 2) {
                return fail("need at least 2 breakpoints per axis");
            }

            const std::size_t nm = machBreakpoints.size();
            const std::size_t na = aoaBreakpointsRad.size();

            auto finiteAndAscending = [](const std::vector<double>& values) {
                for (std::size_t i = 0; i < values.size(); ++i) {
                    if (!std::isfinite(values[i])) return false;
                    if (i > 0 && !(values[i] > values[i - 1])) return false;
                }
                return true;
            };

            if (!finiteAndAscending(machBreakpoints)) {
                return fail("mach_breakpoints must be finite and strictly ascending");
            }
            if (!finiteAndAscending(aoaBreakpointsRad)) {
                return fail("aoa_breakpoints_rad must be finite and strictly ascending");
            }

            for (std::size_t i = 1; i < nm; ++i) {
                if (!(machBreakpoints[i] > machBreakpoints[i - 1])) {
                    return fail("mach_breakpoints not strictly ascending");
                }
            }
            for (std::size_t i = 1; i < na; ++i) {
                if (!(aoaBreakpointsRad[i] > aoaBreakpointsRad[i - 1])) {
                    return fail("aoa_breakpoints_rad not strictly ascending");
                }
            }

            auto dimOk = [nm](std::size_t nAngle,
                              const std::vector<std::vector<double>>& table) {
                if (table.empty()) return true;
                if (nAngle < 2) return false;
                if (table.size() != nm) return false;
                for (const auto& row : table) {
                    if (row.size() != nAngle) return false;
                    for (double value : row) {
                        if (!std::isfinite(value)) return false;
                    }
                }
                return true;
            };

            if (!dimOk(na, clTable)) return fail("cl_table dims != [mach][aoa]");
            if (!dimOk(na, cdTable)) return fail("cd_table dims != [mach][aoa]");
            if (clTable.empty()) return fail("cl_table is empty");
            if (cdTable.empty()) return fail("cd_table is empty");
            if (!dimOk(na, cmTable)) return fail("cm_table dims != [mach][aoa]");

            const bool hasLateral = hasCyTable() || hasCnTable() || hasClRollTable();
            if (hasLateral) {
                if (betaBreakpointsRad.size() < 2 ||
                    !finiteAndAscending(betaBreakpointsRad)) {
                    return fail("beta_breakpoints_rad must be finite and strictly ascending");
                }
                const std::size_t nb = betaBreakpointsRad.size();
                if (!dimOk(nb, cyTable)) return fail("cy_table dims != [mach][beta]");
                if (!dimOk(nb, cnTable)) return fail("cn_table dims != [mach][beta]");
                if (!dimOk(nb, clRollTable)) return fail("cl_roll_table dims != [mach][beta]");
            } else if (!betaBreakpointsRad.empty()) {
                return fail("beta_breakpoints_rad requires a lateral coefficient table");
            }
            return true;
        }
    };

    /**
     * @brief Bilinear interpolation over a rectilinear grid, clamped to the
     *        grid bounds (same clamping pattern as RCSDatabase::getRCS but
     *        without its first-breakpoint off-by-one bug).
     *
     * Queries at or below the first breakpoint clamp to the first column/row
     * and queries at or above the last clamp to the last, so the function is
     * exact at every breakpoint. table is indexed [xBreakpoints][yBreakpoints].
     */
    inline double interpolateCoefficient(
        double x, double y,
        const std::vector<double>& xBreakpoints,
        const std::vector<double>& yBreakpoints,
        const std::vector<std::vector<double>>& table)
    {
        const std::size_t nx = xBreakpoints.size();
        const std::size_t ny = yBreakpoints.size();
        if (nx < 2 || ny < 2) return 0.0;  // degenerate grid

        // Clamp to the grid bounds first; this keeps queries exactly on the
        // first breakpoint in the first bin (tx = 0) rather than shifting to
        // the second bin.
        if (x <= xBreakpoints.front()) x = xBreakpoints.front();
        if (x >= xBreakpoints.back())  x = xBreakpoints.back();
        if (y <= yBreakpoints.front()) y = yBreakpoints.front();
        if (y >= yBreakpoints.back())  y = yBreakpoints.back();

        // Upper-bound index of the enclosing bin. After clamping, x is in
        // (first, last], so xi is in [1, nx-1]; the guard keeps it there.
        std::size_t xi = std::size_t(std::upper_bound(xBreakpoints.begin(),
                                                      xBreakpoints.end(), x)
                                     - xBreakpoints.begin());
        if (xi >= nx) xi = nx - 1;
        if (xi == 0)  xi = 1;
        std::size_t yi = std::size_t(std::upper_bound(yBreakpoints.begin(),
                                                      yBreakpoints.end(), y)
                                     - yBreakpoints.begin());
        if (yi >= ny) yi = ny - 1;
        if (yi == 0)  yi = 1;

        const std::size_t x0 = xi - 1, x1 = xi;
        const std::size_t y0 = yi - 1, y1 = yi;
        const double x0v = xBreakpoints[x0], x1v = xBreakpoints[x1];
        const double y0v = yBreakpoints[y0], y1v = yBreakpoints[y1];

        const double tx = (x - x0v) / (x1v - x0v);
        const double ty = (y - y0v) / (y1v - y0v);

        const double f00 = table[x0][y0];
        const double f01 = table[x0][y1];
        const double f10 = table[x1][y0];
        const double f11 = table[x1][y1];

        return (1.0 - tx) * (1.0 - ty) * f00
             + tx          * (1.0 - ty) * f10
             + (1.0 - tx) * ty          * f01
             + tx          * ty          * f11;
    }

} // namespace StrikeEngine::Models
