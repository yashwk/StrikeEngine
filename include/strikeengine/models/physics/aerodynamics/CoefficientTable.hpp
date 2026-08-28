#pragma once
#include <vector>
#include <string>
#include <cstddef>
#include <algorithm>

namespace StrikeEngine::Models {

    /**
     * @brief Data-driven aerodynamic coefficient tables over a Mach x AoA grid.
     *
     * This mirrors the approach taken by RocketPy / OpenRocket: the
     * (Mach, AoA)-dependent lift and drag coefficients are reduced to lookup
     * tables that are interpolated (bilinearly) at runtime. When present they
     * are authoritative for cd/cl; the flat scalar AeroConfig coefficients
     * remain as the guaranteed fallback when no tables are supplied.
     *
     * clTable/cdTable are indexed [mach][aoa] and must have dimensions
     * [machBreakpoints.size()][aoaBreakpointsRad.size()].
     */
    struct AeroTables {
        std::vector<double> machBreakpoints;        // strictly ascending
        std::vector<double> aoaBreakpointsRad;      // strictly ascending
        std::vector<std::vector<double>> clTable;   // [mach][aoa]
        std::vector<std::vector<double>> cdTable;   // [mach][aoa]

        bool empty() const {
            return machBreakpoints.empty() || aoaBreakpointsRad.empty();
        }

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

            auto dimOk = [nm, na](const std::vector<std::vector<double>>& table) {
                if (table.size() != nm) return false;
                for (const auto& row : table) {
                    if (row.size() != na) return false;
                }
                return true;
            };

            if (!dimOk(clTable)) return fail("cl_table dims != [mach][aoa]");
            if (!dimOk(cdTable)) return fail("cd_table dims != [mach][aoa]");
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
