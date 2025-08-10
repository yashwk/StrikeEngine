#include "strikeengine/flight/AerodynamicsDatabase.hpp"
#include "nlohmann/json.hpp"
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <iostream>

namespace StrikeEngine {

    using json = nlohmann::json;

    bool AerodynamicsDatabase::loadProfile(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            return false;
        }

        try {
            json data = json::parse(file);
            _machBreakpoints = data.at("mach_breakpoints").get<std::vector<double>>();
            _aoaBreakpointsRad = data.at("aoa_breakpoints_rad").get<std::vector<double>>();
            _clTable = data.at("cl_table").get<std::vector<std::vector<double>>>();
            _cdTable = data.at("cd_table").get<std::vector<std::vector<double>>>();
        } catch (const json::exception& e) {
            std::cerr << "Error parsing aerodynamic profile file: " << filepath << std::endl;
            return false;
        }
        return true;
    }

    AeroCoefficients AerodynamicsDatabase::getCoefficients(double mach, double AoARad) const {
        if (_machBreakpoints.empty() || _aoaBreakpointsRad.empty()) {
            return {};
        }

        auto it_mach = std::ranges::upper_bound(_machBreakpoints, mach);
        size_t i_mach1 = std::distance(_machBreakpoints.begin(), it_mach) - 1;
        size_t i_mach2 = i_mach1 + 1;
        if (i_mach1 < 0) i_mach1 = 0;
        if (i_mach2 >= _machBreakpoints.size()) i_mach2 = _machBreakpoints.size() - 1;
        double machFraction = (mach - _machBreakpoints[i_mach1]) / (_machBreakpoints[i_mach2] - _machBreakpoints[i_mach1]);
        machFraction = std::clamp(machFraction, 0.0, 1.0);

        // Find the indices and fractions for interpolation for AoA
        auto it_aoa = std::ranges::upper_bound(_aoaBreakpointsRad, AoARad);
        size_t j_aoa1 = std::distance(_aoaBreakpointsRad.begin(), it_aoa) - 1;
        size_t j_aoa2 = j_aoa1 + 1;
        if (j_aoa1 < 0) j_aoa1 = 0;
        if (j_aoa2 >= _aoaBreakpointsRad.size()) j_aoa2 = _aoaBreakpointsRad.size() - 1;
        double AoAFraction = (AoARad - _aoaBreakpointsRad[j_aoa1]) / (_aoaBreakpointsRad[j_aoa2] - _aoaBreakpointsRad[j_aoa1]);
        AoAFraction = std::clamp(AoAFraction, 0.0, 1.0);

        // Bilinear interpolation function
        auto interpolate = [&](const std::vector<std::vector<double>>& table) {
            const double c00 = table[i_mach1][j_aoa1];
            const double c10 = table[i_mach2][j_aoa1];
            const double c01 = table[i_mach1][j_aoa2];
            const double c11 = table[i_mach2][j_aoa2];

            const double r1 = c00 * (1.0 - machFraction) + c10 * machFraction;
            const double r2 = c01 * (1.0 - machFraction) + c11 * machFraction;

            return r1 * (1.0 - AoAFraction) + r2 * AoAFraction;
        };

        AeroCoefficients coefficients;
        coefficients.Cl = interpolate(_clTable);
        coefficients.Cd = interpolate(_cdTable);
        return coefficients;
    }

} // namespace StrikeEngine
