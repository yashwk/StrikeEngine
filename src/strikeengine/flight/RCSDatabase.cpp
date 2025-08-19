// ===================================================================================
//  RCSDatabase.cpp
//
//  Description:
//  Implementation of the RCSDatabase class. Contains the logic for parsing the
//  RCS JSON file and performing bilinear interpolation on the data table.
//
//  Associated Plan: "Upgrade Plan: Level 2 Radar Simulation" (Step 2)
//
// ===================================================================================

#include "strikeengine/flight/RCSDatabase.hpp"
#include "nlohmann/json.hpp"
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>

#include "glm/detail/func_trigonometric.inl"

namespace StrikeEngine {

    bool RCSDatabase::loadProfile(const std::string& file_path) {
        std::ifstream f(file_path);
        if (!f.is_open()) {
            return false;
        }

        nlohmann::json data;
        try {
            data = nlohmann::json::parse(f);
        } catch (const nlohmann::json::parse_error& e) {
            return false;
        }

        _name = data.value("name", "Unnamed RCS Profile");

        // Load breakpoints and convert from degrees to radians
        auto az_deg = data.at("azimuth_breakpoints_deg").get<std::vector<double>>();
        for (double deg : az_deg) {
            _azimuth_breakpoints_rad.push_back(glm::radians(deg));
        }

        auto el_deg = data.at("elevation_breakpoints_deg").get<std::vector<double>>();
        for (double deg : el_deg) {
            _elevation_breakpoints_rad.push_back(glm::radians(deg));
        }

        _rcs_table_dbsm = data.at("rcs_table_dbsm").get<std::vector<std::vector<double>>>();

        return true;
    }

    double RCSDatabase::getRCS(double azimuth_rad, double elevation_rad) const {
        if (_azimuth_breakpoints_rad.empty() || _elevation_breakpoints_rad.empty()) {
            return 1.0; // Default RCS if no data is loaded
        }

        // --- Bilinear Interpolation Logic ---

        // Find indices for azimuth
        auto it_az = std::ranges::lower_bound(_azimuth_breakpoints_rad, azimuth_rad);
        int j = std::distance(_azimuth_breakpoints_rad.begin(), it_az);
        if (j >= _azimuth_breakpoints_rad.size()) j = _azimuth_breakpoints_rad.size() - 1;
        if (j == 0) j = 1;

        // Find indices for elevation
        auto it_el = std::ranges::lower_bound(_elevation_breakpoints_rad, elevation_rad);
        int i = std::distance(_elevation_breakpoints_rad.begin(), it_el);
        if (i >= _elevation_breakpoints_rad.size()) i = _elevation_breakpoints_rad.size() - 1;
        if (i == 0) i = 1;

        // Get the four corner points for interpolation
        double az1 = _azimuth_breakpoints_rad[j - 1];
        double az2 = _azimuth_breakpoints_rad[j];
        double el1 = _elevation_breakpoints_rad[i - 1];
        double el2 = _elevation_breakpoints_rad[i];

        double rcs_dbsm_11 = _rcs_table_dbsm[i - 1][j - 1];
        double rcs_dbsm_12 = _rcs_table_dbsm[i - 1][j];
        double rcs_dbsm_21 = _rcs_table_dbsm[i][j - 1];
        double rcs_dbsm_22 = _rcs_table_dbsm[i][j];

        // Perform bilinear interpolation on the dBsm values
        double term1 = rcs_dbsm_11 * (az2 - azimuth_rad) * (el2 - elevation_rad);
        double term2 = rcs_dbsm_21 * (azimuth_rad - az1) * (el2 - elevation_rad);
        double term3 = rcs_dbsm_12 * (az2 - azimuth_rad) * (elevation_rad - el1);
        double term4 = rcs_dbsm_22 * (azimuth_rad - az1) * (elevation_rad - el1);

        double interpolated_rcs_dbsm = (term1 + term2 + term3 + term4) / ((az2 - az1) * (el2 - el1));

        // Convert the final result from dBsm back to a linear scale (m^2)
        return std::pow(10.0, interpolated_rcs_dbsm / 10.0);
    }

} // namespace StrikeEngine
