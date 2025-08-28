
#include "strikeengine/flight/IRSignatureDatabase.hpp"
#include "nlohmann/json.hpp"
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>

#include "glm/glm.hpp"
#include "glm/trigonometric.hpp"

namespace StrikeEngine {

    bool IRSignatureDatabase::loadProfile(const std::string& file_path) {
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

        _name = data.value("name", "Unnamed IR Signature Profile");

        auto az_deg = data.at("azimuth_breakpoints_deg").get<std::vector<double>>();
        for (double deg : az_deg) {
            _azimuth_breakpoints_rad.push_back(glm::radians(deg));
        }

        auto el_deg = data.at("elevation_breakpoints_deg").get<std::vector<double>>();
        for (double deg : el_deg) {
            _elevation_breakpoints_rad.push_back(glm::radians(deg));
        }

        _radiant_intensity_table_W_per_sr = data.at("radiant_intensity_table_W_per_sr").get<std::vector<std::vector<double>>>();

        return true;
    }

    double IRSignatureDatabase::getRadiantIntensity(double azimuth_rad, double elevation_rad) const {
        if (_azimuth_breakpoints_rad.empty() || _elevation_breakpoints_rad.empty()) {
            return 0.0; // Default signature if no data is loaded
        }

        // --- Bilinear Interpolation Logic ---

        auto it_az = std::ranges::lower_bound(_azimuth_breakpoints_rad, azimuth_rad);
        int j = std::distance(_azimuth_breakpoints_rad.begin(), it_az);
        if (j >= _azimuth_breakpoints_rad.size()) j = _azimuth_breakpoints_rad.size() - 1;
        if (j == 0) j = 1;

        auto it_el = std::ranges::lower_bound(_elevation_breakpoints_rad, elevation_rad);
        int i = std::distance(_elevation_breakpoints_rad.begin(), it_el);
        if (i >= _elevation_breakpoints_rad.size()) i = _elevation_breakpoints_rad.size() - 1;
        if (i == 0) i = 1;

        double az1 = _azimuth_breakpoints_rad[j - 1];
        double az2 = _azimuth_breakpoints_rad[j];
        double el1 = _elevation_breakpoints_rad[i - 1];
        double el2 = _elevation_breakpoints_rad[i];

        double ri_11 = _radiant_intensity_table_W_per_sr[i - 1][j - 1];
        double ri_12 = _radiant_intensity_table_W_per_sr[i - 1][j];
        double ri_21 = _radiant_intensity_table_W_per_sr[i][j - 1];
        double ri_22 = _radiant_intensity_table_W_per_sr[i][j];

        double term1 = ri_11 * (az2 - azimuth_rad) * (el2 - elevation_rad);
        double term2 = ri_21 * (azimuth_rad - az1) * (el2 - elevation_rad);
        double term3 = ri_12 * (az2 - azimuth_rad) * (elevation_rad - el1);
        double term4 = ri_22 * (azimuth_rad - az1) * (elevation_rad - el1);

        double denominator = (az2 - az1) * (el2 - el1);
        if (std::abs(denominator) < 1e-9) {
            return ri_11;
        }

        return (term1 + term2 + term3 + term4) / denominator;
    }

} // namespace StrikeEngine
