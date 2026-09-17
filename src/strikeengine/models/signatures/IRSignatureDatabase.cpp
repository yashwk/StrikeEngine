#include <strikeengine/models/signatures/IRSignatureDatabase.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/trigonometric.hpp>

namespace StrikeEngine::Models {

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

        // If the table was formatted as [azimuth][elevation], transpose to
        // [elevation][azimuth] (same heuristic as RCSDatabase) so row index i
        // corresponds to elevation and column index j to azimuth.
        if (_radiant_intensity_table_W_per_sr.size() == _azimuth_breakpoints_rad.size() &&
            !_radiant_intensity_table_W_per_sr.empty() &&
            _radiant_intensity_table_W_per_sr[0].size() == _elevation_breakpoints_rad.size() &&
            _azimuth_breakpoints_rad.size() != _elevation_breakpoints_rad.size()) {
            std::vector<std::vector<double>> transposed(
                _elevation_breakpoints_rad.size(),
                std::vector<double>(_azimuth_breakpoints_rad.size(), 0.0));
            for (std::size_t a = 0; a < _azimuth_breakpoints_rad.size(); ++a) {
                for (std::size_t e = 0; e < _elevation_breakpoints_rad.size(); ++e) {
                    if (e < _radiant_intensity_table_W_per_sr[a].size()) {
                        transposed[e][a] = _radiant_intensity_table_W_per_sr[a][e];
                    }
                }
            }
            _radiant_intensity_table_W_per_sr = std::move(transposed);
        }

        // Structural validation: the table must be [elevation][azimuth] and
        // match the breakpoint counts. A malformed profile fails the load
        // instead of reading out of bounds inside getRadiantIntensity.
        if (_radiant_intensity_table_W_per_sr.size() != _elevation_breakpoints_rad.size()) {
            return false;
        }
        for (const auto& row : _radiant_intensity_table_W_per_sr) {
            if (row.size() != _azimuth_breakpoints_rad.size()) {
                return false;
            }
        }

        return true;
    }

    double IRSignatureDatabase::getRadiantIntensity(double azimuth_rad, double elevation_rad) const {
        if (_azimuth_breakpoints_rad.empty() || _elevation_breakpoints_rad.empty() ||
            _radiant_intensity_table_W_per_sr.empty()) {
            return 0.0; // Default signature if no data is loaded
        }

        // Clamp the query to the tabulated envelope before binning: the raw
        // lower_bound bins leave out-of-range weights outside [0, 1], i.e. an
        // unbounded linear EXTRAPOLATION of radiant intensity past the edge.
        // Full-circle azimuth tables wrap across the seam.
        const double azFront = _azimuth_breakpoints_rad.front();
        const double azBack = _azimuth_breakpoints_rad.back();
        if (azBack - azFront >= 2.0 * glm::pi<double>() - 1e-9) {
            azimuth_rad = std::remainder(
                azimuth_rad - azFront, 2.0 * glm::pi<double>()) + azFront;
            if (azimuth_rad < azFront) azimuth_rad += 2.0 * glm::pi<double>();
        }
        azimuth_rad = std::clamp(azimuth_rad, azFront, azBack);
        elevation_rad = std::clamp(elevation_rad,
                                   _elevation_breakpoints_rad.front(),
                                   _elevation_breakpoints_rad.back());

        // --- Bilinear Interpolation Logic ---

        auto it_az = std::ranges::lower_bound(_azimuth_breakpoints_rad, azimuth_rad);
        int j = std::distance(_azimuth_breakpoints_rad.begin(), it_az);
        if (j >= static_cast<int>(_azimuth_breakpoints_rad.size())) j = static_cast<int>(_azimuth_breakpoints_rad.size()) - 1;
        if (j < 1) j = 1;

        auto it_el = std::ranges::lower_bound(_elevation_breakpoints_rad, elevation_rad);
        int i = std::distance(_elevation_breakpoints_rad.begin(), it_el);
        if (i >= static_cast<int>(_elevation_breakpoints_rad.size())) i = static_cast<int>(_elevation_breakpoints_rad.size()) - 1;
        if (i < 1) i = 1;

        // Defensive row/column clamping (mirrors RCSDatabase::getRCS) so a
        // short table row can never index out of bounds.
        const std::size_t numRows = _radiant_intensity_table_W_per_sr.size();
        const std::size_t row1 = std::min(static_cast<std::size_t>(i - 1), numRows - 1);
        const std::size_t row2 = std::min(static_cast<std::size_t>(i), numRows - 1);
        const std::size_t numCols1 = _radiant_intensity_table_W_per_sr[row1].size();
        const std::size_t numCols2 = _radiant_intensity_table_W_per_sr[row2].size();
        if (numCols1 == 0 || numCols2 == 0) return 0.0;
        const std::size_t col1_r1 = std::min(static_cast<std::size_t>(j - 1), numCols1 - 1);
        const std::size_t col2_r1 = std::min(static_cast<std::size_t>(j), numCols1 - 1);
        const std::size_t col1_r2 = std::min(static_cast<std::size_t>(j - 1), numCols2 - 1);
        const std::size_t col2_r2 = std::min(static_cast<std::size_t>(j), numCols2 - 1);

        // The upper breakpoint index is clamped separately from `i`/`j`: those
        // are forced to at least 1 so that `j - 1` is valid, which makes
        // `breakpoints[j]` out of bounds when an axis has a single entry.
        // Clamping it collapses the denominator to zero and the guard below
        // returns the tabulated corner.
        const std::size_t jUp = std::min(static_cast<std::size_t>(j),
                                         _azimuth_breakpoints_rad.size() - 1);
        const std::size_t iUp = std::min(static_cast<std::size_t>(i),
                                         _elevation_breakpoints_rad.size() - 1);
        double az1 = _azimuth_breakpoints_rad[j - 1];
        double az2 = _azimuth_breakpoints_rad[jUp];
        double el1 = _elevation_breakpoints_rad[i - 1];
        double el2 = _elevation_breakpoints_rad[iUp];

        double ri_11 = _radiant_intensity_table_W_per_sr[row1][col1_r1];
        double ri_12 = _radiant_intensity_table_W_per_sr[row1][col2_r1];
        double ri_21 = _radiant_intensity_table_W_per_sr[row2][col1_r2];
        double ri_22 = _radiant_intensity_table_W_per_sr[row2][col2_r2];

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

} // namespace StrikeEngine::Models
