#include <strikeengine/models/signatures/RCSDatabase.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/trigonometric.hpp>

namespace StrikeEngine::Models {

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

        _azimuth_breakpoints_rad.clear();
        _elevation_breakpoints_rad.clear();
        _rcs_table_dbsm.clear();

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

        // If table was formatted as [azimuth][elevation], transpose to [elevation][azimuth]
        // so that row index i corresponds to elevation and column index j to azimuth.
        if (_rcs_table_dbsm.size() == _azimuth_breakpoints_rad.size() &&
            !_rcs_table_dbsm.empty() &&
            _rcs_table_dbsm[0].size() == _elevation_breakpoints_rad.size() &&
            _azimuth_breakpoints_rad.size() != _elevation_breakpoints_rad.size()) {
            std::vector<std::vector<double>> transposed(
                _elevation_breakpoints_rad.size(),
                std::vector<double>(_azimuth_breakpoints_rad.size(), 0.0));
            for (std::size_t a = 0; a < _azimuth_breakpoints_rad.size(); ++a) {
                for (std::size_t e = 0; e < _elevation_breakpoints_rad.size(); ++e) {
                    if (e < _rcs_table_dbsm[a].size()) {
                        transposed[e][a] = _rcs_table_dbsm[a][e];
                    }
                }
            }
            _rcs_table_dbsm = std::move(transposed);
        }

        // Structural validation, mirroring IRSignatureDatabase: the table must
        // be [elevation][azimuth] and match the breakpoint counts. Without it a
        // mis-shaped table loaded silently and getRCS interpolated across the
        // wrong axis (or clamped to the wrong row), producing a plausible but
        // wrong RCS with no error.
        if (_rcs_table_dbsm.size() != _elevation_breakpoints_rad.size()) {
            return false;
        }
        for (const auto& row : _rcs_table_dbsm) {
            if (row.size() != _azimuth_breakpoints_rad.size()) {
                return false;
            }
        }

        return true;
    }

    double RCSDatabase::getRCS(double azimuth_rad, double elevation_rad) const {
        if (_azimuth_breakpoints_rad.empty() || _elevation_breakpoints_rad.empty() || _rcs_table_dbsm.empty()) {
            return 1.0; // Default RCS if no data is loaded
        }

        // Clamp the query to the tabulated envelope before binning. The
        // interpolation is performed in dBsm and converted only at the end, so
        // out-of-range weights (linear EXTRAPOLATION past the edge) would be
        // exponentially amplified in the returned m^2. Azimuth wraps over the
        // full-circle case before clamping.
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

        // Find indices for azimuth
        auto it_az = std::ranges::lower_bound(_azimuth_breakpoints_rad, azimuth_rad);
        int j = std::distance(_azimuth_breakpoints_rad.begin(), it_az);
        if (j >= static_cast<int>(_azimuth_breakpoints_rad.size())) j = static_cast<int>(_azimuth_breakpoints_rad.size()) - 1;
        if (j < 1) j = 1;

        // Find indices for elevation
        auto it_el = std::ranges::lower_bound(_elevation_breakpoints_rad, elevation_rad);
        int i = std::distance(_elevation_breakpoints_rad.begin(), it_el);
        if (i >= static_cast<int>(_elevation_breakpoints_rad.size())) i = static_cast<int>(_elevation_breakpoints_rad.size()) - 1;
        if (i < 1) i = 1;

        // Safe row and column indexing into _rcs_table_dbsm[elevation][azimuth]
        const std::size_t numRows = _rcs_table_dbsm.size();
        const std::size_t row1 = std::min(static_cast<std::size_t>(i - 1), numRows - 1);
        const std::size_t row2 = std::min(static_cast<std::size_t>(i), numRows - 1);

        const std::size_t numCols1 = _rcs_table_dbsm[row1].size();
        const std::size_t numCols2 = _rcs_table_dbsm[row2].size();
        if (numCols1 == 0 || numCols2 == 0) return 1.0;

        const std::size_t col1_r1 = std::min(static_cast<std::size_t>(j - 1), numCols1 - 1);
        const std::size_t col2_r1 = std::min(static_cast<std::size_t>(j), numCols1 - 1);
        const std::size_t col1_r2 = std::min(static_cast<std::size_t>(j - 1), numCols2 - 1);
        const std::size_t col2_r2 = std::min(static_cast<std::size_t>(j), numCols2 - 1);

        // Get the four corner points for interpolation. The upper index is
        // clamped separately from `j`/`i`: those are forced to at least 1 so
        // that `j - 1` is valid, which makes `breakpoints[j]` out of bounds
        // when an axis has a single entry. Clamping it collapses dAz/dEl to
        // zero and the guard below returns the tabulated corner.
        const std::size_t jUp = std::min(static_cast<std::size_t>(j),
                                         _azimuth_breakpoints_rad.size() - 1);
        const std::size_t iUp = std::min(static_cast<std::size_t>(i),
                                         _elevation_breakpoints_rad.size() - 1);
        double az1 = _azimuth_breakpoints_rad[j - 1];
        double az2 = _azimuth_breakpoints_rad[jUp];
        double el1 = _elevation_breakpoints_rad[i - 1];
        double el2 = _elevation_breakpoints_rad[iUp];

        double rcs_dbsm_11 = _rcs_table_dbsm[row1][col1_r1];
        double rcs_dbsm_12 = _rcs_table_dbsm[row1][col2_r1];
        double rcs_dbsm_21 = _rcs_table_dbsm[row2][col1_r2];
        double rcs_dbsm_22 = _rcs_table_dbsm[row2][col2_r2];

        // Perform bilinear interpolation on the dBsm values
        const double dAz = az2 - az1;
        const double dEl = el2 - el1;
        if (std::abs(dAz) < 1e-12 || std::abs(dEl) < 1e-12) {
            return std::pow(10.0, rcs_dbsm_11 / 10.0);
        }

        double term1 = rcs_dbsm_11 * (az2 - azimuth_rad) * (el2 - elevation_rad);
        double term2 = rcs_dbsm_21 * (azimuth_rad - az1) * (el2 - elevation_rad);
        double term3 = rcs_dbsm_12 * (az2 - azimuth_rad) * (elevation_rad - el1);
        double term4 = rcs_dbsm_22 * (azimuth_rad - az1) * (elevation_rad - el1);

        double interpolated_rcs_dbsm = (term1 + term2 + term3 + term4) / (dAz * dEl);

        // Convert the final result from dBsm back to a linear scale (m^2)
        return std::pow(10.0, interpolated_rcs_dbsm / 10.0);
    }

} // namespace StrikeEngine::Models
