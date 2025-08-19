#pragma once

#include <string>
#include <vector>

namespace StrikeEngine {

    class RCSDatabase {
    public:
        /**
         * @brief Loads and parses an RCS profile from a JSON file.
         * @param file_path The path to the RCS JSON profile.
         * @return True if loading was successful, false otherwise.
         */
        bool loadProfile(const std::string& file_path);

        /**
         * @brief Gets the RCS value for a specific aspect angle using bilinear interpolation.
         * @param azimuth_rad The azimuth angle, in radians.
         * @param elevation_rad The elevation angle, in radians.
         * @return The interpolated RCS value in square meters (m^2).
         */
        double getRCS(double azimuth_rad, double elevation_rad) const;

    private:
        std::string _name;

        // Breakpoints for the lookup table axes, stored in radians.
        std::vector<double> _azimuth_breakpoints_rad;
        std::vector<double> _elevation_breakpoints_rad;

        // The 2D table of RCS values in decibels relative to one square meter (dBsm).
        std::vector<std::vector<double>> _rcs_table_dbsm;
    };

} // namespace StrikeEngine
