#pragma once

#include <string>
#include <vector>

namespace StrikeEngine {

    class IRSignatureDatabase {
    public:
        /**
         * @brief Loads and parses an IR signature profile from a JSON file.
         * @param file_path The path to the IR signature JSON profile.
         * @return True if loading was successful, false otherwise.
         */
        bool loadProfile(const std::string& file_path);

        /**
         * @brief Gets the Radiant Intensity for a specific aspect angle using bilinear interpolation.
         * @param azimuth_rad The azimuth angle, in radians.
         * @param elevation_rad The elevation angle, in radians.
         * @return The interpolated Radiant Intensity value in Watts per steradian (W/sr).
         */
        [[nodiscard]] double getRadiantIntensity(double azimuth_rad, double elevation_rad) const;

    private:
        std::string _name;

        // Breakpoints for the lookup table axes, stored in radians.
        std::vector<double> _azimuth_breakpoints_rad;
        std::vector<double> _elevation_breakpoints_rad;

        // The 2D table of Radiant Intensity values in W/sr.
        std::vector<std::vector<double>> _radiant_intensity_table_W_per_sr;
    };

} // namespace StrikeEngine
