#pragma once

#include "strikeengine/atmosphere/AtmosphereModel.hpp"
#include "strikeengine/components/sensors/InfraredSeekerComponent.hpp"
#include <string>
#include <vector>

namespace StrikeEngine {
    /**
     * @brief Manages loading and querying of atmospheric data.
     *
     * This class loads a pre-calculated binary table of atmospheric properties
     * and provides an efficient, interpolating lookup function to retrieve
     * data for any given altitude.
     */
    class AtmosphereManager {
    public:
        AtmosphereManager() = default;

        /**
         * @brief Loads the atmospheric data from a binary file.
         * @param filepath The path to the 'atmosphere_table.bin' file.
         * @return True if loading was successful, false otherwise.
         */
        bool loadTable(const std::string& filepath);

        /**
         * @brief Retrieves atmospheric properties for a given altitude using linear interpolation.
         * @param altitude The altitude in meters.
         * @return An AtmosphereProperties struct with the interpolated data.
         */
        [[nodiscard]] AtmosphereProperties getProperties(double altitude) const;

        /**
         * @brief Gets the atmospheric transmissivity for a given path.
         * @param range_m The length of the path through the atmosphere.
         * @param altitude_m The altitude of the sensor.
         * @param band The IR wavelength band of the sensor.
         * @return The transmissivity factor (0.0 to 1.0).
         */
        [[nodiscard]] static double getTransmissivity(double range_m, double altitude_m, IRWavelengthBand band) ;

        /**
         * @brief Checks if the data table has been successfully loaded.
         * @return True if the table is loaded and ready for use.
         */
        [[nodiscard]] bool isLoaded() const;

    private:
        std::vector<AtmosphereProperties> _table;
        // Future: Add data structures to hold transmissivity lookup tables.
    };
} // namespace StrikeEngine
