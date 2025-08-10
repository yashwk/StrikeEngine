#pragma once

#include <string>
#include <vector>

namespace StrikeEngine {
    /**
     * @brief A data structure to hold atmospheric properties at a specific altitude.
     * This is the single source of truth for this struct.
     */
    struct AtmosphereProperties {
        double altitude;
        double temperature;
        double pressure;
        double density;
        double speedOfSound;
    };

    /**
     * @brief Represents a single atmospheric layer, loaded from JSON.
     */
    struct AtmosphereLayer {
        double altitudeBase;
        double temperatureBase;
        double pressureBase;
        double lapseRate;
    };

    /**
     * @brief Loads atmospheric layer definitions from a JSON file.
     * @param filepath Path to the JSON file.
     * @return A vector of AtmosphereLayer structs.
     */
    std::vector<AtmosphereLayer> loadAtmosphereLayers(const std::string &filepath);

    /**
     * @brief Calculates atmospheric properties based on a given set of layers.
     * @param altitude The altitude in meters.
     * @param layers The atmospheric layer data loaded from JSON.
     * @return An AtmosphereProperties struct with the calculated data.
     */
    AtmosphereProperties calculateAtmosphere(double altitude, const std::vector<AtmosphereLayer> &layers);
} // namespace StrikeEngine
