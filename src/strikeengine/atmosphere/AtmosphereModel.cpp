#include "strikeengine/atmosphere/AtmosphereModel.hpp"
#include "nlohmann/json.hpp"
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace StrikeEngine {
    using json = nlohmann::json;

    // Helper function to deserialize a Layer from JSON
    void from_json(const json &j, AtmosphereLayer &l) {
        j.at("altitude_base").get_to(l.altitudeBase);
        j.at("temperature_base").get_to(l.temperatureBase);
        j.at("pressure_base").get_to(l.pressureBase);
        j.at("lapse_rate").get_to(l.lapseRate);
    }


    std::vector<AtmosphereLayer> loadAtmosphereLayers(const std::string &filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open atmosphere layer definition file: " + filepath);
        }
        json data = json::parse(file);
        return data.at("layers").get<std::vector<AtmosphereLayer> >();
    }

    AtmosphereProperties calculateAtmosphere(double altitude, const std::vector<AtmosphereLayer> &layers) {
        constexpr double g = 9.80665;
        constexpr double R = 287.05;
        constexpr double GAMMA_AIR = 1.4;

        if (altitude >= 86000.0) altitude = 85999.0;
        if (altitude < 0.0) altitude = 0.0;

        const AtmosphereLayer *currentLayer = nullptr;
        for (const auto &layer: layers) {
            if (altitude >= layer.altitudeBase) {
                currentLayer = &layer;
            } else {
                break;
            }
        }

        if (!currentLayer) {
            throw std::runtime_error("Could not find appropriate atmospheric layer for altitude.");
        }

        double temperature, pressure;
        const double altitudeDifference = altitude - currentLayer->altitudeBase;

        if (std::abs(currentLayer->lapseRate) < 1e-9) {
            // Isothermal layer
            temperature = currentLayer->temperatureBase;
            pressure = currentLayer->pressureBase * std::exp(-g * altitudeDifference / (R * temperature));
        } else {
            // Gradient layer
            temperature = currentLayer->temperatureBase + currentLayer->lapseRate * altitudeDifference;
            pressure = currentLayer->pressureBase * std::pow(
                           currentLayer->temperatureBase / temperature,
                           g / (currentLayer->lapseRate * R)
                       );
        }

        const double density = pressure / (R * temperature);
        const double speedOfSound = std::sqrt(GAMMA_AIR * R * temperature);

        return {altitude, temperature, pressure, density, speedOfSound};
    }
} // namespace StrikeEngine
