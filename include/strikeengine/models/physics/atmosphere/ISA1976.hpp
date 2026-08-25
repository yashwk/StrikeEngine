#pragma once
#include <strikeengine/models/physics/atmosphere/AtmosphereModel.hpp>
#include <cmath>
#include <vector>

namespace StrikeEngine::Models {

    struct ISALayer {
        double altitudeBase;
        double temperatureBase;
        double pressureBase;
        double lapseRate;
    };

    class ISA1976 : public AtmosphereModel {
    public:
        ISA1976() {
            // Hardcode standard ISA layers up to 86km
            layers = {
                {0.0, 288.15, 101325.0, -0.0065},
                {11000.0, 216.65, 22632.1, 0.0},
                {20000.0, 216.65, 5474.89, 0.001},
                {32000.0, 228.65, 868.019, 0.0028},
                {47000.0, 270.65, 110.906, 0.0},
                {51000.0, 270.65, 66.9389, -0.0028},
                {71000.0, 214.65, 3.95642, -0.002}
            };
        }

        AtmosphereState evaluate(double altitude) const override {
            constexpr double g = 9.80665;
            constexpr double R = 287.05;
            constexpr double GAMMA_AIR = 1.4;

            if (altitude >= 86000.0) altitude = 85999.0;
            if (altitude < 0.0) altitude = 0.0;

            const ISALayer* currentLayer = &layers.front();
            for (const auto& layer : layers) {
                if (altitude >= layer.altitudeBase) {
                    currentLayer = &layer;
                } else {
                    break;
                }
            }

            double temperature, pressure;
            const double altitudeDifference = altitude - currentLayer->altitudeBase;

            if (std::abs(currentLayer->lapseRate) < 1e-9) {
                temperature = currentLayer->temperatureBase;
                pressure = currentLayer->pressureBase * std::exp(-g * altitudeDifference / (R * temperature));
            } else {
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

    private:
        std::vector<ISALayer> layers;
    };

} // namespace StrikeEngine::Models
