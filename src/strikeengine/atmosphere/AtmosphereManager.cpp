#include "strikeengine/atmosphere/AtmosphereManager.hpp"
#include <fstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace StrikeEngine {

    bool AtmosphereManager::loadTable(const std::string& filepath) {
        std::ifstream file(filepath, std::ios::binary);
        if (!file) {
            return false;
        }

        _table.clear();
        AtmosphereProperties entry{};
        while (file.read(reinterpret_cast<char*>(&entry), sizeof(entry))) {
            _table.push_back(entry);
        }

        return !_table.empty();
    }

    bool AtmosphereManager::isLoaded() const {
        return !_table.empty();
    }

    AtmosphereProperties AtmosphereManager::getProperties(double altitude) const {
        if (!isLoaded()) {
            throw std::runtime_error("AtmosphereManager error: Table not loaded.");
        }

        if (altitude <= _table.front().altitude) {
            return _table.front();
        }
        if (altitude >= _table.back().altitude) {
            return _table.back();
        }

        auto it = std::lower_bound(_table.begin(), _table.end(), altitude,
            [](const AtmosphereProperties& p, double alt) {
                return p.altitude < alt;
            });

        const auto& high = *it;
        const auto& low = *(--it);

        const double fraction = (altitude - low.altitude) / (high.altitude - low.altitude);

        AtmosphereProperties interpolated{};
        interpolated.altitude = altitude;
        interpolated.temperature = low.temperature + fraction * (high.temperature - low.temperature);
        interpolated.pressure = low.pressure + fraction * (high.pressure - low.pressure);
        interpolated.density = low.density + fraction * (high.density - low.density);
        interpolated.speedOfSound = low.speedOfSound + fraction * (high.speedOfSound - low.speedOfSound);

        return interpolated;
    }

    double AtmosphereManager::getTransmissivity(double range_m, double altitude_m, IRWavelengthBand band) {

        double absorption_coefficient;

        if (band == IRWavelengthBand::LongWave) {
            absorption_coefficient = 0.00012; // Higher absorption
        } else {
            absorption_coefficient = 0.00005; // Lower absorption
        }

        // The Beer-Lambert law is a good approximation: T = e^(-beta * range)
        double altitude_factor = std::exp(-altitude_m / 8000.0);

        double effective_coefficient = absorption_coefficient * altitude_factor;

        return std::exp(-effective_coefficient * range_m);
    }

} // namespace StrikeEngine
