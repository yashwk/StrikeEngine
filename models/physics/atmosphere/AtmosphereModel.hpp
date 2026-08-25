#pragma once

namespace StrikeEngine::Models {

    struct AtmosphereState {
        double altitude;
        double temperature;
        double pressure;
        double density;
        double speedOfSound;
    };

    class AtmosphereModel {
    public:
        virtual ~AtmosphereModel() = default;
        virtual AtmosphereState evaluate(double altitude) const = 0;
    };

} // namespace StrikeEngine::Models
