#pragma once

#include "strikeengine/ecs/Component.hpp"

namespace StrikeEngine {
    /**
     * @brief Defines the common infrared wavelength bands for sensors.
     */
    enum class IRWavelengthBand {
        MidWave, // MWIR (3-5 micrometers) - Good for detecting hot engine plumes.
        LongWave // LWIR (8-12 micrometers) - Good for detecting cooler targets like airframes.
    };

    /**
     * @brief Stores the physical hardware parameters of an IR seeker.
     */
    struct InfraredSeekerComponent final : public Component {
        /**
         * @brief The sensitivity of the detector, measured as the minimum amount
         * of thermal energy required for a lock, in Watts.
         */
        double sensitivity_W;

        /**
         * @brief The seeker's field of view, in degrees.
         */
        double field_of_view_deg;

        /**
         * @brief The operational wavelength band of the seeker's detector.
         */
        IRWavelengthBand wavelength_band;
    };
} // namespace StrikeEngine
