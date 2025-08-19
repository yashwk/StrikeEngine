#pragma once

#include "strikeengine/ecs/Component.hpp"

namespace StrikeEngine {

    /**
     * @brief Stores the physical hardware parameters of a radar antenna and receiver.
     */
    struct AntennaComponent final : public Component {
        /**
         * @brief The power of the radar's transmitter, in Watts.
         */
        double transmitter_power_W = 10000.0;

        /**
         * @brief The gain of the antenna, in decibels (dB). Measures how well the
         * antenna focuses energy.
         */
        double antenna_gain_dB = 30.0;

        /**
         * @brief The operational wavelength of the radar signal, in meters.
         * (e.g., 0.03 m for a 10 GHz X-band radar).
         */
        double wavelength_m = 0.03;

        /**
         * @brief The minimum power the receiver can detect, also known as the
         * noise floor, in Watts.
         */
        double noise_floor_W = 1e-12;

        /**
         * @brief The minimum Signal-to-Noise Ratio (SNR), in decibels (dB), required
         * by the signal processor to declare a valid detection or track.
         */
        double snr_threshold_dB = 13.0;
    };

} // namespace StrikeEngine
