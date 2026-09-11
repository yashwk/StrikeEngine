#pragma once

#include <strikeengine/kernel/data/NavigationBlock.hpp>
#include <strikeengine/kernel/data/SensorBlock.hpp>
#include <strikeengine/kernel/data/PhysicsBlock.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <cstddef>
#include <cstdint>
#include <random>

namespace StrikeEngine::Kernel {

    /**
     * @brief Strapdown INS + coupled error-state EKF navigation.
     *
     * Attitude is initialized from the truth quaternion at alignment
     * ("perfect initialization") and propagated from body gyro rates.
     */
    class NavigationSystem {
    public:
        void update(
            const SensorBlock& sensors,
            const PhysicsBlock& physics,
            NavigationBlock& nav,
            double dt,
            const EnvironmentConfig& environment = {});

        // Re-seed the alignment-error draws (see initialAttitudeErrorDeg and
        // friends). Same seed + same scenario + same step sequence stays
        // bit-identical; kernel reset re-seeds through setRandomSeed.
        void setSeed(std::uint32_t seed);

    private:
        // Alignment-error RNG stream (separate object; draws only happen when
        // initial-error sigmas are non-zero, so legacy runs never touch it).
        std::mt19937 alignRng{0xC0FFEEu};

    private:
        void ensureCapacity(std::size_t size, NavigationBlock& nav);
        void strapdownINS(std::size_t id, const SensorBlock& sensors, NavigationBlock& nav,
                          double dt, const EnvironmentConfig& environment);
        void ekfUpdate(std::size_t id, const SensorBlock& sensors, NavigationBlock& nav,
                       const EnvironmentConfig& environment);
    };

} // namespace StrikeEngine::Kernel
