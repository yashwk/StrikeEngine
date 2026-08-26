#pragma once

#include <strikeengine/kernel/data/NavigationBlock.hpp>
#include <strikeengine/kernel/data/SensorBlock.hpp>
#include <strikeengine/kernel/data/PhysicsBlock.hpp>
#include <cstddef>

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
            double dt);

    private:
        void ensureCapacity(std::size_t size, NavigationBlock& nav);
        void strapdownINS(std::size_t id, const SensorBlock& sensors, NavigationBlock& nav, double dt);
        void ekfUpdate(std::size_t id, const SensorBlock& sensors, NavigationBlock& nav);
    };

} // namespace StrikeEngine::Kernel
