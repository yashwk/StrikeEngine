#pragma once

#include <strikeengine/kernel/data/SensorBlock.hpp>
#include <strikeengine/kernel/data/NavigationBlock.hpp>

namespace StrikeEngine::Kernel {

    class NavigationSystem {
    public:
        /**
         * @brief Updates the navigation estimates using INS integration and EKF fusion.
         * @param sensors The noisy sensor measurements.
         * @param nav The navigation estimates to update.
         * @param dt Timestep.
         */
        void update(
            const SensorBlock& sensors,
            NavigationBlock& nav,
            double dt
        );

    private:
        void ensureCapacity(std::size_t size, NavigationBlock& nav);
        void strapdownINS(std::size_t id, const SensorBlock& sensors, NavigationBlock& nav, double dt);
        void ekfUpdate(std::size_t id, const SensorBlock& sensors, NavigationBlock& nav);
    };

} // namespace StrikeEngine::Kernel
