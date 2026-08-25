#pragma once

#include <strikeengine/kernel/data/PhysicsBlock.hpp>
#include <strikeengine/kernel/data/SensorBlock.hpp>
#include <random>
#include <cstddef>
#include <vector>

namespace StrikeEngine::Kernel {

    class SensorSystem {
    public:
        SensorSystem();

        /**
         * @brief Updates all sensor measurements (IMU and GPS) based on true physics state.
         * @param physics The ground-truth physics state.
         * @param sensors The sensor block to populate with noisy measurements.
         * @param currentTime Current simulation time (used for GPS timing).
         * @param dt Timestep.
         */
        void update(
            const PhysicsBlock& physics,
            SensorBlock& sensors,
            double currentTime,
            double dt
        );

    private:
        std::mt19937 rng;
        double lastGpsUpdateTime = 0.0;
        double gpsUpdateRate = 1.0; // 1 Hz GPS update
        
        // Random walk biases (true biases drifting over time)
        // In a perfectly pure SoA, these true biases would live in another block (e.g. TrueSensorStateBlock),
        // but since only SensorSystem uses them, we'll keep them internal for now or put them in SensorBlock.
        // For now, we will store true biases in SensorBlock or internally. Let's add them to SensorSystem internally as arrays to keep SensorBlock strictly as "measurements".
        std::vector<double> trueAccelBiasX, trueAccelBiasY, trueAccelBiasZ;
        std::vector<double> trueGyroBiasX, trueGyroBiasY, trueGyroBiasZ;

        void ensureCapacity(std::size_t size);
    };

} // namespace StrikeEngine::Kernel
