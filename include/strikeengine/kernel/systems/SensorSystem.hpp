#pragma once

#include <strikeengine/kernel/data/PhysicsBlock.hpp>
#include <strikeengine/kernel/data/SensorBlock.hpp>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <random>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <deque>

namespace StrikeEngine::Kernel {

    class SensorSystem {
    public:
        SensorSystem();

        /**
         * @brief Updates all sensor measurements (IMU and GPS) based on true physics state.
         * @param physics The ground-truth physics state.
         * @param sensors The sensor block to populate with noisy measurements.
         * @param status The entity status block (sensor failure flags).
         * @param currentTime Current simulation time (used for GPS timing).
         * @param dt Timestep.
         */
        void update(
            const PhysicsBlock& physics,
            SensorBlock& sensors,
            const EntityStatusBlock& status,
            double currentTime,
            double dt,
            const EnvironmentConfig& environment = {}
        );

        /**
         * @brief Re-seed all stochastic models for reproducible runs.
         * Same seed + same scenario + same step sequence => identical
         * measurements. Without this, every run draws a fresh clock seed and
         * sweeps/Monte-Carlo studies are noise-dominated.
         */
        void setSeed(std::uint32_t seed);

        /**
         * @brief Draw a uniform random value in [0, 1) from the shared kernel
         * RNG stream. Re-seeded by setSeed() like the sensor models, so a fixed
         * seed makes every stochastic consumer of the stream deterministic.
         */
        double nextUniform01();

        /**
         * @brief Clear per-entity sensor state (streaming biases, GPS phase)
         * so a reset kernel starts clean without reseeding the RNG stream.
         */
        void reset();

    private:
        std::mt19937 rng;
        // Per-entity last GPS update time (s). GPS scheduling is now per-entity
        // (enabled flag + gpsUpdateRateHz in SensorBlock).
        std::vector<double> lastGpsUpdateTime;
        // Barometer / magnetometer scheduling + baro bias walk state.
        std::vector<double> lastBaroUpdateTime;
        std::vector<double> lastMagUpdateTime;
        std::vector<double> trueBaroBias;
        // Delayed GPS delivery: queued raw samples (truth + lever arm, noise
        // applied at generation) published once age >= gpsLatencySec.
        struct DelayedGpsSample {
            double timeSec = 0.0;
            double px = 0.0, py = 0.0, pz = 0.0;
            double vx = 0.0, vy = 0.0, vz = 0.0;
        };
        std::vector<std::deque<DelayedGpsSample>> gpsLatencyQueue;
        
        // Random walk biases (true biases drifting over time)
        // In a perfectly pure SoA, these true biases would live in another block (e.g. TrueSensorStateBlock),
        // but since only SensorSystem uses them, we'll keep them internal for now or put them in SensorBlock.
        // For now, we will store true biases in SensorBlock or internally. Let's add them to SensorSystem internally as arrays to keep SensorBlock strictly as "measurements".
        std::vector<double> trueAccelBiasX, trueAccelBiasY, trueAccelBiasZ;
        std::vector<double> trueGyroBiasX, trueGyroBiasY, trueGyroBiasZ;

        void ensureCapacity(std::size_t size);
    };

} // namespace StrikeEngine::Kernel
