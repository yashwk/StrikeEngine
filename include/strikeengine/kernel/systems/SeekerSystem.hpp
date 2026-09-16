#pragma once

#include <strikeengine/kernel/data/PhysicsBlock.hpp>
#include <strikeengine/kernel/data/SeekerBlock.hpp>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>
#include <strikeengine/kernel/data/NavigationBlock.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <strikeengine/models/signatures/RCSDatabase.hpp>
#include <strikeengine/models/signatures/IRSignatureDatabase.hpp>

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <memory>
#include <deque>
#include <random>
#include <cstddef>
#include <cstdint>

namespace StrikeEngine::Kernel {

    class SeekerSystem {
    public:
        void reset();

        // Seed the deterministic measurement-noise / Swerling / duty stream.
        // Draws only happen when the corresponding opt-in model is enabled,
        // so legacy runs never touch this stream.
        void setSeed(std::uint32_t seed);

        void update(
            const PhysicsBlock& physics,
            const EntityStatusBlock& status,
            SeekerBlock& seeker,
            const NavigationBlock& nav,
            double dt,
            const EnvironmentConfig& environment = EnvironmentConfig{}
        );

    private:
        struct DelayedMeasurement {
            std::size_t target = 0;
            double age = 0.0;
            double range = 0.0;
            double rangeRate = 0.0;
            double azimuth = 0.0;
            double elevation = 0.0;
            double azimuthRate = 0.0;
            double elevationRate = 0.0;
            // Body rate averaged over the SAME interval as the angle rates.
            // The gyro decoupling adds it to the frame rate, so publishing the
            // rates without it left a jerk-sized residual (the body rate moved
            // during the measurement latency) -- larger than the LOS rate
            // itself on a maneuvering long-range round.
            double bodyRateX = 0.0;
            double bodyRateY = 0.0;
            double bodyRateZ = 0.0;
            // Measured LOS unit vectors at capture time. Keeping both frames
            // lets the legacy body-rate fallback remain available while the
            // preferred inertial rate differentiates the measurement in the
            // frame it was actually captured in.
            double losBodyX = 0.0;
            double losBodyY = 0.0;
            double losBodyZ = 0.0;
            double losWorldX = 0.0;
            double losWorldY = 0.0;
            double losWorldZ = 0.0;
            double timeSec = 0.0;
        };

        std::unordered_map<std::string, std::unique_ptr<Models::RCSDatabase>> rcsCache;
        std::unordered_map<std::string, std::unique_ptr<Models::IRSignatureDatabase>> irCache;
        // Profile ids that failed to load (warned once): RF/SARH evaluation
        // skips these targets, so a missing file would otherwise blind the
        // seeker with zero diagnostic.
        std::unordered_set<std::string> rcsLoadFailed;
        std::vector<std::deque<DelayedMeasurement>> measurementHistory;
        std::size_t historyEntityCount = 0;

        // Deterministic stream for measurement noise / Swerling / duty.
        std::mt19937 rng{0x5EE7E9u};
    };

} // namespace StrikeEngine::Kernel
