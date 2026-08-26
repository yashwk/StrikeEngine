#pragma once

#include <strikeengine/kernel/data/PhysicsBlock.hpp>
#include <strikeengine/kernel/data/SeekerBlock.hpp>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>
#include <strikeengine/models/signatures/RCSDatabase.hpp>
#include <strikeengine/models/signatures/IRSignatureDatabase.hpp>

#include <unordered_map>
#include <string>
#include <memory>
#include <deque>
#include <cstddef>

namespace StrikeEngine::Kernel {

    class SeekerSystem {
    public:
        void reset();

        void update(
            const PhysicsBlock& physics,
            const EntityStatusBlock& status,
            SeekerBlock& seeker,
            double dt
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
        };

        std::unordered_map<std::string, std::unique_ptr<Models::RCSDatabase>> rcsCache;
        std::unordered_map<std::string, std::unique_ptr<Models::IRSignatureDatabase>> irCache;
        std::vector<std::deque<DelayedMeasurement>> measurementHistory;
        std::size_t historyEntityCount = 0;
    };

} // namespace StrikeEngine::Kernel
