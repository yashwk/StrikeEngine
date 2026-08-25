#pragma once

#include <strikeengine/kernel/data/PhysicsBlock.hpp>
#include <strikeengine/kernel/data/SeekerBlock.hpp>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>
#include <strikeengine/models/signatures/RCSDatabase.hpp>
#include <strikeengine/models/signatures/IRSignatureDatabase.hpp>

#include <unordered_map>
#include <string>
#include <memory>

namespace StrikeEngine::Kernel {

    class SeekerSystem {
    public:
        void update(
            const PhysicsBlock& physics,
            const EntityStatusBlock& status,
            SeekerBlock& seeker,
            double dt
        );

    private:
        std::unordered_map<std::string, std::unique_ptr<Models::RCSDatabase>> rcsCache;
        std::unordered_map<std::string, std::unique_ptr<Models::IRSignatureDatabase>> irCache;
    };

} // namespace StrikeEngine::Kernel
