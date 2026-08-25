#pragma once

#include "../data/PhysicsBlock.hpp"
#include "../data/SeekerBlock.hpp"
#include "../data/EntityStatusBlock.hpp"
#include "../../models/signatures/RCSDatabase.hpp"
#include "../../models/signatures/IRSignatureDatabase.hpp"

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
