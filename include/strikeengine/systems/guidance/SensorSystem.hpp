#pragma once

#include "strikeengine/ecs/System.hpp"
#include "strikeengine/ecs/Registry.hpp"
#include "strikeengine/flight/RCSDatabase.hpp"
#include "strikeengine/flight/IRSignatureDatabase.hpp"
#include <string>
#include <memory>
#include <unordered_map>

#include "include/strikeengine/ecs/System.hpp"

namespace StrikeEngine {
    class SensorSystem final : public System {
    public:
        void update(Registry& registry, double dt) override;

    private:
        // Caches to store loaded databases to avoid reading files every frame.
        std::unordered_map<std::string, std::unique_ptr<RCSDatabase>> _rcs_database_cache;
        std::unordered_map<std::string, std::unique_ptr<IRSignatureDatabase>> _ir_database_cache;
    };
} // namespace StrikeEngine
