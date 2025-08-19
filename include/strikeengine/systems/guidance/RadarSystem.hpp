#pragma once

#include "strikeengine/ecs/System.hpp"
#include "strikeengine/ecs/Registry.hpp"
#include "strikeengine/flight/RCSDatabase.hpp"
#include <string>
#include <memory>
#include <unordered_map>

namespace StrikeEngine {

    class RadarSystem final : public System {
    public:
        void update(Registry& registry, double dt) override;

    private:
        // cache
        std::unordered_map<std::string, std::unique_ptr<RCSDatabase>> _rcs_database_cache;
    };

} // namespace StrikeEngine
