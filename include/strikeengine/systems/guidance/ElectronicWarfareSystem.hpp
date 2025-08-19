#pragma once

#include "strikeengine/ecs/System.hpp"
#include "strikeengine/ecs/Registry.hpp"

namespace StrikeEngine {

    class ElectronicWarfareSystem final : public System {
    public:
        void update(Registry& registry, double dt) override;
    };

} // namespace StrikeEngine
