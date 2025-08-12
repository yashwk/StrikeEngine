// ===================================================================================
//  PropulsionSystem.hpp
//
//  Description:
//  This header defines the PropulsionSystem.
//
//  Architectural Upgrade:
//  The system now requires an AtmosphereManager to calculate atmospheric effects
//  on engine performance (Isp).
//
// ===================================================================================

#pragma once

#include "strikeengine/ecs/System.hpp"
#include "strikeengine/ecs/Registry.hpp"

// Forward declaration
namespace StrikeEngine {
    class AtmosphereManager;
}

namespace StrikeEngine {

    class PropulsionSystem final : public System {
    public:
        /**
         * @brief Constructs the system, requiring an atmosphere manager.
         * @param atmosphereManager A reference to the simulation's atmosphere manager.
         */
        PropulsionSystem(const AtmosphereManager& atmosphereManager);
        ~PropulsionSystem() override;

        void update(Registry& registry, double dt) override;

    private:
        const AtmosphereManager& _atmosphereManager;
    };

} // namespace StrikeEngine
