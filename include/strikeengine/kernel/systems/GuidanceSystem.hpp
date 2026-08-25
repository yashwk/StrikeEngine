#pragma once

#include <strikeengine/kernel/data/NavigationBlock.hpp>
#include <strikeengine/kernel/data/GuidanceBlock.hpp>
#include <strikeengine/kernel/data/ControlBlock.hpp>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>
#include <strikeengine/kernel/data/SeekerBlock.hpp>

namespace StrikeEngine::Kernel {

    class GuidanceSystem {
    public:
        // Core execution function for the system.
        // It iterates over all active entities and processes their guidance logic.
        void update(
            const EntityStatusBlock& status,
            const NavigationBlock& nav,
            const SeekerBlock& seeker,
            GuidanceBlock& guidance,
            ControlBlock& control,
            double dt
        );

    private:
        void updateProNav(
            std::size_t id,
            const NavigationBlock& nav,
            GuidanceBlock& guidance
        );

        void updateWaypoint(
            std::size_t id,
            const NavigationBlock& nav,
            GuidanceBlock& guidance
        );

        void updateSeekerAPN(
            std::size_t id,
            const NavigationBlock& nav,
            const SeekerBlock& seeker,
            GuidanceBlock& guidance
        );
    };

} // namespace StrikeEngine::Kernel
