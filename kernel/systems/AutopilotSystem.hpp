#pragma once

#include "../data/GuidanceBlock.hpp"
#include "../data/NavigationBlock.hpp"
#include "../data/ControlBlock.hpp"
#include "../data/EntityStatusBlock.hpp"

namespace StrikeEngine::Kernel {

    class AutopilotSystem {
    public:
        /**
         * @brief Updates the actuator commands based on guidance commanded acceleration.
         * @param status Entity status block.
         * @param nav The estimated state (used for attitude and body rates).
         * @param guidance The commanded accelerations from Guidance.
         * @param control The control block to populate with pitch/yaw commands.
         * @param dt Timestep.
         */
        void update(
            const EntityStatusBlock& status,
            const NavigationBlock& nav,
            const GuidanceBlock& guidance,
            ControlBlock& control,
            double dt
        );
        
    private:
        void updateFlightController(
            std::size_t id,
            const NavigationBlock& nav,
            const GuidanceBlock& guidance,
            ControlBlock& control
        );
    };

} // namespace StrikeEngine::Kernel
