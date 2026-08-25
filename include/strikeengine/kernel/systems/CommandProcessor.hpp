#pragma once

#include <vector>
#include <cstddef>
#include <strikeengine/kernel/data/GuidanceBlock.hpp>

namespace StrikeEngine::Kernel {

    // Represents an external command injected into the simulation
    struct SimulationCommand {
        std::size_t entityId;
        GuidanceMode mode;
        double targetX;
        double targetY;
        double targetZ;
        double targetVx = 0.0;
        double targetVy = 0.0;
        double targetVz = 0.0;
        double maxAccel = 0.0;   // m/s^2; guidance demand magnitude limit
                                 // (0 = unlimited). Real guidance laws shape
                                 // commanded g; unbounded demands over-drive
                                 // the fins into saturation.
    };

    class CommandProcessor {
    public:
        // Queue a command to be executed
        void enqueueCommand(const SimulationCommand& cmd);

        // Process all queued commands, applying them to the data blocks
        void process(GuidanceBlock& guidance);

    private:
        std::vector<SimulationCommand> commandQueue;
    };

} // namespace StrikeEngine::Kernel
