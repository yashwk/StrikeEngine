#include "CommandProcessor.hpp"

namespace StrikeEngine::Kernel {

    void CommandProcessor::enqueueCommand(const SimulationCommand& cmd) {
        commandQueue.push_back(cmd);
    }

    void CommandProcessor::process(GuidanceBlock& guidance) {
        for (const auto& cmd : commandQueue) {
            if (cmd.entityId < guidance.mode.size()) {
                guidance.mode[cmd.entityId] = cmd.mode;
                guidance.targetX[cmd.entityId] = cmd.targetX;
                guidance.targetY[cmd.entityId] = cmd.targetY;
                guidance.targetZ[cmd.entityId] = cmd.targetZ;
                guidance.targetVx[cmd.entityId] = cmd.targetVx;
                guidance.targetVy[cmd.entityId] = cmd.targetVy;
                guidance.targetVz[cmd.entityId] = cmd.targetVz;
            }
        }
        commandQueue.clear();
    }

} // namespace StrikeEngine::Kernel
