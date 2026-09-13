#include <strikeengine/kernel/systems/CommandProcessor.hpp>

#include <algorithm>
namespace StrikeEngine::Kernel {

    void CommandProcessor::enqueueCommand(const SimulationCommand& cmd) {
        commandQueue.push_back(cmd);
    }

    void CommandProcessor::dropCommandsFor(std::size_t entityId) {
        commandQueue.erase(
            std::remove_if(commandQueue.begin(), commandQueue.end(),
                           [entityId](const SimulationCommand& cmd) {
                               return cmd.entityId == entityId;
                           }),
            commandQueue.end());
    }

    void CommandProcessor::process(GuidanceBlock& guidance, TrackBlock& tracks,
                                   double simTimeSec) {
        for (const auto& cmd : commandQueue) {
            if (cmd.entityId >= guidance.mode.size()) continue;
            const std::size_t id = cmd.entityId;
            guidance.mode[id] = cmd.mode;
            guidance.targetX[id] = cmd.targetX;
            guidance.targetY[id] = cmd.targetY;
            guidance.targetZ[id] = cmd.targetZ;
            guidance.targetVx[id] = cmd.targetVx;
            guidance.targetVy[id] = cmd.targetVy;
            guidance.targetVz[id] = cmd.targetVz;
            guidance.maxAccel[id] = cmd.maxAccel;
            if (cmd.waypointGain > 0.0 && id < guidance.waypointGain.size()) {
                guidance.waypointGain[id] = cmd.waypointGain;
            }
            guidance.targetAccelX[id] = cmd.targetAccelX;
            guidance.targetAccelY[id] = cmd.targetAccelY;
            guidance.targetAccelZ[id] = cmd.targetAccelZ;
            guidance.targetAccelAvailable[id] = cmd.targetAccelAvailable;

            // Optional cooperative-datalink rewiring (see SimulationCommand):
            // handoff (disable), loss fallback (disable + coast on command
            // seed), and recovery (re-enable) are all queued mode-switch
            // commands so they stay deterministic with the step sequence.
            if (cmd.updateDatalink) {
                if (id < guidance.datalinkSourceId.size()) {
                    guidance.datalinkSourceId[id] = cmd.datalinkSourceId;
                }
                if (id < guidance.datalinkTargetId.size()) {
                    guidance.datalinkTargetId[id] = cmd.datalinkTargetId;
                }
            }

            // An external command seeds (or refreshes) the persistent
            // target track (unless seedTrack is false — see SimulationCommand).
            if (cmd.seedTrack && id < tracks.size) {
                const int seedPolicy = id < tracks.seedPolicy.size()
                    ? tracks.seedPolicy[id] : 0;
                const TrackState ts = tracks.state[id];
                const bool measurementMaintained =
                    ts == TrackState::Acquire || ts == TrackState::Maintain ||
                    ts == TrackState::Reacquire;
                bool applySeed = true;
                if (seedPolicy == 1) {
                    applySeed = ts == TrackState::None || ts == TrackState::Lost;
                } else if (seedPolicy == 2) {
                    applySeed = !measurementMaintained;
                }
                if (applySeed) {
                    tracks.state[id] = TrackState::Acquire;
                    tracks.trackId[id] = cmd.targetId;
                    tracks.posX[id] = cmd.targetX;
                    tracks.posY[id] = cmd.targetY;
                    tracks.posZ[id] = cmd.targetZ;
                    tracks.velX[id] = cmd.targetVx;
                    tracks.velY[id] = cmd.targetVy;
                    tracks.velZ[id] = cmd.targetVz;
                    tracks.accelX[id] = cmd.targetAccelX;
                    tracks.accelY[id] = cmd.targetAccelY;
                    tracks.accelZ[id] = cmd.targetAccelZ;
                    tracks.accelAvailable[id] = cmd.targetAccelAvailable;
                    // A command refresh restarts the quality/coast lifecycle, but
                    // measurement updates keep their continuity counters.
                    tracks.timestampSec[id] = simTimeSec;
                    tracks.ageSec[id] = 0.0;
                    tracks.positionStdM[id] = 5.0;
                    tracks.velocityStdMs[id] = 25.0;
                    tracks.quality01[id] = 1.0;
                    if (id < tracks.kfCov.size()) tracks.kfCov[id] = {};
                }
            }
        }
        commandQueue.clear();
    }

} // namespace StrikeEngine::Kernel
