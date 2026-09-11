#pragma once

#include <vector>
#include <cstddef>
#include <strikeengine/kernel/data/GuidanceBlock.hpp>
#include <strikeengine/kernel/data/TrackBlock.hpp>

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
        // Target acceleration for APN feed-forward (augmented PN). Consumed
        // only when targetAccelAvailable is true; otherwise the law explicitly
        // falls back to pure PN (never reads an uninitialized value).
        double targetAccelX = 0.0;
        double targetAccelY = 0.0;
        double targetAccelZ = 0.0;
        bool   targetAccelAvailable = false;

        // Optional target identity for the persistent track (W39).
        // -1 = unknown identity (track is still maintained on state only).
        std::int64_t targetId = -1;

        // Optional cooperative-datalink re-targeting (W41). When
        // updateDatalink is true, the command also rewires which source
        // entity's persistent track steers this entity's midcourse PN
        // (datalinkSourceId) and which target that track is of
        // (datalinkTargetId). -1 = disabled (autonomous: own seeker/command
        // only). Default false preserves legacy behavior: commands never
        // touch the datalink wiring set at createVehicle.
        bool updateDatalink = false;
        int datalinkSourceId = -1;
        int datalinkTargetId = -1;
    };

    class CommandProcessor {
    public:
        // Queue a command to be executed
        void enqueueCommand(const SimulationCommand& cmd);

        // Process all queued commands, applying them to the data blocks and
        // seeding the persistent target track (W39).
        void process(GuidanceBlock& guidance, TrackBlock& tracks, double simTimeSec);

        // Drop queued (not yet applied) commands for an entity, e.g. when its
        // slot is freed: applying them later would silently re-arm whatever
        // vehicle reuses the id.
        void dropCommandsFor(std::size_t entityId);

    private:
        std::vector<SimulationCommand> commandQueue;
    };

} // namespace StrikeEngine::Kernel
