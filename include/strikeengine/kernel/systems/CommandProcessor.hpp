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
        // Optional Waypoint-law gain override (> 0 applies). The Waypoint law
        // commands a fixed magnitude a = waypointGain * unit(aim - pos); an
        // external director that wants to command an exact acceleration aims
        // along the demand direction and transfers the magnitude through
        // maxAccel, which needs a gain above any expected demand. 0 preserves
        // the vehicle's configured gain (legacy behavior).
        double waypointGain = 0.0;
        // Target acceleration for APN feed-forward (augmented PN). Consumed
        // only when targetAccelAvailable is true; otherwise the law explicitly
        // falls back to pure PN (never reads an uninitialized value).
        double targetAccelX = 0.0;
        double targetAccelY = 0.0;
        double targetAccelZ = 0.0;
        bool   targetAccelAvailable = false;

        // Optional target identity for the persistent track.
        // -1 = unknown identity (track is still maintained on state only).
        std::int64_t targetId = -1;

        // Seed the persistent target track from this command's target state.
        // Default true (legacy: commands launch/refresh tracks). Per-step
        // external demand commands (e.g. an app-side guidance override) MUST
        // set this false: re-seeding every step pins the track to the
        // command's aim state, which starves the terminal chain's
        // seeker/track fusion of measurements.
        bool seedTrack = true;

        // Optional cooperative-datalink re-targeting. When
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
        // seeding the persistent target track.
        void process(GuidanceBlock& guidance, TrackBlock& tracks, double simTimeSec);

        // Drop queued (not yet applied) commands for an entity, e.g. when its
        // slot is freed: applying them later would silently re-arm whatever
        // vehicle reuses the id.
        void dropCommandsFor(std::size_t entityId);

        // Drop the whole queue. Kernel reset must call this: a command queued
        // before reset would otherwise be applied after it, against whatever
        // occupies the entity id in the new run.
        void reset() { commandQueue.clear(); }

    private:
        std::vector<SimulationCommand> commandQueue;
    };

} // namespace StrikeEngine::Kernel
