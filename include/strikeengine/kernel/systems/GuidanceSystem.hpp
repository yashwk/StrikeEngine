#pragma once

#include <strikeengine/kernel/data/NavigationBlock.hpp>
#include <strikeengine/kernel/data/TrackBlock.hpp>
#include <strikeengine/kernel/data/GuidanceBlock.hpp>
#include <strikeengine/kernel/data/ControlBlock.hpp>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>
#include <strikeengine/kernel/data/SeekerBlock.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>

namespace StrikeEngine::Kernel {

    /**
     * @brief Traceable, mode-aware guidance stack.
     *
     * Runs after truth physics/sensors/navigation/seekers and before the
     * autopilot, one step later than the physics it commands. Phase selection
     * (None -> Midcourse -> Acquisition -> Terminal, with LostTrack recovery)
     * is a source+blend weight over the shared PN kernel (Tpn / Apn).
     * All demand output passes through the per-entity
     * maxAccel magnitude clamp, and diagnostics (phase, law, track id/age,
     * handoff weight, raw demand, limit/invalid/non-closing flags, tgo) are
     * published per entity each step.
     *
     * Legacy behavior is preserved when no phase policy is configured:
     *   - handoffBlendTimeSec <= 0: seeker lock overrides to APN instantly
     *   - lockLossRetentionSec <= 0: no guidance-layer retention past the
     *     seeker's own dropout logic
     *   - apnFeedforwardEnabled false: pure PN midcourse
     */
    class GuidanceSystem {
    public:
        void update(
            const EntityStatusBlock& status,
            const NavigationBlock& nav,
            const SeekerBlock& seeker,
            const TrackBlock& tracks,
            GuidanceBlock& guidance,
            ControlBlock& control,
            double dt,
            const EnvironmentConfig& environment
        );
    };

} // namespace StrikeEngine::Kernel
