#pragma once

#include <strikeengine/kernel/data/NavigationBlock.hpp>
#include <strikeengine/kernel/data/GuidanceBlock.hpp>
#include <strikeengine/kernel/data/ControlBlock.hpp>
#include <strikeengine/kernel/data/EntityStatusBlock.hpp>
#include <strikeengine/kernel/data/SeekerBlock.hpp>

namespace StrikeEngine::Kernel {

    /**
     * @brief Traceable, mode-aware guidance stack (W36).
     *
     * Runs after truth physics/sensors/navigation/seekers and before the
     * autopilot, one step later than the physics it commands. Phase selection
     * (None -> Midcourse -> Acquisition -> Terminal, with LostTrack recovery)
     * is separated from law computation (PureProNav / SeekerRateAPN /
     * AugmentedProNav). All demand output passes through the per-entity
     * maxAccel magnitude clamp, and diagnostics (phase, law, track id/age,
     * handoff weight, raw demand, limit/invalid/non-closing flags, tgo) are
     * published per entity each step.
     *
     * Legacy behavior is preserved when no W36 policy is configured:
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
            GuidanceBlock& guidance,
            ControlBlock& control,
            double dt
        );
    };

} // namespace StrikeEngine::Kernel
