#pragma once

#include <strikeengine/kernel/data/NavigationBlock.hpp>
#include <strikeengine/kernel/data/TrackBlock.hpp>
#include <strikeengine/kernel/data/SeekerBlock.hpp>

namespace StrikeEngine::Kernel {

    /**
     * @brief Persistent target-track manager (W39).
     *
     * Unifies external command seeds and seeker LOS measurements into one
     * per-entity track: target identity, position/velocity/optional
     * acceleration, measurement timestamp + track age, quality/uncertainty,
     * and the Acquire -> Maintain -> Coast -> Lost -> Reacquire lifecycle.
     * Multi-rate prediction propagates the estimate kinematically at the
     * simulation rate between measurement arrivals (or while coasting).
     *
     * Frame/derivation rules (all deterministic):
     *  - seeker LOS body angles -> world LOS via the navigation estimate;
     *    measured position = nav position + LOS_world * range
     *  - velocity is a finite difference between consecutive measurement
     *    fixes (seeded by the external command velocity when provided)
     *  - uncertainty grows with age since the last measurement and quality
     *    decays exponentially; estimates are never taken from physics truth.
     *
     * Runs after seekers, before guidance (SimulationKernel::step). The
     * guidance layer consumes only these estimates.
     */
    class TrackManagerSystem {
    public:
        void update(
            const NavigationBlock& nav,
            const SeekerBlock& seeker,
            TrackBlock& tracks,
            double timeSec,
            double dt);
    };

} // namespace StrikeEngine::Kernel
