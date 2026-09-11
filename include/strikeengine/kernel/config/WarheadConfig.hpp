#pragma once

#include <cstdint>

namespace StrikeEngine::Kernel {

    enum class FusingType : uint8_t { Impact, Proximity, Timed };

    struct WarheadConfig {
        double massKg = 0.0;
        FusingType fusing = FusingType::Impact;
        double proximityTriggerM = 0.0;
        double timedDelaySec = 0.0;
        double lethalRadiusM = 0.0;
        // Outer edge of the fragmentation/overpressure falloff band (m).
        // 0.0 (or <= lethalRadiusM) disables the band and restores the flat
        // lethal-radius kill law.
        double falloffRadiusM = 0.0;

        // --- Terminal fuse/damage options (defaults = legacy behavior) ------
        // Fuse master switch: false disables all fuzing for this warhead.
        bool fuseEnabled = true;
        // Analytic closest-approach projection. When enabled the proximity
        // fuse triggers on the projected time-to-CPA (within
        // fuseLookaheadSec) instead of the hard-coded 0.02 s range-rate
        // heuristic, and kill probability is evaluated on the projected miss
        // vector instead of the instantaneous range at detonation. This keeps
        // high-closing-speed passes from being penalized by the pre-CPA
        // range (legacy false).
        bool cpaFuzingEnabled = false;
        double fuseLookaheadSec = 0.02;
        // No detonation before this time of flight (arming/safety).
        double armingDelaySec = 0.0;
        // Minimum range-closing rate for a proximity trigger (m/s).
        // 0 disables the gate (the legacy fuse also triggers while receding).
        double minClosingSpeedMps = 0.0;
        // Self-destruct time of flight (s). 0 = never. On expiry the warhead
        // detonates and destroys its own carrier without applying lethality.
        double selfDestructTimeSec = 0.0;
        // Damage applied per lethal hit. 100 = legacy all-or-nothing kill;
        // lower values accumulate with the target's structural hardness.
        double damage = 100.0;
        // Per-step probability that the proximity fuze detects a target in
        // its trigger volume. Values < 1 use the dedicated fuze RNG stream
        // (the warhead lethality stream is untouched).
        double fuseDetectionProbability = 1.0;
        // Aspect-dependent lethality multipliers. Fragment/overpressure
        // effectiveness is interpolated between head-on and tail-on from the
        // cosine between carrier and target velocity:
        //   factor = avg + (head - tail)/2 * (-cos)
        // Both 1.0 = isotropic (legacy).
        double headOnLethalityFactor = 1.0;
        double tailOnLethalityFactor = 1.0;
    };

} // namespace StrikeEngine::Kernel
