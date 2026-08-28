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
    };

} // namespace StrikeEngine::Kernel
