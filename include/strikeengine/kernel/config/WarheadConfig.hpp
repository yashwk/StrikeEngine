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
    };

} // namespace StrikeEngine::Kernel
