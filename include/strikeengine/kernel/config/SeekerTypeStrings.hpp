#pragma once

#include <strikeengine/kernel/data/SeekerBlock.hpp>

#include <stdexcept>
#include <string>

namespace StrikeEngine::Kernel {

    // Shared snake_case string maps for SeekerType. Defined inline in this
    // header so every TU (ConfigSerialization and the profile databases) uses
    // one ODR-safe definition. No to_json/from_json for SeekerType is declared
    // anywhere: callers map the string explicitly.
    inline std::string seekerTypeToString(SeekerType t) {
        switch (t) {
            case SeekerType::None: return "none";
            case SeekerType::RF: return "rf";
            case SeekerType::IR: return "ir";
            case SeekerType::PassiveRF: return "passive_rf";
            case SeekerType::SARH: return "sarh";
        }
        throw std::runtime_error("seekerTypeToString: unhandled SeekerType");
    }

    inline SeekerType seekerTypeFromString(const std::string& s) {
        if (s == "none") return SeekerType::None;
        if (s == "rf") return SeekerType::RF;
        if (s == "ir") return SeekerType::IR;
        if (s == "passive_rf") return SeekerType::PassiveRF;
        if (s == "sarh") return SeekerType::SARH;
        throw std::runtime_error("seekerTypeFromString: unknown SeekerType string '" + s + "'");
    }

} // namespace StrikeEngine::Kernel
