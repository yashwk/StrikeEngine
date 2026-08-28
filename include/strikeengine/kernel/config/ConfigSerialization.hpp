#pragma once

#include <string>
#include <strikeengine/kernel/config/VehicleConfig.hpp>
#include <strikeengine/kernel/config/ScenarioConfig.hpp>

namespace StrikeEngine::Kernel {

    // JSON (de)serialization for the vehicle config subsystem structures.
    // All functions operate on JSON text; implementations live in the
    // library .cpp and never leak <nlohmann/json.hpp> into public headers.
    // Deserializers throw std::runtime_error with a descriptive message on
    // malformed JSON or unknown enum strings.

    std::string serializeVehicleConfig(const VehicleConfig& config);
    VehicleConfig deserializeVehicleConfig(const std::string& jsonText);

    // Serializes/deserializes only the "earth" block of the environment; the
    // terrain/wind callbacks become the default flat/zero environment on load.
    std::string serializeEnvironment(const EnvironmentConfig& environment);
    EnvironmentConfig deserializeEnvironment(const std::string& jsonText);

    std::string serializeScenario(const ScenarioConfig& scenario);
    ScenarioConfig deserializeScenario(const std::string& jsonText);

    // Design file: {"name", "geometry", "physics"}.
    std::string serializeDesign(const std::string& name,
                                const std::string& geometryJson,
                                const VehicleConfig& physics);
    VehicleConfig loadDesignPhysics(const std::string& filePath);  // reads "physics"

} // namespace StrikeEngine::Kernel
