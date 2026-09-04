#include <strikeengine/kernel/profiles/WarheadProfileDatabase.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

namespace StrikeEngine::Kernel {

    static FusingType parseFusing(const std::string& s) {
        if (s == "impact") return FusingType::Impact;
        if (s == "proximity") return FusingType::Proximity;
        if (s == "timed") return FusingType::Timed;
        throw std::invalid_argument("Unknown fusing type: " + s);
    }

    bool WarheadProfileDatabase::loadProfile(const std::string& file_path) {
        std::ifstream f(file_path);
        if (!f.is_open()) {
            return false;
        }

        try {
            nlohmann::json data = nlohmann::json::parse(f);

            WarheadConfig cfg;
            if (data.contains("mass_kg")) cfg.massKg = data.at("mass_kg").get<double>();
            else if (data.contains("massKg")) cfg.massKg = data.at("massKg").get<double>();

            if (data.contains("fusing")) {
                cfg.fusing = parseFusing(data.at("fusing").get<std::string>());
            }

            if (data.contains("proximity_trigger_m")) cfg.proximityTriggerM = data.at("proximity_trigger_m").get<double>();
            else if (data.contains("proximityTriggerM")) cfg.proximityTriggerM = data.at("proximityTriggerM").get<double>();

            if (data.contains("timed_delay_sec")) cfg.timedDelaySec = data.at("timed_delay_sec").get<double>();
            else if (data.contains("timedDelaySec")) cfg.timedDelaySec = data.at("timedDelaySec").get<double>();

            if (data.contains("lethal_radius_m")) cfg.lethalRadiusM = data.at("lethal_radius_m").get<double>();
            else if (data.contains("lethalRadiusM")) cfg.lethalRadiusM = data.at("lethalRadiusM").get<double>();

            if (data.contains("falloff_radius_m")) cfg.falloffRadiusM = data.at("falloff_radius_m").get<double>();
            else if (data.contains("falloffRadiusM")) cfg.falloffRadiusM = data.at("falloffRadiusM").get<double>();

            if (cfg.falloffRadiusM > 0.0 && cfg.falloffRadiusM < cfg.lethalRadiusM) {
                return false;
            }

            _warhead = cfg;
        } catch (const std::exception&) {
            return false;
        }

        return true;
    }

    const WarheadConfig& WarheadProfileDatabase::warhead() const {
        return _warhead;
    }

} // namespace StrikeEngine::Kernel
