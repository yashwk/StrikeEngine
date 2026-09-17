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

    bool WarheadProfileDatabase::loadProfile(const std::string& file_path, std::string* error) {
        std::ifstream f(file_path);
        if (!f.is_open()) {
            if (error) *error = "cannot open file";
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

            // Optional terminal fuse/damage keys (defaults preserve legacy).
            auto getDouble = [&](const char* snake, const char* camel, double def) -> double {
                if (data.contains(snake)) return data.at(snake).get<double>();
                if (data.contains(camel)) return data.at(camel).get<double>();
                return def;
            };
            auto getBool = [&](const char* snake, const char* camel, bool def) -> bool {
                if (data.contains(snake)) return data.at(snake).get<bool>();
                if (data.contains(camel)) return data.at(camel).get<bool>();
                return def;
            };
            cfg.fuseEnabled = getBool("fuse_enabled", "fuseEnabled", cfg.fuseEnabled);
            cfg.cpaFuzingEnabled = getBool("cpa_fuzing_enabled", "cpaFuzingEnabled", cfg.cpaFuzingEnabled);
            cfg.fuseLookaheadSec = getDouble("fuse_lookahead_sec", "fuseLookaheadSec", cfg.fuseLookaheadSec);
            cfg.armingDelaySec = getDouble("arming_delay_sec", "armingDelaySec", cfg.armingDelaySec);
            cfg.minClosingSpeedMps = getDouble("min_closing_speed_mps", "minClosingSpeedMps", cfg.minClosingSpeedMps);
            cfg.selfDestructTimeSec = getDouble("self_destruct_time_sec", "selfDestructTimeSec", cfg.selfDestructTimeSec);
            cfg.damage = getDouble("damage", "damage", cfg.damage);
            cfg.fuseDetectionProbability = getDouble("fuse_detection_probability", "fuseDetectionProbability", cfg.fuseDetectionProbability);
            cfg.headOnLethalityFactor = getDouble("head_on_lethality_factor", "headOnLethalityFactor", cfg.headOnLethalityFactor);
            cfg.tailOnLethalityFactor = getDouble("tail_on_lethality_factor", "tailOnLethalityFactor", cfg.tailOnLethalityFactor);

            if (cfg.falloffRadiusM > 0.0 && cfg.falloffRadiusM < cfg.lethalRadiusM) {
                if (error) *error = "falloff_radius_m is below lethal_radius_m";
                return false;
            }

            _warhead = cfg;
        } catch (const std::exception& e) {
            if (error) *error = e.what();
            return false;
        }

        return true;
    }

    const WarheadConfig& WarheadProfileDatabase::warhead() const {
        return _warhead;
    }

} // namespace StrikeEngine::Kernel
