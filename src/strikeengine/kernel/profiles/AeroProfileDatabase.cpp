#include <strikeengine/kernel/profiles/AeroProfileDatabase.hpp>
#include <nlohmann/json.hpp>
#include <fstream>

namespace StrikeEngine::Kernel {

    bool AeroProfileDatabase::loadProfile(const std::string& file_path) {
        std::ifstream f(file_path);
        if (!f.is_open()) {
            return false;
        }

        try {
            nlohmann::json data = nlohmann::json::parse(f);

            // Start from the struct defaults; every key is optional.
            AeroConfig cfg;
            cfg.referenceArea   = data.value("reference_area", cfg.referenceArea);
            cfg.referenceLength = data.value("reference_length", cfg.referenceLength);
            cfg.cd              = data.value("cd", cfg.cd);
            cfg.clAlpha         = data.value("cl_alpha", cfg.clAlpha);
            cfg.clFin           = data.value("cl_fin", cfg.clFin);
            cfg.clMax           = data.value("cl_max", cfg.clMax);
            if (data.contains("aero_tables")) {
                const auto& t = data["aero_tables"];
                cfg.tables.machBreakpoints =
                    t.at("mach_breakpoints").get<std::vector<double>>();
                cfg.tables.aoaBreakpointsRad =
                    t.at("aoa_breakpoints_rad").get<std::vector<double>>();
                cfg.tables.clTable =
                    t.at("cl_table").get<std::vector<std::vector<double>>>();
                cfg.tables.cdTable =
                    t.at("cd_table").get<std::vector<std::vector<double>>>();
                if (!cfg.tables.isValid()) {
                    return false;
                }
            }
            _aero = cfg;
        } catch (const std::exception&) {
            // Any load failure (syntax, missing required key, wrong type)
            // makes the profile unloadable; createVehicle translates this
            // into a friendly std::runtime_error.
            return false;
        }

        return true;
    }

    const AeroConfig& AeroProfileDatabase::aero() const {
        return _aero;
    }

} // namespace StrikeEngine::Kernel
