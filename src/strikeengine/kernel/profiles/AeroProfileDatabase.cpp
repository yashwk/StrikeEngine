#include <strikeengine/kernel/profiles/AeroProfileDatabase.hpp>
#include <strikeengine/kernel/config/AeroSchema.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace StrikeEngine::Kernel {

    namespace {
        // Fin sets and the airframe block share one schema with the inline
        // VehicleConfig path; see AeroSchema.hpp. Throws on an unknown shape or
        // a missing required key (translated to a load failure below).
        FinsConfig finsConfigFromJson(const nlohmann::json& f) {
            return AeroSchema::finsFromJson(f, "AeroProfileDatabase");
        }
    }

    bool AeroProfileDatabase::loadProfile(const std::string& file_path, std::string* error) {
        std::ifstream f(file_path);
        if (!f.is_open()) {
            if (error) *error = "cannot open file";
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
            cfg.tailControl     = data.value("tail_control",
                                  data.value("tailControl", cfg.tailControl));
            cfg.clMax           = data.value("cl_max", cfg.clMax);
            if (data.contains("aero_tables")) {
                cfg.tables = AeroSchema::aeroTablesFromJson(data.at("aero_tables"),
                                                            "AeroProfileDatabase");
            }
            // Multiple geometric fin sets (canards + tails); `fins` is the
            // single-set legacy form. Both share one schema.
            if (data.contains("fin_sets") && data.at("fin_sets").is_array()) {
                for (const auto& item : data.at("fin_sets")) {
                    cfg.finSets.push_back(finsConfigFromJson(item));
                }
            }
            if (data.contains("fins")) {
                cfg.fins = finsConfigFromJson(data.at("fins"));
            }
            if (data.contains("airframe")) {
                AeroSchema::airframeFromJson(data.at("airframe"), cfg.airframe);
            }
            _aero = cfg;
        } catch (const std::exception& e) {
            if (error) *error = e.what();
            return false;
        }

        return true;
    }

    const AeroConfig& AeroProfileDatabase::aero() const {
        return _aero;
    }

} // namespace StrikeEngine::Kernel
