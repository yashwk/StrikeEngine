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
                if (t.contains("cm_table")) {
                    cfg.tables.cmTable =
                        t.at("cm_table").get<std::vector<std::vector<double>>>();
                }
                if (t.contains("beta_breakpoints_rad")) {
                    cfg.tables.betaBreakpointsRad =
                        t.at("beta_breakpoints_rad").get<std::vector<double>>();
                }
                if (t.contains("cy_table")) {
                    cfg.tables.cyTable =
                        t.at("cy_table").get<std::vector<std::vector<double>>>();
                }
                if (t.contains("cn_table")) {
                    cfg.tables.cnTable =
                        t.at("cn_table").get<std::vector<std::vector<double>>>();
                }
                if (t.contains("cl_roll_table")) {
                    cfg.tables.clRollTable =
                        t.at("cl_roll_table").get<std::vector<std::vector<double>>>();
                }
                if (!cfg.tables.isValid()) {
                    return false;
                }
            }
            if (data.contains("fins")) {
                const auto& f = data.at("fins");
                const std::string shape = f.value("shape", std::string("trapezoidal"));
                if (shape == "trapezoidal") cfg.fins.shape = Models::FinShape::Trapezoidal;
                else if (shape == "elliptical") cfg.fins.shape = Models::FinShape::Elliptical;
                else if (shape == "freeform") cfg.fins.shape = Models::FinShape::FreeForm;
                else return false;
                cfg.fins.count = f.at("count").get<int>();
                cfg.fins.positionM = f.value("position_m", 0.0);
                cfg.fins.cantAngleDeg = f.value("cant_angle_deg", 0.0);
                cfg.fins.rootChordM = f.value("root_chord_m", 0.0);
                cfg.fins.spanM = f.value("span_m", 0.0);
                cfg.fins.tipChordM = f.value("tip_chord_m", 0.0);
                cfg.fins.sweepLengthM = f.value("sweep_length_m", -1.0);
                if (f.contains("shape_points")) {
                    cfg.fins.shapePoints = f.at("shape_points").get<std::vector<std::array<double, 2>>>();
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
