#include <strikeengine/kernel/profiles/AeroProfileDatabase.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace StrikeEngine::Kernel {

    namespace {
        // Shared with the `fins` legacy block and each `fin_sets` entry so
        // both paths accept the same snake_case schema as
        // ConfigSerialization::finConfigFromJson. Throws on unknown shape or
        // a missing required key (translated to a load failure below).
        FinsConfig finsConfigFromJson(const nlohmann::json& f) {
            FinsConfig fin;
            const std::string shape = f.value("shape", std::string("trapezoidal"));
            if (shape == "trapezoidal") fin.shape = Models::FinShape::Trapezoidal;
            else if (shape == "elliptical") fin.shape = Models::FinShape::Elliptical;
            else if (shape == "freeform") fin.shape = Models::FinShape::FreeForm;
            else throw std::runtime_error("AeroProfileDatabase: unknown fins shape '" + shape + "'");
            fin.count = f.at("count").get<int>();
            fin.positionM = f.value("position_m", 0.0);
            fin.cantAngleDeg = f.value("cant_angle_deg", 0.0);
            fin.rootChordM = f.value("root_chord_m", 0.0);
            fin.spanM = f.value("span_m", 0.0);
            fin.tipChordM = f.value("tip_chord_m", 0.0);
            fin.sweepLengthM = f.value("sweep_length_m", -1.0);
            fin.steerable = f.value("steerable", true);
            if (f.contains("shape_points")) {
                fin.shapePoints = f.at("shape_points").get<std::vector<std::array<double, 2>>>();
            }
            return fin;
        }

        void airframeConfigFromJson(const nlohmann::json& af, AeroConfig& cfg) {
            // Same required-key schema as the inline ConfigSerialization path.
            cfg.airframe.wingSpanM       = af.at("wing_span_m").get<double>();
            cfg.airframe.wingRootChordM  = af.at("wing_root_chord_m").get<double>();
            cfg.airframe.wingTipChordM   = af.at("wing_tip_chord_m").get<double>();
            cfg.airframe.wingSweepDeg    = af.at("wing_sweep_deg").get<double>();
            cfg.airframe.wingPositionM   = af.at("wing_position_m").get<double>();
            cfg.airframe.wingDihedralDeg = af.at("wing_dihedral_deg").get<double>();
            cfg.airframe.htailSpanM      = af.at("htail_span_m").get<double>();
            cfg.airframe.htailChordM     = af.at("htail_chord_m").get<double>();
            cfg.airframe.htailPositionM  = af.at("htail_position_m").get<double>();
            cfg.airframe.vtailSpanM      = af.at("vtail_span_m").get<double>();
            cfg.airframe.vtailChordM     = af.at("vtail_chord_m").get<double>();
            cfg.airframe.vtailPositionM  = af.at("vtail_position_m").get<double>();
            cfg.airframe.fuselageDiameterM = af.at("fuselage_diameter_m").get<double>();
            cfg.airframe.fuselageLengthM   = af.at("fuselage_length_m").get<double>();
            cfg.airframe.cd0              = af.at("cd0").get<double>();
            cfg.airframe.oswaldEfficiency = af.at("oswald_efficiency").get<double>();
            cfg.airframe.clMax            = af.at("cl_max").get<double>();
        }
    }

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
                airframeConfigFromJson(data.at("airframe"), cfg);
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
