#pragma once

// Internal aero schema parsing, shared by the two paths that accept the same
// snake_case aero schema: the inline VehicleConfig document
// (ConfigSerialization) and standalone aero profile files
// (AeroProfileDatabase). Before this existed each path had its own copy, so a
// schema change could silently apply to only one of them.
//
// Private to the library: it requires <nlohmann/json.hpp>, which must not leak
// into an installed header (see the dependency note in CMakeLists.txt).

#include <nlohmann/json.hpp>
#include <strikeengine/kernel/config/AeroConfig.hpp>

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace StrikeEngine::Kernel::AeroSchema {

    /**
     * @brief Parses a fin set (a `fins` block or one `fin_sets` entry).
     * @param context Caller name used in the error message.
     * @throws std::runtime_error on an unknown shape or a missing `count`.
     */
    inline FinsConfig finsFromJson(const nlohmann::json& f, const char* context)
    {
        FinsConfig fin;
        const std::string shape = f.value("shape", std::string("trapezoidal"));
        if (shape == "trapezoidal") fin.shape = Models::FinShape::Trapezoidal;
        else if (shape == "elliptical") fin.shape = Models::FinShape::Elliptical;
        else if (shape == "freeform") fin.shape = Models::FinShape::FreeForm;
        else {
            throw std::runtime_error(std::string(context) +
                                     ": unknown fins shape '" + shape + "'");
        }
        fin.count = f.at("count").get<int>();
        fin.positionM = f.value("position_m", 0.0);
        fin.cantAngleDeg = f.value("cant_angle_deg", 0.0);
        fin.rootChordM = f.value("root_chord_m", 0.0);
        fin.spanM = f.value("span_m", 0.0);
        fin.tipChordM = f.value("tip_chord_m", 0.0);
        fin.sweepLengthM = f.value("sweep_length_m", -1.0);
        fin.steerable = f.value("steerable", true);
        if (f.contains("shape_points")) {
            fin.shapePoints =
                f.at("shape_points").get<std::vector<std::array<double, 2>>>();
        }
        return fin;
    }

    /**
     * @brief Parses an `airframe` block into @p out.
     * @throws std::runtime_error if a required key is missing.
     */
    inline void airframeFromJson(const nlohmann::json& af, AirframeConfig& out)
    {
        out.wingSpanM       = af.at("wing_span_m").get<double>();
        out.wingRootChordM  = af.at("wing_root_chord_m").get<double>();
        out.wingTipChordM   = af.at("wing_tip_chord_m").get<double>();
        out.wingSweepDeg    = af.at("wing_sweep_deg").get<double>();
        out.wingPositionM   = af.at("wing_position_m").get<double>();
        out.wingDihedralDeg = af.at("wing_dihedral_deg").get<double>();
        out.htailSpanM      = af.at("htail_span_m").get<double>();
        out.htailChordM     = af.at("htail_chord_m").get<double>();
        out.htailPositionM  = af.at("htail_position_m").get<double>();
        out.vtailSpanM      = af.at("vtail_span_m").get<double>();
        out.vtailChordM     = af.at("vtail_chord_m").get<double>();
        out.vtailPositionM  = af.at("vtail_position_m").get<double>();
        out.fuselageDiameterM = af.at("fuselage_diameter_m").get<double>();
        out.fuselageLengthM   = af.at("fuselage_length_m").get<double>();
        out.cd0              = af.at("cd0").get<double>();
        out.oswaldEfficiency = af.at("oswald_efficiency").get<double>();
        out.clMax            = af.at("cl_max").get<double>();
    }

} // namespace StrikeEngine::Kernel::AeroSchema
