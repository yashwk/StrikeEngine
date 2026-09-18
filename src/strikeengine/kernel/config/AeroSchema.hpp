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
        else if (shape == "delta") fin.shape = Models::FinShape::Delta;
        else if (shape == "cranked") fin.shape = Models::FinShape::Cranked;
        else if (shape == "freeform") fin.shape = Models::FinShape::FreeForm;
        else {
            throw std::runtime_error(std::string(context) +
                                     ": unknown fins shape '" + shape + "'");
        }
        const std::string airfoil = f.value("airfoil", std::string("flat_plate"));
        if (airfoil == "flat_plate" || airfoil == "Flat Slab" || airfoil == "flat_slab") {
            fin.airfoil = Models::FinAirfoil::FlatPlate;
        } else if (airfoil == "double_wedge" || airfoil == "Double Wedge") {
            fin.airfoil = Models::FinAirfoil::DoubleWedge;
        } else if (airfoil == "biconvex" || airfoil == "Biconvex") {
            fin.airfoil = Models::FinAirfoil::Biconvex;
        } else if (airfoil == "hexagonal" || airfoil == "Hexagonal") {
            fin.airfoil = Models::FinAirfoil::Hexagonal;
        } else {
            throw std::runtime_error(std::string(context) +
                                     ": unknown fin airfoil '" + airfoil + "'");
        }
        fin.count = f.at("count").get<int>();
        fin.positionM = f.value("position_m", 0.0);
        fin.cantAngleDeg = f.value("cant_angle_deg", 0.0);
        fin.rootChordM = f.value("root_chord_m", 0.0);
        fin.spanM = f.value("span_m", 0.0);
        fin.tipChordM = f.value("tip_chord_m", 0.0);
        fin.sweepLengthM = f.value("sweep_length_m", -1.0);
        fin.steerable = f.value("steerable", true);
        fin.thicknessRatio = f.contains("thickness_ratio")
            ? f.value("thickness_ratio", 0.0)
            : f.value("thickness_chord_ratio", 0.0);
        fin.maxThicknessLocation = f.value("max_thickness_location", 0.5);
        fin.leadingEdgeRadius = f.value("leading_edge_radius", 0.0);
        fin.trailingEdgeThickness = f.value("trailing_edge_thickness", 0.0);
        fin.crankFraction = f.value("crank_fraction", 0.5);
        fin.crankChordFactor = f.value("crank_chord_factor", 0.5);
        fin.midChordLandFraction = f.value("mid_chord_land_fraction", 0.33);
        const std::string control = f.value("control_type", std::string("all_moving"));
        if (control == "all_moving") fin.controlType = 0;
        else if (control == "trailing_edge_flap") fin.controlType = 1;
        else {
            throw std::runtime_error(std::string(context) +
                                     ": unknown fin control type '" + control + "'");
        }
        fin.controlFraction = f.value("control_fraction", 1.0);
        fin.rootOffsetY = f.value("root_offset_y", 0.0);
        fin.rootOffsetZ = f.value("root_offset_z", 0.0);
        fin.baseAzimuthDeg = f.value("base_azimuth_deg", 0.0);
        fin.dihedralDeg = f.value("dihedral_deg", 0.0);
        if (f.contains("shape_points")) {
            fin.shapePoints =
                f.at("shape_points").get<std::vector<std::array<double, 2>>>();
        }
        return fin;
    }

    /**
     * @brief Parses an `aero_tables` block.
     * @param context Caller name used in the error message.
     * @throws std::runtime_error on a missing breakpoint/table key or a table
     *         that fails AeroTables::isValid (shape mismatch).
     */
    inline Models::AeroTables aeroTablesFromJson(const nlohmann::json& t, const char* context)
    {
        Models::AeroTables tables;
        tables.machBreakpoints = t.at("mach_breakpoints").get<std::vector<double>>();
        tables.aoaBreakpointsRad = t.at("aoa_breakpoints_rad").get<std::vector<double>>();
        tables.clTable = t.at("cl_table").get<std::vector<std::vector<double>>>();
        tables.cdTable = t.at("cd_table").get<std::vector<std::vector<double>>>();
        if (t.contains("cm_table")) {
            tables.cmTable = t.at("cm_table").get<std::vector<std::vector<double>>>();
        }
        if (t.contains("beta_breakpoints_rad")) {
            tables.betaBreakpointsRad = t.at("beta_breakpoints_rad").get<std::vector<double>>();
        }
        if (t.contains("cy_table")) {
            tables.cyTable = t.at("cy_table").get<std::vector<std::vector<double>>>();
        }
        if (t.contains("cn_table")) {
            tables.cnTable = t.at("cn_table").get<std::vector<std::vector<double>>>();
        }
        if (t.contains("cl_roll_table")) {
            tables.clRollTable = t.at("cl_roll_table").get<std::vector<std::vector<double>>>();
        }
        std::string error;
        if (!tables.isValid(&error)) {
            throw std::runtime_error(std::string(context) + ": aero_tables invalid: " + error);
        }
        return tables;
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
