#pragma once

#include <string>
#include <vector>

namespace StrikeEngine {

  /**
   * @brief A struct to hold a set of aerodynamic coefficients.
   */
  struct AeroCoefficients {
    double Cl = 0.0; // Lift coefficient
    double Cd = 0.0; // Drag coefficient
  };

  /**
   * @brief Loads and manages aerodynamic coefficient data from profiles.
   *
   * This class reads aerodynamic lookup tables from a JSON file and provides
   * a function to perform bilinear interpolation to find the coefficients
   * for any given flight condition (Mach number and Angle of Attack).
   */
  class AerodynamicsDatabase {
  public:
    AerodynamicsDatabase() = default;

    /**
     * @brief Loads an aerodynamic profile from a JSON file.
     * @param filepath The path to the aero profile JSON.
     * @return True if loading was successful, false otherwise.
     */
    bool loadProfile(const std::string& filepath);

    /**
     * @brief Gets the interpolated aerodynamic coefficients for a given flight state.
     * @param mach The current Mach number.
     * @param AoARad The current Angle of Attack in radians.
     * @return An AeroCoefficients struct with the interpolated Cl and Cd values.
     */
    [[nodiscard]] AeroCoefficients getCoefficients(double mach, double AoARad) const;

  private:
    std::vector<double> _machBreakpoints;
    std::vector<double> _aoaBreakpointsRad;

    std::vector<std::vector<double>> _clTable;
    std::vector<std::vector<double>> _cdTable;
  };

} // namespace StrikeEngine
