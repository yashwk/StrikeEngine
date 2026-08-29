#pragma once

#include <cstddef>
#include <memory>
#include <limits>
#include <string>
#include <vector>

namespace StrikeEngine::Models {

    /**
     * @brief Geodetic terrain raster sampled at cell centers.
     *
     * Latitude and longitude are radians; elevations are metres above the
     * WGS84 ellipsoid. The raster is stored south-to-north and west-to-east.
     * A wrapped longitude raster is periodic at the dateline.
     */
    class GlobalTerrain {
    public:
        GlobalTerrain(double southLatitudeRad, double westLongitudeRad,
                      double latitudeStepRad, double longitudeStepRad,
                      std::size_t rows, std::size_t columns,
                      std::vector<double> elevationsM,
                      double noDataValue = std::numeric_limits<double>::quiet_NaN(),
                      bool wrapLongitude = false);

        /** Load an ESRI/ArcInfo ASCII Grid (.asc) without external libraries. */
        static std::shared_ptr<const GlobalTerrain> loadEsriAsciiGrid(
            const std::string& path, std::string* error = nullptr);

        bool isValid() const { return valid_; }

        /** Bilinear geodetic sample; missing/out-of-coverage data returns 0 m. */
        double elevationM(double latitudeRad, double longitudeRad) const;

        std::size_t rows() const { return rows_; }
        std::size_t columns() const { return columns_; }
        bool wrapsLongitude() const { return wrapLongitude_; }

    private:
        double southLatitudeRad_ = 0.0;
        double westLongitudeRad_ = 0.0;
        double latitudeStepRad_ = 0.0;
        double longitudeStepRad_ = 0.0;
        std::size_t rows_ = 0;
        std::size_t columns_ = 0;
        std::vector<double> elevationsM_;
        double noDataValue_ = 0.0;
        bool wrapLongitude_ = false;
        bool valid_ = false;
    };

} // namespace StrikeEngine::Models
