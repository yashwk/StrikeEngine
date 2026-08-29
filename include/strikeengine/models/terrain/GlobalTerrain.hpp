#pragma once

#include <array>
#include <cstddef>
#include <list>
#include <memory>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace StrikeEngine::Models {

    enum class TerrainInterpolation {
        Nearest,
        Bilinear
    };

    enum class TerrainSampleStatus {
        Valid,
        PartialNoData,
        NoData,
        OutOfCoverage,
        Invalid
    };

    struct TerrainSample {
        double elevationM = 0.0;
        TerrainSampleStatus status = TerrainSampleStatus::Invalid;

        bool hasElevation() const {
            return status == TerrainSampleStatus::Valid ||
                   status == TerrainSampleStatus::PartialNoData;
        }
    };

    struct TerrainSurface {
        TerrainSample sample{};
        // Upward unit normal expressed in local ENU: east, north, up.
        std::array<double, 3> normalEnu{0.0, 0.0, 1.0};
        double slopeRad = 0.0;
    };

    class TerrainSource {
    public:
        virtual ~TerrainSource() = default;
        virtual bool isValid() const = 0;
        virtual TerrainSample sample(double latitudeRad, double longitudeRad,
                                     TerrainInterpolation interpolation =
                                         TerrainInterpolation::Bilinear) const = 0;
        virtual TerrainSurface surface(double latitudeRad, double longitudeRad) const = 0;
    };

    /**
     * @brief Geodetic terrain raster sampled at cell centers.
     *
     * Latitude and longitude are radians; elevations are metres above the
     * WGS84 ellipsoid. The raster is stored south-to-north and west-to-east.
     * A wrapped longitude raster is periodic at the dateline.
     */
    class GlobalTerrain final : public TerrainSource {
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

        bool isValid() const override { return valid_; }

        TerrainSample sample(double latitudeRad, double longitudeRad,
                             TerrainInterpolation interpolation =
                                 TerrainInterpolation::Bilinear) const override;

        TerrainSurface surface(double latitudeRad, double longitudeRad) const override;

        /** Compatibility elevation-only query; missing/out-of-coverage returns 0 m. */
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

    /**
     * @brief Open any GDAL-supported single-band elevation raster.
     *
     * GeoTIFF, COG, DTED, VRT, and other installed GDAL raster drivers are
     * accepted. The dataset is normalized to a WGS84 geodetic raster for
     * deterministic runtime sampling. Returns nullptr when GDAL support is
     * disabled or the source cannot be opened/warped.
     */
    std::shared_ptr<const TerrainSource> loadGdalTerrain(
        const std::string& path, std::string* error = nullptr);

    /**
     * @brief Thread-safe bounded cache for GDAL terrain sources.
     *
     * The cache is keyed by source path and uses least-recently-used eviction.
     * Loading remains eager: a cache entry owns the normalized WGS84 source
     * returned by loadGdalTerrain, so a hit avoids reopening and resampling the
     * dataset. A VRT can therefore represent a multi-file mosaic while the
     * cache bounds the number of normalized sources retained by the caller.
     */
    class TerrainTileCache {
    public:
        explicit TerrainTileCache(std::size_t capacity = 8);

        std::shared_ptr<const TerrainSource> load(
            const std::string& path, std::string* error = nullptr);

        void clear();
        std::size_t size() const;
        std::size_t capacity() const { return capacity_; }

    private:
        using EntryList = std::list<std::string>;
        using EntryMap = std::unordered_map<
            std::string,
            std::pair<std::shared_ptr<const TerrainSource>, EntryList::iterator>>;

        const std::size_t capacity_;
        mutable std::mutex mutex_;
        EntryList lru_;
        EntryMap entries_;
    };

} // namespace StrikeEngine::Models
