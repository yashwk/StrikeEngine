#include <strikeengine/models/terrain/GlobalTerrain.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#ifdef STRIKEENGINE_WITH_GDAL
#include <gdal_priv.h>
#include <ogr_spatialref.h>
#endif

namespace StrikeEngine::Models {

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDegreesHalfTurn = 180.0;
constexpr double kDegreesFullTurn = 360.0;

double normalizeLongitude(double longitudeRad) {
    double result = std::fmod(longitudeRad + kPi, kTwoPi);
    if (result < 0.0) result += kTwoPi;
    return result - kPi;
}

bool isMissing(double value, double noDataValue) {
    return !std::isfinite(value) ||
        (std::isfinite(noDataValue) && value == noDataValue);
}

void setError(std::string* error, const std::string& message) {
    if (error) *error = message;
}

} // namespace

GlobalTerrain::GlobalTerrain(double southLatitudeRad, double westLongitudeRad,
                             double latitudeStepRad, double longitudeStepRad,
                             std::size_t rows, std::size_t columns,
                             std::vector<double> elevationsM,
                             double noDataValue, bool wrapLongitude)
    : southLatitudeRad_(southLatitudeRad),
      westLongitudeRad_(normalizeLongitude(westLongitudeRad)),
      latitudeStepRad_(latitudeStepRad), longitudeStepRad_(longitudeStepRad),
      rows_(rows), columns_(columns), elevationsM_(std::move(elevationsM)),
      noDataValue_(noDataValue), wrapLongitude_(wrapLongitude) {
    valid_ = std::isfinite(southLatitudeRad_) &&
        std::isfinite(westLongitudeRad_) && std::isfinite(latitudeStepRad_) &&
        std::isfinite(longitudeStepRad_) && latitudeStepRad_ > 0.0 &&
        longitudeStepRad_ > 0.0 && rows_ >= 2 && columns_ >= 2 &&
        elevationsM_.size() == rows_ * columns_ &&
        southLatitudeRad_ >= -0.5 * kPi &&
        southLatitudeRad_ + static_cast<double>(rows_ - 1) * latitudeStepRad_ <=
            0.5 * kPi + 1e-12;
    if (wrapLongitude_) {
        valid_ = valid_ && columns_ >= 2 &&
            static_cast<double>(columns_) * longitudeStepRad_ <= kTwoPi + 1e-9;
    }
}

TerrainSample GlobalTerrain::sample(double latitudeRad, double longitudeRad,
                                    TerrainInterpolation interpolation) const {
    if (!valid_ || !std::isfinite(latitudeRad) || !std::isfinite(longitudeRad)) {
        return {0.0, TerrainSampleStatus::Invalid};
    }

    const double maxLatitude = southLatitudeRad_ +
        static_cast<double>(rows_ - 1) * latitudeStepRad_;
    if (latitudeRad < southLatitudeRad_ || latitudeRad > maxLatitude) {
        return {0.0, TerrainSampleStatus::OutOfCoverage};
    }

    double longitude = normalizeLongitude(longitudeRad);
    if (!wrapLongitude_) {
        // Compare in the raster's unwrapped longitude neighborhood so a tile
        // spanning the dateline (for example 179.5..182.5 degrees) remains
        // addressable after canonical [-180, 180) normalization.
        if (longitude < westLongitudeRad_ - kPi) longitude += kTwoPi;
        if (longitude > westLongitudeRad_ + kPi) longitude -= kTwoPi;
    }
    double columnCoordinate = (longitude - westLongitudeRad_) / longitudeStepRad_;
    const double maxColumn = static_cast<double>(columns_ - 1);
    if (wrapLongitude_) {
        columnCoordinate = std::fmod(columnCoordinate, static_cast<double>(columns_));
        if (columnCoordinate < 0.0) columnCoordinate += static_cast<double>(columns_);
    } else if (columnCoordinate < 0.0 || columnCoordinate > maxColumn) {
        return {0.0, TerrainSampleStatus::OutOfCoverage};
    }

    const double rowCoordinate = (latitudeRad - southLatitudeRad_) / latitudeStepRad_;
    if (interpolation == TerrainInterpolation::Nearest) {
        const auto row = std::min(rows_ - 1,
            static_cast<std::size_t>(std::llround(rowCoordinate)));
        std::size_t column = static_cast<std::size_t>(std::llround(columnCoordinate));
        if (wrapLongitude_) column %= columns_;
        else column = std::min(columns_ - 1, column);
        const double value = elevationsM_[row * columns_ + column];
        if (isMissing(value, noDataValue_)) {
            return {0.0, TerrainSampleStatus::NoData};
        }
        return {value, TerrainSampleStatus::Valid};
    }

    const std::size_t row0 = std::min(rows_ - 1,
        static_cast<std::size_t>(std::floor(rowCoordinate)));
    const std::size_t row1 = std::min(rows_ - 1, row0 + 1);
    const double rowFraction = row0 == row1 ? 0.0 : rowCoordinate - static_cast<double>(row0);

    const std::size_t col0 = std::min(columns_ - 1,
        static_cast<std::size_t>(std::floor(columnCoordinate)));
    const std::size_t col1 = wrapLongitude_
        ? (col0 + 1) % columns_
        : std::min(columns_ - 1, col0 + 1);
    const double colFraction = col0 == col1 ? 0.0 : columnCoordinate - static_cast<double>(col0);

    const std::size_t indices[4] = {
        row0 * columns_ + col0, row0 * columns_ + col1,
        row1 * columns_ + col0, row1 * columns_ + col1};
    const double weights[4] = {
        (1.0 - rowFraction) * (1.0 - colFraction),
        (1.0 - rowFraction) * colFraction,
        rowFraction * (1.0 - colFraction), rowFraction * colFraction};

    double weighted = 0.0;
    double totalWeight = 0.0;
    for (int i = 0; i < 4; ++i) {
        const double value = elevationsM_[indices[i]];
        if (isMissing(value, noDataValue_)) continue;
        weighted += weights[i] * value;
        totalWeight += weights[i];
    }
    if (totalWeight <= 0.0) return {0.0, TerrainSampleStatus::NoData};
    const auto status = totalWeight < 1.0 - 1e-12
        ? TerrainSampleStatus::PartialNoData : TerrainSampleStatus::Valid;
    return {weighted / totalWeight, status};
}

double GlobalTerrain::elevationM(double latitudeRad, double longitudeRad) const {
    return sample(latitudeRad, longitudeRad).elevationM;
}

TerrainSurface GlobalTerrain::surface(double latitudeRad, double longitudeRad) const {
    TerrainSurface result;
    result.sample = sample(latitudeRad, longitudeRad);
    if (!result.sample.hasElevation()) return result;

    const double epsilon = std::max(1e-7,
        std::min(1e-4, 0.5 * std::min(latitudeStepRad_, longitudeStepRad_)));
    const auto south = sample(latitudeRad - epsilon, longitudeRad);
    const auto north = sample(latitudeRad + epsilon, longitudeRad);
    const auto west = sample(latitudeRad, longitudeRad - epsilon);
    const auto east = sample(latitudeRad, longitudeRad + epsilon);
    const auto valid = [](const TerrainSample& sample) { return sample.hasElevation(); };

    double northSlope = 0.0;
    if (valid(south) && valid(north)) {
        northSlope = (north.elevationM - south.elevationM) /
            (2.0 * epsilon * EarthModel::meridionalRadiusM(latitudeRad));
    } else if (valid(north)) {
        northSlope = (north.elevationM - result.sample.elevationM) /
            (epsilon * EarthModel::meridionalRadiusM(latitudeRad));
    } else if (valid(south)) {
        northSlope = (result.sample.elevationM - south.elevationM) /
            (epsilon * EarthModel::meridionalRadiusM(latitudeRad));
    }

    const double eastScale = EarthModel::primeVerticalRadiusM(latitudeRad) *
        std::max(1e-12, std::abs(std::cos(latitudeRad)));
    double eastSlope = 0.0;
    if (valid(west) && valid(east)) {
        eastSlope = (east.elevationM - west.elevationM) / (2.0 * epsilon * eastScale);
    } else if (valid(east)) {
        eastSlope = (east.elevationM - result.sample.elevationM) / (epsilon * eastScale);
    } else if (valid(west)) {
        eastSlope = (result.sample.elevationM - west.elevationM) / (epsilon * eastScale);
    }

    const double norm = std::sqrt(eastSlope * eastSlope + northSlope * northSlope + 1.0);
    result.normalEnu = {-eastSlope / norm, -northSlope / norm, 1.0 / norm};
    result.slopeRad = std::atan(std::hypot(eastSlope, northSlope));
    return result;
}

std::shared_ptr<const GlobalTerrain> GlobalTerrain::loadEsriAsciiGrid(
    const std::string& path, std::string* error) {
    std::ifstream input(path);
    if (!input.is_open()) {
        setError(error, "cannot open terrain grid '" + path + "'");
        return nullptr;
    }

    try {
        std::size_t ncols = 0, nrows = 0;
        double xll = 0.0, yll = 0.0, cellsize = 0.0;
        double noData = -9999.0;
        bool xCenter = false, yCenter = false;
        for (int i = 0; i < 6; ++i) {
            std::string key;
            input >> key;
            std::transform(key.begin(), key.end(), key.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (key == "ncols") input >> ncols;
            else if (key == "nrows") input >> nrows;
            else if (key == "xllcorner" || key == "xllcenter") {
                input >> xll; xCenter = key == "xllcenter";
            } else if (key == "yllcorner" || key == "yllcenter") {
                input >> yll; yCenter = key == "yllcenter";
            } else if (key == "cellsize") input >> cellsize;
            else {
                // The sixth header line may be NODATA_value; keep parsing it
                // as optional and reject unknown mandatory headers below.
                double value = 0.0;
                input >> value;
                if (key == "nodata_value") noData = value;
            }
            if (!input) throw std::runtime_error("malformed ESRI ASCII header");
        }
        if (ncols < 2 || nrows < 2 || !std::isfinite(xll) || !std::isfinite(yll) ||
            !std::isfinite(cellsize) || cellsize <= 0.0) {
            throw std::runtime_error("invalid ESRI ASCII dimensions or cellsize");
        }
        if (xCenter) xll -= 0.5 * cellsize;
        if (yCenter) yll -= 0.5 * cellsize;

        std::vector<double> elevations(nrows * ncols);
        for (std::size_t fileRow = 0; fileRow < nrows; ++fileRow) {
            for (std::size_t col = 0; col < ncols; ++col) {
                double value = 0.0;
                input >> value;
                if (!input) throw std::runtime_error("not enough elevation samples");
                // ESRI rows start at north; the public raster is south-to-north.
                const std::size_t southRow = nrows - 1 - fileRow;
                elevations[southRow * ncols + col] = value;
            }
        }
        const double southCenter = yll + 0.5 * cellsize;
        const double westCenter = xll + 0.5 * cellsize;
        const bool wraps = static_cast<double>(ncols) * cellsize >= 360.0 - 1e-6;
        auto terrain = std::make_shared<GlobalTerrain>(
            southCenter * kPi / 180.0, westCenter * kPi / 180.0,
            cellsize * kPi / 180.0, cellsize * kPi / 180.0,
            nrows, ncols, std::move(elevations), noData, wraps);
        if (!terrain->isValid()) throw std::runtime_error("invalid terrain raster coverage");
        return terrain;
    } catch (const std::exception& exception) {
        setError(error, "invalid ESRI ASCII terrain grid '" + path + "': " + exception.what());
        return nullptr;
    }
}

std::shared_ptr<const TerrainSource> loadGdalTerrain(
    const std::string& path, std::string* error) {
#ifndef STRIKEENGINE_WITH_GDAL
    setError(error, "GDAL terrain support is disabled at build time");
    (void)path;
    return nullptr;
#else
    GDALAllRegister();
    GDALDataset* rawDataset = static_cast<GDALDataset*>(
        GDALOpen(path.c_str(), GA_ReadOnly));
    if (!rawDataset) {
        setError(error, "GDAL could not open terrain dataset '" + path + "'");
        return nullptr;
    }
    struct DatasetCloser {
        void operator()(GDALDataset* dataset) const { if (dataset) GDALClose(dataset); }
    };
    std::unique_ptr<GDALDataset, DatasetCloser> dataset(rawDataset);

    try {
        GDALRasterBand* band = dataset->GetRasterBand(1);
        if (!band || dataset->GetRasterCount() < 1) {
            throw std::runtime_error("dataset has no elevation band");
        }
        const int width = band->GetXSize();
        const int height = band->GetYSize();
        if (width < 2 || height < 2) throw std::runtime_error("elevation raster is smaller than 2x2");

        double geotransform[6]{};
        if (dataset->GetGeoTransform(geotransform) != CE_None) {
            throw std::runtime_error("dataset has no usable geotransform");
        }
        double inverse[6]{};
        if (!GDALInvGeoTransform(geotransform, inverse)) {
            throw std::runtime_error("dataset geotransform is not invertible");
        }
        const char* projection = dataset->GetProjectionRef();
        if (!projection || !*projection) {
            throw std::runtime_error("dataset has no coordinate reference system");
        }

        OGRSpatialReference sourceSrs;
        if (sourceSrs.SetFromUserInput(projection) != OGRERR_NONE) {
            throw std::runtime_error("dataset CRS could not be parsed");
        }
        OGRSpatialReference wgs84;
        wgs84.SetWellKnownGeogCS("WGS84");
        sourceSrs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
        wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
        auto* toWgs84 = OGRCreateCoordinateTransformation(&sourceSrs, &wgs84);
        auto* fromWgs84 = OGRCreateCoordinateTransformation(&wgs84, &sourceSrs);
        if (!toWgs84 || !fromWgs84) {
            if (toWgs84) OCTDestroyCoordinateTransformation(toWgs84);
            if (fromWgs84) OCTDestroyCoordinateTransformation(fromWgs84);
            throw std::runtime_error("dataset CRS has no WGS84 transformation");
        }

        double minLatitude = std::numeric_limits<double>::infinity();
        double maxLatitude = -std::numeric_limits<double>::infinity();
        double longitudes[4]{};
        int longitudeIndex = 0;
        double referenceLongitude = 0.0;
        for (const double pixel : {0.0, static_cast<double>(width)}) {
            for (const double line : {0.0, static_cast<double>(height)}) {
                double x = geotransform[0] + pixel * geotransform[1] + line * geotransform[2];
                double y = geotransform[3] + pixel * geotransform[4] + line * geotransform[5];
                if (!toWgs84->Transform(1, &x, &y)) {
                    OCTDestroyCoordinateTransformation(toWgs84);
                    OCTDestroyCoordinateTransformation(fromWgs84);
                    throw std::runtime_error("dataset coordinate transformation failed");
                }
                if (longitudeIndex == 0) referenceLongitude = x;
                while (x < referenceLongitude - kDegreesHalfTurn) x += kDegreesFullTurn;
                while (x > referenceLongitude + kDegreesHalfTurn) x -= kDegreesFullTurn;
                longitudes[longitudeIndex++] = x;
                minLatitude = std::min(minLatitude, y);
                maxLatitude = std::max(maxLatitude, y);
            }
        }
        const auto minLongitude = *std::min_element(std::begin(longitudes), std::end(longitudes));
        const auto maxLongitude = *std::max_element(std::begin(longitudes), std::end(longitudes));
        if (!std::isfinite(minLatitude) || !std::isfinite(maxLatitude) ||
            minLatitude < -kDegreesHalfTurn || maxLatitude > kDegreesHalfTurn ||
            maxLatitude <= minLatitude || maxLongitude <= minLongitude ||
            maxLongitude - minLongitude > kDegreesFullTurn + 1e-9) {
            OCTDestroyCoordinateTransformation(toWgs84);
            OCTDestroyCoordinateTransformation(fromWgs84);
            throw std::runtime_error("dataset has invalid or unsupported geographic coverage");
        }

        std::vector<double> sourceValues(static_cast<std::size_t>(width) * height);
        if (band->RasterIO(GF_Read, 0, 0, width, height, sourceValues.data(),
                           width, height, GDT_Float64, 0, 0) != CE_None) {
            OCTDestroyCoordinateTransformation(toWgs84);
            OCTDestroyCoordinateTransformation(fromWgs84);
            throw std::runtime_error("elevation band could not be read");
        }
        int hasNoData = 0;
        const double sourceNoData = band->GetNoDataValue(&hasNoData);
        const double scale = std::isfinite(band->GetScale()) ? band->GetScale() : 1.0;
        const double offset = std::isfinite(band->GetOffset()) ? band->GetOffset() : 0.0;
        const double noData = hasNoData ? sourceNoData :
            std::numeric_limits<double>::quiet_NaN();

        auto sourceSample = [&](double latitude, double longitude) {
            double x = longitude;
            double y = latitude;
            if (!fromWgs84->Transform(1, &x, &y)) return std::numeric_limits<double>::quiet_NaN();
            const double pixel = inverse[0] + inverse[1] * x + inverse[2] * y;
            const double line = inverse[3] + inverse[4] * x + inverse[5] * y;
            if (pixel < 0.0 || line < 0.0 || pixel > width - 1.0 || line > height - 1.0) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            const int x0 = std::min(width - 1, static_cast<int>(std::floor(pixel)));
            const int y0 = std::min(height - 1, static_cast<int>(std::floor(line)));
            const int x1 = std::min(width - 1, x0 + 1);
            const int y1 = std::min(height - 1, y0 + 1);
            const double fx = x1 == x0 ? 0.0 : pixel - x0;
            const double fy = y1 == y0 ? 0.0 : line - y0;
            const int xs[4] = {x0, x1, x0, x1};
            const int ys[4] = {y0, y0, y1, y1};
            const double weights[4] = {(1.0 - fx) * (1.0 - fy), fx * (1.0 - fy),
                                       (1.0 - fx) * fy, fx * fy};
            double value = 0.0;
            double weight = 0.0;
            for (int i = 0; i < 4; ++i) {
                const double raw = sourceValues[static_cast<std::size_t>(ys[i]) * width + xs[i]];
                if (!std::isfinite(raw) || (hasNoData && raw == sourceNoData)) continue;
                value += weights[i] * (raw * scale + offset);
                weight += weights[i];
            }
            return weight > 0.0 ? value / weight : std::numeric_limits<double>::quiet_NaN();
        };

        std::vector<double> normalized(static_cast<std::size_t>(height) * width,
                                       std::numeric_limits<double>::quiet_NaN());
        for (int row = 0; row < height; ++row) {
            const double latitude = minLatitude +
                (maxLatitude - minLatitude) * row / static_cast<double>(height - 1);
            for (int column = 0; column < width; ++column) {
                const double longitude = minLongitude +
                    (maxLongitude - minLongitude) * column / static_cast<double>(width - 1);
                normalized[static_cast<std::size_t>(row) * width + column] =
                    sourceSample(latitude, longitude);
            }
        }
        OCTDestroyCoordinateTransformation(toWgs84);
        OCTDestroyCoordinateTransformation(fromWgs84);

        auto result = std::make_shared<GlobalTerrain>(
            minLatitude * kPi / kDegreesHalfTurn,
            minLongitude * kPi / kDegreesHalfTurn,
            (maxLatitude - minLatitude) * kPi /
                (kDegreesHalfTurn * static_cast<double>(height - 1)),
            (maxLongitude - minLongitude) * kPi /
                (kDegreesHalfTurn * static_cast<double>(width - 1)),
            static_cast<std::size_t>(height), static_cast<std::size_t>(width),
            std::move(normalized), noData,
            maxLongitude - minLongitude >= kDegreesFullTurn - 1e-6);
        if (!result->isValid()) {
            throw std::runtime_error("normalized terrain raster is invalid (lat " +
                std::to_string(minLatitude) + ".." + std::to_string(maxLatitude) +
                ", lon " + std::to_string(minLongitude) + ".." + std::to_string(maxLongitude) +
                ", size " + std::to_string(width) + "x" + std::to_string(height) + ")");
        }
        return result;
    } catch (const std::exception& exception) {
        setError(error, "invalid GDAL terrain dataset '" + path + "': " + exception.what());
        return nullptr;
    }
#endif
}

TerrainTileCache::TerrainTileCache(std::size_t capacity)
    : capacity_(capacity) {}

std::shared_ptr<const TerrainSource> TerrainTileCache::load(
    const std::string& path, std::string* error) {
    if (capacity_ > 0) {
        std::lock_guard lock(mutex_);
        const auto found = entries_.find(path);
        if (found != entries_.end()) {
            lru_.splice(lru_.begin(), lru_, found->second.second);
            found->second.second = lru_.begin();
            return found->second.first;
        }
    }

    const auto source = loadGdalTerrain(path, error);
    if (!source || capacity_ == 0) return source;

    std::lock_guard lock(mutex_);
    const auto existing = entries_.find(path);
    if (existing != entries_.end()) {
        lru_.splice(lru_.begin(), lru_, existing->second.second);
        existing->second.second = lru_.begin();
        return existing->second.first;
    }
    while (entries_.size() >= capacity_) {
        entries_.erase(lru_.back());
        lru_.pop_back();
    }
    lru_.push_front(path);
    entries_.emplace(path, std::make_pair(source, lru_.begin()));
    return source;
}

void TerrainTileCache::clear() {
    std::lock_guard lock(mutex_);
    entries_.clear();
    lru_.clear();
}

std::size_t TerrainTileCache::size() const {
    std::lock_guard lock(mutex_);
    return entries_.size();
}

} // namespace StrikeEngine::Models
