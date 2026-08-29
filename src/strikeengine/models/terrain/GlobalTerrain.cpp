#include <strikeengine/models/terrain/GlobalTerrain.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace StrikeEngine::Models {

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTwoPi = 2.0 * kPi;

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

double GlobalTerrain::elevationM(double latitudeRad, double longitudeRad) const {
    if (!valid_ || !std::isfinite(latitudeRad) || !std::isfinite(longitudeRad)) {
        return 0.0;
    }

    const double maxLatitude = southLatitudeRad_ +
        static_cast<double>(rows_ - 1) * latitudeStepRad_;
    if (latitudeRad < southLatitudeRad_ || latitudeRad > maxLatitude) return 0.0;

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
        return 0.0;
    }

    const double rowCoordinate = (latitudeRad - southLatitudeRad_) / latitudeStepRad_;
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
    return totalWeight > 0.0 ? weighted / totalWeight : 0.0;
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

} // namespace StrikeEngine::Models
