#pragma once

#include <cmath>
#include <cstddef>
#include <ostream>

#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <strikeengine/kernel/data/PhysicsBlock.hpp>
#include <strikeengine/models/physics/earth/EarthFrames.hpp>

namespace StrikeEngine::Simulation {

    inline constexpr int csvFormatVersion = 1;

    enum class ReportingFrame {
        LocalEnu,
        Ecef
    };

    struct ReportedState {
        ReportingFrame frame = ReportingFrame::LocalEnu;
        double positionX = 0.0;
        double positionY = 0.0;
        double positionZ = 0.0;
        double velocityX = 0.0;
        double velocityY = 0.0;
        double velocityZ = 0.0;
        double latitudeRad = 0.0;
        double longitudeRad = 0.0;
        double altitudeM = 0.0;
        double speedMps = 0.0;
        double massKg = 0.0;
    };

    inline ReportingFrame reportingFrame(const Kernel::EnvironmentConfig& environment)
    {
        return environment.earth.useEcefTruth
            ? ReportingFrame::Ecef
            : ReportingFrame::LocalEnu;
    }

    inline const char* reportingFrameName(ReportingFrame frame)
    {
        return frame == ReportingFrame::Ecef ? "ECEF" : "LOCAL_ENU";
    }

    /**
     * @brief Convert a truth state into the wrapper output convention.
     *
     * Position and velocity columns remain in the selected simulation frame.
     * Latitude/longitude are WGS84 geodetic coordinates. In local mode they
     * are resolved from the configured reference anchor for convenience and
     * altitude remains the local world Z value. In ECEF mode the geodetic
     * values are derived from the absolute ECEF position.
     */
    inline ReportedState reportState(
        const Kernel::PhysicsBlock& physics,
        std::size_t entityId,
        const Kernel::EnvironmentConfig& environment)
    {
        ReportedState result;
        result.frame = reportingFrame(environment);
        result.positionX = physics.px.at(entityId);
        result.positionY = physics.py.at(entityId);
        result.positionZ = physics.pz.at(entityId);
        result.velocityX = physics.vx.at(entityId);
        result.velocityY = physics.vy.at(entityId);
        result.velocityZ = physics.vz.at(entityId);
        result.speedMps = std::sqrt(
            result.velocityX * result.velocityX +
            result.velocityY * result.velocityY +
            result.velocityZ * result.velocityZ);
        result.massKg = physics.mass.at(entityId);

        Models::GeodeticCoordinate geodetic;
        if (result.frame == ReportingFrame::Ecef) {
            geodetic = Models::ecefToGeodetic({
                result.positionX, result.positionY, result.positionZ});
            result.altitudeM = geodetic.altitudeM;
        } else {
            const Models::GeodeticCoordinate origin{
                environment.earth.referenceLatitudeRad,
                environment.earth.referenceLongitudeRad,
                0.0};
            geodetic = Models::EarthFrames::enuToGeodetic(
                {result.positionX, result.positionY, result.positionZ}, origin);
            result.altitudeM = result.positionZ;
        }
        result.latitudeRad = geodetic.latitudeRad;
        result.longitudeRad = geodetic.longitudeRad;
        return result;
    }

    inline void writeCsvMetadata(
        std::ostream& output,
        const char* recordType,
        const char* frame = "PER_ROW")
    {
        output << "# strikeengine_csv_format=" << csvFormatVersion << "\n";
        output << "# record_type=" << recordType << "\n";
        output << "# frame=" << frame << "\n";
    }

    inline bool finiteState(const ReportedState& state)
    {
        return std::isfinite(state.positionX) &&
               std::isfinite(state.positionY) &&
               std::isfinite(state.positionZ) &&
               std::isfinite(state.velocityX) &&
               std::isfinite(state.velocityY) &&
               std::isfinite(state.velocityZ) &&
               std::isfinite(state.latitudeRad) &&
               std::isfinite(state.longitudeRad) &&
               std::isfinite(state.altitudeM) &&
               std::isfinite(state.speedMps) &&
               std::isfinite(state.massKg);
    }

} // namespace StrikeEngine::Simulation
