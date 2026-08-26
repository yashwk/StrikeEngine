#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <strikeengine/simulation/Reporting.hpp>

namespace StrikeEngine::Simulation {

    struct BatchRunResult;

    inline constexpr std::uint32_t studyOutputFormatVersion = 1;

    enum class StudyOutputFormat : std::uint8_t {
        Csv,
        Binary
    };

    enum class StudyRecordType : std::uint8_t {
        Trajectory,
        ParameterSweep,
        MonteCarlo,
        BatchSummary
    };

    enum class StudyStatus : std::uint8_t {
        Active,
        Completed,
        Impacted
    };

    enum class StudyOutputField : std::uint16_t {
        ScenarioIndex,
        Iteration,
        SweepValue,
        EntityId,
        TimeS,
        Frame,
        PositionX,
        PositionY,
        PositionZ,
        LatitudeRad,
        LongitudeRad,
        AltitudeM,
        VelocityX,
        VelocityY,
        VelocityZ,
        SpeedMps,
        MassKg,
        MaxAltitudeM,
        MaxSpeedMps,
        EntityCount,
        ActiveEntities,
        PrimaryEntityActive,
        Active,
        Status
    };

    /**
     * @brief Output selection and encoding for study-wrapper records.
     *
     * An empty field list selects the record-type default. Binary v1 is a
     * little-endian stream with an 8-byte STRKOUT1 magic, a fixed header, the
     * selected field IDs, and one typed record for each StudyOutputRecord.
     */
    struct StudyOutputConfig {
        StudyOutputFormat format = StudyOutputFormat::Csv;
        std::vector<StudyOutputField> fields;
    };

    struct StudyOutputRecord {
        std::size_t scenarioIndex = 0;
        std::size_t iteration = 0;
        double sweepValue = 0.0;
        std::size_t entityId = 0;
        double timeS = 0.0;
        ReportingFrame frame = ReportingFrame::LocalEnu;
        double positionX = 0.0;
        double positionY = 0.0;
        double positionZ = 0.0;
        double latitudeRad = 0.0;
        double longitudeRad = 0.0;
        double altitudeM = 0.0;
        double velocityX = 0.0;
        double velocityY = 0.0;
        double velocityZ = 0.0;
        double speedMps = 0.0;
        double massKg = 0.0;
        double maxAltitudeM = 0.0;
        double maxSpeedMps = 0.0;
        std::size_t entityCount = 0;
        std::size_t activeEntities = 0;
        bool primaryEntityActive = false;
        bool active = false;
        StudyStatus status = StudyStatus::Completed;
    };

    class StudyOutputWriter {
    public:
        static void write(
            const std::string& outputFile,
            StudyRecordType recordType,
            const std::vector<StudyOutputRecord>& records,
            const StudyOutputConfig& config = {});

        static void writeBatchResults(
            const std::string& outputFile,
            const std::vector<BatchRunResult>& results,
            const StudyOutputConfig& config = {});
    };

    const char* studyRecordTypeName(StudyRecordType type);
    const char* studyStatusName(StudyStatus status);
    const char* studyOutputFieldName(StudyOutputField field);

} // namespace StrikeEngine::Simulation
