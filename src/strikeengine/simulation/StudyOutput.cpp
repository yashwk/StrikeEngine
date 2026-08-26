#include <strikeengine/simulation/StudyOutput.hpp>
#include <strikeengine/simulation/BatchRunner.hpp>

#include <bit>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace StrikeEngine::Simulation {

namespace {

using Field = StudyOutputField;

std::vector<Field> defaultFields(StudyRecordType type)
{
    switch (type) {
    case StudyRecordType::Trajectory:
        return {Field::EntityId, Field::TimeS, Field::Frame,
                Field::PositionX, Field::PositionY, Field::PositionZ,
                Field::LatitudeRad, Field::LongitudeRad, Field::AltitudeM,
                Field::VelocityX, Field::VelocityY, Field::VelocityZ,
                Field::SpeedMps, Field::MassKg, Field::Active, Field::Status};
    case StudyRecordType::ParameterSweep:
        return {Field::SweepValue, Field::EntityId, Field::Frame, Field::TimeS,
                Field::PositionX, Field::PositionY, Field::PositionZ,
                Field::LatitudeRad, Field::LongitudeRad, Field::AltitudeM,
                Field::MaxAltitudeM, Field::MaxSpeedMps,
                Field::Active, Field::Status};
    case StudyRecordType::MonteCarlo:
        return {Field::Iteration, Field::EntityId, Field::Frame, Field::TimeS,
                Field::PositionX, Field::PositionY, Field::PositionZ,
                Field::LatitudeRad, Field::LongitudeRad, Field::AltitudeM,
                Field::Active, Field::Status};
    case StudyRecordType::BatchSummary:
        return {Field::ScenarioIndex, Field::EntityId, Field::Frame, Field::TimeS,
                Field::EntityCount, Field::ActiveEntities,
                Field::PositionX, Field::PositionY, Field::PositionZ,
                Field::LatitudeRad, Field::LongitudeRad, Field::AltitudeM,
                Field::MaxAltitudeM, Field::MaxSpeedMps,
                Field::PrimaryEntityActive, Field::Status};
    }
    throw std::invalid_argument("unknown study record type");
}

bool isSupported(StudyRecordType type, Field field)
{
    switch (type) {
    case StudyRecordType::Trajectory:
        return field == Field::EntityId || field == Field::TimeS ||
               field == Field::Frame || field == Field::PositionX ||
               field == Field::PositionY || field == Field::PositionZ ||
               field == Field::LatitudeRad || field == Field::LongitudeRad ||
               field == Field::AltitudeM || field == Field::VelocityX ||
               field == Field::VelocityY || field == Field::VelocityZ ||
               field == Field::SpeedMps || field == Field::MassKg ||
               field == Field::Active || field == Field::Status;
    case StudyRecordType::ParameterSweep:
        return field == Field::SweepValue || field == Field::EntityId ||
               field == Field::Frame || field == Field::TimeS ||
               field == Field::PositionX || field == Field::PositionY ||
               field == Field::PositionZ || field == Field::LatitudeRad ||
               field == Field::LongitudeRad || field == Field::AltitudeM ||
               field == Field::MaxAltitudeM || field == Field::MaxSpeedMps ||
               field == Field::Active || field == Field::Status;
    case StudyRecordType::MonteCarlo:
        return field == Field::Iteration || field == Field::EntityId ||
               field == Field::Frame || field == Field::TimeS ||
               field == Field::PositionX || field == Field::PositionY ||
               field == Field::PositionZ || field == Field::LatitudeRad ||
               field == Field::LongitudeRad || field == Field::AltitudeM ||
               field == Field::Active || field == Field::Status;
    case StudyRecordType::BatchSummary:
        return field == Field::ScenarioIndex || field == Field::EntityId ||
               field == Field::Frame || field == Field::TimeS ||
               field == Field::PositionX || field == Field::PositionY ||
               field == Field::PositionZ || field == Field::LatitudeRad ||
               field == Field::LongitudeRad || field == Field::AltitudeM ||
               field == Field::MaxAltitudeM || field == Field::MaxSpeedMps ||
               field == Field::EntityCount || field == Field::ActiveEntities ||
               field == Field::PrimaryEntityActive || field == Field::Status;
    }
    return false;
}

std::vector<Field> resolveFields(
    StudyRecordType type,
    const StudyOutputConfig& config)
{
    const auto fields = config.fields.empty() ? defaultFields(type) : config.fields;
    std::vector<Field> resolved;
    resolved.reserve(fields.size());
    for (const auto field : fields) {
        if (!isSupported(type, field)) {
            throw std::invalid_argument(
                "study output field is not supported by the record type");
        }
        if (std::find(resolved.begin(), resolved.end(), field) != resolved.end()) {
            throw std::invalid_argument("study output fields must be unique");
        }
        resolved.push_back(field);
    }
    return resolved;
}

void writeCsvValue(std::ostream& output, Field field, const StudyOutputRecord& record)
{
    switch (field) {
    case Field::ScenarioIndex: output << record.scenarioIndex; break;
    case Field::Iteration: output << record.iteration; break;
    case Field::SweepValue: output << record.sweepValue; break;
    case Field::EntityId: output << record.entityId; break;
    case Field::TimeS: output << record.timeS; break;
    case Field::Frame: output << reportingFrameName(record.frame); break;
    case Field::PositionX: output << record.positionX; break;
    case Field::PositionY: output << record.positionY; break;
    case Field::PositionZ: output << record.positionZ; break;
    case Field::LatitudeRad: output << record.latitudeRad; break;
    case Field::LongitudeRad: output << record.longitudeRad; break;
    case Field::AltitudeM: output << record.altitudeM; break;
    case Field::VelocityX: output << record.velocityX; break;
    case Field::VelocityY: output << record.velocityY; break;
    case Field::VelocityZ: output << record.velocityZ; break;
    case Field::SpeedMps: output << record.speedMps; break;
    case Field::MassKg: output << record.massKg; break;
    case Field::MaxAltitudeM: output << record.maxAltitudeM; break;
    case Field::MaxSpeedMps: output << record.maxSpeedMps; break;
    case Field::EntityCount: output << record.entityCount; break;
    case Field::ActiveEntities: output << record.activeEntities; break;
    case Field::PrimaryEntityActive: output << (record.primaryEntityActive ? 1 : 0); break;
    case Field::Active: output << (record.active ? 1 : 0); break;
    case Field::Status: output << studyStatusName(record.status); break;
    }
}

void writeLittleEndian(std::ostream& output, std::uint64_t value)
{
    for (int byte = 0; byte < 8; ++byte) {
        const auto valueByte = static_cast<char>((value >> (byte * 8)) & 0xffU);
        output.put(valueByte);
    }
}

void writeLittleEndian(std::ostream& output, std::uint32_t value)
{
    for (int byte = 0; byte < 4; ++byte) {
        const auto valueByte = static_cast<char>((value >> (byte * 8)) & 0xffU);
        output.put(valueByte);
    }
}

void writeLittleEndian(std::ostream& output, std::uint16_t value)
{
    output.put(static_cast<char>(value & 0xffU));
    output.put(static_cast<char>((value >> 8) & 0xffU));
}

void writeDouble(std::ostream& output, double value)
{
    writeLittleEndian(output, std::bit_cast<std::uint64_t>(value));
}

void writeBinaryValue(std::ostream& output, Field field, const StudyOutputRecord& record)
{
    switch (field) {
    case Field::ScenarioIndex: writeLittleEndian(output, static_cast<std::uint64_t>(record.scenarioIndex)); break;
    case Field::Iteration: writeLittleEndian(output, static_cast<std::uint64_t>(record.iteration)); break;
    case Field::SweepValue: writeDouble(output, record.sweepValue); break;
    case Field::EntityId: writeLittleEndian(output, static_cast<std::uint64_t>(record.entityId)); break;
    case Field::TimeS: writeDouble(output, record.timeS); break;
    case Field::Frame: output.put(static_cast<char>(record.frame == ReportingFrame::Ecef ? 1 : 0)); break;
    case Field::PositionX: writeDouble(output, record.positionX); break;
    case Field::PositionY: writeDouble(output, record.positionY); break;
    case Field::PositionZ: writeDouble(output, record.positionZ); break;
    case Field::LatitudeRad: writeDouble(output, record.latitudeRad); break;
    case Field::LongitudeRad: writeDouble(output, record.longitudeRad); break;
    case Field::AltitudeM: writeDouble(output, record.altitudeM); break;
    case Field::VelocityX: writeDouble(output, record.velocityX); break;
    case Field::VelocityY: writeDouble(output, record.velocityY); break;
    case Field::VelocityZ: writeDouble(output, record.velocityZ); break;
    case Field::SpeedMps: writeDouble(output, record.speedMps); break;
    case Field::MassKg: writeDouble(output, record.massKg); break;
    case Field::MaxAltitudeM: writeDouble(output, record.maxAltitudeM); break;
    case Field::MaxSpeedMps: writeDouble(output, record.maxSpeedMps); break;
    case Field::EntityCount: writeLittleEndian(output, static_cast<std::uint64_t>(record.entityCount)); break;
    case Field::ActiveEntities: writeLittleEndian(output, static_cast<std::uint64_t>(record.activeEntities)); break;
    case Field::PrimaryEntityActive: output.put(static_cast<char>(record.primaryEntityActive ? 1 : 0)); break;
    case Field::Active: output.put(static_cast<char>(record.active ? 1 : 0)); break;
    case Field::Status: output.put(static_cast<char>(record.status)); break;
    }
}

void writeCsv(
    const std::string& outputFile,
    StudyRecordType recordType,
    const std::vector<Field>& fields,
    const std::vector<StudyOutputRecord>& records)
{
    std::ofstream output(outputFile);
    if (!output.is_open()) {
        throw std::runtime_error("failed to open study CSV output: " + outputFile);
    }

    output << "# strikeengine_output_format=" << studyOutputFormatVersion << "\n";
    output << "# record_type=" << studyRecordTypeName(recordType) << "\n";
    output << "# fields=";
    for (std::size_t i = 0; i < fields.size(); ++i) {
        if (i != 0) output << '|';
        output << studyOutputFieldName(fields[i]);
    }
    output << "\n# frame=PER_ROW\n";

    output << std::setprecision(17);
    for (std::size_t i = 0; i < fields.size(); ++i) {
        if (i != 0) output << ',';
        output << studyOutputFieldName(fields[i]);
    }
    output << '\n';
    for (const auto& record : records) {
        for (std::size_t i = 0; i < fields.size(); ++i) {
            if (i != 0) output << ',';
            writeCsvValue(output, fields[i], record);
        }
        output << '\n';
    }
}

void writeBinary(
    const std::string& outputFile,
    StudyRecordType recordType,
    const std::vector<Field>& fields,
    const std::vector<StudyOutputRecord>& records)
{
    std::ofstream output(outputFile, std::ios::binary);
    if (!output.is_open()) {
        throw std::runtime_error("failed to open study binary output: " + outputFile);
    }

    constexpr char magic[] = "STRKOUT1";
    output.write(magic, sizeof(magic) - 1);
    writeLittleEndian(output, studyOutputFormatVersion);
    writeLittleEndian(output, static_cast<std::uint32_t>(recordType));
    writeLittleEndian(output, static_cast<std::uint32_t>(fields.size()));
    writeLittleEndian(output, static_cast<std::uint64_t>(records.size()));
    for (const auto field : fields) {
        writeLittleEndian(output, static_cast<std::uint16_t>(field));
    }
    for (const auto& record : records) {
        for (const auto field : fields) writeBinaryValue(output, field, record);
    }
}

StudyStatus statusFromBatch(const BatchRunResult& result)
{
    return result.primaryEntityActive ? StudyStatus::Completed : StudyStatus::Impacted;
}

} // namespace

const char* studyRecordTypeName(StudyRecordType type)
{
    switch (type) {
    case StudyRecordType::Trajectory: return "trajectory";
    case StudyRecordType::ParameterSweep: return "parameter_sweep";
    case StudyRecordType::MonteCarlo: return "monte_carlo";
    case StudyRecordType::BatchSummary: return "batch_summary";
    }
    return "unknown";
}

const char* studyStatusName(StudyStatus status)
{
    switch (status) {
    case StudyStatus::Active: return "ACTIVE";
    case StudyStatus::Completed: return "COMPLETED";
    case StudyStatus::Impacted: return "IMPACTED";
    }
    return "UNKNOWN";
}

const char* studyOutputFieldName(StudyOutputField field)
{
    switch (field) {
    case StudyOutputField::ScenarioIndex: return "ScenarioIndex";
    case StudyOutputField::Iteration: return "Iteration";
    case StudyOutputField::SweepValue: return "SweepValue";
    case StudyOutputField::EntityId: return "EntityId";
    case StudyOutputField::TimeS: return "Time_s";
    case StudyOutputField::Frame: return "Frame";
    case StudyOutputField::PositionX: return "PositionX_m";
    case StudyOutputField::PositionY: return "PositionY_m";
    case StudyOutputField::PositionZ: return "PositionZ_m";
    case StudyOutputField::LatitudeRad: return "Latitude_rad";
    case StudyOutputField::LongitudeRad: return "Longitude_rad";
    case StudyOutputField::AltitudeM: return "Altitude_m";
    case StudyOutputField::VelocityX: return "VelocityX_mps";
    case StudyOutputField::VelocityY: return "VelocityY_mps";
    case StudyOutputField::VelocityZ: return "VelocityZ_mps";
    case StudyOutputField::SpeedMps: return "Speed_mps";
    case StudyOutputField::MassKg: return "Mass_kg";
    case StudyOutputField::MaxAltitudeM: return "MaxAltitude_m";
    case StudyOutputField::MaxSpeedMps: return "MaxSpeed_mps";
    case StudyOutputField::EntityCount: return "EntityCount";
    case StudyOutputField::ActiveEntities: return "ActiveEntities";
    case StudyOutputField::PrimaryEntityActive: return "PrimaryEntityActive";
    case StudyOutputField::Active: return "Active";
    case StudyOutputField::Status: return "Status";
    }
    return "Unknown";
}

void StudyOutputWriter::write(
    const std::string& outputFile,
    StudyRecordType recordType,
    const std::vector<StudyOutputRecord>& records,
    const StudyOutputConfig& config)
{
    const auto fields = resolveFields(recordType, config);
    if (config.format == StudyOutputFormat::Binary) {
        writeBinary(outputFile, recordType, fields, records);
    } else {
        writeCsv(outputFile, recordType, fields, records);
    }
}

void StudyOutputWriter::writeBatchResults(
    const std::string& outputFile,
    const std::vector<BatchRunResult>& results,
    const StudyOutputConfig& config)
{
    std::vector<StudyOutputRecord> records;
    records.reserve(results.size());
    for (const auto& result : results) {
        StudyOutputRecord record;
        record.scenarioIndex = result.scenarioIndex;
        record.entityId = result.primaryEntityId;
        record.timeS = result.endTime;
        record.frame = result.frame == "ECEF"
            ? ReportingFrame::Ecef
            : ReportingFrame::LocalEnu;
        record.positionX = result.finalPositionX;
        record.positionY = result.finalPositionY;
        record.positionZ = result.finalPositionZ;
        record.latitudeRad = result.finalLatitudeRad;
        record.longitudeRad = result.finalLongitudeRad;
        record.altitudeM = result.finalAltitudeM;
        record.maxAltitudeM = result.maxAltitudeM;
        record.maxSpeedMps = result.maxSpeedMps;
        record.entityCount = result.entityCount;
        record.activeEntities = result.activeEntities;
        record.primaryEntityActive = result.primaryEntityActive;
        record.active = result.primaryEntityActive;
        record.status = statusFromBatch(result);
        records.push_back(record);
    }
    write(outputFile, StudyRecordType::BatchSummary, records, config);
}

} // namespace StrikeEngine::Simulation
