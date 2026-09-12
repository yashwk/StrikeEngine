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
    // Fail loudly on I/O errors (disk full, ENOSPC) instead of printing
    // "Data saved" next to a truncated file.
    output.flush();
    if (!output) {
        throw std::runtime_error("failed writing study CSV output: " + outputFile);
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
    // Fail loudly on I/O errors (disk full, ENOSPC) instead of printing
    // "Data saved" next to a truncated file.
    output.flush();
    if (!output) {
        throw std::runtime_error("failed writing study binary output: " + outputFile);
    }
}

std::uint8_t readUint8(std::istream& input, const char* what)
{
    char byte = '\0';
    input.read(&byte, 1);
    if (input.gcount() != 1) {
        throw std::runtime_error(
            std::string("unexpected end of study binary input while reading ") + what);
    }
    return static_cast<std::uint8_t>(static_cast<unsigned char>(byte));
}

std::uint16_t readUint16(std::istream& input, const char* what)
{
    const auto low = readUint8(input, what);
    const auto high = readUint8(input, what);
    return static_cast<std::uint16_t>(low) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(high) << 8);
}

std::uint32_t readUint32(std::istream& input, const char* what)
{
    std::uint32_t value = 0;
    for (int byte = 0; byte < 4; ++byte) {
        value |= static_cast<std::uint32_t>(readUint8(input, what)) << (byte * 8);
    }
    return value;
}

std::uint64_t readUint64(std::istream& input, const char* what)
{
    std::uint64_t value = 0;
    for (int byte = 0; byte < 8; ++byte) {
        value |= static_cast<std::uint64_t>(readUint8(input, what)) << (byte * 8);
    }
    return value;
}

double readDouble(std::istream& input, const char* what)
{
    return std::bit_cast<double>(readUint64(input, what));
}

void readBinaryValue(std::istream& input, Field field, StudyOutputRecord& record)
{
    switch (field) {
    case Field::ScenarioIndex:
        record.scenarioIndex = static_cast<std::size_t>(readUint64(input, "ScenarioIndex"));
        break;
    case Field::Iteration:
        record.iteration = static_cast<std::size_t>(readUint64(input, "Iteration"));
        break;
    case Field::SweepValue: record.sweepValue = readDouble(input, "SweepValue"); break;
    case Field::EntityId:
        record.entityId = static_cast<std::size_t>(readUint64(input, "EntityId"));
        break;
    case Field::TimeS: record.timeS = readDouble(input, "TimeS"); break;
    case Field::Frame:
    {
        const auto value = readUint8(input, "Frame");
        if (value > 1) {
            throw std::runtime_error("study binary input has an invalid frame value");
        }
        record.frame = value == 1 ? ReportingFrame::Ecef : ReportingFrame::LocalEnu;
        break;
    }
    case Field::PositionX: record.positionX = readDouble(input, "PositionX"); break;
    case Field::PositionY: record.positionY = readDouble(input, "PositionY"); break;
    case Field::PositionZ: record.positionZ = readDouble(input, "PositionZ"); break;
    case Field::LatitudeRad: record.latitudeRad = readDouble(input, "LatitudeRad"); break;
    case Field::LongitudeRad: record.longitudeRad = readDouble(input, "LongitudeRad"); break;
    case Field::AltitudeM: record.altitudeM = readDouble(input, "AltitudeM"); break;
    case Field::VelocityX: record.velocityX = readDouble(input, "VelocityX"); break;
    case Field::VelocityY: record.velocityY = readDouble(input, "VelocityY"); break;
    case Field::VelocityZ: record.velocityZ = readDouble(input, "VelocityZ"); break;
    case Field::SpeedMps: record.speedMps = readDouble(input, "SpeedMps"); break;
    case Field::MassKg: record.massKg = readDouble(input, "MassKg"); break;
    case Field::MaxAltitudeM: record.maxAltitudeM = readDouble(input, "MaxAltitudeM"); break;
    case Field::MaxSpeedMps: record.maxSpeedMps = readDouble(input, "MaxSpeedMps"); break;
    case Field::EntityCount:
        record.entityCount = static_cast<std::size_t>(readUint64(input, "EntityCount"));
        break;
    case Field::ActiveEntities:
        record.activeEntities = static_cast<std::size_t>(readUint64(input, "ActiveEntities"));
        break;
    case Field::PrimaryEntityActive:
        record.primaryEntityActive = readUint8(input, "PrimaryEntityActive") != 0;
        break;
    case Field::Active:
        record.active = readUint8(input, "Active") != 0;
        break;
    case Field::Status:
    {
        const auto value = readUint8(input, "Status");
        if (value > static_cast<std::uint8_t>(StudyStatus::Impacted)) {
            throw std::runtime_error("study binary input has an invalid status value");
        }
        record.status = static_cast<StudyStatus>(value);
        break;
    }
    }
}

StudyStatus statusFromBatch(const BatchRunResult& result)
{
    // Empty scenarios (no entities) simulated nothing and cannot have
    // impacted: mirror BatchRunner's COMPLETED status for them.
    if (result.entityCount == 0) return StudyStatus::Completed;
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

StudyOutputData StudyOutputReader::read(const std::string& inputFile)
{
    std::ifstream input(inputFile, std::ios::binary);
    if (!input.is_open()) {
        throw std::runtime_error("failed to open study binary input: " + inputFile);
    }

    char magic[8]{};
    input.read(magic, sizeof(magic));
    if (input.gcount() != static_cast<std::streamsize>(sizeof(magic))) {
        throw std::runtime_error("unexpected end of study binary input: missing magic");
    }
    if (std::string(magic, sizeof(magic)) != "STRKOUT1") {
        throw std::runtime_error("study binary input has an invalid magic");
    }

    const auto version = readUint32(input, "format version");
    if (version != studyOutputFormatVersion) {
        throw std::runtime_error(
            "unsupported study binary format version " + std::to_string(version));
    }

    const auto recordTypeValue = readUint32(input, "record type");
    const auto recordType = static_cast<StudyRecordType>(recordTypeValue);
    if (recordType != StudyRecordType::Trajectory &&
        recordType != StudyRecordType::ParameterSweep &&
        recordType != StudyRecordType::MonteCarlo &&
        recordType != StudyRecordType::BatchSummary) {
        throw std::runtime_error("study binary input has an unknown record type");
    }

    const auto fieldCount = readUint32(input, "field count");
    const auto recordCount = readUint64(input, "record count");

    StudyOutputData data;
    data.recordType = recordType;
    for (std::uint32_t i = 0; i < fieldCount; ++i) {
        const auto fieldValue = readUint16(input, "field ID");
        if (fieldValue > static_cast<std::uint16_t>(StudyOutputField::Status)) {
            throw std::runtime_error("study binary input has an unknown output field");
        }
        data.fields.push_back(static_cast<StudyOutputField>(fieldValue));
    }

    for (std::uint64_t i = 0; i < recordCount; ++i) {
        StudyOutputRecord record;
        for (const auto field : data.fields) readBinaryValue(input, field, record);
        data.records.push_back(record);
    }
    return data;
}

} // namespace StrikeEngine::Simulation
