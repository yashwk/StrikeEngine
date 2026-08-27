#include <strikeengine/kernel/config/ScenarioConfig.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>
#include <strikeengine/simulation/BatchRunner.hpp>
#include <strikeengine/simulation/MonteCarlo.hpp>
#include <strikeengine/simulation/ParamSweep.hpp>
#include <strikeengine/simulation/SingleRun.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace StrikeEngine;

namespace {

std::vector<std::string> dataRows(const std::string& path)
{
    std::ifstream input(path);
    std::vector<std::string> rows;
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line[0] != '#') {
            if (line.rfind("Time_s,", 0) != 0 &&
                line.rfind("EntityId,", 0) != 0 &&
                line.rfind("SweepValue,", 0) != 0 &&
                line.rfind("Iteration,", 0) != 0) {
                rows.push_back(line);
            }
        }
    }
    return rows;
}

std::vector<std::string> fields(const std::string& row)
{
    std::vector<std::string> result;
    std::stringstream stream(row);
    std::string field;
    while (std::getline(stream, field, ',')) result.push_back(field);
    return result;
}

Kernel::ScenarioConfig makeEcefScenario()
{
    Kernel::ScenarioConfig scenario;
    scenario.name = "frame-aware-reporting";
    scenario.primaryEntityIndex = 1;
    scenario.environment.earth.useEcefTruth = true;
    scenario.environment.earth.referenceLatitudeRad = 0.1;
    scenario.environment.earth.referenceLongitudeRad = 0.2;

    for (double altitude : {1000.0, 1500.0}) {
        Kernel::ScenarioEntityConfig entity;
        const auto position = Models::geodeticToEcef({0.1, 0.2, altitude});
        entity.initState.px = position.x;
        entity.initState.py = position.y;
        entity.initState.pz = position.z;
        entity.initState.qw = 1.0;
        entity.initState.mass = 100.0;
        entity.vehicleConfig.referenceArea = 0.0;
        entity.vehicleConfig.cd = 0.0;
        scenario.entities.push_back(entity);
    }
    return scenario;
}

void check(bool condition, const char* message, int& failures)
{
    std::printf("  [%s] %s\n", condition ? "PASS" : "FAIL", message);
    if (!condition) ++failures;
}

void checkField(
    Simulation::StudyOutputField field,
    const Simulation::StudyOutputRecord& actual,
    const Simulation::StudyOutputRecord& expected,
    int& failures)
{
    switch (field) {
    case Simulation::StudyOutputField::ScenarioIndex:
        check(actual.scenarioIndex == expected.scenarioIndex,
              "ScenarioIndex round-trips exactly", failures);
        break;
    case Simulation::StudyOutputField::Iteration:
        check(actual.iteration == expected.iteration,
              "Iteration round-trips exactly", failures);
        break;
    case Simulation::StudyOutputField::SweepValue:
        check(actual.sweepValue == expected.sweepValue,
              "SweepValue round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::EntityId:
        check(actual.entityId == expected.entityId,
              "EntityId round-trips exactly", failures);
        break;
    case Simulation::StudyOutputField::TimeS:
        check(actual.timeS == expected.timeS,
              "TimeS round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::Frame:
        check(actual.frame == expected.frame,
              "Frame round-trips exactly", failures);
        break;
    case Simulation::StudyOutputField::PositionX:
        check(actual.positionX == expected.positionX,
              "PositionX round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::PositionY:
        check(actual.positionY == expected.positionY,
              "PositionY round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::PositionZ:
        check(actual.positionZ == expected.positionZ,
              "PositionZ round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::LatitudeRad:
        check(actual.latitudeRad == expected.latitudeRad,
              "LatitudeRad round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::LongitudeRad:
        check(actual.longitudeRad == expected.longitudeRad,
              "LongitudeRad round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::AltitudeM:
        check(actual.altitudeM == expected.altitudeM,
              "AltitudeM round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::VelocityX:
        check(actual.velocityX == expected.velocityX,
              "VelocityX round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::VelocityY:
        check(actual.velocityY == expected.velocityY,
              "VelocityY round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::VelocityZ:
        check(actual.velocityZ == expected.velocityZ,
              "VelocityZ round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::SpeedMps:
        check(actual.speedMps == expected.speedMps,
              "SpeedMps round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::MassKg:
        check(actual.massKg == expected.massKg,
              "MassKg round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::MaxAltitudeM:
        check(actual.maxAltitudeM == expected.maxAltitudeM,
              "MaxAltitudeM round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::MaxSpeedMps:
        check(actual.maxSpeedMps == expected.maxSpeedMps,
              "MaxSpeedMps round-trips bit-identically", failures);
        break;
    case Simulation::StudyOutputField::EntityCount:
        check(actual.entityCount == expected.entityCount,
              "EntityCount round-trips exactly", failures);
        break;
    case Simulation::StudyOutputField::ActiveEntities:
        check(actual.activeEntities == expected.activeEntities,
              "ActiveEntities round-trips exactly", failures);
        break;
    case Simulation::StudyOutputField::PrimaryEntityActive:
        check(actual.primaryEntityActive == expected.primaryEntityActive,
              "PrimaryEntityActive round-trips exactly", failures);
        break;
    case Simulation::StudyOutputField::Active:
        check(actual.active == expected.active,
              "Active round-trips exactly", failures);
        break;
    case Simulation::StudyOutputField::Status:
        check(actual.status == expected.status,
              "Status round-trips exactly", failures);
        break;
    }
}

void checkRoundTrip(
    const std::string& path,
    Simulation::StudyRecordType recordType,
    const std::vector<Simulation::StudyOutputField>& fields,
    const Simulation::StudyOutputRecord& expected,
    int& failures)
{
    const auto data = Simulation::StudyOutputReader::read(path);
    check(data.recordType == recordType,
          "binary reader restores the record type", failures);
    check(data.fields == fields,
          "binary reader restores the field list and order", failures);
    check(data.records.size() == 1,
          "binary reader restores the record count", failures);
    if (data.records.size() != 1) return;
    for (const auto field : fields) {
        checkField(field, data.records.front(), expected, failures);
    }
}

} // namespace

int main()
{
    std::printf("=== reporting_test: frame-aware study outputs ===\n");
    int failures = 0;

    const std::string localPath = "reporting_local.csv";
    const std::string ecefPath = "reporting_ecef.csv";
    const std::string sweepPath = "reporting_sweep.csv";
    const std::string monteCarloPath = "reporting_monte_carlo.csv";
    const std::string binaryPath = "reporting_trajectory.bin";
    const std::string batchPath = "reporting_batch.bin";

    Kernel::VehicleInitState localInit{};
    localInit.pz = 1000.0;
    localInit.qw = 1.0;
    localInit.mass = 100.0;
    Simulation::SingleRun singleRun(0.01, 0.0);
    singleRun.execute(localInit, localPath);
    {
        std::ifstream input(localPath);
        std::stringstream contents;
        contents << input.rdbuf();
        const auto rows = dataRows(localPath);
        check(contents.str().find("# strikeengine_output_format=1") != std::string::npos,
              "SingleRun writes the versioned CSV metadata", failures);
        check(contents.str().find("# frame=PER_ROW") != std::string::npos &&
                  contents.str().find("Latitude_rad") != std::string::npos,
              "SingleRun declares the local frame and geodetic columns", failures);
        check(rows.size() == 1 && fields(rows.front()).size() == 16 &&
                  fields(rows.front())[2] == "LOCAL_ENU" &&
                  std::abs(std::stod(fields(rows.front())[8]) - 1000.0) < 1e-9,
              "SingleRun reports local altitude from world Z", failures);
    }

    const auto ecefScenario = makeEcefScenario();
    const auto ecefPosition = Models::geodeticToEcef({0.1, 0.2, 1000.0});
    Kernel::VehicleInitState ecefInit{};
    ecefInit.px = ecefPosition.x;
    ecefInit.py = ecefPosition.y;
    ecefInit.pz = ecefPosition.z;
    ecefInit.qw = 1.0;
    ecefInit.mass = 100.0;
    singleRun.execute(ecefInit, ecefScenario.environment, ecefPath);
    {
        const auto rows = dataRows(ecefPath);
        const auto row = fields(rows.front());
        check(rows.size() == 1 && row.size() == 16 && row[2] == "ECEF",
              "SingleRun reports the absolute ECEF frame", failures);
        check(std::abs(std::stod(row[3])) > 6.0e6 &&
                  std::abs(std::stod(row[8]) - 1000.0) < 1e-5 &&
                  std::abs(std::stod(row[6]) - 0.1) < 1e-10 &&
                  std::abs(std::stod(row[7]) - 0.2) < 1e-10,
              "SingleRun derives WGS84 coordinates from ECEF position", failures);
    }

    Simulation::ParamSweep sweep(0.01, 0.0);
    sweep.execute(
        ecefScenario, 1.0, 1.0, 1,
        [](Kernel::ScenarioConfig&, double) {}, sweepPath);
    {
        const auto rows = dataRows(sweepPath);
        const auto row = fields(rows.front());
        check(rows.size() == 1 && row.size() == 14 && row[1] == "1" &&
                  row[2] == "ECEF" && std::abs(std::stod(row[9]) - 1500.0) < 1e-5,
              "ParamSweep uses the explicit primary entity and ECEF altitude", failures);
    }

    Simulation::StudyOutputConfig selectedFields;
    selectedFields.fields = {
        Simulation::StudyOutputField::SweepValue,
        Simulation::StudyOutputField::EntityId,
        Simulation::StudyOutputField::Frame,
        Simulation::StudyOutputField::AltitudeM,
        Simulation::StudyOutputField::Status};
    sweep.execute(
        ecefScenario, 1.0, 1.0, 1,
        [](Kernel::ScenarioConfig&, double) {}, sweepPath, selectedFields);
    {
        std::ifstream input(sweepPath);
        std::stringstream contents;
        contents << input.rdbuf();
        check(contents.str().find(
                  "SweepValue,EntityId,Frame,Altitude_m,Status") != std::string::npos &&
                  contents.str().find("MaxAltitude_m") == std::string::npos,
              "study output supports explicit field selection", failures);
    }

    Simulation::MonteCarlo monteCarlo(0.01, 0.0);
    monteCarlo.execute(
        ecefScenario, 1,
        [](Kernel::ScenarioConfig&, std::mt19937&) {}, monteCarloPath);
    {
        const auto rows = dataRows(monteCarloPath);
        const auto row = fields(rows.front());
        check(rows.size() == 1 && row.size() == 12 && row[1] == "1" &&
                  row[2] == "ECEF" && std::abs(std::stod(row[9]) - 1500.0) < 1e-5,
              "MonteCarlo uses the explicit primary entity and ECEF altitude", failures);
    }

    Simulation::BatchRunner batch(0.01, 0.0);
    const auto results = batch.execute({ecefScenario});
    check(results.size() == 1 && results[0].frame == "ECEF" &&
              results[0].primaryEntityId == 1 &&
              std::abs(results[0].finalAltitudeM - 1500.0) < 1e-5 &&
              results[0].maxAltitudeM >= results[0].finalAltitudeM,
          "BatchRunner returns frame-aware primary and aggregate metrics", failures);

    Simulation::StudyOutputConfig binaryConfig;
    binaryConfig.format = Simulation::StudyOutputFormat::Binary;
    binaryConfig.fields = {
        Simulation::StudyOutputField::EntityId,
        Simulation::StudyOutputField::Frame,
        Simulation::StudyOutputField::AltitudeM,
        Simulation::StudyOutputField::Status};
    singleRun.execute(ecefInit, ecefScenario.environment, binaryPath, binaryConfig);
    {
        std::ifstream input(binaryPath, std::ios::binary);
        char magic[8]{};
        input.read(magic, sizeof(magic));
        check(std::string(magic, sizeof(magic)) == "STRKOUT1" &&
                  input.good(),
              "study output writes the versioned binary magic", failures);
    }

    Simulation::StudyOutputWriter::writeBatchResults(batchPath, results, binaryConfig);
    {
        std::ifstream input(batchPath, std::ios::binary);
        char magic[8]{};
        input.read(magic, sizeof(magic));
        check(std::string(magic, sizeof(magic)) == "STRKOUT1" && input.good(),
              "BatchRunner results can be serialized as binary", failures);
    }

    const auto trajectoryData = Simulation::StudyOutputReader::read(binaryPath);
    check(trajectoryData.recordType == Simulation::StudyRecordType::Trajectory,
          "binary reader restores the trajectory record type", failures);
    check(trajectoryData.fields.size() == binaryConfig.fields.size() &&
              std::equal(binaryConfig.fields.begin(), binaryConfig.fields.end(),
                         trajectoryData.fields.begin()),
          "binary reader restores the selected trajectory field list", failures);
    check(trajectoryData.records.size() == 1,
          "binary reader restores the trajectory record count", failures);
    if (trajectoryData.records.size() == 1) {
        const auto& record = trajectoryData.records.front();
        check(record.entityId == 0 && record.frame == Simulation::ReportingFrame::Ecef,
              "binary reader decodes integer and frame fields", failures);
        check(std::abs(record.altitudeM - 1000.0) < 1e-5 &&
                  record.status == Simulation::StudyStatus::Completed,
              "binary reader decodes double and status fields", failures);
    }

    const auto batchData = Simulation::StudyOutputReader::read(batchPath);
    check(batchData.recordType == Simulation::StudyRecordType::BatchSummary,
          "binary reader restores the batch record type", failures);
    check(batchData.fields.size() == binaryConfig.fields.size() &&
              std::equal(binaryConfig.fields.begin(), binaryConfig.fields.end(),
                         batchData.fields.begin()),
          "binary reader restores the selected batch field list", failures);
    check(batchData.records.size() == 1,
          "binary reader restores the batch record count", failures);
    if (batchData.records.size() == 1) {
        const auto& record = batchData.records.front();
        check(record.entityId == results[0].primaryEntityId &&
                  record.frame == Simulation::ReportingFrame::Ecef,
              "binary reader decodes batch integer and frame fields", failures);
        check(record.altitudeM == results[0].finalAltitudeM &&
                  record.status == Simulation::StudyStatus::Completed,
              "binary reader decodes batch double and status fields", failures);
    }

    const std::string trajectoryRoundTripPath = "reporting_roundtrip_trajectory.bin";
    {
        Simulation::StudyOutputRecord record;
        record.entityId = 3;
        record.timeS = 12.5;
        record.frame = Simulation::ReportingFrame::Ecef;
        record.positionX = 1111.0;
        record.positionY = 2222.0;
        record.positionZ = 3333.0;
        record.latitudeRad = 0.11;
        record.longitudeRad = 0.22;
        record.altitudeM = 4444.0;
        record.velocityX = 55.0;
        record.velocityY = 66.0;
        record.velocityZ = 77.0;
        record.speedMps = 88.0;
        record.massKg = 99.0;
        record.active = true;
        record.status = Simulation::StudyStatus::Active;
        Simulation::StudyOutputConfig config;
        config.format = Simulation::StudyOutputFormat::Binary;
        config.fields = {
            Simulation::StudyOutputField::EntityId,
            Simulation::StudyOutputField::TimeS,
            Simulation::StudyOutputField::Frame,
            Simulation::StudyOutputField::PositionX,
            Simulation::StudyOutputField::PositionY,
            Simulation::StudyOutputField::PositionZ,
            Simulation::StudyOutputField::LatitudeRad,
            Simulation::StudyOutputField::LongitudeRad,
            Simulation::StudyOutputField::AltitudeM,
            Simulation::StudyOutputField::VelocityX,
            Simulation::StudyOutputField::VelocityY,
            Simulation::StudyOutputField::VelocityZ,
            Simulation::StudyOutputField::SpeedMps,
            Simulation::StudyOutputField::MassKg,
            Simulation::StudyOutputField::Active,
            Simulation::StudyOutputField::Status};
        Simulation::StudyOutputWriter::write(
            trajectoryRoundTripPath, Simulation::StudyRecordType::Trajectory,
            {record}, config);
        checkRoundTrip(trajectoryRoundTripPath,
                       Simulation::StudyRecordType::Trajectory,
                       config.fields, record, failures);
    }

    const std::string batchRoundTripPath = "reporting_roundtrip_batch.bin";
    {
        Simulation::StudyOutputRecord record;
        record.scenarioIndex = 7;
        record.entityId = 3;
        record.frame = Simulation::ReportingFrame::Ecef;
        record.timeS = 12.5;
        record.entityCount = 2;
        record.activeEntities = 1;
        record.positionX = 1111.0;
        record.positionY = 2222.0;
        record.positionZ = 3333.0;
        record.latitudeRad = 0.11;
        record.longitudeRad = 0.22;
        record.altitudeM = 4444.0;
        record.maxAltitudeM = 5555.0;
        record.maxSpeedMps = 88.0;
        record.primaryEntityActive = true;
        record.status = Simulation::StudyStatus::Completed;
        Simulation::StudyOutputConfig config;
        config.format = Simulation::StudyOutputFormat::Binary;
        config.fields = {
            Simulation::StudyOutputField::ScenarioIndex,
            Simulation::StudyOutputField::EntityId,
            Simulation::StudyOutputField::Frame,
            Simulation::StudyOutputField::TimeS,
            Simulation::StudyOutputField::EntityCount,
            Simulation::StudyOutputField::ActiveEntities,
            Simulation::StudyOutputField::PositionX,
            Simulation::StudyOutputField::PositionY,
            Simulation::StudyOutputField::PositionZ,
            Simulation::StudyOutputField::LatitudeRad,
            Simulation::StudyOutputField::LongitudeRad,
            Simulation::StudyOutputField::AltitudeM,
            Simulation::StudyOutputField::MaxAltitudeM,
            Simulation::StudyOutputField::MaxSpeedMps,
            Simulation::StudyOutputField::PrimaryEntityActive,
            Simulation::StudyOutputField::Status};
        Simulation::StudyOutputWriter::write(
            batchRoundTripPath, Simulation::StudyRecordType::BatchSummary,
            {record}, config);
        checkRoundTrip(batchRoundTripPath,
                       Simulation::StudyRecordType::BatchSummary,
                       config.fields, record, failures);
    }

    const std::string monteCarloRoundTripPath = "reporting_roundtrip_monte_carlo.bin";
    {
        Simulation::StudyOutputRecord record;
        record.iteration = 42;
        record.entityId = 3;
        record.frame = Simulation::ReportingFrame::LocalEnu;
        record.timeS = 1.5;
        record.altitudeM = 1000.0;
        record.active = false;
        record.status = Simulation::StudyStatus::Impacted;
        Simulation::StudyOutputConfig config;
        config.format = Simulation::StudyOutputFormat::Binary;
        config.fields = {
            Simulation::StudyOutputField::Iteration,
            Simulation::StudyOutputField::EntityId,
            Simulation::StudyOutputField::Frame,
            Simulation::StudyOutputField::TimeS,
            Simulation::StudyOutputField::AltitudeM,
            Simulation::StudyOutputField::Active,
            Simulation::StudyOutputField::Status};
        Simulation::StudyOutputWriter::write(
            monteCarloRoundTripPath, Simulation::StudyRecordType::MonteCarlo,
            {record}, config);
        checkRoundTrip(monteCarloRoundTripPath,
                       Simulation::StudyRecordType::MonteCarlo,
                       config.fields, record, failures);
    }

    const std::string sweepRoundTripPath = "reporting_roundtrip_sweep.bin";
    {
        Simulation::StudyOutputRecord record;
        record.sweepValue = 1.25;
        record.entityId = 3;
        record.frame = Simulation::ReportingFrame::LocalEnu;
        record.timeS = 1.5;
        record.altitudeM = 1000.0;
        record.status = Simulation::StudyStatus::Active;
        Simulation::StudyOutputConfig config;
        config.format = Simulation::StudyOutputFormat::Binary;
        config.fields = {
            Simulation::StudyOutputField::SweepValue,
            Simulation::StudyOutputField::EntityId,
            Simulation::StudyOutputField::Frame,
            Simulation::StudyOutputField::TimeS,
            Simulation::StudyOutputField::AltitudeM,
            Simulation::StudyOutputField::Status};
        Simulation::StudyOutputWriter::write(
            sweepRoundTripPath, Simulation::StudyRecordType::ParameterSweep,
            {record}, config);
        checkRoundTrip(sweepRoundTripPath,
                       Simulation::StudyRecordType::ParameterSweep,
                       config.fields, record, failures);
    }

    const std::string truncatedPath = "reporting_truncated.bin";
    {
        Simulation::StudyOutputRecord record;
        record.entityId = 1;
        record.timeS = 1.5;
        record.status = Simulation::StudyStatus::Completed;
        Simulation::StudyOutputConfig config;
        config.format = Simulation::StudyOutputFormat::Binary;
        config.fields = {
            Simulation::StudyOutputField::EntityId,
            Simulation::StudyOutputField::TimeS,
            Simulation::StudyOutputField::Status};
        Simulation::StudyOutputWriter::write(
            truncatedPath, Simulation::StudyRecordType::Trajectory,
            {record}, config);
        std::ifstream input(truncatedPath, std::ios::binary);
        const std::string contents((std::istreambuf_iterator<char>(input)),
                                   std::istreambuf_iterator<char>());
        input.close();
        std::ofstream output(truncatedPath, std::ios::binary | std::ios::trunc);
        output.write(contents.data(),
                     static_cast<std::streamsize>(contents.size() - 1));
        output.close();

        bool threw = false;
        try {
            (void)Simulation::StudyOutputReader::read(truncatedPath);
        } catch (const std::runtime_error&) {
            threw = true;
        }
        check(threw, "binary reader rejects a truncated tail", failures);
    }

    const std::string badMagicPath = "reporting_bad_magic.bin";
    {
        std::ofstream output(badMagicPath, std::ios::binary);
        output.write("STRKOUT2", 8);
        output.close();

        bool threw = false;
        try {
            (void)Simulation::StudyOutputReader::read(badMagicPath);
        } catch (const std::runtime_error&) {
            threw = true;
        }
        check(threw, "binary reader rejects an invalid magic", failures);
    }

    std::remove(localPath.c_str());
    std::remove(ecefPath.c_str());
    std::remove(sweepPath.c_str());
    std::remove(monteCarloPath.c_str());
    std::remove(binaryPath.c_str());
    std::remove(batchPath.c_str());
    std::remove(trajectoryRoundTripPath.c_str());
    std::remove(batchRoundTripPath.c_str());
    std::remove(monteCarloRoundTripPath.c_str());
    std::remove(sweepRoundTripPath.c_str());
    std::remove(truncatedPath.c_str());
    std::remove(badMagicPath.c_str());

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
