#include <strikeengine/kernel/config/ScenarioConfig.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>
#include <strikeengine/simulation/BatchRunner.hpp>
#include <strikeengine/simulation/MonteCarlo.hpp>
#include <strikeengine/simulation/ParamSweep.hpp>
#include <strikeengine/simulation/SingleRun.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
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

    std::remove(localPath.c_str());
    std::remove(ecefPath.c_str());
    std::remove(sweepPath.c_str());
    std::remove(monteCarloPath.c_str());
    std::remove(binaryPath.c_str());
    std::remove(batchPath.c_str());

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
