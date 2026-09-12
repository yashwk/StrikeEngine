#include <strikeengine/simulation/SingleRun.hpp>
#include <strikeengine/simulation/Reporting.hpp>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace StrikeEngine::Simulation {

    namespace {
        // Shared loop body for both execute() overloads. The terminal record
        // must exist even when the clamped final step overshoots maxTime in
        // floating point (t + (maxTime - t) can round above maxTime, skipping
        // the last loop pass) — otherwise the CSV ends on an Active row.
        void runTrajectory(
            Kernel::SimulationKernel& kernel,
            Kernel::PhysicsId id,
            double maxTime,
            double dt,
            const Kernel::EnvironmentConfig& environment,
            std::vector<StudyOutputRecord>& records)
    {
        const auto& physics = kernel.getPhysics();

        const auto appendRecord = [&]() {
            const auto state = reportState(physics, id, environment);
            StudyOutputRecord record;
            record.entityId = id;
            record.timeS = kernel.getSimulationTime();
            record.frame = state.frame;
            record.positionX = state.positionX;
            record.positionY = state.positionY;
            record.positionZ = state.positionZ;
            record.latitudeRad = state.latitudeRad;
            record.longitudeRad = state.longitudeRad;
            record.altitudeM = state.altitudeM;
            record.velocityX = state.velocityX;
            record.velocityY = state.velocityY;
            record.velocityZ = state.velocityZ;
            record.speedMps = state.speedMps;
            record.massKg = state.massKg;
            record.active = physics.active[id];
            record.status = !record.active
                ? StudyStatus::Impacted
                : (record.timeS >= maxTime ? StudyStatus::Completed : StudyStatus::Active);
            records.push_back(record);
        };

        while (kernel.getSimulationTime() <= maxTime) {
            appendRecord();

            if (!physics.active[id]) break;

            const double remaining = maxTime - kernel.getSimulationTime();
            if (remaining <= 0.0) break;
            kernel.step(std::min(dt, remaining));
        }

        // Terminal state guard: with the entity still alive and no record yet
        // at/after maxTime, the last pass was skipped by the overshoot above.
        if (physics.active[id] &&
            (records.empty() || records.back().timeS < maxTime)) {
            appendRecord();
        }
    }
    }

    SingleRun::SingleRun(double timeStep_s, double maxTime_s)
        : dt(timeStep_s), maxTime(maxTime_s)
    {
        if (dt <= 0.0) {
            throw std::invalid_argument("SingleRun timestep must be positive");
        }
        if (maxTime < 0.0) {
            throw std::invalid_argument("SingleRun max time cannot be negative");
        }
    }

    void SingleRun::execute(
        const Kernel::VehicleInitState& init,
        const std::string& outputFile,
        const StudyOutputConfig& outputConfig)
    {
        execute(init, Kernel::EnvironmentConfig{}, outputFile, outputConfig);
    }

    void SingleRun::execute(
        const Kernel::VehicleInitState& init,
        const Kernel::EnvironmentConfig& environment,
        const std::string& outputFile,
        const StudyOutputConfig& outputConfig)
    {
        Kernel::SimulationKernel kernel;
        kernel.setEnvironment(environment);
        kernel.initialize();

        Kernel::PhysicsId id = kernel.createVehicle(init);
        std::vector<StudyOutputRecord> records;

        runTrajectory(kernel, id, maxTime, dt, environment, records);

        StudyOutputWriter::write(
            outputFile, StudyRecordType::Trajectory, records, outputConfig);
        std::cout << "Simulation completed. Impact/End time: " << kernel.getSimulationTime() << "s. Output saved to " << outputFile << std::endl;
    }

    void SingleRun::execute(
        const Kernel::ScenarioEntityConfig& entity,
        const Kernel::EnvironmentConfig& environment,
        const std::string& outputFile,
        const StudyOutputConfig& outputConfig)
    {
        Kernel::SimulationKernel kernel;
        kernel.setEnvironment(environment);
        kernel.initialize();

        Kernel::PhysicsId id = kernel.createVehicle(entity.initState, entity.vehicleConfig);
        std::vector<StudyOutputRecord> records;

        runTrajectory(kernel, id, maxTime, dt, environment, records);

        StudyOutputWriter::write(
            outputFile, StudyRecordType::Trajectory, records, outputConfig);
        std::cout << "Simulation completed. Impact/End time: " << kernel.getSimulationTime() << "s. Output saved to " << outputFile << std::endl;
    }

} // namespace StrikeEngine::Simulation
