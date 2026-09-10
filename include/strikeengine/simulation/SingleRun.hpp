#pragma once

#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <strikeengine/kernel/config/ScenarioConfig.hpp>
#include <strikeengine/simulation/StudyOutput.hpp>
#include <string>

namespace StrikeEngine::Simulation {

    class SingleRun {
    public:
        SingleRun(double timeStep_s, double maxTime_s);

        /**
         * @brief Runs a single simulation and writes a versioned trajectory CSV.
         * @param init The initial state of the vehicle.
         * @param outputFile Path to the CSV file to output data to.
         */
        void execute(
            const Kernel::VehicleInitState& init,
            const std::string& outputFile,
            const StudyOutputConfig& outputConfig = {});

        /**
         * @brief Runs a single simulation with an explicit environment.
         *
         * The CSV includes the selected frame and WGS84 geodetic coordinates.
         * Position/velocity columns remain in the selected simulation frame;
         * altitude is local Z in local mode and geodetic altitude in ECEF mode.
         *
         * NOTE: this overload still flies a default vehicle (no design
         * aero/motor). Prefer the ScenarioEntityConfig overload below when a
         * design manifest and scenario environment must be honored.
         */
        void execute(
            const Kernel::VehicleInitState& init,
            const Kernel::EnvironmentConfig& environment,
            const std::string& outputFile,
            const StudyOutputConfig& outputConfig = {});

        /**
         * @brief Runs a single simulation of a full scenario entity.
         *
         * Uses the entity's init state, its vehicle config (design aero,
         * motor, seeker, warhead — profile ids resolved by createVehicle)
         * and the given environment, so the trajectory reflects the designed
         * vehicle in the scenario's coordinate model instead of a bare
         * inertial body.
         */
        void execute(
            const Kernel::ScenarioEntityConfig& entity,
            const Kernel::EnvironmentConfig& environment,
            const std::string& outputFile,
            const StudyOutputConfig& outputConfig = {});

    private:
        double dt;
        double maxTime;
    };

} // namespace StrikeEngine::Simulation
