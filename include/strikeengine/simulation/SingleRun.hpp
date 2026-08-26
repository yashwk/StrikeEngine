#pragma once

#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
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
        void execute(const Kernel::VehicleInitState& init, const std::string& outputFile);

        /**
         * @brief Runs a single simulation with an explicit environment.
         *
         * The CSV includes the selected frame and WGS84 geodetic coordinates.
         * Position/velocity columns remain in the selected simulation frame;
         * altitude is local Z in local mode and geodetic altitude in ECEF mode.
         */
        void execute(
            const Kernel::VehicleInitState& init,
            const Kernel::EnvironmentConfig& environment,
            const std::string& outputFile);

    private:
        double dt;
        double maxTime;
    };

} // namespace StrikeEngine::Simulation
