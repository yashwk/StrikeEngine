#pragma once

#include "../kernel/SimulationKernel.hpp"
#include <string>

namespace StrikeEngine::Simulation {

    class SingleRun {
    public:
        SingleRun(double timeStep_s, double maxTime_s);

        /**
         * @brief Runs a single simulation and writes the trajectory to a CSV.
         * @param init The initial state of the vehicle.
         * @param outputFile Path to the CSV file to output data to.
         */
        void execute(const Kernel::VehicleInitState& init, const std::string& outputFile);

    private:
        double dt;
        double maxTime;
    };

} // namespace StrikeEngine::Simulation
