#include <strikeengine/simulation/SingleRun.hpp>
#include <strikeengine/simulation/Reporting.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace StrikeEngine::Simulation {

    SingleRun::SingleRun(double timeStep_s, double maxTime_s) 
        : dt(timeStep_s), maxTime(maxTime_s) {}

    void SingleRun::execute(
        const Kernel::VehicleInitState& init,
        const std::string& outputFile)
    {
        execute(init, Kernel::EnvironmentConfig{}, outputFile);
    }

    void SingleRun::execute(
        const Kernel::VehicleInitState& init,
        const Kernel::EnvironmentConfig& environment,
        const std::string& outputFile)
    {
        Kernel::SimulationKernel kernel;
        kernel.setEnvironment(environment);
        kernel.initialize();

        Kernel::PhysicsId id = kernel.createVehicle(init);

        std::ofstream out(outputFile);
        if (!out.is_open()) {
            std::cerr << "Failed to open output file: " << outputFile << std::endl;
            return;
        }

        writeCsvMetadata(out, "trajectory", reportingFrameName(reportingFrame(environment)));
        out << "Time_s,Frame,PositionX_m,PositionY_m,PositionZ_m,"
               "Latitude_rad,Longitude_rad,Altitude_m,"
               "VelocityX_mps,VelocityY_mps,VelocityZ_mps,Speed_mps,Mass_kg\n";

        const auto& physics = kernel.getPhysics();

        while (kernel.getSimulationTime() <= maxTime) {
            if (!physics.active[id]) break;

            const auto state = reportState(physics, id, environment);
            out << std::fixed << std::setprecision(9)
                << kernel.getSimulationTime() << ","
                << reportingFrameName(state.frame) << ","
                << state.positionX << "," << state.positionY << "," << state.positionZ << ","
                << state.latitudeRad << "," << state.longitudeRad << "," << state.altitudeM << ","
                << state.velocityX << "," << state.velocityY << "," << state.velocityZ << ","
                << state.speedMps << "," << state.massKg << "\n";

            const double remaining = maxTime - kernel.getSimulationTime();
            if (remaining <= 0.0) break;
            kernel.step(std::min(dt, remaining));
        }

        out.close();
        std::cout << "Simulation completed. Impact/End time: " << kernel.getSimulationTime() << "s. Output saved to " << outputFile << std::endl;
    }

} // namespace StrikeEngine::Simulation
