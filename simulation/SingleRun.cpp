#include "SingleRun.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>

namespace StrikeEngine::Simulation {

    SingleRun::SingleRun(double timeStep_s, double maxTime_s) 
        : dt(timeStep_s), maxTime(maxTime_s) {}

    void SingleRun::execute(const Kernel::VehicleInitState& init, const std::string& outputFile) {
        Kernel::SimulationKernel kernel;
        kernel.initialize();

        Kernel::PhysicsId id = kernel.createVehicle(init);

        std::ofstream out(outputFile);
        if (!out.is_open()) {
            std::cerr << "Failed to open output file: " << outputFile << std::endl;
            return;
        }

        // CSV Header
        out << "Time,X,Y,Z,Vx,Vy,Vz,Mass\n";

        const auto& physics = kernel.getPhysics();

        while (kernel.getSimulationTime() <= maxTime) {
            if (!physics.active[id]) break;

            // Log state
            out << std::fixed << std::setprecision(4)
                << kernel.getSimulationTime() << ","
                << physics.px[id] << "," << physics.py[id] << "," << physics.pz[id] << ","
                << physics.vx[id] << "," << physics.vy[id] << "," << physics.vz[id] << ","
                << physics.mass[id] << "\n";

            // Assuming Z is altitude (flat earth approximation). Stop if we hit ground after launch.
            if (kernel.getSimulationTime() > 1.0 && physics.pz[id] < 0.0) {
                break; 
            }

            kernel.step(dt);
        }

        out.close();
        std::cout << "Simulation completed. Impact/End time: " << kernel.getSimulationTime() << "s. Output saved to " << outputFile << std::endl;
    }

} // namespace StrikeEngine::Simulation
