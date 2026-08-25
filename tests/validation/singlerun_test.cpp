#include "../../simulation/SingleRun.hpp"
#include <iostream>

int main() {
    using namespace StrikeEngine;

    Simulation::SingleRun runner(0.01, 100.0); // 100Hz, max 100s

    Kernel::VehicleInitState init;
    init.px = 0.0;
    init.py = 0.0;
    init.pz = 1000.0; // Start at 1km altitude

    init.vx = 200.0;  // 200m/s horizontal
    init.vy = 0.0;
    init.vz = 0.0;

    init.qx = 0.0;
    init.qy = 0.0;
    init.qz = 0.0;
    init.qw = 1.0;

    init.wx = 0.0;
    init.wy = 0.0;
    init.wz = 0.0;

    init.mass = 500.0; // 500kg missile

    std::cout << "Starting SingleRun validation test...\n";
    runner.execute(init, "singlerun_output.csv");

    return 0;
}
