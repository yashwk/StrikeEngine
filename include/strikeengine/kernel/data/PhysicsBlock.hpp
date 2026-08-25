#pragma once
#include <vector>
#include <cstddef>

namespace StrikeEngine::Kernel {

struct PhysicsBlock {
	std::vector<double> px, py, pz;
	std::vector<double> vx, vy, vz;
	std::vector<double> ax, ay, az;

	std::vector<double> qw, qx, qy, qz;
	std::vector<double> wx, wy, wz;
	std::vector<double> alphax, alphay, alphaz; // Angular accelerations

	std::vector<double> Ixx, Iyy, Izz; // Moments of Inertia

	std::vector<double> mass;

	std::vector<bool> active;

	size_t size = 0;
};

} // namespace StrikeEngine::Kernel
