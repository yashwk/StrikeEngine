#pragma once

#include <vector>
#include <cstddef>
#include <array>

namespace StrikeEngine::Kernel {

    struct NavigationBlock {
        // Estimated State (Navigation Computer's belief)
        std::vector<double> estPx, estPy, estPz;    // Position
        std::vector<double> estVx, estVy, estVz;    // Velocity
        std::vector<double> estQx, estQy, estQz, estQw; // Attitude Quaternion
        std::vector<double> estWx, estWy, estWz;    // Angular Rates

        // Estimated Sensor Biases
        std::vector<double> estAccelBiasX, estAccelBiasY, estAccelBiasZ;
        std::vector<double> estGyroBiasX, estGyroBiasY, estGyroBiasZ;

        // Error-State EKF Covariance (Diagonal elements for simplification in DoD for now)
        // 15 states: pos(3), vel(3), att(3), accelBias(3), gyroBias(3)
        std::vector<std::array<double, 15>> covarianceDiag;

        // Status
        std::vector<bool> isAligned;

        std::size_t size = 0;
    };

} // namespace StrikeEngine::Kernel
