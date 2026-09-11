#pragma once

#include <vector>
#include <cstddef>
#include <array>

namespace StrikeEngine::Kernel {

    struct NavigationBlock {
        // Estimated State (Navigation Computer's belief)
        std::vector<double> estPx, estPy, estPz;    // Position
        std::vector<double> estVx, estVy, estVz;    // Velocity
        std::vector<double> estAx, estAy, estAz;    // World acceleration (m/s^2)
        std::vector<double> estQx, estQy, estQz, estQw; // Attitude Quaternion
        std::vector<double> estWx, estWy, estWz;    // Angular Rates

        // Estimated Sensor Biases
        std::vector<double> estAccelBiasX, estAccelBiasY, estAccelBiasZ;
        std::vector<double> estGyroBiasX, estGyroBiasY, estGyroBiasZ;

        // Error-state EKF covariance diagonal retained as a convenient public
        // readout. The coupled matrix below is the filter's source of truth.
        std::vector<std::array<double, 15>> covarianceDiag;

        // Full 15-state covariance: pos(3), vel(3), attitude(3),
        // accelerometer bias(3), gyro bias(3), stored row-major.
        std::vector<std::array<double, 225>> covarianceFull;

        // GPS fusion diagnostics. A rejected flag means at least one scalar
        // position/velocity innovation was outside the configured gate during
        // the most recent GPS update.
        std::vector<bool> lastGpsUpdateRejected;
        std::vector<double> lastGpsMaxInnovationSigma;
        // Baro / magnetometer aiding diagnostics (same convention).
        std::vector<bool> lastBaroRejected;
        std::vector<bool> lastMagRejected;

        // Two-sample coning/sculling state: previous IMU increments
        // (delta-angle, delta-velocity in body frame). Zeroed at alignment;
        // consumed only when insConingCompensationEnabled.
        std::vector<double> prevDThetaX, prevDThetaY, prevDThetaZ;
        std::vector<double> prevDVelX, prevDVelY, prevDVelZ;

        // Status
        std::vector<bool> isAligned;

        std::size_t size = 0;
    };

} // namespace StrikeEngine::Kernel
