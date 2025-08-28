#pragma once

#include "strikeengine/ecs/System.hpp"
#include "strikeengine/ecs/Registry.hpp"
#include <glm/glm.hpp>
#include <array>

namespace StrikeEngine {

    /** @brief Represents the 6-element state vector [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z]. */
    using KalmanStateVector = std::array<double, 6>;

    /** @brief Represents the 6x6 error covariance matrix. */
    using KalmanCovarianceMatrix = std::array<std::array<double, 6>, 6>;


    class NavigationSystem final : public System {
    public:
        NavigationSystem();
        void update(Registry& registry, double dt) override;

    private:
        // --- Kalman Filter State ---

        // State vector [position(3), velocity(3)].
        KalmanStateVector _state_estimate{};

        // Error covariance matrix.
        KalmanCovarianceMatrix _covariance{};

        /**
         * @brief Performs the Kalman filter's PREDICT step using IMU data.
         * @param dt The simulation time step.
         * @param imu_acceleration The acceleration measured by the IMU.
         */
        void predict(double dt, const glm::dvec3& imu_acceleration);

        /**
         * @brief Performs the Kalman filter's UPDATE step using a GPS measurement.
         * @param gps_position The noisy position measurement from the GPS.
         * @param gps_error The configured error (accuracy) of the GPS.
         */
        void update(const glm::dvec3& gps_position, double gps_error);
    };

} // namespace StrikeEngine
