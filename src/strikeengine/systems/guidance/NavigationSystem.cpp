#include "strikeengine/systems/guidance/NavigationSystem.hpp"
#include "strikeengine/components/physics/IMUComponent.hpp"
#include "strikeengine/components/sensors/GPSComponent.hpp"
#include "strikeengine/components/physics/NavigationStateComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/physics/VelocityComponent.hpp"
#include "strikeengine/components/physics/ForceAccumulatorComponent.hpp" // To get ground truth acceleration
#include "strikeengine/components/physics/MassComponent.hpp"

#include <random>
#include <stdexcept>

namespace StrikeEngine {

    // --- Matrix Math Helper Functions (as defined previously) ---
    namespace KalmanMath {
        // ... (All the helper functions for 6x6 matrix math remain here) ...
        KalmanStateVector multiply(const KalmanCovarianceMatrix& m, const KalmanStateVector& v) {
            KalmanStateVector result{};
            for (int i = 0; i < 6; ++i) { for (int j = 0; j < 6; ++j) { result[i] += m[i][j] * v[j]; } }
            return result;
        }
        KalmanCovarianceMatrix multiply(const KalmanCovarianceMatrix& a, const KalmanCovarianceMatrix& b) {
            KalmanCovarianceMatrix result{};
            for (int i = 0; i < 6; ++i) { for (int j = 0; j < 6; ++j) { for (int k = 0; k < 6; ++k) { result[i][j] += a[i][k] * b[k][j]; } } }
            return result;
        }
        KalmanCovarianceMatrix add(const KalmanCovarianceMatrix& a, const KalmanCovarianceMatrix& b) {
            KalmanCovarianceMatrix result{};
            for(int i = 0; i < 6; ++i) { for(int j = 0; j < 6; ++j) { result[i][j] = a[i][j] + b[i][j]; } }
            return result;
        }
        KalmanCovarianceMatrix subtract(const KalmanCovarianceMatrix& a, const KalmanCovarianceMatrix& b) {
            KalmanCovarianceMatrix result{};
            for(int i = 0; i < 6; ++i) { for(int j = 0; j < 6; ++j) { result[i][j] = a[i][j] - b[i][j]; } }
            return result;
        }
        KalmanCovarianceMatrix transpose(const KalmanCovarianceMatrix& m) {
            KalmanCovarianceMatrix result{};
            for (int i = 0; i < 6; ++i) { for (int j = 0; j < 6; ++j) { result[j][i] = m[i][j]; } }
            return result;
        }
        glm::dmat3 invert(const glm::dmat3& m) {
            double determinant = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
            if (std::abs(determinant) < 1e-10) { throw std::runtime_error("Matrix is singular and cannot be inverted."); }
            double  inverse_determinant = 1.0 / determinant;
            glm::dmat3 inv;
            inv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) *  inverse_determinant; inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) *  inverse_determinant; inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) *  inverse_determinant;
            inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) *  inverse_determinant; inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) *  inverse_determinant; inv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) *  inverse_determinant;
            inv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) *  inverse_determinant; inv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) *  inverse_determinant; inv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) *  inverse_determinant;
            return inv;
        }
    }

    NavigationSystem::NavigationSystem() {
        _state_estimate.fill(0.0);
        _covariance.fill({});
        for (int i = 0; i < 6; ++i) {
            _covariance[i].fill(0.0);
            _covariance[i][i] = 1.0;
        }
    }

    void NavigationSystem::update(Registry& registry, double dt) {
        auto view = registry.view<IMUComponent, NavigationStateComponent, TransformComponent, ForceAccumulatorComponent, MassComponent>();

        // A single random number generator for the whole system update
        std::random_device rd;
        std::mt19937 gen(rd());

        for (auto [entity, imu, navigation_state, transform, accumulator, mass] : view) {

            // --- 1. Simulate and Process IMU Data ---
            // Get the "perfect" ground truth acceleration for this frame.
            glm::dvec3 ground_truth_acceleration = accumulator.totalForce * mass.inverseMass;

            constexpr double g_to_ms2 = 9.80665;
            double accel_noise_std_dev = imu.accelerometer_noise_density_g_per_sqrt_hz * g_to_ms2 / sqrt(dt);
            std::normal_distribution<> accel_noise_dist(0, accel_noise_std_dev);

            glm::dvec3 noise(accel_noise_dist(gen), accel_noise_dist(gen), accel_noise_dist(gen));
            auto bias = glm::dvec3(imu.accelerometer_bias_milli_g / 1000.0 * g_to_ms2);

            glm::dvec3 imu_measured_acceleration = ground_truth_acceleration + bias + noise;

            // --- 2. Kalman Filter: PREDICT Step ---
            predict(dt, imu_measured_acceleration);

            // --- 3. Kalman Filter: UPDATE Step (if GPS is available) ---
            if (registry.has<GPSComponent>(entity)) {
                auto& gps = registry.get<GPSComponent>(entity);
                gps.time_since_last_update_s += dt;

                if (gps.time_since_last_update_s >= (1.0 / gps.update_rate_hz)) {
                    gps.time_since_last_update_s = 0.0;
                    std::normal_distribution<> gps_noise_dist(0, gps.position_error_m);
                    glm::dvec3 gps_noise(gps_noise_dist(gen), gps_noise_dist(gen), gps_noise_dist(gen));
                    glm::dvec3 gps_measured_position = transform.position + gps_noise;
                    update(gps_measured_position, gps.position_error_m);
                }
            }

            // --- 4. Update the NavigationStateComponent ---
            navigation_state.estimated_position = {_state_estimate[0], _state_estimate[1], _state_estimate[2]};
            navigation_state.estimated_velocity = {_state_estimate[3], _state_estimate[4], _state_estimate[5]};
            // The estimated acceleration is the last (noisy) measurement from the IMU
            navigation_state.estimated_acceleration = imu_measured_acceleration;
        }
    }

    void NavigationSystem::predict(double dt, const glm::dvec3& imu_acceleration) {
        // State transition matrix F
        KalmanCovarianceMatrix F{};
        for(int i=0; i<6; ++i) F[i].fill(0.0);
        F[0][0] = F[1][1] = F[2][2] = F[3][3] = F[4][4] = F[5][5] = 1.0;
        F[0][3] = F[1][4] = F[2][5] = dt;

        // Control input vector u
        KalmanStateVector u_vec{};
        u_vec[0] = 0.5 * dt * dt * imu_acceleration.x;
        u_vec[1] = 0.5 * dt * dt * imu_acceleration.y;
        u_vec[2] = 0.5 * dt * dt * imu_acceleration.z;
        u_vec[3] = dt * imu_acceleration.x;
        u_vec[4] = dt * imu_acceleration.y;
        u_vec[5] = dt * imu_acceleration.z;

        // Predict the state estimate: x' = F*x + u
        _state_estimate = KalmanMath::multiply(F, _state_estimate);
        for(int i=0; i<6; ++i) _state_estimate[i] += u_vec[i];

        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        double noise_variance = 0.1; // tunable

        KalmanCovarianceMatrix Q{};
        Q[0][0] = Q[1][1] = Q[2][2] = dt4 / 4.0 * noise_variance;
        Q[0][3] = Q[1][4] = Q[2][5] = dt3 / 2.0 * noise_variance;
        Q[3][0] = Q[4][1] = Q[5][2] = dt3 / 2.0 * noise_variance;
        Q[3][3] = Q[4][4] = Q[5][5] = dt2 * noise_variance;

        // Predict the error covariance: P' = F*P*F^T + Q
        _covariance = KalmanMath::multiply(KalmanMath::multiply(F, _covariance), KalmanMath::transpose(F));
        _covariance = KalmanMath::add(_covariance, Q);
    }

    void NavigationSystem::update(const glm::dvec3& gps_position, double gps_error) {
        std::array<std::array<double, 6>, 3> H{};
        H[0][0] = H[1][1] = H[2][2] = 1.0;
        glm::dmat3 R = glm::dmat3(1.0) * (gps_error * gps_error);
        glm::dvec3 H_x = { _state_estimate[0], _state_estimate[1], _state_estimate[2] };
        glm::dvec3 innovation = gps_position - H_x;
        KalmanCovarianceMatrix P_ht = KalmanMath::transpose(_covariance);
        glm::dmat3 S{};
        for(int i=0; i<3; ++i) { for(int j=0; j<3; ++j) { double val = 0; for(int k=0; k<6; ++k) { val += H[i][k] * P_ht[k][j]; } S[i][j] = val; } }
        S[0][0] += R[0][0]; S[1][1] += R[1][1]; S[2][2] += R[2][2];
        glm::dmat3 S_inv = KalmanMath::invert(S);
        std::array<std::array<double, 3>, 6> K{};
        for(int i=0; i<6; ++i) { for(int j=0; j<3; ++j) { double val = 0; for(int k=0; k<6; ++k) { val += P_ht[i][k] * S_inv[k][j]; } K[i][j] = val; } }
        for(int i=0; i<6; ++i) { double correction = 0; for(int j=0; j<3; ++j) { correction += K[i][j] * innovation[j]; } _state_estimate[i] += correction; }
        KalmanCovarianceMatrix I_KH{};
        for(int i=0; i<6; ++i) { for(int j=0; j<6; ++j) { double kh_val = 0; for(int k=0; k<3; ++k) { kh_val += K[i][k] * H[k][j]; } I_KH[i][j] = (i==j ? 1.0 : 0.0) - kh_val; } }
        _covariance = KalmanMath::multiply(I_KH, _covariance);
    }

} // namespace StrikeEngine
