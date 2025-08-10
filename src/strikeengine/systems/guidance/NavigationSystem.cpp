#include <chrono>

#include "strikeengine/systems/guidance/NavigationSystem.hpp"
#include "strikeengine/ecs/Registry.hpp"

#include "strikeengine/components/physics/IMUComponent.hpp"
#include "strikeengine/components/physics/NavigationStateComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/physics/VelocityComponent.hpp"
#include "strikeengine/components/physics/ForceAccumulatorComponent.hpp"
#include "strikeengine/components/physics/MassComponent.hpp"

#include <random>

namespace StrikeEngine {
    static std::default_random_engine generator;
    static std::normal_distribution<double> distribution(0.0, 1.0);

    NavigationSystem::NavigationSystem()
    {
        generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    }

    void NavigationSystem::update(Registry& registry, double dt)
    {
        auto view = registry.view<IMUComponent, NavigationStateComponent, TransformComponent, VelocityComponent, ForceAccumulatorComponent, MassComponent>();

        for (auto [entity, imu, nav_state, transform, velocity, accumulator, mass] : view)
        {
            if (!nav_state.is_initialized)
            {
                nav_state.estimated_position = transform.position;
                nav_state.estimated_velocity = velocity.linear;
                nav_state.estimated_orientation = transform.orientation;
                nav_state.is_initialized = true;
            }

            glm::dvec3 true_linear_accel = accumulator.totalForce * mass.inverseMass;
            glm::dvec3 true_angular_vel = velocity.angular; // In body space

            double accel_noise_std_dev = imu.accelerometer_noise_density_g_per_sqrt_hz / sqrt(dt) * 9.80665;
            glm::dvec3 accel_noise(distribution(generator), distribution(generator), distribution(generator));
            accel_noise *= accel_noise_std_dev;

            // Gyro noise
            double gyro_noise_std_dev_rad_s = glm::radians(imu.gyro_noise_density_deg_per_sqrt_hr / 60.0) / sqrt(dt);
            glm::dvec3 gyro_noise(distribution(generator), distribution(generator), distribution(generator));
            gyro_noise *= gyro_noise_std_dev_rad_s;

            glm::dvec3 measured_linear_accel = true_linear_accel + accel_noise;
            glm::dvec3 measured_angular_vel = true_angular_vel + gyro_noise;

            // --- 4. Integrate Noisy Data to Update Estimated State ---
            // (Using simple Euler integration for the navigation estimate)

            // First, transform measured acceleration from body to world space
            glm::dvec3 measured_linear_accel_world = nav_state.estimated_orientation * measured_linear_accel;

            nav_state.estimated_velocity += measured_linear_accel_world * dt;
            nav_state.estimated_position += nav_state.estimated_velocity * dt;

            glm::dquat rotation_delta = glm::angleAxis(glm::length(measured_angular_vel) * dt,
                                                       glm::normalize(measured_angular_vel));
            nav_state.estimated_orientation = glm::normalize(rotation_delta * nav_state.estimated_orientation);
        }
    }
} // namespace StrikeEngine
