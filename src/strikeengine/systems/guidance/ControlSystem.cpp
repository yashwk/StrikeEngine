#include "strikeengine/systems/guidance/ControlSystem.hpp"
#include "strikeengine/atmosphere/AtmosphereManager.hpp"
#include "strikeengine/ecs/Registry.hpp"

#include "strikeengine/components/guidance/AutopilotCommandComponent.hpp"
#include "strikeengine/components/guidance/AutopilotStateComponent.hpp"
#include "strikeengine/components/physics/ControlSurfaceComponent.hpp"
#include "strikeengine/components/physics/NavigationStateComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/physics/VelocityComponent.hpp"

#include <glm/glm.hpp>
#include <algorithm>
#include <vector>

namespace StrikeEngine {

    extern AtmosphereManager g_atmosphere_manager;

    double interpolateGain(const GainSchedule& schedule, double mach, double dynamic_pressure) {
        auto it_mach = std::ranges::lower_bound(schedule.mach_breakpoints, mach);
        int j = std::distance(schedule.mach_breakpoints.begin(), it_mach);
        if (j >= schedule.mach_breakpoints.size()) j = schedule.mach_breakpoints.size() - 1;
        if (j == 0) j = 1;

        auto it_pressure = std::ranges::lower_bound(schedule.dynamic_pressure_breakpoints_pa, dynamic_pressure);
        int i = std::distance(schedule.dynamic_pressure_breakpoints_pa.begin(), it_pressure);
        if (i >= schedule.dynamic_pressure_breakpoints_pa.size()) i = schedule.dynamic_pressure_breakpoints_pa.size() - 1;
        if (i == 0) i = 1;

        // Get the four corner points for interpolation
        double m1 = schedule.mach_breakpoints[j - 1];
        double m2 = schedule.mach_breakpoints[j];
        double q1 = schedule.dynamic_pressure_breakpoints_pa[i - 1];
        double q2 = schedule.dynamic_pressure_breakpoints_pa[i];

        double g11 = schedule.gain_table[i - 1][j - 1];
        double g12 = schedule.gain_table[i - 1][j];
        double g21 = schedule.gain_table[i][j - 1];
        double g22 = schedule.gain_table[i][j];

        // Perform bilinear interpolation
        double term1 = g11 * (m2 - mach) * (q2 - dynamic_pressure);
        double term2 = g21 * (mach - m1) * (q2 - dynamic_pressure);
        double term3 = g12 * (m2 - mach) * (dynamic_pressure - q1);
        double term4 = g22 * (mach - m1) * (dynamic_pressure - q1);

        return (term1 + term2 + term3 + term4) / ((m2 - m1) * (q2 - q1));
    }


    void ControlSystem::update(Registry& registry, double dt) {
        auto view = registry.view<AutopilotCommandComponent, AutopilotStateComponent, ControlSurfaceComponent, NavigationStateComponent, TransformComponent, VelocityComponent>();

        for (auto entity : view) {
            auto& command = view.get<AutopilotCommandComponent>(entity);
            auto& state = view.get<AutopilotStateComponent>(entity);
            auto& fins = view.get<ControlSurfaceComponent>(entity);
            auto& navigation = view.get<NavigationStateComponent>(entity);
            auto& transform = view.get<TransformComponent>(entity);
            auto& velocity = view.get<VelocityComponent>(entity);

            // --- 1. Calculate Current Flight Conditions ---
            const double altitude = glm::length(transform.position);
            const auto& atmosphere_properties = g_atmosphere_manager.getProperties(altitude);
            const double speed = glm::length(velocity.getLinear());
            const double dynamic_pressure = 0.5 * atmosphere_properties.density * speed * speed;
            const double mach_number = speed / atmosphere_properties.speedOfSound;

            // --- 2. Get Dynamic PID Gains via Gain Scheduling ---
            double kp = interpolateGain(state.kp_schedule, mach_number, dynamic_pressure);
            double ki = interpolateGain(state.ki_schedule, mach_number, dynamic_pressure);
            double kd = interpolateGain(state.kd_schedule, mach_number, dynamic_pressure);

            // --- 3. Convert Commanded Acceleration to Body Frame ---
            glm::dvec3 commanded_acceleration_world = command.commanded_acceleration_g * 9.80665;
            glm::dvec3 commanded_acceleration_body = glm::inverse(transform.orientation) * commanded_acceleration_world;

            // --- 4. Get Current State from Navigation ---
            glm::dvec3 current_acceleration_world = navigation.estimated_acceleration;
            glm::dvec3 current_acceleration_body = glm::inverse(transform.orientation) * current_acceleration_world;

            // --- 5. PID Controller Logic (using dynamic gains) ---
            double error_pitch = commanded_acceleration_body.y - current_acceleration_body.y;
            state.integral_error_pitch += error_pitch * dt;
            double pitch_derivative = (error_pitch - state.previous_error_pitch) / dt;
            double pid_output_pitch = (kp * error_pitch) + (ki * state.integral_error_pitch) + (kd * pitch_derivative);
            state.previous_error_pitch = error_pitch;

            double error_yaw = commanded_acceleration_body.z - current_acceleration_body.z;
            state.integral_error_yaw += error_yaw * dt;
            double yaw_derivative = (error_yaw - state.previous_error_yaw) / dt;
            double pid_output_yaw = (kp * error_yaw) + (ki * state.integral_error_yaw) + (kd * yaw_derivative);
            state.previous_error_yaw = error_yaw;

            double desired_deflection_pitch = pid_output_pitch;
            double desired_deflection_yaw = pid_output_yaw;

            // --- 6. Apply Actuator Physical Limits ---
            desired_deflection_pitch = std::clamp(desired_deflection_pitch, -fins.max_deflection_rad, fins.max_deflection_rad);
            desired_deflection_yaw = std::clamp(desired_deflection_yaw, -fins.max_deflection_rad, fins.max_deflection_rad);

            double max_change = fins.max_rate_rad_per_sec * dt;

            double current_pitch = fins.current_deflection_rad_pitch;
            fins.current_deflection_rad_pitch = std::clamp(desired_deflection_pitch, current_pitch - max_change, current_pitch + max_change);

            double current_yaw = fins.current_deflection_rad_yaw;
            fins.current_deflection_rad_yaw = std::clamp(desired_deflection_yaw, current_yaw - max_change, current_yaw + max_change);
        }
    }

} // namespace StrikeEngine
