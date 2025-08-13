#include "strikeengine/systems/guidance/ControlSystem.hpp"
#include "strikeengine/atmosphere/AtmosphereManager.hpp"

// --- Required Components for Autopilot Logic ---
#include "strikeengine/components/guidance/AutopilotCommandComponent.hpp"
#include "strikeengine/components/guidance/AutopilotStateComponent.hpp"
#include "strikeengine/components/physics/ControlSurfaceComponent.hpp"
#include "strikeengine/components/physics/NavigationStateComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/physics/VelocityComponent.hpp"

#include <glm/glm.hpp>
#include <algorithm> // For std::clamp
#include <vector>

#include "strikeengine/ecs/Registry.hpp"

namespace StrikeEngine {

    // A placeholder for a real atmosphere manager service
    extern AtmosphereManager g_atmosphere_manager;

    // Helper function to perform bilinear interpolation on a gain schedule table.
    double interpolateGain(const GainSchedule& schedule, double mach, double dyn_pressure) {
        // Find indices for Mach number
        auto it_mach = std::lower_bound(schedule.mach_breakpoints.begin(), schedule.mach_breakpoints.end(), mach);
        int j = std::distance(schedule.mach_breakpoints.begin(), it_mach);
        if (j >= schedule.mach_breakpoints.size()) j = schedule.mach_breakpoints.size() - 1;
        if (j == 0) j = 1;

        // Find indices for dynamic pressure
        auto it_pres = std::lower_bound(schedule.dynamic_pressure_breakpoints_pa.begin(), schedule.dynamic_pressure_breakpoints_pa.end(), dyn_pressure);
        int i = std::distance(schedule.dynamic_pressure_breakpoints_pa.begin(), it_pres);
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
        double term1 = g11 * (m2 - mach) * (q2 - dyn_pressure);
        double term2 = g21 * (mach - m1) * (q2 - dyn_pressure);
        double term3 = g12 * (m2 - mach) * (dyn_pressure - q1);
        double term4 = g22 * (mach - m1) * (dyn_pressure - q1);

        return (term1 + term2 + term3 + term4) / ((m2 - m1) * (q2 - q1));
    }


    void ControlSystem::update(Registry& registry, double dt) {
        auto view = registry.view<AutopilotCommandComponent, AutopilotStateComponent, ControlSurfaceComponent, NavigationStateComponent, TransformComponent, VelocityComponent>();

        for (auto [entity, cmd, state, fins, nav, transform, velocity] : view) {

            // --- 1. Calculate Current Flight Conditions ---
            const double altitude = glm::length(transform.position);
            const double air_density = g_atmosphere_manager.getProperties(altitude).density;
            const double speed = glm::length(velocity.linear);
            const double dynamic_pressure = 0.5 * air_density * speed * speed;
            const double mach_number = speed / g_atmosphere_manager.getProperties(altitude).speedOfSound;

            // --- 2. Get Dynamic PID Gains via Gain Scheduling ---
            double kp = interpolateGain(state.kp_schedule, mach_number, dynamic_pressure);
            double ki = interpolateGain(state.ki_schedule, mach_number, dynamic_pressure);
            double kd = interpolateGain(state.kd_schedule, mach_number, dynamic_pressure);

            // --- 3. Convert Commanded Acceleration to Body Frame ---
            glm::dvec3 commanded_accel_world = cmd.commanded_acceleration_g * 9.80665;
            glm::dvec3 commanded_accel_body = glm::inverse(transform.orientation) * commanded_accel_world;

            // --- 4. Get Current State from Navigation ---
            glm::dvec3 current_accel_world = nav.estimated_acceleration;
            glm::dvec3 current_accel_body = glm::inverse(transform.orientation) * current_accel_world;

            // --- 5. PID Controller Logic (using dynamic gains) ---
            double error_pitch = commanded_accel_body.y - current_accel_body.y;
            state.integral_error_pitch += error_pitch * dt;
            double derivative_pitch = (error_pitch - state.previous_error_pitch) / dt;
            double pid_output_pitch = (kp * error_pitch) + (ki * state.integral_error_pitch) + (kd * derivative_pitch);
            state.previous_error_pitch = error_pitch;

            double error_yaw = commanded_accel_body.z - current_accel_body.z;
            state.integral_error_yaw += error_yaw * dt;
            double derivative_yaw = (error_yaw - state.previous_error_yaw) / dt;
            double pid_output_yaw = (kp * error_yaw) + (ki * state.integral_error_yaw) + (kd * derivative_yaw);
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
