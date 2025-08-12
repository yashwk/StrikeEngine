#include "strikeengine/systems/guidance/ControlSystem.hpp"

#include "strikeengine/components/guidance/AutopilotCommandComponent.hpp"
#include "strikeengine/components/guidance/AutopilotStateComponent.hpp"
#include "strikeengine/components/physics/ControlSurfaceComponent.hpp"
#include "strikeengine/components/physics/NavigationStateComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"

#include <glm/glm.hpp>
#include <algorithm>

#include "strikeengine/ecs/Registry.hpp"

namespace StrikeEngine {

    void ControlSystem::update(Registry& registry, double dt) {
        auto view = registry.view<AutopilotCommandComponent, AutopilotStateComponent, ControlSurfaceComponent, NavigationStateComponent, TransformComponent>();

        for (auto [entity, command, state, fins, navigation, transform] : view) {

            // --- 1. Convert Commanded Acceleration to Body Frame ---
            // The guidance command is in the world frame, but the autopilot works in the
            // missile's local pitch/yaw frame. We must rotate the command vector.
            glm::dvec3 commanded_accel_world = command.commanded_acceleration_g * 9.80665; // to m/s^2
            glm::dvec3 commanded_accel_body = glm::inverse(transform.orientation) * commanded_accel_world;

            // --- 2. Get Current State from Navigation ---
            // We use the missile's own estimated acceleration as feedback for the PID loop.
            glm::dvec3 current_accel_world = navigation.estimated_acceleration;
            glm::dvec3 current_accel_body = glm::inverse(transform.orientation) * current_accel_world;

            // --- 3. PID Controller Logic (for Pitch and Yaw axes) ---

            // PITCH AXIS (controls vertical acceleration, body Y-axis)
            double error_pitch = commanded_accel_body.y - current_accel_body.y;
            state.integral_error_pitch += error_pitch * dt;
            double derivative_pitch = (error_pitch - state.previous_error_pitch) / dt;
            double pid_output_pitch = (state.kp * error_pitch) + (state.ki * state.integral_error_pitch) + (state.kd * derivative_pitch);
            state.previous_error_pitch = error_pitch;

            // YAW AXIS (controls horizontal acceleration, body Z-axis)
            double error_yaw = commanded_accel_body.z - current_accel_body.z;
            state.integral_error_yaw += error_yaw * dt;
            double derivative_yaw = (error_yaw - state.previous_error_yaw) / dt;
            double pid_output_yaw = (state.kp * error_yaw) + (state.ki * state.integral_error_yaw) + (state.kd * derivative_yaw);
            state.previous_error_yaw = error_yaw;

            // The PID output is the *desired* fin deflection angle.
            double desired_deflection_pitch = pid_output_pitch;
            double desired_deflection_yaw = pid_output_yaw;

            // --- 4. Apply Actuator Physical Limits ---

            // A. Clamp to Maximum Deflection Angle
            desired_deflection_pitch = std::clamp(desired_deflection_pitch, -fins.max_deflection_rad, fins.max_deflection_rad);
            desired_deflection_yaw = std::clamp(desired_deflection_yaw, -fins.max_deflection_rad, fins.max_deflection_rad);

            // B. Clamp to Maximum Rate of Change
            double max_change = fins.max_rate_rad_per_sec * dt;

            double current_pitch = fins.current_deflection_rad_pitch;
            fins.current_deflection_rad_pitch = std::clamp(desired_deflection_pitch, current_pitch - max_change, current_pitch + max_change);

            double current_yaw = fins.current_deflection_rad_yaw;
            fins.current_deflection_rad_yaw = std::clamp(desired_deflection_yaw, current_yaw - max_change, current_yaw + max_change);

            // The AerodynamicsSystem will now read these final, limited deflection values
            // to calculate the forces and torques on the airframe.
        }
    }

} // namespace StrikeEngine
