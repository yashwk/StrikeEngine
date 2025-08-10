#include "strikeengine/systems/guidance/ControlSystem.hpp"
#include "strikeengine/ecs/Registry.hpp"

// Components needed for the autopilot
#include "strikeengine/components/guidance/AutopilotCommandComponent.hpp"
#include "strikeengine/components/guidance/AutopilotStateComponent.hpp"
#include "strikeengine/components/physics/ControlSurfaceComponent.hpp"
#include "strikeengine/components/physics/ForceAccumulatorComponent.hpp" // To get current acceleration
#include "strikeengine/components/physics/MassComponent.hpp"

namespace StrikeEngine {

    // PID Controller Gains (these would be loaded from a profile in a real system)
    constexpr double Kp = 0.8;  // Proportional gain
    constexpr double Ki = 0.2;  // Integral gain
    constexpr double Kd = 0.1;  // Derivative gain

    // Actuator Limits (also would be loaded from a profile)
    constexpr double MAX_DEFLECTION_RAD = 0.35; // ~20 degrees
    constexpr double MAX_RATE_RAD_PER_SEC = 5.0;

    void ControlSystem::update(Registry& registry, double dt) {
        if (dt <= 0.0) return;


        auto view = registry.view<AutopilotCommandComponent, AutopilotStateComponent, ControlSurfaceComponent, ForceAccumulatorComponent, MassComponent>();

        for (auto [entity, cmd, state, fins, accumulator, mass] : view) {

            // --- 1. Calculate Current State & Error ---

            // Get the acceleration achieved in the *previous* frame from the force accumulator.
            // This is our feedback signal. A = F/m
            glm::dvec3 current_acceleration_g = (accumulator.totalForce * mass.inverseMass) / 9.80665;

            // The error is the difference between what guidance commanded and what we achieved.
            glm::dvec3 error = cmd.commanded_acceleration_g - current_acceleration_g;

            // --- 2. PID Controller Logic ---

            // Proportional term
            glm::dvec3 p_term = Kp * error;

            // Integral term (update stored integral error)
            state.integral_error += error * dt;
            glm::dvec3 i_term = Ki * state.integral_error;

            // Derivative term (for now, we can simplify and omit this)
            // A full implementation would need to store the previous error.
            auto d_term = glm::dvec3(0.0);

            // Total PID output
            glm::dvec3 output = p_term + i_term + d_term;

            // --- 3. Map PID Output to Fin Deflections ---

            // This is a highly simplified mapping. A real autopilot would have complex
            // logic to map desired pitch/yaw acceleration to specific fin movements.
            // Here, we'll map Y-axis acceleration to a pitch fin and X-axis to a yaw fin.
            glm::dvec4 desired_deflections_rad;
            desired_deflections_rad.x = -output.y; // Pitch command
            desired_deflections_rad.y = output.x;  // Yaw command
            desired_deflections_rad.z = 0.0;       // Roll command (not implemented)
            desired_deflections_rad.w = 0.0;

            // Clamp the desired deflections to the physical limits of the fins.
            desired_deflections_rad.x = std::clamp(desired_deflections_rad.x, -MAX_DEFLECTION_RAD, MAX_DEFLECTION_RAD);
            desired_deflections_rad.y = std::clamp(desired_deflections_rad.y, -MAX_DEFLECTION_RAD, MAX_DEFLECTION_RAD);

            // --- 4. Write to the ControlSurfaceComponent ---
            // The ControlSystem writes the *target* deflections. It does not update the
            // current deflections directly, as that would be instantaneous. Another system
            // or this one would then have to model the actuator moving at a max rate.
            // For now, we set the commanded deflections.
            fins.commandedDeflections_rad = desired_deflections_rad;

            // A more advanced implementation would also update the `currentDeflections_rad`
            // by moving it towards the `commandedDeflections_rad` at the `MAX_RATE_RAD_PER_SEC`.
            // For example:
            glm::dvec4 delta = fins.commandedDeflections_rad - fins.currentDeflections_rad;
            double max_change = MAX_RATE_RAD_PER_SEC * dt;
            if (glm::length(delta) > max_change) {
                delta = glm::normalize(delta) * max_change;
            }
            fins.currentDeflections_rad += delta;
        }
    }

} // namespace StrikeEngine
