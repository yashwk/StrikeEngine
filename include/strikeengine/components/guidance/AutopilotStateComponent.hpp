// ===================================================================================
//  AutopilotStateComponent.hpp
//
//  Description:
//  This header defines the AutopilotStateComponent. This component holds the
//  internal state variables and tuning parameters (gains) for the PID
//  controllers used by the ControlSystem.
//
//  Architectural Note:
//  By storing the PID state in a component, the ControlSystem itself can remain
//  stateless. This makes the autopilot's behavior entirely data-driven, as
//  these parameters will be loaded from a vehicle's JSON profile.
//
//  Associated Plan: "Control System (Autopilot) Implementation Plan" (Step 5.2)
//
// ===================================================================================

#pragma once

#include "strikeengine/ecs/Component.hpp"

namespace StrikeEngine {

    /**
     * @brief Stores the internal state and gains for the autopilot's PID controllers.
     */
    struct AutopilotStateComponent final : public Component {
        // --- PID Controller Gains (Tuning Parameters) ---

        /** @brief The Proportional gain (Kp). Determines the reaction to the current error. */
        double kp = 0.8;

        /** @brief The Integral gain (Ki). Determines the reaction based on the sum of recent errors. */
        double ki = 0.2;

        /** @brief The Derivative gain (Kd). Determines the reaction based on the rate at which the error has been changing. */
        double kd = 0.1;


        // --- PID Controller State Variables ---

        /** @brief The accumulated integral error for the pitch-axis controller. */
        double integral_error_pitch = 0.0;

        /** @brief The error from the previous frame for the pitch-axis controller, used for the derivative term. */
        double previous_error_pitch = 0.0;

        /** @brief The accumulated integral error for the yaw-axis controller. */
        double integral_error_yaw = 0.0;

        /** @brief The error from the previous frame for the yaw-axis controller. */
        double previous_error_yaw = 0.0;
    };

} // namespace StrikeEngine
