#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <vector>

namespace StrikeEngine {

    /**
     * @brief A data structure to hold a 2D lookup table for a PID gain.
     */
    struct GainSchedule {
        // The axes for the lookup table
        std::vector<double> mach_breakpoints;
        std::vector<double> dynamic_pressure_breakpoints_pa;

        // The 2D table of gain values, where table[mach_index][pressure_index]
        std::vector<std::vector<double>> gain_table;
    };


    /**
     * @brief Stores the internal state and gains for the autopilot's PID controllers.
     */
    struct AutopilotStateComponent final : public Component {
        // --- PID Controller Gain Schedules (Tuning Parameters) ---
        GainSchedule kp_schedule;
        GainSchedule ki_schedule;
        GainSchedule kd_schedule;

        // --- PID Controller State Variables (Updated at runtime) ---
        double integral_error_pitch = 0.0;
        double previous_error_pitch = 0.0;
        double integral_error_yaw = 0.0;
        double previous_error_yaw = 0.0;
    };

} // namespace StrikeEngine
