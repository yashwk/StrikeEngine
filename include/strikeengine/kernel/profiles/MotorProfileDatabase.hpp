#pragma once

#include <string>
#include <strikeengine/kernel/config/PropulsionConfig.hpp>

namespace StrikeEngine::Kernel {

    /**
     * @brief Loads and parses a motor (propulsion) profile from a JSON file
     *        into a PropulsionConfig (same snake_case key schema as
     *        ConfigSerialization).
     */
    class MotorProfileDatabase {
    public:
        /**
         * @brief Loads and parses a motor profile from a JSON file.
         * @param file_path The path to the motor JSON profile.
         * @param error     Optional; set to the failure reason on false.
         * @return True if loading was successful, false otherwise.
         */
        bool loadProfile(const std::string& file_path, std::string* error = nullptr);

        /**
         * @brief Gets the parsed propulsion configuration (defaults when empty).
         */
        [[nodiscard]] const PropulsionConfig& propulsion() const;

    private:
        PropulsionConfig _propulsion;
    };

} // namespace StrikeEngine::Kernel
