#pragma once

#include <string>
#include <strikeengine/kernel/config/SensorConfig.hpp>

namespace StrikeEngine::Kernel {

    /**
     * @brief Loads and parses a sensor profile from a JSON file into a
     *        SensorConfig (same snake_case key schema as ConfigSerialization).
     */
    class SensorProfileDatabase {
    public:
        /**
         * @brief Loads and parses a sensor profile from a JSON file.
         * @param file_path The path to the sensor JSON profile.
         * @param error     Optional; set to the failure reason on false.
         * @return True if loading was successful, false otherwise.
         */
        bool loadProfile(const std::string& file_path, std::string* error = nullptr);

        /**
         * @brief Gets the parsed sensor configuration (defaults when empty).
         */
        [[nodiscard]] const SensorConfig& sensor() const;

    private:
        SensorConfig _sensor;
    };

} // namespace StrikeEngine::Kernel
