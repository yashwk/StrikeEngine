#pragma once

#include <string>
#include <strikeengine/kernel/config/AeroConfig.hpp>

namespace StrikeEngine::Kernel {

    /**
     * @brief Loads and parses an aero profile from a JSON file into an
     *        AeroConfig (same snake_case key schema as ConfigSerialization).
     */
    class AeroProfileDatabase {
    public:
        /**
         * @brief Loads and parses an aero profile from a JSON file.
         * @param file_path The path to the aero JSON profile.
         * @param error     Optional; set to the failure reason on false.
         * @return True if loading was successful, false otherwise.
         */
        bool loadProfile(const std::string& file_path, std::string* error = nullptr);

        /**
         * @brief Gets the parsed aero configuration (defaults when empty).
         */
        [[nodiscard]] const AeroConfig& aero() const;

    private:
        AeroConfig _aero;
    };

} // namespace StrikeEngine::Kernel
