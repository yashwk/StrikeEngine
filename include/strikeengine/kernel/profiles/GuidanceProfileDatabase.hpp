#pragma once

#include <string>
#include <strikeengine/kernel/config/GuidanceAutopilotConfig.hpp>

namespace StrikeEngine::Kernel {

    /**
     * @brief Loads and parses a guidance & autopilot profile from a JSON file
     *        into a GuidanceAutopilotConfig.
     */
    class GuidanceProfileDatabase {
    public:
        /**
         * @brief Loads and parses a guidance profile from a JSON file.
         * @param file_path The path to the guidance JSON profile.
         * @return True if loading was successful, false otherwise.
         */
        bool loadProfile(const std::string& file_path);

        /**
         * @brief Gets the parsed guidance configuration (defaults when empty).
         */
        [[nodiscard]] const GuidanceAutopilotConfig& guidanceAutopilot() const;

    private:
        GuidanceAutopilotConfig _guidanceAutopilot;
    };

} // namespace StrikeEngine::Kernel
