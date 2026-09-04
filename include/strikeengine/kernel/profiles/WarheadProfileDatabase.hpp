#pragma once

#include <string>
#include <strikeengine/kernel/config/WarheadConfig.hpp>

namespace StrikeEngine::Kernel {

    /**
     * @brief Loads and parses a warhead profile from a JSON file into a
     *        WarheadConfig.
     */
    class WarheadProfileDatabase {
    public:
        /**
         * @brief Loads and parses a warhead profile from a JSON file.
         * @param file_path The path to the warhead JSON profile.
         * @return True if loading was successful, false otherwise.
         */
        bool loadProfile(const std::string& file_path);

        /**
         * @brief Gets the parsed warhead configuration (defaults when empty).
         */
        [[nodiscard]] const WarheadConfig& warhead() const;

    private:
        WarheadConfig _warhead;
    };

} // namespace StrikeEngine::Kernel
