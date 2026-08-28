#pragma once

#include <string>
#include <strikeengine/kernel/config/SeekerConfig.hpp>

namespace StrikeEngine::Kernel {

    /**
     * @brief Loads and parses a seeker profile from a JSON file into a
     *        SeekerConfig (same snake_case key schema as ConfigSerialization).
     */
    class SeekerProfileDatabase {
    public:
        /**
         * @brief Loads and parses a seeker profile from a JSON file.
         * @param file_path The path to the seeker JSON profile.
         * @return True if loading was successful, false otherwise.
         */
        bool loadProfile(const std::string& file_path);

        /**
         * @brief Gets the parsed seeker configuration (defaults when empty).
         */
        [[nodiscard]] const SeekerConfig& seeker() const;

    private:
        SeekerConfig _seeker;
    };

} // namespace StrikeEngine::Kernel
