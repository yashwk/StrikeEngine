#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <string>

namespace StrikeEngine {

    /**
     * @brief Links an entity to its high-fidelity, aspect-dependent RCS database.
     */
    struct RCSProfileComponent final : public Component {
        /**
         * @brief The file path to the JSON or binary file containing the RCS data.
         * Example: "data/rcs/f22_raptor.json"
         */
        std::string profile_path;
    };

} // namespace StrikeEngine
