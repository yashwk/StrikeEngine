#pragma once

#include "strikeengine/ecs/Component.hpp"
#include <string>

namespace StrikeEngine {

    /**
     * @brief Links an entity to its high-fidelity, aspect-dependent IR signature database.
     */
    struct InfraredSignatureComponent final : public Component {
        /**
         * @brief The file path to the JSON or binary file containing the IR data.
         * Example: "data/ir/mig29_signature.json"
         */
        std::string profile_path;
    };

} // namespace StrikeEngine
