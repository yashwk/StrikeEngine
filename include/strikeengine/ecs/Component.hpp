#pragma once

namespace StrikeEngine {

    /**
     * @brief Base struct for all components in the ECS.
     * All components must be inherited from this struct to be managed by the Registry.
     */
    struct Component {
        virtual ~Component() = default;
    };

} // namespace StrikeEngine
