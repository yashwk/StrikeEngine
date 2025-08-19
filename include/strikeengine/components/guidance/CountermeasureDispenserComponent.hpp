#pragma once

#include "strikeengine/ecs/Component.hpp"

namespace StrikeEngine {

    /**
     * @brief Manages the inventory of deployable countermeasures.
     */
    struct CountermeasureDispenserComponent final : public Component {
        /**
         * @brief The number of available chaff canisters.
         * Each canister creates a single radar-decoying chaff cloud when deployed.
         */
        int chaff_canisters = 16;

        /**
         * @brief The number of available flare cartridges.
         * Each cartridge creates a single heat-decoying flare when deployed.
         */
        int flare_cartridges = 16;

        /**
         * @brief A flag used by the ElectronicWarfareSystem to command a chaff deployment.
         * The system will set this to true, and the dispenser logic will consume one
         * canister and set it back to false.
         */
        bool deploy_chaff_command = false;

        /**
         * @brief A flag used by the ElectronicWarfareSystem to command a flare deployment.
         */
        bool deploy_flare_command = false;
    };

} // namespace StrikeEngine
