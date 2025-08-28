#include "strikeengine/systems/guidance/ElectronicWarfareSystem.hpp"

#include "strikeengine/components/guidance/JammerComponent.hpp"
#include "strikeengine/components/guidance/AntennaComponent.hpp"
#include "strikeengine/components/guidance/CountermeasureDispenserComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/metadata/RCSProfileComponent.hpp"
#include "strikeengine/components/metadata/InfraredSignatureComponent.hpp"

#include <numbers>

namespace StrikeEngine {

    void ElectronicWarfareSystem::update(Registry& registry, double dt) {
        // --- 1. Process Noise Jammers ---
        // For each active jammer, calculate its effect on every radar receiver.
        auto jammer_view = registry.view<JammerComponent, TransformComponent>();
        auto receiver_view = registry.view<AntennaComponent, TransformComponent>();

        for (auto receiver_entity : receiver_view) {
            auto& antenna = receiver_view.get<AntennaComponent>(receiver_entity);
            auto& receiver_transform = receiver_view.get<TransformComponent>(receiver_entity);
            double total_jamming_power_W = 0.0;

            for (auto jammer_entity : jammer_view) {
                auto& jammer = jammer_view.get<JammerComponent>(jammer_entity);
                auto& jammer_transform = jammer_view.get<TransformComponent>(jammer_entity);

                if (!jammer.active) continue;

                // Calculate range between jammer and receiver
                double range = glm::length(receiver_transform.position - jammer_transform.position);

                // Calculate power density at the receiver using the Friis transmission equation
                // Power Density = ERP / (4 * pi * R^2)
                double power_density = jammer.effective_radiated_power_W / (4.0 * std::numbers::pi * range * range);

                // The power received by the antenna is the power density times the antenna's effective aperture.
                // Effective Aperture = (Gain * Wavelength^2) / (4 * pi)
                double gain_linear = std::pow(10.0, antenna.antenna_gain_dB / 10.0);
                double effective_aperture = (gain_linear * antenna.wavelength_m * antenna.wavelength_m) / (4.0 * std::numbers::pi);

                total_jamming_power_W += power_density * effective_aperture;
            }

            // Add the calculated jamming power to the receiver's natural noise floor.
            // The RadarSystem will now use this higher noise floor, reducing its SNR.
            antenna.noise_floor_W += total_jamming_power_W;
        }


        // --- 2. Process Countermeasure Deployment ---
        auto dispenser_view = registry.view<CountermeasureDispenserComponent, TransformComponent>();
        for (auto entity : dispenser_view) {
            auto& dispenser = dispenser_view.get<CountermeasureDispenserComponent>(entity);
            auto& transform = dispenser_view.get<TransformComponent>(entity);

            // Deploy Chaff
            if (dispenser.deploy_chaff_command && dispenser.chaff_canisters > 0) {
                dispenser.chaff_canisters--;
                dispenser.deploy_chaff_command = false;

                // Create a new entity to represent the chaff cloud
                Entity chaff_cloud = registry.create();
                // Place it at the same position as the deploying aircraft
                registry.add<TransformComponent>(chaff_cloud, transform);
                // Give it a very large, non-aspect-dependent radar signature
                auto& rcs = registry.add<RCSProfileComponent>(chaff_cloud);
                rcs.profile_path = "data/rcs/chaff_cloud_generic.json";
            }

            // Deploy Flare
            if (dispenser.deploy_flare_command && dispenser.flare_cartridges > 0) {
                dispenser.flare_cartridges--;
                dispenser.deploy_flare_command = false;

                // Create a new entity to represent the flare
                Entity flare = registry.create();
                registry.add<TransformComponent>(flare, transform);
                auto& ir_sig = registry.add<InfraredSignatureComponent>(flare);
                ir_sig.profile_path = "data/ir/flare_generic.json";
            }
        }
    }

} // namespace StrikeEngine
