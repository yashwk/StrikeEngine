
#include "strikeengine/systems/guidance/EndgameSystem.hpp"

// --- Include all relevant components ---
#include "strikeengine/components/guidance/FuzeComponent.hpp"
#include "strikeengine/components/guidance/WarheadComponent.hpp"
#include "strikeengine/components/guidance/SeekerComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/metadata/TargetComponent.hpp"

namespace StrikeEngine {

    void EndgameSystem::update(Registry& registry, double dt) {
        // Get a view of all missiles with endgame components that have not yet detonated.
        auto missile_view = registry.view<FuzeComponent, WarheadComponent, SeekerComponent, TransformComponent>();

        for (auto [missile_entity, fuze, warhead, seeker, missile_transform] : missile_view) {
            // Skip if the warhead has already detonated or if there is no locked target.
            if (warhead.has_detonated || !seeker.has_lock) {
                continue;
            }

            Entity target_entity = seeker.locked_target;

            // Ensure the target is still valid and has a transform.
            if (!registry.isValid(target_entity) || !registry.has<TransformComponent>(target_entity)) {
                continue;
            }

            const auto& target_transform = registry.get<TransformComponent>(target_entity);

            // --- 1. Check Fuze Trigger Condition ---
            double distance_to_target = glm::length(missile_transform.position - target_transform.position);

            if (distance_to_target <= fuze.trigger_distance_m) {
                // --- 2. Trigger Detonation ---
                warhead.has_detonated = true;
                // In a full simulation, we might create a visual effect entity here.
                // For now, we just proceed to the lethality check.

                // --- 3. Perform Lethality Assessment ---
                if (distance_to_target <= warhead.lethal_radius_m) {
                    // Target is within the lethal radius. Mark it as destroyed.
                    // A simple way to do this is to remove its "targetable" component.
                    if (registry.has<TargetComponent>(target_entity)) {
                        registry.remove<TargetComponent>(target_entity);
                    }
                     // Or remove the entity entirely
                    registry.destroyEntity(target_entity);
                }

                // The missile itself is also destroyed upon detonation.
                registry.destroyEntity(missile_entity);
            }
        }
    }

} // namespace StrikeEngine
