
#include "strikeengine/systems/guidance/EndgameSystem.hpp"

// --- Include all relevant components ---
#include "strikeengine/components/guidance/FuzeComponent.hpp"
#include "strikeengine/components/guidance/WarheadComponent.hpp"
#include "strikeengine/components/guidance/SeekerComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/metadata/TargetComponent.hpp"

namespace StrikeEngine {
    void EndgameSystem::update(Registry& registry, double dt)
    {
        // Get a view of all missiles with endgame components that have not yet detonated.
        auto missile_view = registry.view<FuzeComponent, WarheadComponent, SeekerComponent, TransformComponent>();

        for (auto missile_entity : missile_view)
        {
            auto& fuze = missile_view.get<FuzeComponent>(missile_entity);
            auto& warhead = missile_view.get<WarheadComponent>(missile_entity);
            auto& seeker = missile_view.get<SeekerComponent>(missile_entity);
            auto& missile_transform = missile_view.get<TransformComponent>(missile_entity);
            // Skip if the warhead has already detonated or if there is no locked target.
            if (warhead.has_detonated || !seeker.has_lock)
            {
                continue;
            }

            Entity target_entity = seeker.locked_target;

            // Ensure the target is still valid and has a transform.
            if (!registry.isAlive(target_entity) || !registry.has<TransformComponent>(target_entity))
            {
                continue;
            }

            const auto& target_transform = registry.get<TransformComponent>(target_entity);

            // --- 1. Check Fuze Trigger Condition ---
            double distance_to_target = glm::length(missile_transform.position - target_transform.position);

            if (distance_to_target <= fuze.trigger_distance_m)
            {
                // --- 2. Trigger Detonation ---
                warhead.has_detonated = true;
                // In a full simulation, we might create a visual effect entity here.
                // For now, we just proceed to the lethality check.

                // --- 3. Perform Lethality Assessment ---
                if (distance_to_target <= warhead.lethal_radius_m)
                {
                    // Target is within the lethal radius. Mark it as destroyed.
                    registry.destroy(target_entity);
                }
            }
        }
    }
} // namespace StrikeEngine
