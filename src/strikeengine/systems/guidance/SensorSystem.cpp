#include "strikeengine/systems/guidance/SensorSystem.hpp"
#include "strikeengine/ecs/Registry.hpp"

// Required components
#include "strikeengine/components/guidance/SeekerComponent.hpp"
#include "strikeengine/components/metadata/TargetComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/guidance/GuidanceComponent.hpp"

#include <glm/gtx/norm.hpp>
#include <cmath>

namespace StrikeEngine {

    void SensorSystem::update(Registry& registry, double dt) {
        auto seeker_view = registry.view<SeekerComponent, TransformComponent, GuidanceComponent>();
        auto target_view = registry.view<TargetComponent, TransformComponent>();

        for (auto [seeker_entity, seeker, seeker_transform, guidance] : seeker_view) {
            // For now, we only care about the designated target from the guidance component
            Entity designated_target_id = guidance.targetEntity;
            if (designated_target_id == NULL_ENTITY || !registry.has<TransformComponent>(designated_target_id)) {
                seeker.has_lock = false;
                seeker.locked_target = NULL_ENTITY;
                continue;
            }

            const auto& target_transform = registry.get<TransformComponent>(designated_target_id);

            // --- 1. Range Check ---
            glm::dvec3 relative_position = target_transform.position - seeker_transform.position;
            double range_to_target = glm::length(relative_position);

            if (range_to_target > seeker.max_range_m) {
                seeker.has_lock = false; // Target is out of range
                continue;
            }

            // --- 2. Field of View (FOV) Check ---
            glm::dvec3 line_of_sight_dir = glm::normalize(relative_position);
            glm::dvec3 seeker_forward_dir = glm::normalize(seeker_transform.orientation * glm::dvec3(0, 0, 1));

            double angle_off_boresight_rad = acos(glm::clamp(glm::dot(seeker_forward_dir, line_of_sight_dir), -1.0, 1.0));
            double fov_rad = glm::radians(seeker.field_of_view_deg);

            if (angle_off_boresight_rad > fov_rad / 2.0) {
                seeker.has_lock = false; // Target is outside the seeker's cone
                continue;
            }

            // --- 3. Lock Acquired ---
            // If both checks pass, the seeker has a lock.
            seeker.has_lock = true;
            seeker.locked_target = designated_target_id;
        }
    }

} // namespace StrikeEngine
