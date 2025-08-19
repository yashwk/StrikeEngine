#include "strikeengine/systems/guidance/GuidanceSystem.hpp"
#include "strikeengine/ecs/Registry.hpp"

// Required Components
#include "strikeengine/components/guidance/GuidanceComponent.hpp"
#include "strikeengine/components/guidance/AutopilotCommandComponent.hpp"
#include "strikeengine/components/physics/NavigationStateComponent.hpp"
#include "strikeengine/components/guidance/SeekerComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/physics/VelocityComponent.hpp"

#include <glm/gtx/norm.hpp>

namespace StrikeEngine {

    // Define standard gravity for converting acceleration from m/s^2 to G's.
    constexpr double STANDARD_GRAVITY = 9.80665;

    void GuidanceSystem::update(Registry& registry, double dt) {
        // The view now requires the full set of components for a realistic GNC loop.
        auto view = registry.view<GuidanceComponent, SeekerComponent, NavigationStateComponent, AutopilotCommandComponent>();

        for (auto [entity, guidance, seeker, navigation_state, autopilot_command] : view) {

            // --- 1. Check for Seeker Lock ---
            // The core logic is now gated by the seeker's ability to track the target.
            if (!seeker.has_lock) {
                autopilot_command.commanded_acceleration_g = glm::dvec3(0.0); // No lock, no command.
                continue;
            }

            Entity targetEntity = seeker.locked_target;

            // --- 2. Validate Target State ---
            if (!registry.has<TransformComponent>(targetEntity) || !registry.has<VelocityComponent>(targetEntity)) {
                autopilot_command.commanded_acceleration_g = glm::dvec3(0.0); // Target is invalid.
                continue;
            }

            // --- 3. Gather Data for PN Calculation ---
            // Get PERFECT "ground truth" data for the target (as if from a perfect sensor).
            const auto& target_transform = registry.get<TransformComponent>(targetEntity);
            const auto& target_velocity = registry.get<VelocityComponent>(targetEntity);

            // Use the missile's own IMPERFECT, ESTIMATED state for its side of the calculation.
            const glm::dvec3& missile_position = navigation_state.estimated_position;
            const glm::dvec3& missile_velocity = navigation_state.estimated_velocity;

            // --- 4. Execute Proportional Navigation Law ---
            const glm::dvec3 relative_position = target_transform.position - missile_position;
            const glm::dvec3 relative_velocity = target_velocity.linear - missile_velocity;

            const glm::dvec3 los_direction = glm::normalize(relative_position);
            const double closing_velocity = -glm::dot(relative_velocity, los_direction);

            // If closing velocity is negative, the missile is moving away from the target,
            // so guidance commands would be ineffective.
            if (closing_velocity < 0.0) {
                autopilot_command.commanded_acceleration_g = glm::dvec3(0.0);
                continue;
            }

            // Calculate the line-of-sight (LOS) rotation rate vector (omega).
            // Using length2 (squared magnitude) is a common optimization to avoid a square root.
            const glm::dvec3 los_rate_vector = glm::cross(relative_position, relative_velocity) / glm::length2(relative_position);

            // Calculate the final commanded acceleration vector in m/s^2.
            // The formula is a_c = N * V_c * (omega x LOS_hat).
            const glm::dvec3 commanded_acceleration_ms2 = guidance.navigation_constant * closing_velocity * glm::cross(los_rate_vector, los_direction);

            // --- 5. Output Command in G's ---
            // Convert the command to G's for the autopilot system.
            autopilot_command.commanded_acceleration_g = commanded_acceleration_ms2 / STANDARD_GRAVITY;
        }
    }

} // namespace StrikeEngine
