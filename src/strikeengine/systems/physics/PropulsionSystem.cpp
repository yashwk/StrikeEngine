#include "strikeengine/systems/physics/PropulsionSystem.hpp"
#include "strikeengine/ecs/Registry.hpp"
#include "strikeengine/components/physics/PropulsionComponent.hpp"
#include "strikeengine/components/physics/MassComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/physics/ForceAccumulatorComponent.hpp"

#include <glm/gtc/quaternion.hpp>
#include <iostream>

namespace StrikeEngine {

    constexpr double STANDARD_GRAVITY = 9.80665; // m/s^2

    void PropulsionSystem::update(Registry& registry, double dt) {
        auto view = registry.view<PropulsionComponent, MassComponent, TransformComponent, ForceAccumulatorComponent>();

        for (auto [entity, propulsion, mass, transform, accumulator] : view) {
            if (!propulsion.active) {
                continue;
            }

            // --- 1. Handle Stage Activation ---
            if (propulsion.currentStageIndex == -1) {
                propulsion.currentStageIndex = 0; // Activate the first stage
                std::cout << "Activating stage 0: " << propulsion.stages[0].name << std::endl;
            }

            if (propulsion.currentStageIndex >= propulsion.stages.size()) {
                propulsion.active = false; // All stages have been burned
                continue;
            }

            // --- 2. Process Active Stage ---
            auto& currentStage = propulsion.stages[propulsion.currentStageIndex];
            propulsion.timeInCurrentStage_seconds += dt;

            // Check for burnout
            if (propulsion.timeInCurrentStage_seconds > currentStage.burnTime_seconds) {
                // Stage is spent. Jettison its mass (casing).
                // A more complex model would distinguish between propellant and casing mass.
                // For now, we assume the entire stage mass is gone.
                mass.currentMass_kg -= currentStage.stage_mass_kg;
                mass.updateInverseMass();
                std::cout << "Stage " << currentStage.name << " burnout. Jettisoning mass." << std::endl;

                // Advance to the next stage
                propulsion.currentStageIndex++;
                propulsion.timeInCurrentStage_seconds = 0.0;
                continue; // Continue to the next frame to process the new stage
            }

            // --- 3. Apply Thrust and Consume Fuel ---
            glm::dvec3 thrust_direction_world = transform.orientation * glm::dvec3(0, 0, 1);
            glm::dvec3 thrust_force = glm::normalize(thrust_direction_world) * currentStage.thrust_newtons;
            accumulator.totalForce += thrust_force;

            if (currentStage.specificImpulse_seconds > 1e-6) {
                double mass_flow_rate = currentStage.thrust_newtons / (currentStage.specificImpulse_seconds * STANDARD_GRAVITY);
                double mass_consumed = mass_flow_rate * dt;
                mass.currentMass_kg -= mass_consumed;
                mass.updateInverseMass();
            }
        }
    }

} // namespace StrikeEngine
