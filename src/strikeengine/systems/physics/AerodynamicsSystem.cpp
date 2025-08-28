#include "strikeengine/systems/physics/AerodynamicsSystem.hpp"
#include "strikeengine/atmosphere/AtmosphereManager.hpp"
#include "strikeengine/flight/AerodynamicsDatabase.hpp"
#include "strikeengine/ecs/Registry.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/physics/VelocityComponent.hpp"
#include "strikeengine/components/physics/AerodynamicProfileComponent.hpp"
#include "strikeengine/components/physics/ForceAccumulatorComponent.hpp"

#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>

namespace StrikeEngine {
   AerodynamicsSystem::AerodynamicsSystem(const AtmosphereManager& atmosphereManager): _atmosphere_manager(
      atmosphereManager)
   {
   }

   AerodynamicsSystem::~AerodynamicsSystem() = default;

   void AerodynamicsSystem::update(Registry& registry, double dt)
   {
      if (!_atmosphere_manager.isLoaded()) { return; }

      auto view = registry.view<TransformComponent, VelocityComponent, AerodynamicProfileComponent,
                                ForceAccumulatorComponent>();
      for (auto entity : view)
      {
         auto& transform = view.get<TransformComponent>(entity);
         auto& velocity = view.get<VelocityComponent>(entity);
         auto& aero = view.get<AerodynamicProfileComponent>(entity);
         auto& accumulator = view.get<ForceAccumulatorComponent>(entity);

         // --- 1. Load Aerodynamic Database if is not already cached ---
         if (!_aeroDatabases.contains(aero.profileID))
         {
            auto db = std::make_unique<AerodynamicsDatabase>();
            std::string profile_path = "data/aero/" + aero.profileID + ".json";
            if (db->loadProfile(profile_path)) { _aeroDatabases[aero.profileID] = std::move(db); }
            else
            {
               continue;
            }
         }
         const auto& aero_db = _aeroDatabases.at(aero.profileID);

         // --- 2. Calculate Current Flight Conditions ---
         if (glm::length2(velocity.getLinear()) < 1e-6)
         {
            aero.current_angle_of_attack_rad = 0.0;
            aero.current_mach_number = 0.0;
            continue;
         }

         // Note: This assumes a spherical Earth model where altitude is distance from the center.
         // For ground effect, we need Altitude Above Ground Level (AGL). We will approximate
         // this with the Y-coordinate, assuming a flat plane at y=0.
         const double altitude_from_center = glm::length(transform.position);
         const AtmosphereProperties atmosphere = _atmosphere_manager.getProperties(altitude_from_center);
         const double speed = glm::length(velocity.getLinear());
         aero.current_mach_number = speed / atmosphere.speedOfSound;

         const glm::dvec3 velocity_direction = glm::normalize(velocity.getLinear());
         const glm::dvec3 body_forward_direction = glm::normalize(transform.orientation * glm::dvec3(0, 0, 1));
         aero.current_angle_of_attack_rad = acos(
            glm::clamp(glm::dot(velocity_direction, body_forward_direction), -1.0, 1.0));

         // --- 3. Look Up Base Aerodynamic Coefficients ---
         auto [base_Cl, base_Cd] = aero_db->getCoefficients(aero.current_mach_number, aero.current_angle_of_attack_rad);

         // --- 4. Ground Effect Calculation ---
         double lift_multiplier = 1.0;
         double drag_multiplier = 1.0;

         const double altitude_agl = transform.position.y; // Approximation for AGL
         const double wingspan = aero.wingspan_m;

         // The ground effect is significant when altitude is less than twice the wingspan.
         if (altitude_agl > 0 && altitude_agl < (2.0 * wingspan))
         {
            // Use a standard engineering approximation for ground effect.
            double h_over_b = altitude_agl / wingspan;
            drag_multiplier = (33.0 * pow(h_over_b, 1.5)) / (1.0 + 33.0 * pow(h_over_b, 1.5));
            lift_multiplier = 1.0 + (0.5 * (1.0 - drag_multiplier));
         }

         // Apply the multipliers to the base coefficients
         const double final_Cl = base_Cl * lift_multiplier;
         const double final_Cd = base_Cd * drag_multiplier;

         // --- 5. Calculate Final Forces ---
         const double dynamic_pressure = 0.5 * atmosphere.density * speed * speed;
         const double lift_magnitude = final_Cl * dynamic_pressure * aero.reference_area_m2;
         const double drag_magnitude = final_Cd * dynamic_pressure * aero.reference_area_m2;

         glm::dvec3 drag_force = -velocity_direction * drag_magnitude;

         // This is a correct calculation for the lift vector direction
         glm::dvec3 body_up_direction = glm::normalize(transform.orientation * glm::dvec3(0, 1, 0));
         glm::dvec3 lift_direction = glm::normalize(
            glm::cross(glm::cross(velocity_direction, body_up_direction), velocity_direction));
         glm::dvec3 lift_force = lift_direction * lift_magnitude;

         // --- 6. Add Forces to Accumulator ---
         accumulator.addForce(drag_force);
         accumulator.addForce(lift_force);
      }
   }
} // namespace StrikeEngine
