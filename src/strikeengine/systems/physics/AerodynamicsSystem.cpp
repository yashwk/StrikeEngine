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
	AerodynamicsSystem::AerodynamicsSystem ( const AtmosphereManager &atmosphereManager )
		: _atmosphereManager( atmosphereManager ) {}

	AerodynamicsSystem::~AerodynamicsSystem () = default;

	void AerodynamicsSystem::update ( Registry &registry , double dt ) {
		if (!_atmosphereManager.isLoaded()) { return; }

		auto view = registry.view < TransformComponent , VelocityComponent , AerodynamicProfileComponent ,
		                            ForceAccumulatorComponent >();
		for (auto [entity, transform, velocity, aero, accumulator] : view) {
			if (!_aeroDatabases.contains( aero.profileID )) {
				auto db                 = std::make_unique < AerodynamicsDatabase >();
				std::string profilePath = "data/aero/" + aero.profileID + ".json";
				if (db->loadProfile( profilePath )) { _aeroDatabases[aero.profileID] = std::move( db ); } else {
					continue;
				}
			}
			const auto &aeroDB = _aeroDatabases.at( aero.profileID );

			if (glm::length2( velocity.linear ) < 1e-6) {
				aero.currentAngleOfAttack_rad = 0.0;
				aero.currentMachNumber        = 0.0;
				continue;
			}

			const double altitude                 = glm::length( transform.position );
			const AtmosphereProperties atmosphere = _atmosphereManager.getProperties( altitude );
			const double speed                    = glm::length( velocity.linear );
			aero.currentMachNumber                = speed / atmosphere.speedOfSound;

			const glm::dvec3 velocity_dir     = glm::normalize( velocity.linear );
			const glm::dvec3 body_forward_dir = glm::normalize( transform.orientation * glm::dvec3( 0 , 0 , 1 ) );
			aero.currentAngleOfAttack_rad     = acos(
				glm::clamp( glm::dot( velocity_dir , body_forward_dir ) , -1.0 , 1.0 ) );

			auto [Cl, Cd] = aeroDB->getCoefficients( aero.currentMachNumber , aero.currentAngleOfAttack_rad );

			const double dynamic_pressure = 0.5 * atmosphere.density * speed * speed;
			const double lift_magnitude   = Cl * dynamic_pressure * aero.referenceArea_m2;
			const double drag_magnitude   = Cd * dynamic_pressure * aero.referenceArea_m2;

			glm::dvec3 drag_force = -velocity_dir * drag_magnitude;

			glm::dvec3 body_up_dir = glm::normalize( transform.orientation * glm::dvec3( 0 , 1 , 0 ) );
			glm::dvec3 lift_dir    = glm::normalize(
				glm::cross( glm::cross( velocity_dir , body_up_dir ) , velocity_dir ) );
			glm::dvec3 lift_force = lift_dir * lift_magnitude;

			accumulator.totalForce += drag_force;
			accumulator.totalForce += lift_force;
		}
	}
} // namespace StrikeEngine
