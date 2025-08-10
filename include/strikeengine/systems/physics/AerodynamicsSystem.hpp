#pragma once

#include "strikeengine/ecs/System.hpp"
#include <string>
#include <map>
#include <memory>

namespace StrikeEngine {
	class Registry;
	class AtmosphereManager;
	class AerodynamicsDatabase;
}

namespace StrikeEngine {
	/**
	 * @brief Calculates and applies aerodynamic forces (lift and drag) to entities.
	 */
	class AerodynamicsSystem final : public System {
		public:
			explicit AerodynamicsSystem ( const AtmosphereManager &atmosphereManager );

			~AerodynamicsSystem () override;


			void update ( Registry &registry , double dt ) override;

		private:
			const AtmosphereManager &_atmosphereManager;

			std::map < std::string , std::unique_ptr < AerodynamicsDatabase > > _aeroDatabases;
	};
} // namespace StrikeEngine
