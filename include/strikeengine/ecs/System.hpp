#pragma once

namespace StrikeEngine {
	class Registry;
}

namespace StrikeEngine {
	/**
	 * @brief The base class for all systems in the engine.
	 */
	class System {
		public:
			virtual ~System () = default;

			/**
			 * @brief The main update function for the system.
			 * @param registry A reference to the ECS registry.
			 * @param dt The time elapsed since the last frame (delta time).
			 */
			virtual void update ( Registry &registry , double dt ) ;
	};
} // namespace StrikeEngine
