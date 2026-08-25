#pragma once

#include "strikeengine/ecs/Component.hpp"
#include "strikeengine/ecs/Entity.hpp"
#include <string>

namespace StrikeEngine {
	enum class SeekerType {
		RF,
		IR
	};

	enum class SeekerMode {
		ACTIVE,
		INACTIVE,
		SEMI_ACTIVE,
		PASSIVE
	};

	/**
	 * @brief Defines the properties and state of an onboard seeker/sensor.
	 */
	struct SeekerComponent final : public Component {
		// --- Properties (Loaded from profile) ---

		SeekerType type;
		SeekerMode mode;
		double field_of_view_deg;
		double gimbal_limit_deg;
		double max_range_m;

		// --- State Variables ---
		// TODO : strict state management
		bool is_active = false;

		bool has_lock = false;
		Entity locked_target = NULL_ENTITY;
	};
} // namespace StrikeEngine
