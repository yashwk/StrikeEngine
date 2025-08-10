#pragma once

#include "strikeengine/ecs/Component.hpp"
#include "strikeengine/ecs/Entity.hpp"
#include <string>

namespace StrikeEngine {

	enum class SeekerType {
		RF,
		IR,
		IIR,
		LASER
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
		std::string type = "RF"; // e.g., "RF" (Radio Frequency), "IR" (Infrared)
		double field_of_view_deg = 10.0;
		double gimbal_limit_deg = 60.0;
		double max_range_m = 25000.0;

		// --- State Variables ---
		bool is_active = false;
		bool has_lock = false;
		Entity locked_target = NULL_ENTITY;
	};

} // namespace StrikeEngine
