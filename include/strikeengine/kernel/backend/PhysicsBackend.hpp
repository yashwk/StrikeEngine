#pragma once

#include <cstddef>

namespace StrikeEngine::Kernel
{

	// Forward declarations
	struct PhysicsBlock;
	struct ControlBlock;

	/**
	 * @brief Abstract physics execution backend.
	 *
	 * Responsible for advancing physics state using SoA blocks.
	 * Must be deterministic and stateless with respect to global data.
	 */
	class PhysicsBackend
	{
	public:
		virtual ~PhysicsBackend() = default;

		virtual void initialize(
			PhysicsBlock& physics,
			ControlBlock& control) = 0;

		virtual void step(
			PhysicsBlock& physics,
			ControlBlock& control,
			double currentTime,
			double dt) = 0;

		virtual void shutdown() = 0;
	};

} // namespace StrikeEngine::Kernel