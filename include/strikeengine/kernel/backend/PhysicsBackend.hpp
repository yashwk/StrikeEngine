#pragma once

#include <cstddef>
#include <memory>

namespace StrikeEngine::Models { class PropulsionModel; }

namespace StrikeEngine::Kernel
{

	// Forward declarations
	struct PhysicsBlock;
	struct ControlBlock;
	struct EnvironmentConfig;

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

		/**
		 * @brief Registers a per-entity propulsion model.
		 * @return Pool index for the entity's propulsionId, or -1 if the
		 *         backend does not support per-entity propulsion models.
		 */
		virtual int registerPropulsion(std::shared_ptr<const Models::PropulsionModel> model)
		{
			(void)model;
			return -1;
		}

		/**
		 * @brief Releases a previously registered propulsion pool slot so the
		 * backend may recycle it for a future entity. Called when an entity
		 * is removed; backends that cannot recycle may ignore the call.
		 */
		virtual void releasePropulsion(int poolId)
		{
			(void)poolId;
		}

		/**
		 * @brief Drops per-run cached state (e.g. registered propulsion
		 * models) so a reset kernel starts clean. No-op by default.
		 */
		virtual void reset()
		{
		}

		virtual void setEnvironment(const EnvironmentConfig& environment)
		{
			(void)environment;
		}

		virtual void setThreadCount(std::size_t threads)
		{
			(void)threads;
		}

		[[nodiscard]] virtual std::size_t threadCount() const
		{
			return 1;
		}

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
