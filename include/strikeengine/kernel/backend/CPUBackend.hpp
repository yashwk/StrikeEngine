#pragma once

#include <strikeengine/kernel/backend/PhysicsBackend.hpp>
#include <strikeengine/kernel/integrator/Integrator.hpp>
#include <strikeengine/kernel/scheduler/HybridScheduler.hpp>
#include <strikeengine/kernel/config/EnvironmentConfig.hpp>
#include <strikeengine/models/physics/atmosphere/AtmosphereModel.hpp>
#include <strikeengine/models/physics/aerodynamics/AeroModel.hpp>
#include <strikeengine/models/physics/propulsion/PropulsionModel.hpp>
#include <strikeengine/kernel/backend/WorkerPool.hpp>
#include <memory>
#include <vector>

namespace StrikeEngine::Kernel
{

	/**
	 * @brief CPU physics backend — derivative-engine form (W2/W3 spine).
	 *
	 * Owns the per-entity propulsion pool (W1). Each step evaluates the full
	 * force/moment model at arbitrary states via evaluateDerivative and hands
	 * it to the (true) integrator, which re-evaluates at intermediate stages.
	 * 6-DOF body-frame rotational dynamics with gyroscopic coupling.
	 */
	class CPUBackend final : public PhysicsBackend
	{
	public:
		CPUBackend(
			std::unique_ptr<Integrator> integrator,
			std::shared_ptr<Models::AtmosphereModel> atmosphere,
			std::shared_ptr<Models::AeroModel> aero,
			EnvironmentConfig environment = {}
		);

		int registerPropulsion(std::shared_ptr<const Models::PropulsionModel> model) override;

		void setEnvironment(const EnvironmentConfig& environment) override;
		void setThreadCount(std::size_t threads) override;
		[[nodiscard]] std::size_t threadCount() const override;

		void initialize(
			PhysicsBlock& physics,
			ControlBlock& control) override;

		void step(
			PhysicsBlock& physics,
			ControlBlock& control,
			double currentTime,
			double dt) override;

		void shutdown() override;

	private:
		// Pure derivative evaluation: fills d with d(state)/dt at arbitrary state.
		void evaluateDerivative(
			const PhysicsBlock& state,
			const ControlBlock& control,
			double t,
			PhysicsBlock& d);

		void evaluateDerivativeChunk(
			const PhysicsBlock& state,
			const ControlBlock& control,
			double t,
			PhysicsBlock& d,
			std::size_t start,
			std::size_t end);

		void ensureDerivCapacity(const PhysicsBlock& state);

		std::unique_ptr<Integrator> integrator;
		std::unique_ptr<HybridScheduler> scheduler;

		std::shared_ptr<Models::AtmosphereModel> atmosphere;
		std::shared_ptr<Models::AeroModel> aero;
		EnvironmentConfig environment;

		// Per-entity propulsion pool (W1); entities reference by index.
		std::vector<std::shared_ptr<const Models::PropulsionModel>> propulsionPool;

		WorkerPool threadPool;
		PhysicsBlock derivBuffer;   // scratch for cache refresh + stage reuse

		// Current step dt, set by step() before integrating. The fuel-
		// depletion guard caps mass flow so the remaining fuel lasts this
		// window; a fixed window over- or under-covers the actual step.
		double currentStepDt = 0.01;
	};

} // namespace StrikeEngine::Kernel
