#pragma once

#include "PhysicsBackend.hpp"
#include "../integrator/Integrator.hpp"
#include "../scheduler/HybridScheduler.hpp"
#include "../../models/physics/atmosphere/AtmosphereModel.hpp"
#include "../../models/physics/aerodynamics/AeroModel.hpp"
#include "../../models/physics/propulsion/PropulsionModel.hpp"
#include <memory>

namespace StrikeEngine::Kernel
{

	class CPUBackend final : public PhysicsBackend
	{
	public:
		CPUBackend(
			std::unique_ptr<Integrator> integrator,
			std::shared_ptr<Models::AtmosphereModel> atmosphere,
			std::shared_ptr<Models::AeroModel> aero,
			std::shared_ptr<Models::PropulsionModel> propulsion
		);

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
		void computeForces(
			PhysicsBlock& physics,
			ControlBlock& control,
			double currentTime,
			double dt);

		std::unique_ptr<Integrator> integrator;
		std::unique_ptr<HybridScheduler> scheduler;

		std::shared_ptr<Models::AtmosphereModel> atmosphere;
		std::shared_ptr<Models::AeroModel> aero;
		std::shared_ptr<Models::PropulsionModel> propulsion;
	};

} // namespace StrikeEngine::Kernel