#include "strikeengine/systems/physics/PropulsionSystem.hpp"
#include "strikeengine/atmosphere/AtmosphereManager.hpp"
#include "strikeengine/components/physics/PropulsionComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/physics/ForceAccumulatorComponent.hpp"
#include "strikeengine/components/physics/MassComponent.hpp"

#include <algorithm>

namespace StrikeEngine {

    // Helper function to perform linear interpolation on the thrust curve
    double getThrustFromCurve(double currentTime, const std::vector<ThrustDataPoint>& curve) {
        if (curve.empty()) return 0.0;
        if (currentTime <= curve.front().first) return curve.front().second;
        if (currentTime >= curve.back().first) return curve.back().second;

        auto it = std::lower_bound(curve.begin(), curve.end(), currentTime,
            [](const ThrustDataPoint& p, double time) { return p.first < time; });

        const auto& p2 = *it;
        const auto& p1 = *(--it);

        double t1 = p1.first, thrust1 = p1.second;
        double t2 = p2.first, thrust2 = p2.second;
        double fraction = (currentTime - t1) / (t2 - t1);
        return thrust1 + fraction * (thrust2 - thrust1);
    }

    PropulsionSystem::PropulsionSystem(const AtmosphereManager& atmosphereManager)
        : _atmosphere_manager(atmosphereManager) {}

    PropulsionSystem::~PropulsionSystem() = default;

    void PropulsionSystem::update(Registry& registry, double dt) {
        if (!_atmosphere_manager.isLoaded()) { return; }

        auto view = registry.view<PropulsionComponent, TransformComponent, ForceAccumulatorComponent, MassComponent>();

        for (auto entity: view) {
            auto& propulsion = view.get<PropulsionComponent>(entity);
            auto& transform = view.get<TransformComponent>(entity);
            auto& accumulator = view.get<ForceAccumulatorComponent>(entity);
            auto& mass = view.get<MassComponent>(entity);
            if (!propulsion.active || propulsion.currentStageIndex < 0 || propulsion.currentStageIndex >= propulsion.stages.size()) {
                continue;
            }

            auto& currentStage = propulsion.stages[propulsion.currentStageIndex];

            if (propulsion.timeInCurrentStage_seconds >= currentStage.burnTime_seconds) {
                mass.currentMass_kg -= currentStage.stage_mass_kg;
                mass.updateInverseMass();
                propulsion.currentStageIndex++;
                propulsion.timeInCurrentStage_seconds = 0.0;
                if (propulsion.currentStageIndex >= propulsion.stages.size()) {
                    propulsion.active = false;
                }
                continue;
            }

            double currentThrust = getThrustFromCurve(propulsion.timeInCurrentStage_seconds, currentStage.thrust_curve);

            if (currentThrust > 0.0) {
                glm::dvec3 thrustDirection = transform.orientation * glm::dvec3(1.0, 0.0, 0.0); // Assuming X is forward
                glm::dvec3 thrustForce = thrustDirection * currentThrust;
                accumulator.addForce(thrustForce);

                // --- NEW: Calculate Fuel Consumption with Atmospheric Effects ---
                const double altitude = glm::length(transform.position); // Approximation
                const double ambient_pressure_pa = _atmosphere_manager.getProperties(altitude).pressure;
                constexpr double sea_level_pressure_pa = 101325.0;

                // Interpolate Isp based on pressure
                double pressure_fraction = std::clamp(ambient_pressure_pa / sea_level_pressure_pa, 0.0, 1.0);
                double current_isp = currentStage.isp_vacuum_s + (currentStage.isp_sea_level_s - currentStage.isp_vacuum_s) * pressure_fraction;

                // Fuel flow rate = Thrust / (Isp * g0)
                const double g0 = 9.80665;
                if (current_isp > 0) {
                    double fuelFlowRate = currentThrust / (current_isp * g0);
                    mass.currentMass_kg -= fuelFlowRate * dt;
                    mass.updateInverseMass();
                }
            }

            propulsion.timeInCurrentStage_seconds += dt;
        }
    }

} // namespace StrikeEngine
