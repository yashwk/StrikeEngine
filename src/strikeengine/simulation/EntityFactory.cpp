#include "strikeengine/simulation/EntityFactory.hpp"
#include "strikeengine/ecs/Registry.hpp"
#include "strikeengine/utils/JsonGlm.hpp"
#include "nlohmann/json.hpp"

// --- Include all component headers that the factory can create ---
#include "strikeengine/components/transform/TransformComponent.hpp"
#include "strikeengine/components/physics/MassComponent.hpp"
#include "strikeengine/components/physics/InertiaComponent.hpp"
#include "strikeengine/components/physics/VelocityComponent.hpp"
#include "strikeengine/components/physics/PropulsionComponent.hpp"
#include "strikeengine/components/physics/AerodynamicProfileComponent.hpp"
#include "strikeengine/components/physics/ControlSurfaceComponent.hpp"
#include "strikeengine/components/physics/ForceAccumulatorComponent.hpp"
#include "strikeengine/components/guidance/GuidanceComponent.hpp"
#include "strikeengine/components/guidance/AutopilotCommandComponent.hpp"
#include "strikeengine/components/guidance/AutopilotStateComponent.hpp"
#include "strikeengine/components/guidance/SeekerComponent.hpp"
#include "strikeengine/components/sensors/GPSComponent.hpp"
#include "strikeengine/components/metadata/TargetComponent.hpp"
#include "strikeengine/components/physics/IMUComponent.hpp"
#include "strikeengine/components/physics/NavigationStateComponent.hpp"

#include <fstream>
#include <stdexcept>
#include <iostream>

namespace StrikeEngine {

    using json = nlohmann::json;

    EntityFactory::EntityFactory(Registry& registry) : _registry(registry) {}

    Entity EntityFactory::createFromProfile(const std::string& profilePath) {
        std::ifstream f(profilePath);
        if (!f.is_open()) {
            throw std::runtime_error("EntityFactory: Could not open profile file: " + profilePath);
        }

        json data;
        try {
            data = json::parse(f);
        } catch (const json::parse_error& e) {
            throw std::runtime_error("EntityFactory: Failed to parse JSON profile '" + profilePath + "': " + e.what());
        }

        Entity newEntity = _registry.createEntity();
        std::cout << "Creating entity '" << data.at("name").get<std::string>() << "' with ID " << newEntity.id << std::endl;

        const auto& componentsToAdd = data.at("simulation").at("components_to_add");

        for (const std::string& componentName : componentsToAdd) {
            if (componentName == "transform") {
                const auto& c = data.at("initial_state").at("transform");
                TransformComponent transform;
                transform.position = c.at("position").get<glm::dvec3>();
                transform.orientation = c.at("orientation").get<glm::dquat>();
                _registry.add<TransformComponent>(newEntity, transform);
            }
            else if (componentName == "mass") {
                const auto& c = data.at("mass_properties");
                MassComponent mass;
                mass.initialMass_kg = c.at("initial_kg").get<double>();
                mass.dryMass_kg = c.at("dry_kg").get<double>();
                mass.currentMass_kg = mass.initialMass_kg;
                mass.updateInverseMass();
                _registry.add<MassComponent>(newEntity, mass);
            }
            else if (componentName == "inertia") {
                const auto& c = data.at("mass_properties");
                InertiaComponent inertia;
                inertia.inertiaTensor = c.at("inertia_tensor").get<glm::dmat3>();
                inertia.updateInverseTensor();
                _registry.add<InertiaComponent>(newEntity, inertia);
            }
            else if (componentName == "velocity") {
                const auto& c = data.at("initial_state").at("velocity");
                VelocityComponent velocity;
                velocity.linear = c.at("linear").get<glm::dvec3>();
                velocity.angular = c.at("angular").get<glm::dvec3>();
                _registry.add<VelocityComponent>(newEntity, velocity);
            }
            else if (componentName == "propulsion") {
                const auto& c = data.at("propulsion");
                auto& propulsion = _registry.add<PropulsionComponent>(newEntity);

                for (const auto& stage_data : c.at("stages")) {
                    PropulsionStage stage;
                    stage.name = stage_data.at("name").get<std::string>();
                    stage.stage_mass_kg = stage_data.at("stage_mass_kg").get<double>();
                    stage.burnTime_seconds = stage_data.at("burnTime_seconds").get<double>();

                    // --- NEW: Parse the sea-level and vacuum Isp values ---
                    stage.isp_sea_level_s = stage_data.value("isp_sea_level_s", 0.0);
                    stage.isp_vacuum_s = stage_data.value("isp_vacuum_s", 0.0);

                    if (stage_data.contains("thrust_curve")) {
                        for (const auto& point : stage_data.at("thrust_curve")) {
                            if (point.is_array() && point.size() == 2) {
                                stage.thrust_curve.push_back({point[0].get<double>(), point[1].get<double>()});
                            }
                        }
                    }
                    propulsion.stages.push_back(stage);
                }

                propulsion.active = c.value("active", false);
                if (propulsion.active && !propulsion.stages.empty()) {
                    propulsion.currentStageIndex = 0;
                }
            }
            else if (componentName == "aerodynamics") {
                const auto& c = data.at("aerodynamics");
                auto& aero = _registry.add<AerodynamicProfileComponent>(newEntity);
                aero.profileID = c.at("profile_id").get<std::string>();
                aero.referenceArea_m2 = c.at("reference_area_m2").get<double>();
                aero.wingspan_m = c.value("wingspan_m", 1.0);
            }
            else if (componentName == "guidance") {
                const auto& c = data.at("guidance");
                GuidanceComponent guidance;
                std::string lawString = c.at("law").get<std::string>();
                if (lawString == "AugmentedProportionalNavigation") {
                    guidance.law = GuidanceLaw::AugmentedProportionalNavigation;
                } else if (lawString == "PurePursuit") {
                    guidance.law = GuidanceLaw::PurePursuit;
                } else {
                    guidance.law = GuidanceLaw::ProportionalNavigation;
                }
                guidance.navigation_constant = c.at("navigation_constant").get<double>();
                _registry.add<GuidanceComponent>(newEntity, guidance);
            }
            else if (componentName == "seeker") {
                const auto& c = data.at("seeker");
                SeekerComponent seeker;
                seeker.type = c.at("type").get<std::string>();
                seeker.field_of_view_deg = c.at("field_of_view_deg").get<double>();
                seeker.gimbal_limit_deg = c.at("gimbal_limit_deg").get<double>();
                seeker.max_range_m = c.at("max_range_m").get<double>();
                _registry.add<SeekerComponent>(newEntity, seeker);
            }
            else if (componentName == "target_signature") {
                const auto& c = data.at("target_signature");
                TargetComponent target;
                target.rcs_m2 = c.at("rcs_m2").get<double>();
                _registry.add<TargetComponent>(newEntity, target);
            }
            else if (componentName == "imu") {
                const auto& c = data.at("imu");
                IMUComponent imu;
                imu.gyro_bias_drift_rate_deg_per_hr = c.at("gyro_bias_drift_rate_deg_per_hr").get<double>();
                imu.gyro_noise_density_deg_per_sqrt_hr = c.at("gyro_noise_density_deg_per_sqrt_hr").get<double>();
                imu.accelerometer_bias_milli_g = c.at("accelerometer_bias_milli_g").get<double>();
                imu.accelerometer_noise_density_g_per_sqrt_hz = c.at("accelerometer_noise_density_g_per_sqrt_hz").get<double>();
                _registry.add<IMUComponent>(newEntity, imu);
            }
            else if (componentName == "gps") {
                if (data.contains("gps")) {
                    const auto& c = data.at("gps");
                    auto& gps_comp = _registry.add<GPSComponent>(newEntity);
                    gps_comp.update_rate_hz = c.value("update_rate_hz", 1.0);
                    gps_comp.position_error_m = c.value("position_error_m", 3.0);
                    gps_comp.time_since_last_update_s = 0.0;
                }
            }
            else if (componentName == "navigation_state") _registry.add<NavigationStateComponent>(newEntity);
            else if (componentName == "control_surfaces") _registry.add<ControlSurfaceComponent>(newEntity);
            else if (componentName == "force_accumulator") _registry.add<ForceAccumulatorComponent>(newEntity);
            else if (componentName == "autopilot_command") _registry.add<AutopilotCommandComponent>(newEntity);
            else if (componentName == "autopilot_state") _registry.add<AutopilotStateComponent>(newEntity);
        }

        if (data.contains("autopilot")) {
            const auto& autopilot_data = data.at("autopilot");
            if (_registry.has<AutopilotStateComponent>(newEntity)) {
                auto& autopilot_state = _registry.get<AutopilotStateComponent>(newEntity);
                autopilot_state.kp = autopilot_data.value("kp", 0.8);
                autopilot_state.ki = autopilot_data.value("ki", 0.2);
                autopilot_state.kd = autopilot_data.value("kd", 0.1);
            }
            if (_registry.has<ControlSurfaceComponent>(newEntity)) {
                auto& control_surface = _registry.get<ControlSurfaceComponent>(newEntity);
                double max_deflection_deg = autopilot_data.value("max_deflection_deg", 20.0);
                control_surface.max_deflection_rad = glm::radians(max_deflection_deg);
                double max_rate_deg_per_sec = autopilot_data.value("max_rate_deg_per_sec", 300.0);
                control_surface.max_rate_rad_per_sec = glm::radians(max_rate_deg_per_sec);
            }
        }

        return newEntity;
    }

} // namespace StrikeEngine
