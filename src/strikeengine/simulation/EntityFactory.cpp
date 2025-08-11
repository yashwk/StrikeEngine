// ===================================================================================
//  EntityFactory.cpp
//
//  Description:
//  This file provides the implementation for the EntityFactory class. The primary
//  responsibility of this factory is to parse JSON profile files and construct
//  fully formed entities within the ECS registry. It acts as the central hub for
//  translating data-driven definitions into live simulation objects, attaching
//  all specified components and configuring their initial parameters.
//
//  This modular approach allows the engine to be highly extensible; new
//  component types can be handled by simply adding a new parsing block to this
//  factory without altering the core simulation logic.
//
// ===================================================================================

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

        for (const std::string& compName : componentsToAdd) {
            if (compName == "transform") {
                const auto& c = data.at("initial_state").at("transform");
                TransformComponent transform;
                transform.position = c.at("position").get<glm::dvec3>();
                transform.orientation = c.at("orientation").get<glm::dquat>();
                _registry.add<TransformComponent>(newEntity, transform);
            }
            else if (compName == "mass") {
                const auto& c = data.at("mass_properties");
                MassComponent mass;
                mass.initialMass_kg = c.at("initial_kg").get<double>();
                mass.dryMass_kg = c.at("dry_kg").get<double>();
                mass.currentMass_kg = mass.initialMass_kg;
                mass.updateInverseMass();
                _registry.add<MassComponent>(newEntity, mass);
            }
            else if (compName == "inertia") {
                const auto& c = data.at("mass_properties");
                InertiaComponent inertia;
                inertia.inertiaTensor = c.at("inertia_tensor").get<glm::dmat3>();
                inertia.updateInverseTensor();
                _registry.add<InertiaComponent>(newEntity, inertia);
            }
            else if (compName == "velocity") {
                const auto& c = data.at("initial_state").at("velocity");
                VelocityComponent velocity;
                velocity.linear = c.at("linear").get<glm::dvec3>();
                velocity.angular = c.at("angular").get<glm::dvec3>();
                _registry.add<VelocityComponent>(newEntity, velocity);
            }
            else if (compName == "propulsion") {
                const auto& c = data.at("propulsion");
                PropulsionComponent propulsion;
                for (const auto& stage_data : c.at("stages")) {
                    propulsion.stages.push_back({
                        stage_data.at("name").get<std::string>(),
                        stage_data.at("stage_mass_kg").get<double>(),
                        stage_data.at("thrust_newtons").get<double>(),
                        stage_data.at("burn_time_seconds").get<double>(),
                        stage_data.at("specific_impulse_seconds").get<double>()
                    });
                }
                propulsion.active = true;
                _registry.add<PropulsionComponent>(newEntity, propulsion);
            }
            else if (compName == "aerodynamics") {
                const auto& c = data.at("aerodynamics");
                AerodynamicProfileComponent aero;
                aero.profileID = c.at("profile_id").get<std::string>();
                aero.referenceArea_m2 = c.at("reference_area_m2").get<double>();
                _registry.add<AerodynamicProfileComponent>(newEntity, aero);
            }
            else if (compName == "guidance") {
                const auto& c = data.at("guidance");
                GuidanceComponent guidance;

                auto lawString = c.at("law").get<std::string>();
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
            else if (compName == "seeker") {
                const auto& c = data.at("seeker");
                SeekerComponent seeker;
                seeker.type = c.at("type").get<std::string>();
                seeker.field_of_view_deg = c.at("field_of_view_deg").get<double>();
                seeker.gimbal_limit_deg = c.at("gimbal_limit_deg").get<double>();
                seeker.max_range_m = c.at("max_range_m").get<double>();
                _registry.add<SeekerComponent>(newEntity, seeker);
            }
            else if (compName == "target_signature") {
                const auto& c = data.at("target_signature");
                TargetComponent target;
                target.rcs_m2 = c.at("rcs_m2").get<double>();
                _registry.add<TargetComponent>(newEntity, target);
            }
            else if (compName == "imu") {
                const auto& c = data.at("imu");
                IMUComponent imu;
                imu.gyro_bias_drift_rate_deg_per_hr = c.at("gyro_bias_drift_rate_deg_per_hr").get<double>();
                imu.gyro_noise_density_deg_per_sqrt_hr = c.at("gyro_noise_density_deg_per_sqrt_hr").get<double>();
                imu.accelerometer_bias_milli_g = c.at("accelerometer_bias_milli_g").get<double>();
                imu.accelerometer_noise_density_g_per_sqrt_hz = c.at("accelerometer_noise_density_g_per_sqrt_hz").get<double>();
                _registry.add<IMUComponent>(newEntity, imu);
            }
            else if (compName == "navigation_state") _registry.add<NavigationStateComponent>(newEntity);
            else if (compName == "control_surfaces") _registry.add<ControlSurfaceComponent>(newEntity);
            else if (compName == "force_accumulator") _registry.add<ForceAccumulatorComponent>(newEntity);
            else if (compName == "autopilot_command") _registry.add<AutopilotCommandComponent>(newEntity);
            else if (compName == "autopilot_state") _registry.add<AutopilotStateComponent>(newEntity);
        }

        return newEntity;
    }

} // namespace StrikeEngine
