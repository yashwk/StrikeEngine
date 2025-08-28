#include "strikeengine/systems/guidance/SensorSystem.hpp"
#include "strikeengine/atmosphere/AtmosphereManager.hpp"

#include "strikeengine/components/guidance/SeekerComponent.hpp"
#include "strikeengine/components/guidance/AntennaComponent.hpp"
#include "strikeengine/components/metadata/RCSProfileComponent.hpp"
#include "strikeengine/components/sensors/InfraredSeekerComponent.hpp"
#include "strikeengine/components/metadata/InfraredSignatureComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"

#include <cmath>
#include <numbers>

namespace StrikeEngine {

    extern AtmosphereManager g_atmosphere_manager;

    void processRadarSeeker(Entity entity, Registry& registry, std::unordered_map<std::string, std::unique_ptr<RCSDatabase>>& cache);
    void processIRSeeker(Entity entity, Registry& registry, std::unordered_map<std::string, std::unique_ptr<IRSignatureDatabase>>& cache);

    // --- Main Update Loop ---
    void SensorSystem::update(Registry& registry, double dt) {
        auto view = registry.view<SeekerComponent>();

        for (auto entity : view) {
            auto& seeker = view.get<SeekerComponent>(entity);
            if (seeker.type == "RF") {
                processRadarSeeker(entity, registry, _rcs_database_cache);
            }
            else if (seeker.type == "IR") {
                processIRSeeker(entity, registry, _ir_database_cache);
            }
        }
    }

    // --- Radar Simulation Logic ---
    double dbToRatio(double db) { return std::pow(10.0, db / 10.0); }

    void processRadarSeeker(Entity entity, Registry& registry, std::unordered_map<std::string, std::unique_ptr<RCSDatabase>>& cache) {
        if (!registry.has<AntennaComponent>(entity) || !registry.has<TransformComponent>(entity)) return;

        auto& seeker = registry.get<SeekerComponent>(entity);
        auto& antenna = registry.get<AntennaComponent>(entity);
        auto& radar_transform = registry.get<TransformComponent>(entity);

        bool lock_maintained = false;
         auto target_view = registry.view<RCSProfileComponent, TransformComponent>();
        for ( auto target_entity : target_view) {
            auto& rcs_profile = target_view.get<RCSProfileComponent>(target_entity);
            auto& target_transform = target_view.get<TransformComponent>(target_entity);


            if (!cache.contains(rcs_profile.profile_path)) {
                auto db = std::make_unique<RCSDatabase>();
                if (db->loadProfile(rcs_profile.profile_path)) {
                    cache[rcs_profile.profile_path] = std::move(db);
                } else continue;
            }
            const auto& rcs_db = cache.at(rcs_profile.profile_path);

            glm::dvec3 range_vec = target_transform.position - radar_transform.position;
            double range = glm::length(range_vec);
            glm::dvec3 los_in_target_frame = glm::inverse(target_transform.orientation) * glm::normalize(range_vec);
            double azimuth_rad = std::atan2(los_in_target_frame.y, los_in_target_frame.x);
            double elevation_rad = std::asin(-los_in_target_frame.z);
            double rcs_m2 = rcs_db->getRCS(azimuth_rad, elevation_rad);

            double received_power = (antenna.transmitter_power_W * std::pow(dbToRatio(antenna.antenna_gain_dB), 2) * std::pow(antenna.wavelength_m, 2) * rcs_m2) /
                                  (std::pow(4.0 * std::numbers::pi, 3.0) * std::pow(range, 4.0));
            double snr_db = 10.0 * std::log10(received_power / antenna.noise_floor_W);

            if (snr_db > antenna.snr_threshold_dB) {
                seeker.has_lock = true;
                seeker.locked_target = target_entity;
                lock_maintained = true;
                break;
            }
        }
        if (!lock_maintained) {
            seeker.has_lock = false;
            seeker.locked_target = NULL_ENTITY;
        }
    }

    // --- Infrared Simulation Logic ---
    void processIRSeeker(Entity entity, Registry& registry, std::unordered_map<std::string, std::unique_ptr<IRSignatureDatabase>>& cache) {
        if (!registry.has<InfraredSeekerComponent>(entity) || !registry.has<TransformComponent>(entity)) return;

        auto& seeker = registry.get<SeekerComponent>(entity);
        auto& ir_seeker = registry.get<InfraredSeekerComponent>(entity);
        auto& seeker_transform = registry.get<TransformComponent>(entity);

        bool lock_maintained = false;
        auto target_view = registry.view<InfraredSignatureComponent, TransformComponent>();
        for (auto target_entity : target_view) {
            auto& ir_profile = target_view.get<InfraredSignatureComponent>(target_entity);
            auto& target_transform = target_view.get<TransformComponent>(target_entity);
            if (!cache.contains(ir_profile.profile_path)) {
                auto db = std::make_unique<IRSignatureDatabase>();
                if (db->loadProfile(ir_profile.profile_path)) {
                    cache[ir_profile.profile_path] = std::move(db);
                } else continue;
            }
            const auto& ir_db = cache.at(ir_profile.profile_path);

            glm::dvec3 range_vec = target_transform.position - seeker_transform.position;
            double range = glm::length(range_vec);
            glm::dvec3 los_in_target_frame = glm::inverse(target_transform.orientation) * glm::normalize(range_vec);
            double azimuth_rad = std::atan2(los_in_target_frame.y, los_in_target_frame.x);
            double elevation_rad = std::asin(-los_in_target_frame.z);

            double radiant_intensity_W_per_sr = ir_db->getRadiantIntensity(azimuth_rad, elevation_rad);
            double irradiance_W_per_m2 = radiant_intensity_W_per_sr / (range * range);

            double altitude = seeker_transform.position.y;
            double transmissivity = g_atmosphere_manager.getTransmissivity(range, altitude, ir_seeker.wavelength_band);

            double final_power_W = irradiance_W_per_m2 * transmissivity;

            if (final_power_W > ir_seeker.sensitivity_W) {
                seeker.has_lock = true;
                seeker.locked_target = target_entity;
                lock_maintained = true;
                break;
            }
        }
        if (!lock_maintained) {
            seeker.has_lock = false;
            seeker.locked_target = NULL_ENTITY;
        }
    }

} // namespace StrikeEngine
