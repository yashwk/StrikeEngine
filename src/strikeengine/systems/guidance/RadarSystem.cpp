#include "strikeengine/systems/guidance/RadarSystem.hpp"
#include "strikeengine/components/guidance/AntennaComponent.hpp"
#include "strikeengine/components/guidance/SeekerComponent.hpp"
#include "strikeengine/components/metadata/RCSProfileComponent.hpp"
#include "strikeengine/components/transform/TransformComponent.hpp"

#include <cmath>

namespace StrikeEngine {

    // Helper functions to convert decibels (dB) to a linear ratio
    double dbToRatio(double db) {
        return std::pow(10.0, db / 10.0);
    }

    void RadarSystem::update(Registry& registry, double dt) {
        auto radar_view = registry.view<AntennaComponent, SeekerComponent, TransformComponent>();
        auto target_view = registry.view<RCSProfileComponent, TransformComponent>();

        for (auto [radar_entity, antenna, seeker, radar_transform] : radar_view) {

            // For now, assume the seeker is always looking for the first available target.
            // A more advanced implementation would have target selection logic.
            bool lock_maintained = false;

            for (auto [target_entity, rcs_profile, target_transform] : target_view) {

                // --- 1. Load RCS Database (if not already cached) ---
                if (!_rcs_database_cache.contains(rcs_profile.profilePath)) {
                    auto db = std::make_unique<RCSDatabase>();
                    if (db->loadProfile(rcs_profile.profilePath)) {
                        _rcs_database_cache[rcs_profile.profilePath] = std::move(db);
                    } else {
                        continue; // Skip the target if its profile cannot be loaded
                    }
                }
                const auto& rcs_db = _rcs_database_cache.at(rcs_profile.profilePath);

                // --- 2. Calculate Geometry & Aspect Angles ---
                glm::dvec3 range_vec = target_transform.position - radar_transform.position;
                double range = glm::length(range_vec);

                // Transform the line-of-sight vector into the target's local reference frame
                glm::dvec3 los_in_target_frame = glm::inverse(target_transform.orientation) * glm::normalize(range_vec);

                // Calculate aspect angles
                double azimuth_rad = std::atan2(los_in_target_frame.y, los_in_target_frame.x);
                double elevation_rad = std::asin(-los_in_target_frame.z);

                // --- 3. Get Dynamic RCS from Database ---
                double rcs_m2 = rcs_db->getRCS(azimuth_rad, elevation_rad);

                // --- 4. Execute Radar Range Equation ---
                double transmitter_power = antenna.transmitter_power_W;
                double antenna_gain = dbToRatio(antenna.antenna_gain_dB);
                double lambda = antenna.wavelength_m;

                // Pr = (P_t * G^2 * lambda^2 * sigma) / ((4pi)^3 * R^4)
                double received_power = (transmitter_power * antenna_gain * antenna_gain * lambda * lambda * rcs_m2) /
                                      (std::pow(4.0 * M_PI, 3.0) * std::pow(range, 4.0));

                // --- 5. Calculate SNR ---
                double snr_linear = received_power / antenna.noise_floor_W;
                double snr_db = 10.0 * std::log10(snr_linear);

                // --- 6. Determine Lock Status ---
                if (snr_db > antenna.snr_threshold_dB) {
                    seeker.has_lock = true;
                    seeker.locked_target = target_entity;
                    lock_maintained = true;
                    break; // Lock acquired, move to the next radar
                }
            }

            // If the loop completes and no target met the lock condition, drop the lock.
            if (!lock_maintained) {
                seeker.has_lock = false;
                seeker.locked_target = NULL_ENTITY;
            }
        }
    }

} // namespace StrikeEngine
