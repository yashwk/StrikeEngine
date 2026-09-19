#pragma once
#include <strikeengine/kernel/data/BlockGrowth.hpp>
#include <vector>
#include <array>
#include <cstdint>
#include <cstddef>

namespace StrikeEngine::Kernel {

    // Persistent target-track lifecycle.
    //
    // Acquire:  initial state (external command seed or first measurement),
    //           waiting for `confirmations` consistent measurement updates.
    // Maintain: track confirmed; estimates refreshed by each measurement.
    // Coast:    no new measurement for `coastTimeoutSec`; multi-rate kinematic
    //           prediction continues at the simulation rate.
    // Lost:     no measurement for `lossTimeoutSec`; the track is dropped from
    //           guidance consumption (external command state becomes the aim).
    // Reacquire: a measurement (or command refresh) arrives for a Coast/Lost
    //           track; returns to Maintain after `confirmations` updates.
    enum class TrackState : uint8_t { None, Acquire, Maintain, Coast, Lost, Reacquire };

    /**
     * @brief Target-specific track relayed by a cooperative source.
     *
     * The ordinary TrackBlock state is one seeker track per entity. A
     * fire-control source can carry several independent tracks, so datalink
     * consumers address these records by (sourceEntityId, targetEntityId)
     * instead of accidentally sharing the source's currently locked target.
     */
    struct DatalinkTrack {
        int sourceEntityId = -1;
        int targetEntityId = -1;
        TrackState state = TrackState::None;
        std::int64_t trackId = -1;
        double posX = 0.0, posY = 0.0, posZ = 0.0;
        double velX = 0.0, velY = 0.0, velZ = 0.0;
        double accelX = 0.0, accelY = 0.0, accelZ = 0.0;
        bool accelAvailable = false;
        double timestampSec = 0.0;
        double ageSec = 0.0;
        double positionStdM = 5.0;
        double velocityStdMs = 25.0;
        double quality01 = 0.0;
        std::uint32_t updateCount = 0;

        bool active() const {
            return state != TrackState::None && state != TrackState::Lost;
        }
    };

    /**
     * @brief Per-entity persistent target-track state.
     *
     * The track manager fuses external command seeds (SimulationCommand) and
     * seeker LOS measurements (converted to the world frame through the
     * navigation estimate) into a single estimate consumed by guidance. The
     * guidance layer reads ONLY these estimates (never physics truth).
     */
    struct TrackBlock {
        // Target-specific cooperative relay tracks. These are runtime state;
        // the ordinary per-entity arrays below remain the seeker track API.
        std::vector<DatalinkTrack> datalinkTracks;

        // --- Per-entity track configuration (from GuidanceAutopilotConfig) ---
        std::vector<int>    confirmations;     // measurement updates to promote to Maintain
        std::vector<double> coastTimeoutSec;   // no measurement: Maintain/Acquire -> Coast
        std::vector<double> lossTimeoutSec;    // no measurement: Coast -> Lost
        // Estimator + association options (legacy defaults).
        std::vector<bool>   filterEnabled;         // constant-acceleration Kalman
        std::vector<double> processNoiseMps2;      // target accel PSD
        std::vector<double> angleStdRad;           // assumed seeker angular sigma
        std::vector<double> measNoiseScale;        // derived-R scale
        std::vector<double> residualGateSigma;     // 0 = off
        std::vector<double> maxAccelMps2;          // 0 = no clamp
        std::vector<int>    retargetConfirmations; // consistent ids before retarget
        std::vector<int>    seedPolicy;            // 0 clobber, 1 init-only, 2 refresh-stale
        std::vector<double> minQuality01;          // active() quality gate
        std::vector<double> qualityTauSec;         // quality decay
        std::vector<double> velocityBlend;         // legacy finite-diff low-pass

        // --- Per-entity track state -----------------------------------------
        std::vector<TrackState> state;         // None = no track
        std::vector<std::int64_t> trackId;     // target identity; -1 = unknown
        // Estimated world-frame target state (local ENU convention, same as nav)
        std::vector<double> posX, posY, posZ;
        std::vector<double> velX, velY, velZ;
        std::vector<double> accelX, accelY, accelZ;
        std::vector<bool>   accelAvailable;
        // Measurement provenance: sim time of the last measurement/seed and the
        // time since it (track age).
        std::vector<double> timestampSec;
        std::vector<double> ageSec;
        // Track quality / uncertainty (deterministic model: uncertainty grows
        // with age since the last measurement; quality decays exponentially).
        std::vector<double> positionStdM;
        std::vector<double> velocityStdMs;
        std::vector<double> quality01;
        // Measurement bookkeeping
        std::vector<std::uint32_t> updateCount;   // consecutive measurement updates
        std::vector<std::uint32_t> dropoutCount;  // consecutive steps without a measurement
        // Last measurement position/time (for finite-difference velocity)
        std::vector<double> measPosX, measPosY, measPosZ;
        std::vector<double> measTimeSec;

        // Filter state (constant-acceleration Kalman, 3x3 per axis packed as
        // [axis*9 + i*3 + j]). Only touched when filterEnabled.
        std::vector<std::array<double, 27>> kfCov;
        // Retarget debounce: candidate identity + consecutive-update count.
        std::vector<std::int64_t> retargetCandidateId;
        std::vector<std::uint32_t> retargetCount;
        // Diagnostics.
        std::vector<double> lastInnovationM;
        std::vector<std::uint32_t> residualRejectCount;

        std::size_t size = 0;

        const DatalinkTrack* findDatalinkTrack(int sourceEntityId,
                                                int targetEntityId) const {
            for (const auto& track : datalinkTracks) {
                if (track.sourceEntityId == sourceEntityId &&
                    track.targetEntityId == targetEntityId) {
                    return &track;
                }
            }
            return nullptr;
        }

        DatalinkTrack* findDatalinkTrack(int sourceEntityId, int targetEntityId) {
            for (auto& track : datalinkTracks) {
                if (track.sourceEntityId == sourceEntityId &&
                    track.targetEntityId == targetEntityId) {
                    return &track;
                }
            }
            return nullptr;
        }

        std::size_t datalinkTrackCount(int sourceEntityId) const {
            std::size_t count = 0;
            for (const auto& track : datalinkTracks) {
                if (track.sourceEntityId == sourceEntityId) ++count;
            }
            return count;
        }

        DatalinkTrack& ensureDatalinkTrack(int sourceEntityId,
                                            int targetEntityId) {
            if (auto* existing = findDatalinkTrack(sourceEntityId, targetEntityId)) {
                return *existing;
            }
            datalinkTracks.push_back({});
            auto& track = datalinkTracks.back();
            track.sourceEntityId = sourceEntityId;
            track.targetEntityId = targetEntityId;
            track.trackId = targetEntityId;
            return track;
        }

        bool active(std::size_t i) const {
            if (i >= size || state[i] == TrackState::None ||
                state[i] == TrackState::Lost) {
                return false;
            }
            if (i < minQuality01.size() && quality01[i] < minQuality01[i]) {
                return false;
            }
            return true;
        }

        /**
         * @brief Grows every vector to @p n entries; see
         *        PhysicsBlock::ensureSize.
         */
        void ensureSize(std::size_t n) {
            growTo(confirmations, n, 3);
            growTo(coastTimeoutSec, n, 0.5);
            growTo(lossTimeoutSec, n, 2.0);
            growTo(filterEnabled, n, false);
            growTo(processNoiseMps2, n, 15.0);
            growTo(angleStdRad, n, 0.003);
            growTo(measNoiseScale, n, 1.0);
            growTo(residualGateSigma, n, 0.0);
            growTo(maxAccelMps2, n, 0.0);
            growTo(retargetConfirmations, n, 1);
            growTo(seedPolicy, n, 0);
            growTo(minQuality01, n, 0.0);
            growTo(qualityTauSec, n, 1.0);
            growTo(velocityBlend, n, 0.08);
            growTo(state, n, TrackState::None);
            growTo(trackId, n, -1);
            growTo(posX, n, 0.0); growTo(posY, n, 0.0); growTo(posZ, n, 0.0);
            growTo(velX, n, 0.0); growTo(velY, n, 0.0); growTo(velZ, n, 0.0);
            growTo(accelX, n, 0.0); growTo(accelY, n, 0.0); growTo(accelZ, n, 0.0);
            growTo(accelAvailable, n, false);
            growTo(timestampSec, n, 0.0);
            growTo(ageSec, n, 0.0);
            growTo(positionStdM, n, 5.0);
            growTo(velocityStdMs, n, 25.0);
            growTo(quality01, n, 0.0);
            growTo(updateCount, n, 0u);
            growTo(dropoutCount, n, 0u);
            growTo(measPosX, n, 0.0); growTo(measPosY, n, 0.0); growTo(measPosZ, n, 0.0);
            growTo(measTimeSec, n, 0.0);
            growTo(kfCov, n);
            growTo(retargetCandidateId, n, -1);
            growTo(retargetCount, n, 0u);
            growTo(lastInnovationM, n, 0.0);
            growTo(residualRejectCount, n, 0u);
            if (n > size) size = n;
        }
    };

} // namespace StrikeEngine::Kernel
