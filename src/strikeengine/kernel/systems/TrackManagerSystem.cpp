#include <strikeengine/kernel/systems/TrackManagerSystem.hpp>
#include <cmath>
#include <algorithm>
#include <array>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace StrikeEngine::Kernel {

    namespace {

        // Deterministic quality/uncertainty model for a seeker-converted LOS
        // fix. These are legacy defaults; the filter path derives uncertainty
        // from the measurement instead. Per-entity state timers come from
        // GuidanceAutopilotConfig.
        constexpr double kQualityTauSec = 1.0;       // quality exp decay time constant
        constexpr double kPosStdBaseM   = 5.0;       // single-fix position uncertainty
        constexpr double kPosDriftMps   = 25.0;      // position uncertainty growth rate
        constexpr double kVelStdBaseMps = 25.0;      // single-fix velocity uncertainty
        constexpr double kVelDriftMps2  = 50.0;      // velocity uncertainty growth rate

        bool trackActive(TrackState s) {
            return s != TrackState::None && s != TrackState::Lost;
        }

        bool flagAt(const std::vector<bool>& v, std::size_t i) {
            return i < v.size() && v[i];
        }
        double valAt(const std::vector<double>& v, std::size_t i, double def) {
            return i < v.size() ? v[i] : def;
        }
        int intAt(const std::vector<int>& v, std::size_t i, int def) {
            return i < v.size() ? v[i] : def;
        }

        // --- Constant-acceleration Kalman blocks (one axis, 3x3 covariance
        // packed row-major). q is the target-acceleration white-noise PSD. ---
        void kfPredictAxis(double& p, double& v, double& a, double* P,
                           double dt, double q, double maxAccel) {
            const double dt2 = 0.5 * dt * dt;
            p += v * dt + a * dt2;
            v += a * dt;
            if (maxAccel > 0.0) a = std::clamp(a, -maxAccel, maxAccel);

            double FP[9];
            for (int j = 0; j < 3; ++j) {
                FP[0 * 3 + j] = P[0 * 3 + j] + dt * P[1 * 3 + j] + dt2 * P[2 * 3 + j];
                FP[1 * 3 + j] = P[1 * 3 + j] + dt * P[2 * 3 + j];
                FP[2 * 3 + j] = P[2 * 3 + j];
            }
            double Pn[9];
            for (int i = 0; i < 3; ++i) {
                Pn[i * 3 + 0] = FP[i * 3 + 0] + dt * FP[i * 3 + 1] + dt2 * FP[i * 3 + 2];
                Pn[i * 3 + 1] = FP[i * 3 + 1] + dt * FP[i * 3 + 2];
                Pn[i * 3 + 2] = FP[i * 3 + 2];
            }
            const double dt3 = dt * dt * dt;
            Pn[0] += q * dt * dt * dt * dt * dt / 20.0;
            Pn[1] += q * dt * dt * dt * dt / 8.0;
            Pn[3] += q * dt * dt * dt * dt / 8.0;
            Pn[2] += q * dt3 / 6.0;
            Pn[6] += q * dt3 / 6.0;
            Pn[4] += q * dt3 / 3.0;
            Pn[5] += q * dt * dt / 2.0;
            Pn[7] += q * dt * dt / 2.0;
            Pn[8] += q * dt;
            for (int k = 0; k < 9; ++k) P[k] = Pn[k];
        }

        // Scalar position update. Returns false if the residual gate rejects.
        bool kfUpdateAxis(double& p, double& v, double& a, double* P,
                          double z, double r, double gateSigma, double& innovation) {
            const double s = P[0] + r;
            if (!(s > 1e-12)) return false;
            const double y = z - p;
            innovation = y;
            if (gateSigma > 0.0 && std::abs(y) > gateSigma * std::sqrt(s)) {
                return false;
            }
            const double K0 = P[0] / s, K1 = P[3] / s, K2 = P[6] / s;
            p += K0 * y;
            v += K1 * y;
            a += K2 * y;
            double Pn[9];
            for (int i = 0; i < 3; ++i) {
                const double Ki = (i == 0) ? K0 : (i == 1 ? K1 : K2);
                for (int j = 0; j < 3; ++j) Pn[i * 3 + j] = P[i * 3 + j] - Ki * P[0 * 3 + j];
            }
            for (int i = 0; i < 3; ++i) {
                for (int j = i + 1; j < 3; ++j) {
                    const double m = 0.5 * (Pn[i * 3 + j] + Pn[j * 3 + i]);
                    Pn[i * 3 + j] = m; Pn[j * 3 + i] = m;
                }
            }
            for (int k = 0; k < 9; ++k) P[k] = Pn[k];
            return true;
        }

        void kfInitAxis(double& p, double& v, double& a, double* P,
                        double z, double r) {
            p = z; v = 0.0; a = 0.0;
            for (int k = 0; k < 9; ++k) P[k] = 0.0;
            P[0] = r;
            P[4] = 1.0e4;   // velocity prior std 100 m/s
            P[8] = 1.0e4;   // acceleration prior std 100 m/s^2
        }

        // Grow only the newly-added state arrays so hand-built unit-test blocks
        // stay safe (legacy arrays are always sized by callers).
        void ensureTrackState(TrackBlock& t, std::size_t n) {
            if (t.retargetCandidateId.size() < n) t.retargetCandidateId.resize(n, -1);
            if (t.retargetCount.size() < n) t.retargetCount.resize(n, 0);
            if (t.kfCov.size() < n) t.kfCov.resize(n);
            if (t.lastInnovationM.size() < n) t.lastInnovationM.resize(n, 0.0);
            if (t.residualRejectCount.size() < n) t.residualRejectCount.resize(n, 0);
        }

    }

    void TrackManagerSystem::update(
        const NavigationBlock& nav,
        const SeekerBlock& seeker,
        TrackBlock& tracks,
        double timeSec,
        double dt)
    {
        const std::size_t n = std::min({tracks.size, nav.size, seeker.size});
        ensureTrackState(tracks, n);
        for (std::size_t i = 0; i < n; ++i) {
            auto& state = tracks.state[i];
            const bool useFilter = flagAt(tracks.filterEnabled, i);

            // --- Convert a seeker LOS fix to a world position once ---------
            const bool rawMeasurement =
                seeker.type[i] != SeekerType::None && seeker.isLocked[i];
            bool measValid = false;
            glm::dvec3 measPos(0.0);
            std::int64_t newId = -1;
            double measVar = 0.0;
            if (rawMeasurement) {
                const double rng = seeker.targetRange[i];
                const double az  = seeker.targetAzimuth[i];
                const double el  = seeker.targetElevation[i];
                if (std::isfinite(rng) && rng > 0.0 && std::isfinite(az) &&
                    std::isfinite(el)) {
                    // Seeker body frame: X forward, Y right, Z down (same
                    // aerospace convention as the airframe). Elevation is
                    // below the X axis.
                    const double cEl = std::cos(el);
                    const glm::dvec3 losBody(
                        cEl * std::cos(az), cEl * std::sin(az), -std::sin(el));
                    const glm::dquat estQ(nav.estQw[i], nav.estQx[i],
                                          nav.estQy[i], nav.estQz[i]);
                    const glm::dvec3 losWorld = estQ * losBody;
                    const glm::dvec3 navPos(nav.estPx[i], nav.estPy[i], nav.estPz[i]);
                    measPos = navPos + losWorld * rng;
                    newId = static_cast<std::int64_t>(seeker.lockedTargetId[i]);

                    // Measurement variance: seeker range noise (when modeled),
                    // angular resolution/noise, and the nav attitude
                    // uncertainty projected over the range.
                    double posVar = 0.0;
                    if (flagAt(seeker.measurementNoiseEnabled, i)) {
                        const double rn = valAt(seeker.rangeNoiseStdDevM, i, 0.0);
                        posVar += rn * rn;
                    }
                    const double angleStd = flagAt(seeker.measurementNoiseEnabled, i)
                        ? valAt(seeker.angleNoiseStdDevRad, i, 0.0)
                        : valAt(tracks.angleStdRad, i, 0.003);
                    posVar += (rng * angleStd) * (rng * angleStd);
                    if (i < nav.covarianceDiag.size()) {
                        const double attVar = std::max(
                            {nav.covarianceDiag[i][6], nav.covarianceDiag[i][7],
                             nav.covarianceDiag[i][8]});
                        posVar += (rng * rng) * std::max(0.0, attVar);
                    }
                    measVar = valAt(tracks.measNoiseScale, i, 1.0) *
                              std::max(posVar, 1e-6);
                    measValid = true;
                }
            }

            // No-measurement step: age, predict, transition, decay uncertainty.
            // `alreadyPredicted` avoids a double predict on a gated fix.
            auto dropoutStep = [&](bool alreadyPredicted) {
                if (state == TrackState::None) return;
                tracks.ageSec[i] += dt;
                tracks.dropoutCount[i] += 1;

                if (!alreadyPredicted) {
                    if (useFilter) {
                        const double q = valAt(tracks.processNoiseMps2, i, 15.0);
                        const double maxA = valAt(tracks.maxAccelMps2, i, 0.0);
                        double* Px = tracks.kfCov[i].data();
                        double* Py = Px + 9;
                        double* Pz = Px + 18;
                        kfPredictAxis(tracks.posX[i], tracks.velX[i], tracks.accelX[i], Px, dt, q, maxA);
                        kfPredictAxis(tracks.posY[i], tracks.velY[i], tracks.accelY[i], Py, dt, q, maxA);
                        kfPredictAxis(tracks.posZ[i], tracks.velZ[i], tracks.accelZ[i], Pz, dt, q, maxA);
                    } else {
                        tracks.posX[i] += tracks.velX[i] * dt;
                        tracks.posY[i] += tracks.velY[i] * dt;
                        tracks.posZ[i] += tracks.velZ[i] * dt;
                        if (tracks.accelAvailable[i]) {
                            tracks.velX[i] += tracks.accelX[i] * dt;
                            tracks.velY[i] += tracks.accelY[i] * dt;
                            tracks.velZ[i] += tracks.accelZ[i] * dt;
                        }
                    }
                }

                const double coastT = tracks.coastTimeoutSec[i];
                const double lossT  = tracks.lossTimeoutSec[i];
                if (state == TrackState::Acquire || state == TrackState::Reacquire) {
                    if (tracks.ageSec[i] >= coastT) state = TrackState::Coast;
                } else if (state == TrackState::Maintain) {
                    if (tracks.ageSec[i] >= coastT) state = TrackState::Coast;
                } else if (state == TrackState::Coast) {
                    if (tracks.ageSec[i] >= lossT) state = TrackState::Lost;
                }

                if (trackActive(state)) {
                    if (useFilter) {
                        const double pVar = std::max(
                            {tracks.kfCov[i][0], tracks.kfCov[i][9], tracks.kfCov[i][18]});
                        const double vVar = std::max(
                            {tracks.kfCov[i][4], tracks.kfCov[i][13], tracks.kfCov[i][22]});
                        tracks.positionStdM[i] = std::sqrt(std::max(0.0, pVar));
                        tracks.velocityStdMs[i] = std::sqrt(std::max(0.0, vVar));
                    } else {
                        const double age = tracks.ageSec[i];
                        tracks.positionStdM[i] = kPosStdBaseM + kPosDriftMps * age;
                        tracks.velocityStdMs[i] = kVelStdBaseMps + kVelDriftMps2 * age;
                    }
                    const double tau = std::max(1e-9, valAt(tracks.qualityTauSec, i, kQualityTauSec));
                    tracks.quality01[i] = std::exp(-tracks.ageSec[i] / tau);
                }
            };

            if (!measValid) {
                dropoutStep(false);
                continue;
            }

            // --- Association / retarget debounce --------------------------
            const bool sameTarget = tracks.trackId[i] == newId;
            if (!sameTarget) {
                if (tracks.retargetCandidateId[i] == newId) {
                    tracks.retargetCount[i] += 1;
                } else {
                    tracks.retargetCandidateId[i] = newId;
                    tracks.retargetCount[i] = 1;
                }
                const int need = std::max(
                    1, intAt(tracks.retargetConfirmations, i, 1));
                if (static_cast<int>(tracks.retargetCount[i]) < need) {
                    dropoutStep(false);   // not a confirmed retarget yet
                    continue;
                }
            }
            tracks.retargetCandidateId[i] = newId;
            tracks.retargetCount[i] = 0;

            tracks.timestampSec[i] = timeSec;
            tracks.ageSec[i] = 0.0;
            tracks.dropoutCount[i] = 0;

            if (useFilter) {
                if (state == TrackState::None || !sameTarget) {
                    // Fresh track: initialise the filter directly from the fix.
                    double* Px = tracks.kfCov[i].data();
                    kfInitAxis(tracks.posX[i], tracks.velX[i], tracks.accelX[i], Px, measPos.x, measVar);
                    kfInitAxis(tracks.posY[i], tracks.velY[i], tracks.accelY[i], Px + 9, measPos.y, measVar);
                    kfInitAxis(tracks.posZ[i], tracks.velZ[i], tracks.accelZ[i], Px + 18, measPos.z, measVar);
                } else {
                    // Predict to now, then fuse with the residual gate.
                    const double q = valAt(tracks.processNoiseMps2, i, 15.0);
                    const double maxA = valAt(tracks.maxAccelMps2, i, 0.0);
                    double* Px = tracks.kfCov[i].data();
                    kfPredictAxis(tracks.posX[i], tracks.velX[i], tracks.accelX[i], Px, dt, q, maxA);
                    kfPredictAxis(tracks.posY[i], tracks.velY[i], tracks.accelY[i], Px + 9, dt, q, maxA);
                    kfPredictAxis(tracks.posZ[i], tracks.velZ[i], tracks.accelZ[i], Px + 18, dt, q, maxA);

                    const double gate = valAt(tracks.residualGateSigma, i, 0.0);
                    double innov = 0.0;
                    const bool okX = kfUpdateAxis(tracks.posX[i], tracks.velX[i], tracks.accelX[i], Px, measPos.x, measVar, gate, innov);
                    tracks.lastInnovationM[i] = innov;
                    const bool okY = kfUpdateAxis(tracks.posY[i], tracks.velY[i], tracks.accelY[i], Px + 9, measPos.y, measVar, gate, innov);
                    const bool okZ = kfUpdateAxis(tracks.posZ[i], tracks.velZ[i], tracks.accelZ[i], Px + 18, measPos.z, measVar, gate, innov);
                    if (!okX || !okY || !okZ) {
                        tracks.residualRejectCount[i] += 1;
                        dropoutStep(true);   // already predicted this step
                        continue;
                    }
                }
                const double pVar = std::max(
                    {tracks.kfCov[i][0], tracks.kfCov[i][9], tracks.kfCov[i][18]});
                const double vVar = std::max(
                    {tracks.kfCov[i][4], tracks.kfCov[i][13], tracks.kfCov[i][22]});
                tracks.positionStdM[i] = std::sqrt(std::max(0.0, pVar));
                tracks.velocityStdMs[i] = std::sqrt(std::max(0.0, vVar));
            } else {
                // Legacy path: raw overwrite + low-passed finite-difference
                // velocity. Byte-identical to the pre-estimator behavior.
                tracks.positionStdM[i] = kPosStdBaseM;
                tracks.velocityStdMs[i] = kVelStdBaseMps;
                if (state != TrackState::None && sameTarget &&
                    tracks.updateCount[i] > 0)
                {
                    const double dtMeas = std::max(1e-9, timeSec - tracks.measTimeSec[i]);
                    const double nvx = (measPos.x - tracks.measPosX[i]) / dtMeas;
                    const double nvy = (measPos.y - tracks.measPosY[i]) / dtMeas;
                    const double nvz = (measPos.z - tracks.measPosZ[i]) / dtMeas;
                    const double alpha = std::clamp(
                        valAt(tracks.velocityBlend, i, 0.08), 0.0, 1.0);
                    tracks.velX[i] = (1.0 - alpha) * tracks.velX[i] + alpha * nvx;
                    tracks.velY[i] = (1.0 - alpha) * tracks.velY[i] + alpha * nvy;
                    tracks.velZ[i] = (1.0 - alpha) * tracks.velZ[i] + alpha * nvz;
                } else if (state == TrackState::None) {
                    tracks.velX[i] = tracks.velY[i] = tracks.velZ[i] = 0.0;
                    tracks.accelAvailable[i] = false;
                }
            }
            const double tau = std::max(1e-9, valAt(tracks.qualityTauSec, i, kQualityTauSec));
            tracks.quality01[i] = std::exp(-tracks.ageSec[i] / tau);
            tracks.measPosX[i] = measPos.x;
            tracks.measPosY[i] = measPos.y;
            tracks.measPosZ[i] = measPos.z;
            tracks.measTimeSec[i] = timeSec;

            // State lifecycle.
            if (state == TrackState::None) {
                state = TrackState::Acquire;
                tracks.updateCount[i] = 1;
                tracks.trackId[i] = newId;
                tracks.accelAvailable[i] = false;
            } else if (!sameTarget) {
                state = TrackState::Acquire;
                tracks.updateCount[i] = 1;
                tracks.trackId[i] = newId;
                tracks.accelAvailable[i] = false;
            } else if (state == TrackState::Coast || state == TrackState::Lost) {
                const bool wasLost = state == TrackState::Lost;
                state = TrackState::Reacquire;
                tracks.updateCount[i] = 1;
                if (wasLost) tracks.accelAvailable[i] = false;
            } else {
                tracks.updateCount[i] += 1;
            }

            // Promotion to Maintain after `confirmations` consistent updates.
            if ((state == TrackState::Acquire || state == TrackState::Reacquire) &&
                tracks.updateCount[i] >=
                    static_cast<std::uint32_t>(std::max(1, tracks.confirmations[i])))
            {
                state = TrackState::Maintain;
                if (useFilter) tracks.accelAvailable[i] = true;
            }

            tracks.posX[i] = useFilter ? tracks.posX[i] : measPos.x;
            tracks.posY[i] = useFilter ? tracks.posY[i] : measPos.y;
            tracks.posZ[i] = useFilter ? tracks.posZ[i] : measPos.z;
        }
    }

} // namespace StrikeEngine::Kernel
