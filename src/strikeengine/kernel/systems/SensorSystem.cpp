#include <strikeengine/kernel/systems/SensorSystem.hpp>
#include <strikeengine/kernel/math/Quaternion.hpp>
#include <strikeengine/models/physics/earth/EarthFrames.hpp>
#include <strikeengine/models/physics/earth/MagneticModel.hpp>
#include <cmath>
#include <chrono>
#include <array>
#include <deque>
#include <numbers>

namespace StrikeEngine::Kernel {

namespace {

bool terrainBlocksRadar(double fromX, double fromY, double fromZ,
                        double toX, double toY, double toZ,
                        const EnvironmentConfig& environment)
{
    if (!environment.globalTerrain && !environment.terrainElevation) return false;
    if (!environment.globalTerrain && environment.earth.useEcefTruth) return false;

    constexpr int kSamples = 12;
    for (int k = 1; k < kSamples; ++k) {
        const double t = static_cast<double>(k) / kSamples;
        const double x = fromX + (toX - fromX) * t;
        const double y = fromY + (toY - fromY) * t;
        const double z = fromZ + (toZ - fromZ) * t;
        double elevation = 0.0;
        bool known = false;
        if (environment.earth.useEcefTruth) {
            const auto geo = Models::ecefToGeodetic({x, y, z});
            if (environment.globalTerrain) {
                const auto sample = environment.globalTerrain->sample(
                    geo.latitudeRad, geo.longitudeRad);
                if (sample.hasElevation()) {
                    elevation = sample.elevationM;
                    known = true;
                }
            }
            if (known && geo.altitudeM < elevation) return true;
        } else {
            if (environment.globalTerrain) {
                const Models::GeodeticCoordinate reference{
                    environment.earth.referenceLatitudeRad,
                    environment.earth.referenceLongitudeRad, 0.0};
                const auto geo = Models::EarthFrames::enuToGeodetic(
                    {x, y, z}, reference);
                const auto sample = environment.globalTerrain->sample(
                    geo.latitudeRad, geo.longitudeRad);
                if (sample.hasElevation()) {
                    elevation = sample.elevationM;
                    known = true;
                }
            } else {
                elevation = environment.terrainElevation(x, y);
                known = true;
            }
            if (known && z < elevation) return true;
        }
    }
    return false;
}

} // namespace

    SensorSystem::SensorSystem() {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        rng = std::mt19937(seed);
    }

    void SensorSystem::setSeed(std::uint32_t seed) {
        // Deterministic stream for reproducible validation runs. Note: the
        // streaming-bias state is a pure function of the RNG, so re-seeding
        // the same scenario reproduces the same biases as well.
        rng.seed(seed);
    }

    double SensorSystem::nextUniform01() {
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        return uniform(rng);
    }

    void SensorSystem::reset() {
        trueAccelBiasX.clear();
        trueAccelBiasY.clear();
        trueAccelBiasZ.clear();
        trueGyroBiasX.clear();
        trueGyroBiasY.clear();
        trueGyroBiasZ.clear();
        lastGpsUpdateTime.clear();
        lastBaroUpdateTime.clear();
        lastMagUpdateTime.clear();
        trueBaroBias.clear();
        gpsLatencyQueue.clear();
        lastRadarScanTime.clear();
        radarLatencyQueue.clear();
    }

    void SensorSystem::resetEntity(std::size_t id) {
        auto clear = [id](std::vector<double>& v) {
            if (v.size() > id) v[id] = 0.0;
        };
        clear(trueAccelBiasX);
        clear(trueAccelBiasY);
        clear(trueAccelBiasZ);
        clear(trueGyroBiasX);
        clear(trueGyroBiasY);
        clear(trueGyroBiasZ);
        clear(trueBaroBias);
        clear(lastGpsUpdateTime);
        clear(lastBaroUpdateTime);
        clear(lastMagUpdateTime);
        if (gpsLatencyQueue.size() > id) gpsLatencyQueue[id].clear();
        clear(lastRadarScanTime);
        if (radarLatencyQueue.size() > id) radarLatencyQueue[id].clear();
    }

    void SensorSystem::ensureCapacity(std::size_t size) {
        if (trueAccelBiasX.size() < size) {
            trueAccelBiasX.resize(size, 0.0);
            trueAccelBiasY.resize(size, 0.0);
            trueAccelBiasZ.resize(size, 0.0);
            trueGyroBiasX.resize(size, 0.0);
            trueGyroBiasY.resize(size, 0.0);
            trueGyroBiasZ.resize(size, 0.0);
        }
        if (lastGpsUpdateTime.size() < size) {
            lastGpsUpdateTime.resize(size, 0.0);
        }
        if (lastBaroUpdateTime.size() < size) {
            lastBaroUpdateTime.resize(size, 0.0);
            trueBaroBias.resize(size, 0.0);
        }
        if (lastMagUpdateTime.size() < size) {
            lastMagUpdateTime.resize(size, 0.0);
        }
        if (gpsLatencyQueue.size() < size) {
            gpsLatencyQueue.resize(size);
        }
        if (lastRadarScanTime.size() < size) {
            lastRadarScanTime.resize(size, 0.0);
        }
        if (radarLatencyQueue.size() < size) {
            radarLatencyQueue.resize(size);
        }
    }

    void SensorSystem::update(
        const PhysicsBlock& physics,
        SensorBlock& sensors,
        const EntityStatusBlock& status,
        double currentTime,
        double dt,
        const EnvironmentConfig& environment)
    {
        std::size_t size = physics.size;
        sensors.size = size;
        ensureCapacity(size);

        // Owner of the slot defaults is SensorBlock::ensureSize. Grow-only, so
        // it cannot shrink arrays already holding state.
        sensors.ensureSize(size);
        sensors.radarMeasurements.clear();

        std::normal_distribution<double> stdNorm(0.0, 1.0);

        for (std::size_t i = 0; i < size; ++i) {
            if (!physics.active[i]) continue;

            // Sensor failure: measurements stop updating entirely (no new
            // IMU/GPS noise is generated; GPS simply never reports fresh data).
            if (i < status.sensorFailed.size() && status.sensorFailed[i]) {
                sensors.gpsUpdated[i] = false;
                if (i < sensors.baroUpdated.size()) sensors.baroUpdated[i] = false;
                if (i < sensors.magUpdated.size()) sensors.magUpdated[i] = false;
                continue;
            }

            // Per-entity GPS refresh scheduling: a sample is produced only when
            // the GPS device is enabled and the per-entity interval has elapsed.
            // A non-positive rate schedules nothing (a 1/0 period is inf, which
            // reads as "never" but only by accident of IEEE arithmetic).
            bool updateGps = false;
            if (sensors.gpsEnabled[i] && sensors.gpsUpdateRateHz[i] > 0.0) {
                const double period = 1.0 / sensors.gpsUpdateRateHz[i];
                if (currentTime - lastGpsUpdateTime[i] >= period) {
                    updateGps = true;
                    lastGpsUpdateTime[i] = currentTime;
                }
            }

            // 1. IMU Specific Force (world-frame acceleration minus gravity).
            std::array<double, 3> gravity{0.0, 0.0, -9.80665};
            if (environment.earth.useEcefTruth) {
                const auto position = Models::ecefToGeodetic({
                    physics.px[i], physics.py[i], physics.pz[i]});
                if (environment.earth.includeJ2Gravity) {
                    gravity = Models::EarthFrames::toVector(
                        Models::j2GravityAccelerationEcef({
                            physics.px[i], physics.py[i], physics.pz[i]}));
                } else if (environment.earth.useWgs84Gravity) {
                    gravity = Models::EarthFrames::ecefNormalGravityAcceleration(
                        position);
                } else {
                    gravity = Models::EarthFrames::toVector(
                        Models::sphericalGravityAccelerationEcef(
                            {physics.px[i], physics.py[i], physics.pz[i]}));
                }
            } else if (environment.earth.includeJ2Gravity) {
                const Models::GeodeticCoordinate reference{
                    environment.earth.referenceLatitudeRad,
                    environment.earth.referenceLongitudeRad,
                    0.0};
                gravity = Models::EarthFrames::localJ2GravityAcceleration(
                    Models::EarthFrames::enuToGeodetic(
                        {physics.px[i], physics.py[i], physics.pz[i]}, reference));
            } else if (environment.earth.useWgs84Gravity) {
                gravity[2] = -Models::normalGravity(
                    environment.earth.referenceLatitudeRad, physics.pz[i]);
            } else if (environment.earth.useSphericalGravity) {
                const Models::GeodeticCoordinate reference{
                    environment.earth.referenceLatitudeRad,
                    environment.earth.referenceLongitudeRad,
                    0.0};
                gravity = Models::EarthFrames::localSphericalGravityAcceleration(
                    Models::EarthFrames::enuToGeodetic(
                        {physics.px[i], physics.py[i], physics.pz[i]}, reference));
            }
            const double fx = physics.ax[i] - gravity[0];
            const double fy = physics.ay[i] - gravity[1];
            const double fz = physics.az[i] - gravity[2];

            // Rotate Specific Force to Body Frame. Uses the shared
            // Quaternion.hpp rotation (v' = q^-1 v q). The historical local
            // helper had a sign error on the cross-product terms (it applied
            // q^-1 twice), which inverted the measured specific force for any
            // non-identity attitude and made the autopilot's lateral channels
            // positive-feedback (growing oscillation -> tumble).
            double bfx, bfy, bfz;
            quatRotateToBody(physics.qw[i], physics.qx[i], physics.qy[i], physics.qz[i],
                             fx, fy, fz, bfx, bfy, bfz);

            // True Gyro is already body frame (assumed in PhysicsBlock wx,wy,wz)
            // Wait, PhysicsBlock wx, wy, wz are body frame angular rates? Yes.
            double bwx = physics.wx[i];
            double bwy = physics.wy[i];
            double bwz = physics.wz[i];

            // Earth-rate gyro modeling (ECEF truth, opt-in): a physical gyro
            // measures the INERTIAL body rate, which includes the earth
            // rotation vector resolved into the body frame,
            //     omega_ie^b = C_e^b * (0, 0, Omega_ie)
            // via the body->ECEF attitude quaternion.
            if (environment.earth.useEcefTruth && environment.earth.includeEarthRateGyro) {
                double ewx, ewy, ewz;
                quatRotateToBody(physics.qw[i], physics.qx[i], physics.qy[i], physics.qz[i],
                                 0.0, 0.0, Models::EarthModel::earthRotationRateRadPerSec,
                                 ewx, ewy, ewz);
                bwx += ewx;
                bwy += ewy;
                bwz += ewz;
            }

            // IMU lever-arm correction: the IMU is mounted at a fixed
            // body-frame offset l from the centre of mass, so it senses the
            // CM specific force plus the rigid-body terms
            //     f_imu = f_cm + alpha x l + omega x (omega x l)
            // where alpha is the body angular acceleration and omega the body
            // angular rate. Zero lever arm yields a zero correction.
            const double lx = sensors.imuLeverArmX[i];
            const double ly = sensors.imuLeverArmY[i];
            const double lz = sensors.imuLeverArmZ[i];
            if (lx != 0.0 || ly != 0.0 || lz != 0.0) {
                // term_a = alpha x l
                const double termAx = physics.alphay[i] * lz - physics.alphaz[i] * ly;
                const double termAy = physics.alphaz[i] * lx - physics.alphax[i] * lz;
                const double termAz = physics.alphax[i] * ly - physics.alphay[i] * lx;
                // c = omega x l; term_c = omega x (omega x l)
                const double cx = bwy * lz - bwz * ly;
                const double cy = bwz * lx - bwx * lz;
                const double cz = bwx * ly - bwy * lx;
                const double termCx = bwy * cz - bwz * cy;
                const double termCy = bwz * cx - bwx * cz;
                const double termCz = bwx * cy - bwy * cx;
                bfx += termAx + termCx;
                bfy += termAy + termCy;
                bfz += termAz + termCz;
            }

            // The IMU is sampled every kernel step, so the configured sigmas are
            // per-sample values at the 10 ms reference rate. Scale white noise by
            // sqrt(dt/dtRef) and random-walk steps by sqrt(dtRef*dt) so the
            // modeled sensor keeps a fixed noise density at any step size while
            // the 10 ms reference behavior is unchanged.
            constexpr double kNoiseRefDt = 0.01;
            const double noiseScale = (dt > 0.0) ? std::sqrt(dt / kNoiseRefDt) : 1.0;
            const double biasStepScale = (dt > 0.0) ? std::sqrt(kNoiseRefDt * dt) : 0.0;

            // Random walk biases + measurement output. A disabled IMU freezes
            // its last measurements and stops its streaming-bias drift.
            if (sensors.imuEnabled[i]) {
                // Random walk biases (slow drift) - very simple model
                trueAccelBiasX[i] += stdNorm(rng) * sensors.accelBiasStdDev[i] * biasStepScale;
                trueAccelBiasY[i] += stdNorm(rng) * sensors.accelBiasStdDev[i] * biasStepScale;
                trueAccelBiasZ[i] += stdNorm(rng) * sensors.accelBiasStdDev[i] * biasStepScale;
                trueGyroBiasX[i]  += stdNorm(rng) * sensors.gyroBiasStdDev[i] * biasStepScale;
                trueGyroBiasY[i]  += stdNorm(rng) * sensors.gyroBiasStdDev[i] * biasStepScale;
                trueGyroBiasZ[i]  += stdNorm(rng) * sensors.gyroBiasStdDev[i] * biasStepScale;

                // Add noise and bias
                sensors.accelX[i] = bfx + trueAccelBiasX[i] + stdNorm(rng) * sensors.accelNoiseStdDev[i] * noiseScale;
                sensors.accelY[i] = bfy + trueAccelBiasY[i] + stdNorm(rng) * sensors.accelNoiseStdDev[i] * noiseScale;
                sensors.accelZ[i] = bfz + trueAccelBiasZ[i] + stdNorm(rng) * sensors.accelNoiseStdDev[i] * noiseScale;

                sensors.gyroX[i] = bwx + trueGyroBiasX[i] + stdNorm(rng) * sensors.gyroNoiseStdDev[i] * noiseScale;
                sensors.gyroY[i] = bwy + trueGyroBiasY[i] + stdNorm(rng) * sensors.gyroNoiseStdDev[i] * noiseScale;
                sensors.gyroZ[i] = bwz + trueGyroBiasZ[i] + stdNorm(rng) * sensors.gyroNoiseStdDev[i] * noiseScale;
            }

            // 2. GPS Update (antenna position = CM + body-frame lever arm).
            // Lever arm is rotated by the TRUE attitude (same convention as
            // the IMU lever arm); a zero lever reproduces the legacy CM fix.
            double antX = physics.px[i], antY = physics.py[i], antZ = physics.pz[i];
            {
                const double lx = sensors.gpsLeverArmX[i];
                const double ly = sensors.gpsLeverArmY[i];
                const double lz = sensors.gpsLeverArmZ[i];
                if (lx != 0.0 || ly != 0.0 || lz != 0.0) {
                    double wox, woy, woz;
                    quatRotateToWorld(physics.qw[i], physics.qx[i], physics.qy[i], physics.qz[i],
                                      lx, ly, lz, wox, woy, woz);
                    antX += wox; antY += woy; antZ += woz;
                }
            }
            const double latency = std::max(0.0, sensors.gpsLatencySec[i]);
            sensors.gpsUpdated[i] = false;
            if (updateGps) {
                // Noise is drawn at SAMPLE time (stream order identical to
                // legacy when latency is 0); delivery may be delayed below.
                DelayedGpsSample sample;
                sample.timeSec = currentTime;
                sample.px = antX + stdNorm(rng) * sensors.gpsPosNoiseStdDev[i];
                sample.py = antY + stdNorm(rng) * sensors.gpsPosNoiseStdDev[i];
                sample.pz = antZ + stdNorm(rng) * sensors.gpsPosNoiseStdDev[i];
                sample.vx = physics.vx[i] + stdNorm(rng) * sensors.gpsVelNoiseStdDev[i];
                sample.vy = physics.vy[i] + stdNorm(rng) * sensors.gpsVelNoiseStdDev[i];
                sample.vz = physics.vz[i] + stdNorm(rng) * sensors.gpsVelNoiseStdDev[i];
                if (latency <= 0.0) {
                    sensors.gpsPosX[i] = sample.px; sensors.gpsPosY[i] = sample.py; sensors.gpsPosZ[i] = sample.pz;
                    sensors.gpsVelX[i] = sample.vx; sensors.gpsVelY[i] = sample.vy; sensors.gpsVelZ[i] = sample.vz;
                    sensors.gpsUpdated[i] = true;
                } else {
                    gpsLatencyQueue[i].push_back(sample);
                }
            }
            if (latency > 0.0 && !gpsLatencyQueue[i].empty()) {
                // Drop superseded samples; publish the newest due one.
                while (gpsLatencyQueue[i].size() > 1 &&
                       currentTime - gpsLatencyQueue[i][1].timeSec >= latency - 1e-12) {
                    gpsLatencyQueue[i].pop_front();
                }
                const DelayedGpsSample& due = gpsLatencyQueue[i].front();
                if (currentTime - due.timeSec >= latency - 1e-12) {
                    sensors.gpsPosX[i] = due.px; sensors.gpsPosY[i] = due.py; sensors.gpsPosZ[i] = due.pz;
                    sensors.gpsVelX[i] = due.vx; sensors.gpsVelY[i] = due.vy; sensors.gpsVelZ[i] = due.vz;
                    sensors.gpsUpdated[i] = true;
                    gpsLatencyQueue[i].pop_front();
                }
            }

            // 3. Barometer altitude (opt-in). Truth altitude is geodetic
            // under ECEF truth, datum-relative Z otherwise; bias random-walks
            // like the IMU biases. No draws happen while disabled.
            sensors.baroUpdated[i] = false;
            if (sensors.baroEnabled[i]) {
                const double baroPeriod = 1.0 / std::max(sensors.baroUpdateRateHz[i], 1e-9);
                if (currentTime - lastBaroUpdateTime[i] >= baroPeriod) {
                    lastBaroUpdateTime[i] = currentTime;
                    double truthAlt = physics.pz[i];
                    if (environment.earth.useEcefTruth) {
                        truthAlt = Models::ecefToGeodetic({
                            physics.px[i], physics.py[i], physics.pz[i]}).altitudeM;
                    }
                    trueBaroBias[i] += stdNorm(rng) * sensors.baroBiasStdDev[i] * dt;
                    sensors.baroAlt[i] = truthAlt + trueBaroBias[i] +
                        stdNorm(rng) * sensors.baroNoiseStdDev[i];
                    sensors.baroUpdated[i] = true;
                }
            }

            // 4. Magnetometer (opt-in). Tilted-dipole world field at the true
            // position, rotated to the body frame by the TRUE attitude, plus
            // white noise per axis. No draws happen while disabled.
            sensors.magUpdated[i] = false;
            if (sensors.magEnabled[i]) {
                const double magPeriod = 1.0 / std::max(sensors.magUpdateRateHz[i], 1e-9);
                if (currentTime - lastMagUpdateTime[i] >= magPeriod) {
                    lastMagUpdateTime[i] = currentTime;
                    Models::EcefCoordinate ecefPos{
                        physics.px[i], physics.py[i], physics.pz[i]};
                    if (!environment.earth.useEcefTruth) {
                        const Models::GeodeticCoordinate reference{
                            environment.earth.referenceLatitudeRad,
                            environment.earth.referenceLongitudeRad,
                            0.0};
                        ecefPos = Models::geodeticToEcef(
                            Models::EarthFrames::enuToGeodetic(
                                {physics.px[i], physics.py[i], physics.pz[i]}, reference));
                    }
                    const auto field = Models::dipoleMagneticFieldEcef(
                        ecefPos.x, ecefPos.y, ecefPos.z);
                    double bbx, bby, bbz;
                    quatRotateToBody(physics.qw[i], physics.qx[i], physics.qy[i], physics.qz[i],
                                     field[0], field[1], field[2], bbx, bby, bbz);
                    sensors.magX[i] = bbx + stdNorm(rng) * sensors.magNoiseStdDev[i];
                    sensors.magY[i] = bby + stdNorm(rng) * sensors.magNoiseStdDev[i];
                    sensors.magZ[i] = bbz + stdNorm(rng) * sensors.magNoiseStdDev[i];
                    sensors.magUpdated[i] = true;
                }
            }
        }

        // Active radar scan. This is deliberately a separate opt-in path from
        // the IMU/GPS loop above: legacy scenarios have no radar measurements,
        // and therefore retain their exact sensor stream and track behavior.
        for (std::size_t source = 0; source < size; ++source) {
            if (!physics.active[source] || !sensors.radarEnabled[source] ||
                (source < status.sensorFailed.size() && status.sensorFailed[source])) {
                continue;
            }
            const double scanRate = sensors.antennaScanRateHz[source];
            if (!(scanRate > 0.0)) continue;
            const double scanPeriod = 1.0 / scanRate;
            if (currentTime - lastRadarScanTime[source] + 1e-12 < scanPeriod) continue;
            lastRadarScanTime[source] = currentTime;

            double antennaOffsetX = 0.0;
            double antennaOffsetY = 0.0;
            double antennaOffsetZ = 0.0;
            quatRotateToWorld(
                physics.qw[source], physics.qx[source], physics.qy[source], physics.qz[source],
                sensors.antennaPositionX[source], sensors.antennaPositionY[source],
                sensors.antennaPositionZ[source], antennaOffsetX, antennaOffsetY,
                antennaOffsetZ);
            const double sensorX = physics.px[source] + antennaOffsetX;
            const double sensorY = physics.py[source] + antennaOffsetY;
            const double sensorZ = physics.pz[source] + antennaOffsetZ;
            const double maxRange = sensors.radarMaxRangeM[source];
            const double fov = std::clamp(
                sensors.radarFieldOfViewHalfAngleRad[source], 0.0,
                std::numbers::pi);

            for (std::size_t target = 0; target < size; ++target) {
                if (target == source || !physics.active[target] ||
                    !status.isAlive[target] ||
                    status.allegiance[source] == status.allegiance[target]) {
                    continue;
                }

                const double dx = physics.px[target] - sensorX;
                const double dy = physics.py[target] - sensorY;
                const double dz = physics.pz[target] - sensorZ;
                const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (!(range > 1e-6) || (maxRange > 0.0 && range > maxRange)) continue;

                double bodyX = 0.0;
                double bodyY = 0.0;
                double bodyZ = 0.0;
                quatRotateToBody(
                    physics.qw[source], physics.qx[source], physics.qy[source], physics.qz[source],
                    dx / range, dy / range, dz / range, bodyX, bodyY, bodyZ);
                const double offBoresight = std::acos(std::clamp(bodyX, -1.0, 1.0));
                if (offBoresight > fov) continue;
                if (sensors.radarTerrainMaskingEnabled[source] &&
                    terrainBlocksRadar(sensorX, sensorY, sensorZ,
                                       physics.px[target], physics.py[target],
                                       physics.pz[target], environment)) {
                    continue;
                }

                RadarMeasurement measurement;
                measurement.sourceEntityId = static_cast<int>(source);
                measurement.targetEntityId = static_cast<int>(target);
                measurement.timestampSec = currentTime;
                measurement.rangeM = range;
                measurement.rangeRateMps =
                    (dx * (physics.vx[target] - physics.vx[source]) +
                     dy * (physics.vy[target] - physics.vy[source]) +
                     dz * (physics.vz[target] - physics.vz[source])) / range;
                measurement.azimuthRad = std::atan2(bodyY, bodyX);
                measurement.elevationRad = std::asin(std::clamp(-bodyZ, -1.0, 1.0));

                const double rangeNoise = sensors.radarRangeNoiseStdDevM[source];
                const double rangeRateNoise = sensors.radarRangeRateNoiseStdDevMps[source];
                const double angleNoise = sensors.radarAngleNoiseStdDevRad[source];
                if (rangeNoise > 0.0) {
                    measurement.rangeM += stdNorm(rng) * rangeNoise;
                    measurement.rangeM = std::max(1.0, measurement.rangeM);
                }
                if (rangeRateNoise > 0.0) {
                    measurement.rangeRateMps += stdNorm(rng) * rangeRateNoise;
                }
                if (angleNoise > 0.0) {
                    measurement.azimuthRad += stdNorm(rng) * angleNoise;
                    measurement.elevationRad += stdNorm(rng) * angleNoise;
                }
                measurement.signalStrengthDb = 20.0 * std::log10(
                    std::max(1.0, (maxRange > 0.0 ? maxRange : 100000.0) / range));

                const double latency = std::max(
                    0.0, sensors.radarMeasurementLatencySec[source]);
                if (latency <= 0.0) {
                    sensors.radarMeasurements.push_back(measurement);
                } else {
                    radarLatencyQueue[source].push_back(measurement);
                }
            }
        }

        for (std::size_t source = 0; source < radarLatencyQueue.size(); ++source) {
            const double latency = std::max(
                0.0, sensors.radarMeasurementLatencySec[source]);
            auto& queue = radarLatencyQueue[source];
            while (!queue.empty() &&
                   currentTime - queue.front().timestampSec >= latency - 1e-12) {
                sensors.radarMeasurements.push_back(queue.front());
                queue.pop_front();
            }
        }
    }

} // namespace StrikeEngine::Kernel
