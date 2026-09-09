#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/backend/BackendFactory.hpp>
#include <strikeengine/kernel/backend/CPUBackend.hpp>
#include <strikeengine/kernel/profiles/AeroProfileDatabase.hpp>
#include <strikeengine/kernel/profiles/MotorProfileDatabase.hpp>
#include <strikeengine/kernel/profiles/SeekerProfileDatabase.hpp>
#include <strikeengine/kernel/profiles/SensorProfileDatabase.hpp>
#include <strikeengine/kernel/profiles/GuidanceProfileDatabase.hpp>
#include <strikeengine/kernel/profiles/WarheadProfileDatabase.hpp>
#include <strikeengine/models/physics/atmosphere/ISA1976.hpp>
#include <strikeengine/models/physics/aerodynamics/AeroModel.hpp>
#include <strikeengine/models/physics/propulsion/PropulsionModel.hpp>
#include <strikeengine/models/warhead/WarheadEffects.hpp>
#include <stdexcept>
#include <algorithm>
#include <cmath>

namespace StrikeEngine::Kernel {

    SimulationKernel::SimulationKernel(BackendType backendType, IntegratorType integratorType) {
        if (backendType == BackendType::Vulkan) {
            auto factory = getPhysicsBackendFactory(static_cast<int>(BackendType::Vulkan));
            if (!factory) {
                throw std::runtime_error(
                    "Vulkan backend requested but not linked; "
                    "build with -DSTRIKEENGINE_WITH_VULKAN=ON and link strikeengine_vulkan");
            }
            backend = factory();
        } else {
            // Truth models: atmosphere + aero are shared (state-less, take
            // per-entity coefficients from the PhysicsBlock). Propulsion is
            // per-entity (W1) and registered through createVehicle(config).
            auto atmosphere = std::make_shared<Models::ISA1976>();
            auto aero = std::make_shared<Models::BasicAeroModel>();

            auto integrator = IntegratorFactory::create(integratorType);

            backend = std::make_unique<CPUBackend>(
                std::move(integrator),
                atmosphere,
                aero,
                environment
            );
        }
        
        backend->initialize(physicsBlock, controlBlock);
    }

    SimulationKernel::~SimulationKernel() = default;

    void SimulationKernel::initialize() {
        reset();
    }

    void SimulationKernel::setRandomSeed(std::uint32_t seed) {
        sensorSystem.setSeed(seed);
    }

    void SimulationKernel::setEnvironment(const EnvironmentConfig& environmentConfig) {
        environment = environmentConfig;
        backend->setEnvironment(environment);
    }

    void SimulationKernel::setThreadCount(std::size_t threads) {
        if (backend) {
            backend->setThreadCount(threads);
        }
    }

    std::size_t SimulationKernel::threadCount() const {
        return backend ? backend->threadCount() : 1;
    }

    void SimulationKernel::reset() {
        physicsBlock = PhysicsBlock();
        controlBlock = ControlBlock();
        guidanceBlock = GuidanceBlock();
        statusBlock = EntityStatusBlock();
        sensorBlock = SensorBlock();
        navigationBlock = NavigationBlock();
        seekerBlock = SeekerBlock();
        seekerSystem.reset();
        sensorSystem.reset();
        freeList.clear();
        stagePlans.clear();
        warheads.clear();
        time.reset();
    }

    PhysicsId SimulationKernel::createVehicle(const VehicleInitState& init) {
        return createVehicle(init, VehicleConfig{});
    }

    PhysicsId SimulationKernel::createVehicle(const VehicleInitState& init, const VehicleConfig& config) {
        // Profile-id resolution: a non-empty profile id replaces the inline
        // sub-config for that subsystem (profile is authoritative, same
        // precedent as designRef overriding inline vehicleConfig). Failing to
        // load a referenced profile is fatal (fail fast).
        VehicleConfig resolved = config;
        if (!config.aeroProfileId.empty()) {
            AeroProfileDatabase aeroDb;
            if (!aeroDb.loadProfile(config.aeroProfileId)) {
                throw std::runtime_error("AeroProfileDatabase could not load profile '" +
                                         config.aeroProfileId + "'");
            }
            resolved.aero = aeroDb.aero();
        }
        if (!config.motorProfileId.empty()) {
            MotorProfileDatabase motorDb;
            if (!motorDb.loadProfile(config.motorProfileId)) {
                throw std::runtime_error("MotorProfileDatabase could not load profile '" +
                                         config.motorProfileId + "'");
            }
            resolved.propulsion = motorDb.propulsion();
        }
        if (!config.seekerProfileId.empty()) {
            SeekerProfileDatabase seekerDb;
            if (!seekerDb.loadProfile(config.seekerProfileId)) {
                throw std::runtime_error("SeekerProfileDatabase could not load profile '" +
                                         config.seekerProfileId + "'");
            }
            resolved.seeker = seekerDb.seeker();
        }
        if (!config.sensorProfileId.empty()) {
            SensorProfileDatabase sensorDb;
            if (!sensorDb.loadProfile(config.sensorProfileId)) {
                throw std::runtime_error("SensorProfileDatabase could not load profile '" +
                                         config.sensorProfileId + "'");
            }
            resolved.sensor = sensorDb.sensor();
        }
        if (!config.guidanceProfileId.empty()) {
            GuidanceProfileDatabase guidanceDb;
            if (!guidanceDb.loadProfile(config.guidanceProfileId)) {
                throw std::runtime_error("GuidanceProfileDatabase could not load profile '" +
                                         config.guidanceProfileId + "'");
            }
            resolved.guidanceAutopilot = guidanceDb.guidanceAutopilot();
        }
        if (!config.warheadProfileId.empty()) {
            WarheadProfileDatabase warheadDb;
            if (!warheadDb.loadProfile(config.warheadProfileId)) {
                throw std::runtime_error("WarheadProfileDatabase could not load profile '" +
                                         config.warheadProfileId + "'");
            }
            resolved.warhead = warheadDb.warhead();
        }

        std::string propulsionError;
        if (!validatePropulsionConfig(resolved.propulsion, &propulsionError)) {
            throw std::invalid_argument("Invalid propulsion configuration: " + propulsionError);
        }

        PhysicsId id;

        if (!freeList.empty()) {
            id = freeList.back();
            freeList.pop_back();
        } else {
            id = physicsBlock.size++;
            // Keep the SoA block size fields in sync with the physics entity
            // count. SeekerSystem reads seeker.size/status.size directly, so
            // an unsynced size silently disabled kernel-path seekers.
            statusBlock.size = physicsBlock.size;
            sensorBlock.size = physicsBlock.size;
            navigationBlock.size = physicsBlock.size;
            seekerBlock.size = physicsBlock.size;
            trackBlock.size = physicsBlock.size;   // W39 track manager reads tracks.size
            
            // Resize arrays
            physicsBlock.px.push_back(0); physicsBlock.py.push_back(0); physicsBlock.pz.push_back(0);
            physicsBlock.vx.push_back(0); physicsBlock.vy.push_back(0); physicsBlock.vz.push_back(0);
            physicsBlock.ax.push_back(0); physicsBlock.ay.push_back(0); physicsBlock.az.push_back(0);
            physicsBlock.qw.push_back(1); physicsBlock.qx.push_back(0); physicsBlock.qy.push_back(0); physicsBlock.qz.push_back(0);
            physicsBlock.wx.push_back(0); physicsBlock.wy.push_back(0); physicsBlock.wz.push_back(0);
            physicsBlock.alphax.push_back(0); physicsBlock.alphay.push_back(0); physicsBlock.alphaz.push_back(0);
            physicsBlock.Ixx.push_back(1.0); physicsBlock.Iyy.push_back(10.0); physicsBlock.Izz.push_back(10.0);
            physicsBlock.Ixy.push_back(0.0); physicsBlock.Ixz.push_back(0.0); physicsBlock.Iyz.push_back(0.0);
            physicsBlock.mass.push_back(1.0);
            physicsBlock.massDry.push_back(1.0);
            physicsBlock.referenceArea.push_back(0.1);
            physicsBlock.referenceLength.push_back(1.0);
            physicsBlock.cd.push_back(0.3);
            physicsBlock.clAlpha.push_back(0.0);
            physicsBlock.clFin.push_back(0.0);
            physicsBlock.clMax.push_back(2.0);
            physicsBlock.aeroTables.push_back(nullptr);
            physicsBlock.fins.push_back(nullptr);
            physicsBlock.finSets.push_back({});
            physicsBlock.propulsionId.push_back(-1);
            physicsBlock.ignitionTime.push_back(0.0);
            physicsBlock.stageIndex.push_back(-1);
            physicsBlock.stageCount.push_back(0);
            physicsBlock.finPitch.push_back(0.0); physicsBlock.finYaw.push_back(0.0); physicsBlock.finRoll.push_back(0.0);
            physicsBlock.maxDeflectionRad.push_back(0.43);
            physicsBlock.servoTimeConstantSec.push_back(0.02);
            physicsBlock.maxServoRateRadPerSec.push_back(5.24);
            physicsBlock.stageMinMass.push_back(0.0);
            physicsBlock.gimbalPitch.push_back(0.0); physicsBlock.gimbalYaw.push_back(0.0);
            physicsBlock.maxGimbalPitchRad.push_back(0.0); physicsBlock.maxGimbalYawRad.push_back(0.0);
            physicsBlock.gimbalTimeConstantSec.push_back(0.02);
            physicsBlock.maxGimbalRateRadPerSec.push_back(0.0);
            physicsBlock.enginePositionX.push_back(0.0);
            physicsBlock.enginePositionY.push_back(0.0);
            physicsBlock.enginePositionZ.push_back(0.0);
            physicsBlock.mach.push_back(0.0);
            physicsBlock.dynamicPressure.push_back(0.0);
            physicsBlock.airDensity.push_back(0.0);
            physicsBlock.localSpeedOfSound.push_back(0.0);
            physicsBlock.active.push_back(true);
            physicsBlock.motorFailed.push_back(false);
            physicsBlock.engineFailed.push_back(false);
            physicsBlock.tankFailed.push_back(false);
            physicsBlock.actuatorFailed.push_back(false);

            stagePlans.push_back(StagePlan{});
            warheads.push_back(WarheadState{});

            controlBlock.thrustCommand.push_back(0);
            controlBlock.pitchCommand.push_back(0);
            controlBlock.yawCommand.push_back(0);
            controlBlock.rollCommand.push_back(0);
            controlBlock.thrustVectorPitchCommand.push_back(0);
            controlBlock.thrustVectorYawCommand.push_back(0);
            controlBlock.kAccelP.push_back(0.030);
            controlBlock.kRateP.push_back(1.000);
            controlBlock.kAlphaP.push_back(0.200);
            controlBlock.kRollP.push_back(0.10);
            controlBlock.kRollD.push_back(0.05);
            controlBlock.maxDeflectionRad.push_back(0.43);
            controlBlock.gainSchedulingEnabled.push_back(false);
            controlBlock.refDynamicPressurePa.push_back(50000.0);
            controlBlock.minDynamicPressurePa.push_back(2000.0);
            controlBlock.maxDynamicPressurePa.push_back(300000.0);
            controlBlock.pitchSaturated.push_back(false);
            controlBlock.yawSaturated.push_back(false);
            controlBlock.rollSaturated.push_back(false);

            guidanceBlock.mode.push_back(GuidanceMode::None);
            guidanceBlock.targetX.push_back(0); guidanceBlock.targetY.push_back(0); guidanceBlock.targetZ.push_back(0);
            guidanceBlock.targetVx.push_back(0); guidanceBlock.targetVy.push_back(0); guidanceBlock.targetVz.push_back(0);
            guidanceBlock.targetAccelX.push_back(0); guidanceBlock.targetAccelY.push_back(0); guidanceBlock.targetAccelZ.push_back(0);
            guidanceBlock.targetAccelAvailable.push_back(false);
            guidanceBlock.commandedAccelX.push_back(0); guidanceBlock.commandedAccelY.push_back(0); guidanceBlock.commandedAccelZ.push_back(0);
            guidanceBlock.maxAccel.push_back(0.0);
            guidanceBlock.navigationConstant.push_back(3.5);
            guidanceBlock.waypointGain.push_back(20.0);
            guidanceBlock.handoffBlendTimeSec.push_back(0.0);
            guidanceBlock.lockLossRetentionSec.push_back(0.0);
            guidanceBlock.apnFeedforwardEnabled.push_back(false);
            guidanceBlock.gravityCompensationEnabled.push_back(false);
            guidanceBlock.phase.push_back(GuidancePhase::None);
            guidanceBlock.law.push_back(GuidanceLaw::None);
            guidanceBlock.trackId.push_back(-1);
            guidanceBlock.trackAgeSec.push_back(0.0);
            guidanceBlock.handoffWeight.push_back(0.0);
            guidanceBlock.lockLossCount.push_back(0);
            guidanceBlock.rawAccelX.push_back(0); guidanceBlock.rawAccelY.push_back(0); guidanceBlock.rawAccelZ.push_back(0);
            guidanceBlock.limitedByMaxAccel.push_back(false);
            guidanceBlock.lawInvalid.push_back(false);
            guidanceBlock.nonClosing.push_back(false);
            guidanceBlock.tgoSec.push_back(0.0);
            guidanceBlock.retainedAccelX.push_back(0);
            guidanceBlock.retainedAccelY.push_back(0);
            guidanceBlock.retainedAccelZ.push_back(0);
            guidanceBlock.trajectoryMinSpeedMps.push_back(30.0);
            guidanceBlock.trajectoryFeasibilityAccelFactor.push_back(0.95);
            guidanceBlock.datalinkSourceId.push_back(-1);
            guidanceBlock.datalinkTargetId.push_back(-1);
            guidanceBlock.predictedInterceptX.push_back(0);
            guidanceBlock.predictedInterceptY.push_back(0);
            guidanceBlock.predictedInterceptZ.push_back(0);
            guidanceBlock.predictedTgoSec.push_back(0.0);
            guidanceBlock.trajectoryRequiredAccel.push_back(0.0);
            guidanceBlock.trajectoryAimSource.push_back(GuidanceAimSource::None);
            guidanceBlock.trajectoryFeasible.push_back(false);
            guidanceBlock.trajectoryReason.push_back(TrajectoryReason::None);

            statusBlock.type.push_back(EntityType::Missile);
            statusBlock.allegiance.push_back(Allegiance::Friendly);
            statusBlock.health.push_back(100.0);
            statusBlock.isAlive.push_back(true);
            statusBlock.motorFailed.push_back(false);
            statusBlock.engineFailed.push_back(false);
            statusBlock.tankFailed.push_back(false);
            statusBlock.actuatorFailed.push_back(false);
            statusBlock.sensorFailed.push_back(false);
            statusBlock.commsFailed.push_back(false);
            statusBlock.rcsProfileId.push_back("");
            statusBlock.irProfileId.push_back("");
            statusBlock.emitterEirpW.push_back(0.0);

            sensorBlock.accelNoiseStdDev.push_back(0.1);
            sensorBlock.accelBiasStdDev.push_back(0.01);
            sensorBlock.gyroNoiseStdDev.push_back(0.01);
            sensorBlock.gyroBiasStdDev.push_back(0.001);
            sensorBlock.gpsPosNoiseStdDev.push_back(5.0);
            sensorBlock.gpsVelNoiseStdDev.push_back(0.5);
            sensorBlock.gpsInnovationGateSigma.push_back(5.0);
            sensorBlock.imuLeverArmX.push_back(0.0);
            sensorBlock.imuLeverArmY.push_back(0.0);
            sensorBlock.imuLeverArmZ.push_back(0.0);
            sensorBlock.imuEnabled.push_back(true);
            sensorBlock.gpsEnabled.push_back(true);
            sensorBlock.gpsUpdateRateHz.push_back(1.0);

            seekerBlock.type.push_back(SeekerType::None);
            seekerBlock.transmitterPowerW.push_back(1000.0);
            seekerBlock.antennaGainDb.push_back(30.0);
            seekerBlock.wavelengthM.push_back(0.03); // X-band
            seekerBlock.noiseFloorW.push_back(1e-12);
            seekerBlock.snrThresholdDb.push_back(13.0);
            seekerBlock.sensitivityW.push_back(1e-9);
            seekerBlock.wavelengthBand.push_back(0);
            seekerBlock.irExtinctionPerM.push_back(1e-4);
            seekerBlock.illuminatorPx.push_back(0.0);
            seekerBlock.illuminatorPy.push_back(0.0);
            seekerBlock.illuminatorPz.push_back(0.0);
            seekerBlock.illuminatorPowerW.push_back(5.0e5);
            seekerBlock.illuminatorGainDb.push_back(38.0);
            seekerBlock.illuminatorWavelengthM.push_back(0.03);
            seekerBlock.fieldOfViewHalfAngleRad.push_back(1.0471975512); // 60 deg
            seekerBlock.gimbalAzimuthLimitRad.push_back(1.0471975512);
            seekerBlock.gimbalElevationLimitRad.push_back(1.0471975512);
            seekerBlock.lockHysteresisDb.push_back(3.0);
            seekerBlock.lockDropoutTimeSec.push_back(0.10);
            seekerBlock.measurementLatencySec.push_back(0.0);
            seekerBlock.isLocked.push_back(false);
            seekerBlock.lockedTargetId.push_back(0);
            seekerBlock.targetRange.push_back(0);
            seekerBlock.targetRangeRate.push_back(0);
            seekerBlock.targetAzimuth.push_back(0);
            seekerBlock.targetElevation.push_back(0);
            seekerBlock.targetAzimuthRate.push_back(0);
            seekerBlock.targetElevationRate.push_back(0);
            seekerBlock.previousAzimuth.push_back(0);
            seekerBlock.previousElevation.push_back(0);
            seekerBlock.lockLostTimeSec.push_back(0);
            seekerBlock.hasPreviousLos.push_back(false);

            // W39 persistent track state (config defaults; reset in the common path)
            trackBlock.confirmations.push_back(3);
            trackBlock.coastTimeoutSec.push_back(0.5);
            trackBlock.lossTimeoutSec.push_back(2.0);
            trackBlock.state.push_back(TrackState::None);
            trackBlock.trackId.push_back(-1);
            trackBlock.posX.push_back(0); trackBlock.posY.push_back(0); trackBlock.posZ.push_back(0);
            trackBlock.velX.push_back(0); trackBlock.velY.push_back(0); trackBlock.velZ.push_back(0);
            trackBlock.accelX.push_back(0); trackBlock.accelY.push_back(0); trackBlock.accelZ.push_back(0);
            trackBlock.accelAvailable.push_back(false);
            trackBlock.timestampSec.push_back(0.0);
            trackBlock.ageSec.push_back(0.0);
            trackBlock.positionStdM.push_back(5.0);
            trackBlock.velocityStdMs.push_back(25.0);
            trackBlock.quality01.push_back(0.0);
            trackBlock.updateCount.push_back(0);
            trackBlock.dropoutCount.push_back(0);
            trackBlock.measPosX.push_back(0); trackBlock.measPosY.push_back(0); trackBlock.measPosZ.push_back(0);
            trackBlock.measTimeSec.push_back(0.0);
        }

        // Per-entity defaults apply to BOTH fresh and reused slots so a
        // freed slot cannot leak the removed entity's stale state.
        controlBlock.thrustCommand[id] = 0;
        controlBlock.pitchCommand[id] = 0;
        controlBlock.yawCommand[id] = 0;
        controlBlock.rollCommand[id] = 0;
        controlBlock.thrustVectorPitchCommand[id] = 0;
        controlBlock.thrustVectorYawCommand[id] = 0;
        controlBlock.kAccelP[id] = resolved.guidanceAutopilot.kAccelP;
        controlBlock.kRateP[id] = resolved.guidanceAutopilot.kRateP;
        controlBlock.kAlphaP[id] = resolved.guidanceAutopilot.kAlphaP;
        controlBlock.kRollP[id] = resolved.guidanceAutopilot.kRollP;
        controlBlock.kRollD[id] = resolved.guidanceAutopilot.kRollD;
        controlBlock.maxDeflectionRad[id] = resolved.guidanceAutopilot.maxDeflectionRad;
        controlBlock.gainSchedulingEnabled[id] = resolved.guidanceAutopilot.gainSchedulingEnabled;
        controlBlock.refDynamicPressurePa[id] = resolved.guidanceAutopilot.refDynamicPressurePa;
        controlBlock.minDynamicPressurePa[id] = resolved.guidanceAutopilot.minDynamicPressurePa;
        controlBlock.maxDynamicPressurePa[id] = resolved.guidanceAutopilot.maxDynamicPressurePa;
        physicsBlock.maxDeflectionRad[id] = resolved.guidanceAutopilot.maxDeflectionRad;
        physicsBlock.servoTimeConstantSec[id] = resolved.guidanceAutopilot.servoTimeConstantSec;
        physicsBlock.maxServoRateRadPerSec[id] = resolved.guidanceAutopilot.maxServoRateRadPerSec;

        guidanceBlock.mode[id] = GuidanceMode::None;
        guidanceBlock.targetX[id] = 0; guidanceBlock.targetY[id] = 0; guidanceBlock.targetZ[id] = 0;
        guidanceBlock.targetVx[id] = 0; guidanceBlock.targetVy[id] = 0; guidanceBlock.targetVz[id] = 0;
        guidanceBlock.targetAccelX[id] = 0; guidanceBlock.targetAccelY[id] = 0; guidanceBlock.targetAccelZ[id] = 0;
        guidanceBlock.targetAccelAvailable[id] = false;
        guidanceBlock.commandedAccelX[id] = 0; guidanceBlock.commandedAccelY[id] = 0; guidanceBlock.commandedAccelZ[id] = 0;
        guidanceBlock.maxAccel[id] = 0.0;
        guidanceBlock.navigationConstant[id] = resolved.guidanceAutopilot.navigationConstant;
        guidanceBlock.waypointGain[id] = resolved.guidanceAutopilot.waypointGain;
        // W36 phase-manager config (defaults preserve the legacy path).
        guidanceBlock.handoffBlendTimeSec[id] = resolved.guidanceAutopilot.handoffBlendTimeSec;
        guidanceBlock.lockLossRetentionSec[id] = resolved.guidanceAutopilot.lockLossRetentionSec;
        guidanceBlock.apnFeedforwardEnabled[id] = resolved.guidanceAutopilot.apnFeedforwardEnabled;
        guidanceBlock.gravityCompensationEnabled[id] = resolved.guidanceAutopilot.gravityCompensationEnabled;
        // W36 state/diagnostics reset (fresh and reused slots).
        guidanceBlock.phase[id] = GuidancePhase::None;
        guidanceBlock.law[id] = GuidanceLaw::None;
        guidanceBlock.trackId[id] = -1;
        guidanceBlock.trackAgeSec[id] = 0.0;
        guidanceBlock.handoffWeight[id] = 0.0;
        guidanceBlock.lockLossCount[id] = 0;
        guidanceBlock.rawAccelX[id] = 0; guidanceBlock.rawAccelY[id] = 0; guidanceBlock.rawAccelZ[id] = 0;
        guidanceBlock.limitedByMaxAccel[id] = false;
        guidanceBlock.lawInvalid[id] = false;
        guidanceBlock.nonClosing[id] = false;
        guidanceBlock.tgoSec[id] = 0.0;
        guidanceBlock.retainedAccelX[id] = 0;
        guidanceBlock.retainedAccelY[id] = 0;
        guidanceBlock.retainedAccelZ[id] = 0;
        // W40 trajectory-core config + state/diagnostics reset (fresh and reused).
        guidanceBlock.trajectoryMinSpeedMps[id] = resolved.guidanceAutopilot.trajectoryMinSpeedMps;
        guidanceBlock.trajectoryFeasibilityAccelFactor[id] = resolved.guidanceAutopilot.trajectoryFeasibilityAccelFactor;
        guidanceBlock.datalinkSourceId[id] = resolved.guidanceAutopilot.datalinkSourceId;
        guidanceBlock.datalinkTargetId[id] = resolved.guidanceAutopilot.datalinkTargetId;
        guidanceBlock.predictedInterceptX[id] = 0;
        guidanceBlock.predictedInterceptY[id] = 0;
        guidanceBlock.predictedInterceptZ[id] = 0;
        guidanceBlock.predictedTgoSec[id] = 0.0;
        guidanceBlock.trajectoryRequiredAccel[id] = 0.0;
        guidanceBlock.trajectoryAimSource[id] = GuidanceAimSource::None;
        guidanceBlock.trajectoryFeasible[id] = false;
        guidanceBlock.trajectoryReason[id] = TrajectoryReason::None;

        controlBlock.pitchSaturated[id] = false;
        controlBlock.yawSaturated[id] = false;
        controlBlock.rollSaturated[id] = false;

        // W39 track config + state reset (fresh and reused slots).
        trackBlock.confirmations[id] = resolved.guidanceAutopilot.trackConfirmations;
        trackBlock.coastTimeoutSec[id] = resolved.guidanceAutopilot.trackCoastTimeoutSec;
        trackBlock.lossTimeoutSec[id] = resolved.guidanceAutopilot.trackLossTimeoutSec;
        trackBlock.state[id] = TrackState::None;
        trackBlock.trackId[id] = -1;
        trackBlock.posX[id] = 0; trackBlock.posY[id] = 0; trackBlock.posZ[id] = 0;
        trackBlock.velX[id] = 0; trackBlock.velY[id] = 0; trackBlock.velZ[id] = 0;
        trackBlock.accelX[id] = 0; trackBlock.accelY[id] = 0; trackBlock.accelZ[id] = 0;
        trackBlock.accelAvailable[id] = false;
        trackBlock.timestampSec[id] = 0.0;
        trackBlock.ageSec[id] = 0.0;
        trackBlock.positionStdM[id] = 5.0;
        trackBlock.velocityStdMs[id] = 25.0;
        trackBlock.quality01[id] = 0.0;
        trackBlock.updateCount[id] = 0;
        trackBlock.dropoutCount[id] = 0;
        trackBlock.measPosX[id] = 0; trackBlock.measPosY[id] = 0; trackBlock.measPosZ[id] = 0;
        trackBlock.measTimeSec[id] = 0.0;

        sensorBlock.accelNoiseStdDev[id] = resolved.sensor.accelNoiseStdDev;
        sensorBlock.accelBiasStdDev[id] = resolved.sensor.accelBiasStdDev;
        sensorBlock.gyroNoiseStdDev[id] = resolved.sensor.gyroNoiseStdDev;
        sensorBlock.gyroBiasStdDev[id] = resolved.sensor.gyroBiasStdDev;
        sensorBlock.gpsPosNoiseStdDev[id] = resolved.sensor.gpsPosNoiseStdDev;
        sensorBlock.gpsVelNoiseStdDev[id] = resolved.sensor.gpsVelNoiseStdDev;
        sensorBlock.gpsInnovationGateSigma[id] = resolved.sensor.gpsInnovationGateSigma;
        sensorBlock.imuLeverArmX[id] = resolved.sensor.imuLeverArmX;
        sensorBlock.imuLeverArmY[id] = resolved.sensor.imuLeverArmY;
        sensorBlock.imuLeverArmZ[id] = resolved.sensor.imuLeverArmZ;
        sensorBlock.imuEnabled[id] = resolved.sensor.imuEnabled;
        sensorBlock.gpsEnabled[id] = resolved.sensor.gpsEnabled;
        sensorBlock.gpsUpdateRateHz[id] = resolved.sensor.gpsUpdateRateHz;

        physicsBlock.px[id] = init.px; physicsBlock.py[id] = init.py; physicsBlock.pz[id] = init.pz;
        physicsBlock.vx[id] = init.vx; physicsBlock.vy[id] = init.vy; physicsBlock.vz[id] = init.vz;
        physicsBlock.qw[id] = init.qw; physicsBlock.qx[id] = init.qx; physicsBlock.qy[id] = init.qy; physicsBlock.qz[id] = init.qz;
        physicsBlock.wx[id] = init.wx; physicsBlock.wy[id] = init.wy; physicsBlock.wz[id] = init.wz;
        physicsBlock.alphax[id] = 0.0; physicsBlock.alphay[id] = 0.0; physicsBlock.alphaz[id] = 0.0;
        // Ambient mirrors are populated by the first post-step refresh.
        physicsBlock.mach[id] = 0.0;
        physicsBlock.dynamicPressure[id] = 0.0;
        physicsBlock.airDensity[id] = 0.0;
        physicsBlock.localSpeedOfSound[id] = 0.0;
        physicsBlock.Ixx[id] = config.Ixx; physicsBlock.Iyy[id] = config.Iyy; physicsBlock.Izz[id] = config.Izz;
        physicsBlock.Ixy[id] = config.Ixy; physicsBlock.Ixz[id] = config.Ixz; physicsBlock.Iyz[id] = config.Iyz;

        // Multi-stage propulsion: register every stage with a non-empty thrust
        // curve in the backend pool and wire the first stage. Separable dry
        // masses (all stages except the last) form the initial massDry floor;
        // processStaging() drops them and advances the active stage.
        StagePlan plan;
        for (const auto& stage : resolved.propulsion.stages) {
            if (stage.thrustCurve.empty()) continue;
            auto prop = std::make_shared<Models::PropulsionModel>(
                Models::ThrustCurve(stage.thrustCurve),
                stage.vacuumIsp, stage.seaLevelIsp,
                Models::PropulsionModelOptions{
                    stage.ignitionDelaySec, stage.ignitionRampSec,
                    stage.shutdownTimeSec, stage.shutdownRampSec,
                    stage.maxGimbalPitchRad, stage.maxGimbalYawRad});
            const double burnDuration = prop->burnDuration();
            plan.poolIds.push_back(backend->registerPropulsion(std::move(prop)));
            plan.burnDurations.push_back(burnDuration);
            plan.dropMasses.push_back(stage.dryMassKg);
            plan.propellantCaps.push_back(stage.propellantMassKg);
            plan.maxGimbalPitchRad.push_back(stage.maxGimbalPitchRad);
            plan.maxGimbalYawRad.push_back(stage.maxGimbalYawRad);
            plan.gimbalTimeConstantSec.push_back(stage.gimbalTimeConstantSec);
            plan.maxGimbalRateRadPerSec.push_back(stage.maxGimbalRateRadPerSec);
            plan.enginePositionX.push_back(stage.enginePositionX);
            plan.enginePositionY.push_back(stage.enginePositionY);
            plan.enginePositionZ.push_back(stage.enginePositionZ);
        }
        // reservedAfter[i] = sum of later stages' positive propellant caps
        // (fuel kept off-limits for the current stage).
        plan.reservedAfter.assign(plan.poolIds.size(), 0.0);
        double reserved = 0.0;
        for (std::size_t k = plan.poolIds.size(); k-- > 0;) {
            plan.reservedAfter[k] = reserved;
            reserved += std::max(0.0, plan.propellantCaps[k]);
        }
        double separableDry = 0.0;
        for (std::size_t s = 0; s + 1 < plan.dropMasses.size(); ++s) {
            separableDry += plan.dropMasses[s];
        }
        const bool hasStages = !plan.poolIds.empty();
        const double reservedAfter0 = plan.reservedAfter.empty() ? 0.0 : plan.reservedAfter[0];
        if (hasStages) {
            physicsBlock.propulsionId[id] = plan.poolIds.front();
            physicsBlock.stageIndex[id] = 0;
            physicsBlock.stageCount[id] = static_cast<int>(plan.poolIds.size());
            stagePlans[id] = std::move(plan);
        } else {
            physicsBlock.propulsionId[id] = -1;
            physicsBlock.stageIndex[id] = -1;
            physicsBlock.stageCount[id] = 0;
            stagePlans[id] = StagePlan{};
        }

        physicsBlock.mass[id] = (config.initialMass >= 0.0) ? config.initialMass : init.mass;
        const double finalDry = (config.massDry < 0.0) ? physicsBlock.mass[id] : config.massDry;
        physicsBlock.massDry[id] = finalDry + separableDry;
        // The stage floor reserves later stages' propellant: the active stage
        // may burn only down to dry mass + reserved fuel.
        physicsBlock.stageMinMass[id] = hasStages ? physicsBlock.massDry[id] + reservedAfter0 : 0.0;
        physicsBlock.gimbalPitch[id] = 0.0;
        physicsBlock.gimbalYaw[id] = 0.0;
        if (hasStages) {
            physicsBlock.maxGimbalPitchRad[id] = stagePlans[id].maxGimbalPitchRad.front();
            physicsBlock.maxGimbalYawRad[id] = stagePlans[id].maxGimbalYawRad.front();
            physicsBlock.gimbalTimeConstantSec[id] = stagePlans[id].gimbalTimeConstantSec.front();
            physicsBlock.maxGimbalRateRadPerSec[id] = stagePlans[id].maxGimbalRateRadPerSec.front();
            physicsBlock.enginePositionX[id] = stagePlans[id].enginePositionX.front();
            physicsBlock.enginePositionY[id] = stagePlans[id].enginePositionY.front();
            physicsBlock.enginePositionZ[id] = stagePlans[id].enginePositionZ.front();
        } else {
            physicsBlock.maxGimbalPitchRad[id] = 0.0;
            physicsBlock.maxGimbalYawRad[id] = 0.0;
            physicsBlock.gimbalTimeConstantSec[id] = 0.02;
            physicsBlock.maxGimbalRateRadPerSec[id] = 0.0;
            physicsBlock.enginePositionX[id] = 0.0;
            physicsBlock.enginePositionY[id] = 0.0;
            physicsBlock.enginePositionZ[id] = 0.0;
        }

        physicsBlock.referenceArea[id] = resolved.aero.referenceArea;
        physicsBlock.referenceLength[id] = resolved.aero.referenceLength;
        physicsBlock.cd[id] = resolved.aero.cd;
        physicsBlock.clAlpha[id] = resolved.aero.clAlpha;
        physicsBlock.clFin[id] = resolved.aero.clFin;
        physicsBlock.clMax[id] = resolved.aero.clMax;
        physicsBlock.aeroTables[id] = resolved.aero.tables.empty()
            ? nullptr
            : std::make_shared<const Models::AeroTables>(resolved.aero.tables);
        std::vector<std::shared_ptr<const Models::FinsGeometry>> builtFinSets;
        for (const auto& fc : resolved.aero.allFinSets()) {
            if (fc.enabled()) {
                std::string err;
                auto g = Models::buildFinsGeometry(
                    fc.shape, fc.count,
                    fc.rootChordM, fc.tipChordM,
                    fc.spanM, fc.sweepLengthM,
                    fc.positionM, fc.cantAngleDeg,
                    fc.shapePoints, resolved.aero.referenceArea, &err,
                    fc.steerable);
                if (!g) {
                    throw std::runtime_error("Invalid fins configuration: " + err);
                }
                builtFinSets.push_back(g);
            }
        }
        physicsBlock.finSets[id] = builtFinSets;
        physicsBlock.fins[id] = builtFinSets.empty() ? nullptr : builtFinSets.front();
        physicsBlock.finPitch[id] = 0.0; physicsBlock.finYaw[id] = 0.0; physicsBlock.finRoll[id] = 0.0;
        physicsBlock.ignitionTime[id] = time.currentTime();
        physicsBlock.active[id] = true;
        physicsBlock.motorFailed[id] = false;
        physicsBlock.engineFailed[id] = false;
        physicsBlock.tankFailed[id] = false;
        physicsBlock.actuatorFailed[id] = false;

        statusBlock.type[id] = config.type;
        statusBlock.allegiance[id] = init.allegiance;
        statusBlock.health[id] = 100.0;
        statusBlock.isAlive[id] = true;
        statusBlock.motorFailed[id] = false;
        statusBlock.engineFailed[id] = false;
        statusBlock.tankFailed[id] = false;
        statusBlock.actuatorFailed[id] = false;
        statusBlock.sensorFailed[id] = false;
        statusBlock.commsFailed[id] = false;
        statusBlock.rcsProfileId[id] = config.rcsProfileId;
        statusBlock.irProfileId[id] = config.irProfileId;
        statusBlock.emitterEirpW[id] = config.emitterEirpW;

        // Warhead state (fusing + lethality handled by processWarheads()).
        if (resolved.warhead.falloffRadiusM > 0.0 &&
            resolved.warhead.falloffRadiusM < resolved.warhead.lethalRadiusM) {
            throw std::runtime_error("SimulationKernel: warhead falloff_radius_m (" +
                                     std::to_string(resolved.warhead.falloffRadiusM) +
                                     " m) is below lethal_radius_m (" +
                                     std::to_string(resolved.warhead.lethalRadiusM) + " m)");
        }
        warheads[id] = WarheadState{};
        warheads[id].lethalRadiusM = resolved.warhead.lethalRadiusM;
        warheads[id].falloffRadiusM = resolved.warhead.falloffRadiusM;
        warheads[id].fusing = resolved.warhead.fusing;
        warheads[id].proximityTriggerM = resolved.warhead.proximityTriggerM;
        warheads[id].timedDelaySec = resolved.warhead.timedDelaySec;
        warheads[id].launchTime = time.currentTime();
        warheads[id].detonated = false;

        // Per-entity seeker configuration (public SeekerConfig surface).
        seekerBlock.type[id] = resolved.seeker.type;
        seekerBlock.transmitterPowerW[id] = resolved.seeker.transmitterPowerW;
        seekerBlock.antennaGainDb[id] = resolved.seeker.antennaGainDb;
        seekerBlock.wavelengthM[id] = resolved.seeker.wavelengthM;
        seekerBlock.noiseFloorW[id] = resolved.seeker.noiseFloorW;
        seekerBlock.snrThresholdDb[id] = resolved.seeker.snrThresholdDb;
        seekerBlock.sensitivityW[id] = resolved.seeker.sensitivityW;
        seekerBlock.wavelengthBand[id] = resolved.seeker.wavelengthBand;
        seekerBlock.irExtinctionPerM[id] = resolved.seeker.irExtinctionPerM;
        seekerBlock.illuminatorPx[id] = resolved.seeker.illuminatorPx;
        seekerBlock.illuminatorPy[id] = resolved.seeker.illuminatorPy;
        seekerBlock.illuminatorPz[id] = resolved.seeker.illuminatorPz;
        seekerBlock.illuminatorPowerW[id] = resolved.seeker.illuminatorPowerW;
        seekerBlock.illuminatorGainDb[id] = resolved.seeker.illuminatorGainDb;
        seekerBlock.illuminatorWavelengthM[id] = resolved.seeker.illuminatorWavelengthM;
        seekerBlock.fieldOfViewHalfAngleRad[id] = resolved.seeker.fieldOfViewHalfAngleRad;
        seekerBlock.gimbalAzimuthLimitRad[id] = resolved.seeker.gimbalAzimuthLimitRad;
        seekerBlock.gimbalElevationLimitRad[id] = resolved.seeker.gimbalElevationLimitRad;
        seekerBlock.lockHysteresisDb[id] = resolved.seeker.lockHysteresisDb;
        seekerBlock.lockDropoutTimeSec[id] = resolved.seeker.lockDropoutTimeSec;
        seekerBlock.measurementLatencySec[id] = resolved.seeker.measurementLatencySec;
        seekerBlock.isLocked[id] = false;
        seekerBlock.lockedTargetId[id] = 0;
        seekerBlock.targetRange[id] = 0.0;
        seekerBlock.targetRangeRate[id] = 0.0;
        seekerBlock.targetAzimuth[id] = 0.0;
        seekerBlock.targetElevation[id] = 0.0;
        seekerBlock.targetAzimuthRate[id] = 0.0;
        seekerBlock.targetElevationRate[id] = 0.0;
        seekerBlock.previousAzimuth[id] = 0.0;
        seekerBlock.previousElevation[id] = 0.0;
        seekerBlock.lockLostTimeSec[id] = 0.0;
        seekerBlock.hasPreviousLos[id] = false;

        return id;
    }

    void SimulationKernel::removeVehicle(PhysicsId id) {
        if (id >= physicsBlock.size || !physicsBlock.active[id]) return;
        physicsBlock.active[id] = false;
        statusBlock.isAlive[id] = false;
        freeList.push_back(id);
    }

    void SimulationKernel::failEntity(PhysicsId id, FailureMode mode) {
        if (id >= physicsBlock.size) {
            throw std::out_of_range("SimulationKernel::failEntity: entity id out of range");
        }
        if (mode == FailureMode::None) return;

        SimulationEvent evt;
        evt.entityId = id;
        evt.timestamp = time.currentTime();

        switch (mode) {
            case FailureMode::MotorFailure:
                statusBlock.motorFailed[id] = true;
                statusBlock.engineFailed[id] = true;
                physicsBlock.motorFailed[id] = true;
                physicsBlock.engineFailed[id] = true;
                evt.type = EventType::MotorFailure;
                break;
            case FailureMode::EngineFailure:
                statusBlock.motorFailed[id] = true;
                statusBlock.engineFailed[id] = true;
                physicsBlock.motorFailed[id] = true;
                physicsBlock.engineFailed[id] = true;
                evt.type = EventType::EngineFailure;
                break;
            case FailureMode::TankFailure:
                statusBlock.tankFailed[id] = true;
                physicsBlock.tankFailed[id] = true;
                evt.type = EventType::TankFailure;
                break;
            case FailureMode::ActuatorFailure:
                statusBlock.actuatorFailed[id] = true;
                physicsBlock.actuatorFailed[id] = true;
                evt.type = EventType::ActuatorFailure;
                break;
            case FailureMode::SensorFailure:
                statusBlock.sensorFailed[id] = true;
                evt.type = EventType::SensorFailure;
                break;
            case FailureMode::CommunicationFailure:
                statusBlock.commsFailed[id] = true;
                evt.type = EventType::CommunicationFailure;
                break;
            case FailureMode::StructuralFailure: {
                // Deactivate exactly like a ground impact: kill the entity,
                // zero the motion state, and dispatch the event.
                statusBlock.isAlive[id] = false;
                statusBlock.health[id] = 0.0;
                physicsBlock.active[id] = false;
                physicsBlock.vx[id] = 0.0;
                physicsBlock.vy[id] = 0.0;
                physicsBlock.vz[id] = 0.0;
                physicsBlock.ax[id] = 0.0;
                physicsBlock.ay[id] = 0.0;
                physicsBlock.az[id] = 0.0;
                evt.type = EventType::StructuralFailure;
                break;
            }
            case FailureMode::None:
                return;
        }
        eventSystem.dispatch(evt);
    }

    void SimulationKernel::applyDamage(PhysicsId id, double damage) {
        if (id >= physicsBlock.size) {
            throw std::out_of_range("SimulationKernel::applyDamage: entity id out of range");
        }
        if (damage < 0.0) {
            throw std::invalid_argument("SimulationKernel::applyDamage: damage must be non-negative");
        }
        statusBlock.health[id] = std::max(0.0, statusBlock.health[id] - damage);
        if (statusBlock.health[id] <= 0.0 && statusBlock.isAlive[id]) {
            failEntity(id, FailureMode::StructuralFailure);
        }
    }

    void SimulationKernel::processStaging() {
        for (std::size_t i = 0; i < physicsBlock.size; ++i) {
            if (!physicsBlock.active[i]) continue;
            if (physicsBlock.stageCount[i] <= 0) continue;
            const int si = physicsBlock.stageIndex[i];
            if (si < 0 || si + 1 >= physicsBlock.stageCount[i]) continue;
            const StagePlan& plan = stagePlans[i];
            if (si + 1 >= static_cast<int>(plan.poolIds.size())) continue;

            // Burnout: the active stage's thrust curve has fully elapsed, or
            // the stage's propellant cap has been drawn down to its
            // stage-aware floor (mass can no longer decrease).
            const bool curveElapsed =
                time.currentTime() - physicsBlock.ignitionTime[i] >= plan.burnDurations[si];
            const bool propellantExhausted =
                si < static_cast<int>(plan.propellantCaps.size()) &&
                plan.propellantCaps[si] > 0.0 &&
                physicsBlock.mass[i] <= physicsBlock.stageMinMass[i] + 1e-9;
            if (!curveElapsed && !propellantExhausted) {
                continue;
            }

            // Separation: dump the spent stage's leftover propellant (unburned
            // mass above the stage's floor of dry + fuel reserved for the
            // remaining stages), drop its structure, and rescale inertia with
            // the post-dump mass ratio. A stage that burned out by propellant
            // exhaustion pinned itself to the floor, so leftover is ~0 and the
            // behavior is unchanged.
            const double drop = plan.dropMasses[si];
            const double oldMass = physicsBlock.mass[i];
            double leftover = oldMass - (physicsBlock.massDry[i] + plan.reservedAfter[si]);
            if (leftover < 1e-6) leftover = 0.0;
            const double newMass = oldMass - drop - leftover;
            physicsBlock.mass[i] = newMass;
            physicsBlock.massDry[i] -= drop;
            const double ratio = (oldMass > 1e-12) ? (newMass / oldMass) : 1.0;
            physicsBlock.Ixx[i] *= ratio;
            physicsBlock.Iyy[i] *= ratio;
            physicsBlock.Izz[i] *= ratio;
            physicsBlock.Ixy[i] *= ratio;
            physicsBlock.Ixz[i] *= ratio;
            physicsBlock.Iyz[i] *= ratio;

            // Ignite the next stage.
            ++physicsBlock.stageIndex[i];
            physicsBlock.propulsionId[i] = plan.poolIds[physicsBlock.stageIndex[i]];
            physicsBlock.ignitionTime[i] = time.currentTime();

            // Raise the floor for the new active stage (reserve later stages'
            // propellant again).
            const int nextSi = physicsBlock.stageIndex[i];
            if (nextSi >= 0 && nextSi < static_cast<int>(plan.reservedAfter.size())) {
                physicsBlock.stageMinMass[i] = physicsBlock.massDry[i] + plan.reservedAfter[nextSi];
            }
            physicsBlock.gimbalPitch[i] = 0.0;
            physicsBlock.gimbalYaw[i] = 0.0;
            physicsBlock.maxGimbalPitchRad[i] = plan.maxGimbalPitchRad[nextSi];
            physicsBlock.maxGimbalYawRad[i] = plan.maxGimbalYawRad[nextSi];
            physicsBlock.gimbalTimeConstantSec[i] = plan.gimbalTimeConstantSec[nextSi];
            physicsBlock.maxGimbalRateRadPerSec[i] = plan.maxGimbalRateRadPerSec[nextSi];
            physicsBlock.enginePositionX[i] = plan.enginePositionX[nextSi];
            physicsBlock.enginePositionY[i] = plan.enginePositionY[nextSi];
            physicsBlock.enginePositionZ[i] = plan.enginePositionZ[nextSi];

            SimulationEvent evt;
            evt.type = EventType::StageSeparation;
            evt.entityId = i;
            evt.timestamp = time.currentTime();
            evt.customCode = si;
            evt.dumpedMassKg = leftover;
            eventSystem.dispatch(evt);
        }
    }

    void SimulationKernel::processWarheads() {
        for (std::size_t i = 0; i < warheads.size(); ++i) {
            WarheadState& wh = warheads[i];
            if (wh.detonated || wh.lethalRadiusM <= 0.0) continue;

            bool trigger = false;
            switch (wh.fusing) {
                case FusingType::Impact:
                    trigger = !statusBlock.isAlive[i];
                    break;
                case FusingType::Proximity: {
                    if (wh.proximityTriggerM > 0.0) {
                        const double r2 = wh.proximityTriggerM * wh.proximityTriggerM;
                        for (std::size_t j = 0; j < physicsBlock.size; ++j) {
                            if (j == i || !statusBlock.isAlive[j]) continue;
                            if (statusBlock.allegiance[i] == statusBlock.allegiance[j]) continue;
                            const double dx = physicsBlock.px[j] - physicsBlock.px[i];
                            const double dy = physicsBlock.py[j] - physicsBlock.py[i];
                            const double dz = physicsBlock.pz[j] - physicsBlock.pz[i];
                            const double dist2 = dx*dx + dy*dy + dz*dz;
                            if (dist2 <= r2) {
                                const double dvx = physicsBlock.vx[j] - physicsBlock.vx[i];
                                const double dvy = physicsBlock.vy[j] - physicsBlock.vy[i];
                                const double dvz = physicsBlock.vz[j] - physicsBlock.vz[i];
                                const double rdotv = dx * dvx + dy * dvy + dz * dvz;
                                const double dist = std::sqrt(dist2);
                                const double vrel = std::sqrt(dvx*dvx + dvy*dvy + dvz*dvz);
                                if (rdotv >= 0.0 || dist <= 2.0 || (dist - vrel * 0.02) <= 0.0) {
                                    trigger = true;
                                    break;
                                }
                            }
                        }
                    }
                    break;
                }
                case FusingType::Timed:
                    trigger = (time.currentTime() - wh.launchTime) >= wh.timedDelaySec;
                    break;
            }
            if (!trigger) continue;

            wh.detonated = true;
            SimulationEvent evt;
            evt.type = EventType::Detonation;
            evt.entityId = i;
            evt.timestamp = time.currentTime();
            eventSystem.dispatch(evt);

            // Lethality with an optional fragmentation/overpressure falloff
            // band: guaranteed kill inside lethalRadiusM, probabilistic kill
            // across (lethalRadiusM, falloffRadiusM], no effect beyond. The
            // RNG draw is taken only when the outcome is genuinely uncertain
            // (0 < p < 1) so flat-law warheads (falloffRadiusM <= 0) never
            // consume the kernel RNG stream and keep today's behavior.
            for (std::size_t j = 0; j < physicsBlock.size; ++j) {
                if (j == i || !statusBlock.isAlive[j]) continue;
                if (statusBlock.allegiance[i] == statusBlock.allegiance[j]) continue;
                const double dx = physicsBlock.px[j] - physicsBlock.px[i];
                const double dy = physicsBlock.py[j] - physicsBlock.py[i];
                const double dz = physicsBlock.pz[j] - physicsBlock.pz[i];
                const double missDistance = std::sqrt(dx*dx + dy*dy + dz*dz);
                const double prob = Models::warheadKillProbability(
                    missDistance, wh.lethalRadiusM, wh.falloffRadiusM);
                const bool kill = (prob >= 1.0) ||
                    (prob > 0.0 && prob >= sensorSystem.nextUniform01());
                if (kill) {
                    applyDamage(j, 100.0);
                }
            }
        }
    }

    void SimulationKernel::queueCommand(const SimulationCommand& cmd) {
        commandProcessor.enqueueCommand(cmd);
    }

    void SimulationKernel::setThrustVectorCommand(PhysicsId id, double pitchRad, double yawRad) {
        if (id >= physicsBlock.size) {
            throw std::out_of_range("SimulationKernel::setThrustVectorCommand: entity id out of range");
        }
        if (!std::isfinite(pitchRad) || !std::isfinite(yawRad)) {
            throw std::invalid_argument("SimulationKernel::setThrustVectorCommand: angles must be finite");
        }
        controlBlock.thrustVectorPitchCommand[id] = pitchRad;
        controlBlock.thrustVectorYawCommand[id] = yawRad;
    }

    void SimulationKernel::step(double dt) {
        if (dt <= 0.0) {
            throw std::invalid_argument(
                "SimulationKernel::step requires a positive timestep");
        }
        const std::vector<double> previousPx = physicsBlock.px;
        const std::vector<double> previousPy = physicsBlock.py;
        const std::vector<double> previousPz = physicsBlock.pz;
        time.advance(dt);
        commandProcessor.process(guidanceBlock, trackBlock, time.currentTime());
        
        // 1. Advance true physics
        backend->step(physicsBlock, controlBlock, time.currentTime(), dt);

        // 1.5 Stage separation (multi-stage propulsion)
        processStaging();

        // 2. Generate noisy sensor measurements
        sensorSystem.update(physicsBlock, sensorBlock, statusBlock, time.currentTime(), dt, environment);
        
        // 3. Compute Navigation estimates (INS + EKF)
        navigationSystem.update(sensorBlock, physicsBlock, navigationBlock, dt, environment);
        
        // 3.5 Process Seekers
        seekerSystem.update(physicsBlock, statusBlock, seekerBlock, dt);

        // 3.75 Persistent target-track manager (seeker measurements + command
        // seeds -> estimate; prediction at the sim rate between measurements)
        trackManagerSystem.update(navigationBlock, seekerBlock, trackBlock,
                                  time.currentTime(), dt);

        // 4. Update Guidance based on estimates, tracks, and seekers
        guidanceSystem.update(statusBlock, navigationBlock, seekerBlock,
                              trackBlock, guidanceBlock, controlBlock, dt);

        // 4.5 Update Autopilot to translate commanded accel to fin deflections
        autopilotSystem.update(statusBlock, navigationBlock, sensorBlock, guidanceBlock, controlBlock, dt);
        
        // 5. Evaluate truth events (impacts)
        eventSystem.evaluate(
            physicsBlock, statusBlock, time.currentTime(), dt,
            previousPx, previousPy, previousPz, environment);

        // 5.5 Warhead fusing/detonation (after impacts are known)
        processWarheads();

        eventSystem.processQueue();
    }

    void SimulationKernel::runSteps(std::size_t steps, double dt) {
        for (std::size_t i = 0; i < steps; ++i) {
            step(dt);
        }
    }

} // namespace StrikeEngine::Kernel
