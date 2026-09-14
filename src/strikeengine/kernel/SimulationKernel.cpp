#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/backend/BackendFactory.hpp>
#include <strikeengine/models/physics/earth/EarthModel.hpp>
#include <strikeengine/kernel/backend/CPUBackend.hpp>
#include <strikeengine/kernel/profiles/AeroProfileDatabase.hpp>
#include <strikeengine/kernel/profiles/MotorProfileDatabase.hpp>
#include <strikeengine/kernel/profiles/SeekerProfileDatabase.hpp>
#include <strikeengine/kernel/profiles/SensorProfileDatabase.hpp>
#include <strikeengine/kernel/profiles/GuidanceProfileDatabase.hpp>
#include <strikeengine/kernel/profiles/WarheadProfileDatabase.hpp>
#include <strikeengine/models/physics/atmosphere/ISA1976.hpp>
#include <strikeengine/models/physics/aerodynamics/AeroModel.hpp>
#include <strikeengine/models/physics/aerodynamics/AirframeModel.hpp>
#include <strikeengine/models/physics/propulsion/PropulsionModel.hpp>
#include <strikeengine/models/warhead/WarheadEffects.hpp>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

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
        randomSeed = seed;
        sensorSystem.setSeed(seed);
        navigationSystem.setSeed(seed);
        seekerSystem.setSeed(seed);
        // Split stream: golden-ratio mix keeps warhead draws disjoint from
        // the sensor stream for every seed.
        warheadRng.seed(seed ^ 0x9E3779B9u);
        // Separate fuze-detection stream so a probabilistic proximity fuze
        // never shifts the lethality draws.
        fuzeRng.seed(seed ^ 0xF00D5EEDu);
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
        trackBlock = TrackBlock();
        seekerSystem.reset();
        sensorSystem.reset();
        // Re-apply the stored seed so a reset kernel reproduces the same
        // streams (reset used to leave both RNGs wherever they stopped,
        // silently breaking the seed contract on kernel reuse).
        sensorSystem.setSeed(randomSeed);
        navigationSystem.setSeed(randomSeed);
        seekerSystem.setSeed(randomSeed);
        warheadRng.seed(randomSeed ^ 0x9E3779B9u);
        fuzeRng.seed(randomSeed ^ 0xF00D5EEDu);
        eventSystem.resetTransientState();
        if (backend) backend->reset();
        freeList.clear();
        stagePlans.clear();
        warheads.clear();
        pendingLaunches.clear();
        activeFlyouts.clear();
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

        // Mass properties: the backend divides by mass every derivative
        // evaluation, so a non-positive or non-finite launch mass would poison
        // the entity with inf/NaN on the first step. Fail fast instead. The
        // dry floor must also stay at or below the launch mass (massDry > mass
        // would imply negative fuel and trip staging burnout immediately).
        const double launchMass =
            (config.initialMass >= 0.0) ? config.initialMass : init.mass;
        if (!std::isfinite(launchMass) || launchMass <= 0.0) {
            throw std::invalid_argument(
                "SimulationKernel::createVehicle: launch mass must be positive and finite (got " +
                std::to_string(launchMass) + " kg; set VehicleInitState::mass or VehicleConfig::initialMass)");
        }
        if (config.massDry >= 0.0 &&
            (!std::isfinite(config.massDry) || config.massDry > launchMass)) {
            throw std::invalid_argument(
                "SimulationKernel::createVehicle: massDry (" +
                std::to_string(config.massDry) +
                " kg) must be finite and <= launch mass (" +
                std::to_string(launchMass) + " kg)");
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
            trackBlock.size = physicsBlock.size;   // the track manager reads tracks.size
            
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
            physicsBlock.airframe.push_back(nullptr);
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
            controlBlock.kRatePitchP.push_back(-1.0);
            controlBlock.kRateYawP.push_back(-1.0);
            controlBlock.scheduleAllTerms.push_back(false);
            controlBlock.integralEnabled.push_back(false);
            controlBlock.kIntegralPitch.push_back(0.0);
            controlBlock.kIntegralYaw.push_back(0.0);
            controlBlock.integralClampRad.push_back(0.05);
            controlBlock.kAccelErrP.push_back(-1.0);
            controlBlock.threeLoopEnabled.push_back(false);
            controlBlock.controlEffectivenessEnabled.push_back(false);
            controlBlock.controlEffBase.push_back(1.0);
            controlBlock.controlEffMachSlope.push_back(0.0);
            controlBlock.controlEffMachQuad.push_back(0.0);
            controlBlock.controlEffMin.push_back(0.2);
            controlBlock.controlEffMax.push_back(5.0);
            controlBlock.yawDeadbandSmoothEnabled.push_back(false);
            controlBlock.yawDeadbandWidthMps2.push_back(0.5);
            controlBlock.commandLagSec.push_back(0.0);
            controlBlock.commandRateLimitRadPerSec.push_back(0.0);
            controlBlock.useMeasuredRatesEnabled.push_back(false);
            controlBlock.rollSuppressLateralAccelMps2.push_back(0.0);
            controlBlock.useTruthGravityModel.push_back(false);
            controlBlock.pitchIntegral.push_back(0.0);
            controlBlock.yawIntegral.push_back(0.0);
            controlBlock.pitchCommandPrev.push_back(0.0);
            controlBlock.yawCommandPrev.push_back(0.0);
            controlBlock.rollCommandPrev.push_back(0.0);
            controlBlock.specificForceDemandY.push_back(0.0);
            controlBlock.specificForceDemandZ.push_back(0.0);
            controlBlock.effectiveKAccel.push_back(0.030);
            controlBlock.controlEffectiveness.push_back(1.0);
            controlBlock.machNumber.push_back(0.0);
            controlBlock.feedForwardPitch.push_back(0.0);
            controlBlock.feedForwardYaw.push_back(0.0);
            controlBlock.rateDampingPitch.push_back(0.0);
            controlBlock.rateDampingYaw.push_back(0.0);
            controlBlock.aoaDampingPitch.push_back(0.0);
            controlBlock.aoaDampingYaw.push_back(0.0);
            controlBlock.accelErrPitch.push_back(0.0);
            controlBlock.accelErrYaw.push_back(0.0);
            controlBlock.rateCommandPitch.push_back(0.0);
            controlBlock.rateCommandYaw.push_back(0.0);
            controlBlock.achievedSpecificForceY.push_back(0.0);
            controlBlock.achievedSpecificForceZ.push_back(0.0);
            controlBlock.authorityMargin01.push_back(1.0);
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
            guidanceBlock.scheduledNavN.push_back(3.5);
            guidanceBlock.navScheduleEnabled.push_back(false);
            guidanceBlock.navConstantTerminal.push_back(3.0);
            guidanceBlock.navScheduleTgoSec.push_back(8.0);
            guidanceBlock.waypointGain.push_back(20.0);
            guidanceBlock.cruiseAltitudeM.push_back(0.0);
            guidanceBlock.cruiseAltitudeGain.push_back(0.05);
            guidanceBlock.cruiseAltitudeDamping.push_back(0.30);
            guidanceBlock.cruiseWaypointGain.push_back(0.8);
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
            guidanceBlock.terminalLaw.push_back(0);
            guidanceBlock.commandLagSec.push_back(0.0);
            guidanceBlock.commandSlewLimitMps3.push_back(0.0);
            guidanceBlock.scaleDemandOnInfeasible.push_back(false);
            guidanceBlock.rangeGainShapingEnabled.push_back(false);
            guidanceBlock.rangeGainRefM.push_back(10000.0);
            guidanceBlock.trackAimMinQuality01.push_back(0.0);
            guidanceBlock.apnFeedforwardMinQuality01.push_back(0.0);
            guidanceBlock.loftEnabled.push_back(false);
            guidanceBlock.loftAltitudeM.push_back(0.0);
            guidanceBlock.loftGain.push_back(0.0);
            guidanceBlock.loftRangeM.push_back(40000.0);
            guidanceBlock.authorityAwareLimitEnabled.push_back(false);
            guidanceBlock.shapedAccelX.push_back(0); guidanceBlock.shapedAccelY.push_back(0); guidanceBlock.shapedAccelZ.push_back(0);
            guidanceBlock.losRateMag.push_back(0.0);
            guidanceBlock.closingSpeed.push_back(0.0);
            guidanceBlock.trackLossActive.push_back(false);
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
            statusBlock.jammerEirpW.push_back(0.0);

            sensorBlock.accelNoiseStdDev.push_back(0.1);
            sensorBlock.accelBiasStdDev.push_back(0.01);
            sensorBlock.gyroNoiseStdDev.push_back(0.01);
            sensorBlock.gyroBiasStdDev.push_back(0.001);
            sensorBlock.gpsPosNoiseStdDev.push_back(5.0);
            sensorBlock.gpsVelNoiseStdDev.push_back(0.5);
            sensorBlock.gpsInnovationGateSigma.push_back(5.0);
            sensorBlock.baroInnovationGateSigma.push_back(-1.0);
            sensorBlock.magInnovationGateSigma.push_back(-1.0);
            sensorBlock.imuLeverArmX.push_back(0.0);
            sensorBlock.imuLeverArmY.push_back(0.0);
            sensorBlock.imuLeverArmZ.push_back(0.0);
            sensorBlock.imuEnabled.push_back(true);
            sensorBlock.gpsEnabled.push_back(true);
            sensorBlock.gpsUpdateRateHz.push_back(1.0);
            sensorBlock.baroUpdated.push_back(false);
            sensorBlock.baroAlt.push_back(0.0);
            sensorBlock.magUpdated.push_back(false);
            sensorBlock.magX.push_back(0.0); sensorBlock.magY.push_back(0.0); sensorBlock.magZ.push_back(0.0);
            sensorBlock.baroEnabled.push_back(false);
            sensorBlock.baroNoiseStdDev.push_back(1.0);
            sensorBlock.baroBiasStdDev.push_back(0.0);
            sensorBlock.baroUpdateRateHz.push_back(1.0);
            sensorBlock.magEnabled.push_back(false);
            sensorBlock.magNoiseStdDev.push_back(50e-9);
            sensorBlock.magUpdateRateHz.push_back(10.0);
            sensorBlock.magDisturbanceGateRel.push_back(0.25);
            sensorBlock.gpsLatencySec.push_back(0.0);
            sensorBlock.gpsLeverArmX.push_back(0.0);
            sensorBlock.gpsLeverArmY.push_back(0.0);
            sensorBlock.gpsLeverArmZ.push_back(0.0);
            sensorBlock.gpsFixConsistencyEnabled.push_back(false);
            sensorBlock.insConingCompensationEnabled.push_back(false);
            sensorBlock.insAdaptiveQEnabled.push_back(false);
            sensorBlock.insAdaptiveQGain.push_back(1.0);
            sensorBlock.initialAttitudeErrorDeg.push_back(0.0);
            sensorBlock.initialPositionErrorM.push_back(0.0);
            sensorBlock.initialVelocityErrorMps.push_back(0.0);
            sensorBlock.insGravityGradientEnabled.push_back(false);
            sensorBlock.insEarthRotationCouplingEnabled.push_back(false);
            sensorBlock.gpsBatchUpdateEnabled.push_back(false);
            sensorBlock.gpsLeverArmCompensationEnabled.push_back(false);
            sensorBlock.gpsYawCorrectionDamping.push_back(0.1);
            sensorBlock.gpsFixConsistencyThreshold.push_back(16.81);
            sensorBlock.gpsFixConsistencyConfidence.push_back(0.99);
            sensorBlock.gpsFixConsistencyDof.push_back(6);
            sensorBlock.maxAccelBiasEstimate.push_back(0.5);
            sensorBlock.maxGyroBiasEstimate.push_back(0.02);
            sensorBlock.baroAttitudeCorrectionEnabled.push_back(false);

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
            seekerBlock.measurementNoiseEnabled.push_back(false);
            seekerBlock.angleNoiseStdDevRad.push_back(0.001);
            seekerBlock.angleNoiseRefSnrDb.push_back(20.0);
            seekerBlock.rangeNoiseStdDevM.push_back(1.0);
            seekerBlock.rangeRateNoiseStdDevMps.push_back(0.5);
            seekerBlock.glintSigmaM.push_back(0.0);
            seekerBlock.glintCorrelationTauSec.push_back(1.0);
            seekerBlock.swerlingEnabled.push_back(false);
            seekerBlock.gimbalRateLimitRadPerSec.push_back(0.0);
            seekerBlock.minRangeGateM.push_back(0.0);
            seekerBlock.maxRangeGateM.push_back(0.0);
            seekerBlock.terrainMaskingEnabled.push_back(false);
            seekerBlock.minClosingRateMps.push_back(0.0);
            seekerBlock.rateFilterTauSec.push_back(0.05);
            seekerBlock.decoyRejectionDb.push_back(0.0);
            seekerBlock.passiveRfDutyCycle.push_back(1.0);
            seekerBlock.illuminatorEntityId.push_back(-1);
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
            seekerBlock.lockActive.push_back(false);
            seekerBlock.hasPublishedMeasurement.push_back(false);
            seekerBlock.measurementAgeSec.push_back(0.0);
            seekerBlock.gimbalAzimuthRad.push_back(0.0);
            seekerBlock.gimbalElevationRad.push_back(0.0);
            seekerBlock.lastSignalStrength.push_back(0.0);
            seekerBlock.lockRejectReason.push_back(
                static_cast<int>(SeekerRejectReason::None));
            seekerBlock.glintAzM.push_back(0.0);
            seekerBlock.glintElM.push_back(0.0);

            // Persistent track state (config defaults; reset in the common path)
            trackBlock.confirmations.push_back(3);
            trackBlock.coastTimeoutSec.push_back(0.5);
            trackBlock.lossTimeoutSec.push_back(2.0);
            trackBlock.filterEnabled.push_back(false);
            trackBlock.processNoiseMps2.push_back(15.0);
            trackBlock.angleStdRad.push_back(0.003);
            trackBlock.measNoiseScale.push_back(1.0);
            trackBlock.residualGateSigma.push_back(0.0);
            trackBlock.maxAccelMps2.push_back(0.0);
            trackBlock.retargetConfirmations.push_back(1);
            trackBlock.seedPolicy.push_back(0);
            trackBlock.minQuality01.push_back(0.0);
            trackBlock.qualityTauSec.push_back(1.0);
            trackBlock.velocityBlend.push_back(0.08);
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
            trackBlock.kfCov.push_back({});
            trackBlock.retargetCandidateId.push_back(-1);
            trackBlock.retargetCount.push_back(0);
            trackBlock.lastInnovationM.push_back(0.0);
            trackBlock.residualRejectCount.push_back(0);
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
        controlBlock.kRatePitchP[id] = resolved.guidanceAutopilot.kRatePitchP;
        controlBlock.kRateYawP[id] = resolved.guidanceAutopilot.kRateYawP;
        controlBlock.scheduleAllTerms[id] = resolved.guidanceAutopilot.scheduleAllTerms;
        controlBlock.integralEnabled[id] = resolved.guidanceAutopilot.autopilotIntegralEnabled;
        controlBlock.kIntegralPitch[id] = resolved.guidanceAutopilot.kIntegralPitch;
        controlBlock.kIntegralYaw[id] = resolved.guidanceAutopilot.kIntegralYaw;
        controlBlock.integralClampRad[id] = resolved.guidanceAutopilot.integralClampRad;
        controlBlock.kAccelErrP[id] = resolved.guidanceAutopilot.kAccelErrP;
        controlBlock.threeLoopEnabled[id] = resolved.guidanceAutopilot.autopilotThreeLoopEnabled;
        controlBlock.controlEffectivenessEnabled[id] = resolved.guidanceAutopilot.controlEffectivenessEnabled;
        controlBlock.controlEffBase[id] = resolved.guidanceAutopilot.controlEffBase;
        controlBlock.controlEffMachSlope[id] = resolved.guidanceAutopilot.controlEffMachSlope;
        controlBlock.controlEffMachQuad[id] = resolved.guidanceAutopilot.controlEffMachQuad;
        controlBlock.controlEffMin[id] = resolved.guidanceAutopilot.controlEffMin;
        controlBlock.controlEffMax[id] = resolved.guidanceAutopilot.controlEffMax;
        controlBlock.yawDeadbandSmoothEnabled[id] = resolved.guidanceAutopilot.yawDeadbandSmoothEnabled;
        controlBlock.yawDeadbandWidthMps2[id] = resolved.guidanceAutopilot.yawDeadbandWidthMps2;
        controlBlock.commandLagSec[id] = resolved.guidanceAutopilot.commandLagSec;
        controlBlock.commandRateLimitRadPerSec[id] = resolved.guidanceAutopilot.commandRateLimitRadPerSec;
        controlBlock.useMeasuredRatesEnabled[id] = resolved.guidanceAutopilot.useMeasuredRatesEnabled;
        controlBlock.rollSuppressLateralAccelMps2[id] = resolved.guidanceAutopilot.rollSuppressLateralAccelMps2;
        controlBlock.useTruthGravityModel[id] = resolved.guidanceAutopilot.useTruthGravityModel;
        controlBlock.pitchIntegral[id] = 0.0;
        controlBlock.yawIntegral[id] = 0.0;
        controlBlock.pitchCommandPrev[id] = 0.0;
        controlBlock.yawCommandPrev[id] = 0.0;
        controlBlock.rollCommandPrev[id] = 0.0;
        controlBlock.specificForceDemandY[id] = 0.0;
        controlBlock.specificForceDemandZ[id] = 0.0;
        controlBlock.effectiveKAccel[id] = resolved.guidanceAutopilot.kAccelP;
        controlBlock.controlEffectiveness[id] = 1.0;
        controlBlock.machNumber[id] = 0.0;
        controlBlock.feedForwardPitch[id] = 0.0;
        controlBlock.feedForwardYaw[id] = 0.0;
        controlBlock.rateDampingPitch[id] = 0.0;
        controlBlock.rateDampingYaw[id] = 0.0;
        controlBlock.aoaDampingPitch[id] = 0.0;
        controlBlock.aoaDampingYaw[id] = 0.0;
        controlBlock.accelErrPitch[id] = 0.0;
        controlBlock.accelErrYaw[id] = 0.0;
        controlBlock.rateCommandPitch[id] = 0.0;
        controlBlock.rateCommandYaw[id] = 0.0;
        controlBlock.achievedSpecificForceY[id] = 0.0;
        controlBlock.achievedSpecificForceZ[id] = 0.0;
        controlBlock.authorityMargin01[id] = 1.0;
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
        guidanceBlock.scheduledNavN[id] = resolved.guidanceAutopilot.navigationConstant;
        guidanceBlock.navScheduleEnabled[id] = resolved.guidanceAutopilot.navScheduleEnabled;
        guidanceBlock.navConstantTerminal[id] = resolved.guidanceAutopilot.navConstantTerminal;
        guidanceBlock.navScheduleTgoSec[id] = resolved.guidanceAutopilot.navScheduleTgoSec;
        guidanceBlock.waypointGain[id] = resolved.guidanceAutopilot.waypointGain;
        // Aircraft cruise config.
        guidanceBlock.cruiseAltitudeM[id] = resolved.guidanceAutopilot.cruiseAltitudeM;
        guidanceBlock.cruiseAltitudeGain[id] = resolved.guidanceAutopilot.cruiseAltitudeGain;
        guidanceBlock.cruiseAltitudeDamping[id] = resolved.guidanceAutopilot.cruiseAltitudeDamping;
        guidanceBlock.cruiseWaypointGain[id] = resolved.guidanceAutopilot.cruiseWaypointGain;
        // Phase-manager config (defaults preserve the legacy path).
        guidanceBlock.handoffBlendTimeSec[id] = resolved.guidanceAutopilot.handoffBlendTimeSec;
        guidanceBlock.lockLossRetentionSec[id] = resolved.guidanceAutopilot.lockLossRetentionSec;
        guidanceBlock.apnFeedforwardEnabled[id] = resolved.guidanceAutopilot.apnFeedforwardEnabled;
        guidanceBlock.gravityCompensationEnabled[id] = resolved.guidanceAutopilot.gravityCompensationEnabled;
        // Phase state/diagnostics reset (fresh and reused slots).
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
        // Trajectory-core config + state/diagnostics reset (fresh and reused).
        guidanceBlock.trajectoryMinSpeedMps[id] = resolved.guidanceAutopilot.trajectoryMinSpeedMps;
        guidanceBlock.trajectoryFeasibilityAccelFactor[id] = resolved.guidanceAutopilot.trajectoryFeasibilityAccelFactor;
        guidanceBlock.terminalLaw[id] = resolved.guidanceAutopilot.terminalLaw;
        guidanceBlock.commandLagSec[id] = resolved.guidanceAutopilot.guidanceCommandLagSec;
        guidanceBlock.commandSlewLimitMps3[id] = resolved.guidanceAutopilot.guidanceCommandSlewLimitMps3;
        guidanceBlock.scaleDemandOnInfeasible[id] = resolved.guidanceAutopilot.guidanceScaleDemandOnInfeasible;
        guidanceBlock.rangeGainShapingEnabled[id] = resolved.guidanceAutopilot.guidanceRangeGainShapingEnabled;
        guidanceBlock.rangeGainRefM[id] = resolved.guidanceAutopilot.guidanceRangeGainRefM;
        guidanceBlock.trackAimMinQuality01[id] = resolved.guidanceAutopilot.guidanceTrackAimMinQuality01;
        guidanceBlock.apnFeedforwardMinQuality01[id] = resolved.guidanceAutopilot.guidanceApnFeedforwardMinQuality01;
        guidanceBlock.loftEnabled[id] = resolved.guidanceAutopilot.guidanceLoftEnabled;
        guidanceBlock.loftAltitudeM[id] = resolved.guidanceAutopilot.guidanceLoftAltitudeM;
        guidanceBlock.loftGain[id] = resolved.guidanceAutopilot.guidanceLoftGain;
        guidanceBlock.loftRangeM[id] = resolved.guidanceAutopilot.guidanceLoftRangeM;
        guidanceBlock.authorityAwareLimitEnabled[id] = resolved.guidanceAutopilot.guidanceAuthorityAwareLimitEnabled;
        guidanceBlock.shapedAccelX[id] = 0; guidanceBlock.shapedAccelY[id] = 0; guidanceBlock.shapedAccelZ[id] = 0;
        guidanceBlock.losRateMag[id] = 0.0;
        guidanceBlock.closingSpeed[id] = 0.0;
        guidanceBlock.trackLossActive[id] = false;
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

        // Track config + state reset (fresh and reused slots).
        trackBlock.confirmations[id] = resolved.guidanceAutopilot.trackConfirmations;
        trackBlock.coastTimeoutSec[id] = resolved.guidanceAutopilot.trackCoastTimeoutSec;
        trackBlock.lossTimeoutSec[id] = resolved.guidanceAutopilot.trackLossTimeoutSec;
        trackBlock.filterEnabled[id] = resolved.guidanceAutopilot.trackFilterEnabled;
        trackBlock.processNoiseMps2[id] = resolved.guidanceAutopilot.trackProcessNoiseMps2;
        trackBlock.angleStdRad[id] = resolved.guidanceAutopilot.trackAngleStdRad;
        trackBlock.measNoiseScale[id] = resolved.guidanceAutopilot.trackMeasNoiseScale;
        trackBlock.residualGateSigma[id] = resolved.guidanceAutopilot.trackResidualGateSigma;
        trackBlock.maxAccelMps2[id] = resolved.guidanceAutopilot.trackMaxAccelMps2;
        trackBlock.retargetConfirmations[id] = resolved.guidanceAutopilot.trackRetargetConfirmations;
        trackBlock.seedPolicy[id] = resolved.guidanceAutopilot.trackSeedPolicy;
        trackBlock.minQuality01[id] = resolved.guidanceAutopilot.trackMinQuality01;
        trackBlock.qualityTauSec[id] = resolved.guidanceAutopilot.trackQualityTauSec;
        trackBlock.velocityBlend[id] = resolved.guidanceAutopilot.trackVelocityBlend;
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
        trackBlock.kfCov[id] = {};
        trackBlock.retargetCandidateId[id] = -1;
        trackBlock.retargetCount[id] = 0;
        trackBlock.lastInnovationM[id] = 0.0;
        trackBlock.residualRejectCount[id] = 0;

        // Fresh and reused slots: clear the navigation filter so a recycled
        // entity ID re-aligns from truth instead of inheriting the previous
        // occupant's covariance, biases and alignment flag.
        navigationSystem.resetEntity(navigationBlock, id);
        // Same contract for the sensor layer: random-walk biases, cadence
        // phase and queued GPS samples must not leak across slot reuse.
        sensorSystem.resetEntity(id);

        sensorBlock.accelNoiseStdDev[id] = resolved.sensor.accelNoiseStdDev;
        sensorBlock.accelBiasStdDev[id] = resolved.sensor.accelBiasStdDev;
        sensorBlock.gyroNoiseStdDev[id] = resolved.sensor.gyroNoiseStdDev;
        sensorBlock.gyroBiasStdDev[id] = resolved.sensor.gyroBiasStdDev;
        sensorBlock.gpsPosNoiseStdDev[id] = resolved.sensor.gpsPosNoiseStdDev;
        sensorBlock.gpsVelNoiseStdDev[id] = resolved.sensor.gpsVelNoiseStdDev;
        sensorBlock.gpsInnovationGateSigma[id] = resolved.sensor.gpsInnovationGateSigma;
        sensorBlock.baroInnovationGateSigma[id] = resolved.sensor.baroInnovationGateSigma;
        sensorBlock.magInnovationGateSigma[id] = resolved.sensor.magInnovationGateSigma;
        sensorBlock.imuLeverArmX[id] = resolved.sensor.imuLeverArmX;
        sensorBlock.imuLeverArmY[id] = resolved.sensor.imuLeverArmY;
        sensorBlock.imuLeverArmZ[id] = resolved.sensor.imuLeverArmZ;
        sensorBlock.imuEnabled[id] = resolved.sensor.imuEnabled;
        sensorBlock.gpsEnabled[id] = resolved.sensor.gpsEnabled;
        sensorBlock.gpsUpdateRateHz[id] = resolved.sensor.gpsUpdateRateHz;
        sensorBlock.baroEnabled[id] = resolved.sensor.baroEnabled;
        sensorBlock.baroNoiseStdDev[id] = resolved.sensor.baroNoiseStdDev;
        sensorBlock.baroBiasStdDev[id] = resolved.sensor.baroBiasStdDev;
        sensorBlock.baroUpdateRateHz[id] = resolved.sensor.baroUpdateRateHz;
        sensorBlock.magEnabled[id] = resolved.sensor.magEnabled;
        sensorBlock.magNoiseStdDev[id] = resolved.sensor.magNoiseStdDev;
        sensorBlock.magUpdateRateHz[id] = resolved.sensor.magUpdateRateHz;
        sensorBlock.magDisturbanceGateRel[id] = resolved.sensor.magDisturbanceGateRel;
        sensorBlock.gpsLatencySec[id] = resolved.sensor.gpsLatencySec;
        sensorBlock.gpsLeverArmX[id] = resolved.sensor.gpsLeverArmX;
        sensorBlock.gpsLeverArmY[id] = resolved.sensor.gpsLeverArmY;
        sensorBlock.gpsLeverArmZ[id] = resolved.sensor.gpsLeverArmZ;
        sensorBlock.gpsFixConsistencyEnabled[id] = resolved.sensor.gpsFixConsistencyEnabled;
        sensorBlock.insConingCompensationEnabled[id] = resolved.sensor.insConingCompensationEnabled;
        sensorBlock.insAdaptiveQEnabled[id] = resolved.sensor.insAdaptiveQEnabled;
        sensorBlock.insAdaptiveQGain[id] = resolved.sensor.insAdaptiveQGain;
        sensorBlock.initialAttitudeErrorDeg[id] = resolved.sensor.initialAttitudeErrorDeg;
        sensorBlock.initialPositionErrorM[id] = resolved.sensor.initialPositionErrorM;
        sensorBlock.initialVelocityErrorMps[id] = resolved.sensor.initialVelocityErrorMps;
        sensorBlock.insGravityGradientEnabled[id] = resolved.sensor.insGravityGradientEnabled;
        sensorBlock.insEarthRotationCouplingEnabled[id] = resolved.sensor.insEarthRotationCouplingEnabled;
        sensorBlock.gpsBatchUpdateEnabled[id] = resolved.sensor.gpsBatchUpdateEnabled;
        sensorBlock.gpsLeverArmCompensationEnabled[id] = resolved.sensor.gpsLeverArmCompensationEnabled;
        sensorBlock.gpsYawCorrectionDamping[id] = resolved.sensor.gpsYawCorrectionDamping;
        sensorBlock.gpsFixConsistencyThreshold[id] = resolved.sensor.gpsFixConsistencyThreshold;
        sensorBlock.gpsFixConsistencyConfidence[id] = resolved.sensor.gpsFixConsistencyConfidence;
        sensorBlock.gpsFixConsistencyDof[id] = resolved.sensor.gpsFixConsistencyDof;
        sensorBlock.maxAccelBiasEstimate[id] = resolved.sensor.maxAccelBiasEstimate;
        sensorBlock.maxGyroBiasEstimate[id] = resolved.sensor.maxGyroBiasEstimate;
        sensorBlock.baroAttitudeCorrectionEnabled[id] = resolved.sensor.baroAttitudeCorrectionEnabled;

        physicsBlock.px[id] = init.px; physicsBlock.py[id] = init.py; physicsBlock.pz[id] = init.pz;
        physicsBlock.vx[id] = init.vx; physicsBlock.vy[id] = init.vy; physicsBlock.vz[id] = init.vz;
        physicsBlock.qw[id] = init.qw; physicsBlock.qx[id] = init.qx; physicsBlock.qy[id] = init.qy; physicsBlock.qz[id] = init.qz;
        physicsBlock.wx[id] = init.wx; physicsBlock.wy[id] = init.wy; physicsBlock.wz[id] = init.wz;
        physicsBlock.alphax[id] = 0.0; physicsBlock.alphay[id] = 0.0; physicsBlock.alphaz[id] = 0.0;
        // Cached truth accel + last aiding readouts must not leak from the
        // removed occupant: accel refreshes on the first post-step, but stale
        // baro/mag readouts would persist until the next aiding update.
        physicsBlock.ax[id] = 0.0; physicsBlock.ay[id] = 0.0; physicsBlock.az[id] = 0.0;
        sensorBlock.baroUpdated[id] = false;
        sensorBlock.baroAlt[id] = 0.0;
        sensorBlock.magUpdated[id] = false;
        sensorBlock.magX[id] = 0.0; sensorBlock.magY[id] = 0.0; sensorBlock.magZ[id] = 0.0;
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
        // Aircraft airframe geometry -> AirframeParams (DATCOM-light wing-body-tail).
        physicsBlock.airframe[id] = nullptr;
        if (resolved.aero.airframe.enabled()) {
            const auto& af = resolved.aero.airframe;
            auto ap = Models::buildAirframeParams(
                af.wingSpanM, af.wingRootChordM, af.wingTipChordM,
                af.wingSweepDeg, af.wingPositionM, af.wingDihedralDeg,
                af.htailSpanM, af.htailChordM, af.htailPositionM,
                af.vtailSpanM, af.vtailChordM, af.vtailPositionM,
                af.fuselageDiameterM, af.fuselageLengthM,
                af.cd0, af.oswaldEfficiency, af.clMax);
            if (!ap) {
                throw std::runtime_error("Invalid aircraft airframe configuration");
            }
            physicsBlock.airframe[id] = ap;
        }
        physicsBlock.finPitch[id] = 0.0; physicsBlock.finYaw[id] = 0.0; physicsBlock.finRoll[id] = 0.0;
        physicsBlock.ignitionTime[id] = time.currentTime();
        physicsBlock.active[id] = true;
        physicsBlock.motorFailed[id] = false;
        physicsBlock.engineFailed[id] = false;
        physicsBlock.tankFailed[id] = false;
        physicsBlock.actuatorFailed[id] = false;

        statusBlock.type[id] = config.type;
        statusBlock.allegiance[id] = init.allegiance;
        statusBlock.health[id] = resolved.structuralHardness;
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
        statusBlock.jammerEirpW[id] = config.jammerEirpW;

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
        warheads[id].fuseEnabled = resolved.warhead.fuseEnabled;
        warheads[id].cpaFuzingEnabled = resolved.warhead.cpaFuzingEnabled;
        warheads[id].fuseLookaheadSec = resolved.warhead.fuseLookaheadSec;
        warheads[id].armingDelaySec = resolved.warhead.armingDelaySec;
        warheads[id].minClosingSpeedMps = resolved.warhead.minClosingSpeedMps;
        warheads[id].selfDestructTimeSec = resolved.warhead.selfDestructTimeSec;
        warheads[id].damage = resolved.warhead.damage;
        warheads[id].fuseDetectionProbability = resolved.warhead.fuseDetectionProbability;
        warheads[id].headOnLethalityFactor = resolved.warhead.headOnLethalityFactor;
        warheads[id].tailOnLethalityFactor = resolved.warhead.tailOnLethalityFactor;
        // Explicit-fuse footgun: an enabled proximity fuse with a
        // non-positive trigger radius can never fire. Fail fast instead of
        // building a silently inert warhead (set fuse_enabled=false to
        // intentionally disable).
        if (warheads[id].fuseEnabled && warheads[id].fusing == FusingType::Proximity &&
            warheads[id].proximityTriggerM <= 0.0) {
            throw std::runtime_error(
                "SimulationKernel: proximity warhead has fuse_enabled=true but "
                "proximity_trigger_m <= 0 (set fuse_enabled=false to disable)");
        }

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
        seekerBlock.measurementNoiseEnabled[id] = resolved.seeker.measurementNoiseEnabled;
        seekerBlock.angleNoiseStdDevRad[id] = resolved.seeker.angleNoiseStdDevRad;
        seekerBlock.angleNoiseRefSnrDb[id] = resolved.seeker.angleNoiseRefSnrDb;
        seekerBlock.rangeNoiseStdDevM[id] = resolved.seeker.rangeNoiseStdDevM;
        seekerBlock.rangeRateNoiseStdDevMps[id] = resolved.seeker.rangeRateNoiseStdDevMps;
        seekerBlock.glintSigmaM[id] = resolved.seeker.glintSigmaM;
        seekerBlock.glintCorrelationTauSec[id] = resolved.seeker.glintCorrelationTauSec;
        seekerBlock.swerlingEnabled[id] = resolved.seeker.swerlingEnabled;
        seekerBlock.gimbalRateLimitRadPerSec[id] = resolved.seeker.gimbalRateLimitRadPerSec;
        seekerBlock.minRangeGateM[id] = resolved.seeker.minRangeGateM;
        seekerBlock.maxRangeGateM[id] = resolved.seeker.maxRangeGateM;
        seekerBlock.terrainMaskingEnabled[id] = resolved.seeker.terrainMaskingEnabled;
        seekerBlock.minClosingRateMps[id] = resolved.seeker.minClosingRateMps;
        seekerBlock.rateFilterTauSec[id] = resolved.seeker.rateFilterTauSec;
        seekerBlock.decoyRejectionDb[id] = resolved.seeker.decoyRejectionDb;
        seekerBlock.passiveRfDutyCycle[id] = resolved.seeker.passiveRfDutyCycle;
        seekerBlock.illuminatorEntityId[id] = resolved.seeker.illuminatorEntityId;
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
        seekerBlock.lockActive[id] = false;
        seekerBlock.hasPublishedMeasurement[id] = false;
        seekerBlock.measurementAgeSec[id] = 0.0;
        seekerBlock.gimbalAzimuthRad[id] = 0.0;
        seekerBlock.gimbalElevationRad[id] = 0.0;
        seekerBlock.lastSignalStrength[id] = 0.0;
        seekerBlock.lockRejectReason[id] = static_cast<int>(SeekerRejectReason::None);
        seekerBlock.glintAzM[id] = 0.0;
        seekerBlock.glintElM[id] = 0.0;

        return id;
    }

    void SimulationKernel::removeVehicle(PhysicsId id) {
        if (id >= physicsBlock.size || !physicsBlock.active[id]) return;
        physicsBlock.active[id] = false;
        statusBlock.isAlive[id] = false;
        // A despawn is not a kill: disarm the warhead, otherwise an Impact
        // fuse (triggered by !isAlive) or a Proximity/Timed fuse detonates
        // the removed slot on a later step and rolls lethality against live
        // opponents (phantom kill + spurious Detonation event). Structural
        // kills keep their detonation — they go through failEntity, not here.
        if (id < warheads.size()) {
            warheads[id].detonated = true;
            warheads[id].lethalRadiusM = 0.0;
            warheads[id].falloffRadiusM = 0.0;
        }
        // Stale queued commands must not re-arm whatever reuses the slot.
        commandProcessor.dropCommandsFor(id);
        // Return the removed entity's registered propulsion models to the
        // backend pool so repeated spawn/despawn cycles do not grow it.
        if (id < stagePlans.size()) {
            for (const int poolId : stagePlans[id].poolIds) {
                backend->releasePropulsion(poolId);
            }
        }
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
            if (si < 0 || si >= physicsBlock.stageCount[i]) continue;
            const StagePlan& plan = stagePlans[i];
            if (si >= static_cast<int>(plan.poolIds.size())) continue;

            // Burnout: the active stage's thrust curve has fully elapsed, or
            // the stage's propellant cap has been drawn down to its
            // stage-aware floor (mass can no longer decrease).
            const bool curveElapsed =
                si < static_cast<int>(plan.burnDurations.size()) &&
                time.currentTime() - physicsBlock.ignitionTime[i] >= plan.burnDurations[si];
            const bool propellantExhausted =
                si < static_cast<int>(plan.propellantCaps.size()) &&
                plan.propellantCaps[si] > 0.0 &&
                physicsBlock.mass[i] <= physicsBlock.stageMinMass[i] + 1e-9;
            if (!curveElapsed && !propellantExhausted) {
                continue;
            }

            // Final-stage burnout has no separation; it reports MotorBurnout
            // once so the event stream covers the whole burn (previously the
            // last stage ended silently).
            if (si + 1 >= physicsBlock.stageCount[i]) {
                if (!plan.burnoutReported) {
                    SimulationEvent evt;
                    evt.type = EventType::MotorBurnout;
                    evt.entityId = i;
                    evt.timestamp = time.currentTime();
                    eventSystem.dispatch(evt);
                    stagePlans[i].burnoutReported = true;
                }
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

    const WarheadState& SimulationKernel::getWarhead(PhysicsId id) const {
        if (id >= warheads.size()) {
            throw std::out_of_range("SimulationKernel::getWarhead: entity id out of range");
        }
        return warheads[id];
    }

    void SimulationKernel::processWarheads() {
        // Analytic closest-approach projection of the relative state onto
        // the miss vector. Returns {time-to-CPA (clamped >= 0), CPA range}.
        const auto projectCpa = [](double dx, double dy, double dz,
                                   double dvx, double dvy, double dvz) {
            const double rdotv = dx * dvx + dy * dvy + dz * dvz;
            const double v2 = dvx * dvx + dvy * dvy + dvz * dvz;
            double tcpa = 0.0;
            if (v2 > 1e-12) tcpa = std::max(0.0, -rdotv / v2);
            const double mx = dx + dvx * tcpa;
            const double my = dy + dvy * tcpa;
            const double mz = dz + dvz * tcpa;
            return std::pair<double, double>{tcpa, std::sqrt(mx*mx + my*my + mz*mz)};
        };
        for (std::size_t i = 0; i < warheads.size(); ++i) {
            WarheadState& wh = warheads[i];
            if (wh.detonated || wh.lethalRadiusM <= 0.0) continue;
            if (!wh.fuseEnabled) continue;
            // A dead carrier keeps only its death burst: Impact fusing fires
            // on the kill itself, but Proximity/Timed/self-destruct must not
            // trigger from a dead body (zombie detonations), regardless of
            // which path killed the carrier.
            if (!statusBlock.isAlive[i] && wh.fusing != FusingType::Impact) continue;

            const double tof = time.currentTime() - wh.launchTime;
            const bool armed = tof >= wh.armingDelaySec;

            bool trigger = false;
            std::size_t targetId = 0;
            bool haveTarget = false;
            double predictedCpaM = 0.0;

            if (armed) {
                switch (wh.fusing) {
                    case FusingType::Impact:
                        trigger = !statusBlock.isAlive[i];
                        break;
                    case FusingType::Proximity: {
                        if (wh.proximityTriggerM > 0.0) {
                            const double r2 = wh.proximityTriggerM * wh.proximityTriggerM;
                            if (wh.cpaFuzingEnabled) {
                                // CPA fuzing: fire on the hostile whose
                                // projected miss is smallest inside the
                                // lookahead window, and score the kill on
                                // that miss so a fast pass is not penalized
                                // by the pre-CPA range. Selection is by miss,
                                // not by time: a past-CPA hostile (tcpa
                                // clamped to 0) must not beat a genuinely
                                // closing threat.
                                double bestCpa = std::numeric_limits<double>::infinity();
                                std::size_t bestId = 0;
                                bool found = false;
                                for (std::size_t j = 0; j < physicsBlock.size; ++j) {
                                    if (j == i || !statusBlock.isAlive[j]) continue;
                                    if (statusBlock.allegiance[i] == statusBlock.allegiance[j]) continue;
                                    const double dx = physicsBlock.px[j] - physicsBlock.px[i];
                                    const double dy = physicsBlock.py[j] - physicsBlock.py[i];
                                    const double dz = physicsBlock.pz[j] - physicsBlock.pz[i];
                                    const double dist2 = dx*dx + dy*dy + dz*dz;
                                    if (dist2 > r2) continue;
                                    const double dvx = physicsBlock.vx[j] - physicsBlock.vx[i];
                                    const double dvy = physicsBlock.vy[j] - physicsBlock.vy[i];
                                    const double dvz = physicsBlock.vz[j] - physicsBlock.vz[i];
                                    const double dist = std::sqrt(dist2);
                                    const double closing = dist > 1e-9
                                        ? -(dx*dvx + dy*dvy + dz*dvz) / dist : 0.0;
                                    if (wh.minClosingSpeedMps > 0.0 && closing < wh.minClosingSpeedMps) continue;
                                    const auto [tcpa, cpaDist] = projectCpa(dx, dy, dz, dvx, dvy, dvz);
                                    if (tcpa > wh.fuseLookaheadSec) continue;
                                    if (cpaDist < bestCpa) {
                                        bestCpa = cpaDist;
                                        bestId = j;
                                        found = true;
                                    }
                                }
                                if (found) {
                                    bool detected = true;
                                    if (wh.fuseDetectionProbability < 1.0) {
                                        detected = std::uniform_real_distribution<double>(0.0, 1.0)(fuzeRng)
                                            < wh.fuseDetectionProbability;
                                    }
                                    if (detected) {
                                        trigger = true;
                                        targetId = bestId;
                                        haveTarget = true;
                                        predictedCpaM = bestCpa;
                                    }
                                }
                            } else {
                                // Legacy proximity fuse (0.02 s range-rate
                                // lookahead + instantaneous-miss PK).
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
                                        if (wh.minClosingSpeedMps > 0.0) {
                                            const double closing = dist > 1e-9 ? -rdotv / dist : 0.0;
                                            if (closing < wh.minClosingSpeedMps) continue;
                                        }
                                        if (rdotv >= 0.0 || dist <= 2.0 || (dist - vrel * 0.02) <= 0.0) {
                                            bool detected = true;
                                            if (wh.fuseDetectionProbability < 1.0) {
                                                detected = std::uniform_real_distribution<double>(0.0, 1.0)(fuzeRng)
                                                    < wh.fuseDetectionProbability;
                                            }
                                            if (!detected) continue;
                                            trigger = true;
                                            targetId = j;
                                            haveTarget = true;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        break;
                    }
                    case FusingType::Timed:
                        trigger = tof >= wh.timedDelaySec;
                        break;
                }
            }

            // Self-destruct on timer expiry (only when the primary fuse did
            // not trigger this step). Destroys the carrier; no lethality.
            if (!trigger) {
                if (wh.selfDestructTimeSec > 0.0 && tof >= wh.selfDestructTimeSec) {
                    wh.detonated = true;
                    SimulationEvent evt;
                    evt.type = EventType::Detonation;
                    evt.entityId = i;
                    evt.timestamp = time.currentTime();
                    evt.selfDestruct = true;
                    eventSystem.dispatch(evt);
                    failEntity(i, FailureMode::StructuralFailure);
                }
                continue;
            }

            // Lethality with an optional fragmentation/overpressure falloff
            // band (guaranteed kill inside the lethal radius, linear decay to
            // the falloff edge). The RNG draw happens only for genuinely
            // uncertain outcomes (0 < p < 1), so flat-law warheads never
            // consume the kernel stream. Hostiles are evaluated in index
            // order; results are collected first so the event payload and
            // the kill rolls share one pass.
            struct HitResult {
                std::size_t id;
                double miss;
                double prob;
                bool kill;
            };
            std::vector<HitResult> hits;
            const double vmx = physicsBlock.vx[i];
            const double vmy = physicsBlock.vy[i];
            const double vmz = physicsBlock.vz[i];
            const double vm2 = vmx*vmx + vmy*vmy + vmz*vmz;
            for (std::size_t j = 0; j < physicsBlock.size; ++j) {
                if (j == i || !statusBlock.isAlive[j]) continue;
                if (statusBlock.allegiance[i] == statusBlock.allegiance[j]) continue;
                const double dx = physicsBlock.px[j] - physicsBlock.px[i];
                const double dy = physicsBlock.py[j] - physicsBlock.py[i];
                const double dz = physicsBlock.pz[j] - physicsBlock.pz[i];
                double missDistance;
                if (wh.cpaFuzingEnabled) {
                    const double dvx = physicsBlock.vx[j] - physicsBlock.vx[i];
                    const double dvy = physicsBlock.vy[j] - physicsBlock.vy[i];
                    const double dvz = physicsBlock.vz[j] - physicsBlock.vz[i];
                    missDistance = projectCpa(dx, dy, dz, dvx, dvy, dvz).second;
                } else {
                    missDistance = std::sqrt(dx*dx + dy*dy + dz*dz);
                }
                // Aspect-dependent lethality: fragments/overpressure are
                // interpolated between head-on and tail-on from the angle
                // between carrier and target velocity. Defaults keep 1.0.
                double factor = 1.0;
                if (wh.headOnLethalityFactor != 1.0 || wh.tailOnLethalityFactor != 1.0) {
                    const double vtx = physicsBlock.vx[j];
                    const double vty = physicsBlock.vy[j];
                    const double vtz = physicsBlock.vz[j];
                    const double vt2 = vtx*vtx + vty*vty + vtz*vtz;
                    if (vm2 > 1e-12 && vt2 > 1e-12) {
                        const double cosAspect = (vmx*vtx + vmy*vty + vmz*vtz) /
                            std::sqrt(vm2 * vt2);
                        factor = 0.5 * (wh.headOnLethalityFactor + wh.tailOnLethalityFactor) +
                            0.5 * (wh.headOnLethalityFactor - wh.tailOnLethalityFactor) * (-cosAspect);
                    }
                }
                const double prob = Models::warheadKillProbability(
                    missDistance,
                    wh.lethalRadiusM * factor,
                    wh.falloffRadiusM * factor);
                bool kill = (prob >= 1.0) ||
                    (prob > 0.0 && prob >= std::uniform_real_distribution<double>(0.0, 1.0)(warheadRng));
                hits.push_back(HitResult{j, missDistance, prob, kill});
            }

            // Primary target telemetry: the fuse-engaged threat, else the
            // closest hostile in the evaluated set.
            const HitResult* primary = nullptr;
            if (haveTarget) {
                for (const auto& h : hits) {
                    if (h.id == targetId) { primary = &h; break; }
                }
            }
            if (!primary) {
                for (const auto& h : hits) {
                    if (!primary || h.miss < primary->miss) primary = &h;
                }
            }

            wh.detonated = true;
            SimulationEvent evt;
            evt.type = EventType::Detonation;
            evt.entityId = i;
            evt.timestamp = time.currentTime();
            if (primary) {
                evt.targetEntityId = primary->id;
                evt.missDistanceM = primary->miss;
                evt.killProbability = primary->prob;
            }
            if (wh.cpaFuzingEnabled) {
                evt.predictedCpaM = haveTarget ? predictedCpaM
                    : (primary ? primary->miss : 0.0);
            }
            for (const auto& h : hits) {
                if (h.kill) { evt.kill = true; break; }
            }
            eventSystem.dispatch(evt);

            // A detonating warhead consumes its own carrier. Impact-fused
            // detonations find the carrier already dead, so the guard keeps
            // those a no-op.
            if (statusBlock.isAlive[i]) {
                statusBlock.isAlive[i] = false;
                statusBlock.health[i] = 0.0;
                physicsBlock.active[i] = false;
                physicsBlock.vx[i] = 0.0;
                physicsBlock.vy[i] = 0.0;
                physicsBlock.vz[i] = 0.0;
                physicsBlock.ax[i] = 0.0;
                physicsBlock.ay[i] = 0.0;
                physicsBlock.az[i] = 0.0;
            }

            if (primary) {
                wh.lastTargetId = primary->id;
                wh.lastMissDistanceM = primary->miss;
                wh.lastKillProbability = primary->prob;
            }
            wh.lastPredictedCpaM = evt.predictedCpaM;
            wh.lastKill = evt.kill;

            const double damage = wh.damage > 0.0 ? wh.damage : 100.0;
            for (const auto& h : hits) {
                if (h.kill) applyDamage(h.id, damage);
            }
        }
    }

    void SimulationKernel::queueCommand(const SimulationCommand& cmd) {
        commandProcessor.enqueueCommand(cmd);
    }

    void SimulationKernel::addPendingLaunch(const ScenarioEntityConfig& entityCfg) {
        PendingLaunch pl;
        pl.cfg = entityCfg;
        pendingLaunches.push_back(std::move(pl));
    }

    void SimulationKernel::queueInitialGuidance(PhysicsId id, const ScenarioEntityConfig& cfg) {
        if (cfg.initialGuidanceMode == GuidanceMode::None) return;
        SimulationCommand cmd;
        cmd.entityId = id;
        cmd.mode = cfg.initialGuidanceMode;
        // Seed the aim from the launcher's relayed datalink track when one
        // exists (its solution is strictly better than the stale pre-launch
        // seed); fall back to the entity's configured seed.
        const std::size_t src = static_cast<std::size_t>(
            cfg.vehicleConfig.guidanceAutopilot.datalinkSourceId);
        bool seeded = false;
        if (cfg.vehicleConfig.guidanceAutopilot.datalinkSourceId >= 0 &&
            src < trackBlock.size && trackBlock.updateCount[src] > 0) {
            cmd.targetX = trackBlock.posX[src];
            cmd.targetY = trackBlock.posY[src];
            cmd.targetZ = trackBlock.posZ[src];
            cmd.targetVx = trackBlock.velX[src];
            cmd.targetVy = trackBlock.velY[src];
            cmd.targetVz = trackBlock.velZ[src];
            cmd.targetAccelX = trackBlock.accelX[src];
            cmd.targetAccelY = trackBlock.accelY[src];
            cmd.targetAccelZ = trackBlock.accelZ[src];
            cmd.targetAccelAvailable = trackBlock.accelAvailable[src];
            seeded = true;
        }
        if (!seeded) {
            cmd.targetX = cfg.initialTargetX;
            cmd.targetY = cfg.initialTargetY;
            cmd.targetZ = cfg.initialTargetZ;
            cmd.targetVx = cfg.initialTargetVx;
            cmd.targetVy = cfg.initialTargetVy;
            cmd.targetVz = cfg.initialTargetVz;
            cmd.targetAccelX = cfg.initialTargetAccelX;
            cmd.targetAccelY = cfg.initialTargetAccelY;
            cmd.targetAccelZ = cfg.initialTargetAccelZ;
            cmd.targetAccelAvailable = cfg.initialTargetAccelAvailable;
        }
        cmd.maxAccel = cfg.initialMaxAccel;
        cmd.targetId = cfg.initialTargetId;
        queueCommand(cmd);
    }

    // Spawn one pending rail-launched entity: rail-release geometry relative
    // to its parent (drop along local up, small push, parent attitude, zero
    // rates), then its initial guidance command — exactly the sequence
    // ScenarioConfig::loadInto uses for t=0 entities.
    PhysicsId SimulationKernel::spawnPendingLaunch(PendingLaunch& pl) {
        const auto& spec = pl.cfg.launch;
        const std::size_t parent = spec.parentIndex;
        if (parent >= physicsBlock.size || !physicsBlock.active[parent]) {
            return std::numeric_limits<PhysicsId>::max();
        }

        // Local up: geodetic nadir under ECEF truth, +Z otherwise.
        double ux = 0.0, uy = 0.0, uz = 1.0;
        if (environment.earth.useEcefTruth) {
            const auto geo = Models::ecefToGeodetic(
                {physicsBlock.px[parent], physicsBlock.py[parent], physicsBlock.pz[parent]});
            const double cLat = std::cos(geo.latitudeRad), sLat = std::sin(geo.latitudeRad);
            const double cLon = std::cos(geo.longitudeRad), sLon = std::sin(geo.longitudeRad);
            ux = cLat * cLon; uy = cLat * sLon; uz = sLat;
        }

        VehicleInitState init;
        init.px = physicsBlock.px[parent] - ux * spec.dropM;
        init.py = physicsBlock.py[parent] - uy * spec.dropM;
        init.pz = physicsBlock.pz[parent] - uz * spec.dropM;
        init.vx = physicsBlock.vx[parent] - ux * spec.pushMps;
        init.vy = physicsBlock.vy[parent] - uy * spec.pushMps;
        init.vz = physicsBlock.vz[parent] - uz * spec.pushMps;
        init.qw = physicsBlock.qw[parent]; init.qx = physicsBlock.qx[parent];
        init.qy = physicsBlock.qy[parent]; init.qz = physicsBlock.qz[parent];
        init.wx = 0.0; init.wy = 0.0; init.wz = 0.0;
        init.mass = (pl.cfg.initState.mass > 0.0)
            ? pl.cfg.initState.mass : pl.cfg.vehicleConfig.initialMass;
        init.allegiance = pl.cfg.initState.allegiance;

        const PhysicsId id = createVehicle(init, pl.cfg.vehicleConfig);

        // Separation flyout: hold a straight-ahead Waypoint for flyoutSec
        // (the vehicle's own gentle waypointGain shapes the demand), then
        // switch to the initial guidance command. The aim direction is the
        // PARENT's velocity (the rail heading) — not the pushed spawn
        // velocity: the separation push tilts the round ~1 deg off the rail
        // axis, and thrust-vector ignition amplifies that into a divergent
        // terminal geometry.
        if (spec.flyoutSec > 0.0 && pl.cfg.initialGuidanceMode != GuidanceMode::None) {
            double sx = physicsBlock.vx[parent], sy = physicsBlock.vy[parent],
                   sz = physicsBlock.vz[parent];
            const double spd = std::sqrt(sx * sx + sy * sy + sz * sz);
            if (spd > 1.0) { sx /= spd; sy /= spd; sz /= spd; }
            else { sx = ux; sy = uy; sz = uz; }
            SimulationCommand wp;
            wp.entityId = id;
            wp.mode = GuidanceMode::Waypoint;
            wp.targetX = init.px + sx * spec.flyoutAheadM + ux * spec.flyoutClimbM;
            wp.targetY = init.py + sy * spec.flyoutAheadM + uy * spec.flyoutClimbM;
            wp.targetZ = init.pz + sz * spec.flyoutAheadM + uz * spec.flyoutClimbM;
            wp.maxAccel = pl.cfg.initialMaxAccel;
            wp.targetId = -1;
            queueCommand(wp);
            ActiveFlyout fo;
            fo.entityId = id;
            fo.switchTime = time.currentTime() + spec.flyoutSec;
            fo.cfg = pl.cfg;
            activeFlyouts.push_back(std::move(fo));
        } else {
            queueInitialGuidance(id, pl.cfg);
        }

        SimulationEvent evt;
        evt.timestamp = time.currentTime();
        evt.entityId = id;
        evt.type = EventType::Custom;
        evt.customCode = 1;  // scenario rail launch (see app-side labeling)
        eventSystem.dispatch(evt);
        return id;
    }

    // Evaluate pending rail launches: lock-hold on the parent's seeker plus
    // an optional slant-range gate. Runs before the systems update so the
    // spawned entity participates in the current step.
    void SimulationKernel::processPendingLaunches() {
        if (pendingLaunches.empty()) return;
        for (std::size_t i = 0; i < pendingLaunches.size();) {
            auto& pl = pendingLaunches[i];
            const auto& spec = pl.cfg.launch;
            const std::size_t parent = spec.parentIndex;
            const std::size_t tgt = spec.targetIndex;
            if (parent >= physicsBlock.size || !physicsBlock.active[parent] ||
                (spec.targetIndex >= 0 &&
                 (tgt >= physicsBlock.size || !physicsBlock.active[tgt]))) {
                ++i;
                continue;
            }

            bool locked = parent < seekerBlock.isLocked.size() && seekerBlock.isLocked[parent];
            if (locked && spec.targetIndex >= 0) {
                locked = (parent < seekerBlock.lockedTargetId.size() &&
                          seekerBlock.lockedTargetId[parent] ==
                              static_cast<std::int64_t>(tgt));
            }
            if (!locked) {
                pl.lockSince = -1.0;
                ++i;
                continue;
            }
            const double t = time.currentTime();
            if (pl.lockSince < 0.0) pl.lockSince = t;
            if (t - pl.lockSince < spec.lockHoldSec) {
                ++i;
                continue;
            }
            if (spec.rangeGateM > 0.0 && spec.targetIndex >= 0) {
                const double dx = physicsBlock.px[tgt] - physicsBlock.px[parent];
                const double dy = physicsBlock.py[tgt] - physicsBlock.py[parent];
                const double dz = physicsBlock.pz[tgt] - physicsBlock.pz[parent];
                if (std::sqrt(dx * dx + dy * dy + dz * dz) > spec.rangeGateM) {
                    ++i;
                    continue;
                }
            }
            spawnPendingLaunch(pl);
            pendingLaunches.erase(pendingLaunches.begin() + static_cast<std::ptrdiff_t>(i));
        }
    }

    // End separation flyouts whose timer expired: hand the round to its
    // initial guidance law with the launcher's freshest relayed track.
    void SimulationKernel::processActiveFlyouts() {
        if (activeFlyouts.empty()) return;
        const double t = time.currentTime();
        for (std::size_t i = 0; i < activeFlyouts.size();) {
            auto& fo = activeFlyouts[i];
            if (fo.entityId >= physicsBlock.size || !physicsBlock.active[fo.entityId]) {
                activeFlyouts.erase(activeFlyouts.begin() + static_cast<std::ptrdiff_t>(i));
                continue;
            }
            if (t < fo.switchTime) {
                ++i;
                continue;
            }
            queueInitialGuidance(fo.entityId, fo.cfg);
            activeFlyouts.erase(activeFlyouts.begin() + static_cast<std::ptrdiff_t>(i));
        }
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
        // Rail launches + flyout switches run BEFORE the clock advances, at
        // the same evaluation point the old app-side directors used. Spawning
        // after time.advance shifted every time-gated event (motor ignition,
        // pulse timing) by one tick, which the marginal terminal fight
        // amplifies into a materially different engagement.
        processPendingLaunches();
        processActiveFlyouts();
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
        seekerSystem.update(physicsBlock, statusBlock, seekerBlock, navigationBlock, dt, environment);

        // 3.75 Persistent target-track manager (seeker measurements + command
        // seeds -> estimate; prediction at the sim rate between measurements)
        trackManagerSystem.update(navigationBlock, seekerBlock, trackBlock,
                                  time.currentTime(), dt);

        // 4. Update Guidance based on estimates, tracks, and seekers
        guidanceSystem.update(statusBlock, navigationBlock, seekerBlock,
                              trackBlock, guidanceBlock, controlBlock, dt,
                              environment);

        // 4.5 Update Autopilot to translate commanded accel to fin deflections
        autopilotSystem.update(statusBlock, navigationBlock, sensorBlock, guidanceBlock, controlBlock, dt, environment);
        
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
