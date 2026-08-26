#include <strikeengine/kernel/SimulationKernel.hpp>
#include <strikeengine/kernel/backend/BackendFactory.hpp>
#include <strikeengine/kernel/backend/CPUBackend.hpp>
#include <strikeengine/kernel/integrator/RK4Integrator.hpp>
#include <strikeengine/models/physics/atmosphere/ISA1976.hpp>
#include <strikeengine/models/physics/aerodynamics/AeroModel.hpp>
#include <strikeengine/models/physics/propulsion/PropulsionModel.hpp>
#include <stdexcept>

namespace StrikeEngine::Kernel {

    SimulationKernel::SimulationKernel(BackendType backendType) {
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

            auto integrator = std::make_unique<RK4Integrator>();

            backend = std::make_unique<CPUBackend>(
                std::move(integrator),
                atmosphere,
                aero
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

    void SimulationKernel::reset() {
        physicsBlock = PhysicsBlock();
        controlBlock = ControlBlock();
        guidanceBlock = GuidanceBlock();
        statusBlock = EntityStatusBlock();
        sensorBlock = SensorBlock();
        navigationBlock = NavigationBlock();
        seekerBlock = SeekerBlock();
        seekerSystem.reset();
        freeList.clear();
        time.reset();
    }

    PhysicsId SimulationKernel::createVehicle(const VehicleInitState& init) {
        return createVehicle(init, VehicleConfig{});
    }

    PhysicsId SimulationKernel::createVehicle(const VehicleInitState& init, const VehicleConfig& config) {
        PhysicsId id;

        if (!freeList.empty()) {
            id = freeList.back();
            freeList.pop_back();
        } else {
            id = physicsBlock.size++;
            
            // Resize arrays
            physicsBlock.px.push_back(0); physicsBlock.py.push_back(0); physicsBlock.pz.push_back(0);
            physicsBlock.vx.push_back(0); physicsBlock.vy.push_back(0); physicsBlock.vz.push_back(0);
            physicsBlock.ax.push_back(0); physicsBlock.ay.push_back(0); physicsBlock.az.push_back(0);
            physicsBlock.qw.push_back(1); physicsBlock.qx.push_back(0); physicsBlock.qy.push_back(0); physicsBlock.qz.push_back(0);
            physicsBlock.wx.push_back(0); physicsBlock.wy.push_back(0); physicsBlock.wz.push_back(0);
            physicsBlock.alphax.push_back(0); physicsBlock.alphay.push_back(0); physicsBlock.alphaz.push_back(0);
            physicsBlock.Ixx.push_back(1.0); physicsBlock.Iyy.push_back(10.0); physicsBlock.Izz.push_back(10.0);
            physicsBlock.mass.push_back(1.0);
            physicsBlock.massDry.push_back(1.0);
            physicsBlock.referenceArea.push_back(0.1);
            physicsBlock.referenceLength.push_back(1.0);
            physicsBlock.cd.push_back(0.3);
            physicsBlock.clAlpha.push_back(0.0);
            physicsBlock.clFin.push_back(0.0);
            physicsBlock.clMax.push_back(2.0);
            physicsBlock.propulsionId.push_back(-1);
            physicsBlock.ignitionTime.push_back(0.0);
            physicsBlock.finPitch.push_back(0.0); physicsBlock.finYaw.push_back(0.0); physicsBlock.finRoll.push_back(0.0);
            physicsBlock.active.push_back(true);

            controlBlock.thrustCommand.push_back(0);
            controlBlock.pitchCommand.push_back(0);
            controlBlock.yawCommand.push_back(0);
            controlBlock.rollCommand.push_back(0);

            guidanceBlock.mode.push_back(GuidanceMode::None);
            guidanceBlock.targetX.push_back(0); guidanceBlock.targetY.push_back(0); guidanceBlock.targetZ.push_back(0);
            guidanceBlock.targetVx.push_back(0); guidanceBlock.targetVy.push_back(0); guidanceBlock.targetVz.push_back(0);
            guidanceBlock.commandedAccelX.push_back(0); guidanceBlock.commandedAccelY.push_back(0); guidanceBlock.commandedAccelZ.push_back(0);
            guidanceBlock.maxAccel.push_back(0.0);

            statusBlock.type.push_back(EntityType::Missile);
            statusBlock.allegiance.push_back(Allegiance::Friendly);
            statusBlock.health.push_back(100.0);
            statusBlock.isAlive.push_back(true);
            statusBlock.rcsProfileId.push_back("");
            statusBlock.irProfileId.push_back("");

            sensorBlock.accelNoiseStdDev.push_back(0.1);
            sensorBlock.accelBiasStdDev.push_back(0.01);
            sensorBlock.gyroNoiseStdDev.push_back(0.01);
            sensorBlock.gyroBiasStdDev.push_back(0.001);
            sensorBlock.gpsPosNoiseStdDev.push_back(5.0);
            sensorBlock.gpsVelNoiseStdDev.push_back(0.5);

            seekerBlock.type.push_back(SeekerType::None);
            seekerBlock.transmitterPowerW.push_back(1000.0);
            seekerBlock.antennaGainDb.push_back(30.0);
            seekerBlock.wavelengthM.push_back(0.03); // X-band
            seekerBlock.noiseFloorW.push_back(1e-12);
            seekerBlock.snrThresholdDb.push_back(13.0);
            seekerBlock.sensitivityW.push_back(1e-9);
            seekerBlock.wavelengthBand.push_back(0);
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
        }

        physicsBlock.px[id] = init.px; physicsBlock.py[id] = init.py; physicsBlock.pz[id] = init.pz;
        physicsBlock.vx[id] = init.vx; physicsBlock.vy[id] = init.vy; physicsBlock.vz[id] = init.vz;
        physicsBlock.qw[id] = init.qw; physicsBlock.qx[id] = init.qx; physicsBlock.qy[id] = init.qy; physicsBlock.qz[id] = init.qz;
        physicsBlock.wx[id] = init.wx; physicsBlock.wy[id] = init.wy; physicsBlock.wz[id] = init.wz;
        physicsBlock.alphax[id] = 0.0; physicsBlock.alphay[id] = 0.0; physicsBlock.alphaz[id] = 0.0;
        physicsBlock.Ixx[id] = init.Ixx; physicsBlock.Iyy[id] = init.Iyy; physicsBlock.Izz[id] = init.Izz;
        physicsBlock.mass[id] = init.mass;
        physicsBlock.massDry[id] = (config.massDry < 0.0) ? init.mass : config.massDry;
        physicsBlock.referenceArea[id] = config.referenceArea;
        physicsBlock.referenceLength[id] = config.referenceLength;
        physicsBlock.cd[id] = config.cd;
        physicsBlock.clAlpha[id] = config.clAlpha;
        physicsBlock.clFin[id] = config.clFin;
        physicsBlock.clMax[id] = config.clMax;
        physicsBlock.finPitch[id] = 0.0; physicsBlock.finYaw[id] = 0.0; physicsBlock.finRoll[id] = 0.0;
        physicsBlock.ignitionTime[id] = time.currentTime();
        physicsBlock.active[id] = true;

        // Per-entity propulsion registration (W1): empty curve => coasting vehicle
        if (!config.thrustCurve.empty()) {
            auto prop = std::make_shared<Models::PropulsionModel>(
                Models::ThrustCurve(config.thrustCurve),
                config.vacuumIsp, config.seaLevelIsp);
            physicsBlock.propulsionId[id] = backend->registerPropulsion(std::move(prop));
        } else {
            physicsBlock.propulsionId[id] = -1;
        }

        statusBlock.type[id] = init.type;
        statusBlock.allegiance[id] = init.allegiance;
        statusBlock.health[id] = 100.0;
        statusBlock.isAlive[id] = true;
        statusBlock.rcsProfileId[id] = init.rcsProfileId;
        statusBlock.irProfileId[id] = init.irProfileId;

        seekerBlock.type[id] = init.seekerType;
        seekerBlock.fieldOfViewHalfAngleRad[id] = 1.0471975512;
        seekerBlock.gimbalAzimuthLimitRad[id] = 1.0471975512;
        seekerBlock.gimbalElevationLimitRad[id] = 1.0471975512;
        seekerBlock.lockHysteresisDb[id] = 3.0;
        seekerBlock.lockDropoutTimeSec[id] = 0.10;
        seekerBlock.measurementLatencySec[id] = 0.0;
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

    void SimulationKernel::queueCommand(const SimulationCommand& cmd) {
        commandProcessor.enqueueCommand(cmd);
    }

    void SimulationKernel::step(double dt) {
        const std::vector<double> previousPz = physicsBlock.pz;
        time.advance(dt);
        commandProcessor.process(guidanceBlock);
        
        // 1. Advance true physics
        backend->step(physicsBlock, controlBlock, time.currentTime(), dt);
        
        // 2. Generate noisy sensor measurements
        sensorSystem.update(physicsBlock, sensorBlock, time.currentTime(), dt);
        
        // 3. Compute Navigation estimates (INS + EKF)
        navigationSystem.update(sensorBlock, physicsBlock, navigationBlock, dt);
        
        // 3.5 Process Seekers
        seekerSystem.update(physicsBlock, statusBlock, seekerBlock, dt);

        // 4. Update Guidance based on estimates and seekers
        guidanceSystem.update(statusBlock, navigationBlock, seekerBlock, guidanceBlock, controlBlock, dt);

        // 4.5 Update Autopilot to translate commanded accel to fin deflections
        autopilotSystem.update(statusBlock, navigationBlock, sensorBlock, guidanceBlock, controlBlock, dt);
        
        // 5. Evaluate truth events (impacts)
        eventSystem.evaluate(
            physicsBlock, statusBlock, time.currentTime(), dt, previousPz);
        eventSystem.processQueue();
    }

    void SimulationKernel::runSteps(std::size_t steps, double dt) {
        for (std::size_t i = 0; i < steps; ++i) {
            step(dt);
        }
    }

} // namespace StrikeEngine::Kernel
