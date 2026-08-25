## new roadmap

- [ ] strikeengine/

    - [ ] kernel/ (simulation core)
        - [ ] SimulationKernel.hpp
        - [ ] SimulationKernel.cpp

        - [ ] config/
            - [ ] KernelConfig.hpp
            - [ ] ScenarioConfig.hpp

        - [ ] data/ (Pure SoA state)
            - [ ] PhysicsBlock.hpp
            - [ ] ControlBlock.hpp
            - [ ] GuidanceBlock.hpp
            - [ ] EntityStatusBlock.hpp

        - [ ] systems/ (Internal kernel systems)
            - [ ] GuidanceSystem.hpp
            - [ ] EventSystem.hpp
            - [ ] CommandProcessor.hpp

        - [ ] backend/ (Physics execution backends)
            - [ ] PhysicsBackend.hpp
            - [ ] CPUBackend.hpp
            - [ ] GPUBackend.hpp (future)

        - [ ] integrator/
            - [ ] Integrator.hpp
            - [ ] RK4Integrator.hpp

        - [ ] time/
            - [ ] KernelTime.hpp

    - [ ] domain/ (Pure math models)
        - [ ] atmosphere/
        - [ ] aerodynamics/
        - [ ] propulsion/
        - [ ] guidance/
        - [ ] radar/ (Later)
        - [ ] terrain/ (Later)

    - [ ] simulation/ (Wraps SimulationKernel)
        - [ ] SingleRun.hpp
        - [ ] ParamSweep.hpp
        - [ ] MonteCarlo.hpp
        - [ ] Optimizer.hpp

    - [ ] ecs/ (Optional orchestration layer)
        - [ ] Registry.hpp
        - [ ] components/
            - [ ] PhysicsHandle.hpp
            - [ ] GuidanceConfig.hpp
            - [ ] RadarSignature.hpp (Later)

    - [ ] io/
    - [ ] tooling/
    - [ ] utils/
    - [ ] tests/

[domain math models]
↓
[kernel/data blocks (SoA)]
↓
[integrator]
↓
[physics backend]
↓
[simulation kernel]
↓
[simulation wrappers]
↓
[ecs layer (optional)]



##  PHASE 1 — Deterministic Single Missile (MVP Core)

Goal: One missile, deterministic physics, CPU only.

### Build in this order:

-  domain/atmosphere (minimal ISA)

-  domain/aerodynamics (basic Cd model)

-  domain/propulsion (thrust curve)

-  PhysicsBlock (single entity only)

-  KernelTime

-  RK4Integrator

-  CPUBackend (single entity loop)

-  SimulationKernel (minimal orchestration)

-  SingleRun wrapper



---

##  PHASE 2 — Multi-Entity SoA Engine

Goal: True scalable architecture.

Add:

-  Expand PhysicsBlock to arrays

-  ControlBlock

-  GuidanceBlock

-  EntityStatusBlock

-  GuidanceSystem

-  EventSystem

-  CommandProcessor

-  ScenarioConfig



---

##  PHASE 3 — Simulation Power Tools

Goal: Research-grade capability.

-  ParamSweep

-  MonteCarlo

-  Optimizer

---

##  PHASE 4 — Backend Abstraction

Goal: Hardware acceleration.

-  PhysicsBackend interface refinement

-  Parallel CPU backend

-  GPUBackend (Vulkan compute or CUDA)

---

##  PHASE 5 — Extended Domains

-  radar/

-  terrain/

-  advanced atmosphere

-  adaptive RK45

-  symplectic integrator


# A more detailed structure

- [ ] strikeengine/

    - [ ] kernel/ (Simulation Core)

        - [ ] SimulationKernel.hpp
        - [ ] SimulationKernel.cpp

        - [ ] config/
            - [ ] KernelConfig.hpp
            - [ ] ScenarioConfig.hpp
            - [ ] VehicleConfig.hpp

        - [ ] data/
            - [ ] PhysicsBlock.hpp
            - [ ] ControlBlock.hpp
            - [ ] GuidanceBlock.hpp
            - [ ] NavigationBlock.hpp
            - [ ] SensorBlock.hpp
            - [ ] EntityStatusBlock.hpp

        - [ ] systems/
            - [ ] PhysicsSystem.hpp
            - [ ] SensorSystem.hpp
            - [ ] NavigationSystem.hpp
            - [ ] GuidanceSystem.hpp
            - [ ] ControlSystem.hpp
            - [ ] EventSystem.hpp
            - [ ] CommandProcessor.hpp

        - [ ] backend/
            - [ ] PhysicsBackend.hpp
            - [ ] CPUBackend.hpp
            - [ ] GPUBackend.hpp

        - [ ] integrator/
            - [ ] Integrator.hpp
            - [ ] RK4Integrator.hpp
            - [ ] RK45Integrator.hpp
            - [ ] EulerIntegrator.hpp

        - [ ] time/
            - [ ] KernelTime.hpp

    - [ ] domain/ (Pure Mathematical Models)

        - [ ] atmosphere/
            - [ ] AtmosphereModel.hpp
            - [ ] ISA1976.hpp
            - [ ] AtmosphereState.hpp

        - [ ] aerodynamics/
            - [ ] AeroModel.hpp
            - [ ] CoefficientTables.hpp
            - [ ] AeroForces.hpp

        - [ ] propulsion/
            - [ ] PropulsionModel.hpp
            - [ ] ThrustCurve.hpp
            - [ ] FuelModel.hpp

        - [ ] gravity/
            - [ ] GravityModel.hpp
            - [ ] SphericalGravity.hpp
            - [ ] WGS84Gravity.hpp

        - [ ] earth/
            - [ ] EarthRotation.hpp
            - [ ] Coriolis.hpp
            - [ ] WGS84.hpp

        - [ ] frames/
            - [ ] ECEF.hpp
            - [ ] NED.hpp
            - [ ] ENU.hpp
            - [ ] Geodetic.hpp

        - [ ] guidance/
            - [ ] ProNav.hpp
            - [ ] AugmentedProNav.hpp
            - [ ] Pursuit.hpp
            - [ ] WaypointGuidance.hpp

        - [ ] control/
            - [ ] PID.hpp
            - [ ] LQR.hpp
            - [ ] MPC.hpp

        - [ ] radar/ (Future)
            - [ ] RadarModel.hpp
            - [ ] DetectionModel.hpp
            - [ ] TrackingModel.hpp

        - [ ] terrain/ (Future)
            - [ ] TerrainModel.hpp
            - [ ] DEMLoader.hpp
            - [ ] TerrainQuery.hpp

    - [ ] navigation/

        - [ ] ins/
            - [ ] StrapdownINS.hpp
            - [ ] INSState.hpp

        - [ ] filters/
            - [ ] KalmanFilter.hpp
            - [ ] ErrorStateEKF.hpp
            - [ ] UKF.hpp
            - [ ] Covariance.hpp

        - [ ] sensors/
            - [ ] IMU.hpp
            - [ ] GPS.hpp
            - [ ] Magnetometer.hpp
            - [ ] Barometer.hpp
            - [ ] RadarAltimeter.hpp
            - [ ] SensorNoise.hpp

        - [ ] fusion/
            - [ ] SensorFusion.hpp

    - [ ] guidance/

        - [ ] midcourse/
            - [ ] TrajectoryManager.hpp
            - [ ] WaypointManager.hpp
            - [ ] EnergyManager.hpp

        - [ ] terminal/
            - [ ] TerminalGuidance.hpp
            - [ ] TargetTracker.hpp
            - [ ] SeekerManager.hpp

        - [ ] autopilot/
            - [ ] AttitudeController.hpp
            - [ ] FlightController.hpp
            - [ ] ActuatorMixer.hpp

    - [ ] seekers/

        - [ ] radar/
            - [ ] ActiveRadarSeeker.hpp
            - [ ] SemiActiveRadarSeeker.hpp

        - [ ] infrared/
            - [ ] IRSeeker.hpp

        - [ ] optical/
            - [ ] EOSeeker.hpp

    - [ ] simulation/
        - [ ] SingleRun.hpp
        - [ ] ParamSweep.hpp
        - [ ] MonteCarlo.hpp
        - [ ] BatchRunner.hpp
        - [ ] Optimizer.hpp

    - [ ] ecs/

        - [ ] Registry.hpp

        - [ ] components/
            - [ ] PhysicsHandle.hpp
            - [ ] GuidanceConfig.hpp
            - [ ] NavigationConfig.hpp
            - [ ] RadarSignature.hpp

    - [ ] io/
        - [ ] ScenarioLoader.hpp
        - [ ] ConfigParser.hpp
        - [ ] CSVWriter.hpp
        - [ ] BinaryRecorder.hpp

    - [ ] tooling/
        - [ ] Plotting/
        - [ ] Analysis/
        - [ ] ScenarioGenerator/

    - [ ] visualization/ (Optional)
        - [ ] Renderer/
        - [ ] Camera/
        - [ ] DebugDraw/

    - [ ] utils/
        - [ ] Math/
        - [ ] Memory/
        - [ ] Logging/
        - [ ] Units/
        - [ ] Profiling/

    - [ ] tests/
        - [ ] physics/
        - [ ] navigation/
        - [ ] guidance/
        - [ ] sensors/
        - [ ] integration/
        - [ ] validation/


# MVP implementation order

- [ ] PhysicsBlock
- [ ] KernelTime
- [ ] Integrator Interface
- [ ] RK4Integrator
- [ ] AtmosphereModel
- [ ] PropulsionModel
- [ ] AeroModel
- [ ] CPUBackend
- [ ] SimulationKernel
- [ ] SingleRun
- [ ] Validation Tests

## Phase 2
- [ ] Multi-entity SoA support
- [ ] GuidanceBlock
- [ ] ControlBlock
- [ ] GuidanceSystem
- [ ] EventSystem

## Phase 3
- [ ] StrapdownINS
- [ ] IMU Sensor Model
- [ ] ErrorStateEKF
- [ ] GPS Fusion

## Phase 4
- [ ] Midcourse Guidance
- [ ] Autopilot
- [ ] Terminal Guidance
- [ ] Seekers

## Phase 5
- [ ] MonteCarlo
- [ ] ParamSweep
- [ ] Optimizer

## Phase 6
- [ ] ECEF
- [ ] WGS84 Gravity
- [ ] Earth Rotation
- [ ] Coriolis Effects
- [ ] GPU Backend