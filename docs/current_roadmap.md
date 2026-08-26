## new roadmap

> **Status overlay — verified 2026-08-26:** `[x]` means implemented and
> covered by the current validation suite; `[~]` means an MVP or integrated
> equivalent exists but the exact standalone structure is not complete; `[ ]`
> remains pending. The original roadmap text and future items are retained.
> The active implementation uses `models/` plus `kernel/systems/`; the older
> `domain/`, `navigation/`, and `seekers/` names below are preserved as design
> intent rather than literal current paths.

- [x] strikeengine/

    - [x] kernel/ (simulation core)
        - [x] SimulationKernel.hpp
        - [x] SimulationKernel.cpp

        - [~] config/
            - [ ] KernelConfig.hpp (placeholder remains empty)
            - [x] ScenarioConfig.hpp

        - [x] data/ (Pure SoA state)
            - [x] PhysicsBlock.hpp
            - [x] ControlBlock.hpp
            - [x] GuidanceBlock.hpp
            - [x] EntityStatusBlock.hpp

        - [x] systems/ (Internal kernel systems)
            - [x] GuidanceSystem.hpp
            - [x] EventSystem.hpp
            - [x] CommandProcessor.hpp

        - [~] backend/ (Physics execution backends)
            - [x] PhysicsBackend.hpp
            - [x] CPUBackend.hpp
            - [~] GPUBackend.hpp (future; optional Vulkan backend exists under `backend/vulkan/`)

        - [x] integrator/
            - [x] Integrator.hpp
            - [x] RK4Integrator.hpp

        - [x] time/
            - [x] KernelTime.hpp

    - [~] domain/ (Pure math models; implemented under `models/`)
        - [x] atmosphere/
        - [x] aerodynamics/
        - [x] propulsion/
        - [~] guidance/ (implemented in `kernel/systems/GuidanceSystem`)
        - [~] radar/ (RCS/IR signature databases and seeker MVP exist; standalone radar model pending)
        - [~] terrain/ (terrain callback MVP exists; DEM/query module pending)

    - [x] simulation/ (Wraps SimulationKernel)
        - [x] SingleRun.hpp
        - [x] ParamSweep.hpp
        - [x] MonteCarlo.hpp
        - [x] Optimizer.hpp

    - [ ] ecs/ (Optional orchestration layer)
        - [ ] Registry.hpp
        - [ ] components/
            - [ ] PhysicsHandle.hpp
            - [ ] GuidanceConfig.hpp
            - [ ] RadarSignature.hpp (Later)

    - [ ] io/
    - [ ] tooling/
    - [ ] utils/
    - [x] tests/

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



##  PHASE 1 — Deterministic Single Missile (MVP Core) — [x] DONE

Goal: One missile, deterministic physics, CPU only.

### Build in this order:

- [x] domain/atmosphere (minimal ISA)

- [x] domain/aerodynamics (basic Cd model)

- [x] domain/propulsion (thrust curve)

- [x] PhysicsBlock (expanded beyond the original single-entity scope)

- [x] KernelTime

- [x] RK4Integrator

- [x] CPUBackend (multi-entity loop)

- [x] SimulationKernel (minimal orchestration)

- [x] SingleRun wrapper



---

##  PHASE 2 — Multi-Entity SoA Engine — [x] DONE

Goal: True scalable architecture.

Add:

- [x] Expand PhysicsBlock to arrays

- [x] ControlBlock

- [x] GuidanceBlock

- [x] EntityStatusBlock

- [x] GuidanceSystem

- [x] EventSystem

- [x] CommandProcessor

- [x] ScenarioConfig



---

##  PHASE 3 — Simulation Power Tools — [x] DONE

Goal: Research-grade capability.

- [x] ParamSweep

- [x] MonteCarlo

- [x] Optimizer

---

##  PHASE 4 — Backend Abstraction — [~] MVP PARTIAL

Goal: Hardware acceleration.

- [x] PhysicsBackend interface refinement

- [ ] Parallel CPU backend

- [~] GPUBackend (optional Vulkan compute exists; CUDA/general support pending)

---

##  PHASE 5 — Extended Domains — [~] MVP PARTIAL

- [~] radar/ (signature/seeker MVP exists; standalone radar model pending)

- [~] terrain/ (terrain callback MVP exists; DEM/query module pending)

- [ ] advanced atmosphere

- [x] adaptive RK45

- [x] symplectic integrator


# A more detailed structure

> Status is annotated below using the same `[x]` / `[~]` / `[ ]` convention.

- [x] strikeengine/

    - [x] kernel/ (Simulation Core)

        - [x] SimulationKernel.hpp
        - [x] SimulationKernel.cpp

        - [~] config/
            - [ ] KernelConfig.hpp (placeholder remains empty)
            - [x] ScenarioConfig.hpp
            - [x] VehicleConfig.hpp

        - [x] data/
            - [x] PhysicsBlock.hpp
            - [x] ControlBlock.hpp
            - [x] GuidanceBlock.hpp
            - [x] NavigationBlock.hpp
            - [x] SensorBlock.hpp
            - [x] EntityStatusBlock.hpp

        - [~] systems/
            - [~] PhysicsSystem.hpp (physics execution is in `CPUBackend`)
            - [x] SensorSystem.hpp
            - [x] NavigationSystem.hpp
            - [x] GuidanceSystem.hpp
            - [~] ControlSystem.hpp (implemented as `AutopilotSystem`)
            - [x] EventSystem.hpp
            - [x] CommandProcessor.hpp

        - [~] backend/
            - [x] PhysicsBackend.hpp
            - [x] CPUBackend.hpp
            - [~] GPUBackend.hpp (placeholder; optional Vulkan backend exists)

        - [x] integrator/
            - [x] Integrator.hpp
            - [x] RK4Integrator.hpp
            - [x] RK45Integrator.hpp
            - [x] EulerIntegrator.hpp

        - [x] time/
            - [x] KernelTime.hpp

    - [~] domain/ (Pure Mathematical Models; implemented under `models/`)

        - [x] atmosphere/
            - [x] AtmosphereModel.hpp
            - [x] ISA1976.hpp
            - [~] AtmosphereState.hpp (state is defined in `AtmosphereModel.hpp`)

        - [~] aerodynamics/
            - [x] AeroModel.hpp
            - [ ] CoefficientTables.hpp
            - [ ] AeroForces.hpp

        - [~] propulsion/
            - [x] PropulsionModel.hpp
            - [x] ThrustCurve.hpp
            - [ ] FuelModel.hpp

        - [~] gravity/ (WGS84 normal gravity is integrated in EarthModel)
            - [~] GravityModel.hpp (normal-gravity API exists in EarthModel.hpp)
            - [ ] SphericalGravity.hpp
            - [~] WGS84Gravity.hpp (normal gravity is implemented in EarthModel.hpp)

        - [~] earth/ (opt-in local-earth MVP)
            - [~] EarthRotation.hpp (rotation rate used by Coriolis helper)
            - [x] Coriolis.hpp (local ENU helper is implemented in EarthModel.hpp)
            - [~] WGS84.hpp (constants and geodetic/ECEF conversion are in EarthModel.hpp)

        - [~] frames/ (conversion foundation is integrated in EarthModel)
            - [~] ECEF.hpp (ECEF coordinate and conversion API exists in EarthModel.hpp)
            - [ ] NED.hpp
            - [ ] ENU.hpp
            - [~] Geodetic.hpp (geodetic coordinate and conversion API exists in EarthModel.hpp)

        - [~] guidance/ (MVP logic is in `kernel/systems/GuidanceSystem`)
            - [~] ProNav.hpp (explicit PN MVP; standalone model is in `models/guidance`)
            - [~] AugmentedProNav.hpp (explicit APN MVP; standalone model is in `models/guidance`)
            - [ ] Pursuit.hpp
            - [~] WaypointGuidance.hpp (waypoint mode exists in the kernel)

        - [~] control/
            - [~] PID.hpp (PD-like autopilot MVP exists; standalone PID pending)
            - [ ] LQR.hpp
            - [ ] MPC.hpp

        - [~] radar/ (Future; signature/seeker MVP exists)
            - [ ] RadarModel.hpp
            - [ ] DetectionModel.hpp
            - [ ] TrackingModel.hpp

        - [~] terrain/ (Future; callback MVP exists)
            - [~] TerrainModel.hpp (callback equivalent exists)
            - [ ] DEMLoader.hpp
            - [ ] TerrainQuery.hpp

    - [~] navigation/ (integrated into `kernel/systems`)

        - [~] ins/
            - [x] StrapdownINS.hpp (integrated into `NavigationSystem`)
            - [~] INSState.hpp (navigation state is in `NavigationBlock`)

        - [~] filters/
            - [ ] KalmanFilter.hpp
            - [x] ErrorStateEKF.hpp (integrated into `NavigationSystem`)
            - [ ] UKF.hpp
            - [x] Covariance.hpp (full covariance is stored in `NavigationBlock`)

        - [~] sensors/
            - [x] IMU.hpp (integrated into `SensorSystem`)
            - [x] GPS.hpp (integrated into `SensorSystem`)
            - [ ] Magnetometer.hpp
            - [ ] Barometer.hpp
            - [ ] RadarAltimeter.hpp
            - [~] SensorNoise.hpp (noise configuration is in `SensorBlock`)

        - [ ] fusion/
            - [ ] SensorFusion.hpp

    - [~] guidance/

        - [ ] midcourse/
            - [ ] TrajectoryManager.hpp
            - [ ] WaypointManager.hpp
            - [ ] EnergyManager.hpp

        - [~] terminal/
            - [~] TerminalGuidance.hpp (seeker APN MVP exists)
            - [~] TargetTracker.hpp (seeker lock/tracking MVP exists)
            - [ ] SeekerManager.hpp

        - [~] autopilot/
            - [~] AttitudeController.hpp (integrated into `AutopilotSystem`)
            - [~] FlightController.hpp (integrated into `AutopilotSystem`)
            - [~] ActuatorMixer.hpp (integrated into the autopilot/actuator path)

    - [~] seekers/ (MVP implemented in `kernel/systems/SeekerSystem`)

        - [~] radar/
            - [~] ActiveRadarSeeker.hpp
            - [ ] SemiActiveRadarSeeker.hpp

        - [~] infrared/
            - [~] IRSeeker.hpp

        - [ ] optical/
            - [ ] EOSeeker.hpp

    - [~] simulation/
        - [x] SingleRun.hpp
        - [x] ParamSweep.hpp
        - [x] MonteCarlo.hpp
        - [x] BatchRunner.hpp
        - [x] Optimizer.hpp

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


# MVP implementation order (status overlay)

- [x] PhysicsBlock
- [x] KernelTime
- [x] Integrator Interface
- [x] RK4Integrator
- [x] AtmosphereModel
- [x] PropulsionModel
- [x] AeroModel
- [x] CPUBackend
- [x] SimulationKernel
- [x] SingleRun
- [x] Validation Tests

## Phase 2
- [x] Multi-entity SoA support
- [x] GuidanceBlock
- [x] ControlBlock
- [x] GuidanceSystem
- [x] EventSystem

## Phase 3
- [x] StrapdownINS (integrated into NavigationSystem)
- [x] IMU Sensor Model (integrated into SensorSystem)
- [x] ErrorStateEKF (integrated into NavigationSystem)
- [x] GPS Fusion (integrated into NavigationSystem)

## Phase 4
- [x] Midcourse Guidance (explicit PN MVP)
- [x] Autopilot
- [x] Terminal Guidance (filtered-rate seeker APN MVP)
- [x] Seekers (radar/IR MVP)

## Phase 5
- [x] MonteCarlo
- [x] ParamSweep
- [x] Optimizer

## Phase 6
- [~] ECEF (WGS84 conversion MVP)
- [x] WGS84 Gravity (opt-in normal gravity)
- [~] Earth Rotation (rotation-rate foundation)
- [x] Coriolis Effects (opt-in local ENU acceleration)
- [~] GPU Backend (optional Vulkan implementation exists; broader backend support pending)
