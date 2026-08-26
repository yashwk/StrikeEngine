# StrikeEngine Specification

**Status:** authoritative normative specification
**Version:** 0.1.0
**Verified:** 2026-08-26
**Repository:** StrikeEngine

This document defines what StrikeEngine means, what its public interfaces
guarantee, and which capabilities are implemented, partial, or intentionally
not supported. Requirements use **MUST**, **MUST NOT**, **SHOULD**, and **MAY**
in their usual normative sense.

`docs/IMPLEMENTATION.md` is the companion authoritative implementation record.
`docs/FIDELITY_AUDIT.md` is the measured evidence report; it does not override
this specification.

## 1. Product boundary

StrikeEngine is a C++23 simulation library for deterministic, data-oriented
multi-entity vehicle simulation. It owns:

- 6-DOF vehicle truth dynamics and per-entity physical configuration;
- atmosphere, aerodynamics, propulsion, gravity, and earth-frame models;
- sensors, navigation, seekers, guidance, autopilot, and events;
- CPU execution, an optional Vulkan backend, and simulation study wrappers;
- public headers, validation tests, and installable CMake packaging.

StrikeEngine does not own the StrikeDesigner UI, StrikeSim application shell,
or StrikeCEM physical-optics solver. Those systems may consume this library
through an explicit integration layer. Design identity, geometry provenance,
and exported asset validation belong to the producer/consumer contract of the
owning application, not to an implicit StrikeEngine side effect.

The project boundaries are explicit:

- StrikeEngine is library-only. An executable shell, editor, visualization
  layer, or optional ECS/editor-entity mapping belongs to a consuming
  application and must not become an implicit kernel dependency.
- StrikeSim is the intended application owner for Design, Simulate, and CEM
  surfaces. It consumes the versioned/installable StrikeEngine package.
- StrikeCEM remains a separate project. Its CLI is retained for offline batch
  work and its `strikecem_lib` shared library is the intended embedding
  boundary for StrikeSim; StrikeCEM source is not part of StrikeEngine.
- StrikeDesigner owns design intent and identity, StrikeCEM owns computation,
  validation, and provenance, and StrikeEngine owns runtime simulation and
  signature/database lookup. Integration must carry identity and revision
  metadata explicitly.

## 2. Capability status vocabulary

Every feature in this specification has one of these statuses:

| Status | Meaning |
| --- | --- |
| Implemented | Public behavior exists and is covered by the current Release validation suite. |
| MVP / partial | A bounded implementation exists, but named fidelity or integration limits remain. |
| Planned | A desired capability is recorded here but is not part of the supported runtime contract. |
| Unsupported | Callers MUST NOT rely on the capability; no silent fallback is promised. |

The current validated checkpoint is **20/20 CTest tests passing** in Release.
The test count is evidence for the current checkout, not a promise that every
future model or integration is complete.

## 3. Global contracts

### 3.1 Units and numerical conventions

- Position is metres, velocity and acceleration are metres per second and
  metres per second squared, mass is kilograms, force is newtons, moment is
  newton-metres, time is seconds, and angles are radians.
- Quaternions are stored as `(qw, qx, qy, qz)` in the state arrays and map
  body vectors into the selected world frame.
- Angular rates and achieved fin deflections are state variables, not merely
  command inputs.
- Public vectors use zero-based component order and public SoA arrays use a
  common entity index.
- The library is not thread-safe at the individual `SimulationKernel` level.
  Independent kernels MAY run concurrently when their callbacks and models
  are independent.

### 3.2 State ownership and invariants

`SimulationKernel` owns the truth, command, navigation, sensor, seeker,
guidance, and entity-status blocks. Callers MUST treat returned const blocks
as read-only snapshots. A vehicle ID is a stable SoA slot until the slot is
reused after removal.

For every active entity:

- all per-entity arrays have an entry at the entity index;
- the attitude quaternion is normalized after integration;
- mass is never below `massDry`;
- achieved fin deflections are limited to ±0.43 rad;
- accelerations in `ax/ay/az` are refreshed after each step;
- an entity deactivated by ground impact is not advanced by later systems.

### 3.3 Determinism

The CPU truth models are deterministic for the same initial state, commands,
configuration, timestep sequence, and sensor seed. `setRandomSeed` controls
sensor noise and random-walk bias generation. Unseeded sensor operation is
allowed but is not reproducible. `BatchRunner` MUST create an isolated kernel
for each scenario.

## 4. Coordinate frames

### 4.1 Body frame

The body frame is aerospace-like: X forward, Y right, Z down. Positive
pitch-fin deflection produces a nose-up moment; positive yaw-fin deflection
produces a nose-right moment. `wx/wy/wz` are body angular rates.

### 4.2 Local world frame

The default world frame is a local flat-earth frame with X east, Y north, and
Z up. The existing short-range scenarios use Z as altitude and ground at
`z = 0` unless a terrain callback is configured. This is the backward-
compatible default.

### 4.3 ECEF truth frame

When `EnvironmentConfig::earth.useEcefTruth` is true, `PhysicsBlock` position
and velocity are absolute WGS84 ECEF values. The kernel resolves geodetic
altitude for atmosphere and ground checks, evaluates earth acceleration in
ECEF, and keeps GPS/INS values in the same ECEF frame.

`referenceLatitudeRad` and `referenceLongitudeRad` anchor terrain callbacks in
local ENU coordinates. Scenario authors using ECEF truth MUST provide absolute
ECEF initial states, normally by calling `geodeticToEcef`.

### 4.4 Earth models

The public earth model headers provide:

- WGS84 geodetic/ECEF conversion;
- ECEF↔ENU and ECEF↔NED direction transforms;
- ENU↔NED axis conversion;
- normal gravity using the Somigliana formula and altitude correction;
- spherical point-mass gravity using the WGS84 gravitational parameter;
- local Coriolis, centrifugal, and moving-origin transport terms;
- a standalone rotating-Earth ECEF RK4 propagator.

The local kernel earth options are independently selectable:

| Option | Behavior |
| --- | --- |
| `useWgs84Gravity` | Uses latitude/altitude-dependent normal gravity. |
| `useSphericalGravity` | Uses radial point-mass gravity in local ENU mode. |
| `includeCoriolis` | Adds `-2 Ω × v`. |
| `includeCentrifugal` | Adds `-Ω × (Ω × r)`. |
| `includeTransportRate` | Adds local moving-origin `-Ω_en × v`; ignored in ECEF truth mode. |
| `useEcefTruth` | Selects absolute ECEF state and ECEF-aware kernel consumers. |

If both gravity flags are true, WGS84 normal gravity takes precedence. In ECEF
truth mode, spherical gravity is the default fallback when normal gravity is
not selected. The default with all options false remains the legacy constant
gravity `-9.80665 m/s²` in local Z.

The local earth model is not a complete global geophysical model. It does not
yet promise geoid separation, terrain streaming, polar/dateline scenario
management, atmospheric rotation/wind coupling, or a full moving-origin
global propagator for every subsystem.

## 5. Kernel API contract

The primary public class is `StrikeEngine::Kernel::SimulationKernel`.

### 5.1 Lifecycle

1. Construct a CPU kernel, or request Vulkan when the optional Vulkan library
   is linked.
2. Call `setEnvironment` before creating or stepping entities when non-default
   environment behavior is required.
3. Optionally call `setRandomSeed` before stepping.
4. Create entities with `createVehicle(init[, vehicleConfig])`.
5. Queue guidance commands as needed.
6. Call `step(dt)` or `runSteps(count, dt)`.
7. Read const state snapshots and process subscribed events externally.

`reset()` clears all SoA blocks, commands, time, free slots, and seeker
tracking state while preserving the kernel object and backend. `initialize()`
resets the simulation.

`step(dt)` advances truth first, then updates sensors, navigation, seekers,
guidance, autopilot commands, and finally events. A non-positive timestep is
not a supported simulation step.

### 5.2 Vehicle initialization and configuration

`VehicleInitState` supplies position, velocity, quaternion, body rates, mass,
principal inertias, entity type, allegiance, and signature profile IDs.
`VehicleConfig` supplies per-entity reference area/length, drag and lift
coefficients, dry mass, thrust curve, and Isp values.

An empty thrust curve means coasting. A configured motor burns only while fuel
remains and mass flow is clamped at dry mass. Thrust acts along body +X.

### 5.3 Commands and guidance state

`SimulationCommand` supplies guidance mode, target position, target velocity,
and an optional maximum acceleration demand. Supported modes are:

- `None`: ballistic/uncontrolled;
- `ProportionalNavigation`: explicit relative-position/relative-velocity PN;
- `Waypoint`: static point navigation.

The command queue is applied on the next kernel step. `ControlBlock::thrustCommand`
exists in the public state but is not currently a throttle interface; callers
MUST NOT expect it to change motor output.

### 5.4 Environment callbacks

`terrainElevation(x, y)` returns metres above the selected datum. In local
mode, x/y are the local world horizontal coordinates. In ECEF truth mode,
x/y are the ENU displacement from the configured reference.

`windVelocity(x, y, z, time)` returns world/ECEF air-mass velocity in m/s. The
truth model subtracts it from vehicle velocity before aerodynamic evaluation.
Null callbacks fall back to zero wind and zero terrain.

## 6. Truth dynamics

### 6.1 Translational dynamics

The CPU backend rotates body aero/thrust force to the selected world frame,
divides by current mass, and adds the configured gravity and rotating-earth
terms. Position derivative is world-frame velocity.

### 6.2 Aerodynamics

`BasicAeroModel` evaluates dynamic pressure from ISA density, drag opposing
body air-relative velocity, lift from angle of attack and fin pitch, bounded
lift coefficient, yaw-fin side force, static pitch stability, rate damping,
and bounded aerodynamic moments. Coefficients and geometry are per entity.

This remains an engineering model, not CFD or a validated coefficient-table
solver. Body lateral stability and coefficient tables are future extensions.

### 6.3 Rotation and actuators

The CPU truth model integrates diagonal body inertia with gyroscopic coupling,
body angular rates, and a body-to-world quaternion. The autopilot transforms
world acceleration demands to body axes and supplies bounded fin commands.
Achieved fins follow commands through first-order servo lag and a physical rate
limit; the integrator clamps the final deflection range.

### 6.4 Integration

The supported integrators are Euler, true RK4, Symplectic/Velocity-Verlet,
and adaptive RK45. RK4 and RK45 use derivative callbacks and re-evaluate
forces at intermediate stages. RK45 uses bounded adaptive substeps. Impact
crossing is interpolated by the event system after a physics step.

## 7. Sensors, navigation, seekers, and guidance

### 7.1 Sensors

The IMU produces body specific force and body angular-rate measurements with
Gaussian noise and random-walk biases. GPS produces noisy position and velocity
at the configured internal update cadence. GPS values use the selected kernel
world frame, including ECEF truth mode.

### 7.2 Navigation

Navigation performs perfect initial alignment, strapdown INS propagation, and
a coupled 15-state error-state EKF. The covariance state is position,
velocity, attitude error, accelerometer bias, and gyro bias in row-major 15×15
storage. GPS position and velocity updates correct the coupled state.

The navigation model is an MVP and does not yet promise full coning/sculling,
Earth-rate gyro compensation for every local mode, sensor lever arms, or
multi-rate timestamp interpolation.

### 7.3 Seekers and signatures

RF seekers use an RCS profile and radar range equation; IR seekers use an IR
radiant-intensity profile, inverse-square irradiance, and placeholder
atmospheric extinction. FOV, independent azimuth/elevation gimbal limits,
lock hysteresis, dropout timing, filtered LOS rates, and measurement latency
are implemented. Friendly targets are rejected.

### 7.4 Guidance and autopilot

Stateless PN and APN helpers are available under `models/guidance`. Kernel PN
uses target position/velocity and kernel APN uses filtered seeker LOS rates on
lock. The autopilot translates commanded world acceleration into bounded body
fin demands.

Trajectory management, pursuit, LQR/MPC, blended guidance handoff, and richer
seeker families are planned, not supported requirements.

## 8. Events and simulation tools

Ground impact is a real state transition: position is clamped, velocity and
acceleration are cleared, the entity is deactivated, and a timestamped
`GroundImpact` event is dispatched. Other event enum values exist as extension
points but are not all generated by the current kernel.

Available wrappers:

| API | Contract | Status |
| --- | --- | --- |
| `SingleRun` | Runs one vehicle and writes trajectory records. | Implemented MVP; versioned CSV or binary frame-aware output. |
| `ParamSweep` | Repeats a scenario over a scalar callback and writes result records. | Implemented MVP; reports the configured `primaryEntityIndex`. |
| `MonteCarlo` | Perturbs copied scenarios and writes result records. | Implemented MVP; reports the configured `primaryEntityIndex`; RNG controls remain limited. |
| `Optimizer` | Runs particle-swarm parameter studies with callbacks. | Implemented MVP. |
| `BatchRunner` | Runs isolated scenarios and returns structured summary results. | Implemented MVP; frame-aware primary and aggregate metrics can be serialized by `StudyOutputWriter`. |

`StudyOutputWriter` provides the common study-output contract. CSV files use
format version `1` metadata comments; binary files use the eight-byte
`STRKOUT1` magic followed by version, record type, field count, record count,
field IDs, and typed little-endian record values. An empty field list selects
the record-type defaults; a non-empty list selects unique supported fields in
the requested order. Every default record contains status and frame identity.
Every state record contains its `Frame` (`LOCAL_ENU` or `ECEF`) and
selected-frame position and velocity columns, plus WGS84 latitude/longitude
in radians and `Altitude_m`; local mode uses world Z for altitude, while ECEF
mode derives altitude from the absolute ECEF position.
`ScenarioConfig::primaryEntityIndex` selects the entity represented by sweep,
Monte Carlo, and batch primary-state summaries; its default is zero for
compatibility. Binary readers and richer telemetry schemas are not yet part of
the supported contract.

## 9. Capability matrix and boundaries

Implemented or MVP: CPU SoA kernel, per-entity vehicle physics, 6-DOF rigid
body, ISA1976 atmosphere, aero/propulsion, RK4/RK45/Euler/Symplectic,
terrain/wind callbacks, impact events, sensors, navigation EKF, RF/IR seeker
MVP, PN/APN/waypoint guidance, autopilot, WGS84/ECEF/local-earth models,
optional ECEF kernel truth, batch/sweep/Monte Carlo/optimizer tooling, and
installable CMake packaging.

Planned or partial: coefficient tables and higher-fidelity aero, fuel/staging
models, failure and damage semantics, advanced atmosphere, full global
terrain/DEM ingestion, geoid models, richer sensors and seeker families,
sensor fusion, trajectory/energy management, pursuit, LQR/MPC, binary readers,
richer telemetry schemas, parallel CPU execution,
CUDA, and a production-grade GPU backend. Optional ECS/editor mapping,
visualization, plotting/analysis/scenario-generation tooling, and an API
server wrapper are integration or tooling ideas, not current kernel features.

Unsupported behavior MUST fail clearly or remain opt-in. The engine MUST NOT
silently substitute a future solver, GPU path, terrain database, material
model, or controller that is not described and validated here.

## 10. Change control

Any new public behavior MUST update this specification, the implementation
record, affected schemas/configuration comments, and at least one regression
test. A feature is not “implemented” merely because a header or roadmap item
exists: its accepted inputs, outputs, defaults, failure behavior, and test
evidence must be documented.
