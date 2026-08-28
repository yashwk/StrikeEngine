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
  StrikeCEM produces the radar-cross-section (RCS) signature tables
  (physical-optics solver). The aerodynamic coefficient tables come from
  **StrikeCFD**, a separate project in a different folder (Barrowman first-cut
  → CFD validation). The eventual goal is to build the **StrikeDesigner**
  surface inside **StrikeSim** on the stabilized StrikeEngine.
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

The current validated checkpoint is **35/35 CTest tests passing** in Release.
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
as read-only snapshots. All public block accessors, including `getSeekers()`,
expose only const references, enforcing that contract. A vehicle ID is a
stable SoA slot until the slot is reused after removal. Reusing a freed slot
restores default guidance (mode `None`), zeroed targets and commands, and
default sensor noise configuration, so stale state is never inherited.

For every active entity:

- all per-entity arrays have an entry at the entity index;
- the attitude quaternion is normalized after integration;
- mass is never below `massDry`;
- achieved fin deflections are limited to the per-entity `maxDeflectionRad`
  (the integrator clamp, §7.4);
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

`reset()` clears all SoA blocks, commands, time, free slots, seeker tracking
state, and the sensor system's internal random-walk bias and GPS update phase
while preserving the kernel object and backend. `initialize()` resets the
simulation.

`step(dt)` advances truth first, then updates sensors, navigation, seekers,
guidance, autopilot commands, and finally events. A non-positive timestep is
rejected: `step(dt)` throws `std::invalid_argument` when `dt <= 0.0`.

### 5.2 Vehicle initialization and configuration

`VehicleInitState` supplies the 6-DOF pose (position, velocity, quaternion,
body rates), mass, and allegiance. Fields that previously lived on the init
state (type, seeker type, signature profile IDs, emitter EIRP, principal
inertias) now live on `VehicleConfig`.

`VehicleConfig` is the flattened per-vehicle subsystem view: the structural
summary (`type`, `initialMass`, `massDry`, `Ixx/Iyy/Izz`) and the subsystem
structs `aero`, `propulsion`, `seeker`, `sensor`, `guidanceAutopilot`,
`warhead`, the subsystem profile-id fields `aeroProfileId`,
`motorProfileId`, `seekerProfileId`, and `sensorProfileId` (all default
`""`), plus the signature fields `rcsProfileId`, `irProfileId`, and
`emitterEirpW`.

- `AeroConfig`: reference area/length and drag/lift coefficients.
- `PropulsionConfig`: an ordered `stages` list of `StageConfig`
  (`thrustCurve`, `vacuumIsp`/`seaLevelIsp`, `propellantMassKg`, `dryMassKg`).
- `SensorConfig`: `imuEnabled`/`gpsEnabled` flags, IMU and GPS noise/bias
  standard deviations, `gpsUpdateRateHz`, and the IMU body-frame lever arm.
- `GuidanceAutopilotConfig`: `navigationConstant`, `waypointGain`, autopilot
  gains `kAccelP/kRateP/kAlphaP/kRollP/kRollD`, `maxDeflectionRad`,
  `servoTimeConstantSec`, and `maxServoRateRadPerSec`.
- `WarheadConfig`: `massKg`, `FusingType` (`Impact`/`Proximity`/`Timed`),
  `proximityTriggerM`, `timedDelaySec`, `lethalRadiusM`, and the optional
  `falloffRadiusM` (0.0 default = flat lethal-radius law; see §8).
- `SeekerConfig`: per-entity seeker type and RF/IR/SARH parameters.

An empty thrust curve means coasting. A configured motor burns only while fuel
remains and mass flow is clamped at dry mass. Thrust acts along body +X.
Multi-stage propulsion is described in §6.5 and warhead fusing in §8.

Profile-id resolution is implemented. Each of `aeroProfileId`,
`motorProfileId`, `seekerProfileId`, and `sensorProfileId` may name a JSON
profile file. On `createVehicle`, a non-empty profile id loads the referenced
file and its parsed config REPLACES the inline sub-config for that subsystem
(the profile is authoritative, consistent with the `designRef` precedent). An
empty id leaves the inline sub-config untouched. A referenced profile that
cannot be loaded (missing file, malformed JSON, missing required key, or
wrong-typed field) makes `createVehicle` throw `std::runtime_error` naming the
file. Guidance/autopilot, warhead, mass/inertia, and the RCS/IR/emitter
signature fields are NOT profile-resolved.

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

### 5.5 Configuration serialization and design interchange

All configuration structs serialize to snake_case JSON with enums encoded as
snake_case strings. The public API operates on JSON text; the underlying
nlohmann/json dependency is never exposed in public headers.

- `serializeVehicleConfig` / `deserializeVehicleConfig`: the full `VehicleConfig`.
- `serializeEnvironment` / `deserializeEnvironment`: serialize only the
  `earth` block; on load the terrain/wind `std::function` callbacks reset to
  the default flat-terrain/zero-wind environment.
- `serializeScenario` / `deserializeScenario`, plus
  `ScenarioConfig::save(path)` (returns false if the file cannot be opened)
  and static `ScenarioConfig::load(path)` (throws `std::runtime_error` if the
  file is missing or the JSON is malformed).
- Design interchange: `serializeDesign(name, geometryJson, physics)` writes a
  `{"name", "geometry", "physics"}` document; `loadDesignPhysics(path)` reads
  the `physics` block back into a `VehicleConfig`. A non-empty
  `ScenarioEntityConfig::designRef` is resolved on scenario load and
  OVERRIDES any inline `vehicleConfig`.

The StrikeDesigner-facing artifacts shipped under `data/profiles` are now
engine design manifests: a `{"name", "description", "physics": <VehicleConfig
snake_case>}` document whose `physics` block `loadDesignPhysics` (via a
scenario `designRef`) parses into a `VehicleConfig`. The manifest maps onto
the engine as follows: the `physics` block IS the `VehicleConfig`, and the
subsystem profile-id fields (`aeroProfileId`, `motorProfileId`,
`seekerProfileId`, `sensorProfileId`, `rcsProfileId`) reference the per-part
files under `data/aero`, `data/motors`, `data/seekers`, `data/sensors`, and
`data/rcs`. On `createVehicle` those profile ids are authoritative and replace
the manifest's inline placeholder sub-configs; the placeholders must still be
present because `VehicleConfig::from_json` requires every sub-object key. The
whole chain — manifest `design_ref` → `loadDesignPhysics` → profile-id
resolution → kernel SoA wiring → RF seeker/RCS signature → guided intercept —
is exercised end-to-end by `designer_pipeline_test`.

Frame note for designers: the intercept scenario's original coordinates were
ECEF-style Earth-radius values (`[0, 6371010, 0]` / `[20000, 6381010, 0]`) that
sit below the WGS84 ellipsoid and are therefore unflyable. The shipped
`data/scenarios/intercept_test_01.json` runs in the engine's local ENU frame
(§4.2, Z up), with the engagement re-baselined to 15 km downrange / 8 km up
(the table-aero missile flies faster and lower-drag, so the original
20 km / 10 km geometry no longer fits the seeker's ~3 km acquisition range).
Designer producers MUST emit local ENU coordinates, not ECEF
Earth-radius offsets.

`intercept_test_01` is a deterministically tuned data set (sensor seed
`0xDEADBEEF`) used to demonstrate the designer→engine pipeline end-to-end; it
is not a guidance-performance claim. Its assertions — a real guided intercept
(min miss 45.4 m < 50 m at t≈11.5 s, now flying on the data-driven aero
coefficient tables) and a proximity-warhead kill via the
event system — are pipeline evidence, not a general accuracy guarantee.

`VehicleInitState` fields are optional in JSON with safe defaults (zero pose,
identity quaternion, mass 0, allegiance `friendly`). Deserializers throw
`std::runtime_error` on malformed JSON or unknown enum strings.

Profile files (aero/motor/seek/sensor) share the same flat snake_case key schema
as the inline `ConfigSerialization` representation, so a profile file and an
inline sub-config are interchangeable. Schema policy for profile files: only the
seeker `type` and the motor `stages` keys are required; every other field
defaults from the struct when omitted.

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

Optional data-driven coefficient tables (`aero_tables` on `AeroConfig` and
`data/aero/*.json` profiles) provide cd(M, α) and cl(M, α) as a rectilinear
grid (`mach_breakpoints`, `aoa_breakpoints_rad`, `cl_table`, `cd_table`,
dimensioned `[mach][aoa]`) that is bilinearly interpolated at runtime and
clamped to the grid bounds. When a valid table block is present it is
authoritative for cd/cl; otherwise the flat scalar coefficients (`cd`,
`clAlpha`, `clFin`, `clMax`) are used as a guaranteed byte-identical fallback.
A structurally invalid grid (missing or non-ascending breakpoints, wrong
dimensions, fewer than two breakpoints per axis) is rejected at parse time and
at the profile loader, and is never allowed to reach the interpolator.

Moments and side-force remain the linear engineering model; coefficient tables
for moments (cm) and lateral/β stability, plus Mach-scaled fin effectiveness,
are future extensions. cd/cl tables are produced by the StrikeCFD program
(Barrowman first-cut → CFD validation, a separate project in a different
folder) and consumed by StrikeEngine; StrikeCEM separately produces the
radar-cross-section signature tables.

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
crossing is interpolated by the event system after a physics step. The
integrator is also selectable at kernel construction via `IntegratorType`
(default RK4) for the CPU backend, in addition to the standalone availability
of each integrator.

### 6.5 Multi-stage propulsion (staging)

`PropulsionConfig::stages` is an ordered list of `StageConfig`. Every stage
with a non-empty thrust curve is registered with the backend propulsion pool
and the first such stage is wired as active at launch. The initial `massDry`
equals the final dry mass plus the sum of the separable dry masses of all
stages except the last.

After each physics step the kernel's staging pass detects the active stage's
burnout — the last time in its thrust curve that still produces positive
thrust. On burnout it jettisons the spent stage's unburned (leftover)
propellant, drops the spent stage's `dryMassKg` from both current mass and
`massDry`, rescales `Ixx/Iyy/Izz` by the post-dump current-to-new mass ratio,
advances to the next stage, resets the stage ignition time, and dispatches a
timestamped `StageSeparation` event.

A stage with `propellantMassKg > 0` burns only its declared propellant: the
mass-flow floor becomes `massDry` plus the propellant reserved for later
stages, and the stage separates on propellant exhaustion (or curve end,
whichever comes first). A stage with `propellantMassKg <= 0` keeps the
curve-end behavior.

The leftover-propellant dump lands the vehicle mass exactly on the new stage's
floor (`massDry` plus the remaining stages' reserved propellant) at separation;
the dumped amount is reported on the `StageSeparation` event as `dumpedMassKg`.
A stage that burned out by propellant exhaustion had already pinned its mass to
that floor, so its leftover is ~0 and exhaustion burnouts are byte-identical to
the pre-dump behavior.

The propulsion law `T = ṁ·Isp·g0` is enforced at every instant, including fuel
exhaustion: the fuel-depletion guard caps mass flow to the propellant remaining
in the depletion window and scales thrust with the capped flow, so thrust
self-terminates when propellant is exhausted. There is no free-thrust tail of
full thrust on the final grams of fuel.

## 7. Sensors, navigation, seekers, and guidance

### 7.1 Sensors

The IMU produces body specific force and body angular-rate measurements with
Gaussian noise and random-walk biases. The IMU supports an optional per-entity
body-frame lever arm (`VehicleConfig::imuLeverArmX/Y/Z`, default zero): the
reported specific force is the centre-of-mass value plus the rigid-body
correction `alpha x l + omega x (omega x l)` for the offset sensor. The default
zero lever arm preserves prior behaviour. In ECEF truth mode,
`EnvironmentConfig::earth.includeEarthRateGyro` (default false) models the gyro
as measuring the INERTIAL body rate: the earth-rotation vector
`omega_ie^b = C_e^b (0, 0, Omega)` is added to the body rate, and the strapdown
INS subtracts it (resolved with the estimated attitude) so attitude
propagation sees the earth-fixed body rate with no spurious drift. The flag is
ignored in local flat-earth mode. GPS produces noisy position and velocity at
the configured internal update cadence. GPS values use the selected kernel
world frame, including ECEF truth mode.

Sensor enablement is per entity. `imuEnabled=false` freezes the IMU output:
the last accel/gyro sample is held and the streaming random-walk bias drift
stops, while GPS continues to update and aid the EKF. This is a stale-sample
degradation, not a full GPS-only positioning mode. `gpsEnabled=false` stops
GPS updates entirely, and `gpsUpdateRateHz` sets the per-entity GPS cadence.

### 7.2 Navigation

Navigation performs perfect initial alignment, strapdown INS propagation, and
a coupled 15-state error-state EKF. The strapdown attitude update is a
rotation-vector step (exact delta-quaternion per sample, validated against a
fine-step reference integrator), which bounds the coning-drift error of a
first-order quaternion step; the velocity update applies the single-interval
sculling/rotation compensation `+0.5 (omega x f) dt^2` in the body frame. The
covariance state is position, velocity, attitude error, accelerometer bias,
and gyro bias in row-major 15×15 storage. GPS position and velocity updates
correct the coupled state.

The navigation model is an MVP and does not yet promise multi-rate timestamp
interpolation, nor earth-rate gyro compensation outside ECEF truth mode (in
local flat-earth truth the gyro already resolves the non-rotating-frame body
rate; correct earth-rate compensation requires the rotating-frame ECEF path).

### 7.3 Seekers and signatures

RF seekers use an RCS profile and the monostatic radar range equation. SARH
(semi-active radar homing) seekers use the bistatic radar equation with a
configured off-board illuminator (`SeekerConfig::illuminator*`; the target RCS
lookup approximates the bistatic cross section). PassiveRF seekers home on a
target's own emission using its effective radiated power
(`VehicleConfig::emitterEirpW`); targets with `emitterEirpW = 0` are not
passively detectable. IR seekers use an IR radiant-intensity profile,
inverse-square irradiance, and Beer-Lambert atmospheric transmittance
`exp(-k·r)` with `SeekerConfig::irExtinctionPerM` (default `1e-4` m^-1, i.e.
0.1/km, which reproduces the legacy placeholder). Acquisition selects the
strongest valid signal (SNR dB for RF/SARH/PassiveRF, received power W for IR)
among all hostile targets rather than the first in index order, while
maintaining a single-track lock. Chaff and flare decoys are entity types that
carry an RCS (chaff) or IR (flare) profile and can seduce the seeker under the
strongest-signal rule. Per-entity seeker parameters are exposed through the
public `SeekerConfig` struct attached to `VehicleConfig::seeker`. FOV,
independent azimuth/elevation gimbal limits, lock hysteresis, dropout timing,
filtered LOS rates, and measurement latency are implemented. Friendly targets
are rejected. The SARH illuminator is a STATIC configured position for this
MVP; dynamic illuminator-entity tracking is future work.

### 7.4 Guidance and autopilot

Stateless PN and APN helpers are available under `models/guidance`. Kernel PN
uses target position/velocity and kernel APN uses filtered seeker LOS rates on
lock. Seeker-locked APN clamps commanded acceleration to the per-entity
`maxAccel` magnitude limit, matching PN and Waypoint. The autopilot translates
commanded world acceleration into bounded body fin demands.

Guidance and autopilot constants are per entity and read from the config at
vehicle creation: `navigationConstant` and `waypointGain` drive seeker APN and
waypoint guidance, and the autopilot gains (`kAccelP/kRateP/kAlphaP/kRollP/
kRollD`) plus `maxDeflectionRad` (the fin clamp, default 0.43 rad) drive fin
command generation. Guidance and autopilot systems read these from the per-
entity guidance and control blocks rather than from class constants.

Trajectory management, pursuit, LQR/MPC, blended guidance handoff, imaging IR,
multi-target tracking, and dynamic SARH illuminator tracking are planned, not
supported requirements.

## 8. Events and simulation tools

Ground impact is a real state transition: position is clamped, velocity and
acceleration are cleared, the entity is deactivated, and a timestamped
`GroundImpact` event is dispatched. Other event enum values exist as extension
points but are not all generated by the current kernel.

Deterministic failure and damage injection is implemented (MVP): the kernel API
exposes `failEntity(id, mode)` and `applyDamage(id, damage)`. Motor,
actuator, sensor, and communication failures set per-entity flags and dispatch
`MotorFailure`, `ActuatorFailure`, `SensorFailure`, and `CommunicationFailure`
events with the trigger timestamp and entity id. Structural failure (direct
`failEntity(StructuralFailure)` or health reaching zero after damage) performs
the same deactivation as a ground impact — `isAlive=false`, `health=0`,
`active=false`, velocity/acceleration cleared — and dispatches a
`StructuralFailure` event. The flags are consumed by the systems: a failed
motor zeroes thrust and mass flow, a failed actuator freezes the achieved fin
deflections, a failed sensor stops measurement updates, and a communication
failure zeroes the commanded acceleration (fly ballistic, guidance and seeker
handoff ignored). The model is deliberately bounded: flags are deterministic
and not probabilistic, partial health has no effect beyond deactivation, and
repair is not modeled.

Multi-stage vehicles dispatch a `StageSeparation` event when a spent stage
separates (§6.5); the event reports the jettisoned leftover propellant in
`dumpedMassKg`. Warhead fusing is evaluated each step after ground-impact
detection. The fuse triggers as follows:

- `Impact` detonates when the carrying entity is deactivated (ground impact);
- `Proximity` detonates when an alive entity is within `proximityTriggerM`;
- `Timed` detonates `timedDelaySec` after launch.

Detonation dispatches a timestamped `Detonation` event and applies a
fragmentation/overpressure kill law to every other alive entity. With
`falloffRadiusM <= 0` (or `<= lethalRadiusM`) the law is flat: guaranteed kill
inside `lethalRadiusM`, none beyond — the pre-feature behavior, byte-identical.
With `falloffRadiusM > lethalRadiusM` the kill probability is 1 inside
`lethalRadiusM`, decays linearly
`(falloffRadiusM − d) / (falloffRadiusM − lethalRadiusM)` across the band
`(lethalRadiusM, falloffRadiusM]`, and is 0 beyond `falloffRadiusM`. A kill is
deterministic when the probability is >= 1; otherwise a uniform draw from the
kernel RNG stream decides only when `0 < p < 1`, so flat-law warheads
(`falloffRadiusM <= 0`) never consume the kernel RNG stream. A warhead with
`lethalRadiusM <= 0` is inert. A `falloff_radius_m` below `lethal_radius_m` is
rejected at load and at `createVehicle`; a missing `falloff_radius_m` key loads
as 0.0 (flat law), preserving backward compatibility.

Available wrappers:

| API | Contract | Status |
| --- | --- | --- |
| `SingleRun` | Runs one vehicle and writes trajectory records. | Implemented MVP; versioned CSV or binary frame-aware output. |
| `ParamSweep` | Repeats a scenario over a scalar callback and writes result records. | Implemented MVP; reports the configured `primaryEntityIndex`. |
| `MonteCarlo` | Perturbs copied scenarios and writes result records. | Implemented MVP; reports the configured `primaryEntityIndex`; RNG controls remain limited. |
| `Optimizer` | Runs particle-swarm parameter studies with callbacks. | Implemented MVP. |
| `BatchRunner` | Runs isolated scenarios and returns structured summary results. | Implemented MVP; frame-aware primary metrics and trajectory-maximum aggregate metrics (`maxAltitudeM`, `maxSpeedMps`) can be serialized by `StudyOutputWriter`. |

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
compatibility. A binary reader (`StudyOutputReader::read`) is part of the
supported contract: it inverts the binary v1 layout exactly, returning the
record type, the field list in file order, and the decoded records. Richer
telemetry schemas are not yet part of the supported contract.

## 9. Capability matrix and boundaries

Implement or MVP: CPU SoA kernel, per-entity vehicle physics, 6-DOF rigid
body, ISA1976 atmosphere, aero/propulsion, RK4/RK45/Euler/Symplectic,
terrain/wind callbacks, impact events, deterministic failure/damage models
(motor, actuator, sensor, structural, communication) with events, sensors,
navigation EKF, RF/IR/SARH/PassiveRF seekers with chaff/flare decoys,
PN/APN/waypoint guidance, autopilot, WGS84/ECEF/local-earth models,
optional ECEF kernel truth, batch/sweep/Monte Carlo/optimizer tooling, and
installable CMake packaging. The flattened per-vehicle subsystem `VehicleConfig`
surface, snake_case JSON/design/scenario serialization, per-entity sensor
enablement, multi-stage propulsion with stage separation, and warhead fusing
(impact/proximity/timed) are also implemented (MVP). Designer-facing design
manifests (`data/profiles`) and scenarios (`data/scenarios`) are engine-
consumable and exercised end-to-end via `designer_pipeline_test`; the profile-id
database layer (aero/motor/seeker/sensor/RCS) resolves the manifest's parts.
A first-principles WGS84 rocket-launch verification (`rocket_mvp_test`) cross-
checks the propulsion and ballistic truth against hand-computed expectations:
initial acceleration against `T/m − g_lat`, burnout time against the pressure-
interpolated-Isp band, and burnout velocity against the ideal rocket equation
`Δv = Isp·g0·ln(m0/mdry)`, confirming the `T = ṁ·Isp·g0` motor law.
Data-driven cd(M, α)/cl(M, α) aerodynamic coefficient tables (`aero_tables`),
bilinearly interpolated and clamped to grid bounds with a constant-coefficient
fallback, are also implemented (MVP); when present they are authoritative for
cd/cl, while moments and side-force remain the linear engineering model.

Planned or partial: higher-fidelity aero (moment and lateral/β coefficient
tables, Mach-scaled fin effectiveness),
probabilistic failure degradation, partial
health effects and repair, advanced atmosphere, full global
terrain/DEM ingestion, geoid models, imaging IR, multi-target seeker
tracking, dynamic SARH illuminator tracking, band-resolved extinction,
sensor fusion, trajectory/energy management, pursuit, LQR/MPC,
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
