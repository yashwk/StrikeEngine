# StrikeEngine Specification

**Status:** authoritative normative specification
**Version:** 0.1.0
**Verified:** 2026-08-29
**Repository:** StrikeEngine

Requirements use **MUST / MUST NOT / SHOULD / MAY** in their usual normative
sense. [`IMPLEMENTATION.md`](IMPLEMENTATION.md) is the companion implementation
record; [`FIDELITY_AUDIT.md`](FIDELITY_AUDIT.md) is measured evidence and does
not override this specification.

## 1. Product boundary

StrikeEngine is a C++23 simulation library for deterministic, data-oriented
multi-entity vehicle simulation. It owns 6-DOF truth dynamics, atmosphere, aero,
propulsion, gravity, earth-frame models, sensors/navigation/seeker/guidance/
autopilot/events, CPU + optional Vulkan backends, study wrappers, public headers,
validation tests, and installable CMake packaging.

Boundaries:

- StrikeEngine is library-only; shell/editor/visualization/ECS mapping belongs to
  a consuming application.
- StrikeSim owns Design/Simulate/CEM surfaces and consumes the installable
  StrikeEngine package.
- StrikeCEM is separate (offline CLI + `strikecem_lib` embedding boundary for
  StrikeSim); it produces RCS signature tables. StrikeCFD, a separate project in
  another folder, produces aero tables (Barrowman first-cut → CFD validation).
  StrikeDesigner is intended to be built inside StrikeSim on the stabilized
  engine.
- StrikeDesigner owns design intent/identity, StrikeCEM computation/validation/
  provenance, StrikeEngine runtime simulation + signature/DB lookup. Integration
  carries identity and revision metadata explicitly.

## 2. Capability status vocabulary

| Status | Meaning |
| --- | --- |
| Implemented | Public behavior exists and is covered by the current Release validation suite. |
| MVP / partial | A bounded implementation exists; named fidelity or integration limits remain. |
| Planned | Recorded as desired; not part of the supported runtime contract. |
| Unsupported | Callers MUST NOT rely on it; no silent fallback is promised. |

Current validated checkpoint: **34/34 CTest tests passing** in Release.

## 3. Global contracts

### 3.1 Units and numerical conventions

- Metres, m/s, m/s², kg, N, N·m, seconds, radians.
- Quaternions stored `(qw, qx, qy, qz)`, mapping body vectors to the world frame.
- Angular rates and achieved fin deflections are state variables.
- Zero-based component order; SoA arrays share a common entity index.
- Not thread-safe at `SimulationKernel` level; independent kernels MAY run
  concurrently when callbacks/models are independent.

### 3.2 State ownership and invariants

`SimulationKernel` owns truth/command/navigation/sensor/seeker/guidance/status
blocks; callers MUST treat const blocks as read-only snapshots (all accessors incl.
`getSeekers()` return const refs). A vehicle ID is a stable SoA slot until reused;
reuse restores default guidance (`None`), zeroed targets/commands, default sensor
noise. Invariants per active entity: arrays have an entry at the index; quaternion
normalized after integration; mass ≥ `massDry`; fins ≤ `maxDeflectionRad`
(integrator clamp, §7.4); `ax/ay/az` refreshed each step; a ground-impact-
deactivated entity is not advanced.

### 3.3 Determinism

CPU truth is deterministic for the same initial state, commands, configuration,
timestep sequence, sensor seed. `setRandomSeed` controls noise/random-walk bias;
unseeded operation is allowed but not reproducible. `BatchRunner` MUST create an
isolated kernel per scenario.

## 4. Coordinate frames

### 4.1 Body frame

Aerospace-like: X forward, Y right, Z down. Positive pitch-fin → nose-up; yaw-fin
→ nose-right. `wx/wy/wz` are body rates.

### 4.2 Local world frame

Default is local flat-earth: X east, Y north, Z up. Short-range scenarios use Z as
altitude, ground at `z = 0` unless a terrain callback is configured. Backward
compatible.

### 4.3 ECEF truth frame

When `EnvironmentConfig::earth.useEcefTruth` is true, `PhysicsBlock` position/
velocity are absolute WGS84 ECEF. The kernel resolves geodetic altitude for
atmosphere/ground, evaluates earth acceleration in ECEF, keeps GPS/INS in ECEF.
`referenceLatitudeRad`/`referenceLongitudeRad` anchor terrain callbacks in local
ENU. Scenario authors MUST provide absolute ECEF states, normally via
`geodeticToEcef`.

### 4.4 Earth models

Public earth headers provide: WGS84 geodetic/ECEF conversion; explicit
geodetic-state (ENU velocity) ↔ ECEF-state (ECEF velocity) conversion;
ECEF↔ENU, ECEF↔NED, ENU↔NED transforms; normal gravity (Somigliana formula +
altitude/free-air correction); spherical point-mass gravity (WGS84 µ); optional
central + J2 zonal-harmonic gravity; local Coriolis, centrifugal, and
moving-origin transport terms; and a standalone rotating-Earth ECEF RK4
propagator.

| Option | Behavior |
| --- | --- |
| `useWgs84Gravity` | Latitude/altitude-dependent normal gravity. |
| `useSphericalGravity` | Radial point-mass gravity in local ENU/ECEF. |
| `includeJ2Gravity` | Central + WGS84 J2 gravity; takes precedence over the other gravity models. |
| `includeCoriolis` | `-2 Ω × v`. |
| `includeCentrifugal` | `-Ω × (Ω × r)`. |
| `includeTransportRate` | Moving-origin `-Ω_en × v`; ignored in ECEF truth. |
| `useEcefTruth` | Absolute ECEF state and ECEF-aware consumers. |

`includeJ2Gravity` takes precedence over `useWgs84Gravity` and
`useSphericalGravity` in the kernel and is also available as an option on the
standalone rotating-Earth propagator. Without J2, both legacy gravity flags set
→ WGS84 wins;
in ECEF truth, spherical is the fallback when normal gravity is off. All-options-
false default: legacy constant `-9.80665 m/s²` in local Z. Not a complete
geophysical model (no geoid separation, automatic multi-tile terrain
discovery/streaming, atmospheric rotation/wind coupling, or full moving-origin
global propagator).

## 5. Kernel API contract

Primary class: `StrikeEngine::Kernel::SimulationKernel`.

### 5.1 Lifecycle

1. Construct a CPU kernel (or Vulkan when the optional library is linked).
2. `setEnvironment` before creating/stepping when non-default behavior is needed.
3. Optionally `setRandomSeed` before stepping.
4. `createVehicle(init[, vehicleConfig])`.
5. Queue guidance commands.
6. `step(dt)` or `runSteps(count, dt)`.
7. Read const snapshots; process subscribed events externally.

`reset()` clears SoA blocks, commands, time, free slots, seeker tracking, sensor
random-walk bias and GPS phase, preserving kernel/backend. `initialize()` resets
the simulation. `step(dt)` advances truth, then sensors, navigation, seekers,
guidance, autopilot, finally events; throws `std::invalid_argument` when
`dt <= 0.0`.

### 5.2 Vehicle initialization and configuration

`VehicleInitState` supplies the 6-DOF pose, mass, allegiance. Type, seeker type,
signature profile IDs, emitter EIRP, and principal inertias moved to
`VehicleConfig`.

`VehicleConfig` is the flattened per-vehicle view: structural summary (`type`, an
`EntityType` defaulting to `Missile`, `initialMass`, `massDry`, `Ixx/Iyy/Izz`)
plus subsystem structs `aero`, `propulsion`, `seeker`, `sensor`,
`guidanceAutopilot`, `warhead`, the profile-id fields `aeroProfileId`/
`motorProfileId`/`seekerProfileId`/`sensorProfileId` (default `""`), and signature
fields `rcsProfileId`, `irProfileId`, `emitterEirpW`.

- `AeroConfig`: `referenceArea` (JSON `reference_area`)/length, drag/lift
  coefficients, optional `aero_tables` (§6.2), optional `fins` (§6.2).
- `PropulsionConfig`: ordered `stages` of `StageConfig` (`thrustCurve`,
  `vacuumIsp`/`seaLevelIsp`, `propellantMassKg`, `dryMassKg`, ignition/shutdown
  timing, TVC limits/servo parameters, and engine position).
- `SensorConfig`: `imuEnabled`/`gpsEnabled`, IMU/GPS noise/bias σ, `gpsUpdateRateHz`,
  IMU body-frame lever arm.
- `GuidanceAutopilotConfig`: `navigationConstant`, `waypointGain`, gains
  `kAccelP/kRateP/kAlphaP/kRollP/kRollD`, `maxDeflectionRad`,
  `servoTimeConstantSec`, `maxServoRateRadPerSec`.
- `WarheadConfig`: `massKg`, `FusingType` (`Impact`/`Proximity`/`Timed`),
  `proximityTriggerM`, `timedDelaySec`, `lethalRadiusM`, optional `falloffRadiusM`
  (0.0 default = flat lethal-radius law; §8).
- `SeekerConfig`: per-entity seeker type and RF/IR/SARH params.

Empty propulsion stages are inactive/coasting. A motor burns only while fuel
remains; mass flow is clamped at the stage-aware dry-mass floor. Fixed axial
stages produce thrust along body +X. Stages with TVC use achieved pitch/yaw
gimbal angles and apply thrust torque about the configured engine position.
Staging §6.5, transient/TVC details §6.6, failure details §8.

Profile-id resolution: `aeroProfileId`/`motorProfileId`/`seekerProfileId`/
`sensorProfileId` (JSON keys `aero_profile_id`/`motor_profile_id`/
`seeker_profile_id`/`sensor_profile_id`; RCS key `rcs_profile_id`) may name a JSON
profile. On `createVehicle`, a non-empty id loads the file and its parsed config
REPLACES the inline sub-config (profile authoritative, per the `designRef`
precedent); empty id leaves inline untouched. An unloadable profile (missing file,
malformed JSON, missing required key, wrong-typed field) makes `createVehicle`
throw `std::runtime_error` naming the file. Guidance/autopilot, warhead,
mass/inertia, and RCS/IR/emitter signature fields are NOT profile-resolved.

### 5.3 Commands and guidance state

`SimulationCommand` supplies guidance mode (a `GuidanceMode`), target
position/velocity, and an optional max-accel demand. Modes: `None` (ballistic),
`ProportionalNavigation` (relative-position/velocity PN), `Waypoint` (static
point). Queue applied on next step. `ControlBlock::thrustCommand` exists but is
not a throttle interface; callers MUST NOT expect it to change motor output.

### 5.4 Environment callbacks

`terrainElevation(x, y)` returns metres; local x/y are world horizontal coords,
ECEF-truth x/y are ENU displacement from the reference. `windVelocity(x, y, z,
time)` returns world/ECEF air-mass velocity; truth subtracts it before aero
evaluation. `EnvironmentConfig::globalTerrain` optionally supplies a WGS84
geodetic terrain source sampled by latitude/longitude. It takes precedence over
the local callback and is used in both local ENU and absolute ECEF truth modes.
Null callbacks/database → zero wind/terrain.

`GlobalTerrain` stores south-to-north, west-to-east cell-center elevations in
metres above the WGS84 ellipsoid. Callers select nearest-neighbor or bilinear
sampling. Samples explicitly report `Valid`, `PartialNoData`, `NoData`,
`OutOfCoverage`, or `Invalid`; partial bilinear neighborhoods renormalize valid
weights. Longitude is normalized across the dateline, and full-width rasters may
wrap periodically. `surface()` returns an upward ENU unit normal and slope angle
from WGS84 local metric scales. The dependency-free loader accepts ESRI/ArcInfo
ASCII Grid files, reversing their north-first row order and recognizing
`xllcorner`/`xllcenter`, `yllcorner`/`yllcenter`, `cellsize`, and `NODATA_value`.

When built with `STRIKEENGINE_WITH_GDAL=ON`, `loadGdalTerrain()` accepts any
installed GDAL single-band raster driver, including GeoTIFF/COG, DTED, and VRT.
The source MUST provide a geotransform and CRS transformable to WGS84; the
adapter applies band scale/offset and NODATA handling, then eagerly normalizes
the source to a deterministic WGS84 raster. `TerrainTileCache` provides a
thread-safe bounded LRU cache of these normalized sources; a VRT is the supported
way to supply a multi-file mosaic. GDAL-disabled builds retain the ASCII and
in-memory paths and reject GDAL loading clearly. `GroundImpact` events include
the sampled terrain elevation, ENU normal, and slope.

### 5.5 Configuration serialization and design interchange

All config structs serialize to snake_case JSON, enums as snake_case strings.
Public API operates on JSON text; nlohmann/json never exposed in public headers.

- `serializeVehicleConfig`/`deserializeVehicleConfig`: full `VehicleConfig`.
- `serializeEnvironment`/`deserializeEnvironment`: `earth` block only; on load
  terrain/wind callbacks and the non-serializable `globalTerrain` pointer reset
  to flat/zero.
- `serializeScenario`/`deserializeScenario`, `ScenarioConfig::save(path)` (false
  if unopenable), static `ScenarioConfig::load(path)` (throws `std::runtime_error`
  on missing/malformed).
- `serializeDesign(name, geometryJson, physics)` writes `{"name","geometry",
  "physics"}` (`geometry` omitted when blank); `loadDesignPhysics(path)` reads
  `physics` back into a `VehicleConfig`. A non-empty
  `ScenarioEntityConfig::design_ref` is resolved on scenario load and OVERRIDES
  any inline `vehicleConfig`.

Design manifests (`data/profiles`) are `{"name","description","physics":
<VehicleConfig snake_case>}` docs; `physics` IS the `VehicleConfig`, and its
profile-id fields (`aeroProfileId`/`motorProfileId`/`seekerProfileId`/
`sensorProfileId`/`rcsProfileId`) reference files under `data/aero`, `data/motors`,
`data/seekers`, `data/sensors`, `data/rcs`. On `createVehicle` profile ids are
authoritative and replace the inline placeholders, which must still exist because
`VehicleConfig::from_json` requires every sub-object key. The chain — manifest
`design_ref` → `loadDesignPhysics` → profile-id resolution → SoA wiring → RF/RCS →
guided intercept — is covered by `designer_pipeline_test`.

Frame note: designers MUST emit local ENU coordinates, not ECEF Earth-radius
offsets. `intercept_test_01` runs in local ENU (§4.2), re-baselined to 15 km/8 km
(the table-aero missile is faster/lower-drag; 20 km/10 km no longer fits the
seeker's ~3 km acquisition range) — a deterministically tuned set (seed
`0xDEADBEEF`), pipeline evidence (min miss 45.4 m at t≈11.5 s; proximity kill),
not a performance claim.

`VehicleInitState` fields are optional in JSON with safe defaults (zero pose,
identity quaternion, mass 0, `friendly`). Deserializers throw `std::runtime_error`
on malformed JSON or unknown enum strings. Profile files share the inline flat
snake_case schema; only seeker `type` and motor `stages` are required, the rest
default from the struct.

## 6. Truth dynamics

### 6.1 Translational dynamics

CPU backend rotates body aero/thrust force to the world frame, divides by current
mass, adds configured gravity and rotating-earth terms. Position derivative is
world-frame velocity.

### 6.2 Aerodynamics

`BasicAeroModel` evaluates drag (ISA dynamic pressure opposing body air-relative
velocity), AoA/fin lift, bounded lift coefficient, yaw-fin side force, static pitch
stability, rate damping, bounded moments. Coefficients/geometry per entity.

Optional tables (`aero_tables` on `AeroConfig`; `data/aero/*.json`) give
static aerodynamic coefficients on rectilinear grids. `cd(M, α)`, `cl(M, α)`,
and `cm(M, α)` use `mach_breakpoints` × `aoa_breakpoints_rad`; `cy(M, β)`,
`cn(M, β)`, and rolling `cl(M, β)` use `mach_breakpoints` ×
`beta_breakpoints_rad`. The JSON fields are `cl_table`, `cd_table`, `cm_table`,
`cy_table`, `cn_table`, and `cl_roll_table`, each dimensioned `[mach][angle]`.
The `cl`/`cd` pair is required in a populated table block; moment and lateral
tables are optional. All supplied tables are bilinearly interpolated and
clamped to grid bounds. A valid supplied table replaces the corresponding
static scalar coefficient while fin-control and rate-damping terms remain
active. Without a supplied table, the established scalar/static fallback is
used byte-for-byte. A structurally invalid grid (missing/non-ascending/invalid
breakpoints, non-finite values, wrong dimensions, or fewer than two points per
axis) is rejected at parse and profile load. `Cy` is body +Y force, `Cm` is
body +Y pitch moment, `Cn` is body +Z yaw moment, and rolling `Cl` is body +X
moment. cd/cl tables come from StrikeCFD; moment/lateral tables are expected
from validated StrikeCFD data.

Optional geometric fins (`fins` on `AeroConfig`) port RocketPy's fin aerodynamic
model. `FinShape` selects `Trapezoidal`, `Elliptical`, or `FreeForm`; `count` 0
disables fins, `>=3` enables them. Geometry inputs: `rootChordM`, `tipChordM`,
`spanM`, `sweepLengthM` (<0 => root-tip sweep), `positionM` (fin-root leading-edge
axial offset from CG along body +X; nose positive, tail negative), `cantAngleDeg`,
and `shapePoints` (free-form). `buildFinsGeometry` precomputes Mach-dependent
`clAlpha`/`rollForcingPerRad`/`rollDampingCoeff` via RocketPy formulas: Diederich
planform lift slope with a Prandtl–Glauert Mach correction, fin-number correction
{5:2.37, 6:2.74, 7:2.99, 8:3.24} else n/2, lift-interference factor 1+1/τ
(τ=(span+r)/r), per-shape CP (trapezoidal closed-form; elliptical 0.288·root;
free-form mac_lead+0.25·mac_length), and roll-forcing/damping interference
factors. `cpLeverArmM` is the SIGNED body-X coordinate of the fin CP relative to CG
(negative = tail), producing restoring (stabilizing) pitch/yaw moments for
positive α/β.

When `fins` is present and valid, the geometry-derived Mach-dependent fin terms
REPLACE the flat abstract fins (`clFin`/`CM_delta`/`Cl_delta`/`CN_beta`); the body
terms (body `clAlpha` lift, body `CM_alpha` where applicable, `Cq`/`Clp` damping)
stay. The lateral/β side-force and static-stability terms are then modeled from fin
geometry. When `fins` is absent (default), the legacy flat-fin path is
byte-identical. JSON: optional `fins` object under `aero` with `"shape"`
("trapezoidal"/"elliptical"/"freeform"), `count`, `position_m`, `cant_angle_deg`,
`root_chord_m`, `span_m`, plus `tip_chord_m`+`sweep_length_m` (trapezoidal) or
`shape_points` (free-form). Parsed in `ConfigSerialization.cpp` and
`AeroProfileDatabase.cpp` with fail-fast validation: `count` <3, or a free-form
with <3 points, throws.

### 6.3 Rotation and actuators

CPU truth integrates diagonal inertia with gyroscopic coupling, body rates,
body-to-world quaternion. Autopilot transforms world accel demands to body axes
and supplies bounded fin commands; fins follow via first-order servo lag and a
rate limit; integrator clamps final deflection.

### 6.4 Integration

Supported: Euler, true RK4, Symplectic/Velocity-Verlet, adaptive RK45. RK4/RK45
re-evaluate forces at intermediate stages; RK45 uses bounded adaptive substeps.
Impact crossing interpolated by the event system. Selectable at construction via
`IntegratorType` (default RK4, CPU backend) in addition to standalone availability.

### 6.5 Multi-stage propulsion (staging)

`PropulsionConfig::stages` is ordered; every non-empty-curve stage registers with
the propulsion pool, the first active at launch. Initial `massDry` = final dry
mass + sum of separable dry masses of all stages except the last.

Each physics step the staging pass detects active-stage burnout (last positive-
thrust curve time); on burnout it jettisons leftover (unburned) propellant, drops
the spent stage's `dryMassKg` from mass and `massDry`, rescales `Ixx/Iyy/Izz` by
the post-dump current:new mass ratio, advances to the next stage, resets ignition
time, dispatches timestamped `StageSeparation`.

A stage with `propellantMassKg > 0` burns only its declared propellant (mass-flow
floor = `massDry` + later-stage reserves), separating on exhaustion or curve end;
`<= 0` keeps curve-end behavior. The leftover dump lands mass exactly on the new
floor (`dumpedMassKg` on the event); exhaustion burnouts had already pinned mass
there, so leftover ≈ 0 and they are byte-identical to pre-dump behavior.

`T = ṁ·Isp·g0` is enforced at every instant, including fuel exhaustion: the
fuel-depletion guard caps mass flow to remaining propellant and scales thrust with
capped flow, so thrust self-terminates at exhaustion — no free-thrust tail.

### 6.6 Propulsion transients, TVC, and validation

Each stage may define `ignition_delay_sec` and `ignition_ramp_sec` relative to
its ignition time. Before the delay thrust and flow are zero; during the ramp
the thrust/flow multiplier increases linearly to one. `shutdown_time_sec` is
relative to stage ignition after the ignition delay and is optional (`-1.0`
means curve-controlled cutoff). If non-negative, `shutdown_ramp_sec` linearly
reduces thrust/flow to zero; zero gives an immediate cutoff. Staging burnout
time includes ignition delay and the effective shutdown endpoint.

TVC fields are `max_gimbal_pitch_rad`, `max_gimbal_yaw_rad`,
`gimbal_time_constant_sec`, and `max_gimbal_rate_rad_per_sec` (`0` rate means
unlimited). Zero pitch/yaw limits preserve fixed axial thrust. Positive pitch
gimbals thrust toward body `-Z`; positive yaw toward body `+Y`. Commands are
set with `SimulationKernel::setThrustVectorCommand(id, pitch, yaw)` and are
clamped to stage limits. Achieved gimbal angles are first-order servo state,
rate-limited, integrated by every supported CPU integrator, and reset at stage
separation. `engine_position_x/y/z` is the body-frame engine position relative
to the center of gravity; its cross product with the thrust vector contributes
to rotational dynamics.

Non-empty curves require at least two points, start at time zero, have finite
strictly increasing non-negative times, non-negative finite thrust, and contain
positive thrust. Isp, mass, timing, TVC, and engine-position fields are checked
for finite values and valid ranges before backend registration. Invalid inline
configs throw `std::invalid_argument`; invalid motor profiles fail to load.
Serialization reads these fields when present and defaults them for legacy
profiles.

## 7. Sensors, navigation, seekers, and guidance

### 7.1 Sensors

IMU: body specific force and rates with Gaussian noise and random-walk biases.
Optional per-entity lever arm (`imuLeverArmX/Y/Z`, default zero) adds the
rigid-body correction `α×l + ω×(ω×l)`. In ECEF truth, `includeEarthRateGyro`
(default false) models the gyro as measuring the INERTIAL body rate — adds
`ω_ie^b = C_e^b (0,0,Ω)` to body rate; INS subtracts it (resolved with estimated
attitude). Ignored in local mode. GPS: noisy position/velocity at the configured
cadence, in the selected world frame (incl. ECEF).

Per entity: `imuEnabled=false` freezes the IMU (held sample, bias drift stops)
while GPS continues to aid the EKF — a stale-sample degradation, not full GPS-only
positioning. `gpsEnabled=false` stops GPS updates; `gpsUpdateRateHz` sets cadence.

### 7.2 Navigation

Perfect initial alignment, strapdown INS, coupled 15-state error-state EKF.
Attitude update is a rotation-vector step (exact delta-quaternion per sample,
validated against a fine-step reference), bounding coning drift; velocity update
applies single-interval sculling/rotation compensation `+0.5(ω×f)dt²` in the body
frame. Covariance: position, velocity, attitude error, accel bias, gyro bias,
row-major 15×15. GPS position/velocity correct the coupled state. MVP: no
multi-rate timestamp interpolation; earth-rate compensation only in ECEF truth
(local truth gyro resolves the non-rotating-frame rate; correct local compensation
needs the rotating-frame ECEF path).

### 7.3 Seekers and signatures

RF: monostatic radar range equation with RCS profile. SARH: bistatic equation with
configured off-board illuminator (`SeekerConfig::illuminator*`; target RCS
approximates the bistatic cross section). PassiveRF: homes on target EIRP
(`emitterEirpW`); `emitterEirpW = 0` targets not passively detectable. IR:
radiant-intensity profile, inverse-square irradiance, Beer-Lambert transmittance
`exp(-k·r)` with `irExtinctionPerM` (default `1e-4` m⁻¹ = 0.1/km). Acquisition
picks the strongest valid signal (SNR dB for RF/SARH/PassiveRF, W for IR) among
hostiles, single-track lock; chaff/flare decoys carry RCS/IR profiles and can
seduce under it. Seeker params via public `SeekerConfig` on `VehicleConfig::seeker`.
FOV, azimuth/elevation gimbal limits, lock hysteresis, dropout timing, filtered LOS
rates, measurement latency implemented. Friendly targets rejected. SARH
illuminator is a STATIC configured position; dynamic illuminator tracking future.

### 7.4 Guidance and autopilot

Stateless PN/APN helpers in `models/guidance`. Kernel PN uses target
position/velocity; kernel APN uses filtered seeker LOS rates on lock. Seeker-locked
APN clamps commanded accel to per-entity `maxAccel`, matching PN/Waypoint.
Autopilot translates world accel into bounded body fin demands.

Constants per entity, read from config at creation: `navigationConstant`,
`waypointGain`, gains (`kAccelP/kRateP/kAlphaP/kRollP/kRollD`), `maxDeflectionRad`
(fin clamp, default 0.43 rad). Read from per-entity blocks, not class constants.
Planned, not supported: trajectory management, pursuit, LQR/MPC, blended handoff,
imaging IR, multi-target tracking, dynamic SARH illuminator tracking.

## 8. Events and simulation tools

Ground impact is a real transition: position clamped, velocity/acceleration
cleared, entity deactivated, timestamped `GroundImpact` event. Other event enum
values are extension points, not all generated.

Failure/damage injection (MVP): `failEntity(id, mode)` and `applyDamage(id,
damage)`. Legacy `MotorFailure` and explicit `EngineFailure` set the motor/engine
flags, stop thrust and flow, and dispatch their corresponding events.
`TankFailure` sets a separate tank/feed flag, stops thrust and flow, retains
residual propellant, and dispatches `TankFailure`. Actuator/sensor/communication
failures set per-entity flags and dispatch `ActuatorFailure`/`SensorFailure`/
`CommunicationFailure` with timestamp + id. Structural failure (`failEntity(StructuralFailure)` or health
0) deactivates like ground impact (`isAlive=false`, `health=0`, `active=false`,
velocity/accel cleared) and dispatches `StructuralFailure`. Flags consumed by
systems: motor/engine/tank zero thrust and mass flow; actuator freezes fins; sensor stops
updates; comms zeroes commanded accel (ballistic). Flags deterministic, not
probabilistic; partial health has no effect; repair not modeled.

Multi-stage vehicles dispatch `StageSeparation` with `dumpedMassKg` (§6.5). Warhead
fusing evaluated each step after ground-impact detection:

- `Impact`: detonates when the carrying entity is deactivated (ground impact);
- `Proximity`: detonates when an alive entity is within `proximityTriggerM`;
- `Timed`: detonates `timedDelaySec` after launch.

Detonation dispatches `Detonation` and applies a fragmentation/overpressure kill
law to every other alive entity. `falloffRadiusM <= 0` (or `<= lethalRadiusM`):
flat law, guaranteed kill inside `lethalRadiusM`, none beyond — pre-feature,
byte-identical. `falloffRadiusM > lethalRadiusM`: p=1 inside `lethalRadiusM`,
linear decay `(falloffRadiusM − d)/(falloffRadiusM − lethalRadiusM)` across
`(lethalRadiusM, falloffRadiusM]`, 0 beyond. Kill deterministic when p ≥ 1;
otherwise an RNG draw only when `0 < p < 1`, so flat-law warheads never consume the
RNG stream. `lethalRadiusM <= 0` is inert. `falloff_radius_m` below
`lethal_radius_m` is rejected at load and at `createVehicle`; a missing
`falloff_radius_m` key loads as 0.0 (flat law).

| API | Contract | Status |
| --- | --- | --- |
| `SingleRun` | Runs one vehicle; writes trajectory records. | Implemented MVP; versioned CSV or binary. |
| `ParamSweep` | Repeats a scenario over a scalar callback; writes results. | Implemented MVP; reports `primaryEntityIndex`. |
| `MonteCarlo` | Perturbs copied scenarios; writes results. | Implemented MVP; `primaryEntityIndex`; limited RNG controls. |
| `Optimizer` | Particle-swarm parameter studies via callbacks. | Implemented MVP. |
| `BatchRunner` | Isolated scenarios; structured summaries. | Implemented MVP; frame-aware primary + trajectory-max aggregates (`maxAltitudeM`, `maxSpeedMps`) via `StudyOutputWriter`. |

`StudyOutputWriter` is the common study-output contract. CSV uses format-version
`1` metadata; binary uses 8-byte `STRKOUT1` magic + version, record type, field
count, record count, field IDs, typed little-endian records. Empty field list
selects record-type defaults; non-empty selects unique supported fields in order.
Every default record has status + frame identity; every state record has its
`Frame` (`LOCAL_ENU`/`ECEF`), selected-frame position/velocity, WGS84 lat/long
radians, and `Altitude_m` (world Z in local; derived from ECEF in ECEF).
`ScenarioConfig::primaryEntityIndex` selects the summarized entity (default 0).
`StudyOutputReader::read` inverts the binary v1 layout exactly. Richer telemetry
not yet in the supported contract.

## 9. Capability matrix and boundaries

Implemented or MVP: CPU SoA kernel; per-entity physics; 6-DOF rigid body; ISA1976
atmosphere; aero/propulsion; RK4/RK45/Euler/Symplectic; terrain/wind callbacks;
geodetic global terrain rasters with ESRI ASCII loading, nearest/bilinear
sampling, NODATA/coverage status, surface normals/slope, optional GDAL
GeoTIFF/DTED/VRT loading, and bounded source caching;
impact events; deterministic failure/damage (motor, actuator, sensor, structural,
communication) with events; sensors; navigation EKF; RF/IR/SARH/PassiveRF seekers
with chaff/flare; PN/APN/waypoint guidance; autopilot; WGS84/ECEF/local-earth
models; optional ECEF kernel truth; batch/sweep/Monte Carlo/optimizer tooling;
installable CMake packaging; flattened `VehicleConfig`; snake_case JSON/design/
scenario serialization; per-entity sensor enablement; multi-stage staging +
separation; warhead fusing (impact/proximity/timed); designer manifests
(`data/profiles`) and scenarios (`data/scenarios`) consumed end-to-end via
`designer_pipeline_test`; profile-id database layer (aero/motor/seeker/sensor/
RCS); data-driven static cd(M,α)/cl(M,α)/cm(M,α)/cy(M,β)/cn(M,β)/cl(M,β)
tables (`aero_tables`, bilinear + clamped, scalar fallback); geometric fins
(trapezoidal/elliptical/free-form, RocketPy port, Mach-dependent lift/stability/
roll). `rocket_mvp_test` cross-checks propulsion/ballistic truth
(initial accel vs `T/m − g_lat`, burnout time vs pressure-interpolated Isp band,
burnout velocity vs `Δv = Isp·g0·ln(m0/mdry)`), confirming `T = ṁ·Isp·g0`.

Planned or partial: CFD validation and higher-order aero (Reynolds dependence,
nonlinear stall/post-stall, body/fin interference, flexible-body effects);
probabilistic failure degradation; partial health/repair;
advanced atmosphere; automatic spatial multi-tile terrain discovery/streaming,
terrain tile prefetch, vertical datum/geoid models, and higher-fidelity polar
coverage; imaging IR; multi-target
tracking; dynamic SARH illuminator tracking; band-resolved extinction; sensor
fusion; trajectory/energy management; pursuit; LQR/MPC; richer telemetry; parallel
CPU; CUDA; production GPU backend. Optional ECS/editor mapping,
visualization/plotting/analysis/scenario tooling, and an API server are
integration/tooling ideas, not kernel features.

Unsupported behavior MUST fail clearly or remain opt-in. The engine MUST NOT
silently substitute a future solver, GPU path, terrain database, material model, or
controller not described and validated here.

## 10. Change control

New public behavior MUST update this specification, the implementation record,
affected schemas/configuration comments, and at least one regression test. A
feature is not "implemented" merely because a header or roadmap item exists:
accepted inputs, outputs, defaults, failure behavior, and test evidence must be
documented.
