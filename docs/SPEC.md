# StrikeEngine Specification

**Status:** authoritative normative specification
**Version:** 0.1.0
**Verified:** 2026-08-29
**Repository:** StrikeEngine

Requirements use **MUST / MUST NOT / SHOULD / MAY** in their usual normative
sense. [`IMPLEMENTATION.md`](IMPLEMENTATION.md) is the companion implementation
record; [`FIDELITY_AUDIT.md`](FIDELITY_AUDIT.md) is measured evidence and does
not override this specification. [`STRIKEDESIGNER_INTEGRATION.md`](STRIKEDESIGNER_INTEGRATION.md)
defines the downstream Designer handoff.

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

Current validated checkpoint: **38/38 CTest tests passing** in Release.

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
the simulation. `step(dt)` advances truth, then sensors, navigation, seekers, the
persistent target-track manager (§7.5), guidance, autopilot, finally events;
throws `std::invalid_argument` when `dt <= 0.0`.

### 5.2 Vehicle initialization and configuration

`VehicleInitState` supplies the 6-DOF pose, mass, allegiance. Type, seeker type,
signature profile IDs, emitter EIRP, and principal inertias moved to
`VehicleConfig`.

`VehicleConfig` is the flattened per-vehicle view: structural summary (`type`, an
`EntityType` defaulting to `Missile`, `initialMass`, `massDry`, `Ixx/Iyy/Izz` and
optional products of inertia `Ixy/Ixz/Iyz`) plus subsystem structs `aero`, `propulsion`,
`seeker`, `sensor`, `guidanceAutopilot`, `warhead`, the profile-id fields `aeroProfileId`/
`motorProfileId`/`seekerProfileId`/`sensorProfileId`/`guidanceProfileId`/`warheadProfileId`
(default `""`), and signature fields `rcsProfileId`, `irProfileId`, `emitterEirpW`.

- `AeroConfig`: `referenceArea` (JSON `reference_area`)/length, drag/lift
  coefficients, optional `aero_tables` (§6.2), optional `fins` (§6.2).
- `PropulsionConfig`: ordered `stages` of `StageConfig` (`thrustCurve`,
  `vacuumIsp`/`seaLevelIsp`, `propellantMassKg`, `dryMassKg`, ignition/shutdown
  timing, TVC limits/servo parameters, and engine position).
- `SensorConfig`: `imuEnabled`/`gpsEnabled`, IMU/GPS noise/bias σ, `gpsUpdateRateHz`,
  `gpsInnovationGateSigma` (<=0 disables scalar GPS outlier rejection; default
  5σ), IMU body-frame lever arm.
- `GuidanceAutopilotConfig`: `navigationConstant`, `waypointGain`, gains
  `kAccelP/kRateP/kAlphaP/kRollP/kRollD`, `maxDeflectionRad`,
  `servoTimeConstantSec`, `maxServoRateRadPerSec`. Dynamic pressure gain scheduling keys
  `gainSchedulingEnabled` (default false), `refDynamicPressurePa` (default 50000 Pa),
  `minDynamicPressurePa` (default 2000 Pa), and `maxDynamicPressurePa` (default 300000 Pa)
  scale the feedforward acceleration gain to prevent max-Q control flutter (§6.3). W38 phase/track keys
  `handoffBlendTimeSec` (default 0), `lockLossRetentionSec` (default 0), and
  `apnFeedforwardEnabled` (default false) are optional (§7.4). W39 track-manager
  keys `trackConfirmations` (default 3), `trackCoastTimeoutSec` (default 0.5),
  and `trackLossTimeoutSec` (default 2.0) are optional (§7.5).
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
`sensorProfileId`/`guidanceProfileId`/`warheadProfileId` (JSON keys
`aero_profile_id`/`motor_profile_id`/`seeker_profile_id`/`sensor_profile_id`/
`guidance_profile_id`/`warhead_profile_id`; RCS key `rcs_profile_id`) may name
a JSON profile. On `createVehicle`, a non-empty id loads the file and its parsed config
REPLACES the inline sub-config (profile authoritative, per the `designRef`
precedent); empty id leaves inline untouched. An unloadable profile (missing file,
malformed JSON, missing required key, wrong-typed field) makes `createVehicle`
throw `std::runtime_error` naming the file. Structural summary (mass/inertia) and
RCS/IR/emitter signature fields remain non-profile-resolved on `VehicleConfig`
(RCS tables are resolved via `RCSDatabase`).

### 5.3 Commands and guidance state

`SimulationCommand` supplies guidance mode (a `GuidanceMode`), target
position/velocity, an optional max-accel demand, and an optional `targetId`
(int64; `-1` = unknown identity) that seeds/refreshes the W39 persistent target
track (§7.5, §5.5). Modes: `None` (ballistic),
`ProportionalNavigation` (relative-position/velocity PN), `Waypoint` (static
point). Queue applied on next step. `ControlBlock::thrustCommand` exists but is
not a throttle interface; callers MUST NOT expect it to change motor output.
A scenario entity may seed identity via the optional `initial_target_id`
(`ScenarioEntityConfig`, default `-1`) which flows into `SimulationCommand::
targetId` on the initial guidance command (§5.5).

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
`0xDEADBEEF`, 120 m/s² initial guidance cap), pipeline evidence (min miss 9.7 m
at t≈11.5 s; proximity kill),
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

### 6.1.1 Multi-threaded CPUBackend parallel execution

`CPUBackend` supports multi-threaded evaluation across entities via an internal persistent
`WorkerPool`. Derivative evaluation (`evaluateDerivative`) and post-step acceleration cache
refreshing partition the active entity population into disjoint index ranges evaluated in parallel.
Worker thread counts can be configured at runtime via `SimulationKernel::setThreadCount(std::size_t)`
and queried via `threadCount()`. When configured with `threads <= 1` (the default), execution executes
directly on the calling thread with zero thread-synchronization or allocation overhead, ensuring
strictly bit-identical determinism with single-threaded baselines.

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

Optional geometric fins (`fins` or `finSets` on `AeroConfig`) port RocketPy's fin aerodynamic
model. Multiple fin sets (e.g. canards + aft tail fins) are supported via `finSets`;
when empty and `fins` is enabled, `fins` is treated as a single set for backward
compatibility. `FinShape` selects `Trapezoidal`, `Elliptical`, or `FreeForm`; `count` 0
disables fins, `>=3` enables them. `steerable` (default `true`) specifies whether the
fin set responds to servo control deflections (`finPitch`/`finYaw`/`finRoll`) or acts
as a passive stabilizing surface. Geometry inputs: `rootChordM`, `tipChordM`,
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
positive α/β. Control deflections produce consistent nose-UP / nose-RIGHT moments
proportional to `|cpLeverArmM|` across both canard and tail geometries.

When geometric fins are present and valid, the geometry-derived Mach-dependent fin terms
REPLACE the flat abstract fins (`clFin`/`CM_delta`/`Cl_delta`/`CN_beta`); the body
terms (body `clAlpha` lift, body `CM_alpha` where applicable, `Cq`/`Clp` damping)
stay. The lateral/β side-force and static-stability terms are then modeled from fin
geometry across all enabled fin sets. When no geometric fins are configured (default),
the legacy flat-fin path is byte-identical. JSON: optional `fins` object or `fin_sets`
array under `aero` with `"shape"` ("trapezoidal"/"elliptical"/"freeform"), `count`,
`position_m`, `cant_angle_deg`, `root_chord_m`, `span_m`, optional `steerable` (bool),
plus `tip_chord_m`+`sweep_length_m` (trapezoidal) or `shape_points` (free-form).
Parsed in `ConfigSerialization.cpp` and `AeroProfileDatabase.cpp` with fail-fast
validation: `count` <3, or a free-form with <3 points, throws.

### 6.3 Rotation and actuators

CPU truth integrates the full $3\times3$ symmetric rigid-body inertia tensor
$\mathbf{I}$ (`Ixx`, `Iyy`, `Izz`, `Ixy`, `Ixz`, `Iyz`) with dynamic gyroscopic
cross-coupling $\vec{M} = \mathbf{I}\dot{\vec{\omega}} + \vec{\omega}\times(\mathbf{I}\vec{\omega})$.
Angular acceleration is resolved via closed-form analytical $3\times3$ matrix inversion
$\dot{\vec{\omega}} = \mathbf{I}^{-1}(\vec{M} - \vec{\omega}\times(\mathbf{I}\vec{\omega}))$,
with a zero-divergence diagonal fast path when products of inertia are zero.
Quaternion propagation ensures exact unit-norm integration.

Autopilot transforms world acceleration demands from guidance into aerospace body axes
(X forward, Y right, Z down) and produces bounded fin commands. When dynamic pressure
gain scheduling is enabled (`gainSchedulingEnabled = true`), feed-forward acceleration
gains are scaled by $S_q = \text{clamp}\left(\sqrt{q_{\text{ref}} / \text{clamp}(q_{\text{est}}, q_{\text{min}}, q_{\text{max}})}, 0.2, 5.0\right)$
using estimated dynamic pressure $q_{\text{est}} = \frac{1}{2}\rho(h)V^2$ evaluated
via the ISA-1976 atmosphere. This prevents max-Q control saturation and flutter while
maintaining responsiveness at high altitude. Fins follow via first-order servo lag
and a rate limit; the integrator clamps final deflection.

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
the spent stage's `dryMassKg` from mass and `massDry`, rescales `Ixx/Iyy/Izz` and
products of inertia `Ixy/Ixz/Iyz` by the post-dump current:new mass ratio, advances
to the next stage, resets ignition time, dispatches timestamped `StageSeparation`.

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
Each GPS position/velocity scalar is normalized by its predicted innovation
standard deviation. If it exceeds `gpsInnovationGateSigma`, that scalar is
rejected without changing the state or covariance; valid scalars in the same
fix remain eligible. `NavigationBlock::lastGpsUpdateRejected` and
`lastGpsMaxInnovationSigma` expose diagnostics for the most recent fix.

### 7.2 Navigation

Perfect initial alignment, strapdown INS, coupled 15-state error-state EKF.
Attitude update is a rotation-vector step (exact delta-quaternion per sample,
validated against a fine-step reference), bounding coning drift; velocity update
applies single-interval sculling/rotation compensation `+0.5(ω×f)dt²` in the body
frame. Covariance: position, velocity, attitude error, accel bias, gyro bias,
row-major 15×15. GPS position/velocity correct the coupled state with the
configurable innovation gate above. MVP: no
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
position/velocity; kernel APN uses filtered seeker LOS rates on lock. Seeker
azimuth/elevation rates are mapped into the airframe's X-forward/Y-right/Z-down
body frame before rotating the command to world coordinates (azimuth → +Y,
elevation → −Z; unchanged contract). Seeker-locked APN
clamps commanded accel to per-entity `maxAccel`, matching PN/Waypoint.
Autopilot translates world accel into bounded body fin demands.

Phase selection is separated from law computation (§W38). Per entity the
guidance system tracks an explicit `GuidancePhase`
(`None`/`Midcourse`/`Acquisition`/`Terminal`/`LostTrack`) and a `GuidanceLaw`
(`None`/`Waypoint`/`Tpn`/`Apn`/`BodyRatePn`/`InertialPn`/`Trajectory`/`Cruise`). A seeker lock moves the
state `Midcourse → Acquisition → Terminal`: during `Acquisition` the terminal
APN weight `handoffWeight` ramps 0 → 1 over the configured
`handoffBlendTimeSec` (0 = the legacy instant override), blending midcourse PN
with the gyro-decoupled seeker PN. On lock loss during `Acquisition`/`Terminal`,
the layer retains the track identity and applies the bounded predicted terminal
command (the last valid seeker-PN demand, already clamped by `maxAccel`) for up to
`lockLossRetentionSec` (0 = none); once retention expires it drops to
`LostTrack` and recovers via midcourse PN on the commanded target. APN
target-acceleration feed-forward is emitted only when
`apnFeedforwardEnabled && targetAccelAvailable`, and only when the target-
acceleration inputs are finite; otherwise the law is pure PN. Target
acceleration is supplied through `SimulationCommand` (`targetAccelX/Y/Z`,
`targetAccelAvailable`) or scenario `initial_target_accel_*`. Non-finite
guidance input, a zero/negative closing (`N ≤ 0`, `Vc ≤ 0`) marks the demand
`lawInvalid`/`nonClosing` rather than emitting a spurious vector. Waypoint mode
reports `GuidanceLaw::Waypoint` and validates non-finite geometry the same way.

Midcourse PN consumes the persistent target track (§7.5) when it is
measurement-anchored (state active AND `updateCount > 0`), using the track's
world-frame position/velocity in preference to the raw external command aim; the
APN target-acceleration feed-forward then comes from the track when
`accelAvailable` and finite. When the track is inactive, lost, or absent (state
`None`/`Lost`, or `updateCount == 0`), midcourse PN falls back to the external
command state (`SimulationCommand`/scenario) exactly as before — the legacy
behavior is preserved byte-for-byte. Guidance NEVER reads physics truth: all aim
states come from the navigation estimate plus either the command or the track.

Guidance publishes per-entity diagnostics on `GuidanceBlock`: `phase`, `law`,
`trackId` (−1 none), `trackAgeSec`, `handoffWeight`, `lockLossCount`,
`rawAccelX/Y/Z` (pre-clamp demand), `limitedByMaxAccel` (demand clamp),
`lawInvalid`, `nonClosing`, and `tgoSec` (range / closing speed), plus
`retainedAccelX/Y/Z` (last valid post-clamp terminal demand, replayed during the
lock-loss retention window). Every demand
passes the per-entity `maxAccel` clamp and publishes the raw + limited flags.
`ControlBlock` adds `pitchSaturated`/`yawSaturated`/`rollSaturated` autopilot
fin-clamp diagnostics, distinct from guidance `limitedByMaxAccel`.

The `handoffBlendTimeSec`/`lockLossRetentionSec`/`apnFeedforwardEnabled` keys
are config-backed and optional (defaults 0/0/false). Defaults select the legacy
path byte-for-byte: seeker lock overrides to APN instantly, no guidance-layer
retention, no feed-forward. `GuidanceAutopilotConfig` uses explicit
`to_json`/`from_json` (not the NLOHMANN macro) so legacy design manifests and
scenarios that omit the three keys still load.

Constants per entity, read from config at creation: `navigationConstant`,
`waypointGain`, gains (`kAccelP/kRateP/kAlphaP/kRollP/kRollD`), `maxDeflectionRad`
(fin clamp, default 0.43 rad). Read from per-entity blocks, not class constants.
The `Acquisition->Terminal` seeker handoff blend is implemented and
config-backed (§7.4). Trajectory-aware midcourse management is implemented
(§7.6). Planned, not supported: trajectory optimization/energy management,
pursuit, LQR/MPC, imaging IR, multi-target tracking, dynamic SARH illuminator
tracking.

### 7.5 Persistent target-track manager (W39)

`SimulationKernel::step()` runs the target-track manager (step 3.75) AFTER
seekers and BEFORE guidance (§7.4). It fuses external command seeds
(`SimulationCommand`, §5.3) and seeker LOS measurements into one PER-ENTITY track
consumed by guidance. This is a SINGLE persistent track per seeker entity — NOT
multi-target tracking (that remains planned, §7.4).

Per-entity state lives in `TrackBlock` (`include/strikeengine/kernel/data/
TrackBlock.hpp`):

- `TrackState` enum: `None` (no track) / `Acquire` / `Maintain` / `Coast` /
  `Lost` / `Reacquire`; `active()` is true for every state except `None`/`Lost`.
- identity `trackId` (int64; `-1` = unknown).
- world-frame `posX/Y/Z`, `velX/Y/Z`, `accelX/Y/Z` + `accelAvailable` (optional
  target acceleration, from the command seed only).
- `timestampSec` (sim time of the last measurement/seed) and `ageSec` (time since
  it).
- `positionStdM` / `velocityStdMs` (growing uncertainty model) and `quality01`
  (exponential decay).
- `updateCount` / `dropoutCount` (consecutive measurement updates / steps without
  one), plus `measPosX/Y/Z` / `measTimeSec` (previous fix, for finite-difference
  velocity).

Lifecycle (deterministic, per entity):

1. A `SimulationCommand` seeds the track into `Acquire` with the command
   identity/position/velocity/optional acceleration; a scenario can seed identity
   via `initial_target_id` (§5.3, §5.5).
2. A seeker LOS fix converts body LOS angles to a world-frame estimate through
   the navigation estimate (LOS body angles → body LOS via the aerospace
   X-forward/Y-right/Z-down convention → world via the nav quaternion;
   measured position = nav position + LOS_world × range; velocity = finite
   difference between consecutive fixes — NEVER physics truth). A re-lock on a
   different target starts a fresh track.
3. After `trackConfirmations` (default 3) consistent measurement updates the
   state promotes `Acquire → Maintain` (`updateCount` resets to 1 on the first
   fix, then increments per consistent fix).
4. No measurement for `trackCoastTimeoutSec` (0.5 s) moves `Maintain`/`Acquire` →
   `Coast`; while coasting (or between measurements) the track predicts
   kinematically at the simulation rate (`pos += vel·dt`; `vel += accel·dt` when
   `accelAvailable`).
5. No measurement for `trackLossTimeoutSec` (2.0 s) moves `Coast → Lost` (the
   track leaves guidance consumption; the external command becomes the aim).
6. A new fix on `Coast`/`Lost` moves the track `→ Reacquire`, then `→ Maintain`
   after the confirmation count is satisfied again.

Track-quality model (deterministic, updated each step with no measurement while
active): `quality01 = exp(−age/1.0)` (tau 1 s); `positionStdM = 5 + 25·age`;
`velocityStdMs = 25 + 50·age`. A new measurement resets them to the single-fix
bases (quality 1.0, position 5 m, velocity 25 m/s) and resets `ageSec`/
`dropoutCount`.

Per-entity config, exposed as optional camelCase `GuidanceAutopilotConfig` keys
with legacy-compatible defaults (omitted keys load the defaults and legacy
manifests keep the external-command aim): `trackConfirmations` (3, int),
`trackCoastTimeoutSec` (0.5), `trackLossTimeoutSec` (2.0). Guidance consumes the
track as described in §7.4 (measurement-anchored track aim with legacy external-
command fallback; APN feed-forward from the track when available).

### 7.6 Trajectory-aware midcourse guidance (W40)

`GuidanceMode::Trajectory` is an explicit, opt-in midcourse capability that
manages the prediction and feasibility of an intercept over the aim state
(command or the W39 track) instead of only steering toward the raw aim. It
consumes the SAME inputs as midcourse PN (navigation estimate + command/track,
never physics truth), nests inside the existing phase manager
(`Midcourse`/`LostTrack`), and does not change terminal behavior: a seeker lock
still moves `Midcourse → Acquisition → Terminal` and overrides trajectory
management exactly as it overrides `ProportionalNavigation` (§7.4). When the
mode is not selected, guidance is byte-identical to the legacy path.

Prediction model (deterministic, in `models/guidance/GuidanceModels.hpp`):
`predictIntercept` solves the intercept time-to-go `t*` over relative kinematics
`‖r + v·t + 0.5·arel·t²‖ = s_m(t)` (`r = aimPos − navPos`, `v = aimVel − navVel`,
`arel = at − ai`, `s_m(t) = |Vi|·t + 0.5·aiAxial·t²`). When own-ship acceleration is
available from `NavigationBlock` (`estAx/estAy/estAz`), `predictIntercept` refines
`t*` via Newton-Raphson accounting for axial boost acceleration or drag deceleration,
and evaluates closing velocity at intercept time (`vClose = (Vt + At·t*) − (Vi + Ai·t*)`).
When interceptor acceleration is omitted or zero, it evaluates the exact closed-form
velocity quadratic `‖r + v·t‖² = |Vi|²·t²` with byte-identical legacy precision.
It publishes the predicted intercept point `PIP = T + Vt·t* + 0.5·At·t*²` and the
required acceleration (the `|PN|` demand aimed at the PIP). It rejects non-finite input,
a non-positive navigation constant, own est speed below `trajectoryMinSpeedMps`
(`VelocityLow`), and geometry with no positive-time intercept (`NoIntercept`).

Aim-source precedence matches §7.4/§7.5 exactly: a measurement-anchored track
(state active AND `updateCount > 0`) wins; a `Coast` track keeps streaming
kinematic predictions; a `Lost` track (or a bare command seed with no
measurements) falls back to the external command aim. Per-entity outputs on
`GuidanceBlock`: `predictedInterceptX/Y/Z`, `predictedTgoSec`,
`trajectoryRequiredAccel`, `trajectoryAimSource` (`None`/`Command`/`Track`),
`trajectoryFeasible`, and `trajectoryReason`
(`None`/`Ok`/`VelocityLow`/`NoIntercept`/`AccelLimited`/`NonFinite`); `law`
reports `GuidanceLaw::Trajectory`.

Feasibility is the W40 energy/accel limit: a predicted intercept is feasible
when a positive-time intercept exists AND its required acceleration fits the
per-entity `maxAccel` budget
(`requiredAccel ≤ trajectoryFeasibilityAccelFactor × maxAccel`, factor 0.95;
`maxAccel ≤ 0` = budget-free, geometry-only). A feasible prediction commands
PN toward the PIP. An infeasible or unpredicable geometry commands bounded PN
toward the raw aim (a finite best-effort demand) while `trajectoryFeasible`
stays false with the reason published — never a non-finite vector and never a
silent fallback. Actuator (fin) saturation remains diagnosed downstream by
`ControlBlock` `pitchSaturated`/`yawSaturated`/`rollSaturated`. Constraint/
limits: predictor uses constant interceptor speed and optionally constant target
acceleration (no drag/thrust model); trajectory **optimization** and specific-
energy corridor/energy **management** remain planned (§9).

Config keys (optional, camelCase, legacy-compatible defaults):
`trajectoryMinSpeedMps` (30.0), `trajectoryFeasibilityAccelFactor` (0.95).
The mode is selected per entity through `SimulationCommand::mode` /
scenario `initial_guidance_mode` `"trajectory"`.

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
separation; warhead fusing (impact/proximity/timed); traceable mode-aware
guidance (phase/law state machine, acquisition blend, lock-loss retention,
APN feed-forward availability, per-entity diagnostics) and seeker-intercept
regression (`seeker_intercept_test`); persistent single target-track manager
(W39, §7.5: per-entity track fusing command seeds + seeker LOS fixes,
Acquire/Maintain/Coast/Lost/Reacquire lifecycle, multi-rate prediction,
quality/covariance model, track-based midcourse aim with legacy fallback);
trajectory-aware midcourse guidance (W40, §7.6: explicit
`GuidanceMode::Trajectory`, constant-speed intercept predictor with predicted
intercept point / tgo / required acceleration, `maxAccel`-budget feasibility
gate with `trajectoryReason` diagnostics, track/command aim-source precedence,
dropout/reacquisition response — midcourse only, seeker-lock override retained);
designer manifests
(`data/profiles`) and scenarios (`data/scenarios`) consumed end-to-end via
`designer_pipeline_test`; profile-id database layer (aero/motor/seeker/sensor/
guidance/warhead/RCS); multi-threaded CPUBackend parallel execution (`WorkerPool`);
data-driven static cd(M,α)/cl(M,α)/cm(M,α)/cy(M,β)/cn(M,β)/cl(M,β)
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
fusion; trajectory optimization and energy management (W40 implemented the
trajectory-core predictor + feasibility gate only); pursuit; LQR/MPC; richer
telemetry; CUDA; production GPU backend. Optional ECS/editor mapping,
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
