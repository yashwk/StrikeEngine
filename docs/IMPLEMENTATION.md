# StrikeEngine Implementation Record

**Status:** authoritative implementation record
**Companion specification:** [`SPEC.md`](SPEC.md)
**Verified:** 2026-08-26
**Runtime checkpoint:** `9ee7ef9`

This document maps the normative behavior in [`SPEC.md`](SPEC.md) to source
files, build targets, execution order, validation, packaging, and remaining
work. [`FIDELITY_AUDIT.md`](FIDELITY_AUDIT.md) supplies measured evidence;
it is not an alternative authority.

## 1. Verified state

- Project version: `0.1.0`; language: C++23; minimum CMake: 3.23.
- Default build: static `strikeengine` library with CPU backend.
- Optional companion: `strikeengine_vulkan`, enabled with
  `STRIKEENGINE_WITH_VULKAN=ON`.
- Release validation: **32/32 CTest tests pass**.
- Default local frame and constant-gravity behavior remain backward-compatible.
- The requested `.idea` project metadata change is included in this next
  documentation checkpoint; it is not runtime behavior.

## 2. Repository map

| Area | Responsibility |
| --- | --- |
| `include/strikeengine/kernel` | Public state blocks, kernel, systems, backends, integrators, subsystem config |
| `include/strikeengine/models` | Stateless atmosphere, aero, propulsion, earth, guidance, signatures |
| `include/strikeengine/simulation` | Single run, sweep, Monte Carlo, optimizer, batch wrappers |
| `src/strikeengine` | Runtime, config serialization, and signature database implementations |
| `tests/validation` | End-to-end and subsystem regression programs |
| `data/` | Example profiles, scenarios, tables, and schemas |
| `docs/` | Authoritative contracts and measured evidence records |

The public include prefix is `<strikeengine/...>`. Public headers do not expose
GLM or nlohmann/json as required consumer dependencies.

## 3. Build and package workflow

```sh
cmake -S . -B build-linux -DCMAKE_BUILD_TYPE=Release
cmake --build build-linux --config Release -j2
ctest --test-dir build-linux -C Release --output-on-failure
```

GLM `1.0.2` and nlohmann/json `v3.12.0` are resolved with FetchContent. An
offline environment must provide a configured build or those dependencies.

The optional Vulkan build is:

```sh
cmake -S . -B build-vulkan -DCMAKE_BUILD_TYPE=Release \
  -DSTRIKEENGINE_WITH_VULKAN=ON
cmake --build build-vulkan --config Release
```

Vulkan requires a Vulkan SDK and `glslc`. Requesting `BackendType::Vulkan`
without linking `strikeengine_vulkan` throws a clear runtime error. Vulkan is
not considered CPU-fidelity equivalent until separately validated.

The install exports `StrikeEngine::strikeengine` and, when enabled,
`StrikeEngine::strikeengine_vulkan`, plus all public headers and a CMake package
config.

## 4. Runtime architecture

`PhysicsBlock` is the truth SoA. It stores per-entity translation, velocity,
acceleration cache, quaternion, body rates, inertia, mass, aero coefficients,
propulsion ID, ignition time, achieved fins, and active state. The companion
blocks are:

| Block | State |
| --- | --- |
| `ControlBlock` | commanded thrust and fin channels |
| `GuidanceBlock` | mode, target state, demand limit, acceleration command |
| `NavigationBlock` | estimate, biases, alignment, full row-major 15×15 covariance |
| `SensorBlock` | IMU/GPS measurements and noise settings |
| `SeekerBlock` | RF/IR configuration, lock, measurements, latency history |
| `EntityStatusBlock` | type, allegiance, health, profile IDs, alive state |

Entity slots are reused through the kernel free list. All blocks share entity
indices; inactive entities are skipped.

`SimulationKernel::step(dt)` has this fixed order:

1. Save previous positions and advance kernel time.
2. Apply queued commands.
3. Advance truth physics through the selected backend/integrator.
4. Run the staging pass (`processStaging`, multi-stage separation).
5. Generate IMU/GPS measurements.
6. Propagate and fuse navigation.
7. Update seekers, lock state, filtered LOS rates, and delayed outputs.
8. Compute guidance demands.
9. Convert demands to actuator commands.
10. Evaluate terrain/ground events.
11. Run the warhead pass (`processWarheads`, after impacts are known).
12. Dispatch the queued event queue.

Truth is advanced before sensing; guidance and autopilot output affects the next
physics step. Staging runs immediately after truth so a burnout can drop mass
and rescale inertia before sensing. Warheads run after impact evaluation so an
impact fuse observes the deactivation state. This ordering is part of the
integration contract.

`SimulationKernel` accepts an `IntegratorType` (default RK4) which is applied
CPU-side only. The `PhysicsBlock` truth SoA also carries the per-entity
`stageIndex`/`stageCount` staging state.

## 5. Module ownership

### Core and configuration

- `SimulationKernel.hpp/.cpp`: lifecycle, entity management, command queue,
  orchestration, environment, public block access, deterministic
  failure/damage injection (`failEntity`, `applyDamage`), and the
  `createVehicle(config)` wiring of subsystem config into the per-entity SoA
  blocks, including the profile-id resolution step (a non-empty profile id
  replaces the inline sub-config; a failed load throws `std::runtime_error`
  naming the file), plus the `processStaging` and `processWarheads` passes.
- `EnvironmentConfig.hpp`: terrain/wind callbacks and earth options including
  `useEcefTruth`, gravity selection, Coriolis, centrifugal, and transport.
- `ScenarioConfig.hpp`: scenario metadata, environment, entity/vehicle setup,
  target state, kernel loading, `save`/`load`, and the per-entity `designRef`
  override.
- `VehicleConfig.hpp`: the flattened per-vehicle subsystem view (`type`,
  `initialMass`, `massDry`, `Ixx/Iyy/Izz`, `aero`, `propulsion`, `seeker`,
  `sensor`, `guidanceAutopilot`, `warhead`, the subsystem profile IDs
  `aeroProfileId`/`motorProfileId`/`seekerProfileId`/`sensorProfileId`,
  signature profile IDs, EIRP).
- `config/AeroConfig.hpp`, `config/PropulsionConfig.hpp` (with `StageConfig`),
  `config/SensorConfig.hpp`, `config/GuidanceAutopilotConfig.hpp`,
  `config/WarheadConfig.hpp`, and `config/SeekerConfig.hpp`: per-subsystem
  configuration structs.
- `config/ConfigSerialization.hpp/.cpp`: snake_case JSON (de)serialization of
  all config structs, enums as snake_case strings, `ScenarioConfig::save/load`,
  and `serializeDesign`/`loadDesignPhysics`. `AeroConfig` and `SensorConfig`
  serialize via manual snake_case `to_json`/`from_json` (not
  `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE`) so inline configs and profile files
  share one schema. Uses nlohmann/json privately behind a public string API;
  nlohmann is never exposed in public headers.
- `profiles/AeroProfileDatabase.hpp/.cpp`, `profiles/MotorProfileDatabase.hpp/
  .cpp`, `profiles/SeekerProfileDatabase.hpp/.cpp`, and
  `profiles/SensorProfileDatabase.hpp/.cpp`: single-profile loaders mirroring
  the RCSDatabase pattern. Each `loadProfile(path)` returns `false` on ANY load
  failure (unopenable file, malformed JSON, missing required key, wrong-typed
  field); the getter (`aero()`, `propulsion()`, `seeker()`, `sensor()`)
  returns the parsed config. `SimulationKernel::createVehicle` resolves a
  non-empty `aeroProfileId`/`motorProfileId`/`seekerProfileId`/`sensorProfileId`
  by loading the referenced file; the parsed config REPLACES the inline
  sub-config, and a failed load throws `std::runtime_error` naming the file.

### Truth and earth models

- `ISA1976.hpp`: layered atmosphere through 86 km.
- `AeroModel.hpp`: drag, bounded lift, fin side force, stability, damping, and
  bounded moments.
- `PropulsionModel.hpp` / `ThrustCurve.hpp`: thrust interpolation, Isp, mass flow,
  and dry-mass limiting.
- `CPUBackend.cpp`: stage-re-evaluated force model, body Euler dynamics,
  quaternion propagation, and servo dynamics. Reads the physical truth mirror
  of the motor/actuator failure flags (`PhysicsBlock::motorFailed` /
  `actuatorFailed`). Each non-empty-thrust-curve propulsion stage is registered
  in the pool by `SimulationKernel::createVehicle`. The fuel-depletion guard
  caps mass flow to the propellant remaining in the depletion window and scales
  thrust with the capped flow, so `T = ṁ·Isp·g0` holds at every instant and
  thrust self-terminates at fuel exhaustion (no free-thrust tail of full thrust
  on the final grams of fuel).
- `EarthModel.hpp`: WGS84 conversion, normal gravity, Coriolis, curvature, and
  transport APIs.
- `EarthFrames.hpp`: ECEF/ENU/NED transforms, local gravity, and centrifugal
  acceleration.
- `EarthFixedPropagator.hpp`: standalone rotating-Earth ECEF RK4 propagator.
- `EventSystem.cpp`: local terrain views, geodetic altitude, ellipsoid
  impact clamping, and the failure/damage event vocabulary (`MotorFailure`,
  `ActuatorFailure`, `SensorFailure`, `StructuralFailure`,
  `CommunicationFailure`).

### GNC and studies

- `SensorSystem.cpp`: noisy frame-aware IMU/GPS; per-entity GPS scheduling
  (`gpsEnabled`, `gpsUpdateRateHz`); IMU disablement freezes the held accel/gyro
  sample and stops the streaming-bias drift while GPS continues; sensor-failure
  flag stops measurement updates.
- `NavigationSystem.cpp`: alignment, strapdown INS, and coupled 15-state EKF.
- `SeekerSystem.cpp`: RF/IR signatures, geometry, lock, rates, and latency.
- `GuidanceSystem.cpp`: PN, waypoint guidance, and seeker APN handoff; reads
  the per-entity `navigationConstant`/`waypointGain` from `GuidanceBlock`;
  communication-failure flag zeroes commanded acceleration (ballistic).
- `AutopilotSystem.cpp`: world-to-body demand conversion and bounded fin
  control; reads the per-entity gains (`kAccelP/kRateP/kAlphaP/kRollP/kRollD`)
  and `maxDeflectionRad` from `ControlBlock`.
- `EntityStatusBlock.hpp`: per-entity health, alive state, and the deterministic
  failure flags (motor, actuator, sensor, communications); structural failure
  is `isAlive=false` + `health=0`.
- `SingleRun`, `ParamSweep`, `MonteCarlo`, `Optimizer`, `BatchRunner`: study
  wrappers; frame normalization is implemented in `Reporting.hpp` and
  versioned CSV/binary output in `StudyOutput.hpp/.cpp`, including the binary
  reader `StudyOutputReader::read`. `BatchRunner`
  aggregate metrics `maxAltitudeM`/`maxSpeedMps` track the trajectory maximum,
  not just the final state.

### Designer → engine pipeline

- `data/profiles/sa_missile_mk1.json` and `data/profiles/target_drone.json`
  are engine design manifests (`{ name, description, physics: <VehicleConfig
  snake_case> }`), consumed via `designRef`. The missile manifest wires the
  four subsystem profile ids (`aeroProfileId`/`motorProfileId`/
  `seekerProfileId`/`sensorProfileId`) to `data/aero/sa_missile_mk1_aero.json`,
  `data/motors/sa_missile_mk1_motor.json` (two-stage booster + sustainer),
  `data/seekers/aesa_tracker_v1.json` (RF, FOV 6° / gimbal 65°), and
  `data/sensors/sa_missile_mk1_imu.json`. The profile ids are authoritative
  over the inline placeholder blocks, which must remain present because
  `VehicleConfig::from_json` requires all sub-object keys. The drone manifest
  has no propulsion/seeker of consequence and wires `rcsProfileId` to the new
  flat RCS table.
- `data/rcs/target_drone_rcs.json`: flat 1.5 m² (1.7609 dBsm) RCS table in
  RCSDatabase format (two breakpoints each axis, uniform `rcs_table_dbsm`).
- `data/scenarios/intercept_test_01.json`: rewritten in the engine ScenarioConfig
  schema (`environment`, `primary_entity_index`, two entities with
  `design_ref` + `init_state` + `initial_guidance_mode` /
  `initial_target_*` / `initial_max_accel`). The designer's original ECEF-style
  Earth-radius coordinates sit below the WGS84 ellipsoid (unflyable); the
  scenario runs in the engine's local ENU frame preserving the exact relative
  geometry (20 km downrange, 10 km up). The scenario is a deterministically
  tuned data set (sensor seed `0xDEADBEEF`); the missile uses proximity fusing
  (40 m trigger / 40 m lethal radius) and the drone a low-drag airframe
  (S=1.0, cd=0.02, clAlpha=0.5).
- `config/SeekerTypeStrings.hpp`: shared inline `seekerTypeToString`/
  `seekerTypeFromString` snake_case maps for `SeekerType`, deduplicated so
  `ConfigSerialization.cpp` and `SeekerProfileDatabase.cpp` use one ODR-safe
  definition. No `to_json`/`from_json` for `SeekerType` is declared anywhere;
  callers map the string explicitly.
- Cleanup: the stale 0-byte `data/profiles/aesa_tracker_v1.json` was deleted;
  the seeker part now lives at `data/seekers/aesa_tracker_v1.json`.

## 6. ECEF kernel integration

Set `EnvironmentConfig::earth.useEcefTruth = true` before creating entities.
Initial positions and velocities are then absolute ECEF values, normally made
with `Models::geodeticToEcef`.

| Consumer | ECEF behavior |
| --- | --- |
| CPU truth | Geodetic atmosphere and ECEF earth acceleration |
| Sensors | ECEF/world GPS and ECEF gravity-compensated IMU |
| Navigation | ECEF initial alignment, INS, and EKF state |
| Seekers/guidance | Relative vectors in the shared ECEF frame |
| Events | Local ENU terrain anchor and WGS84 ellipsoid impact clamp |

The local mode remains the default. Study wrappers use the shared reporting
layer for ECEF altitude/coordinate summaries and versioned CSV/binary
recording; a binary reader is implemented, and richer production telemetry
remains future work.

## 7. Validation inventory

| Test | Coverage |
| --- | --- |
| `singlerun`, `sweep`, `montecarlo`, `optimizer` | study wrappers |
| `rigidbody`, `vehicleconfig`, `intercept` | truth dynamics and control |
| `integrator` | RK4/RK45 and impact interpolation |
| `seeker`, `navigation`, `environment` | GNC and environment MVPs |
| `earth`, `earth_frames`, `earth_transport` | WGS84 conversion and local frames |
| `spherical_gravity`, `earth_fixed` | gravity and standalone ECEF propagation |
| `ecef_kernel` | kernel ECEF physics/events/sensors/navigation |
| `guidance`, `scenario` | guidance and scenario contracts |
| `kernel_lifecycle` | freed-slot reuse reset and non-positive-timestep rejection |
| `failure` | deterministic failure/damage semantics: motor thrust/mass-flow stop, actuator fin freeze, sensor measurement stop, communication guidance zero, structural deactivation, per-type events, and argument validation |
| `lever_arm` | per-entity IMU lever-arm specific-force correction (α×l + ω×(ω×l)) |
| `reporting` | versioned local/ECEF wrapper output, field selection, and binary recording/reading |
| `config_wiring` | per-entity sensor enablement (IMU freeze, GPS scheduling/disable) and guidance/autopilot gain propagation with the configurable `maxDeflectionRad` clamp |
| `staging_warhead` | two-stage separation (stage drop, inertia rescale, `StageSeparation` event) and impact/proximity/timed warhead fusing (`Detonation` event, flat lethal-radius kill) |
| `serialization` | JSON round-trip of all config structs, scenario/design load-save, malformed-input errors, the `designRef` override (a design file's `physics` supersedes inline `vehicleConfig`), the round-trip of the four profile-id keys, and a legacy-compat case (a pre-feature `VehicleConfig` without the profile-id keys still deserializes with empty ids) |
| `profile_database` | per-subsystem profile DB parsing (aero/motor/seek/sensor) with defaults for omitted keys; `loadProfile` returning `false` on any failure (missing file, malformed JSON, wrong-typed field, missing required seeker `type`/motor `stages`); `createVehicle` profile-wins resolution into the SoA blocks; empty-id regression (inline config untouched); missing/schema-broken profile → `std::runtime_error` naming the file; shipped `data/aero|motors|seekers|sensors` examples parse |
| `designer_pipeline` | end-to-end designer→engine chain: loads `data/scenarios/intercept_test_01.json`, resolves both `design_ref` manifests into `VehicleConfig` (opposing allegiances, ProNav mode), verifies the four missile subsystem profile ids and the drone `rcsProfileId` reach the SoA blocks (motor stage count 2, seeker FOV 6°/gimbal 65° vs the inline 60° placeholder, drone RCS id in the status block), runs the intercept, and asserts a real guided intercept (min miss 16.77 m < 50 m at t≈19.7 s) plus a proximity-warhead kill via the event system |
| `rocket_mvp` | first-principles WGS84 single-stage rocket-launch verification (local ENU, Somigliana normal gravity at 28.5°N, ISA-1976, pressure-interpolated Isp, fuel-limited burnout, RK4 dt=0.01 s, ballistic): T0 thrust (60000 N), T0 mass flow (27.81 kg/s at sea-level Isp), initial acceleration (T/m − g_lat ≈ 110.2 m/s²), burnout time within the pressure-interpolated-Isp band [5.39, 6.13] s, mass at cutoff ≈ 350 kg, cutoff velocity vs the ideal rocket equation Δv = Isp·g0·ln(m0/mdry), apogee band and no-drag bound, max dynamic pressure (~242 kPa), ISA-1976 sea-level density 1.225 kg/m³, WGS84 geodetic↔ECEF round trip, Somigliana gravity monotone and altitude-accurate (<0.1%), lateral drift ~0, and a 70°-elevation arcing case; discriminates the free-thrust-tail propulsion bug |

Every runtime increment MUST add or update a deterministic regression, run
`git diff --check`, build Release, and run complete CTest.

## 8. Milestone record

| Milestone | Result | Checkpoint |
| --- | --- | --- |
| W1–W7 | vehicle, rigid body, control, integration, events, seeker, navigation MVPs | cumulative through `2611f6f` |
| W8 | scenario/guidance contract and isolated batch execution | `cac69c4` |
| W9 | WGS84 normal gravity and local Coriolis | `0d4ec1c` |
| W10 | local ECEF/ENU/NED frames and centrifugal term | `c548e88` |
| W11 | local moving-origin transport rate | `f781b3d` |
| W12 | spherical point-mass gravity | `a47d329` |
| W13 | standalone rotating-Earth ECEF propagator | `5a8d0f9` |
| W14 | opt-in kernel ECEF truth across physics/events/sensors/navigation | `40af724` |
| W15 | frame-aware study-wrapper reporting and versioned CSV output | `d1bd26f` |
| W16 | configurable CSV/binary study output and structured status metadata | `1ff6d1e` |
| W21 | flattened per-vehicle subsystem `VehicleConfig`, snake_case JSON/design/scenario serialization | current |
| W22 | per-entity sensor enablement and guidance/autopilot gain wiring (`config_wiring_test`) | current |
| W23 | multi-stage propulsion staging and warhead fusing (`staging_warhead_test`) | current |
| W24 | profile-id database layer (`profile_database_test`): aero/motor/seek/sensor single-profile loaders, `createVehicle` profile-wins resolution, shared snake_case profile schema | current |
| W25 | designer→engine pipeline (`designer_pipeline_test`): engine-consumable `data/profiles` design manifests, flat `data/rcs/target_drone_rcs.json` table, rewritten `data/scenarios/intercept_test_01.json` in the ScenarioConfig schema, `SeekerTypeStrings.hpp` dedup, end-to-end guided intercept + proximity-warhead kill | current |
| W26 | first-principles rocket-launch verification (`rocket_mvp_test`) + propulsion-law fix: WGS84 single-stage launch cross-checked by hand (T0 thrust/mass flow, initial accel, ideal rocket-equation cutoff velocity, apogee/max-Q), and the fuel-depletion guard now scales thrust with the capped mass flow so `T = ṁ·Isp·g0` holds at fuel exhaustion (no free-thrust tail) | current |

## 9. Project boundaries and deferred feature inventory

### 9.1 External architecture decisions

StrikeEngine is library-only. The executable shell, Designer UI, visualization,
and optional ECS/editor mapping belong to StrikeSim or another consuming
application. StrikeSim is expected to own the Design, Simulate, and CEM
surfaces and consume the installable, versioned StrikeEngine package.

StrikeCEM remains a separate project with its own authority documents, CLI, and
`strikecem_lib` shared library. StrikeSim is the embedding boundary for CEM;
StrikeEngine does not absorb CEM source. Designer identity, revision, geometry
provenance, and export validation must remain explicit across the
StrikeDesigner → StrikeCEM → StrikeEngine handoff.

The intended delivery order is: stabilize the public C++ API and package,
then build the StrikeSim shell/Designer integration, then embed StrikeCEM.
StrikeCEM offline work may proceed independently. A future protobuf/gRPC or
REST API may wrap the stable C++ API, but it is not part of the current library
contract.

The pre-restructure implementation is preserved in the git tag
`legacy/pre-restructure-v1`; it is rollback history, not an active source tree.

### 9.2 Deferred feature inventory

The following ideas came from the original architecture inventory and remain
tracked here so they are not mistaken for missing documentation or current
runtime guarantees:

- **Physics and environment:** validated aerodynamic coefficient tables and
  `AeroForces` data, advanced atmosphere and
  weather, DEM/DTED loading and query services, global terrain tile streaming,
  datum/geoid handling, and polar/dateline policy.
- **Navigation and sensing:** sensor-fusion services, magnetometer, barometer,
  radar altimeter, and richer measurement timing/calibration. Sensor lever
  arms, coning/sculling, and earth-rate gyro compensation are implemented
  (MVP). Lever arms: per-entity `VehicleConfig::imuLeverArmX/Y/Z` drives the
  IMU specific-force correction `alpha x l + omega x (omega x l)`. Coning and
  sculling: the strapdown uses a rotation-vector attitude update (exact
  delta-quaternion, validated 450x tighter than the first-order step on a
  coning environment) plus the single-interval sculling compensation
  `+0.5 (omega x f) dt^2` (the two-interval Bortz cross-terms were evaluated
  numerically and do not improve the point-sampled per-step scheme). Earth-rate
  gyro: `EnvironmentConfig::earth.includeEarthRateGyro` (ECEF truth mode only)
  models the gyro as measuring the inertial body rate and compensates it in the
  INS. Earth-rate gyro remains off for local flat-earth truth because the gyro
  there already resolves the non-rotating-frame body rate; correct local
  earth-rate compensation requires the rotating-frame (ECEF) navigation path.
- **Guidance, control, and seekers:** trajectory, waypoint, and energy
  managers; pursuit guidance; LQR/MPC; seeker management and blended handoff;
  imaging IR; dynamic SARH illuminator tracking; multi-target tracking; and
  band-resolved IR extinction. Semi-active radar (SARH, bistatic with a static
  illuminator), passive RF (homes on target EIRP), Beer-Lambert IR
  transmittance, and chaff/flare decoys are implemented (MVP); the seeker
  parameters are exposed through the public `SeekerConfig` surface
  (`VehicleConfig::seeker`).
- **Execution and platforms:** parallel CPU execution, validated Vulkan/CPU
  parity, GPU ECEF truth, and CUDA if a concrete requirement is established.
- **Tools and integration:** richer telemetry schemas,
  versioned scenario/config schema evolution, plotting/analysis/
  scenario-generation tools, shared logging/units/profiling utilities,
  visualization/debug drawing, and the future API server wrapper.

## 10. Known limitations and prioritized backlog

1. **Study output consumers:** the binary reader is implemented
   (`StudyOutputReader::read`); richer telemetry schemas, streaming record
   sinks, and configurable output selection beyond the current wrapper records
   remain.
2. **Global terrain:** runtime terrain is still a callback with no DEM/DTED
   ingestion. Add DEM/DTED tiles, interpolation, streaming, datum/geoid
   policy, dateline/polar handling, and frame-aware collision queries.
3. **Failures:** motor, actuator, sensor, structural, and communications
   failure models and event semantics are implemented (MVP) as deterministic
   per-entity flags with events (`failure_test`). Probabilistic degradation,
   partial health effects beyond deactivation, and repair remain open.
4. **GNC fidelity:** sensor lever arms, coning/sculling, and earth-rate gyro
   compensation are implemented (MVP). Lever arms: per-entity
   `VehicleConfig::imuLeverArm*` rigid-body specific-force correction
   (`lever_arm_test`). Coning/sculling: rotation-vector attitude update and
   single-interval sculling compensation (`coning_sculling_test`). Earth-rate
   gyro: `includeEarthRateGyro` in ECEF truth mode only (`earth_rate_gyro_test`).
   Earth-rate gyro stays off for local flat-earth truth because the gyro there
   already resolves the non-rotating-frame body rate; correct local earth-rate
   compensation requires the rotating-frame (ECEF) navigation path. Richer
   seekers are implemented (MVP): SARH (bistatic, static illuminator), passive
   RF (EIRP), Beer-Lambert IR transmittance, chaff/flare decoys, and
   strongest-signal acquisition via the public `SeekerConfig` surface
   (`seeker_rich_test`). Remaining: imaging IR, multi-target tracking, dynamic
   illuminator tracking, and band-resolved extinction.
5. **Guidance/aero:** add trajectory management, pursuit, LQR/MPC, blended
   handoff, and validated coefficient tables.
6. **GPU parity:** validate Vulkan against CPU truth, add GPU ECEF support, and
   implement CUDA if required.
7. **Applications:** implement the explicit versioned StrikeSim/
   StrikeDesigner/StrikeCEM handoffs and provenance; do not hide them in this
   library.

## 11. Change and release gate

Public behavior changes MUST update `SPEC.md`, this record, related config/data
documentation, and regression tests. A historical note or backlog line is not
implementation evidence; accepted inputs, outputs, defaults, failure behavior,
and executable validation are required.
