# StrikeEngine Implementation Record

**Status:** authoritative implementation record
**Companion specification:** [`SPEC.md`](SPEC.md)
**Verified:** 2026-08-29
**Runtime checkpoint:** `working tree after 74fb567`

[`SPEC.md`](SPEC.md) is normative; [`FIDELITY_AUDIT.md`](FIDELITY_AUDIT.md) is
measured evidence. This record maps behavior to files, build, execution order,
validation, and remaining work.

## 1. Verified state

- Version `0.1.0`; C++23; CMake ≥ 3.23.
- Default build: static `strikeengine` library, CPU backend.
- Optional `strikeengine_vulkan` via `STRIKEENGINE_WITH_VULKAN=ON`.
- Release validation: **36/36 CTest tests pass**.
- Default local frame and constant-gravity behavior remain backward-compatible.
- `.idea` project metadata change is in this documentation checkpoint (not runtime
  behavior).

## 2. Repository map

| Area | Responsibility |
| --- | --- |
| `include/strikeengine/kernel` | State blocks, kernel, systems, backends, integrators, config |
| `include/strikeengine/models` | Stateless atmosphere, aero, propulsion, earth, guidance, signatures |
| `include/strikeengine/simulation` | Single run, sweep, Monte Carlo, optimizer, batch wrappers |
| `src/strikeengine` | Runtime, config serialization, signature DB |
| `tests/validation` | Regression programs |
| `data/` | Profiles, scenarios, tables, schemas |
| `docs/` | Contracts and evidence records |

Public include prefix `<strikeengine/...>`; no GLM/nlohmann in public headers.

## 3. Build and package workflow

```sh
cmake -S . -B build-linux -DCMAKE_BUILD_TYPE=Release
cmake --build build-linux --config Release -j2
ctest --test-dir build-linux -C Release --output-on-failure
```

GLM `1.0.2` and nlohmann/json `v3.12.0` via FetchContent; offline builds must
provide them.

```sh
cmake -S . -B build-vulkan -DCMAKE_BUILD_TYPE=Release -DSTRIKEENGINE_WITH_VULKAN=ON
cmake --build build-vulkan --config Release
```

Vulkan needs a SDK + `glslc`; requesting `BackendType::Vulkan` without linking
`strikeengine_vulkan` throws. Vulkan is not CPU-fidelity-equivalent until
validated.

Install exports `StrikeEngine::strikeengine` (+ `strikeengine_vulkan` when enabled),
all public headers, and a CMake package config.

## 4. Runtime architecture

`PhysicsBlock` is the truth SoA: per-entity translation, velocity, acceleration
cache, quaternion, body rates, inertia, mass, aero coefficients, propulsion ID,
ignition time, achieved fins, active state.

| Block | State |
| --- | --- |
| `ControlBlock` | commanded thrust and fin channels |
| `GuidanceBlock` | mode, target state, demand limit, acceleration command |
| `NavigationBlock` | estimate, biases, alignment, row-major 15×15 covariance |
| `SensorBlock` | IMU/GPS measurements and noise settings |
| `SeekerBlock` | RF/IR config, lock, measurements, latency history |
| `EntityStatusBlock` | type, allegiance, health, profile IDs, alive state |

Entity slots reused via kernel free list; blocks share entity indices; inactive
entities skipped.

`SimulationKernel::step(dt)` fixed order:

1. Save previous positions; advance time. 2. Apply queued commands.
3. Advance truth physics. 4. Staging pass (`processStaging`).
5. Generate IMU/GPS. 6. Propagate + fuse navigation. 7. Update seekers/lock/LOS/
latency. 8. Compute guidance. 9. Convert to actuator commands.
10. Evaluate terrain/ground events. 11. Warhead pass (`processWarheads`).
12. Dispatch event queue.

Truth before sensing; guidance/autopilot affect the next physics step. Staging runs
right after truth so a burnout drops mass/rescales inertia before sensing; warheads
run after impact so an impact fuse observes deactivation — part of the integration
contract. `IntegratorType` (default RK4) applies CPU-side only; `PhysicsBlock`
carries per-entity `stageIndex`/`stageCount`.

## 5. Module ownership

### Core and configuration

- `SimulationKernel.hpp/.cpp`: lifecycle, entities, command queue, orchestration,
  environment, block access, `failEntity`/`applyDamage`, `createVehicle` (profile-
  id resolution: non-empty id replaces inline sub-config; failed load throws
  `std::runtime_error` naming the file), `processStaging` (leftover dump, dry-mass
  drop, post-dump inertia rescale, `dumpedMassKg`), `processWarheads` (impact/
  proximity/timed fusing, falloff band, RNG draw only when `0 < p < 1`).
- `EnvironmentConfig.hpp`: terrain/wind callbacks; earth options (`useEcefTruth`,
  gravity, Coriolis, centrifugal, transport).
- `ScenarioConfig.hpp`: scenario metadata, environment, entities, target, kernel
  loading, `save`/`load`, per-entity `designRef` override.
- `VehicleConfig.hpp`: flattened view (`type` = `EntityType`, `initialMass`,
  `massDry`, `Ixx/Iyy/Izz`, `aero`, `propulsion`, `seeker`, `sensor`,
  `guidanceAutopilot`, `warhead`, profile IDs, signature IDs, EIRP).
- `config/*.hpp`: per-subsystem structs; `WarheadConfig` carries optional
  `falloffRadiusM` (0.0 or `<= lethalRadiusM` = flat law).
- `config/ConfigSerialization.cpp`: snake_case JSON, enums as strings, `save`/`load`,
  `serializeDesign`/`loadDesignPhysics`. `AeroConfig`/`SensorConfig` use manual
  snake_case `to_json`/`from_json` so inline + profile files share one schema.
  `AeroConfig` has optional `aero_tables` (parsed `AeroTables` failing `isValid`
  throws). `WarheadConfig` emits `falloff_radius_m` always, parses 0.0 default,
  throws on `0 < falloff < lethal`. nlohmann private behind a string API.
- `profiles/{Aero,Motor,Seeker,Sensor}ProfileDatabase`: single-profile loaders;
  `loadProfile(path)` → `false` on ANY failure, getter returns parsed config; aero
  loader parses optional `aero_tables` (false if `!isValid`); `createVehicle`
  resolves non-empty ids, failed load throws.

### Truth and earth models

- `ISA1976.hpp`: layered atmosphere through 86 km.
- `AeroModel.hpp`/`CoefficientTable.hpp`: drag, bounded lift, fin side force,
  stability, damping, bounded moments. `AeroParams::tables` = optional static
  cd(M,α)/cl(M,α)/cm(M,α)/cy(M,β)/cn(M,β)/cl(M,β) grids (`AeroTables`), with
  bilinear `interpolateCoefficient`, clamping, finite/strict-grid validation,
  and optional moment/lateral dimensions. Valid supplied tables replace the
  corresponding static coefficient; fin control and rate damping remain active.
  With no optional table, the established scalar fallback is byte-identical.
- `FinsModel.hpp`: `FinShape` (`Trapezoidal`/`Elliptical`/`FreeForm`), `FinsGeometry`
  (precomputed geometry + Mach-dependent `clAlpha`/`rollForcingPerRad`/
  `rollDampingCoeff`), `buildFinsGeometry` (RocketPy port: Diederich + Prandtl–Glauert
  lift slope, fin-number/interference corrections, per-shape CP, roll factors).
  `AeroConfig::fins` (count 0 disables, ≥3 enables) wires into `BasicAeroModel::
  computeWrench`; when non-null the geometry-derived Mach-dependent fin terms
  REPLACE the abstract `clFin`/`CM_delta`/`Cl_delta`/`CN_beta` (body terms stay),
  and when null the legacy path is byte-identical. `fins` JSON parsed in
  `ConfigSerialization.cpp` and `AeroProfileDatabase.cpp` with fail-fast validation.
- `PropulsionModel.hpp`/`ThrustCurve.hpp`: thrust interpolation, Isp, mass flow,
  dry-mass limiting.
- `CPUBackend.cpp`: stage-re-evaluated forces, body Euler dynamics, quaternion
  propagation, servo dynamics; reads `motorFailed`/`actuatorFailed`. Fuel-depletion
  guard caps mass flow and scales thrust, so `T = ṁ·Isp·g0` holds at every instant
  (no free-thrust tail).
- `EarthModel.hpp`: WGS84, normal gravity, Coriolis, curvature, transport.
- `EarthFrames.hpp`: ECEF/ENU/NED transforms, local gravity, centrifugal.
- `EarthFixedPropagator.hpp`: standalone rotating-Earth ECEF RK4.
- `EventSystem.cpp`: local terrain views, geodetic altitude, ellipsoid impact
  clamp, failure event vocabulary (`MotorFailure`/`ActuatorFailure`/
  `SensorFailure`/`StructuralFailure`/`CommunicationFailure`); `dumpedMassKg` on
  `StageSeparation` (0.0 on exhaustion burnouts).

### GNC and studies

- `SensorSystem.cpp`: frame-aware IMU/GPS; per-entity GPS scheduling; IMU disable
  freezes sample + stops bias drift while GPS continues; sensor-failure stops
  updates. Exposes `nextUniform01()` (uniform [0,1) from shared kernel RNG) used by
  `processWarheads`.
- `NavigationSystem.cpp`: alignment, strapdown INS, coupled 15-state EKF.
- `SeekerSystem.cpp`: RF/IR signatures, geometry, lock, rates, latency.
- `GuidanceSystem.cpp`: PN, waypoint, seeker APN handoff; reads
  `navigationConstant`/`waypointGain`; comms-failure zeroes accel.
- `AutopilotSystem.cpp`: world→body demand conversion, bounded fins; reads gains
  + `maxDeflectionRad`.
- `EntityStatusBlock.hpp`: health, alive state, deterministic failure flags;
  structural failure = `isAlive=false` + `health=0`.
- Study wrappers (`SingleRun`, `ParamSweep`, `MonteCarlo`, `Optimizer`,
  `BatchRunner`); frame normalization in `Reporting.hpp`, versioned CSV/binary in
  `StudyOutput.hpp/.cpp` incl. `StudyOutputReader::read`; `BatchRunner`
  `maxAltitudeM`/`maxSpeedMps` track trajectory maxima.

### Designer → engine pipeline

- `data/profiles/sa_missile_mk1.json`, `target_drone.json`: engine design manifests
  (`{ name, description, physics: <VehicleConfig> }`), consumed via `design_ref`.
  Missile wires the four profile ids to `data/aero/sa_missile_mk1_aero.json`,
  `data/motors/sa_missile_mk1_motor.json` (two-stage),
  `data/seekers/aesa_tracker_v1.json` (RF, FOV 6°/gimbal 65°), `data/sensors/
  sa_missile_mk1_imu.json`; profile ids authoritative over inline placeholders
  (which must exist: `from_json` requires all sub-object keys). Drone wires
  `rcsProfileId` to the flat RCS table.
- `data/rcs/target_drone_rcs.json`: flat 1.5 m² (1.7609 dBsm) RCS table (two
  breakpoints per axis, uniform `rcs_table_dbsm`).
- `data/scenarios/intercept_test_01.json`: ScenarioConfig schema (`environment`,
  `primary_entity_index`, entities with `design_ref` + `init_state` +
  `initial_guidance_mode` + `initial_target_*` + `initial_max_accel`); local ENU,
  15 km/8 km (SPEC §5.5); seed `0xDEADBEEF`; missile proximity fusing (50/50/90 m),
  drone low-drag (S=1.0, cd=0.02, clAlpha=0.5).
- `config/SeekerTypeStrings.hpp`: shared snake_case seeker-type maps, one ODR-safe
  definition.
- Cleanup: stale 0-byte `data/profiles/aesa_tracker_v1.json` deleted; seeker part
  moved to `data/seekers/aesa_tracker_v1.json`.

## 6. ECEF kernel integration

Set `EnvironmentConfig::earth.useEcefTruth = true` before creating entities;
initial states are absolute ECEF, normally via `Models::geodeticToEcef`.

| Consumer | ECEF behavior |
| --- | --- |
| CPU truth | Geodetic atmosphere, ECEF earth acceleration |
| Sensors | ECEF/world GPS, ECEF gravity-compensated IMU |
| Navigation | ECEF initial alignment, INS, EKF |
| Seekers/guidance | Relative vectors in shared ECEF frame |
| Events | Local ENU terrain anchor, WGS84 ellipsoid impact clamp |

Local mode remains default. Study wrappers share reporting for ECEF summaries;
binary reader implemented; richer telemetry future.

## 7. Validation inventory

36 deterministic CTest programs:

| Test | Coverage |
| --- | --- |
| `singlerun`, `sweep`, `montecarlo`, `optimizer` | study wrappers |
| `rigidbody`, `vehicleconfig`, `intercept` | truth dynamics and control |
| `integrator` | RK4/RK45 and impact interpolation |
| `seeker`, `navigation`, `environment` | GNC and environment MVPs |
| `seeker_rich`, `earth_rate_gyro`, `coning_sculling` | richer seekers; earth-rate gyro; coning/sculling |
| `earth`, `earth_frames`, `earth_transport` | WGS84 conversion and local frames |
| `spherical_gravity`, `earth_fixed` | gravity and standalone ECEF propagation |
| `ecef_kernel` | kernel ECEF physics/events/sensors/navigation |
| `guidance`, `scenario` | guidance and scenario contracts |
| `kernel_lifecycle` | freed-slot reuse reset; non-positive-timestep rejection |
| `failure` | deterministic failure/damage semantics and events |
| `lever_arm` | per-entity IMU lever-arm correction (α×l + ω×(ω×l)) |
| `reporting` | versioned local/ECEF output, field selection, binary record/read |
| `config_wiring` | per-entity sensor enablement, guidance/autopilot gain wiring |
| `staging_warhead` | two-stage separation (dump, inertia rescale, event) + fusing |
| `warhead_falloff` | falloff law (1/linear/0), fixed-seed in-band outcome, flat-law RNG exclusion, round-trip + reject |
| `serialization` | config round-trip, scenario/design load-save, `designRef` override, four profile-id keys, legacy-compat |
| `profile_database` | aero/motor/seek/sensor DB, fail-fast `loadProfile`, profile-wins resolution |
| `designer_pipeline` | manifest→`designRef`→profile-id→intercept (45.4 m miss) + kill |
| `rocket_mvp` | WGS84 launch: T0 60000 N, flow 27.81 kg/s, init accel ~110 m/s², burnout Isp band [5.39, 6.13] s, cutoff vs Δv = Isp·g0·ln(m0/mdry), apogee, max-Q ~242 kPa |
| `coefficient_table` | `interpolateCoefficient` breakpoint/interior/clamp, `AeroTables::isValid` |
| `rocket_mvp_tables` | constant vs tables: apogee 24.79 > 17.19 km, burnout V 713.6 > 686.8 m/s, max-Q 260.4 > 242.2 kPa; fallback byte-identical |
| `fins` | geometric fins: three-shape geometry hand-checks (trapezoidal/elliptical/free-form), Mach lift-slope monotonicity, tail-fin restoring-moment sign, positive-cant roll forcing, JSON round-trip (incl. `shape_points`), validation throws (count<3, free-form <3 points), ballistic rocket_mvp with 4 tail fins (trapezoidal + elliptical; apogee 17.2 km, drift ~0 m) |

Every runtime increment MUST add/update a deterministic regression, run
`git diff --check`, build Release, run complete CTest.

## 8. Milestone record

| Milestone | Result | Checkpoint |
| --- | --- | --- |
| W1–W7 | vehicle/rigid-body/control/integration/events/seeker/navigation MVPs | cumulative `2611f6f` |
| W8 | scenario/guidance contract, isolated batch | `cac69c4` |
| W9 | WGS84 normal gravity + local Coriolis | `0d4ec1c` |
| W10 | local ECEF/ENU/NED frames + centrifugal | `c548e88` |
| W11 | local moving-origin transport rate | `f781b3d` |
| W12 | spherical point-mass gravity | `a47d329` |
| W13 | standalone rotating-Earth ECEF propagator | `5a8d0f9` |
| W14 | opt-in kernel ECEF truth | `40af724` |
| W15 | frame-aware study reporting, versioned CSV | `d1bd26f` |
| W16 | configurable CSV/binary output + status metadata | `1ff6d1e` |
| W21 | flattened `VehicleConfig`, snake_case serialization | current |
| W22 | per-entity sensor enablement + gain wiring (`config_wiring_test`) | current |
| W23 | multi-stage staging + warhead fusing (`staging_warhead_test`) | current |
| W24 | profile-id DB layer (`profile_database_test`) | current |
| W25 | designer→engine pipeline (`designer_pipeline_test`): manifests, flat RCS table, scenario, `SeekerTypeStrings` dedup, guided intercept + kill | current |
| W26 | rocket verification (`rocket_mvp_test`) + propulsion-law fix (`T = ṁ·Isp·g0`) | current |
| W27 | data-driven cd/cl aero tables (`coefficient_table_test`, `rocket_mvp_tables_test`); pipeline re-baselined (45.4 m miss after W28, 15 km/8 km) | current |
| W28 | leftover dump + warhead falloff (`staging_warhead_test`, `warhead_falloff_test`); SAM 50/50/90 m; 45.4 m miss | current |
| W29 | geometric fins (RocketPy trapezoidal/elliptical/free-form) (`fins_test`) | current |
| W30 | static moment/lateral aero tables (`coefficient_table_test`, `serialization_test`, `profile_database_test`) | current |

## 9. Project boundaries and deferred feature inventory

### 9.1 External architecture decisions

StrikeEngine is library-only; shell/Designer UI/visualization/ECS mapping belong
to StrikeSim or another consumer. StrikeSim owns Design/Simulate/CEM surfaces and
consumes the installable versioned package. StrikeCEM is separate (own authority
docs, CLI, `strikecem_lib`); StrikeEngine does not absorb CEM source. Identity,
revision, geometry provenance, and export validation stay explicit across the
StrikeDesigner → StrikeCEM (RCS) / StrikeCFD (aero) → StrikeEngine handoff. Order:
stabilize the public C++ API/package → StrikeSim shell/Designer → embed StrikeCEM;
StrikeCEM offline work may proceed independently. A future protobuf/gRPC or REST
API may wrap the stable C++ API (not current contract). Pre-restructure code is in
git tag `legacy/pre-restructure-v1` (rollback history, not active source).

### 9.2 Deferred feature inventory

Tracked so these are not mistaken for missing docs or current guarantees:

- **Physics/environment:** CFD-validated coefficient data, `AeroForces`,
  Reynolds-dependent/nonlinear stall and post-stall behavior, advanced
  atmosphere/weather, DEM/DTED loading + streaming, datum/geoid, polar/dateline.
  (static cd/cl/cm/cy/cn/cl tables implemented — W27/W30; geometric fins
  implemented — W29.)
- **Navigation/sensing:** sensor fusion, magnetometer, barometer, radar altimeter,
  richer timing/calibration. (Lever arms, coning/sculling, earth-rate gyro done —
  MVP; SPEC §7.1–7.2.)
- **Guidance/control/seekers:** trajectory/waypoint/energy managers, pursuit,
  LQR/MPC, seeker management + blended handoff, imaging IR, dynamic SARH
  illuminator tracking, multi-target tracking, band-resolved extinction. (SARH,
  PassiveRF, Beer-Lambert IR, chaff/flare done — MVP.)
- **Execution/platforms:** parallel CPU, validated Vulkan/CPU parity, GPU ECEF,
  CUDA (if required).
- **Tools/integration:** richer telemetry schemas, versioned schema evolution,
  plotting/analysis/scenario tools, shared logging/units/profiling,
  visualization/debug drawing, future API server wrapper.

## 10. Known limitations and prioritized backlog

Consolidated with §9.2 and FIDELITY §5 — highest-value gaps: study-output
telemetry (binary reader done); global terrain (DEM/DTED ingestion, streaming,
datum/geoid, dateline/polar); probabilistic failure degradation/partial
health/repair (deterministic flags done, MVP); GNC depth (imaging IR, multi-target,
dynamic illuminator, band-resolved extinction, multi-rate timestamp
interpolation); guidance/aero depth (trajectory management, pursuit, LQR/MPC,
blended handoff, CFD validation, Reynolds/nonlinear aero — static coefficient
tables done); GPU parity (validate Vulkan
vs CPU, GPU ECEF, CUDA if required); explicit versioned StrikeSim/StrikeDesigner/
StrikeCEM/StrikeCFD handoffs and provenance.

## 11. Change and release gate

Public behavior changes MUST update `SPEC.md`, this record, related config/data
docs, and regression tests. A historical note or backlog line is not evidence;
accepted inputs, outputs, defaults, failure behavior, and executable validation are
required.
