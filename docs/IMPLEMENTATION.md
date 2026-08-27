# StrikeEngine Implementation Record

**Status:** authoritative implementation record
**Companion specification:** [`SPEC.md`](SPEC.md)
**Verified:** 2026-08-26
**Runtime checkpoint:** `1ff6d1e`

This document maps the normative behavior in [`SPEC.md`](SPEC.md) to source
files, build targets, execution order, validation, packaging, and remaining
work. [`FIDELITY_AUDIT.md`](FIDELITY_AUDIT.md) supplies measured evidence;
it is not an alternative authority.

## 1. Verified state

- Project version: `0.1.0`; language: C++23; minimum CMake: 3.23.
- Default build: static `strikeengine` library with CPU backend.
- Optional companion: `strikeengine_vulkan`, enabled with
  `STRIKEENGINE_WITH_VULKAN=ON`.
- Release validation: **21/21 CTest tests pass**.
- Default local frame and constant-gravity behavior remain backward-compatible.
- The requested `.idea` project metadata change is included in this next
  documentation checkpoint; it is not runtime behavior.

## 2. Repository map

| Area | Responsibility |
| --- | --- |
| `include/strikeengine/kernel` | Public state blocks, kernel, systems, backends, integrators |
| `include/strikeengine/models` | Stateless atmosphere, aero, propulsion, earth, guidance, signatures |
| `include/strikeengine/simulation` | Single run, sweep, Monte Carlo, optimizer, batch wrappers |
| `src/strikeengine` | Runtime and signature database implementations |
| `tests/validation` | End-to-end and subsystem regression programs |
| `data/` | Example profiles, scenarios, tables, and schemas |
| `tools/` | Atmosphere generation and terrain conversion utilities |
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
4. Generate IMU/GPS measurements.
5. Propagate and fuse navigation.
6. Update seekers, lock state, filtered LOS rates, and delayed outputs.
7. Compute guidance demands.
8. Convert demands to actuator commands.
9. Evaluate terrain/ground events and dispatch events.

Truth is advanced before sensing; guidance and autopilot output affects the next
physics step. This ordering is part of the integration contract.

`SimulationKernel` accepts an `IntegratorType` (default RK4) which is applied
CPU-side only.

## 5. Module ownership

### Core and configuration

- `SimulationKernel.hpp/.cpp`: lifecycle, entity management, command queue,
  orchestration, environment, and public block access.
- `EnvironmentConfig.hpp`: terrain/wind callbacks and earth options including
  `useEcefTruth`, gravity selection, Coriolis, centrifugal, and transport.
- `ScenarioConfig.hpp`: scenario metadata, environment, entity/vehicle setup,
  target state, and kernel loading.
- `VehicleConfig.hpp`: per-entity geometry, aero, dry mass, thrust, and Isp.

### Truth and earth models

- `ISA1976.hpp`: layered atmosphere through 86 km.
- `AeroModel.hpp`: drag, bounded lift, fin side force, stability, damping, and
  bounded moments.
- `PropulsionModel.hpp` / `ThrustCurve.hpp`: thrust interpolation, Isp, mass flow,
  and dry-mass limiting.
- `CPUBackend.cpp`: stage-re-evaluated force model, body Euler dynamics,
  quaternion propagation, and servo dynamics.
- `EarthModel.hpp`: WGS84 conversion, normal gravity, Coriolis, curvature, and
  transport APIs.
- `EarthFrames.hpp`: ECEF/ENU/NED transforms, local gravity, and centrifugal
  acceleration.
- `EarthFixedPropagator.hpp`: standalone rotating-Earth ECEF RK4 propagator.
- `EventSystem.cpp`: local terrain views, geodetic altitude, and ellipsoid
  impact clamping.

### GNC and studies

- `SensorSystem.cpp`: noisy frame-aware IMU/GPS.
- `NavigationSystem.cpp`: alignment, strapdown INS, and coupled 15-state EKF.
- `SeekerSystem.cpp`: RF/IR signatures, geometry, lock, rates, and latency.
- `GuidanceSystem.cpp`: PN, waypoint guidance, and seeker APN handoff.
- `AutopilotSystem.cpp`: world-to-body demand conversion and bounded fin control.
- `SingleRun`, `ParamSweep`, `MonteCarlo`, `Optimizer`, `BatchRunner`: study
  wrappers; frame normalization is implemented in `Reporting.hpp` and
  versioned CSV/binary output in `StudyOutput.hpp/.cpp`. `BatchRunner`
  aggregate metrics `maxAltitudeM`/`maxSpeedMps` track the trajectory maximum,
  not just the final state.

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
recording; binary readers and richer production telemetry remain future work.

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
| `reporting` | versioned local/ECEF wrapper output, field selection, and binary recording |

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
  `AeroForces` data, a fuel/staging model, advanced atmosphere and weather,
  DEM/DTED loading and query services, global terrain tile streaming,
  datum/geoid handling, and polar/dateline policy. Existing atmosphere and
  terrain-conversion tools are preparation, not completion of these features.
- **Navigation and sensing:** sensor-fusion services, magnetometer, barometer,
  radar altimeter, coning/sculling, lever arms, complete earth-rate gyro
  compensation, and richer measurement timing/calibration.
- **Guidance, control, and seekers:** trajectory, waypoint, and energy
  managers; pursuit guidance; LQR/MPC; seeker management and blended handoff;
  semi-active radar and optical/EO seekers; richer radar detection/tracking;
  and higher-fidelity RF/IR propagation.
- **Execution and platforms:** parallel CPU execution, validated Vulkan/CPU
  parity, GPU ECEF truth, and CUDA if a concrete requirement is established.
- **Tools and integration:** binary output readers, richer telemetry schemas,
  versioned scenario/config loading, plotting/analysis/scenario-generation
  tools, shared logging/units/profiling utilities, visualization/debug drawing,
  and the future API server wrapper.

## 10. Known limitations and prioritized backlog

1. **Study output consumers:** add a binary reader, richer telemetry schemas,
   streaming record sinks, and configurable output selection beyond the current
   wrapper records.
2. **Global terrain:** `tools/convert_srtm.cpp` exists, but runtime terrain is
   still a callback. Add DEM/DTED tiles, interpolation, streaming, datum/geoid
   policy, dateline/polar handling, and frame-aware collision queries.
3. **Failures:** add motor, actuator, sensor, structural, and communications
   failure models and event semantics.
4. **GNC fidelity:** add coning/sculling, lever arms, full earth-rate gyro
   compensation, richer RF/IR propagation, and more seeker types.
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
