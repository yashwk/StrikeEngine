# StrikeEngine — Fidelity Audit

**Audit date:** 2026-08-26<br>
**Runtime checkpoint:** `9ee7ef9`<br>
**Validation result:** Release build, **32/32 CTest tests passed**

> This document records measured fidelity and current limitations. [`SPEC.md`](SPEC.md)
> is the normative product contract and [`IMPLEMENTATION.md`](IMPLEMENTATION.md)
> is the authoritative source-to-feature map. This audit does not override either
> document.

## 1. Audit method and status vocabulary

The audit treats a capability as implemented only when its behavior is present
in the runtime and covered by deterministic regression evidence. A feature can
therefore be present but still marked **MVP / partial** when its model is
deliberately bounded or its production integration is incomplete.

| Status | Meaning |
| --- | --- |
| **Implemented** | Runtime behavior exists and is covered by the current Release validation suite. |
| **MVP / partial** | A bounded implementation is validated, but important fidelity or integration limits remain. |
| **Planned** | The capability is recorded as future work and is not part of the supported contract. |
| **Unsupported** | Callers must not rely on the capability or on an implicit fallback. |

The evidence below is intentionally consolidated: each workstream appears once,
and each limitation is listed once in the subsystem assessment or backlog.

## 2. Current workstream verification

| Workstream | Status | Evidence and current boundary |
| --- | --- | --- |
| W1 — Per-entity vehicle configuration | Implemented | `vehicleconfig_test`; mass, dry-mass limiting, geometry, aero, thrust, and Isp are per vehicle. |
| W2 — 6-DOF rigid-body truth | Implemented | `rigidbody_test`; quaternion normalization, body-frame rates, diagonal inertia, gyroscopic coupling, and commanded climb pass. |
| W3 — Control authority | MVP / partial | `intercept_test`; minimum miss is 25.30 m and post-burnout control is exercised. Guidance and control remain engineering-model fidelity. |
| W4 — True integration and impact timing | MVP / partial | `integrator_test`; derivative-callback Euler, RK4, RK45, Symplectic/Velocity-Verlet, bounded adaptation, and interpolated ground crossing are covered. |
| W5 — Events and environment | MVP / partial | `environment_test`; terrain and wind callbacks plus real impact deactivation are covered. Failure/damage workstream is Implemented/MVP via `failure_test`. Terrain databases are open. |
| W17 — Failure and damage semantics | Implemented / MVP | `failure_test`; deterministic motor, actuator, sensor, structural, and communication failure flags with real state transitions and per-type events are covered. Boundary: no probabilistic degradation, no partial health effects beyond deactivation, no repair. |
| W18 — IMU lever-arm compensation | Implemented / MVP | `lever_arm_test`; per-entity body-frame lever-arm specific-force correction (α×l + ω×(ω×l)) is covered. |
| W19 — Earth-rate gyro modeling + compensation | Implemented / MVP | `earth_rate_gyro_test`; opt-in `includeEarthRateGyro` in ECEF truth mode adds ω_ie^b to the gyro and compensates it in the INS with no attitude drift. Local flat-earth mode is intentionally excluded. |
| W20 — Strapdown coning/sculling corrections | Implemented / MVP | `coning_sculling_test`; rotation-vector attitude update (456× tighter than the first-order step on a coning environment) and single-interval sculling compensation `+0.5 (ω×f) dt^2` are validated against a fine-step reference. |
| W6 — Seeker and sensor fidelity | Implemented / MVP | `seeker_test` and `seeker_rich_test`; FOV, gimbal limits, hysteresis, filtered LOS rates, latency, RF/IR/SARH/PassiveRF detection, Beer-Lambert IR transmittance, chaff/flare decoys, strongest-signal acquisition, and the public `SeekerConfig` surface are covered. Imaging IR, multi-target tracking, and dynamic illuminator tracking remain open. |
| W7 — Navigation EKF | MVP / partial | `navigation_test`; coupled 15-state covariance, GPS corrections, bounds, and deterministic bias convergence are covered. Full inertial compensation is not complete. |
| W8 — Scenario and guidance contract | MVP / partial | `guidance_test` and `scenario_test`; PN/APN, moving-target response, seeker handoff, scenario propagation, and isolated batch execution are covered. |
| W9 — WGS84 local-earth model | MVP / partial | `earth_test`; geodetic/ECEF conversion, normal gravity, and local Coriolis are covered. This is not a complete geophysical model. |
| W10 — Earth frames and acceleration | MVP / partial | `earth_frames_test`; ECEF/ENU/NED transforms and centrifugal acceleration are covered in the local-earth path. |
| W11 — Moving-origin transport | MVP / partial | `earth_transport_test`; WGS84 curvature, local geodetic resolution, transport acceleration, and CPU integration are covered. |
| W12 — Spherical gravity | MVP / partial | `spherical_gravity_test`; radial ECEF point-mass gravity, local projection, and CPU integration are covered. |
| W13 — Standalone global ECEF propagation | MVP / partial | `earth_fixed_test`; rotating-Earth gravity, Coriolis, centrifugal terms, caller force, and deterministic RK4 are covered. |
| W14 — Kernel ECEF truth mode | MVP / partial | `ecef_kernel_test`; ECEF truth, geodetic atmosphere/ground handling, ECEF GPS/INS flow, and ellipsoid-clamped impact are covered. |
| W15 — Frame-aware study reporting | MVP / partial | `reporting_test`; versioned local/ECEF CSV metadata, geodetic coordinates, normalized altitude, and explicit primary-entity selection are covered. |
| W16 — Structured study output | MVP / partial | `reporting_test`; configurable fields, status/entity metadata, versioned CSV output, binary recording, and the binary reader are covered. Richer telemetry and streaming remain open. |
| W21 — Subsystem config and serialization | Implemented / MVP | `serialization_test`; flattened per-vehicle `VehicleConfig`, snake_case JSON round-trip of all config structs (including the four profile-id keys and a legacy-compat case), and scenario/design load-save are covered. The `designRef` override (overrides inline `vehicleConfig` on scenario load) is regression-covered. |
| W24 — Profile-id database layer | Implemented / MVP | `profile_database_test`; per-subsystem aero/motor/seek/sensor profile loaders (false on any load failure), `createVehicle` profile-wins resolution into the SoA blocks, empty-id regression, and missing/schema-broken profile → `std::runtime_error` naming the file. Shipped `data/aero`, `data/motors`, `data/seekers`, `data/sensors` example profiles parse. |
| W22 — Sensor enablement and gain wiring | Implemented / MVP | `config_wiring_test`; per-entity IMU freeze, GPS scheduling/disable, and guidance/autopilot gain propagation with the configurable `maxDeflectionRad` clamp are covered. |
| W23 — Staging and warhead fusing | MVP / partial | `staging_warhead_test`; two-stage separation (dry-mass drop, inertia rescale, `StageSeparation`) and impact/proximity/timed fusing (`Detonation`, flat lethal-radius kill) are covered. |
| W25 — Designer→engine pipeline | Implemented / MVP | `designer_pipeline_test`; engine-consumable `data/profiles` design manifests, flat `data/rcs/target_drone_rcs.json` RCS table, rewritten `data/scenarios/intercept_test_01.json` in the ScenarioConfig schema, `SeekerTypeStrings.hpp` dedup, and an end-to-end guided intercept (16.77 m miss < 50 m at t≈19.7 s) with a proximity-warhead kill. Boundary: the scenario is a deterministically tuned data set (seed `0xDEADBEEF`) — a pipeline demonstration, not a guidance-performance claim; versioned provenance contracts remain future work. |
| W26 — Rocket-launch verification + propulsion-law fix | Implemented | `rocket_mvp_test`; first-principles WGS84 single-stage launch cross-checked by hand: T0 thrust (60000 N), T0 mass flow (27.81 kg/s at sea-level Isp), initial acceleration (measured 110.275 vs hand-computed 110.208 m/s², ~T/m − g_lat), burnout time within the pressure-interpolated-Isp band, mass at cutoff ≈ 350 kg, cutoff velocity (686.8 m/s) vs the ideal rocket equation Δv = Isp·g0·ln(m0/mdry) (874.4 m/s, loss ≈ 187.7 m/s explained by gravity ≈ 53.4 + drag + pressure-interpolated Isp), apogee (17.19 km), max-Q (~242 kPa), ISA-1976 sea-level density, WGS84 round trip, Somigliana monotone gravity (<0.1% at altitude), lateral drift ~0, and a 70°-elevation arcing case. The fuel-depletion guard scales thrust with the capped mass flow so `T = ṁ·Isp·g0` holds at fuel exhaustion (no free-thrust tail); this fix reduced the designer-pipeline min miss to 16.77 m. |

## 3. Subsystem fidelity assessment

| Area | Current implementation | Assessment and limit |
| --- | --- | --- |
| Translational truth | Body aero/thrust forces are rotated into the selected world frame, divided by current mass, and combined with gravity and configured earth terms. | **MVP / partial:** an engineering model; no validated coefficient-table or CFD-backed model. |
| Rotational truth | Diagonal body inertia, gyroscopic coupling, body angular rates, quaternion attitude, and bounded fin moments. | **MVP / partial:** no full inertia-tensor or flexible-body model. |
| Atmosphere | Layered ISA1976 atmosphere through 86 km, with density, pressure, temperature, and sound speed. | **Implemented for the stated envelope:** no weather or high-fidelity atmospheric model. |
| Aerodynamics | Air-relative drag, angle-of-attack lift, fin pitch lift, yaw-fin side force, stability, rate damping, and bounded moments. | **MVP / partial:** coefficients are simplified and not validated against tables, CFD, or wind-tunnel data. |
| Propulsion | Per-entity thrust curves, vacuum/sea-level Isp interpolation, mass flow, dry-mass limiting, axial body +X thrust, and ordered multi-stage staging (dry-mass drop, inertia rescale, stage separation). The motor law `T = ṁ·Isp·g0` is enforced at every instant: the fuel-depletion guard scales thrust with the capped mass flow, so thrust self-terminates at fuel exhaustion (no free-thrust tail). | **MVP / partial:** per-stage `propellantMassKg` caps stage drawdown (later-stage propellant reserved via a stage mass floor); no thrust vectoring or fuel-tank/engine failure model. |
| Actuators and control | World-to-body acceleration demand, bounded fin commands, first-order servo lag, rate limiting, pitch/yaw sign conventions, and per-entity configurable gains/clamp. | **MVP / partial:** fixed per-entity engineering gains; no gain scheduling, actuator failure, or advanced controller. |
| Integration | Derivative callbacks with true stage re-evaluation for RK4/RK45; bounded adaptive substeps; interpolated impact crossing; kernel integrator selection exposed via `IntegratorType` at construction (CPU-side). | **MVP / partial:** no multirate solver or complete event-aware adaptive policy. |
| Earth and frames | WGS84 conversion, normal and spherical gravity, ECEF/ENU/NED transforms, Coriolis, centrifugal, transport terms, standalone ECEF propagation, and opt-in kernel ECEF truth. | **MVP / partial:** no geoid, global terrain streaming, polar/dateline scenario policy, or complete earth-rate treatment across every subsystem. |
| Sensors | Body-frame IMU specific force and rates with noise/bias; per-entity IMU lever-arm specific-force correction; opt-in ECEF earth-rate gyro modeling (inertial body rate); noisy GPS in the selected frame; per-entity IMU/GPS enablement and GPS rate. | **MVP / partial:** a disabled IMU freezes its held sample and stops bias drift (GPS-only aiding, not a full GPS-only positioning mode); the full timing/interpolation contract remains open. |
| Profile database layer | Aero/motor/seek/sensor single-profile loaders (`loadProfile` returns false on any failure) resolved by `createVehicle`; a non-empty `aeroProfileId`/`motorProfileId`/`seekerProfileId`/`sensorProfileId` replaces the inline sub-config, and a failed load throws `std::runtime_error` naming the file. | **Implemented / MVP:** guidance/autopilot, warhead, mass/inertia, and RCS/IR/emitter signatures are NOT profile-resolved; the loader layer resolves one profile per file with no caching or database index. |
| Navigation | Perfect initial alignment, strapdown INS (rotation-vector attitude update + single-interval sculling compensation), and coupled 15-state error-state EKF with GPS position/velocity updates. | **MVP / partial:** earth-rate gyro applies to ECEF truth mode only (the flat-earth local truth gyro already resolves the non-rotating-frame body rate; correct local earth-rate compensation requires the rotating-frame ECEF navigation path); multi-rate timestamp interpolation remains open. |
| Seekers | Monostatic RF, SARH (bistatic, static illuminator), PassiveRF (target EIRP), and IR (Beer-Lambert transmittance) seekers; FOV/gimbal limits, lock hysteresis, filtered LOS rates, latency, friendly rejection, chaff/flare decoys, strongest-signal acquisition, and the public `SeekerConfig` surface. | **MVP / partial:** imaging IR, multi-target tracking, dynamic illuminator tracking, and band-resolved extinction are not implemented. |
| Guidance | Stateless PN/APN helpers, target velocity, waypoint mode, seeker-lock APN handoff, and per-entity navigation/waypoint gains. | **MVP / partial:** no trajectory manager, pursuit, LQR/MPC, or blended handoff. |
| Events and terrain | Terrain/wind callbacks, geodetic/local terrain views, real impact deactivation, position clamping, timestamped ground-impact events, and deterministic failure/damage events. | **MVP / partial:** no runtime DEM/DTED database, streaming, datum/geoid policy, or probabilistic failure model. |
| Warhead and fusing | Impact/proximity/timed fusing driven by `WarheadConfig`; detonation dispatches a `Detonation` event and destroys every alive entity within `lethalRadiusM`; stage separation dispatches a `StageSeparation` event. | **MVP / partial:** the lethal-radius kill is a flat cut with no fragmentation or overpressure falloff curve; `lethalRadiusM <= 0` warheads are inert. |
| Failure and damage | Deterministic per-entity flags (`failEntity`/`applyDamage`) driving thrust/mass-flow cutoff, fin freeze, sensor dropout, ballistic comms loss, and structural deactivation with per-type events. | **MVP / partial:** flags are deterministic and not probabilistic; partial health has no effect beyond deactivation; repair is not modeled. |
| Study wrappers and outputs | Single run, sweep, Monte Carlo, optimizer, and isolated batch runner with configurable versioned CSV/binary reporting and a binary reader. | **MVP / partial:** richer telemetry and streaming remain, and some optimizer paths remain primary-entity oriented. |
| Backends and packaging | Deterministic CPU/static library, CMake packaging, and optional Vulkan target. | **MVP / partial:** Vulkan parity is not validated, ECEF GPU support is open, and CUDA is not implemented. |

## 4. Quantitative validation evidence

- The complete Release CTest suite is green: **32/32 tests passed** at the
  checkpoint recorded above.
- The control regression reports a **25.30 m minimum miss** for its validated
  intercept scenario. This demonstrates the MVP control path; it is not a
  general accuracy guarantee.
- The designer→engine pipeline regression reports a **16.77 m minimum miss**
  at t≈19.7 s with a proximity-warhead kill (`designer_pipeline_test`). This
  demonstrates the end-to-end designer→engine data contract (design manifest →
  `designRef` → profile-id wiring → kernel → intercept); it is a
  deterministically tuned data set and not a guidance-performance claim.
- The test inventory covers study wrappers; vehicle configuration; rigid-body
  truth; intercept control; integration; seeker (FOV/hysteresis/LOS-rate,
  plus rich seekers: SARH, passive RF, IR transmittance, decoys, strongest
  signal, config round-trip); navigation; environment;
  earth/frame/transport/gravity models; standalone ECEF propagation; kernel
  ECEF truth; guidance; scenario loading; kernel slot-reuse and timestep
  validation; deterministic failure/damage semantics; IMU lever-arm
  compensation; earth-rate gyro modeling/compensation; strapdown
  coning/sculling corrections; subsystem config serialization and
  scenario/design interchange (including the four profile-id keys and a
  legacy-compat case); per-entity sensor enablement and
  guidance/autopilot gain wiring; multi-stage staging plus warhead
   fusing; the profile-id database layer; the designer→engine pipeline
   end-to-end (design manifests, flat RCS table, rewritten scenario, guided
   intercept); and the first-principles rocket-launch verification
   (`rocket_mvp_test`) cross-checking burnout against the ideal rocket equation
   and initial acceleration against `T/m − g`.

The results establish regression coverage for the implemented paths. They do
not establish production-grade aerodynamics, global geophysics, sensor
calibration, or GPU equivalence.

## 5. Prioritized open work

These are the remaining fidelity and integration gaps, ordered by their value
to reliable downstream use:

1. **Study output consumers:** the binary reader is implemented; richer
   telemetry schemas, streaming record sinks, and configurable output selection
   beyond the current wrapper records remain.
2. **Global terrain:** add DEM/DTED ingestion, tile indexing and streaming,
   interpolation, datum/geoid policy, dateline/polar handling, and frame-aware
   collision queries.
3. **Failure and damage semantics:** deterministic motor, actuator, sensor,
   structural, and communications failures with state transitions and events
   are implemented (MVP, `failure_test`). Remaining boundary: probabilistic
   degradation, partial health effects beyond deactivation, and repair.
4. **GNC fidelity:** sensor lever-arm compensation, coning/sculling, and
   earth-rate gyro compensation are implemented (MVP). Lever arms: per-entity
   `VehicleConfig::imuLeverArm*` rigid-body specific-force correction
   (`lever_arm_test`). Coning/sculling: rotation-vector attitude update and
   single-interval sculling compensation (`coning_sculling_test`). Earth-rate
   gyro: `includeEarthRateGyro` in ECEF truth mode only (`earth_rate_gyro_test`);
   the flat-earth local truth gyro already resolves the non-rotating-frame body
   rate, and correct local earth-rate compensation requires the rotating-frame
   ECEF navigation path. Richer seekers are implemented (MVP): SARH (bistatic,
   static illuminator), passive RF (EIRP), Beer-Lambert IR transmittance,
   chaff/flare decoys, strongest-signal acquisition, and the public
   `SeekerConfig` surface (`seeker_rich_test`). Per-entity sensor enablement is
   implemented (MVP, `config_wiring_test`): a disabled IMU freezes its held
   sample and stops bias drift while GPS continues to aid the EKF (this is not
   a full GPS-only positioning mode); a disabled GPS stops updates. Remaining:
   imaging IR, multi-target tracking, dynamic illuminator tracking, band-resolved
   extinction, and multi-rate timestamp interpolation.
5. **Staging and warhead fidelity:** multi-stage staging (dry-mass drop, inertia
    rescale, `StageSeparation`) and impact/proximity/timed fusing (flat lethal-
    radius kill, `Detonation`) are implemented (MVP, `staging_warhead_test`).
    The propulsion model now enforces `T = ṁ·Isp·g0` at fuel exhaustion (the
    fuel-depletion guard scales thrust with the capped mass flow, so there is
    no free-thrust tail), verified by `rocket_mvp_test`. Remaining: a
    fragmentation/overpressure falloff curve and arbitrary stage-count
    validation. (Per-stage propellant drawdown is implemented; leftover
    propellant in a spent stage is not dumped.)
6. **Guidance and aero depth:** add validated coefficient tables, trajectory
   management, pursuit, LQR/MPC, and blended guidance handoff.
7. **Backend parity:** validate Vulkan against CPU truth, add GPU ECEF support,
   and implement CUDA only if a project requirement is established.
8. **Application handoffs:** the designer→engine data contract is now exercised
   end-to-end (`designer_pipeline_test`): design manifests under `data/profiles`
   are consumed via `design_ref` into `VehicleConfig`, profile ids resolve the
   subsystem parts, and the scenario runs in the engine's local ENU frame. What
   remains open is the explicit versioned StrikeSim, StrikeDesigner, and
   StrikeCEM integration and provenance contracts (identity/revision/geometry
   provenance metadata carried across the handoff).

The profile-id database layer resolves aero/motor/seek/sensor lookups, but it
does not change the status of the other deferred work, which stays deferred:
power/comms/ECM models, StrikeCEM/CFD coupling, a full GPS-only positioning
mode (current GPS-only aiding is not one), the fragmentation/overpressure
falloff curve, and leftover-propellant-not-dumped in spent stages. The revived
`data/aero`, `data/motors`, `data/seekers`, `data/sensors` profile artifacts and
the rewritten flat snake_case `data/schemas/seeker_schema.json` are part of this
layer; the designer-facing `data/profiles` manifests and `data/scenarios`
scenario are now engine-consumable (the missile/drone manifests and the
`intercept_test_01` scenario under the ScenarioConfig schema), with `data/rcs`
added for the flat target-drone RCS table.

## 6. Historical baseline

The pre-restart audit from 2026-08-25 recorded a 5/7 workstream result and
identified failures in control signs, aerodynamic authority, integration,
events, seeker fidelity, and navigation. Those measurements described the
older implementation and are superseded by the W1–W26 verification above.

The historical measurements and commits remain available in repository
history. They are not repeated here because retaining their stale tables in
the current audit made the document contradictory and obscured the current
status.

## 7. Conclusion

StrikeEngine is a validated deterministic CPU simulation MVP with per-entity
vehicle physics, GNC, terrain/wind callbacks, local-earth models, and opt-in
kernel ECEF truth. The current implementation is suitable for continued
engineering development and regression testing. It should not yet be treated
as a production-grade global geophysical, sensor, aerodynamic, or GPU-equivalent
simulation until the backlog above is addressed.
