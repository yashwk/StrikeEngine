# StrikeEngine — Fidelity Audit

**Audit date:** 2026-08-29<br>
**Runtime checkpoint:** `9c42316`<br>
**Validation result:** Release build, **34/34 CTest tests passed**

[`SPEC.md`](SPEC.md) is the normative contract;
[`IMPLEMENTATION.md`](IMPLEMENTATION.md) is the source-to-feature map. This audit
does not override either.

## 1. Audit method and status vocabulary

A capability is implemented only when present in the runtime and covered by
deterministic regression evidence; a present-but-bounded feature stays
**MVP / partial**.

| Status | Meaning |
| --- | --- |
| **Implemented** | Runtime behavior exists and is covered by the current Release validation suite. |
| **MVP / partial** | A bounded implementation is validated; fidelity or integration limits remain. |
| **Planned** | Recorded as future work; not part of the supported contract. |
| **Unsupported** | Callers must not rely on the capability or an implicit fallback. |

## 2. Current workstream verification

| Workstream | Status | Evidence |
| --- | --- | --- |
| W1 — Per-entity vehicle configuration | Implemented | `vehicleconfig_test` |
| W2 — 6-DOF rigid-body truth | Implemented | `rigidbody_test` |
| W3 — Control authority | MVP / partial | `intercept_test`; min miss 0.76 m, post-burnout control |
| W4 — True integration and impact timing | MVP / partial | `integrator_test`; Euler/RK4/RK45/Symplectic, adaptive, interpolated crossing |
| W5 — Events and environment | MVP / partial | `environment_test`; failure/damage via `failure_test`; terrain DBs open |
| W17 — Failure and damage semantics | Implemented / MVP | `failure_test`; deterministic flags + events; no probabilistic/partial-health/repair |
| W18 — IMU lever-arm compensation | Implemented / MVP | `lever_arm_test`; α×l + ω×(ω×l) |
| W19 — Earth-rate gyro modeling + compensation | Implemented / MVP | `earth_rate_gyro_test`; ECEF truth only, local excluded |
| W20 — Strapdown coning/sculling corrections | Implemented / MVP | `coning_sculling_test`; rotation-vector update + `+0.5(ω×f)dt²` |
| W6 — Seeker and sensor fidelity | Implemented / MVP | `seeker_test`, `seeker_rich_test`; RF/IR/SARH/PassiveRF, decoys, strongest-signal |
| W7 — Navigation EKF | Implemented / MVP | `navigation_test`; coupled 15-state, GPS corrections, scalar innovation gating and rejection diagnostics |
| W8 — Scenario and guidance contract | Implemented / MVP | `guidance_test`, `scenario_test`; PN/APN, configured navigation constant, waypoint, batch |
| W9 — WGS84 local-earth model | MVP / partial | `earth_test`; geodetic/ECEF, normal gravity, Coriolis |
| W10 — Earth frames and acceleration | MVP / partial | `earth_frames_test`; ECEF/ENU/NED, centrifugal |
| W11 — Moving-origin transport | MVP / partial | `earth_transport_test` |
| W12 — Spherical gravity | MVP / partial | `spherical_gravity_test` |
| W13 — Standalone global ECEF propagation | MVP / partial | `earth_fixed_test`; rotating-Earth RK4 |
| W14 — Kernel ECEF truth mode | MVP / partial | `ecef_kernel_test` |
| W15 — Frame-aware study reporting | MVP / partial | `reporting_test` |
| W16 — Structured study output | MVP / partial | `reporting_test`; CSV/binary + reader |
| W21 — Subsystem config and serialization | Implemented / MVP | `serialization_test`; `VehicleConfig`, snake_case round-trip (four profile-id keys, legacy-compat), `designRef` override |
| W24 — Profile-id database layer | Implemented / MVP | `profile_database_test`; fail-fast loaders, profile-wins resolution |
| W22 — Sensor enablement and gain wiring | Implemented / MVP | `config_wiring_test`; IMU freeze, GPS schedule, gain wiring |
| W23 — Staging and warhead fusing | MVP / partial | `staging_warhead_test`; two-stage separation + fusing |
| W25 — Designer→engine pipeline | Implemented / MVP | `designer_pipeline_test`; 49.1 m miss < 50 m at t≈11.5 s + kill after configured-PN rebaseline; seed `0xDEADBEEF`, pipeline demo not a performance claim |
| W26 — Rocket-launch verification + propulsion-law fix | Implemented | `rocket_mvp_test`; T0 60000 N, flow 27.81 kg/s, init accel 110.275 vs 110.208 m/s², cutoff vs Δv = Isp·g0·ln(m0/mdry), apogee 17.19 km, max-Q ~242 kPa; `T = ṁ·Isp·g0`, no free-thrust tail |
| W27 — Data-driven aero coefficient tables | Implemented / MVP | `coefficient_table_test`, `rocket_mvp_tables_test`; table run higher/faster (apogee 24.79 vs 17.19 km, burnout V 713.6 vs 686.8 m/s, max-Q 260.4 vs 242.2 kPa); fallback byte-identical; awaiting StrikeCFD |
| W28 — Leftover-propellant dump + warhead falloff | Implemented / MVP | `staging_warhead_test`, `warhead_falloff_test`; dump lands mass on new floor, inertia rescale, `dumpedMassKg`; falloff band 1/linear/0, RNG draw only when `0 < p < 1`; SAM 50/50/90 m; 49.1 m miss |
| W29 — Geometric fins (RocketPy port) | Implemented / MVP | `fins_test`; three shapes (trapezoidal/elliptical/free-form); Diederich planform lift slope + Prandtl–Glauert Mach correction; fin-number + interference corrections; per-shape CP; tail lever-arm sign convention (restoring); positive-cant roll forcing; ballistic flight on rocket_mvp with 4 tail fins — apogee 17.2 km, drift ~0 m; byte-identical fallback when `fins` absent |
| W30 — Static moment/lateral aero tables | Implemented / MVP | `coefficient_table_test`, `serialization_test`, `profile_database_test`; Cm(M,α), Cy(M,β), Cn(M,β), rolling Cl(M,β), finite/strict-grid validation, bilinear interpolation, scalar fallback |
| W31 — ECEF/geodetic states + J2 gravity | Implemented / MVP | `earth_test`, `earth_fixed_test`, `ecef_kernel_test`, `serialization_test`; explicit ENU/ECEF velocity conversion, pole/dateline round-trip, WGS84 J2 gravity, standalone propagator and truth/sensor/navigation consistency |
| W32 — Propulsion transients, TVC, and feed failures | Implemented / MVP | `propulsion_test`; strict curve/config validation, ignition delay/ramp, shutdown ramp, two-axis achieved-gimbal limits, engine-position torque, explicit engine/tank failure flags/events, legacy motor-failure compatibility |
| W33 — Geodetic global terrain raster | Implemented / MVP | `global_terrain_test`; in-memory raster, ESRI ASCII Grid loading, NODATA handling, dateline normalization, local ENU and ECEF terrain impact/clamping |
| W34 — Extended terrain sources and surfaces | Implemented / MVP | `global_terrain_test`; nearest/bilinear sampling, explicit coverage status, WGS84-scaled normals/slope, optional GDAL GeoTIFF/DTED/VRT loading, bounded LRU reuse, impact-event surface payload |
| W35 — GPS fusion robustness | Implemented / MVP | `navigation_test`, `serialization_test`; configurable scalar normalized-innovation gate rejects gross/non-finite fixes while preserving valid channels |
| W36 — Configured PN authority | Implemented / MVP | `guidance_test`; kernel PN now applies each entity's configured navigation constant rather than the model default |

## 3. Subsystem fidelity assessment

| Area | Current implementation | Assessment and limit |
| --- | --- | --- |
| Translational truth | World-frame force rotation ÷ mass + gravity/earth terms | **MVP:** cd/cl/cy data-driven when present; no CFD-backed or nonlinear aero model |
| Rotational truth | Diagonal inertia, gyroscopic coupling, quaternion, bounded fins | **MVP:** no inertia-tensor or flexible-body model |
| Atmosphere | Layered ISA1976 through 86 km | **Implemented for stated envelope:** no weather model |
| Aerodynamics | Drag/AoA-lift/fin/side-force/stability/damping; optional cd/cl/cm/cy/cn/rolling-cl tables; geometric fins (trapezoidal/elliptical/free-form) | **MVP:** static tables authoritative with scalar fallback; fins implement Mach-scaled fin effectiveness + lateral (β) side-force/stability for angled fins; tables are not yet CFD-validated and omit Reynolds/nonlinear stall/post-stall effects |
| Actuators and control | World→body demand, bounded fins, servo lag, rate limit, per-entity gains | **MVP:** fixed gains; no scheduling/failure/advanced control |
| Integration | Euler/RK4/RK45/Symplectic, adaptive, interpolated impact | **MVP:** no multirate or full event-aware adaptive policy |
| Earth and frames | WGS84, normal/spherical/J2 gravity, explicit state conversion, frames, Coriolis/centrifugal/transport, ECEF, geodetic terrain sources | **MVP:** no geoid, automatic spatial multi-tile discovery/streaming, atmospheric rotation/wind coupling, or full moving-origin global propagator |
| Sensors | IMU/GPS with lever arm, earth-rate gyro, per-entity enablement | **MVP:** IMU-disable = GPS-only aiding, not a full GPS-only mode; timing contract open |
| Profile database layer | Aero/motor/seek/sensor loaders, `createVehicle` resolution | **Implemented / MVP:** guidance/autopilot, warhead, mass/inertia, RCS/IR/emitter NOT profile-resolved; one profile per file |
| Navigation | Alignment, strapdown INS, 15-state EKF, scalar GPS innovation gating and diagnostics | **MVP:** earth-rate gyro ECEF-only; no multi-rate timestamp interpolation or broader sensor fusion |
| Seekers | RF/SARH/PassiveRF/IR, FOV/gimbal, hysteresis, LOS rates, latency, decoys | **MVP:** no imaging IR, multi-target, dynamic illuminator, band-resolved extinction |
| Guidance | Stateless PN/APN, waypoint, seeker handoff, per-entity gains | **MVP:** no trajectory manager, pursuit, LQR/MPC, blended handoff |
| Events and terrain | Local callbacks plus geodetic raster sources, nearest/bilinear status-aware sampling, terrain normals/slope, real local/ECEF impact deactivation and clamping, enriched impact events | **MVP:** GDAL source loading is eager and single-source; no automatic spatial tile discovery/streaming, prefetch, datum/geoid, or probabilistic failure |
| Warhead and fusing | Impact/proximity/timed fusing; flat or linear falloff; `StageSeparation` | **MVP:** linear band; `lethalRadiusM <= 0` inert; `falloff < lethal` rejected |
| Failure and damage | Deterministic motor/engine/tank flags → thrust/feed cut, fin freeze, sensor dropout, ballistic comms, structural | **MVP:** deterministic no-leak feed failure only; partial health no effect; no repair |
| Study wrappers and outputs | Single run, sweep, Monte Carlo, optimizer, batch; versioned CSV/binary + reader | **MVP:** richer telemetry/streaming remain; some optimizer paths primary-entity oriented |
| Backends and packaging | Deterministic CPU/static library, CMake packaging, optional Vulkan | **MVP:** Vulkan parity not validated, no GPU ECEF/CUDA |

## 4. Quantitative validation evidence

- Release CTest suite green: **34/34 tests passed** at the checkpoint above.
- Control regression: **0.76 m minimum miss** (MVP control path; not a general
  accuracy guarantee).
- Designer→engine pipeline: **49.1 m minimum miss** at t≈11.5 s with a
  proximity-warhead kill (`designer_pipeline_test`), flying on data-driven aero
  tables; seed `0xDEADBEEF`, a data-contract demonstration, not a
  guidance-performance claim.
- `rocket_mvp_test` cross-checks initial acceleration vs `T/m − g` and burnout vs
  `Δv = Isp·g0·ln(m0/mdry)`; `rocket_mvp_tables_test` shows the byte-identical
  constant fallback and higher table-run apogee/burnout-V/max-Q.

These results establish regression coverage for implemented paths, not
production-grade aerodynamics, global geophysics, sensor calibration, or GPU
equivalence.

## 5. Prioritized open work

Prioritized gaps (details in IMPLEMENTATION §9.2):

1. **Study output:** binary reader done; richer telemetry, streaming sinks, output
   selection remain.
2. **Global terrain:** geodetic sources, nearest/bilinear sampling,
   coverage/NODATA status, surface normals/slope, optional GDAL
   GeoTIFF/DTED/VRT ingestion, bounded caching, and frame-aware collision/event
   queries are implemented (`global_terrain_test`); automatic spatial multi-tile
   discovery/streaming, prefetch, datum/geoid, and higher-fidelity polar coverage
   remain.
3. **Failure/damage:** deterministic flags done (MVP, `failure_test`);
   probabilistic degradation, partial health, repair remain.
4. **GNC fidelity:** lever arms, coning/sculling, earth-rate gyro done (MVP,
   `lever_arm_test`, `coning_sculling_test`, `earth_rate_gyro_test`); imaging IR,
   multi-target, dynamic illuminator, band-resolved extinction, multi-rate
   timestamp interpolation remain.
5. **Staging/warhead:** leftover dump + falloff band and propulsion transients/TVC
   are implemented (`staging_warhead_test`, `warhead_falloff_test`,
   `propulsion_test`); `T = ṁ·Isp·g0` enforced at exhaustion (`rocket_mvp_test`);
   arbitrary stage-count validation and a physics-based blast/debris model remain.
6. **Guidance/aero:** static cd/cl/cm/cy/cn/rolling-cl table plumbing is done
   (`coefficient_table_test`, `serialization_test`, `profile_database_test`);
   CFD validation, Reynolds/nonlinear aero, trajectory management, pursuit,
   LQR/MPC, and blended handoff remain.
7. **Backend parity:** validate Vulkan vs CPU, GPU ECEF, CUDA if required.
8. **Application handoffs:** designer→engine contract exercised end-to-end
   (`designer_pipeline_test`); explicit versioned StrikeSim/StrikeDesigner/
   StrikeCEM integration + provenance contracts remain.

Still deferred: power/comms/ECM models, StrikeCEM/StrikeCFD coupling (and
CFD validation of the moment (cm)/lateral (β) coefficient tables), a full
 GPS-only positioning mode (current GPS-only aiding is not one), guidance depth
(trajectory/pursuit/LQR/MPC), Vulkan/CPU parity, and automatic multi-tile global
terrain discovery/streaming. The
falloff band, leftover-propellant dump, and geometric fins (Mach-scaled fin
effectiveness, lateral β side-force/stability for angled fins) are implemented
and no longer deferred.
Revived artifacts (`data/aero`, `data/motors`, `data/seekers`, `data/sensors`,
`data/rcs`, `data/profiles`, `data/scenarios/intercept_test_01`,
`data/schemas/seeker_schema.json`) are part of this layer.

## 6. Historical baseline

The 2026-08-25 pre-restart audit recorded 5/7 workstreams with failures in control
signs, aero authority, integration, events, seeker fidelity, and navigation; it is
superseded by W1–W36 above. Historical measurements/commits remain in repository
history and are not repeated here.

## 7. Conclusion

StrikeEngine is a validated deterministic CPU simulation MVP (per-entity physics,
GNC, terrain/wind callbacks, WGS84 terrain sources, local-earth models, opt-in
kernel ECEF truth). It is suitable for continued engineering development and
regression testing, but not yet production-grade global geophysics, sensor, aero,
or GPU-equivalent simulation until the backlog is addressed.
