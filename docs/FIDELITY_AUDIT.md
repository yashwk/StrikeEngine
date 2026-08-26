# StrikeEngine — Fidelity Audit

**Audit date:** 2026-08-26<br>
**Runtime checkpoint:** `40af724`<br>
**Validation result:** Release build, **19/19 CTest tests passed**

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
| W5 — Events and environment | MVP / partial | `environment_test`; terrain and wind callbacks plus real impact deactivation are covered. Failure models and terrain databases are open. |
| W6 — Seeker and sensor fidelity | MVP / partial | `seeker_test`; FOV, gimbal limits, hysteresis, filtered LOS rates, and latency are covered. Propagation and seeker-family depth remain limited. |
| W7 — Navigation EKF | MVP / partial | `navigation_test`; coupled 15-state covariance, GPS corrections, bounds, and deterministic bias convergence are covered. Full inertial compensation is not complete. |
| W8 — Scenario and guidance contract | MVP / partial | `guidance_test` and `scenario_test`; PN/APN, moving-target response, seeker handoff, scenario propagation, and isolated batch execution are covered. |
| W9 — WGS84 local-earth model | MVP / partial | `earth_test`; geodetic/ECEF conversion, normal gravity, and local Coriolis are covered. This is not a complete geophysical model. |
| W10 — Earth frames and acceleration | MVP / partial | `earth_frames_test`; ECEF/ENU/NED transforms and centrifugal acceleration are covered in the local-earth path. |
| W11 — Moving-origin transport | MVP / partial | `earth_transport_test`; WGS84 curvature, local geodetic resolution, transport acceleration, and CPU integration are covered. |
| W12 — Spherical gravity | MVP / partial | `spherical_gravity_test`; radial ECEF point-mass gravity, local projection, and CPU integration are covered. |
| W13 — Standalone global ECEF propagation | MVP / partial | `earth_fixed_test`; rotating-Earth gravity, Coriolis, centrifugal terms, caller force, and deterministic RK4 are covered. |
| W14 — Kernel ECEF truth mode | MVP / partial | `ecef_kernel_test`; ECEF truth, geodetic atmosphere/ground handling, ECEF GPS/INS flow, and ellipsoid-clamped impact are covered. |

## 3. Subsystem fidelity assessment

| Area | Current implementation | Assessment and limit |
| --- | --- | --- |
| Translational truth | Body aero/thrust forces are rotated into the selected world frame, divided by current mass, and combined with gravity and configured earth terms. | **MVP / partial:** an engineering model; no validated coefficient-table or CFD-backed model. |
| Rotational truth | Diagonal body inertia, gyroscopic coupling, body angular rates, quaternion attitude, and bounded fin moments. | **MVP / partial:** no full inertia-tensor or flexible-body model. |
| Atmosphere | Layered ISA1976 atmosphere through 86 km, with density, pressure, temperature, and sound speed. | **Implemented for the stated envelope:** no weather or high-fidelity atmospheric model. |
| Aerodynamics | Air-relative drag, angle-of-attack lift, fin pitch lift, yaw-fin side force, stability, rate damping, and bounded moments. | **MVP / partial:** coefficients are simplified and not validated against tables, CFD, or wind-tunnel data. |
| Propulsion | Per-entity thrust curves, vacuum/sea-level Isp interpolation, mass flow, dry-mass limiting, and axial body +X thrust. | **MVP / partial:** no thrust vectoring, staged propulsion, or fuel-tank/engine failure model. |
| Actuators and control | World-to-body acceleration demand, bounded fin commands, first-order servo lag, rate limiting, and pitch/yaw sign conventions. | **MVP / partial:** fixed engineering gains; no gain scheduling, actuator failure, or advanced controller. |
| Integration | Derivative callbacks with true stage re-evaluation for RK4/RK45; bounded adaptive substeps; interpolated impact crossing. | **MVP / partial:** no multirate solver or complete event-aware adaptive policy. |
| Earth and frames | WGS84 conversion, normal and spherical gravity, ECEF/ENU/NED transforms, Coriolis, centrifugal, transport terms, standalone ECEF propagation, and opt-in kernel ECEF truth. | **MVP / partial:** no geoid, global terrain streaming, polar/dateline scenario policy, or complete earth-rate treatment across every subsystem. |
| Sensors | Body-frame IMU specific force and rates with noise/bias; noisy GPS in the selected frame. | **MVP / partial:** no coning/sculling compensation, lever arms, or full timing/interpolation contract. |
| Navigation | Perfect initial alignment, strapdown INS, and coupled 15-state error-state EKF with GPS position/velocity updates. | **MVP / partial:** earth-rate gyro compensation and advanced inertial error sources remain open. |
| Seekers | RF RCS/radar-range and IR irradiance/extinction models; FOV/gimbal limits, lock hysteresis, filtered LOS rates, latency, and friendly rejection. | **MVP / partial:** propagation is simplified and additional seeker families/phenomena are not implemented. |
| Guidance | Stateless PN/APN helpers, target velocity, waypoint mode, and seeker-lock APN handoff. | **MVP / partial:** no trajectory manager, pursuit, LQR/MPC, or blended handoff. |
| Events and terrain | Terrain/wind callbacks, geodetic/local terrain views, real impact deactivation, position clamping, and timestamped ground-impact events. | **MVP / partial:** no runtime DEM/DTED database, streaming, datum/geoid policy, or failure-event system. |
| Study wrappers and outputs | Single run, sweep, Monte Carlo, optimizer, and isolated batch runner. | **MVP / partial:** ECEF-aware reporting and a versioned telemetry/output contract are still needed; some wrappers remain primary-entity oriented. |
| Backends and packaging | Deterministic CPU/static library, CMake packaging, and optional Vulkan target. | **MVP / partial:** Vulkan parity is not validated, ECEF GPU support is open, and CUDA is not implemented. |

## 4. Quantitative validation evidence

- The complete Release CTest suite is green: **19/19 tests passed** at the
  checkpoint recorded above.
- The control regression reports a **25.30 m minimum miss** for its validated
  intercept scenario. This demonstrates the MVP control path; it is not a
  general accuracy guarantee.
- The test inventory covers study wrappers; vehicle configuration; rigid-body
  truth; intercept control; integration; seeker; navigation; environment;
  earth/frame/transport/gravity models; standalone ECEF propagation; kernel
  ECEF truth; guidance; and scenario loading.

The results establish regression coverage for the implemented paths. They do
not establish production-grade aerodynamics, global geophysics, sensor
calibration, or GPU equivalence.

## 5. Prioritized open work

These are the remaining fidelity and integration gaps, ordered by their value
to reliable downstream use:

1. **Frame-aware study reporting:** normalize altitude, coordinates, and
   summaries for ECEF scenarios in `SingleRun`, `ParamSweep`, `MonteCarlo`,
   and `BatchRunner`; define versioned telemetry fields.
2. **Global terrain:** add DEM/DTED ingestion, tile indexing and streaming,
   interpolation, datum/geoid policy, dateline/polar handling, and frame-aware
   collision queries. `tools/convert_srtm.cpp` is preparation, not runtime
   terrain support.
3. **Failure and damage semantics:** add motor, actuator, sensor, structural,
   and communications failures with deterministic state transitions and events.
4. **GNC fidelity:** add coning/sculling, lever arms, complete earth-rate gyro
   compensation, richer RF/IR propagation, and additional seeker types.
5. **Guidance and aero depth:** add validated coefficient tables, trajectory
   management, pursuit, LQR/MPC, and blended guidance handoff.
6. **Backend parity:** validate Vulkan against CPU truth, add GPU ECEF support,
   and implement CUDA only if a project requirement is established.
7. **Application handoffs:** define explicit versioned StrikeSim,
   StrikeDesigner, and StrikeCEM integration and provenance contracts.

## 6. Historical baseline

The pre-restart audit from 2026-08-25 recorded a 5/7 workstream result and
identified failures in control signs, aerodynamic authority, integration,
events, seeker fidelity, and navigation. Those measurements described the
older implementation and are superseded by the W1–W14 verification above.

The historical source documents and commits remain available in repository
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
