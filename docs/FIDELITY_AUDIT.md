# StrikeEngine — Fidelity Audit (2026-08-25; verified 2026-08-26)

Baseline audit before the engine-fidelity deep session. Every claim below is
traceable to the code at commits `48775a6` (engine) / `d82f851` (StrikeSim).
Per-subsystem: what is modeled, what is crude, what is missing. Evidence in
`(file:line)` form.

> ## Post-session status (same day, end of deep session) — measured
>
> Session goal: W1+W2+W3 spine (per-entity config → 6-DOF rigid body → control
> authority) proven by `ctest`. **Result: spine implemented and committed, DoD
> NOT met.** Full suite: 5/7 pass, 2 red. Evidence is test output, not narration.
>
> | Workstream | Status | Measured evidence |
> | --- | --- | --- |
> | W1 per-entity vehicle config | DONE (`7018693`) | `vehicleconfig_test` PASS: target coasts 70 m/s, mass constant 100 kg; missile 500 → 390 kg, floors at dry 370 kg (burnout honored). `VehicleConfig` is public API. |
> | W2 6-DOF rigid body | DONE (`7018693`) | `rigidbody_test` PARTIAL: energy drift 2.1e-4, \|q\| error 2.2e-16, gyroscopic coupling PASS (wx drifts from initial). **RED: nose-up → world-climb assertion fails (alt change −2.8 m)** — end-to-end sign chain still broken. |
> | W3 control authority | PARTIAL (`0f8c3e3`) | Fin lift from deflection/AoA, rate-command loop, α/β damping, servo lag + rate limit all implemented and committed. **DoD RED: `intercept_test` min miss 2978 m** (was ~1281 m mid-session with older gains; both fail). Failure mode: violent pitch+yaw limit cycle ~t=2 s at high dynamic pressure; missile dives through ground (z→−1937). Thrust ruled out by isolation run (coasting missile, no motor → same instability). Root cause unconfirmed; outer-loop body-frame sign chain is the prime suspect. |
> | W4 true RK4/RK45 | PARTIAL (`0f8c3e3`) | Derivative-callback API + true stage re-eval (RK4, RK45 Fehlberg 4(5) w/ error control + step halving, Euler, Symplectic) done. Remaining W4 scope: adaptive step control policies + impact-time interpolation — not started. |
> | W5 events/environment | NOT STARTED | Ghosts confirmed still present: intercept run ends at z = −1937 (ground hit at z=0 set `isAlive=false`, `physics.active` stayed true). |
> | W6 seeker/sensor | NOT STARTED | — |
> | W7 navigation EKF | NOT STARTED | — |
>
> Audit items fixed by this session (baseline sections below are now stale on
> these points): CL=0 and fin-force gap (W3 lift term, `AeroModel.hpp`);
> fake-RK4/constant-accel integration (integrator refactor); world-frame
> rotational placeholder (body-frame Euler, `CPUBackend.cpp`); shared 50 kN/5 s
> motor (per-entity propulsion pool); per-entity ref area/drag (`PhysicsBlock`);
> nav estQ/estP/estV never aligned from truth (`NavigationSystem.cpp`).
>
> Still open audit-critical items (baseline sections remain accurate): sign
> convention unverified (`AutopilotSystem.cpp`, comment in §3 — now RED in
> `rigidbody_test`), actuator chain exists but unstable at speed, ghosts (§8),
> fixed-gain autopilot (no gain scheduling by dynamic pressure), seeker FOV/
> latency (§6), EKF (§7), impact interpolation (§9).

## Current verification after restart

The restart fixes were completed and re-run on 2026-08-26. The complete CTest
suite is green: **14/14 tests passed** after adding navigation, environment,
earth, guidance, and scenario regressions.

| Workstream | Current status | Evidence |
| --- | --- | --- |
| W1 per-entity vehicle config | DONE | `vehicleconfig_test` PASS |
| W2 6-DOF rigid body | DONE | `rigidbody_test` PASS: quaternion norm, gyro coupling, and commanded climb |
| W3 control authority | DONE for the MVP DoD | `intercept_test` PASS: minimum miss 25.30 m; actuator and post-burnout control path exercised |
| W4 true RK4/RK45 | DONE for the integrator/event MVP | Derivative callbacks, stage re-evaluation, bounded RK45 adaptation, and interpolated ground-crossing timestamps are covered by `integrator_test` |
| W5 events/environment | DONE for the environment MVP | Terrain elevation callbacks, wind-relative aerodynamics, and interpolated terrain impacts are covered by `environment_test`; failure models and spherical earth remain |
| W6 seeker/sensor | DONE for the seeker MVP | FOV cone, gimbal limits, lock hysteresis/dropout, filtered LOS rates, and configurable measurement latency are covered by `seeker_test` |
| W7 navigation EKF | DONE for the navigation MVP | Full 15-state covariance propagation, coupled GPS corrections into attitude and IMU biases, covariance bounds, and deterministic accelerometer-bias convergence are covered by `navigation_test` |
| W8 scenario/guidance contract | DONE for the integration MVP | Explicit PN/APN behavior, moving-target response, seeker handoff, scenario configuration propagation, and isolated batch execution are covered by `guidance_test` and `scenario_test` |
| W9 earth model | DONE for the opt-in local-earth MVP | WGS84 geodetic/ECEF conversion, latitude/altitude-dependent normal gravity, and local ENU Coriolis acceleration are covered by `earth_test` |

The W3 repair uses a bounded acceleration-command autopilot with gravity-aware
specific-force conversion, body-rate/AoA damping, and corrected yaw-fin force
signs. W8 now supplies explicit classical PN from relative position/velocity
and filtered-rate seeker APN behavior; trajectory management and additional
guidance laws remain future work.

The W5 repair changes ground impact from a status-only notification to a true
state transition: position is clamped to ground, velocity/acceleration are
cleared, and `physics.active` is disabled. This removes the previously observed
below-ground integration ghosts. Impact timestamps use the interpolated ground
crossing within the final integration step.

The W7 navigation repair adds a row-major full covariance for the 15-state
error model (position, velocity, attitude, accelerometer bias, and gyro bias).
Strapdown propagation carries position/velocity/attitude/bias coupling and
sensor-configured process noise; sequential GPS position/velocity updates apply
the resulting correction to all coupled states while keeping covariance values
finite and bounded. The bias regression constrains the known initial level
attitude so it measures accelerometer-bias observability independently of the
stationary tilt/bias ambiguity.

The W5 environment increment adds public terrain and wind callbacks to the
kernel. Terrain is evaluated at the current and previous positions for local
ground crossing and clamping; wind is evaluated in the truth derivative and
subtracted from world velocity before body-frame aerodynamic forces are formed.

The W8 integration increment makes `ScenarioConfig` carry per-vehicle and
environment configuration, including guidance demand limits, and adds a
structured `BatchRunner` result API. Guidance math is exposed as stateless PN
and APN model helpers while the kernel retains seeker-lock handoff behavior.

The W9 earth increment adds an opt-in `EnvironmentConfig::earth` block. It
provides WGS84 geodetic/ECEF conversion helpers, Somigliana normal gravity with
altitude correction, and local ENU Coriolis acceleration in the CPU truth
backend. The legacy flat-earth gravity and no-Coriolis behavior remain the
default; full earth-fixed transport/centrifugal dynamics and standalone frame
modules remain future work.

---

## 1. Truth dynamics — translational

| Aspect | State | Fidelity |
| ------ | ----- | -------- |
| Gravity | Flat-earth constant by default; opt-in WGS84 normal gravity with altitude correction | GOOD for the local-earth MVP; full earth-fixed gravity/frame coupling remains pending |
| Drag | `CD = 0.3` constant, `S = 0.1 m²` hardcoded, force along velocity only | LOW |
| Lift | **`CL = 0.0`** in MVP kernel config — zero aerodynamic lift | **CRITICAL GAP** |
| Atmosphere | ISA1976 layered (T/P/ρ/a), clamped to 86 km | GOOD |
| Integration | "RK4" but **all four stages reuse the same precomputed acceleration** (`k2_vx = ax`, `k3_vx = ax`, `k4_vx = ax`) — constant-acceleration stepping; effectively Euler-in-drag | CRITICAL GAP |
| Per-entity body | Reference area, drag, thrust curve shared by **all** entities via one backend model set | LOW |

Evidence: `BasicAeroModel` `(0.3, 0.0)` in `SimulationKernel.cpp:24`; `S=0.1`
hardcoded `CPUBackend.cpp:80`; stage reuse `RK4Integrator.cpp` (k2..k4 = k1).

## 2. Truth dynamics — rotational (6-DOF)

| Aspect | State | Fidelity |
| ------ | ----- | -------- |
| Equations | `alpha = torque_world / I_diag` — **uncoupled world-frame**, labeled "MVP placeholder until full tensor rotation" | LOW |
| Inertia tensor | Diagonal only (`Ixx/Iyy/Izz` scalars in block); no body-frame tensor, no gyroscopic `ω × (I·ω)` | LOW |
| Rotation integration | Explicit Euler for `ω` (`RK4Integrator.cpp`: "Angular velocity integration (Euler for MVP)") | LOW |
| Quaternion | `q += 0.5·(q⊗ω)·dt` + normalize — adequate at 100 Hz | MID |
| Fin torques | `CM_delta=1.5`, `Cl_delta=0.5` per rad, ∝ q·S — modeled | MID |
| **Fin forces** | Deflections produce torque only, **no side force** → no lift → no post-burnout control | **CRITICAL GAP** |

## 3. Control (autopilot)

| Aspect | State | Fidelity |
| ------ | ----- | -------- |
| Structure | P-D: `deflection = k_p·accel_body − k_d·rate`, clamped ±0.43 rad | MID |
| Sign convention | Code comment admits uncertainty ("Body Z is usually Down or Up… depends on convention") — pitch/yaw channel mapping unverified | **CRITICAL GAP** |
| Actuators | No servo lag/rate limit — deflection applied instantly | LOW |
| Thrust command | `control.thrustCommand` exists but is never used; motor runs the fixed shared curve, no throttle | LOW |
| Net authority | Steering works **only while thrust misaligns with velocity**; after 5 s burn vehicles are unguided rocks. Measured: guided ≈ ballistic (min miss ~475 m head-on) | **CRITICAL GAP** |

Evidence: `AutopilotSystem.cpp:67-71` convention comment; measured in the
Phase 3.3 headless probes (`/tmp` harness, guided vs ballistic within 5 m).

## 4. Propulsion

| Aspect | State | Fidelity |
| ------ | ----- | -------- |
| Model | Thrust curve + vacuum/SL Isp interpolation + mass flow | MID |
| Per entity | **One shared 50 kN / 5 s curve for every vehicle** — target "drones" also boost at 500 m/s² | LOW |
| Mass budget | Mass drops by flow rate, floor 1.0 kg — no fuel quantity, no burnout mass from config; a 100 kg target ends at ~1 kg | LOW |
| Vectoring | Thrust fixed along body X — vectored by rotating the body | MID |

Evidence: shared curve `SimulationKernel.cpp:26-36`; mass floor `CPUBackend.cpp:92-93`.

## 5. Guidance

| Aspect | State | Fidelity |
| ------ | ----- | -------- |
| ProNav | True LOS-rate form `ω = r×ṙ/|r|²`, `N=3`, closing velocity from relative velocity — good math | GOOD |
| Aimpoint | `targetX/Y/Z` are the **initial commanded values, never updated**; |r| uses stale geometry (OK for constant-V targets when |ṙ| is exact, approximation otherwise) | MID |
| Target velocity | Now routable (`d84108d`) — previously hardcoded 0 | GOOD |
| APN/seeker path | Pseudo-LOS-rate = `5 × angle` (steering-to-zero approximation), not a true rate estimator; N=3.5 | LOW |
| Handoff | Binary swap PN → seeker-APN on lock; no blend, no dropout hysteresis | LOW |

## 6. Sensors

| Aspect | State | Fidelity |
| ------ | ----- | -------- |
| IMU | Gaussian noise + random-walk bias; specific-force and gyro in body frame | MID |
| GPS | Position/velocity at uniform rate (default ~1–10 Hz?) with noise | MID |
| Seeker RF | Radar range equation vs RCS profile DB; SNR threshold lock | GOOD (conceptually) |
| Seeker IR | Irradiance + Beer-Lambert placeholder extinction | MID |
| Lock logic | **No FOV cone, no gimbal limits, no hysteresis** — seeker can lock targets behind it; lock chatters at threshold | LOW |
| Timing | Measurements at every physics tick (IMU); GPS uniform; **zero latency** to guidance | LOW |

## 7. Navigation

| Aspect | State | Fidelity |
| ------ | ----- | -------- |
| INS | Strapdown integration, perfect initial alignment | MID |
| EKF | **Diagonal-only** P; bias correction via fixed `0.001` gain (biases effectively unobservable); covariance grows unboundedly (`+0.001·dt` every tick) | LOW |

Evidence: `NavigationSystem.cpp` strapdown + scalar-gain EKF.

## 8. Events & environment

| Aspect | State | Fidelity |
| ------ | ----- | -------- |
| Ground | z ≤ 0 → `isAlive=false` + event | LOW |
| Ghosts | **Ground-hit entities keep integrating**: `status.isAlive` set false but `physics.active` stays true, no removal/freeze — missiles dive to −9 km | **CRITICAL GAP** |
| Impact time | Step-granular (up to ±dt error); no crossing interpolation | LOW |
| Other | No terrain, no wind/gusts, no failure models, no telemetry conditions | — |

Evidence: dead-but-integrated ghosts observed in headless probes; `EventSystem.cpp`.

## 9. Integration architecture

| Aspect | State | Fidelity |
| ------ | ----- | -------- |
| API | `Integrator::integrate(physics, dt)` with precomputed accel — **no derivative callback**, so true RK4/RK45 stage re-evaluation is impossible without API change | CRITICAL GAP (architectural) |
| RK45/HybridScheduler | Adaptive path exists but integrates the same constant accel; "hybrid" is a substep loop, not stiffness switching | LOW |
| Event accuracy | All events sampled at step boundaries | LOW |

## 10. What is actually good (keep)

- Clean SoA blocks + systems pipeline (`truth → sensors → nav → seeker → guidance → autopilot → events`)
- ISA1976 layered atmosphere
- RCS/IR signature databases wired into the seeker physically (range equation)
- ProNav math, PSO/UQ tooling, package discipline (Phase 1), StrikeSim integration surface
- The guidance target-velocity fix landed during Phase 3.3 is correct and additive

---

## Measured consequence (why this matters)

Headless probes with the current engine: guided and ballistic trajectories
differ by < 5 m; best achievable miss ~475 m even dead-ahead with a heavy
stationary target. The causal chain: `CL=0` → fins can't produce force →
steering exists only while the 5 s shared motor burns → after burnout the
vehicle cannot respond to any command; during burn the rate-dominated
autopilot oscillates. **Nothing in the guidance stack can be validated until
the control-authority chain is fixed** — which is exactly what the deep
session should start with.

## Proposed workstreams (each = one engine feature, sized like a PR)

| # | Workstream | Fixes | DoD (measurable) |
| - | ---------- | ----- | ---------------- |
| W1 | **Per-entity vehicle config** | Shared models → per-vehicle mass props, ref area, aero coeffs, thrust curve, fuel mass | Targets stop boosting: drone coasts 80 m/s; mass stays sane; `VehicleConfig` in the public API |
| W2 | **6-DOF rigid body** | Body-frame Euler equations, inertia tensor rotation, gyroscopic coupling, body-frame angular integration | Tumbling disappears; angular momentum behavior sane in tests |
| W3 | **Control authority** | Fin lift from deflection & AoA, rate-command loop, verified pitch/yaw signs, actuator lag | Headless: guided head-on intercept < 50 m miss; authority persists post-burnout |
| W4 | **True RK4/RK45** | Derivative-callback API, stage re-evaluation, adaptive step control, impact-time crossing interpolation | Step-size rejection works; energy/no-drift checks; impact times exact |
| W5 | **Events & environment** | Ground freeze/removal (no ghosts), terrain/wind, optional spherical earth | No entities below z=0 in any test; wind mixes into truth |
| W6 | **Seeker/sensor fidelity** | FOV cone, gimbal limits, lock hysteresis, true LOS-rate estimation for APN, measurement latency | No locks behind seeker; APN uses filtered rate; latencies modeled |
| W7 | **Navigation EKF** | Coupled 15-state corrections, bounded covariance, bias observability | Bias estimates converge to truth in a test |

Recommended order: **W1 → W2+W3 → W4 → W5 → W6 → W7**. W1–W3 are the spine:
they make intercepts physically possible and observable in the StrikeSim
Simulate surface (reinstall engine per workstream, demo in-app).
