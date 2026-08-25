# StrikeEngine — Fidelity Audit (2026-08-25)

Baseline audit before the engine-fidelity deep session. Every claim below is
traceable to the code at commits `48775a6` (engine) / `d82f851` (StrikeSim).
Per-subsystem: what is modeled, what is crude, what is missing. Evidence in
`(file:line)` form.

---

## 1. Truth dynamics — translational

| Aspect | State | Fidelity |
| ------ | ----- | -------- |
| Gravity | Flat-earth constant `-9.80665` | MID (fine for short-range MVP; no earth rotation/Coriolis, no altitude dependence) |
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
