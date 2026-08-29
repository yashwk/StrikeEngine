# Guidance System Improvement Specification

## Purpose

Improve the complete StrikeEngine guidance system from a mostly stateless
acceleration-command path into a traceable, frame-correct, mode-aware guidance
stack. The seeker-guided missile intercept is one validation scenario for this
work; it is not the whole deliverable.

```text
command / target track -> guidance state and mode manager
                       -> midcourse, terminal, or blended guidance law
                       -> bounded world-frame acceleration
                       -> autopilot / actuator interface
                       -> vehicle dynamics and achieved response
```

The implementation agent MUST diagnose failures before changing scenario
geometry, acceleration limits, warhead radii, or other test parameters.

## Non-goals

- Do not replace or weaken the existing pure-PN `intercept_test`.
- Do not change `data/scenarios/intercept_test_01.json` to hide a guidance
  defect.
- Do not claim this work validates a full operational missile model.
- Do not add terrain, GDAL, or a new propulsion model as a prerequisite.

## Deliverables

The primary deliverable is the guidance-system implementation, organized into
the phases in this document. It includes guidance state, explicit law and phase
selection, seeker handoff/track retention, frame correctness, limits, and
diagnostics.

The integrated validation artifact is a new test named `seeker_intercept_test`:

```text
tests/validation/seeker_intercept_test.cpp
```

Register it in:

```text
tests/CMakeLists.txt
```

The test must run in both normal and GDAL-enabled builds. It should not depend
on terrain or GDAL. It validates the implementation; it is not a substitute
for the implementation.

## Current baseline and required direction

The current implementation is in:

- `src/strikeengine/kernel/systems/GuidanceSystem.cpp`
- `src/strikeengine/kernel/systems/AutopilotSystem.cpp`
- `src/strikeengine/kernel/systems/SeekerSystem.cpp`
- `include/strikeengine/kernel/data/GuidanceBlock.hpp`

The current guidance priority is:

1. Communication failure: zero acceleration.
2. Valid seeker lock: terminal seeker APN overrides the configured mode.
3. Otherwise: `None`, PN, or Waypoint mode.

Existing MVP behavior must remain available:

- PN using target position and velocity
- Waypoint point-seeking
- Seeker LOS-rate APN
- Per-entity navigation constant and waypoint gain
- Per-entity acceleration limit
- World-to-body autopilot conversion

The improvement must address these limitations:

- No explicit persistent target-track or guidance-phase state
- Abrupt seeker acquisition handoff
- No bounded short-term lock-loss retention
- No clear distinction between guidance demand and achieved vehicle response
- Limited diagnostics for invalid geometry, non-closing targets, or saturation
- No explicit interface for future pursuit, trajectory, energy, LQR, or MPC laws

## Guidance system requirements

### Inputs and state

Guidance MUST consume clearly identified inputs:

- Navigation estimate and attitude
- External target command or persistent target track
- Seeker track, quality, lock state, and LOS rates
- Vehicle alive, failure, and communications state
- Per-vehicle guidance configuration
- Timestep and measurement timestamps where available

Add or organize per-vehicle state for:

- Active phase: `None`, `Midcourse`, `Acquisition`, `Terminal`, or `LostTrack`
- Active guidance law
- Target identity, when known
- Track age and last valid measurement time
- Seeker handoff/blend weight
- Raw and limited acceleration demands
- Limit/saturation flags
- Invalid-input and non-closing flags

State MUST reset correctly when vehicles are created, destroyed, or slots are
reused.

### Outputs and diagnostics

Guidance MUST produce a finite, bounded world-frame acceleration command and
diagnostics containing, at minimum:

- Commanded `ax`, `ay`, `az` and magnitude
- Whether `maxAccel` limited the command
- Whether the selected law was invalid or non-closing
- Active phase, law, and target identity
- Handoff weight and track age when seeker guidance is involved

Diagnostics should distinguish guidance demand limiting from downstream
autopilot, servo, fin, or aerodynamic-authority limits.

### Frame contract

All guidance code MUST use:

```text
Body X: forward
Body Y: right
Body Z: down
```

Relative vectors use target-minus-interceptor convention:

```text
r = target_position − interceptor_position
v = target_velocity − interceptor_velocity
```

Positive seeker azimuth rate maps to positive body-Y acceleration. Positive
elevation rate maps to negative body-Z acceleration. Frame conversions MUST be
explicit and covered by axis/sign tests.

Guidance produces total world-frame acceleration demand. Gravity compensation
MUST happen exactly once in the autopilot/control path.

## Guidance-law requirements

### Proportional navigation

PN MUST use the configured navigation constant, target-relative position and
velocity, and a safe closing-speed calculation. It must return zero or an
explicit invalid status for zero range, non-closing geometry, non-finite input,
or non-positive navigation constant.

The current mathematical form is:

```text
Vc = −dot(r, v) / |r|
ω  = cross(r, v) / |r|²
a  = N × Vc × cross(ω, r_hat)
```

The output must be normal to the line of sight and tested independently of
vehicle dynamics.

### Seeker APN

Seeker APN MUST use filtered LOS rates and closing speed, with this body-frame
mapping:

```text
body-Y =  N × closing_speed × azimuth_rate
body-Z = −N × closing_speed × elevation_rate
body-X =  0
```

The command must then be rotated into the world frame using navigation
attitude and bounded by `maxAccel`.

The implementation should clearly distinguish this seeker-rate APN from APN
with target-acceleration feed-forward. If target acceleration is unavailable,
the behavior must be explicit rather than reading an uninitialized value.

### Waypoint and future laws

Waypoint guidance remains a backward-compatible point-seeking law. Future
pursuit, trajectory, energy-management, LQR, or MPC laws must be separate
explicit laws or managers, not hidden changes to waypoint behavior.

## Phase selection and seeker handoff

Replace the current abrupt lock override with an explicit, observable phase
manager:

```text
No guidance -> Midcourse PN -> Acquisition blend -> Terminal APN
                         ↘ LostTrack recovery -> Midcourse PN
```

Requirements:

- Midcourse PN remains active until a valid seeker track exists.
- Terminal APN weight ramps from 0 to 1 over a configured interval or quality
  threshold.
- Acceleration remains bounded throughout the transition.
- A LOS-rate discontinuity is rate-limited or diagnosed.
- Short permitted seeker dropouts retain target identity and timestamp.
- Stale LOS rates cannot be used indefinitely.
- After retention expiry, phase becomes `LostTrack` and recovery is explicit.

The first blend may be linear or critically damped, but it must be
deterministic and configuration-backed.

## Autopilot integration requirements

The autopilot remains downstream of guidance and MUST:

- Consume world-frame total acceleration demand.
- Convert to body frame exactly once.
- Account for gravity exactly once.
- Apply acceleration, rate, AoA, fin, servo, and roll limits.
- Preserve documented pitch/yaw signs.
- Expose commanded fin deflection and, where practical, achieved acceleration.

The implementation must make it possible to tell whether a miss came from an
incorrect guidance command, autopilot saturation, servo-rate limitation, fin
authority, or aerodynamics.

## Implementation phases

### G0 — Baseline and observability

- Preserve pure-PN `intercept_test`.
- Add phase, law, target, command, lock, handoff, and saturation diagnostics.
- Establish a baseline without changing scenario geometry or limits.

### G1 — Explicit guidance state and law interface

- Add per-vehicle phase and track state.
- Separate phase selection from law calculation.
- Make invalid and non-closing outputs explicit.
- Preserve existing public behavior where no new policy is configured.

### G2 — Handoff and track retention

- Implement acquisition-to-terminal blending.
- Implement bounded short-term dropout retention.
- Implement lost-track recovery and transition diagnostics.

### G3 — Guidance-law fidelity

- Validate PN for head-on, crossing, and non-closing geometries.
- Add APN target-acceleration feed-forward with explicit availability semantics.
- Add time-to-go or predicted-intercept diagnostics where numerically sound.

### G4 — Autopilot coupling

- Report guidance demand versus achieved response.
- Verify acceleration, rate, AoA, fin, servo, and aero saturation paths.
- Add achieved-acceleration feedback only after checking control-loop stability.

### G5 — Integrated validation

- Add the two-missile seeker intercept regression below.
- Add moving-target and controlled maneuvering-target cases.
- Compare truth, navigation estimate, seeker track, guidance demand, actuator
  output, and achieved acceleration.
- Update authoritative project documents with measured evidence.

## Engagement setup

Create exactly two explicit missile entities.

### Interceptor

The interceptor must have:

- `EntityType::Missile`
- Friendly allegiance
- Non-zero initial forward velocity
- Propulsion with a finite fuel supply
- Aerodynamic body and fins
- A configured RF seeker
- A proximity warhead
- Guidance/autopilot gains
- A finite guidance acceleration limit

The RF seeker must have enough range and field of view for the chosen
geometry. Its mechanical gimbal and field-of-view limits must remain realistic;
do not solve acquisition by making them unlimited.

### Target

The target must have:

- `EntityType::Missile` explicitly set, rather than relying on the default
- Hostile allegiance
- A non-empty RCS profile ID
- Its own initial velocity
- No seeker requirement
- No target guidance requirement for the first version

The target may be coasting for the first implementation. A later test may add
propulsion or a scripted maneuver, but the initial regression should isolate
the interceptor’s seeker and terminal guidance path.

The existing `data/rcs/target_drone_rcs.json` may be reused initially if its
semantics are suitable. If a missile-specific signature is needed, add a
clearly named profile such as `data/rcs/target_missile_rcs.json` rather than
silently relying on the drone filename.

## Guidance sequence

The test must exercise both guidance phases.

### Phase 1: midcourse

Queue a `SimulationCommand` for the interceptor with:

- `GuidanceMode::ProportionalNavigation`
- Target position equal to the target’s initial position
- Target velocity equal to the target’s initial velocity
- A finite `maxAccel`

The target position and velocity must be explicit. A moving target must not be
treated as a stationary aimpoint.

### Phase 2: terminal homing

Allow the seeker to acquire the hostile target. Once locked, the normal engine
path must take over:

```text
SeekerSystem -> filtered LOS rates -> GuidanceSystem seeker APN
             -> world-frame acceleration -> AutopilotSystem -> controls
```

Do not call private guidance functions directly or inject a fake lock into the
kernel. The lock must result from seeker geometry, signal strength, allegiance,
and the target RCS profile.

## Required assertions

The test must assert all of the following:

1. Exactly two entities are created.
2. Both entities are explicitly missiles.
3. The interceptor is friendly and the target is hostile.
4. The interceptor has a non-`None` seeker.
5. The target has a non-empty RCS profile.
6. The seeker locks the hostile target before closest approach.
7. The seeker does not lock a friendly decoy if one is used in an extension.
8. Terminal APN generates a non-zero commanded acceleration after lock.
9. The missile remains numerically stable and respects configured speed and
   acceleration sanity limits.
10. Closest approach is within the configured proximity trigger radius.
11. A detonation event is emitted.
12. The target is no longer alive after the proximity event.
13. The detonation/kill time is close to the measured closest-approach time.

The test should print diagnostic values on failure:

- Lock time and lock range
- Closest-approach distance and time
- Maximum interceptor speed
- Maximum commanded acceleration
- Number of seeker lock losses
- Final interceptor and target positions
- Whether detonation and target structural-failure events occurred

## Determinism

Use a fixed kernel random seed. Use a fixed timestep, initially:

```cpp
constexpr double dt = 0.01;
```

The test must pass repeatedly with identical reported lock and miss metrics.
If sensor or seeker noise is enabled, keep the seed fixed and document the
noise configuration.

Do not assert an overly precise miss distance. Assert the physical requirement
needed by the warhead, and separately report the measured miss distance. A
reasonable first acceptance criterion is:

```text
closest approach < proximity_trigger_m
```

The exact distance should be recorded as evidence, not used as an unexplained
tuning target.

## Recommended geometry

Choose a head-on or shallow crossing engagement that satisfies all of these:

- Initial target is outside or near the edge of seeker acquisition range so
  midcourse guidance is genuinely exercised.
- PN brings the target into seeker field of view.
- Target remains inside the seeker gimbal limits after acquisition.
- The target does not start inside the lethal radius.
- The interceptor has enough energy and aerodynamic authority to turn.
- The target does not immediately hit the ground before engagement.

Start with a non-maneuvering moving target. Once this passes, add a separate
maneuvering-target test rather than making the first test ambiguous.

## Runtime integration reference

The simulation step order is:

```text
commands
-> truth physics
-> sensors
-> navigation
-> seekers
-> guidance
-> autopilot
-> events/warhead
```

This is implemented in `SimulationKernel::step()`.

Guidance and autopilot run after the physics update, so a newly generated
control demand is applied by the backend on the following simulation step.
The requested guidance acceleration is therefore not guaranteed to equal the
actual vehicle acceleration.

## Failure diagnosis order

If the test fails, classify the failure before changing parameters:

| Symptom | Investigate |
| --- | --- |
| No lock | RCS profile, allegiance, signal equation, FOV, gimbal, range |
| Lock occurs too late | Midcourse PN, seeker range, target geometry, velocity signs |
| Lock immediately drops | APN frame signs, gimbal/FOV, seeker dropout behavior |
| Commanded acceleration is wrong | LOS-rate signs, body-frame mapping, navigation constant |
| Command is correct but vehicle misses | Autopilot signs, fin authority, servo limits, aero tables |
| Vehicle turns but loses energy | Drag, propulsion staging, mass flow, acceleration limit |
| Detonation absent | Proximity event ordering, warhead radius, target alive state |
| Different result per run | Random seed, uninitialized state, unordered target selection |

Do not alter target position, velocity, acceleration limits, or warhead radius
until the failing subsystem has been identified and a diagnostic has been
added.

## Test layering

The final test suite should make the responsibilities clear:

```text
guidance_test          mathematical PN/APN and body-frame signs
seeker_test            seeker detection, lock, dropout, and LOS rates
intercept_test         pure-PN intercept without seeker
seeker_intercept_test  missile + missile seeker-guided end-to-end intercept
designer_pipeline_test profile/manifests + RF seeker + pipeline intercept
```

The new test complements the existing tests; it must not remove coverage from
the pure-PN path.

## Documentation and validation

After implementation:

1. Update `docs/SPEC.md` only if the public contract changes.
2. Update `docs/IMPLEMENTATION.md` with the new test and measured evidence.
3. Update `docs/FIDELITY_AUDIT.md` with a new milestone or evidence entry.
4. Run `git diff --check`.
5. Build and run the complete CTest suite without GDAL.
6. Build and run the complete CTest suite with GDAL enabled.
7. Repeat the new test at least twice and confirm deterministic output.
8. Commit the implementation, tests, and documentation together.

## Definition of done

This work is complete only when a fresh checkout can run
`seeker_intercept_test` and demonstrate, from real configured entities:

```text
two explicit missiles
-> hostile missile RCS detection
-> seeker lock
-> terminal APN handoff
-> bounded autopilot control
-> proximity detonation
-> target kill
```

The original scenario and pure-PN regression must remain unchanged unless a
separate, evidence-backed defect requires a documented change.
