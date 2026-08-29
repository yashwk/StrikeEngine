# Seeker-Guided Missile Intercept Test

## Purpose

Add a deterministic end-to-end regression that proves the engine can perform a
seeker-guided missile engagement:

```text
midcourse command -> navigation/guidance -> seeker acquisition -> terminal APN
                  -> autopilot -> fins/aero/propulsion -> proximity detonation
```

This test is needed because the existing `intercept_test` validates pure PN
guidance only. It does not configure a seeker, test seeker lock, or exercise
the terminal seeker-APN handoff.

## Non-goals

- Do not replace or weaken the existing pure-PN `intercept_test`.
- Do not change `data/scenarios/intercept_test_01.json` to make this test pass.
- Do not tune the scenario around a single miss-distance threshold without
  first diagnosing seeker, guidance, autopilot, and propulsion behavior.
- Do not claim this test validates a full operational missile model. It is an
  engine integration regression at MVP fidelity.

## Deliverable

Add a new test named `seeker_intercept_test`:

```text
tests/validation/seeker_intercept_test.cpp
```

Register it in:

```text
tests/CMakeLists.txt
```

The test must run in both normal and GDAL-enabled builds. It should not depend
on terrain or GDAL.

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

## Implementation notes from the current engine

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

The guidance priority is:

1. Communication failure: zero commanded acceleration.
2. Valid seeker lock: terminal seeker APN overrides the configured mode.
3. Otherwise use `None`, PN, or Waypoint mode.

Terminal APN uses filtered seeker azimuth/elevation rates in the airframe
convention:

```text
X = forward
Y = right
Z = down

body-Y command =  N * closing_speed * azimuth_rate
body-Z command = -N * closing_speed * elevation_rate
```

The body-frame command is rotated into world coordinates using the navigation
attitude estimate and then limited by `maxAccel`.

The autopilot converts world-frame acceleration into bounded fin commands. The
requested guidance acceleration is therefore not guaranteed to equal the
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
