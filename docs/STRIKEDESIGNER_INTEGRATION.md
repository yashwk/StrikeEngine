# StrikeDesigner Integration

**Status:** designer handoff contract for StrikeEngine
**Companion specification:** [`SPEC.md`](SPEC.md) (normative),
[`IMPLEMENTATION.md`](IMPLEMENTATION.md) (implementation record)

This document defines the design-time handoff between **StrikeDesigner** (the
design-time authoring application, intended to be built inside StrikeSim on the
stabilized StrikeEngine package) and the StrikeEngine runtime. It consolidates
the design-interchange contract that is otherwise spread across
[`SPEC.md`](SPEC.md) §5.5 and the subsystem sections.

## 1. Roles and boundaries

```text
StrikeDesigner (design intent, geometry, manifests)
    -> StrikeCEM   (RCS signature tables; separate project, strikecem_lib)
    -> StrikeCFD   (aero cd/cl(/cm/cy) tables; separate project/folder)
    -> StrikeEngine consumes the handoff artifacts and simulates
```

- **StrikeDesigner owns design intent**: vehicle identity, geometry, subsystem
  selection, and the derived physics summary. It owns design-time validation and
  provenance; StrikeEngine owns runtime truth.
- StrikeEngine is library-only. It never edits designs, never shows UI, and
  never writes files except through the explicit serialization API.
- StrikeCEM (offline CLI + `strikecem_lib`) and StrikeCFD (separate project)
  produce signature/aero coefficient tables that the designer attaches to a
  design by profile id. StrikeEngine never computes them.

## 2. Handoff artifacts

The designer produces three artifact classes, all JSON, all validated
fail-fast by the engine:

| Artifact | Location (convention) | Consumed by |
| --- | --- | --- |
| Design manifest | `data/profiles/<design>.json` | `ScenarioConfig` `design_ref` -> `loadDesignPhysics` |
| Subsystem profiles | `data/aero`, `data/motors`, `data/seekers`, `data/sensors`, `data/rcs` | profile-id resolution at `createVehicle` |
| Scenario | `data/scenarios/*.json` | `ScenarioConfig::load` + `loadInto(kernel)` |

### 2.1 Design manifest

`{"name", "description"?, "geometry"?, "physics": <VehicleConfig>}`.

- `physics` IS the full `VehicleConfig` in snake_case (see §5.5 of the SPEC and
  §3 below). `geometry` is an opaque designer-owned blob the engine passes
  through (`serializeDesign(name, geometryJson, physics)`) and ignores.
- `loadDesignPhysics(path)` reads `physics` back into a `VehicleConfig`.
- A non-empty `ScenarioEntityConfig::design_ref` is resolved on scenario load
  and OVERRIDES any inline `vehicleConfig` (designer wins, same precedent as
  profile ids winning over inline sub-configs).
- Relative paths inside a manifest (profile ids) are engine-literal paths; a
  consuming application SHOULD resolve them against its own data root.

### 2.2 Subsystem profile ids (authoritative at `createVehicle`)

A manifest's `physics.aeroProfileId`, `motorProfileId`, `seekerProfileId`,
`sensorProfileId` (plus `rcsProfileId`/`irProfileId` on the entity status)
reference files under `data/{aero,motors,seekers,sensors,rcs}`. On
`createVehicle` the referenced profile REPLACES the corresponding inline
sub-config (placeholders must still exist because `VehicleConfig::from_json`
requires every sub-object key). Profile load failures are fatal and fail fast.

| Subsystem | File (example) | Designer-owned content |
| --- | --- | --- |
| Aero | `data/aero/*.json` | `reference_area`, `reference_length`, `cd`, `cl_alpha`, `cl_fin`, `cl_max`, optional `aero_tables` grid (Mach x AoA; moment/lateral tables per SPEC §6.2), optional `fins` |
| Motor | `data/motors/*.json` | `stages[]` (thrust curve, sea-level/vacuum Isp, propellant/dry mass); multi-stage staging |
| Seeker | `data/seekers/*.json` | type + RF/IR params, FOV/gimbal half-angles, hysteresis, dropout, latency |
| Sensor | `data/sensors/*.json` | IMU/GPS enablement, noise/bias, GPS rate + innovation gate, lever arm |
| RCS / IR | `data/rcs/*.json`, `data/ir/...` | signature tables consumed by the seeker models |

Existing schemas: `data/profiles/sa_missile_mk1.json` (guided missile),
`data/profiles/target_drone.json` (coasting target), and the referenced
subsystem profiles. New designs SHOULD follow the same file layout; profile ids
are paths relative to the consuming application's data root.

### 2.2 Warhead and guidance/autopilot (inline, always inline)

`warhead` (mass, fusing, proximity trigger, lethal/falloff radius) and
`guidance_autopilot` serialize INLINE in the manifest `physics` (camelCase keys
for the guidance/autopilot block: `navigationConstant`, `waypointGain`,
`kAccelP`, `kRateP`, `kAlphaP`, `kRollP`, `kRollD`, `maxDeflectionRad`,
`servoTimeConstantSec`, `maxServoRateRadPerSec`, and the optional
W36/W39 keys `handoffBlendTimeSec`, `lockLossRetentionSec`,
`apnFeedforwardEnabled`, `trackConfirmations`, `trackCoastTimeoutSec`,
`trackLossTimeoutSec`).

## 3. Frame and units contract

- Designers MUST emit **local ENU** coordinates (z up), not ECEF Earth-radius
  offsets; ECEF truth is an engine opt-in mode.
- Body frame: X forward (nose), Y right, Z down. Fin `positionM` is the fin
  root leading-edge axial offset from the CG along body +X (tail fins negative);
  cant is degrees.
- Angles in radians unless a key ends in `_deg`. Accelerations in m/s², masses
  in kg, thrust in N, Isp in s.
- Initial states carry `allegiance` (`friendly`/`hostile`); seeker acquisition
  is allegiance-gated by the engine.

## 4. Determinism contract

- A scenario's outcome is a pure function of its files + the kernel random
  seed; sensor/seeker noise draws from the seeded RNG. Designers MUST record the
  seed they validated with.
- All guidance/track policy is config-backed with legacy-safe defaults
  (`handoffBlendTimeSec` 0, `lockLossRetentionSec` 0, `apnFeedforwardEnabled`
  false, `trackConfirmations` 3, `trackCoastTimeoutSec` 0.5,
  `trackLossTimeoutSec` 2.0); omitted keys preserve prior behavior exactly.
- Non-finite/invalid design data MUST fail at load (`createVehicle`/
  `loadDesignPhysics` throw `std::runtime_error`); the engine never silently
  repairs a design.

## 5. Validation pipeline (engine side, implemented)

The engine validates a design end-to-end via the profile-id chain:

```text
manifest design_ref -> loadDesignPhysics -> profile-id resolution
  -> SoA wiring -> RF/RCS acquisition -> guided intercept -> kill
```

covered by `designer_pipeline_test` (9.7 m miss + proximity kill; a
deterministically tuned set, not a performance claim). The designer-side
equivalents MUST treat that test as the reference engagement for
sa_missile_mk1/target_drone-class designs.

## 6. Designer ownership (what StrikeDesigner owns)

- Geometry -> physics derivation: the manifest's `physics` block IS the engine
  contract; the designer derives `initial_mass`/`mass_dry`/inertias from its
  geometry model (the engine does no geometry). The `geometry` JSON blob is
  designer-owned and passed through opaquely.
- Subsystem selection and tuning: all per-entity subsystem configs are
  design-time configurable (W21-W39 surface: aero tables + geometric fins,
  multi-stage propulsion with transients/TVC, seeker class + radar/IR params,
  sensor grades, guidance/autopilot gains + phase-manager + track-manager keys,
  warhead fusing/radii).
- Signature attachment: RCS/IR profile ids from StrikeCEM outputs; aero tables
  from StrikeCFD (both drop-in `data/*` profile files).
- Target authoring: explicit `EntityType`, allegiance, RCS profile id,
  initial state (local ENU) and initial guidance aim (`initial_target_*`,
  including `initial_target_id` for the W39 track and `initial_target_accel_*`
  for APN feed-forward).

## 7. Out of scope (engine-side non-goals)

- StrikeEngine does not author or visualize designs; it consumes manifests.
- No runtime design mutation: a loaded design is immutable per entity; design
  changes are new artifacts.
- StrikeDesigner UX, ECS mapping, and study orchestration live in StrikeSim
  (library-only boundary, SPEC §1).

## 8. Current implementation status

- Design manifests, `design_ref` resolution, profile-id databases, and the
  end-to-end RF-seeker intercept pipeline are implemented and validated
  (`designer_pipeline_test`, W25; since W27/W30/W32/W38/W39 the profiles carry
  data-driven aero, two-stage motors, and the full guidance surface).
- The engine-side authoring API is `serializeDesign`/`loadDesignPhysics` +
  `ScenarioConfig::save/load` (SPEC §5.5). StrikeDesigner itself (geometry
  modeling, editors, ECS mapping) is a StrikeSim concern and not started here.
- Next designer-facing steps: versioned manifest/schema negotiation, and
  StrikeCFD/StrikeCEM provenance contracts for attached tables.
