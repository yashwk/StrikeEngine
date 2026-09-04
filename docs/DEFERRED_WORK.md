# Deferred Work and Remaining Features

Authoritative inventory of features that are planned, partial, or owned by
downstream integration projects. Implemented features are intentionally not
listed as pending.

## Kernel and physics

- CFD-validated aerodynamic coefficient data and StrikeCFD coupling.
- Higher-fidelity aerodynamics: Reynolds dependence, nonlinear stall and
  post-stall behavior, body/fin interference, and flexible-body effects.
- Advanced atmosphere and weather models.
- Power, communications, and ECM models.
- Probabilistic failure degradation and partial health/repair behavior.
- Full GPS-only positioning mode; current GPS aiding is not equivalent.
- CFD validation of moment (`cm`) and lateral (`beta`) coefficient tables.

## Earth, terrain, and environment

- Automatic spatial multi-tile terrain discovery and streaming.
- Terrain tile prefetch.
- Vertical datum and geoid models.
- Higher-fidelity polar terrain coverage.
- Atmospheric rotation/wind coupling and a full moving-origin global
  propagator.

Optional GDAL GeoTIFF/DTED/VRT loading and bounded source caching are already
implemented MVP features, not deferred work.

## Navigation and sensing

- General sensor fusion.
- Magnetometer, barometer, and radar-altimeter models.
- Richer sensor timing and calibration.
- Multi-rate timestamp interpolation.
- Imaging IR.
- Dynamic SARH illuminator tracking.
- Band-resolved atmospheric extinction.
- Multi-target tracking; W39 currently provides a persistent single-target
  track manager.

## Guidance and control

- Trajectory optimization and full trajectory management. W40's
  trajectory-aware midcourse predictor and acceleration-feasibility gate are
  implemented; optimization remains deferred.
- Energy-aware guidance, energy corridors, and energy management.
- Advanced pursuit guidance.
- LQR and MPC control laws.
- Seeker-management handoff beyond the implemented acquisition-to-terminal
  blend.

W40 is complete: trajectory-aware midcourse guidance consumes the W39
`TrackBlock`, provides predicted intercept/tgo/required-acceleration
diagnostics, handles dropout/reacquisition, supports acceleration-aware
relative kinematics (Newton-Raphson refinement for axial boost and drag deceleration),
and preserves seeker-lock and legacy fallbacks.

The next guidance milestone is W41: full trajectory optimization, energy corridors,
and energy management with deterministic tests for energy limits, actuator limits,
and feasible target interception.

## Execution and platform support

- Validated Vulkan/CPU parity.
- GPU ECEF support.
- CUDA support, if required.
- Production GPU backend.

Parallel CPU execution is implemented: multi-threaded `CPUBackend` with persistent `WorkerPool`, parallelizing derivative evaluation and acceleration cache refresh across entities with configurable worker count (`setThreadCount`) and bit-identical single-thread fallback (`multithread_test`). Guidance and Warhead profile database loaders are also implemented, completing the subsystem profile-id database suite.

## Telemetry, tooling, and integration

- Richer telemetry schemas and versioned schema evolution.
- Plotting, analysis, and scenario-authoring tools.
- Shared logging, units, and profiling infrastructure.
- Visualization/debug drawing.
- Optional ECS/editor mapping in downstream applications.
- Future API server wrapper (protobuf/gRPC or REST).
- Explicit versioned StrikeSim/StrikeDesigner/StrikeCEM/StrikeCFD handoffs
  and provenance contracts.

## Scope boundaries

StrikeEngine remains a library. StrikeSim owns the shell/UI and Designer
surfaces; StrikeCEM and StrikeCFD remain separate producers. Future solvers,
GPU paths, terrain databases, material models, or external APIs must not be
silently substituted into the supported runtime contract.

## Source of truth

This inventory consolidates the deferred-work sections of `SPEC.md`,
`IMPLEMENTATION.md`, and `FIDELITY_AUDIT.md`. When a feature is implemented,
update those documents and remove or reclassify its entry here.
