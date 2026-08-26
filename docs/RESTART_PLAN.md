# StrikeEngine / StrikeSim / StrikeCEM — Restart Plan

**Status:** approved; engine W1–W3 spine verified; execution in progress
**Date:** 2026-02 (restart session)

This document is the safety record for the project restructure. It captures the verified
starting state, the locked-in architecture decisions, the phase-by-phase execution plan,
and the rollback safety net. **Commit discipline: one recoverable checkpoint per phase;
nothing is deleted before it exists in git history.**

### Current engine checkpoint — 2026-08-26

The interrupted engine-fidelity repair was resumed and completed through W7.
The CPU build succeeds and the full CTest suite passes **11/11**. The repair
covered the autopilot frame/sign chain,
bounded aerodynamic control authority, predictive intercept guidance,
ground-impact deactivation, interpolated impact timestamps, and bounded RK45
adaptation, the W6 seeker MVP (geometry gates, tracking hysteresis, LOS rates,
and measurement latency), the coupled navigation EKF and bounded
covariance/bias regression, and the W5 terrain/wind environment MVP. The
historical
starting state and phase plan below are retained as the restart record.

---

## 1. Locked decisions (user-approved 2026-02)

1. **Separate git repositories.** StrikeEngine becomes a library-only repo (core + public
   headers + tests). StrikeSim becomes its own app repo that consumes StrikeEngine (and
   StrikeCEM) as versioned dependencies.
2. **StrikeCEM stays its own project** per its authority docs (`IMPLEMENTATION.md`,
   `SPEC.md`), embedded into StrikeSim as a **library** (`strikecem_lib`), with the CLI
   retained for offline batch runs. Its source does **not** get merged into StrikeSim.
3. **Work order: Engine + API first.** Stabilize/define the StrikeEngine public API and
   get the core lib + tests green, *then* StrikeSim shell + Designer, *then* embed
   StrikeCEM. (StrikeCEM offline development can run in parallel.)
4. **Commit the rewrite, archive the old ECS.** The mid-rewrite (`kernel/`, `models/`,
   `simulation/`, new Designer code) is committed to history; the old ECS tree
   (`include/`, `src/`) is preserved behind a git tag, then removed from the active tree.

---

## 2. Verified starting state (pre-restructure)

### StrikeEngine repo (`~/projects/StrikeEngine`) — git `main`, commit 1d114f4

- **New rewrite (the keeper):**
  - `kernel/` — `SimulationKernel`, SoA blocks (`PhysicsBlock`, `ControlBlock`,
    `GuidanceBlock`, `NavigationBlock`, `SensorBlock`, `SeekerBlock`,
    `EntityStatusBlock`), integrators (RK4/RK45/Euler/Symplectic/Factory),
    backends (`PhysicsBackend` interface, `CPUBackend`, `VulkanBackend` +
    `VulkanContext` + `physics_step.comp` shader), systems (Sensor/Navigation/Seeker/
    Guidance/Autopilot/Event/CommandProcessor), `HybridScheduler`, `KernelTime`,
    `KernelConfig`, `ScenarioConfig`.
  - `models/` — stateless headers-only math models: `ISA1976`/`AtmosphereModel`,
    `BasicAeroModel`, `ThrustCurve`/`PropulsionModel`; `models/signatures/` has
    `RCSDatabase` + `IRSignatureDatabase` (.cpp present, the StrikeCEM consumer side).
  - `simulation/` — `SingleRun`, `ParamSweep`, `MonteCarlo`, `Optimizer` (PSO).
  - `tests/validation/` — `singlerun_test.cpp`, `sweep_test.cpp`, `mc_test.cpp`
    (untracked); `tests/optimizer_test.cpp` (untracked); `tests/atmosphere_test.cpp`
    (tracked).
  - `docs/current_roadmap.md`, `docs/roadmap.md` — the rewrite's own roadmap
    (Phases 1–6; no `apps/` in the new scaffold — engine = library).
- **Old ECS tree (to archive):** `include/strikeengine/**` + `src/strikeengine/**`
  (80 files) — orphaned: root `CMakeLists.txt` never adds `src/`; `src/CMakeLists.txt`
  defines a **duplicate `strikeengine` target** of the same name. Old `#include`
  paths are comp-ponents/ecs-era. `docs/cem/` (old CEM planning) is gitignored and
  superseded by the StrikeCEM docs.
- **Apps (to migrate out):**
  - `apps/StrikeDesigner/` — substantial new code (untracked): `app/` (App.cpp/hpp,
    main.cpp), `gui/` (GuiLayer, Dockspace, MenuBar, 9 panels), `core/VehicleModel.{hpp,cpp}`
    (self-contained; glm/imgui/nlohmann only), `renderer/VulkanContext.{hpp,cpp}` (677 L),
    `data/fonts/Inter-Regular.ttf`. Its `CMakeLists.txt` is **Windows-only**
    (hardcoded MinGW SDL3 zip + DLL copy) — needs a Linux port later.
  - `apps/StrikeSim/` — empty stubs (`main.cpp`, `App.hpp`, `ScenarioLoader.hpp` all
    0 bytes) + CMakeLists. To become the main executable shell.
  - `apps/MissionCLI/` — already deleted (staged) in git.
- **Build state (verified on this Linux box):** cmake configure OK; `strikeengine`
  static lib compiles; **every executable fails to link** because
  `kernel/backend/vulkan/VulkanContext.cpp` calls `vkCreateDebugUtilsMessengerEXT` /
  `vkDestroyDebugUtilsMessengerEXT` without loading the extension function pointers.
  StrikeDesigner additionally cannot configure on Linux (SDL3 MinGW zip).
- **Git hygiene problem:** the entire rewrite is **untracked** (exists only on disk);
  `.idea/` churn is staged+dirty; generated junk (`build/`, `.cache/`, `build_log.txt`,
  `singlerun_output.csv`, `test.cpp`) is not ignored.

### StrikeCEM (`~/projects/StrikeCEM`) — docs complete, source ~empty

- **Authority docs** (`~/Documents/projects/StrikeCEM/`): `SPEC.md` (v1 contract),
  `IMPLEMENTATION.md` (repo layout + phase 0–2), `FULL_IMPLEMENTATION.md` (long-term,
  non-normative), `BACKLOG.md`, `docs/CONVENTIONS.md` (normative),
  `docs/OUTPUT_FORMAT.md` (flat sample table, CSV/HDF5), `docs/STRIKEDESIGNER_INTEGRATION.md`
  (export package + manifest + hash rules), `schemas/config.schema.json` (strict,
  draft-07, complete), 4 ADRs.
- **Actual repo:** stale PoC contradicting the docs — `main.cpp`, `app/CLI.cpp`,
  `io/MeshLoader.cpp` + header, `core/Config.cpp` + header, `solvers/PhysicalOptics.*`,
  `Solver.hpp`, `Hybrid.hpp`, `MoM.hpp` are all **0 bytes / empty**. README + examples
  still claim MoM/Hybrid/CUDA/adaptive meshing (explicitly **removed from v1** by SPEC).
- CMake builds `StrikeCEM` exe + `strikecem_lib` **shared** library (the intended
  integration boundary for StrikeEngine/StrikeSim — keep this).
- Net: docs-contract done; **source is Phase 0 scaffold work** per its own docs.

### Documents authority

- StrikeCEM's documents (`~/Documents/projects/StrikeCEM/`) are authoritative for the
  CEM product and the Designer↔CEM↔Engine handoff contract.
- The old `docs/cem/*.md` inside StrikeEngine are superseded history (gitignored).
- The old `StrikeEngine.md` / `StrikeEngine Overview.md` docs in
  `~/Documents/projects/StrikeEngine/` are pre-rewrite planning (protobuf design target
  noted for the future API layer, but not normative for current work).

---

## 3. Target architecture

```
~/projects/
├── StrikeEngine/      # library repo (own git history)
│   ├── include/strikeengine/   # PUBLIC API: kernel/, models/, simulation/
│   ├── src/strikeengine/       # implementation .cpp (same sub-trees)
│   ├── tests/                  # validation + unit tests (ctest)
│   ├── tools/                  # engine tools (GenerateAtmosphereTable, …)
│   ├── data/                   # shared assets (profiles, scenarios, aero, …)
│   └── (optional) strikeengine_vulkan lib — Vulkan compute backend, opt-in
│
├── StrikeSim/         # main app repo (own git history) — the executable
│   ├── designer/      # StrikeDesigner surface (GUI, panels, VehicleModel, viewport)
│   ├── cem/           # StrikeCEM integration surface (validate → run → plot RCS)
│   ├── sim/           # simulation/analysis surface (SingleRun, MC, sweep, optimizer)
│   ├── shell/         # app entry, window, mode routing
│   └── consumes StrikeEngine + StrikeCEM via FetchContent / find_package
│
└── StrikeCEM/         # per its docs: own repo — CLI + strikecem_lib
                        # (lib consumed by StrikeSim; docs stay authoritative)
```

Ownership (per `docs/STRIKEDESIGNER_INTEGRATION.md`):
StrikeDesigner owns design intent; StrikeCEM owns computation, validation, provenance;
StrikeEngine owns runtime simulation + database lookup.

---

## 4. Phase 0 — Repo restructure (one-time)

**Goal:** everything committed, nothing lost, three clean repos.

### 0a. Git hygiene + snapshot (in StrikeEngine)

1. Extend `.gitignore`: `build/`, `.cache/`, `build_log.txt`, `singlerun_output.csv`,
   `test.cpp` (scratch). (`.idea/`, `cmake-build-*/`, `docs/cem`, `*.bin` already ignored.)
2. Untrack IDE/structural churn: `git rm -r --cached .idea`, `git rm structure.txt`.
3. `git add -A` → single snapshot commit of the **entire** working tree (rewrite +
   rollback state).
4. `git tag legacy/pre-restructure-v1` — this tag holds the old ECS tree,
   old `apps/`, and everything else. **Rollback for any later step is this tag.**

### 0b. Split StrikeSim repo out

1. Create `~/projects/StrikeSim` (git init).
2. `git mv apps/StrikeDesigner → StrikeSim/designer` (carries untracked new code on disk).
3. `git mv apps/StrikeSim → StrikeSim/shell` (stubs for future shell).
4. StrikeSim gets a skeleton `CMakeLists.txt` (FetchContent declares StrikeEngine +
   StrikeCEM; everything guarded until Phases 1–3 land) and a `README.md` describing
   the intended layout. Commit as initial import.
5. In StrikeEngine: delete `apps/` from root CMake (`add_subdirectory(apps)`),
   commit the move.

### 0c. Archive old ECS in StrikeEngine

1. `git rm -rf include/strikeengine src` (old tree; preserved in the tag).
2. Root `CMakeLists.txt`: drop the stale Windows bits, keep glm/nlohmann,
   keep tests; `src/CMakeLists.txt` is deleted with `src/`.
3. Commit "archive legacy ECS tree". Verify configure + lib build still OK
   (tests will link once Phase 1a lands).

**Definition of done (Phase 0):** three repos; StrikeEngine = kernel/models/simulation/
tests/data/docs only; legacy state reachable via `legacy/pre-restructure-v1`.

---

## 5. Phase 1 — Engine stabilization + public API (Engine+API first)

### 1a. Split Vulkan out of the core (fixes the link bug)

- New option `STRIKEENGINE_WITH_VULKAN` (default OFF).
- Remove `find_package(Vulkan REQUIRED)` + glslc prebuild from the core path; core
  globs `kernel/*.cpp` **excluding** `backend/vulkan/*`.
- New `strikeengine_vulkan` target (VulkanContext.cpp + VulkanBackend.cpp + shader
  prebuild), linked into apps that want GPU.
- Fix `VulkanContext.cpp` debug messenger: resolve
  `vkCreateDebugUtilsMessengerEXT` / `vkDestroyDebugUtilsMessengerEXT` via
  `vkGetInstanceProcAddr` (no volk dependency, no undefined symbols).
- `SimulationKernel` gets a tiny factory shim: `createVulkanPhysicsBackend()` declared
  in `backend/vulkan/`, defined in the vulkan lib; core calls it under
  `STRIKEENGINE_ENABLE_VULKAN`, else throws a clear "built without Vulkan" error.
- `tests/CMakeLists.txt`: drop `Vulkan::Vulkan` link (CPU tests need no Vulkan).

### 1b. Public API layout (`include/strikeengine/`)

- Move `.hpp` → `include/strikeengine/{kernel,models,simulation}/...`;
  `.cpp` → `src/strikeengine/{kernel,models,simulation}/...`.
- Rewrite every project include to canonical `#include <strikeengine/...>` (scripted,
  mechanical; external includes glm/nlohmann/j/vulkan untouched).
- `target_include_directories(strikeengine PUBLIC "${CMAKE_SOURCE_DIR}/include")`.
- Update test includes to the canonical form.

### 1c. Install / package consumability

- `install(TARGETS strikeengine …)` + headers + a `CMakePackageConfigHelpers`
  config so StrikeSim can `find_package` / FetchContent a pinned tag.
- Optional `strikeengine_vulkan` installed alongside.

**Definition of done (Phase 1):** fresh configure OFF/ON both build; `ctest` green
(run_tests / sweep / mc / optimizer); a throwaway consumer main compiles+links against
the installed API.

---

## 6. Phase 2 — StrikeCEM Phase 0–1 (offline; can run in parallel)

Follow the authority docs exactly (`IMPLEMENTATION.md` §2, §5–11; `SPEC.md`).

1. Scrub the stale PoC: delete 0-byte/empty files (`main.cpp`, `CLI.cpp`,
   `Config.cpp` content, `MeshLoader.cpp`, `PhysicalOptics.*`, empty `Solver/Hybrid/MoM`).
2. Phase 0 scaffold: `src/{cli,config,geometry,solver/po,output,runtime,util}` +
   `tests/` per doc layout; schema loader (validate-but-no-unknown-keys, defaults
   resolution, canonical JSON + SHA-256 config hash); `strikecem validate|estimate|run`
   routing with the documented exit codes (0,2,3,4,5,6).
3. Phase 1: STL/OBJ mesh loader + validator (`off|report|repair` modes, mesh report
   fields); deterministic CPU PO (hard-lit PEC, canonical far-field equation,
   HH/VV → HV/VH → circular); CSV + flat-row HDF5 writer with provenance and
   validity/status; goldens (single triangle, flat plate, rotated plate,
   electrically-large sphere).
4. **Reader fixture alongside the writer** (SPEC §10 — StrikeEngine's
   `models/signatures/RCSDatabase` is the consumer; the reader validates format
   version, hashes, coverage, no `error` samples, design identity).
5. `strikecem_lib` shared target retained (embed boundary for StrikeSim).

**DoD:** `strikecem run examples/plate…` produces a valid HDF5+CSV; engine-side reader
fixture test passes; `validate` rejects unknown keys/materials/GPU with the right
exit code.

---

## 7. Phase 3 — StrikeSim shell + Designer

1. Main executable shell with Design / Simulate / CEM surfaces (one app, shared
   context — a design can be simmed and its RCS computed in-session).
2. Port Designer: `gui/` (9 panels), `core/VehicleModel`, viewport renderer
   (`renderer/VulkanContext.cpp`). Linux tasks: SDL3 fetch → system/vcpkg SDL3,
   ImGui docking, Vulkan device wiring — done as its own milestone.
3. Engine integration via API: scenario build (`ScenarioConfig.loadInto`),
   step loop, `SingleRun`/`MonteCarlo`/`ParamSweep`/`Optimizer` surfaces, live state
   readouts from the SoA blocks.
4. CEM integration via `strikecem_lib`: Designer's **Export package** workflow
   (`export_manifest.json` + `strikecem.config.json` + `geometry/`) per
   `docs/STRIKEDESIGNER_INTEGRATION.md`; in-app validate → estimate → run → plot RCS.

**DoD:** designer builds on Linux; a designed part exports a valid package;
StrikeCEM run consumes it; RCS database loads into the sim (design identity matched).

---

## 8. Beyond (deferred, from old design targets)

- StrikeEngine API server layer (protobuf/gRPC or REST) wrapping the same public API —
  the old "StrikeEngine Overview" target. Only after the C++ API is stable.
- `tools/` wiring in engine repo (GenerateAtmosphereTable exists; convert_srtm is a
  0-byte stub), `data/` cleanup, mission CLI re-homed into StrikeSim or a `tools/` there.

---

## 9. Safety net & rollback

- **`legacy/pre-restructure-v1`** tag = full pre-restructure snapshot (old ECS +
  old apps + rewrite). Anything deleted in Phases 0–1 is in this tag.
- Commits are small and per-file-meaningful so individual steps can be reverted.
- Destructive ops (`git rm`, moves) happen only after the snapshot commit exists.
- Generated junk is ignored, never committed.

---

## 10. Execution log

| Date | Step | Result / commit |
| ------ | ------ | ----------------- |
| 2026-02 | Plan approved (4 decisions) | A: build engine in place, StrikeSim fresh; B: plan doc |
| 2026-02 | Doc written | `docs/RESTART_PLAN.md` |
| 2026-02 | Phase 0a snapshot + tag | engine `a9e60ad` + `legacy/pre-restructure-v1` |
| 2026-02 | Phase 0b StrikeSim split | StrikeSim `78e4b67`; engine `90d03c1` (apps out) |
| 2026-02 | Phase 0c archive old ECS | engine `90d03c1`, `63cbf87`, `d1507fb`; tree clean |
| 2026-02 | Phase 1a Vulkan split | engine `9e9ad4d` — core lib compiles w/o Vulkan; ctest 4/4 green; Vulkan-ON build verified |
| 2026-02 | Phase 1b API layout | engine `08a64ba` — 66 renames to include/src; canonical `<strikeengine/...>` includes; both builds green; consumer smoke test passes |
| 2026-08-25 | Phase 1c install/package | engine `0b61905` — install/export package (`strikeengine` + optional `strikeengine_vulkan`), `StrikeEngine::` namespace, package config (`find_package(StrikeEngine 0.1)`), GNUInstallDirs; installed package is self-contained (glm/nlohmann build-tree only) |
| 2026-08-25 | Phase 1 gate | engine `0b61905` — fresh configure **OFF** and **ON** both build; `ctest` 4/4 green in both; `cmake --install` to staging prefix verified for both; throwaway consumer (`find_package` + `SimulationKernel`/`ISA1976` smoke main) compiles, links and runs against both installed packages (Vulkan-dependency branch exercised with ON) |
| 2026-08-25 | Phase 3.1–3.2 Linux Designer port | StrikeSim `9c376fe` — `strikesim` single executable: `shell/` = real entry (SDL window + `--surface=design\|sim\|cem`, Simulate/CEM placeholders), `designer/` = surface library (9 panels, VehicleModel, viewport). Linux CMake replaces Windows-only MinGW build: system SDL3 + Vulkan, ImGui docking pinned `fd13a1e8`, glm/json at engine versions; **StrikeEngine consumed as installed Phase 1c package** (`find_package(StrikeEngine 0.1)`, prefix `~/projects/dist/strikeengine`). Verified on Linux: build clean; app initializes SDL+Vulkan+ImGui, renders, exits cleanly for all three surfaces. |
| 2026-08-25 | Phase 3.3 Simulate surface | StrikeSim `d82f851` + engine `d84108d` — interactive kernel loop (canned scenario → step → SoA readouts) with Run Controls/Trajectory/Live Readouts panels; studies on a joined worker thread with in-app results parsed from engine CSVs: SingleRun, ParamSweep (target-x), MonteCarlo (velocity noise, mean±σ), Optimizer (PSO ballistic throw). Engine fix found during integration: `ScenarioConfig` now routes target velocity into PN (`initialTargetVx/Vy/Vz`, additive). Verified: builds + boots on Linux; sweep CSV path checked headlessly vs installed engine. |
| 2026-08-25 | Follow-up (engine fidelity, deferred) | **Guidance loop vs flight model:** guided ≈ ballistic in all headless probes (min miss ~475 m even dead-ahead). Root causes: steering authority is boost-phase-only (fins torque the body, no lift force — `CL=0` in `BasicAeroModel`, fin deflection only rotates thrust), and the autopilot rate loop is under-tuned (**no** fin-limit issue — ±0.43 rad clamp exists). Suggested next engine session: rate-command autopilot + lift from fin deflection + per-entity propulsion. Until then the Simulate surface documents "wide miss expected". |
| 2026-08-25 | Deep fidelity session W1–W3 (committed, DoD NOT met) | engine `7018693` (W1 per-entity `VehicleConfig` + W2 body-frame 6-DOF, `Quaternion.hpp`, propulsion pool), `0f8c3e3` (W3 control authority: fin lift from δ/α, rate-command loop, α/β damping, servo lag+rate limit; W4 part: derivative-callback integrators, true RK4/RK45), `51bbef2` (nav truth alignment estQ/estP/estV), `d9acd44` (DoD tests: rigidbody/vehicleconfig/intercept). **Measured: 5/7 ctest green; `intercept_test` RED (min miss 2978 m — high-q pitch+yaw limit cycle ~t=2 s; thrust ruled out via coasting isolation run; sign chain suspect); `rigidbody_test` RED on world-climb assertion (nose-up → +Z); ghosts (W5) still live.** See `docs/FIDELITY_AUDIT.md` post-session status. |
