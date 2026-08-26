# StrikeEngine Unified Roadmap

This roadmap outlines the complete Data-Oriented Design (SoA) architecture for StrikeEngine, enforcing a strict separation between pure mathematical models (`models/`) and execution state (`kernel/`).

> **Status overlay — verified 2026-08-26:** This is an older roadmap and its
> original phase text is retained. Phase status below reflects the current
> repository. `[x]` means implemented and validated; `[~]` means an MVP or
> integrated equivalent exists; `[ ]` means pending. The current code uses
> `kernel/systems/` for several GNC implementations that this document
> originally planned as standalone `models/` classes.

## Unified Architecture

* **`kernel/`**: Pure state and execution. Contains SoA data blocks, integrators, CPU/GPU backends, and the `SimulationKernel` orchestrator.
* **`models/`**: Pure, stateless mathematical functions.
  * `physics/`: Atmosphere, aerodynamics, propulsion, gravity, earth frames.
  * `gnc/`: Navigation (INS, filters, sensors), Guidance (midcourse, terminal, autopilot), and Seekers (radar, IR, EO).
* **`simulation/`**: Execution wrappers. Feeds scenario data to the kernel (SingleRun, MonteCarlo, ParamSweep, Optimizer).
* **`ecs/`**: *Optional.* Maps SoA kernel indices to visual/editor entities.
* **`io/`, `utils/`, `tooling/`**: Supporting systems.

---

## Phase 1: Deterministic MVP (The Core Loop)
**Goal:** A single, deterministic missile flying a ballistic/simple thrust trajectory using CPU.
* **Tasks — [x] DONE:** Basic `PhysicsBlock`, `KernelTime`, `RK4Integrator`, Minimal `AtmosphereModel`, `AeroModel`, `PropulsionModel`, `CPUBackend`, `SimulationKernel`, and `SingleRun` wrapper. Validation tests.

## Phase 2: Multi-Entity SoA Engine (Scale & Control)
**Goal:** Expand to handle arrays of entities and basic control systems.
* **Tasks — [x] DONE:** Expand `PhysicsBlock` to SoA arrays, add `ControlBlock`, `GuidanceBlock`, `CommandProcessor`, `EventSystem`, and basic `GuidanceSystem`.

## Phase 3: Navigation & Estimation (The Sensor Loop)
**Goal:** Realistic flight sensing and error states.
* **Tasks — [x] DONE for MVP:** Add `NavigationBlock`, `IMU`, `StrapdownINS`, `ErrorStateEKF`, and `GPS Fusion`.

## Phase 4: Advanced GNC (The Brain)
**Goal:** Intercept capabilities and autonomy.
* **Tasks — [x] MVP DONE:** Midcourse Guidance, Autopilot, Terminal Guidance, and Seekers (IR/Radar models). Explicit PN/APN behavior is implemented; trajectory management and additional seeker types remain pending.

## Phase 5: High-Fidelity Physics & Hardware Acceleration
**Goal:** Real-world planet models and massive scale.
* **Tasks — [~] PARTIAL:** WGS84 Gravity, Earth Rotation, Coriolis effects, Adaptive `RK45Integrator`, and the `GPUBackend` (Vulkan/CUDA). Adaptive RK45, an optional Vulkan backend, WGS84 normal gravity, and local Coriolis effects are implemented; full earth-fixed frames/transport, centrifugal terms, and CUDA remain pending.

## Phase 6: Simulation Power Tools (The Lab)
**Goal:** Research-grade analytical capabilities.
* **Tasks — [x] MVP DONE:** `ParamSweep`, `MonteCarlo`, `BatchRunner`, and `Optimizer`. Structured batch execution is implemented; richer output writers remain pending.

### Current validation checkpoint

The current engine checkpoint has **14/14 CTest tests passing**. W1–W9 MVP
fidelity work is recorded in `docs/FIDELITY_AUDIT.md`; terrain/wind MVP support
is included in W5 and opt-in WGS84/local-earth effects in W9. Remaining
roadmap items are full earth-fixed frame/transport models, expanded
sensors/seekers/controllers, structured I/O/tooling, failure models, and a
generalized GPU backend.
