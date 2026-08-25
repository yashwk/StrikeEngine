# StrikeEngine Unified Roadmap

This roadmap outlines the complete Data-Oriented Design (SoA) architecture for StrikeEngine, enforcing a strict separation between pure mathematical models (`models/`) and execution state (`kernel/`).

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
* **Tasks:** Basic `PhysicsBlock`, `KernelTime`, `RK4Integrator`, Minimal `AtmosphereModel`, `AeroModel`, `PropulsionModel`, `CPUBackend`, `SimulationKernel`, and `SingleRun` wrapper. Validation tests.

## Phase 2: Multi-Entity SoA Engine (Scale & Control)
**Goal:** Expand to handle arrays of entities and basic control systems.
* **Tasks:** Expand `PhysicsBlock` to SoA arrays, add `ControlBlock`, `GuidanceBlock`, `CommandProcessor`, `EventSystem`, and basic `GuidanceSystem`.

## Phase 3: Navigation & Estimation (The Sensor Loop)
**Goal:** Realistic flight sensing and error states.
* **Tasks:** Add `NavigationBlock`, `IMU`, `StrapdownINS`, `ErrorStateEKF`, and `GPS Fusion`.

## Phase 4: Advanced GNC (The Brain)
**Goal:** Intercept capabilities and autonomy.
* **Tasks:** Midcourse Guidance, Autopilot, Terminal Guidance, and Seekers (IR/Radar models).

## Phase 5: High-Fidelity Physics & Hardware Acceleration
**Goal:** Real-world planet models and massive scale.
* **Tasks:** WGS84 Gravity, Earth Rotation, Coriolis effects, Adaptive `RK45Integrator`, and the `GPUBackend` (Vulkan/CUDA).

## Phase 6: Simulation Power Tools (The Lab)
**Goal:** Research-grade analytical capabilities.
* **Tasks:** `ParamSweep`, `MonteCarlo`, `BatchRunner`, and `Optimizer`.
