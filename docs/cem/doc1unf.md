# StrikeCEM — Comprehensive Design & Planning Document

## 1. Project Intent

**StrikeCEM** is a standalone C++ computational electromagnetics (CEM) tool designed to generate radar cross-section (RCS) databases for StrikeEngine. It supports multiple solvers (PO, MoM, PO+MoM hybrid, PO+UTD, MoM+FMM), and outputs azimuth–elevation–frequency–polarization lookup tables. The design balances high-fidelity EM modeling with practical performance for large-scale simulations.

Outputs from StrikeCEM feed directly into StrikeEngine’s runtime RCS models, replacing or augmenting precomputed RCS tables.

---

## 2. Key Goals

* **Solver support**: PO, MoM, PO+MoM hybrid, PO+UTD, MoM+FMM (MLFMM).
* **Input flexibility**: load arbitrary CAD meshes (STL/OBJ), apply transforms, and define solver parameters.
* **Scalability**: multi-threaded CPU solvers, GPU acceleration for PO, iterative solvers with MLFMM for MoM.
* **Accuracy**: tunable mesh element size, edge corrections (PTD-lite/full), polarization handling, near/far-field consistency.
* **Output formats**: CSV, HDF5, VTK, compact binary, with metadata headers.
* **Integration with StrikeEngine**: API/DB compatibility.

---

## 3. High-Level Architecture

* **CLI tool**: `strikecem config.json`
* **Core subsystems**:

    * Config parser (JSON schema validated).
    * Mesh I/O (STL/OBJ/STEP via tessellation), mesh preprocessor (symmetry, tagging).
    * Mesh processing (scaling, transforms, meshing control, edge tagging, higher-order panel support).
    * Solvers:

        * **PO**: Physical Optics (fast, CPU/GPU-enabled).
        * **MoM**: Method of Moments (dense and iterative, MLFMM-ready).
        * **Hybrid**: PO for body, MoM for critical subdomains.
        * **UTD**: Uniform Theory of Diffraction for edges.
        * **FMM/MLFMM**: Fast Multipole for MoM acceleration.
    * Polarization & field projection engine.
    * Output writer (CSV/HDF5/VTK/binary).
    * Logging, timing, and benchmarking.

---

## 4. Input System — Core + Advanced Knobs

This section lists every input StrikeCEM accepts, grouped by category, with defaults and recommended ranges.

### 4.1 Model & Geometry

* `model.path` (string) — file path to mesh (.stl, .obj). For CAD (STEP/IGES) provide `model.tessellate:true`.
* `model.units` ("m"|"mm"|"in") — default "m".
* `model.transform.translate` (\[x,y,z]) — meters.
* `model.transform.rotate_euler_deg` (\[rx,ry,rz]) — degrees.
* `model.transform.scale` (number) — uniform scale.
* `model.symmetry` (object) — `{axial: bool, sector_deg: number, planes: ["xy","xz"]}`. Use to reduce domain.
* `model.part_tags` (map) — per-face tag assignments for Hybrid/MoM partitioning (e.g., seeker\_window, fin\_root).
* `model.mesh_source` ("premeshed" | "tessellate") — premeshed means loader uses given triangles; tessellate uses CAD tessellation.

Advanced geometry knobs:

* `model.cache_hash` (string) — optional user-provided hash for caching.
* `model.fix_nonmanifold` (bool) — auto-attempt repair on import.

### 4.2 Frequency & Physics

* `frequency.frequency_hz` (number) — required for single-frequency run.
* `frequency.sweep` (object) — `{start, stop, step}` for multi-frequency runs.
* `frequency.medium` — `{epsilon_r, mu_r, sigma}` (default air: 1,1,0).
* `physics.monostatic` (bool) — default true.
* `physics.bistatic_receiver_dir` — if monostatic false.

Advanced physics knobs:

* `physics.polarization_basis` ("linear"|"circular")
* `physics.incident_amplitude` (number) — default 1 V/m.

### 4.3 Angles & Sampling

* `angles.az` — `{start, stop, step}` (degrees, 0–360).
* `angles.el` — `{start, stop, step}` (degrees, -90–90).
* `angles.list` — explicit list of `[az,el]` pairs if non-uniform sampling required.
* `angles.windows` — array of refinement windows `{center:[az,el], span:[az_span,el_span], step}`.
* `angles.coarse_refine` — `{coarse_step, refine_step, peak_window_deg, peak_threshold_dB}`.

Advanced angle knobs:

* `angles.interpolate_mode` ("linear"|"spherical") — interpolation used when merging windows.
* `angles.parallel_batch_size` — number of directions per GPU/CPU batch.

### 4.4 Solver & Numerical Settings

* `solver.type` — "PO" | "MoM" | "Hybrid" | "GO" | "PO+UTD"
* `solver.precision` — "float32" | "float64"
* `solver.rcs_units` — "dBsm" | "sqm"

PO-specific:

* `po.edge_correction` — "none" | "PTD-lite" | "PTD-full"
* `po.illumination_model` — "hard" | "soft" (soft uses smooth windowing near shadow)
* `po.specular_refine_angle_deg` — refine threshold around specular direction

MoM-specific:

* `mom.equation` — "EFIE" | "MFIE" | "CFIE"
* `mom.basis` — "RWG" | "higher\_order"
* `mom.engine` — "dense" | "HSS" | "FMM" | "MLFMM"
* `mom.iterative_solver` — "GMRES" | "BiCGStab"
* `mom.tol` — e.g., 1e-3
* `mom.max_iter`
* `mom.reuse_matrix` — bool (very important for many-angle sweeps)
* `mom.preconditioner` — "ILU" | "Block-Jacobi" | "None"
* `mom.quadrature_order` — integration precision; useful near-singular treatment

Hybrid-specific:

* `hybrid.partitioning` — list of `mom_regions` (named tags on mesh)
* `hybrid.coupling` — "one\_way" | "two\_way\_iterative"
* `hybrid.max_coupling_iters`

FMM/MLFMM knobs:

* `fmm.levels` — "auto" | integer
* `fmm.compression_tol` — default 1e-3
* `fmm.near_zone_radius_m`

### 4.5 Mesh & Element Controls

* `mesh.lod` — "low" | "medium" | "high" | "adaptive"
* `mesh.global_h_over_lambda` — e.g., 0.1 (λ/10)
* `mesh.smooth_body_h_over_lambda` — e.g., 0.166 (λ/6)
* `mesh.edges_h_over_lambda` — e.g., 0.05 (λ/20)
* `mesh.min_edge_length_m`, `mesh.max_edge_length_m`
* `mesh.higher_order_panels` — bool
* `mesh.curvature_threshold` — degrees; controls adaptivity
* `mesh.facet_limit` — soft cap for memory control
* `mesh.transition_grading` — geometric grading factor between regions

Advanced mesh knobs:

* `mesh.edge_strip_width_h_over_lambda` — width of PTD edge strip
* `mesh.symmetry_sector_deg` — for axial symmetry meshing
* `mesh.quality_aspect_ratio_limit`

### 4.6 Parallelization & Runtime

* `parallel.cpu_threads` — integer
* `parallel.use_gpu` — bool
* `parallel.gpu_device_index` — integer
* `parallel.batch_directions` — integer; aim 1024–8192 for GPU
* `parallel.angle_scheduler` — "static" | "dynamic" | "mpi\_farm"
* `parallel.blas_backend` — "MKL" | "OpenBLAS" | "cuBLAS"
* `parallel.max_memory_gb` — memory cap; auto-batching respects this

### 4.7 Output & IO

* `output.path` — required
* `output.format` — "csv" | "hdf5" | "vtk" | "binary"
* `output.fields` — array; recommended `az,el,freq,pol,rcs_dBsm,phase` optional
* `output.precision` — integer decimal places
* `output.include_metadata` — bool
* `output.hdf5_layout` — "3d\_cube" | "flat\_rows" | "separate\_pols"
* `output.compression` — bool (for HDF5)
* `output.vtk_options` — near\_field surfaces, current maps etc.

### 4.8 Run / Checkpointing

* `run.checkpoint_every_n_angles` — integer; write partial outputs
* `run.resume_from_checkpoint` — path
* `run.log_level` — "info" | "debug" | "trace"

---

## 5. Comprehensive JSON Schema (condensed but exhaustive)

Below is a canonical JSON schema fragment. Use a JSON validator in your toolchain.

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "StrikeCEM Config",
  "type": "object",
  "required": ["model","frequency","angles","solver","output"],
  "properties": {
    "model": {
      "type": "object",
      "required": ["path"],
      "properties": {
        "path": {"type":"string"},
        "units": {"type":"string","enum":["m","mm","in"],"default":"m"},
        "tessellate": {"type":"boolean","default":false},
        "transform": {"type":"object"},
        "symmetry": {"type":"object"},
        "part_tags": {"type":"object"}
      }
    },
    "frequency": {"type":"object","required":["frequency_hz"]},
    "angles": {"type":"object"},
    "solver": {"type":"object","required":["type"]},
    "polarization": {"type":"array"},
    "mesh": {"type":"object"},
    "parallel": {"type":"object"},
    "output": {"type":"object","required":["path"]},
    "run": {"type":"object"}
  }
}
```

> Note: For engineering use, keep the schema in a separate file `schemas/strikecem_config.schema.json` and validate at startup, failing fast on invalid configs.

---

## 6. IO Diversity (Formats & Why)

* **CSV**: easiest to integrate with StrikeEngine loader; human-readable; good for small outputs.
* **HDF5**: efficient for large multidimensional datasets (az × el × freq × pol); supports compression and chunking; best for multi-frequency and multi-pol runs.
* **VTK (legacy or XML)**: for visualization of near-field surfaces, current/phase maps, and per-facet data (good for ParaView). Include both `.vtu` (unstructured grid) and `.vtp` (polydata) options.
* **Compact Binary (.bin) + index**: Very fast reads, small footprint for runtime; include versioning and metadata block header.
* **JSON metadata**: always accompany binary outputs: `{solver,mesh_stats,config_hash,git_rev,timestamp}`.

Output hooks:

* Optionally write **per-component** RCS (component-wise contribution) to aid debugging.
* Write intermediate checkpoints (partial angle sets) so long runs can resume.

---

## 7. Scalability & Performance Elaboration

This section explains how StrikeCEM scales and how to design for cluster runs.

### 7.1 Levels of Parallelism

1. **Angle-level** (embarrassingly parallel): each observation direction is independent in PO and many hybrid workflows. Best place to farm jobs to cores/nodes.
2. **Facet-level / intra-angle**: reductions over facets; vectorizable and GPU-friendly (use SoA memory layout).
3. **MoM matrix assembly**: parallelize near-zone assembly; use BLAS for dense ops.
4. **MoM iterative solves**: block RHS solves, GPU-accelerated mat-vec (cuBLAS/cuSPARSE), reuse preconditioners.
5. **Job-farm / MPI**: split angle set across nodes; aggregate outputs at the end.

### 7.2 GPU Strategy (PO)

* Move triangle arrays (centroid, normal, area, precomputed k·r) to device memory as SoA.
* Batch directions (M per kernel). Each thread block handles one or multiple directions and a chunk of facets.
* Use one-block-per-direction when M small; otherwise block-row × block-col tiling.
* Avoid atomics: prefer block reductions and per-direction accumulation buffers.
* Use double-precision when k·r phase accuracy matters (X/Ku/Ka bands).

Batch sizing guidance: `batch_directions` ≈ 1024–8192 depending on VRAM and warp occupancy. Tune per GPU.

### 7.3 Cluster & Job-Farm

* **Strategy**: split azimuth domain into N chunks, dispatch N jobs to nodes. Each job writes partial HDF5; master job merges.
* **Data locality**: keep mesh and facet buffers on node local disk or shared parallel FS (Lustre). Avoid NFS throttling.
* **Checkpointing**: write after every K directions; resume using `run.resume_from_checkpoint`.

### 7.4 Memory & Runtime Estimation

Use simple heuristics to estimate resource needs before run:

* `N_facets` from mesh.
* PO memory: \~ `N_facets * bytes_per_facet` (centroid(3x8) + normal(3x8) + area(8) + flags) ≈ 64–128 B/facet.
* MoM dense memory: \~ `8 * N_unknowns^2` bytes (double complex) — enormous; use MLFMM beyond \~10k unknowns.
* MLFMM memory: O(N log N) with factor depending on levels and expansion order.

Provide an estimator utility `strikecem_estimate` that reads config+mesh and prints estimated RAM and runtime (ballpark) and warns if limits exceeded.

### 7.5 Caching & Reuse

* Cache `mesh -> facet SoA` per frequency if geometry unchanged.
* Cache MoM operator/factorization if `mom.reuse_matrix` true; persist to disk (binary) for reuse across runs.
* Cache angle-level partial sums for coarse→refine strategies.

### 7.6 Adaptive Sampling & Work Reduction

* Coarse sweep (10°) → detect lobes → refine (3°) only in windows: typical runtime reduction 5–20×.
* For MoM-heavy parts, only run MoM for windows where PO cannot explain large residuals.

### 7.7 Logging & Telemetry

* Log runtime per-direction and per-batch.
* Export perf traces for GPU kernel profiling (nvprof / Nsight) and CPU performance counters.

---

## 8. Checkpointing, Robustness & Failure Modes

* **Checkpoint** after each batch: write partial HDF5 or CSV and metadata with progress index.
* **Resume**: read checkpoint, skip completed angles, continue.
* **OOM handling**: when memory cap exceeded, auto-fallback to smaller `batch_directions` or CPU-only mode.
* **Mesh errors**: detect non-manifold/open meshes and offer automatic repair with report.

---

## 9. Examples & Golden Outputs (expanded)

Include small canonical configs and expected outputs (used as regression tests / golden data).

### 9.1 PEC Sphere (golden)

Config: (as earlier) — produce constant RCS ≈ π r² in m². Golden CSV/HDF5 saved in `examples/golden/sphere_xband_5deg.h5`.

### 9.2 Flat Plate (golden)

Config: 1 m² square plate, broadside at 10 GHz. Golden CSV: peak RCS close to theoretical 4πA²/λ².

### 9.3 Simple Antenna Dipole (MoM validation)

Config: small wire/dipole modeled as RWG patch (or thin-wire approx). Compare far-field pattern magnitude and nulls to analytic.

Store golden outputs (CSV/HDF5/VTK) in `examples/golden/` to validate future changes.

---

## 10. Deliverables & Next Steps

1. Finalize and store the JSON Schema at `schemas/strikecem_config.schema.json`.
2. Implement the estimator utility (mesh reader + resource estimator).
3. Implement PO CPU baseline and golden tests (sphere, plate).
4. Add checkpoint/resume and HDF5 output.
5. Prototype GPU PO kernel and validate against CPU.

---

## 11. Appendix — Quick Reference of Advanced Knobs

* Mesh: `global_h_over_lambda`, `smooth_body_h_over_lambda`, `edges_h_over_lambda`, `higher_order_panels`, `facet_limit`.
* PO: `edge_correction`, `illumination_model`, `specular_refine_angle_deg`.
* MoM: `engine`, `basis`, `reuse_matrix`, `preconditioner`, `qladrature_order`.
* FMM: `levels`, `compression_tol`, `near_zone_radius`.
* Parallel: `cpu_threads`, `use_gpu`, `batch_directions`, `max_memory_gb`.
* Run: `checkpoint_every_n_angles`, `resume_from_checkpoint`.

---

This document can be used as the canonical design spec and implementation roadmap for StrikeCEM. I will keep it in the project canvas as `Strikecem Detailed Doc`.

If you want, I can now:

* (A) generate the full, validated `schemas/strikecem_config.schema.json` file (expanded and exact),
* (B) produce the `strikecem_estimate` utility (Python script) that reads mesh + config and prints resource estimates,
* (C) create the golden-output HDF5/CSV files for sphere/plate/dipole testcases (synthetic if needed) so CI can use them.

Which of (A),(B),(C) do you want me to produce next?
