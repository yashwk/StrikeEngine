# StrikeCEM — Comprehensive Design & Planning Document

## 1. Project Intent
**StrikeCEM** is a standalone C++ computational electromagnetics (CEM) tool designed to generate radar cross-section (RCS) databases for StrikeEngine. It supports multiple solvers (PO, MoM, PO+MoM hybrid, PO+UTD, MoM+FMM), and outputs azimuth–elevation–frequency–polarization lookup tables. The design balances high-fidelity EM modeling with practical performance for large-scale simulations.

Outputs from StrikeCEM feed directly into StrikeEngine’s runtime RCS models, replacing or augmenting precomputed RCS tables.

---

## 2. Key Goals
- **Solver support**: PO, MoM, PO+MoM hybrid, PO+UTD, MoM+FMM (MLFMM).
- **Input flexibility**: load arbitrary CAD meshes (STL/OBJ), apply transforms, and define solver parameters.
- **Scalability**: multi-threaded CPU solvers, GPU acceleration for PO, iterative solvers with MLFMM for MoM.
- **Accuracy**: tunable mesh element size, edge corrections (PTD-lite/full), polarization handling, near/far-field consistency.
- **Output formats**: CSV, HDF5, with metadata headers.
- **Integration with StrikeEngine**: API/DB compatibility.

---

## 3. High-Level Architecture
- **CLI tool**: `strikecem config.json`
- **Core subsystems**:
  - Config parser (JSON schema validated).
  - Mesh I/O (STL/OBJ loader).
  - Mesh processing (scaling, transforms, meshing control, edge tagging).
  - Solvers:
    - **PO**: Physical Optics (fast, GPU-enabled).
    - **MoM**: Method of Moments (dense and iterative).
    - **Hybrid**: PO for body, MoM for critical subdomains.
    - **UTD**: Uniform Theory of Diffraction for edges.
    - **FMM**: Fast Multipole Method for MoM acceleration.
  - Polarization & field projection engine.
  - Output writer (CSV/HDF5).
  - Logging, timing, and benchmarking.

---

## 4. Input System
### Core Inputs
- **Model**: path, units, transforms (translate, rotate, scale).
- **Frequency**: center frequency, monostatic vs bistatic.
- **Angles**: az, el ranges, step size, coarse→refine driver.
- **Solver type**: PO, MoM, Hybrid, GO, UTD.
- **Polarization**: HH, VV, HV, VH, RHCP, LHCP.
- **Mesh**: LOD, element size (h/λ), adaptive control.
- **Parallelization**: threads, GPU flag, batch sizes.
- **Output**: path, format (CSV/HDF5), fields, precision.
- **PO Options**: edge correction, lit-only.

### Advanced Inputs
- Element size control per part.
- Adaptive meshing strategies.
- MoM solver options: basis functions, preconditioner, tolerance.
- UTD edge tagging and diffraction orders.
- Hybrid partitioning (part-wise solver assignment).
- Memory limits for MoM/MLFMM.

---

## 5. Meshing & Facet Strategy
- **Element shapes**: triangular facets (default), optional higher-order patches.
- **Element size trade-off**:
  - Smaller facets → higher accuracy everywhere.
  - Larger/high-order facets → fewer elements, same accuracy on smooth regions, but lower accuracy on sharp/complex geometry.
- **Adaptive meshing**:
  - Critical regions (seeker radomes, fins, antennas) use smaller elements.
  - Smooth body regions use larger facets.
- **User control**: per-part meshing strategy and global defaults.

---

## 6. Solver Methods
### Physical Optics (PO)
- Compute induced currents on lit facets.
- Integrate scattered field contributions per direction.
- Options: lit-only or soft shadow.
- Edge corrections via PTD-lite/full.
- GPU kernel for angle batches.

### Method of Moments (MoM)
- RWG basis functions on mesh edges.
- Dense impedance matrix assembly.
- Iterative solve via GMRES.
- Hybrid MoM+MLFMM for scalability.

### Hybrid PO+MoM
- Partition geometry: large smooth regions via PO, small detail via MoM.
- Coupling handled via equivalent currents/fields.
- Reduces MoM unknown count while capturing fine effects.

### PO+UTD
- PO for smooth body.
- Edge diffraction modeled via UTD (creeping waves, shadow zones).
- Requires edge tagging and wedge diffraction coefficients.

### MoM+FMM
- Use Fast Multipole Method for accelerating MoM matrix-vector products.
- Reduces memory from O(N²) to O(N log N) and enables larger problems.

---

## 7. Outputs
- **CSV**: simple, human-readable, easy for StrikeEngine ingestion.
- **HDF5**: compact, fast, scalable for large datasets.
- **Metadata**: mesh stats, solver type, config hash, date/time.

Fields: azimuth, elevation, frequency, polarization, RCS_dBsm, optional metadata.

---

## 8. Repository Scaffold
```
strikecem/
├─ CMakeLists.txt
├─ src/
│   ├─ main.cpp
│   ├─ mesh/
│   ├─ solvers/{PO,MoM,Hybrid,UTD}
│   ├─ io/
│   ├─ utils/
├─ examples/
│   ├─ sphere.stl
│   ├─ po_default.json
├─ tests/
│   ├─ test_sphere.cpp
│   ├─ test_plate.cpp
```

---

## 9. Validation Strategy
- **Analytic tests**:
  - PEC sphere: compare to Mie solution (check PO asymptote).
  - Flat plate normal incidence: known RCS formula.
- **Regression tests**:
  - Ensure PO CPU = PO GPU (within tolerance).
- **Scaling benchmarks**:
  - Measure runtime vs facet count, vs angles.

---

## 10. Milestones
- **Phase 1**: Repo scaffold, PO CPU, CSV output, sphere/plate tests.
- **Phase 2**: GPU PO, HDF5 output, coarse→refine driver.
- **Phase 3**: MoM (dense), Hybrid PO+MoM.
- **Phase 4**: MLFMM, PO+UTD.
- **Phase 5**: UX, docs, API for StrikeEngine.

---

## 11. Performance Expectations
- PO CPU: minutes for medium-resolution sweeps on laptop.
- PO GPU: 10×–100× speedup (angle batching).
- MoM dense: thousands of unknowns feasible.
- MoM+FMM: millions of unknowns feasible with cluster.

---

## 12. Integration with StrikeEngine
- StrikeCEM outputs are ingested as RCS databases.
- Hybrid workflows: user designs missile in StrikeDesigner, chooses component solvers, StrikeCEM generates RCS table, StrikeEngine runs simulations.

---

## 13. Example Configs & Expected Outputs

### Config: PEC Sphere (test case)
```json
{
  "model": {"path":"examples/sphere.stl","units":"m"},
  "frequency": {"frequency_hz": 10e9},
  "angles": {
    "az": {"start":0,"stop":360,"step":5},
    "el": {"start":0,"stop":90,"step":5}
  },
  "solver": {"type":"PO"},
  "polarization": ["HH","VV"],
  "mesh": {"lod":"adaptive","global_h_over_lambda":0.1},
  "output": {"path":"out/sphere_rcs.csv","format":"csv"}
}
```

### Expected Output (CSV excerpt)
```
azimuth,elevation,frequency,polarization,rcs_dBsm
0,0,1e10,HH,10.02
0,0,1e10,VV,10.01
5,0,1e10,HH,9.98
5,0,1e10,VV,9.97
...
```
- For a large PEC sphere: RCS ~ constant ≈ πr² (in m²). In dBsm this is roughly stable across angles.

### Config: Flat Plate (1m², normal incidence)
```json
{
  "model": {"path":"examples/plate.stl","units":"m"},
  "frequency": {"frequency_hz": 10e9},
  "angles": {
    "az": {"start":-90,"stop":90,"step":1},
    "el": {"start":0,"stop":0,"step":1}
  },
  "solver": {"type":"PO"},
  "polarization": ["HH"],
  "output": {"path":"out/plate_rcs.csv","format":"csv"}
}
```

### Expected Output (CSV excerpt)
```
azimuth,elevation,frequency,polarization,rcs_dBsm
0,0,1e10,HH,35.0
1,0,1e10,HH,34.8
2,0,1e10,HH,34.5
...
```
- At broadside, RCS ≈ 4πA²/λ². At oblique incidence, drops rapidly.

---

## 14. Next Steps
- Finalize repo scaffold + JSON schema.
- Implement PO CPU solver + unit tests.
- Benchmark against analytic flat plate case.
- Plan GPU PO kernel (CUDA/Vulkan).

