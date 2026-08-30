# StrikeEngine

StrikeEngine is a C++23 data-oriented multi-entity simulation library for
vehicle dynamics, sensors, navigation, seekers, guidance, earth models, and
simulation studies.

The authoritative project contracts are:

- [`docs/SPEC.md`](docs/SPEC.md) — normative behavior and capability boundary.
- [`docs/IMPLEMENTATION.md`](docs/IMPLEMENTATION.md) — source mapping, build,
  validation, and remaining work.
- [`docs/FIDELITY_AUDIT.md`](docs/FIDELITY_AUDIT.md) — measured fidelity and
  regression evidence.
- [`docs/DEFERRED_WORK.md`](docs/DEFERRED_WORK.md) — consolidated pending work
  and project boundaries.
- [`docs/STRIKEDESIGNER_INTEGRATION.md`](docs/STRIKEDESIGNER_INTEGRATION.md) —
  Designer export, validation, and provenance boundary.

Build and test:

```sh
cmake -S . -B build-linux -DCMAKE_BUILD_TYPE=Release
cmake --build build-linux --config Release -j2
ctest --test-dir build-linux -C Release --output-on-failure
```
