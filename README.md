# StrikeEngine

StrikeEngine is a C++23 data-oriented multi-entity simulation library for
vehicle dynamics, sensors, navigation, seekers, guidance, earth models, and
simulation studies.

The authoritative project contracts are:

- [`docs/SPEC.md`](docs/SPEC.md) — normative behavior and capability boundary.
- [`docs/IMPLEMENTATION.md`](docs/IMPLEMENTATION.md) — source mapping, build,
  validation, and remaining work.

Build and test:

```sh
cmake -S . -B build-linux -DCMAKE_BUILD_TYPE=Release
cmake --build build-linux --config Release -j2
ctest --test-dir build-linux -C Release --output-on-failure
```
