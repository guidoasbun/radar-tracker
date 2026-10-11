# Radar Tracker

A C++20 multi-target radar tracking system: a simulated radar produces noisy,
cluttered detections of several aircraft, and a tracker recovers each aircraft's
position and velocity with Kalman filters, scored against hidden ground truth.

See [`Documentation/Radar Tracker — Project Specification.md`](Documentation/Radar%20Tracker%20—%20Project%20Specification.md)
for the full specification, and [`Documentation/planning/`](Documentation/planning/)
for the implementation plan (phase by phase).

> Status: **Phase 0 — scaffold & toolchain.** No tracking logic yet; the build
> system, dependency management, and test harness are in place.

## Requirements

- CMake ≥ 3.24 and Ninja
- A C++20 compiler (GCC ≥ 12 or Clang ≥ 15)
- Internet access on first configure (dependencies are fetched via FetchContent)

Dependencies (Eigen, GoogleTest, nlohmann/json, CLI11) are fetched automatically
and pinned to Git tags — no manual installation required.

## Build and test

```bash
# Configure + build (release), then run the tests
cmake --preset release
cmake --build --preset release
ctest --preset release --output-on-failure
```

Available presets:

| Preset | Purpose |
| --- | --- |
| `debug` | `-O0 -g`, day-to-day development |
| `release` | `-O2 -DNDEBUG`, optimized |
| `asan` | AddressSanitizer + UndefinedBehaviorSanitizer on the test suite |
| `ci` | release + `-Werror` + `-ffp-contract=off` (the gate CI enforces) |

Build with Clang instead of GCC:

```bash
CC=clang CXX=clang++ cmake --preset ci
cmake --build --preset ci
```

## License

[MIT](LICENSE) © 2026 Guido Asbun
