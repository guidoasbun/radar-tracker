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

- CMake ≥ 3.25 and Ninja
- A C++20 compiler. The build matrix is **GCC ≥ 12** (`ci` preset) and
  **Clang ≥ 15** (`ci-clang` preset). The `asan` preset requires Clang.
- Internet access on first configure (dependencies are fetched via FetchContent)

Dependencies (Eigen, GoogleTest, nlohmann/json, CLI11) are fetched automatically and
pinned to full commit SHAs — no manual installation required.

> On first configure you may see CMake *deprecation warnings* originating from
> CLI11's and Eigen's own `cmake_minimum_required`. These come from the
> dependencies, not this project, and are expected/harmless.

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
| `release` | `-O2 -DNDEBUG`, optimized + hardening |
| `ci` | release + `-Werror` + clang-tidy (GCC) — the gate CI enforces |
| `ci-clang` | same as `ci` but built with Clang (own build dir) |
| `asan` | AddressSanitizer + UndefinedBehaviorSanitizer on the test suite (Clang) |

Build with Clang instead of GCC — use the dedicated preset (it has its own build
directory and pins the compiler, so it never silently reuses a GCC-configured tree):

```bash
cmake --preset ci-clang
cmake --build --preset ci-clang
ctest --preset ci-clang --output-on-failure
```

## Secret scanning (git hooks)

This repo ships a [gitleaks](https://github.com/gitleaks/gitleaks) pre-commit hook
that scans staged changes for secrets and blocks the commit if one is found.
After cloning, activate it once:

```bash
./scripts/setup-hooks.sh
```

This points `core.hooksPath` at the tracked `.githooks/` directory and checks that
`gitleaks` is installed (the script prints a checksum-verified install command if
it is not). Hook config lives in `.gitleaks.toml`.

- Bypass a single commit only when you are certain a finding is a false positive:
  `git commit --no-verify` — and prefer adding an allowlist entry to `.gitleaks.toml`.
- The hook is local and can be skipped, so it is a convenience, not the last line of
  defence. GitHub **push protection + secret scanning** are enabled on the repo, and a
  **gitleaks CI job** (which cannot be bypassed) runs on every push and pull request — see
  [Continuous integration](#continuous-integration) below.

## Continuous integration

Every push and pull request to `main` runs [`.github/workflows/ci.yml`](.github/workflows/ci.yml)
on `ubuntu-24.04`:

| Job | What it enforces |
| --- | --- |
| `gcc (ci preset)` | release + `-Werror` + clang-tidy build and full test suite (GCC) |
| `clang (ci-clang preset)` | the same strict gate under Clang |
| `asan (Clang, ASan/UBSan)` | sanitizer-clean **and** runs the test suite as a Debug build, so `assert`-based preconditions and death tests (compiled out of the `ci` presets by `-DNDEBUG`) actually execute |
| `gitleaks (secret scan)` | server-side, full-history secret scan with `.gitleaks.toml` — cannot be skipped with `--no-verify`, unlike the local hook |

The build jobs run exactly the local presets (`cmake --preset … && cmake --build … &&
ctest --preset …`), so CI and local developer runs cannot drift.

## License

[MIT](LICENSE) © 2026 Guido Asbun
