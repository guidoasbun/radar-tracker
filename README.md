# Radar Tracker

A C++20 multi-target radar tracking system: a simulated radar produces noisy,
cluttered detections of several aircraft, and a tracker recovers each aircraft's
position and velocity with Kalman filters, scored against hidden ground truth.

See [`Documentation/Radar Tracker — Project Specification.md`](Documentation/Radar%20Tracker%20—%20Project%20Specification.md)
for the full specification, and [`Documentation/planning/`](Documentation/planning/)
for the implementation plan (phase by phase).

> Status: **M1 in progress.** The constant-velocity Kalman filter — the keystone the
> whole project depends on — is built and statistically verified, and the scenario/CSV
> I/O boundary is in place. There is **no end-to-end application yet**: the `metrics`
> (RMSE) and `apps/radar_sim` layers that wire a scenario from file to output are the
> remaining M1 steps. Today the build produces the `libradar` static library and a
> **89-test** suite (GoogleTest + CTest); see [What's implemented](#whats-implemented).

## What's implemented

Everything below lives in the single static library `libradar` (one header per module
under `include/radar/`). The algorithm modules have no file or socket dependencies, so
they build and test in isolation.

| Module | What it does | Status |
| --- | --- | --- |
| `types` | Shared plain structs: `Measurement`, `Scan`, `TrackState`, `TrackStatus`, state-vector indices; origin-isolation enforced by the type system | ✅ complete (PR #3) |
| `motion` | Constant-velocity truth propagation (`cv_transition_matrix`, `propagate_cv`); coordinated-turn model deferred to M2 | ✅ complete (PR #4) |
| `sensor` | Noise-only sensor slice: Gaussian measurement noise, hidden origin tags, order shuffle, seeded RNG; P_D / clutter deferred to M2 | ✅ complete (PR #10) |
| `kalman` | CV linear Kalman filter: predict, Joseph-form update, innovation + covariance S, Mahalanobis gate distance, one-point track init; Cholesky SPD guards | ✅ complete (PR #11) |
| **Phase K** | Exhaustive statistical filter-verification battery — NEES, NIS (×2), covariance credibility, and convergence over a fixed-seed Monte-Carlo, model-matched truth so NEES is a genuine χ²(4) consistency test | ✅ complete (PR #12) |
| `io` | Scenario JSON → validated `ScenarioConfig` (fails loudly at the boundary via a dedicated `ConfigError`), plus locale-independent CSV **writers** for `truth.csv` / `meas.csv`; CSV readers + `tracks.csv` deferred to when `metrics`/tracker need them | ✅ complete (PR #14) |
| CI / toolchain | Reproducible CMake-preset build, four-job GitHub Actions gate, gitleaks secret scanning | ✅ complete (PR #2, #7) |

Still to come in M1: `metrics` (RMSE), `apps/radar_sim` (the first runnable executable),
and the README plot. M2 adds multi-target tracking, clutter, association, and the track
lifecycle; M3 adds the two-service UDP split; M4 the optional live display. See
[`Documentation/planning/00-overview.md`](Documentation/planning/00-overview.md) for the
full phase map and [`Documentation/decisions/`](Documentation/decisions/) for the ADRs.

### What you can run today

There is no `radar_sim` binary yet, so the runnable artifact is the test suite:

```bash
cmake --preset release
cmake --build --preset release
ctest --preset release --output-on-failure    # 89 tests
```

Scenario loading is wired and testable now — `scenarios/single.json` (one straight-line
CV target) parses and validates through `radar::load_scenario`, exercised by `test_io`.

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
| `gitleaks (secret scan)` | server-side secret scan with the same pinned, checksum-verified gitleaks as the local hook (`.gitleaks.toml`) — scans all commits reachable from HEAD on push/PR and **all history** (all refs) on a weekly schedule; cannot be skipped with `--no-verify` |

The build jobs run exactly the local presets (`cmake --preset … && cmake --build … &&
ctest --preset …`), so CI and local developer runs cannot drift.

## License

[MIT](LICENSE) © 2026 Guido Asbun
