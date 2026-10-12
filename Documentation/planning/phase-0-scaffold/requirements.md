# Phase 0 — Scaffold & Toolchain · Requirements

## Purpose

Prove the project's skeleton, build system, and dependency management work on a
clean machine **before any domain code is written**. Phase 0 produces no tracking
logic — it produces a repository that configures, builds, and runs a trivial test
under every preset, so that from M1 onward every change lands on a known-good base.

## Scope

In scope:
- Repository directory layout per the spec's "Repository layout" section.
- CMake project with `CMakePresets.json` (debug, release, asan, ci).
- Dependency acquisition via FetchContent (Eigen, GoogleTest, nlohmann/json, CLI11), each pinned to a Git tag.
- Baseline compiler warning flags and C++20 standard enforcement.
- Tooling config files: `.clang-format`, `.clang-tidy`, `.gitignore`, `LICENSE` placeholder, `README.md` stub.
- One trivial `libradar` symbol and one trivial GoogleTest to exercise the build+test+link path.
- Git repository initialized with an initial commit.

Out of scope (deferred to later phases):
- Any tracking, sensor, motion, Kalman, I/O, or networking logic.
- The GitHub Actions workflow file (created in M1, though the `ci` preset exists now).
- Docker, plotting, lcov, and their installs.

## Acceptance criteria

Each criterion is objectively checkable. Phase 0 is done when all pass.

### AC-1 — Clean configure, all presets
On a fresh checkout with no `build/` directory:
- `cmake --preset debug` configures with no errors.
- `cmake --preset release` configures with no errors.
- `cmake --preset asan` configures with no errors.
- `cmake --preset ci` configures with no errors.

### AC-2 — Dependencies resolve via FetchContent
- The four dependencies (Eigen 3.4, GoogleTest 1.14, nlohmann/json 3.11, CLI11 2.3) are fetched by Git tag into `build/_deps` with no manual installation.
- A second configure reuses the cache (no re-download).

### AC-3 — Clean build under all presets
- `cmake --build --preset debug` builds `libradar` and the trivial test with zero errors.
- `cmake --build --preset release` builds with zero errors.
- `cmake --build --preset asan` builds with zero errors.
- `cmake --build --preset ci` builds with zero errors **and zero warnings** (the `ci` preset uses `-Werror`).

### AC-4 — Both compilers
- The project configures and builds with **GCC** (g++ 14) using the `ci` preset.
- The project configures and builds with **Clang** (clang++ 21) using the `ci-clang`
  preset (its own compiler cache vars and `binaryDir`, so it never silently reuses a
  GCC-configured build directory).
- **Our code** (`radar` library + tests, including a translation unit that includes
  `<Eigen/Dense>` and performs matrix operations) builds with **zero warnings** under
  both compilers with the full `-Werror` set. Any Clang-21-specific warning in our code
  is fixed, not suppressed.
- **Third-party code is exempt**: dependencies are consumed as SYSTEM includes
  (`-isystem`), so their warnings do not reach our `-Werror`. One narrow, documented
  suppression applies to GoogleTest's own targets only (see ADR-0002).

### AC-5 — Trivial test passes
- `ctest --preset debug` runs the trivial test and reports 100% passed.
- `ctest --preset release` likewise.
- The trivial test links against `libradar` and calls one trivial library symbol, proving the lib→test link path works.

### AC-6 — ASan/UBSan clean on the trivial path
- `ctest --preset asan` runs the trivial test with AddressSanitizer + UndefinedBehaviorSanitizer and reports no sanitizer findings.

### AC-7 — Tooling configs valid
- `clang-format --dry-run --Werror` over the (minimal) sources passes.
- `clang-tidy` runs over the trivial translation unit with no errors (warnings reviewed; config present).
- `.gitignore` excludes `build/`, editor dirs, and generated artifacts.

### AC-8 — Reproducible from scratch
- Deleting `build/` and repeating `cmake --preset release && cmake --build --preset release && ctest --preset release` succeeds with no manual steps.
- The exact command sequence is recorded in `README.md`.

### AC-9 — Version control
- Git repository initialized; `.gitignore` in place so `build/` and `_deps` are never committed.
- Initial commit contains the scaffold only (no fetched dependency sources).

## Explicitly verifiable commands

These are the commands used to demonstrate the ACs (recorded here so verification is repeatable):

```bash
# AC-1..AC-5, GCC
rm -rf build
cmake --preset release && cmake --build --preset release && ctest --preset release --output-on-failure
cmake --preset ci      && cmake --build --preset ci      # zero warnings expected

# AC-4, Clang
rm -rf build
CC=clang CXX=clang++ cmake --preset ci && cmake --build --preset ci

# AC-6, sanitizers
cmake --preset asan && cmake --build --preset asan && ctest --preset asan --output-on-failure

# AC-7, tooling
clang-format --dry-run --Werror $(git ls-files '*.hpp' '*.cpp')
```

## Non-functional notes

- First configure will download dependency sources; subsequent configures must be offline-capable from the cache.
- Build output stays under `build/<preset>/` and is never committed.
- No network calls at build time beyond the one-time FetchContent download.
