# Phase CI · GitHub Actions workflow — Design

## 1. Shape

One workflow file, `.github/workflows/ci.yml`, with four independent jobs that run in
parallel on `ubuntu-24.04`:

| Job | Compiler | Preset | What it proves |
| --- | --- | --- | --- |
| `gcc` | GCC | `ci` | strict `-Werror` + clang-tidy build, tests pass (the gate) |
| `clang` | Clang | `ci-clang` | same strict set under the other compiler |
| `asan` | Clang | `asan` | ASan/UBSan clean **and** asserts/death tests run (Debug, no `-DNDEBUG`) |
| `gitleaks` | — | — | unbypassable server-side secret scan |

Jobs are independent (no `needs:`) so a failure in one still reports the others, and the
matrix compilers run concurrently.

## 2. Key decisions

- **Mirror the presets, don't reinvent flags.** Each build job runs exactly
  `cmake --preset X && cmake --build --preset X && ctest --preset X`. CI must test the same
  thing developers run locally; duplicating compiler flags in YAML would let the two drift.
  This is the same single-source-of-truth principle used for the CV `F` matrix.

- **A sanitizer job that actually runs the tests (the N2 fix).** The `ci`/`ci-clang` presets
  inherit `-DNDEBUG` from `release`, so `assert(...)` is compiled out and the
  `MotionDeathTest` cases are `#ifndef NDEBUG`-skipped there. The `asan` preset is a Debug
  build, so running `ctest` under it is the only automated place the finite-input asserts
  and death tests execute. Without this job the H1 protection would be CI-invisible.

- **`asan` uses Clang (ADR-0004).** The build machine lacks GCC's sanitizer runtime; we
  standardize on Clang for parity with local. GitHub runners have both, but Clang keeps CI
  and local identical.

- **`ubuntu-24.04`, not the local RHEL versions (ADR-0002).** CI is the final arbiter of the
  matrix; its GCC/Clang may differ from the author's 14.3/21.1. That is intended — a second,
  independent toolchain is extra coverage. The `ci` preset's `-Werror` makes any
  version-specific warning a visible failure to fix, not silent drift.

- **gitleaks as a first-class, unbypassable job.** The local pre-commit hook is a
  convenience that `--no-verify` skips; this server-side job is the real control the README
  commits to. It scans full history with the tracked `.gitleaks.toml`, using the official
  `gitleaks/gitleaks-action`. `fetch-depth: 0` so history is available to scan.

- **Dependency caching.** FetchContent downloads Eigen/GoogleTest/json/CLI11 on first
  configure. Cache `build/<preset>/_deps` keyed on `cmake/Dependencies.cmake` so later runs
  skip the re-download. A cache miss is only slower, never wrong.

- **Pinned action versions and least privilege.** Actions are pinned to major-version tags
  (`actions/checkout@v4`, etc.). `permissions: contents: read` at the workflow level (the
  jobs only read the repo); nothing needs write access.

## 3. Toolchain install on the runner

`ubuntu-24.04` ships GCC, Clang, and CMake, but not always Ninja or a recent-enough CMake
by default, and the `ci` presets require clang-tidy. The jobs install what they need:
- Ninja (`ninja-build`) for all build jobs.
- `clang` + `clang-tidy` for the Clang and asan jobs (and clang-tidy for the GCC `ci` job,
  since the `ci` preset enables `RADAR_CLANG_TIDY`).
- CMake ≥ 3.25 is present on `ubuntu-24.04`; if the runner's is too old we pin via an action.
  (Checked at implementation time.)

## 4. Risks / watch items

| Risk | Mitigation |
| --- | --- |
| Runner GCC/Clang emit a warning local versions don't | That is the point of a second toolchain; `-Werror` surfaces it as a fixable failure (ADR-0002). |
| clang-tidy not on the runner → `ci` preset configure fails | Jobs `apt-get install clang-tidy`; pinned to the distro's version. |
| gitleaks action version drift | Pin the action to a tag. |
| First run slow (deps download) | `_deps` cache keyed on `Dependencies.cmake`. |
| ASan job flakes on a leak in a dependency, not our code | `asan` preset + `ASAN_OPTIONS` already tuned in `CMakePresets.json`; our code is allocation-free so findings would be real. |

## 5. Why pull this forward now (vs step 10)

The plan put `ci.yml` last, after the app runs end-to-end. But CI's value is proportional to
how much it guards, and it guards *future* commits — so the earlier it exists, the more it
protects. With randomness about to enter in `sensor` (where a seed/logic regression is easy
to miss) and the filter after that, locking the gate in now is cheap insurance. This is a
deviation from the documented ordering, recorded here and reflected in `00-overview.md`.
