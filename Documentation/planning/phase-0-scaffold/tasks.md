# Phase 0 — Scaffold & Toolchain · Tasks

Ordered checklist. Each task is small and leaves the repo in a buildable or
clearly-in-progress state. Check off as completed; this doubles as the execution record.

## T1 — Repository skeleton
- [x] Create directory layout: `cmake/`, `include/radar/`, `src/`, `tests/`.
- [x] Add `.gitignore` (comprehensive, security-first: secrets, keys, build/runtime output, IDE, OS, Python).
- [x] Add `LICENSE` — MIT, 2026 Guido Asbun (ADR-0003; resolved the spec open question).
- [x] Add `README.md` stub with the exact build/run command sequence.

## T2 — Dependency management
- [x] Write `cmake/Dependencies.cmake` with FetchContent for Eigen 3.4.0, GoogleTest v1.14.0, nlohmann/json v3.11.3, CLI11 v2.3.2 (pinned tags).
- [x] Mark dependency includes as SYSTEM so their warnings don't trip `-Werror` (Eigen via its own interface; GoogleTest marked SYSTEM in tests/CMakeLists + `-Wno-character-conversion` on gtest targets for Clang 21, ADR-0002).
- [x] Add `RADAR_USE_SYSTEM_DEPS` option (default OFF) stub for later Docker caching.

## T3 — Top-level CMake + presets
- [x] Write top-level `CMakeLists.txt`: `project(radar_tracker CXX)`, C++20, warning baseline (`radar_warnings` interface), include `cmake/Dependencies.cmake`, `add_subdirectory(src)`, `enable_testing()`, `add_subdirectory(tests)`.
- [x] Write `CMakePresets.json` with `debug`, `release`, `asan`, `ci`. `asan` pinned to Clang (ADR-0004). `-Werror` + `-ffp-contract=off` only in `ci`.

## T4 — libradar + trivial symbol
- [x] `include/radar/version.hpp` — declare `radar::version_string()`.
- [x] `src/version.cpp` — implement it.
- [x] `src/CMakeLists.txt` — define `radar` static library (alias `radar::radar`), apply warning flags, link Eigen.

## T5 — Smoke test
- [x] `tests/test_smoke.cpp` — link `libradar`, assert `version_string()` non-empty + semver shape.
- [x] `tests/CMakeLists.txt` — GoogleTest + `gtest_discover_tests` + `radar_add_test` helper.

## T6 — Tooling config
- [x] `.clang-format` — Google base, 100 columns.
- [x] `.clang-tidy` — enabled check groups per design; header filter for `include/radar` and `src`.

## T7 — Verify (maps to requirements ACs) — ALL PASS
- [x] AC-1/3/5 (GCC release): clean configure + build + `ctest` → 2/2 tests pass.
- [x] AC-2: `build/release/_deps` populated with all 4 deps; second configure reuses cache.
- [x] AC-3 (ci, GCC): `-Werror` build → **0 warnings**, 2/2 tests pass.
- [x] AC-4 (Clang 21, ci): `-Werror` build → **0 warnings** (after SYSTEM/`-Wno-character-conversion` on gtest), 2/2 tests pass.
- [x] AC-6 (asan, Clang): configure + build + `ctest` → sanitizer-clean, 2/2 tests pass.
- [x] AC-7: `clang-format --dry-run --Werror` passes (after formatting `version.cpp`); `clang-tidy` → 0 warnings in our code (all suppressed are non-user).
- [x] AC-8: full rebuild from deleted `build/` via README sequence → succeeds, no manual steps.

### Verification notes
- **asan under GCC failed** on RHEL 10: `libasan.so.8.0.0` runtime not installed (only a linker script present). Resolved by pinning the `asan` preset to Clang, whose compiler-rt runtime is complete and verified detecting an injected leak. See **ADR-0004**.
- **Clang 21 emitted `-Wcharacter-conversion`** from GoogleTest's own headers (not our code). Silenced narrowly on gtest targets only; our TUs compile 0-warning under both compilers. See **ADR-0002**.

## T8 — Commit
- [x] Repo already initialized; remote `origin` → github.com/guidoasbun/radar-tracker; default branch renamed `master` → `main`.
- [x] `.idea/` unstaged and git-ignored.
- [ ] Stage scaffold (no `build/`/`_deps`) and commit — pending user go-ahead.

## Exit criteria
Phase 0 is complete when every acceptance criterion in `requirements.md` is
demonstrated by the commands in T7 and the scaffold is committed (T8). At that point
we proceed to M1 starting with the `types` module.
