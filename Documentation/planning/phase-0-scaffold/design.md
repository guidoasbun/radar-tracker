# Phase 0 — Scaffold & Toolchain · Design

Phase 0 is small, so this design is deliberately light: it records the structure we
commit to and the decisions we'll live with for the whole project. Decisions with
lasting impact also get a standalone ADR under `../../decisions/`.

## 1. Repository layout

We follow the spec's layout, creating only what Phase 0 needs now and leaving
placeholder directories for later phases so the structure is visible from the start.

```text
radar-tracker/
├── CMakeLists.txt              # top-level: project(), options, add_subdirectory
├── CMakePresets.json           # debug, release, asan, ci
├── cmake/
│   └── Dependencies.cmake      # FetchContent for Eigen, GoogleTest, json, CLI11
├── .clang-format               # Google base, 100 columns
├── .clang-tidy                 # bugprone/cppcoreguidelines/modernize/... 
├── .gitignore
├── LICENSE                     # placeholder until license decided (spec open question)
├── README.md                   # build/run instructions; result plots added in M1
├── include/radar/
│   └── version.hpp             # single trivial public symbol for Phase 0
├── src/
│   ├── CMakeLists.txt          # defines the libradar static library
│   └── version.cpp             # trivial implementation
└── tests/
    ├── CMakeLists.txt          # GoogleTest discovery
    └── test_smoke.cpp          # trivial test: links libradar, asserts version symbol
```

Later phases add `include/radar/{types,motion,sensor,kalman,...}.hpp`, matching
`src/*.cpp`, `apps/*`, `scenarios/`, `tools/`, `docker/`, and `.github/`. The
`libradar` target and the `tests/` wiring created now are the templates those extend.

## 2. Build system

- **Generator: Ninja.** Chosen over Make. Rationale and tradeoffs in ADR-0001.
- **CMake ≥ 3.24** (3.31.8 present) for presets and modern FetchContent.
- **Out-of-source builds** under `build/<preset>/` via `CMakePresets.json` `binaryDir`.
- **`libradar` is a static library**; `apps/*` (later) and `tests/*` link it. All
  logic lives there so it is unit-testable; executables stay thin.

### Presets

| Preset | Build type | Extra flags | Purpose |
| --- | --- | --- | --- |
| `debug` | Debug | `-O0 -g` | day-to-day development; no `-Werror` |
| `release` | Release | `-O2 -DNDEBUG` | optimized; acceptance runs |
| `asan` | Debug | debug + `-fsanitize=address,undefined -fno-omit-frame-pointer` (**pinned to Clang**, see ADR-0004) | sanitizer runs on the test suite |
| `ci` | Release | release + `-Werror` + `-ffp-contract=off` + clang-tidy hook | the gate CI enforces |

`-ffp-contract=off` in `ci` is from the spec — it reduces cross-compiler
floating-point divergence so the acceptance thresholds behave consistently on GCC
and Clang.

## 3. Compiler warning baseline

Applied to `libradar`, `apps`, and `tests` (not to fetched dependency targets):

```
-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
-Wnon-virtual-dtor -Wold-style-cast
```

`-Werror` is added **only in the `ci` preset**, not in `debug`. This keeps the inner
dev loop productive while making the CI gate strict. See ADR-0002 for why this
matters specifically with Clang 21 (much newer than the spec's assumed Clang 15).

Dependency headers are included as **SYSTEM** includes so their warnings do not trip
our `-Werror`.

## 4. Dependencies (FetchContent, pinned by tag)

`cmake/Dependencies.cmake` fetches each by an explicit Git tag — never a branch — so
builds are reproducible. Target pins (confirmed/adjusted during execution):

| Dependency | Tag | Spec minimum | Use |
| --- | --- | --- | --- |
| Eigen | `3.4.0` | 3.4 | fixed-size linear algebra |
| GoogleTest | `v1.14.0` | 1.14 | unit tests + CTest |
| nlohmann/json | `v3.11.3` | 3.11 | scenario config parsing |
| CLI11 | `v2.3.2` | 2.3 | argument parsing |

Notes:
- Eigen is header-only; we link its interface target and mark includes SYSTEM.
- `-DRADAR_USE_SYSTEM_DEPS=ON` option is stubbed now (switches to `find_package`)
  for Docker layer caching later; default OFF uses FetchContent.
- `build/_deps` is cached and git-ignored.

## 5. Tooling configuration

- **`.clang-format`** — Google base style, 100-column limit (per spec).
- **`.clang-tidy`** — enables `bugprone-*`, `cppcoreguidelines-*`, `modernize-*`,
  `performance-*`, `readability-*`, `misc-*`; any disabled check carries an inline
  reason. In Phase 0 the only code is trivial, so this mainly establishes the file.
- **`.gitignore`** — ignores `build/`, `.idea/` is already present, plus common
  editor/OS artifacts and Python `__pycache__`.

## 6. The trivial symbol + smoke test

To exercise the full build→link→test path without domain logic:
- `include/radar/version.hpp` declares `radar::version_string()` returning a
  `std::string_view` compile-time constant.
- `src/version.cpp` implements it (so the static library has at least one object).
- `tests/test_smoke.cpp` links `libradar`, calls `radar::version_string()`, and
  asserts it is non-empty. This proves: the library compiles, the test links against
  it, GoogleTest is wired to CTest, and sanitizers run clean on a real executable.

## 7. Risks / watch items for Phase 0

| Risk | Mitigation |
| --- | --- |
| Clang 21 emits warnings GCC 14 does not (newer, stricter) | `-Werror` scoped to `ci`; fix real warnings, document any targeted suppression (ADR-0002) |
| FetchContent first build is slow | cache `build/_deps`; this is a one-time cost |
| A dependency tag doesn't build on GCC 14 / Clang 21 | pin to the nearest known-good tag; record any bump here and in the overview table |
| `-Wconversion`/`-Wsign-conversion` noisy with Eigen | Eigen headers marked SYSTEM so its internal conversions don't trip us |

## 8. Decisions recorded as ADRs

- **ADR-0001** — Ninja chosen as the CMake generator (over Make).
- **ADR-0002** — Compiler versions newer than spec; `-Werror` scoped to the `ci` preset.
- **ADR-0003** — License: MIT.
- **ADR-0004** — `asan` preset pinned to Clang (GCC sanitizer runtime missing on RHEL 10).
