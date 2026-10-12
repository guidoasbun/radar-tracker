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
│   ├── Dependencies.cmake      # FetchContent for Eigen, GoogleTest, json, CLI11 (SYSTEM, SHA-pinned)
│   └── version.hpp.in          # template: version single-sourced from PROJECT_VERSION
├── .clang-format               # Google base, 100 columns
├── .clang-tidy                 # bugprone/cert/clang-analyzer/cppcoreguidelines/modernize/...
├── .gitignore
├── LICENSE                     # MIT (ADR-0003)
├── README.md                   # build/run instructions; result plots added in M1
├── include/radar/
│   └── linalg.hpp              # symmetrize() — real Eigen-using API (Kalman P guard)
├── src/
│   ├── CMakeLists.txt          # defines the libradar static library
│   └── linalg.cpp              # Eigen-using implementation (exercises the ci Eigen path)
└── tests/
    ├── CMakeLists.txt          # GoogleTest discovery + per-target clang-tidy helper
    ├── test_smoke.cpp          # links libradar, asserts generated version symbol
    └── test_linalg.cpp         # includes <Eigen/Dense>, does matrix ops — Eigen regression guard
```

The generated `radar/version.hpp` (from `cmake/version.hpp.in`) lands under
`${PROJECT_BINARY_DIR}/generated/radar/` and is added to `libradar`'s include path.

Later phases add `include/radar/{types,motion,sensor,kalman,...}.hpp`, matching
`src/*.cpp`, `apps/*`, `scenarios/`, `tools/`, `docker/`, and `.github/`. The
`libradar` target and the `tests/` wiring created now are the templates those extend.

## 2. Build system

- **Generator: Ninja.** Chosen over Make. Rationale and tradeoffs in ADR-0001.
- **CMake ≥ 3.25** (3.31.8 present) — required for the `SYSTEM` keyword on
  `FetchContent_Declare`, which is how dependency headers become `-isystem`.
- **Out-of-source builds** under `build/<preset>/` via `CMakePresets.json` `binaryDir`.
- **`libradar` is a static library**; `apps/*` (later) and `tests/*` link it. All
  logic lives there so it is unit-testable; executables stay thin.

### Presets

| Preset | Build type | Extra flags | Purpose |
| --- | --- | --- | --- |
| `debug` | Debug | `-O0 -g` | day-to-day development; no `-Werror` |
| `release` | Release | `-O2 -DNDEBUG` + hardening | optimized; acceptance runs |
| `ci` | Release | release + `-Werror` + per-target clang-tidy (`--warnings-as-errors=*`) | the gate CI enforces (GCC) |
| `ci-clang` | Release | same as `ci`, own `binaryDir`, forces Clang | the Clang half of the matrix |
| `asan` | Debug | `-fsanitize=address,undefined,float-divide-by-zero,implicit-conversion` (**Clang**, ADR-0004) | sanitizer runs on the test suite |

`-ffp-contract=off` lives on the **base** preset (inherited by all), not just `ci`, so
local `release` runs produce the same floating-point results as CI — important for the
RMSE acceptance thresholds. clang-tidy is attached **per target** (our code only), never
globally, so it does not analyze fetched dependency sources.

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
- **`.clang-tidy`** — enables `bugprone-*`, `cert-*`, `clang-analyzer-*` (listed
  explicitly), `cppcoreguidelines-*`, `modernize-*`, `performance-*`, `readability-*`,
  `misc-*`; each disabled check carries an inline reason. Enforced per-target in the
  `ci`/`ci-clang` presets via `--warnings-as-errors=*`.
- **`.gitignore`** — security-first but deliberately NOT used as a security control
  (it only prevents accidental staging). Patterns are anchored (`/build/`, `/out/`) and
  avoid broad substring/extension globs that would silently drop source files or
  fixtures. Specific secret file types (`*.pem`, `*.key`, `.env*`, `id_rsa`, `*.tfstate`)
  are ignored; `/.idea/` is ignored (not committed). Real secret protection is delegated
  to a gitleaks hook + GitHub push protection.

## 6. The version symbol + smoke/Eigen tests

To exercise the full build→link→test path and the Eigen path specifically:
- `cmake/version.hpp.in` is configured by CMake into
  `${PROJECT_BINARY_DIR}/generated/radar/version.hpp`, single-sourcing the version from
  `project(... VERSION ...)`. `version_string()` is `constexpr`. There is no hand-written
  `version.hpp`/`version.cpp` to drift.
- `include/radar/linalg.hpp` + `src/linalg.cpp` provide `symmetrize()` (a real helper
  the Kalman covariance update needs), giving `libradar` an Eigen-using object so the
  `ci` build actually compiles Eigen under `-Werror`.
- `tests/test_smoke.cpp` links `libradar` and checks the generated version symbol.
- `tests/test_linalg.cpp` includes `<Eigen/Dense>` and does matrix operations — the
  regression guard proving Eigen is consumed as `-isystem` (it would fail to compile
  under `ci` if Eigen reverted to a plain `-I` include).

## 7. Risks / watch items for Phase 0

| Risk | Mitigation |
| --- | --- |
| Clang 21 emits warnings GCC 14 does not (newer, stricter) | `-Werror` scoped to `ci`/`ci-clang`; fix real warnings; one documented third-party suppression (ADR-0002) |
| FetchContent first build is slow | cache `build/<preset>/_deps`; one-time cost |
| A dependency SHA doesn't build on GCC 14 / Clang 21 | deps pinned by full commit SHA; record any bump here and in the overview table |
| `-Wconversion`/`-Wsign-conversion` fire inside Eigen | deps declared with the `SYSTEM` keyword → `-isystem`, so their warnings never reach our `-Werror`; verified by `test_linalg.cpp` |
| Eigen registers its own ~900-test suite into CTest | `BUILD_TESTING=OFF` set (non-FORCE) before Eigen is populated; only our tests are discovered |
| `-fsanitize=implicit-conversion` (asan preset) may fire inside Eigen/nlohmann templates used by our code — it flags *defined* behaviour (e.g. intentional unsigned wrap in hashing), and `-fno-sanitize-recover=all` makes it fatal | Watch item for M1. If third-party false positives appear, add a sanitizer **ignorelist** (`-fsanitize-ignorelist=`) scoped to the offending third-party paths rather than dropping the check. Do **not** remove `implicit-conversion` wholesale. |

## 8. Decisions recorded as ADRs

- **ADR-0001** — Ninja chosen as the CMake generator (over Make).
- **ADR-0002** — Compiler versions newer than spec; `-Werror` scoped to the `ci` preset.
- **ADR-0003** — License: MIT.
- **ADR-0004** — `asan` preset pinned to Clang (GCC sanitizer runtime missing on RHEL 10).
- **ADR-0005** — Dependency integrity via full-SHA pins; `.gitignore` is hygiene, not security.
