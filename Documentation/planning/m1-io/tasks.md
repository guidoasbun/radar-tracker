# M1 · `io` module — Tasks

> **Status: 🔶 in progress.**

## T1 — Planning docs + ADR
- [x] Branch `m1-io` off `main`.
- [x] `requirements.md` — scope (writers-only, deferrals), AC-1..5, issue #5 battery.
- [x] `design.md` — structs, `ConfigError`, json integration, CSV writers, verification.
- [x] `tasks.md` — this checklist.
- [x] `ADR-0007` — dedicated `ConfigError` vs `std::invalid_argument` (decision recorded).

## T2 — config: header + source
- [x] `include/radar/io/config.hpp`: `ScenarioConfig`/`SensorSpec`/`TrackerSpec`/`TargetSpec`/
      `ManeuverSpec` structs; `class ConfigError : std::runtime_error`; `load_scenario(path)`
      and `parse_scenario(text, name)` declarations. Does **not** include `<nlohmann/json.hpp>`
      (json stays an implementation detail).
- [x] `src/io/config.cpp`: parse via nlohmann; every nlohmann access wrapped so a missing key /
      wrong type / syntax error becomes a `ConfigError` naming the path. Issue #5 validation on
      every field.

## T3 — csv: header + source
- [x] `include/radar/io/csv.hpp`: `write_truth_header/row`, `write_meas_header/row` over
      `std::ostream&`.
- [x] `src/io/csv.cpp`: fixed, locale-independent precision; exact spec columns.

## T4 — Build wiring
- [x] Add `io/config.cpp`, `io/csv.cpp` to the `radar` library in `src/CMakeLists.txt`.
- [x] Link `nlohmann_json::nlohmann_json` into `radar` (PRIVATE — not in the public API).
- [x] Register `test_io` in `tests/CMakeLists.txt` via `radar_add_test`.

## T5 — Scenario file
- [x] `scenarios/single.json` — one straight-line CV target (matches the spec example shape,
      one target, empty maneuvers).

## T6 — Tests (issue #5 is the headline)
- [x] Valid parse vs literals (AC-1).
- [x] Issue #5 `EXPECT_THROW(ConfigError)` battery: NaN/inf/≤0 `dt_s`; non-finite state;
      non-finite/≤0 `duration_s`/`area`; bad `sigma_m`; negative id; duplicate ids;
      non-empty maneuvers (AC-2).
- [x] Malformed: missing file, bad syntax, missing key, wrong type → `ConfigError` (AC-3).
- [x] CSV exact-format + round-trip precision (AC-4).
- [x] `scenarios/single.json` loads and validates.

## T7 — Verify — ALL PASS
- [x] `release` → `ctest` green.
- [x] `ci` (GCC) + `ci-clang` 0 warnings in our code; full suite green.
- [x] `asan` green.
- [x] `clang-format --dry-run --Werror` clean.

## T8 — Docs sync (decision: document the changes everywhere)
- [x] `00-overview.md`: mark M1 order item 6 (`io`) in progress → done; note the **writers-only
      deferral** (readers + `tracks.csv` come with `metrics`/tracker) and the **ConfigError /
      ADR-0007** decision.
- [x] `ADR-0007` committed under `Documentation/decisions/`.
- [x] README: no status change until the app runs; confirmed the dependency line is accurate
      — `nlohmann/json` is now actually used by `io` (CLI11 still staged for `apps/*`).

## T9 — Ship
- [ ] Single clean commit on `m1-io`; push; open PR; 4 required checks green.

## Exit criteria
All ACs in `requirements.md` met; `io/{config,csv}.{hpp,cpp}` + `test_io.cpp` +
`scenarios/single.json` committed and green across all presets; ADR-0007 recorded; docs
synced. Unblocks `metrics` (RMSE) and `apps/radar_sim` (the first runnable app).
