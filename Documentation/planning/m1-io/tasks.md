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
- [x] Single clean commit on `m1-io`; push; open PR #14; 4 required checks green.

## T10 — PR #14 review (9 findings, addressed; each reverified green)
- [x] #1 integers read as `int64` + range-checked before narrowing (`id`, tracker ints) —
      `4294967297` no longer wraps to `1`. Test: `RejectsOutOfRangeInteger`.
- [x] #2 `seed` reads the full unsigned range (`get<uint64>` when `is_number_unsigned`) —
      `18446744073709551615` accepted. Test: `AcceptsFullRangeUnsignedSeed`.
- [x] #3 wrong-typed `sensor`/`tracker` block rejected via `optional_object()`. Test:
      `RejectsWrongTypedSensorOrTrackerBlock`.
- [x] #4 range checks: `p_detect∈[0,1]`, `clutter_per_scan≥0`, `gate_chi2>0`,
      `1≤confirm_m≤confirm_n`, `delete_after_misses≥1`. Tests: `RejectsOutOfRange…` +
      `AcceptsInRange…`.
- [x] #5 + #6 CSV rewritten on `std::to_chars` for **all** numeric fields (locale-independent
      incl. integers; no per-field `imbue` churn). Test: `IntegerColumnsAreLocaleIndependent`.
- [x] #7 error paths have no leading dot for top-level keys; non-object root names `<root>`.
      Tests: `TopLevelErrorPathHasNoLeadingDot`, `NonObjectRootNamesRoot`.
- [x] #8 `load_scenario` rejects a directory up front (`std::filesystem::is_directory`) with
      "is a directory", not a misleading "invalid JSON" (a dir read leaves no stream error on
      Linux — verified). Test: `DirectoryPathReportsReadError`.
- [x] #9 negative-id check + message reference `radar::kClutterOrigin`.
- [x] clang-tidy: scoped NOLINTs for the `std::to_chars` pointer-arithmetic idiom and the
      swappable message/row params; test multi-case blocks refactored through an
      `expect_config_error` helper to stay under the cognitive-complexity threshold.
- [x] UBSan: libstdc++ `<charconv>` added to `cmake/sanitizer-ignorelist.txt` — `to_chars`
      on a negative `int` (origin_id −1) converts to unsigned internally (correct, flagged by
      implicit-conversion); same third-party-false-positive treatment as the Eigen entry.
- [x] Re-verified: `release`/`ci`/`ci-clang` 87/87, `asan` 89/89, `clang-format` clean.

## Exit criteria
All ACs in `requirements.md` met; `io/{config,csv}.{hpp,cpp}` + `test_io.cpp` +
`scenarios/single.json` committed and green across all presets; ADR-0007 recorded; docs
synced. Unblocks `metrics` (RMSE) and `apps/radar_sim` (the first runnable app).
