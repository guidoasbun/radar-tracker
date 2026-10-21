# M1 · `io` module — Requirements

## Purpose

`io` is the boundary between the outside world (scenario files, CSV output) and the pure
algorithm modules. It owns **scenario JSON → structs** (config parsing) and the **CSV
writers** `radar_sim` uses. It is the single place that validates untrusted input, so a
malformed or hostile scenario fails **loudly at load** instead of silently producing a
`NaN` trajectory that poisons every downstream number (issue #5).

Dependency direction (spec module table): `io` depends on `nlohmann/json` and `types`.
Nothing from `tracker` downward includes `io` — the algorithms compile and test with no
file code. `io` is the M1 step that unblocks `metrics` (RMSE) and `apps/radar_sim`.

## Scope

In scope (M1):

- **`io/config.hpp` + `src/io/config.cpp`** — parse a scenario JSON into a `ScenarioConfig`:
  - top level: `dt_s`, `duration_s`, `seed`, `area_half_width_m`;
  - `sensor`: `sigma_m` (M1 field). `p_detect` and `clutter_per_scan` are **parsed if
    present and carried, but unused in M1** (the sensor is noise-only until M2) — see the
    deferred note;
  - `tracker`: `sigma_a`, `gate_chi2`, `confirm_m`, `confirm_n`, `delete_after_misses`
    (carried for M2; `sigma_a` is used by the M1 filter config);
  - `targets[]`: `id`, `x0`, `y0`, `vx0`, `vy0`, `maneuvers[]` (the maneuver list is
    **parsed but must be empty in M1** — CT is deferred, ADR-0006 / motion design).
- **`io/csv.hpp` + `src/io/csv.cpp`** — CSV **writers** for the two files `radar_sim`
  produces:
  - `truth.csv`: `t_s, target_id, x_m, y_m, vx_mps, vy_mps`;
  - `meas.csv`: `t_s, meas_id, x_m, y_m, origin_id` (`origin_id` is the target id or −1 for
    clutter).
  Header row, SI units, deterministic fixed formatting.
- **`scenarios/single.json`** — the one M1 scenario (a single straight-line CV target), so
  there is a real config to load and, later, run end to end.
- A dedicated error type **`radar::ConfigError`** (see ADR-0007) thrown on any invalid
  scenario, carrying a message that names the offending JSON path (maps to the spec's
  exit code 1, "the message names the JSON path").

Out of scope / deferred (documented so the contract is explicit):

- **CSV readers** — only `radar_score`/`metrics` consume CSVs, and `tracks.csv` needs the
  tracker (M2). Readers are added when `metrics`/`radar_score` first needs them; the spec's
  `io/csv.hpp` "writers and readers" is satisfied incrementally. (Recorded in `00-overview.md`
  and the module table note.)
- **`tracks.csv` writer** — needs `TrackStatus`/`Track` lifecycle from the tracker (M2).
- **`sensor.p_detect`, `sensor.clutter_per_scan`** — parsed/carried but unused until M2.
- **`maneuvers`** — parsed but rejected if non-empty in M1 (no CT model yet).
- **UDP / `net`** — M3.

## Acceptance criteria

### AC-1 — Valid scenario parses to the expected struct
`load_scenario("scenarios/single.json")` returns a `ScenarioConfig` whose fields equal the
file's values exactly (checked against literals): `dt_s`, `duration_s`, `seed`,
`area_half_width_m`, `sensor.sigma_m`, the `tracker` block, and one target with its
`id`/`x0`/`y0`/`vx0`/`vy0`.

### AC-2 — Issue #5 validation (fail loudly at the boundary)
All of the following throw `radar::ConfigError` with a message naming the offending field
(no leading dot for a top-level key; `<root>` for a non-object root), and are each covered
by a test (`EXPECT_THROW`), so they run in every build:

- Non-finite (`NaN`/`inf`) or **non-positive** `dt_s`. (nlohmann rejects `NaN`/`Infinity`
  tokens and overflowing literals like `1e400` at parse time; the field-level `isfinite`
  guard is documented defense-in-depth.)
- Non-finite `duration_s` or `area_half_width_m`; non-positive `duration_s`.
- Non-finite target initial state (any of `x0`, `y0`, `vx0`, `vy0`).
- Non-finite or **negative** `sensor.sigma_m` (a negative/NaN σ_m would otherwise disable
  noise — a silent "perfect sensor").
- **Negative** target `id` (`id = −1` collides with `radar::kClutterOrigin`, scoring a real
  target as clutter — the check references the named constant).
- **Duplicate** target ids (origin must uniquely identify a target for scoring).
- A non-empty `maneuvers` list on any target (CT deferred to M2).
- **(PR #14 review) An integer outside the 32-bit `int` range** — read as `int64` and
  range-checked before narrowing, so e.g. `"id": 4294967297` is rejected rather than
  silently wrapping to `1`.
- **(PR #14 review) A present-but-wrong-typed `sensor`/`tracker` block** (`0`, `"x"`,
  `[ … ]`) — rejected, not silently replaced by defaults.
- **(PR #14 review) Out-of-range M2 values** carried for later use: `sensor.p_detect` ∉
  [0, 1], `sensor.clutter_per_scan` < 0, `tracker.gate_chi2` ≤ 0, `confirm_m` not in
  `1 ≤ confirm_m ≤ confirm_n`, `delete_after_misses` < 1.

A full-range unsigned `seed` (up to 2⁶⁴−1) is **accepted** (`ScenarioConfig::seed` is
`uint64`), not misread as negative.

### AC-3 — Malformed / missing input is a clear ConfigError, not a crash
- A missing file, invalid JSON syntax, a missing required key, or a wrong JSON type for a
  key throws `radar::ConfigError` (never an uncaught `nlohmann::json` exception, never a
  crash). The message names the file and/or the JSON key path.

### AC-4 — CSV writers produce the exact spec format
- `write_truth` emits the header `t_s,target_id,x_m,y_m,vx_mps,vy_mps` then one row per
  target per scan; `write_meas` emits `t_s,meas_id,x_m,y_m,origin_id`.
- Columns are comma-separated, SI units, with deterministic fixed-precision formatting.
  **Every numeric field — integers included — is written with `std::to_chars`**, which is
  locale-independent by construction (a `.` decimal point, never a thousands separator),
  so an id like `1000` can never print as `1,000` and split a column even if the app imbues
  or sets a global locale. Verified by writing to an in-memory stream (including under an
  `en_US.UTF-8` locale) and comparing the exact text against a literal.
- Round-trip sanity: a value written and re-parsed as a double recovers it to tolerance
  (guards the formatting precision).

### AC-5 — Tooling + tested
- `clang-format --dry-run --Werror` clean; `ci`/`ci-clang` 0 warnings in our code
  (nlohmann/json consumed as a SYSTEM include so its headers do not trip our `-Werror`).
- `tests/test_io.cpp` registered via `radar_add_test`; green under `release`, `ci`,
  `ci-clang`, and `asan`.

## Notes

- `ConfigError` is a dedicated exception rather than the `std::invalid_argument` the pure
  modules throw; the rationale (error path naming + the spec's exit-code-1 mapping, while
  keeping the one-way dependency clean) is **ADR-0007**.
- The validation here is the release-build guard that backs `motion`'s debug-only
  finite-input assert (issue #5): `io` rejects what `propagate_cv` merely asserts.
