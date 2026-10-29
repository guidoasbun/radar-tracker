# M1 · `io` module — Design

## 1. Shape

Two headers under `include/radar/io/` and two sources under `src/io/`, added to
`libradar`:

- `io/config.hpp` + `src/io/config.cpp` — `ScenarioConfig` structs + `load_scenario`.
- `io/csv.hpp` + `src/io/csv.cpp` — `write_truth` / `write_meas`.

A dedicated `radar::ConfigError` (in `io/config.hpp`) is thrown for every invalid-scenario
case (ADR-0007). `io` depends on `types` (for `LabeledMeasurement`, state indices) and
`nlohmann/json`; it is below nothing in the algorithm graph (the one-way rule holds — no
algorithm module includes `io`).

```cpp
namespace radar {

struct ManeuverSpec {            // parsed; must be empty in M1 (CT deferred, ADR-0006)
  double t_start_s{0.0};
  double t_end_s{0.0};
  double turn_rate_deg_s{0.0};
};

struct TargetSpec {
  int id{0};
  double x0{0.0}, y0{0.0}, vx0{0.0}, vy0{0.0};
  std::vector<ManeuverSpec> maneuvers;  // empty in M1
};

struct SensorSpec {
  double sigma_m{50.0};
  // M2 (parsed if present, carried, unused in M1):
  double p_detect{0.9};
  double clutter_per_scan{2.0};
};

struct TrackerSpec {             // carried for M2; sigma_a feeds the M1 KalmanConfig
  double sigma_a{5.0};
  double gate_chi2{9.21};
  int confirm_m{3}, confirm_n{5}, delete_after_misses{5};
};

struct ScenarioConfig {
  double dt_s{0.1};
  double duration_s{60.0};
  std::uint64_t seed{42};
  double area_half_width_m{20000.0};
  SensorSpec sensor;
  TrackerSpec tracker;
  std::vector<TargetSpec> targets;
};

class ConfigError : public std::runtime_error { using std::runtime_error::runtime_error; };

[[nodiscard]] ScenarioConfig load_scenario(const std::filesystem::path& file);
[[nodiscard]] ScenarioConfig parse_scenario(std::string_view json_text,
                                            std::string_view source_name = "<memory>");

// CSV writers — stream-based so tests write to an ostringstream, apps to a file.
void write_truth_header(std::ostream&);
void write_truth_row(std::ostream&, double t_s, int target_id,
                     const Eigen::Vector4d& state);
void write_meas_header(std::ostream&);
void write_meas_row(std::ostream&, double t_s, int meas_id,
                    const Eigen::Vector2d& pos, int origin_id);

}  // namespace radar
```

## 2. Key decisions

- **Parse via a string, validate into structs, then hand back a plain value.** `load_scenario`
  reads the file and delegates to `parse_scenario(text, name)`; the latter is what tests
  drive directly with literals (no temp files needed for the validation battery). All
  nlohmann access goes through `.at()` / typed getters wrapped so a missing key or wrong
  type becomes a `ConfigError` naming the path, never a leaked `nlohmann::json::exception`.

- **Validate once, at the boundary, fail loudly (issue #5).** Every numeric field is checked
  for finiteness; `dt_s`/`duration_s` for positivity; `sigma_m` for non-negativity; target
  ids for non-negativity and uniqueness; `maneuvers` for emptiness. The first failure throws
  `ConfigError` with a message like `single.json: targets[1].vx0 is not finite`. This is the
  release-build guard that backs `motion`'s debug-only assert: `io` rejects what
  `propagate_cv` only asserts.

- **`ConfigError`, not `std::invalid_argument` (ADR-0007).** The pure modules throw
  `std::invalid_argument` for programmer error; a bad *scenario file* is user/operator input
  that `apps/*` must map to **exit code 1 with the JSON path named** (spec CLI table).
  A distinct type lets `main` `catch (const ConfigError&)` precisely without swallowing a
  genuine `std::invalid_argument` bug. It derives from `std::runtime_error`, so a caller that
  only catches `std::exception` still works.

- **Carry M2 fields, don't drop them.** `p_detect`, `clutter_per_scan`, and the whole
  `tracker` block are parsed and stored even though M1 ignores most of them, so the scenario
  file is the single source of truth for a run (spec) and M2 needs no re-plumbing. `sigma_a`
  is the one `tracker` field the M1 filter actually consumes.

- **Stream-based CSV writers, free functions.** Writers take an `std::ostream&` so the same
  code serves a file (`radar_sim`) and an `ostringstream` (tests), and there is no hidden
  file handling in the formatting path. **Every numeric field — doubles and integers — is
  formatted with `std::to_chars`** (`chars_format::fixed` for doubles), which is
  locale-independent by construction and allocation-free: output is byte-identical across
  hosts (a `.` decimal point, no thousands separator) *regardless of any imbued or global
  locale*, and there is no per-field locale `imbue`/restore. (PR #14 review: an earlier
  draft forced only the doubles to the classic locale via a local `imbue`, which left the
  integer columns at the mercy of the stream locale — `1000` could print `1,000` and split a
  column — and churned the locale once per field.) The writers take `types` values
  (`Eigen::Vector4d` truth state, `Eigen::Vector2d` measurement position) so the app layer
  passes module outputs straight in. One consequence: `to_chars` on a negative `int`
  (`origin_id = −1`) converts to its unsigned magnitude inside libstdc++'s `<charconv>`,
  which UBSan's implicit-conversion check flags — handled as a third-party false positive in
  `cmake/sanitizer-ignorelist.txt`, exactly like the Eigen SSE entry.

- **No readers in M1 (deferred, documented).** `radar_score`/`metrics` are the only readers,
  and `tracks.csv` needs the tracker (M2). Adding a reader now would be untested surface with
  no caller. The deferral is recorded in `requirements.md`, `00-overview.md`, and the spec
  module-table note so the "writers and readers" line is satisfied incrementally.

## 3. nlohmann/json integration

- It is already fetched and declared `SYSTEM` (`cmake/Dependencies.cmake`), so its headers
  reach our TUs as `-isystem` and do not trip `-Werror`/clang-tidy — the same treatment Eigen
  and GoogleTest get. `src/io/config.cpp` is the first file to `#include <nlohmann/json.hpp>`;
  `io/config.hpp` does **not** include it (keep json out of the public API — callers see only
  structs + `ConfigError`), so json stays an implementation detail and compile times for
  consumers stay low.
- Parsing uses `nlohmann::json::parse(text)` with exceptions on; we catch
  `nlohmann::json::exception` at the `parse_scenario` boundary and rethrow as `ConfigError`
  with the source name, so no json exception type escapes `io`.

## 4. Verification approach

`tests/test_io.cpp`:
1. **Valid parse** — a literal single-target scenario string parsed by `parse_scenario`;
   assert every field equals the literal (AC-1).
2. **Issue #5 battery** — one `EXPECT_THROW(..., ConfigError)` per rejected case (NaN/inf/≤0
   `dt_s`, non-finite state, bad `sigma_m`, negative id, duplicate ids, non-empty maneuvers),
   each with a minimal mutated JSON string (AC-2). Runs in every build (not NDEBUG-guarded).
3. **Malformed input** — missing key, wrong type, invalid syntax, missing file → `ConfigError`,
   message names the path (AC-3).
4. **CSV format** — write a header + a couple of rows to an `ostringstream`; compare the exact
   text to a literal; re-parse a field to confirm precision (AC-4).
5. **`scenarios/single.json` loads** — the shipped scenario parses and its targets are valid
   (ties the file to the test).

The weight is on groups 2–3 (the issue-#5 boundary), because that validation is the whole
reason `io` exists as the untrusted-input gate.

## 5. Risks / watch items

| Risk | Mitigation |
| --- | --- |
| A leaked `nlohmann::json::exception` reaches `main` as an opaque crash | catch at the `parse_scenario` boundary, rethrow `ConfigError` with the source name; tested (AC-3). |
| `1e400`→`inf`, `NaN` literals slip through | explicit `std::isfinite` on every numeric field; tested (AC-2). |
| Locale makes CSV use `,` as the decimal point or a thousands separator → corrupt columns | all numeric fields written with `std::to_chars` (locale-independent by construction, integers included); test imbues a custom thousands-grouping `numpunct` facet (no installed locale needed, so it always runs on CI) and asserts the exact `.`-formatted text (AC-4). |
| A large finite double overflows the `to_chars` buffer → silently truncated/garbage CSV | buffer sized for the worst-case fixed-notation magnitude (`max_exponent10` + slack); `to_chars`'s `ec` is checked and a `std::range_error` thrown rather than writing an unspecified buffer (PR #14 re-review); tested with `1e60` (`LargeFiniteDoubleWritesInFull`). |
| An unsigned JSON integer above `INT64_MAX` wraps negative in `req_int` | `req_int` reads `is_number_unsigned` values as `uint64` and compares against `INT_MAX` before narrowing (mirrors the seed path), so a huge id is rejected as "out of range", not mislabeled a clutter id (PR #14 re-review); tested (`RejectsHugeUnsignedIntegerWithCorrectMessage`). |
| json headers trip our `-Werror`/clang-tidy | consumed `SYSTEM`; included only in `.cpp`, never the public header. |
| Duplicate/negative ids silently mis-score later | rejected at load with a named error (AC-2), mirroring the sensor's runtime guard (issue #5). |

## 6. What this unblocks

With `io`, a scenario file can be loaded into a validated `ScenarioConfig` and truth/meas
streams written as CSV. Next: `metrics` (RMSE) and `apps/radar_sim`, which wires
`io`→`motion`→`sensor`→`kalman`→CSV and prints RMSE — the first runnable application.
