# M1 · `sensor` module — Tasks

> **Status: ✅ complete — merged (PR #10).**

## T1 — Header + source
- [x] `include/radar/sensor.hpp`: `SensorConfig{double sigma_m=50}`, `TrueTarget{int id;
      Eigen::Vector2d position}`, and
      `sense(const std::vector<TrueTarget>&, const SensorConfig&, std::mt19937_64&)
      -> std::vector<LabeledMeasurement>`. Guard `RADAR_SENSOR_HPP`; `namespace radar`;
      include `<random>`, `<vector>`, `radar/types.hpp`.
- [x] `src/sensor.cpp`: per target, draw two `N(0, sigma_m)` samples, `z = p + n`,
      `origin = target.id`; **shuffle** the output; **throw `std::invalid_argument`** on
      non-finite/negative `sigma_m`, `id < 0`, or non-finite position (runtime, not assert).
- [x] Document the deferred M2 sensor steps (P_D<1, clutter, square culling) in the header
      (comment only, no stub).

## T2 — Build wiring
- [x] Add `sensor.cpp` to the `radar` library in `src/CMakeLists.txt`.
- [x] Register `test_sensor` in `tests/CMakeLists.txt` via `radar_add_test`.

## T3 — Tests
- [x] `tests/test_sensor.cpp`: per-target + origin preserved, looked up **by origin** (order is
      shuffled); 2-target-same-point test; `.measurement` is a plain position; exact
      determinism (same seed ⇒ identical, `==`); different seed ⇒ differs; `sigma_m = 0` ⇒
      `z == p` exact; statistical (fixed seed, 2e5 draws): mean≈0, std≈`sigma_m`,
      RMS‖z−p‖≈`sigma_m·√2`, **Cov(x,y)≈0** (independence); `EXPECT_THROW` cases (negative /
      NaN / inf `sigma_m`, negative id, NaN / inf position) that run in **every** build.

## T4 — Verify (maps to ACs) — ALL PASS
- [x] `release` build → `ctest` passes.
- [x] `ci` + `ci-clang` → 0 warnings in our code, **34/34** pass (10 Sensor tests).
- [x] `asan` → Sensor 10/10, sanitizer-clean.
- [x] `clang-format --dry-run --Werror` clean.
- [x] **Mutation check**: `ny = nx` now **fails** `NoiseMatchesGaussianStatistics` (covariance).

### Verification findings fixed
- `std::normal_distribution(0, 0)` aborts under libstdc++ (`stddev > 0`), so `sigma_m == 0`
  is special-cased in `sense` (emit `z == p`, consume no RNG).
- clang-tidy `cert-msc51-cpp` (fixed test seed) → single `seeded()` factory with one `NOLINT`;
  `modernize-use-ranges` → `std::ranges::find_if`; GCC `-Werror=null-dereference` false positive
  on a chained `by_origin()->…` → bind to a local + `ASSERT_NE` first.

## T5 — Ship
- [x] Single clean commit, push, open PR #10 (CI gated it green).

## PR #10 review follow-up
Addressed the security-first review on PR #10:
- **H1** — bad `sigma_m` failed *open* to a perfect sensor in release (asserts compiled out).
  `sense` now **throws** `std::invalid_argument` (runtime); tests are `EXPECT_THROW`, so they
  run in `ci` too. `sigma_m` + target-id validation added to issue #5.
- **M1** — reject `id < 0` (collides with `kClutterOrigin`).
- **M2** — added the x/y **covariance** check; a `ny = nx` mutation now fails it.
- **M3** — `std::shuffle` the output (list position no longer leaks origin for >1 target;
  honors the `Scan` "already shuffled" contract). 0/1-element shuffle draws no RNG, so
  `single.json` still reproduces. Tests look up by origin, not index.
- **L1** — corrected the abort-cause doc (`_GLIBCXX_ASSERTIONS` via `RADAR_HARDENING` in
  release/ci, not asan). **L2** — noted `normal_distribution`'s pair-caching. **L3** — added a
  "Deviations from the spec" section for the deferred *tests*. **L4** — `<cstdint>`, extra
  non-finite/inf throw cases, tasks ticked.

## PR #10 follow-up review (R1, R2, nit)
- **R1** — the shuffle was untested (deleting it left all tests green). Added
  `TEST(Sensor, OrderIsShuffled)`: two same-point targets, 1000 trials, first element's origin
  is ~50/50; unshuffled it would be 1000/1000. **Mutation-verified**: deleting the
  `std::shuffle` now fails this test. Fixed the requirements wording (it had claimed the
  by-origin lookups covered the shuffle — they don't).
- **R2** — a throw used to advance `rng` (target checks ran inside the draw loop, so target
  0's noise was drawn before an invalid target 1 threw; a catch-and-retry caller would land on
  a different stream). `sense` now validates **all** targets in a pre-pass before any draw.
  Added `TEST(Sensor, FailedCallLeavesRngUnchanged)` (after a throw, `rng == fresh seeded`).
  **Mutation-verified**: moving a target check back into the draw loop fails this test. Header
  documents the no-RNG-on-throw guarantee.
- **Nit** — fixed the `sensor.cpp` comment ("output is a pure function of `rng`'s state because
  the distribution is per-call"), consistent with the L2 pair-caching note.
- Verified: `ci`/`ci-clang` 36/36 (12 Sensor tests), `asan` 38/38, clang-format clean; both
  mutations fail as intended.

## Exit criteria
All ACs in `requirements.md` demonstrated by T4; `sensor.{hpp,cpp}` + `test_sensor.cpp`
committed. Deferred M2 sensor steps recorded. Next module: `kalman` (predict, Joseph update,
innovation, S, gate distance), then **Phase K**.
