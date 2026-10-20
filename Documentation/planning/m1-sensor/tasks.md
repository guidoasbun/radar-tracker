# M1 · `sensor` module — Tasks

## T1 — Header + source
- [ ] `include/radar/sensor.hpp`: `SensorConfig{double sigma_m=50}`, `TrueTarget{int id;
      Eigen::Vector2d position}`, and
      `sense(const std::vector<TrueTarget>&, const SensorConfig&, std::mt19937_64&)
      -> std::vector<LabeledMeasurement>`. Guard `RADAR_SENSOR_HPP`; `namespace radar`;
      include `<random>`, `<vector>`, `radar/types.hpp`.
- [ ] `src/sensor.cpp`: per target, draw two `N(0, sigma_m)` samples, `z = p + n`,
      `origin = target.id`. Assert finite positions + `sigma_m >= 0`.
- [ ] Document the deferred M2 sensor steps (P_D<1, clutter, shuffle, square culling) in the
      header (comment only, no stub).

## T2 — Build wiring
- [ ] Add `sensor.cpp` to the `radar` library in `src/CMakeLists.txt`.
- [ ] Register `test_sensor` in `tests/CMakeLists.txt` via `radar_add_test`.

## T3 — Tests
- [ ] `tests/test_sensor.cpp`: one-measurement-per-target + origin preserved (P_D=1);
      `.measurement` is a plain position; exact determinism (same seed ⇒ identical, `==`);
      different seed ⇒ differs; `sigma_m = 0` ⇒ `z == p` exact; statistical (fixed seed,
      ≥1e5 draws): mean≈0, std≈`sigma_m`, RMS‖z−p‖≈`sigma_m·√2`; `NDEBUG`-guarded death tests
      for non-finite position / negative `sigma_m`.

## T4 — Verify (maps to ACs) — ALL PASS
- [x] `release` build → `ctest` passes.
- [x] `ci` + `ci-clang` → 0 warnings in our code, 29/29 pass.
- [x] `asan` → 33/33 incl both `SensorDeathTest` cases + the statistical test (passes under
      a different `normal_distribution` impl, confirming tolerances absorb cross-impl diffs).
- [x] `clang-format --dry-run --Werror` clean.

### Verification findings fixed
- `std::normal_distribution(0, 0)` **aborts** under libstdc++ (`_M_stddev > 0` assert), so
  `sigma_m == 0` is now special-cased in `sense` (emit `z == p`, consume no RNG) rather than
  relying on a zero-stddev distribution. Docs/design updated.
- clang-tidy `cert-msc51-cpp` flags the deliberately fixed test seed; routed all generators
  through a single `seeded()` factory with one documented `NOLINT`.

## T5 — Ship
- [ ] Single clean commit, push, open PR (CI now gates it).

## Exit criteria
All ACs in `requirements.md` demonstrated by T4; `sensor.{hpp,cpp}` + `test_sensor.cpp`
committed. Deferred M2 sensor steps recorded. Next module: `kalman` (predict, Joseph update,
innovation, S, gate distance), then **Phase K**.
