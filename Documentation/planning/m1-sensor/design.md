# M1 · `sensor` module — Design

## 1. Shape of the module

`sensor` is `include/radar/sensor.hpp` + `src/sensor.cpp`, added to `libradar`. It depends on
`types` (for `Measurement`/`LabeledMeasurement`/`kClutterOrigin`) and the standard library's
`<random>`; no Eigen beyond the `Vector2d` already in `types`.

```cpp
namespace radar {

struct SensorConfig {
  double sigma_m{50.0};  // measurement-noise std-dev per axis (m); spec default
};

// A true target the sensor can see this scan: a stable id and its true position.
struct TrueTarget {
  int id{0};                     // >= 0; becomes the measurement's hidden origin
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};  // true (x, y) in meters
};

// Noise-only sensor (M1 slice): emit one LabeledMeasurement per target,
// z = p + n with n ~ N(0, sigma_m^2 I). P_D = 1, no clutter, no shuffle (M2).
// `rng` is the single scenario generator, owned by the caller and passed by ref.
[[nodiscard]] std::vector<LabeledMeasurement> sense(
    const std::vector<TrueTarget>& targets, const SensorConfig& cfg, std::mt19937_64& rng);

}  // namespace radar
```

## 2. Key decisions

- **Randomness is explicit and caller-owned.** The spec mandates *one* `std::mt19937_64`
  seeded from the scenario (default 42), threaded through the whole run so results reproduce
  exactly. `sense` therefore takes `std::mt19937_64&` — it holds no RNG state, no hidden
  global, no internal reseeding. The simulator owns the generator and passes the same one to
  `motion` truth setup, the sensor, and anywhere else randomness is consumed, giving a single
  reproducible stream per seed. A free function (not a stateful `Sensor` object) keeps that
  explicit; if the full M2 sensor wants to cache distributions it can become a small struct
  without changing this contract.

- **Returns `LabeledMeasurement`, not `Measurement`.** The sensor is the one place that knows
  each detection's origin, so it produces the *labelled* value for the simulator (truth
  record) and the scorer. The tracker-facing value is `labeled.measurement` — a plain
  `Measurement` with no origin, exactly the type-system isolation `types` enforces. The sensor
  never hands the tracker anything but `Measurement`s (that handoff is the simulator's, M2+).

- **Noise generation.** Per axis, draw from `std::normal_distribution<double>(0.0, sigma_m)`
  and add to the true position: `z.x = p.x + nx`, `z.y = p.y + ny`. Two independent draws give
  `n ~ N(0, sigma_m^2 I)`. The distribution object is cheap; we construct it per call (state is
  in `rng`, so this does not affect reproducibility). **`sigma_m == 0` is special-cased**:
  we emit `z == p` directly and consume no randomness, rather than constructing
  `normal_distribution(0, 0)` — libstdc++ asserts `stddev > 0`, so a zero-stddev distribution
  aborts in debug/ASan builds (caught by AC-4 under asserts).

- **`TrueTarget` input, not `TrackState`.** The sensor needs only id + true position; velocity
  is irrelevant to a position measurement. Taking a small purpose-built struct keeps the
  signature honest and avoids implying the sensor reads a filter estimate. The simulator builds
  `TrueTarget{id, truth.head<2>()}` from each target's true state each scan.

- **No `P_D`/clutter/shuffle in M1.** With `P_D = 1` every target yields exactly one
  measurement and order follows the input — so no shuffle is needed yet (a one-target scan has
  nothing to shuffle, and order can't leak origin when origin is only on the labelled side).
  Bernoulli trials, Poisson clutter, uniform placement, and shuffling arrive together in M2,
  where multi-target/clutter scenarios test them. See requirements "Deferred to M2".

## 3. Reproducibility and the statistical test

Randomness makes this the first module whose correctness is *statistical*. Two complementary
checks:
- **Exact determinism** (AC-3): seed a generator, call `sense`, reseed with the same value,
  call again — the measurements must be bit-for-bit identical (`==`). A different seed must
  differ. This pins reproducibility without any tolerance.
- **Distribution shape** (AC-5): with a fixed seed, draw a large sample (≥ 100 000) for one
  fixed target and check empirical mean ≈ 0, sample std ≈ `sigma_m`, and
  RMS `‖z − p‖` ≈ `sigma_m·√2`. The fixed seed makes the test deterministic in CI (no
  flakiness); tolerances are chosen loose enough that the specific seed's sample always passes
  but tight enough to catch a wrong σ (e.g. variance vs std-dev confusion) or a mean offset.
  `σ_m·√2 ≈ 70.7 m` at σ_m = 50 is the spec's "about 71 m" raw RMSE, so this test also anchors
  the number M1's acceptance gate compares against.

## 4. Risks / watch items

| Risk | Mitigation |
| --- | --- |
| Hidden/global RNG would make runs irreproducible | `sense` takes `std::mt19937_64&`; no internal state (AC-3 exact-determinism test). |
| std vs variance mix-up (`sigma_m` used where `sigma_m^2` belongs or vice versa) | `normal_distribution(0, sigma_m)` takes a **std-dev**; the std≈σ and RMSE≈σ√2 tests catch a square/÷ error. |
| Statistical test flaky in CI | fixed seed + fixed sample size → deterministic; tolerances sized for that seed. |
| Origin leaks to the tracker | sensor returns `LabeledMeasurement`; tracker-facing value is `.measurement` (plain), enforced by `types`' header asserts. |
| Non-finite `sigma_m`/position silently poisons measurements | assert finite + `sigma_m >= 0` (debug/ASan), per the `types`/`motion` precedent; `io` validates at the boundary. |
| `std::normal_distribution` cross-implementation differences (libstdc++ vs libc++) | determinism is asserted per-implementation (same binary), not across; statistical tests use tolerances, not byte-equality, matching the spec's cross-compiler stance. |

## 5. What this unblocks

With `motion` (truth) and a Gaussian `sensor` in place, `kalman` can be built and then
**Phase K** can generate Monte-Carlo truth→measurement data to run its NEES/NIS consistency
battery — the heavy verification gate before building the tracker upward.
