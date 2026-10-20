# M1 · `kalman` module — Design

## 1. Shape of the module

`kalman` is `include/radar/kalman.hpp` + `src/kalman.cpp`, added to `libradar`. It depends on
`types` (`TrackState`, `Measurement`, state indices), `motion` (`cv_transition_matrix` for F —
ADR-0006), `linalg` (`symmetrize`), and Eigen.

Free functions over `TrackState`, not a stateful `Filter` object: a track's state lives in a
`TrackState` (owned by `track`/`tracker` in M2), and the filter steps are pure functions of
`(state, measurement, config)`. This keeps the filter testable in isolation and leaves
lifecycle/ownership to M2.

```cpp
namespace radar {

struct KalmanConfig {
  double dt{0.1};        // scan interval (s)
  double sigma_a{1.0};   // process-noise acceleration std-dev (m/s^2) — tuned in M2
  double sigma_m{50.0};  // measurement noise std-dev per axis (m) — matches the sensor
};

inline constexpr double kGate99 = 9.21;  // chi^2, 2 DOF, 99% — gate threshold for callers
inline constexpr double kSigmaV0 = 150.0;  // initial velocity std-dev (m/s) for P0

using MeasurementMatrix = Eigen::Matrix<double, 2, 4>;  // H

[[nodiscard]] Eigen::Matrix4d  process_noise(double dt, double sigma_a);
[[nodiscard]] MeasurementMatrix measurement_matrix() noexcept;      // H
[[nodiscard]] Eigen::Matrix2d  measurement_noise(double sigma_m);   // R

[[nodiscard]] TrackState predict(const TrackState&, const KalmanConfig&);
[[nodiscard]] TrackState initiate_track(const Measurement&, const KalmanConfig&);

[[nodiscard]] Eigen::Vector2d innovation(const TrackState&, const Measurement&);
[[nodiscard]] Eigen::Matrix2d innovation_covariance(const TrackState&, const KalmanConfig&);
[[nodiscard]] double          gate_distance(const Eigen::Vector2d& y, const Eigen::Matrix2d& S);

[[nodiscard]] TrackState update(const TrackState&, const Measurement&, const KalmanConfig&);

}  // namespace radar
```

## 2. Key decisions

- **`F` is reused, not redefined (ADR-0006).** `predict` calls `motion::cv_transition_matrix(dt)`.
  The common-mode risk this creates — a bug in F corrupting truth and filter identically — is
  bought off by an **independent** F check in the tests (see §4 / issue #6): `test_kalman`
  verifies `predict == F·x` with hand-computed numbers, never another `cv_transition_matrix`
  call. This is the mitigation ADR-0006 committed to.

- **Joseph-form covariance update.** `P = (I − K·H)·P·(I − K·H)ᵀ + K·R·Kᵀ`, then
  `linalg::symmetrize`. The textbook `(I − K·H)·P` is cheaper but loses symmetry and positive
  semi-definiteness under rounding; the spec mandates Joseph plus a symmetrize guard, because a
  non-PSD P breaks the gate (a negative-variance S) and the whole estimate. We pay the few extra
  matrix multiplies for a filter that stays well-formed over a 600-scan run.

- **S⁻¹ for a 2×2.** `S` is 2×2 SPD, so `gate_distance` and the Kalman gain use Eigen's direct
  `S.inverse()` (closed-form for 2×2, no pivoting cost) — but we guard it: `S` is checked SPD-ish
  (finite, positive determinant) so a degenerate S throws rather than producing NaN/Inf d². The
  gain uses `K = P·Hᵀ·S⁻¹`.

- **`gate_distance` takes `y` and `S`, not a track+measurement.** Association (M2) computes an
  `N×M` table of d² values and wants to reuse one `S` per track across many measurements; passing
  the already-computed `y`/`S` keeps `kalman` free of the association loop and avoids recomputing
  `S` per pair. `innovation` and `innovation_covariance` are the pieces that build them.

- **One-point initialization.** `x̂ = [z_x, z_y, 0, 0]`, `P₀ = diag(σ_m², σ_m², σ_v0², σ_v0²)`,
  `σ_v0 = 150 m/s` (covers every scenario aircraft speed). Velocity starts at zero with wide
  variance so the first few updates pull it in. Two-point differencing (velocity from the first
  two hits) is the documented upgrade if Phase K / plots show slow convergence — deferred.

- **Config, not globals.** `dt`, `σ_a`, `σ_m` travel in `KalmanConfig` (from the scenario via
  `io` later). `σ_a` is the one genuinely *tuned* parameter (M2, against the NIS target); it has a
  placeholder default here. `σ_m` must match the sensor's.

- **Validation throws (AC-7).** Like `sensor`, invalid input is a runtime `throw
  std::invalid_argument`, not an assert: a filter fed a NaN `dt` or measurement silently produces
  NaN states that poison every track and the NEES/NIS battery. Checked once per call, not a hot
  path at M1 scale.

## 3. The math, mapped to code

| Spec | Code |
| --- | --- |
| `x̂_{k|k−1} = F·x̂` | `predict().x = cv_transition_matrix(dt) * x` |
| `P_{k|k−1} = F·P·Fᵀ + Q` | `predict().P = F*P*F.transpose() + process_noise(dt, σ_a)` |
| `y = z − H·x̂` | `innovation() = z - H*x` (H selects the position block) |
| `S = H·P·Hᵀ + R` | `innovation_covariance() = P.topLeftCorner<2,2>() + R` |
| `K = P·Hᵀ·S⁻¹` | `K = P * H.transpose() * S.inverse()` (4×2) |
| `x̂_{k|k} = x̂ + K·y` | `update().x = x + K*y` |
| `P_{k|k} = (I−KH)P(I−KH)ᵀ + KRKᵀ` | Joseph, then `symmetrize` |
| `d² = yᵀ·S⁻¹·y ≤ γ=9.21` | `gate_distance(y, S)`, `kGate99` |

Note `H·P·Hᵀ` is just the top-left 2×2 (position) block of P, and `H·x̂` is the first two
components of `x̂` — we use the named state indices / Eigen blocks rather than materializing H
multiplications where a block is clearer, but `measurement_matrix()` is still provided (exact H)
for tests and for association code that prefers the explicit form.

## 4. Verification approach (and the issue #6 mandate)

`tests/test_kalman.cpp`:
1. **Model matrices vs literal spec** — `process_noise`, `H`, `R` compared against matrices
   written out by hand from the spec (independent of the implementation's construction).
2. **Predict, hand-computed (issue #6)** — a known `x`/`P` and `dt` propagated by numbers worked
   out on paper, asserting `predict` matches — crucially **without** calling
   `cv_transition_matrix`, so a shared-F bug can't hide. Also `P` symmetry and growth by Q.
3. **initiate_track** — exact `x̂` and `P₀`.
4. **innovation / S / gate_distance** — `y = z − Hx̂`; `S = Ppos + R`; `d²` zero at `y = 0`,
   `‖y‖²/σ²` for `S = σ²I`, and a worked 2×2 example.
5. **Joseph update** — `y = 0` leaves `x` fixed and shrinks `P`; posterior symmetric + PSD
   (eigenvalues ≥ 0); position-trace decreases; matches `(I−KH)P` to tolerance on a benign case.
6. **Convergence smoke (AC-6)** — CV truth + Gaussian sensor (reuse `motion`/`sensor`), run the
   filter, assert the position error drops below raw noise within a few scans. Fixed seed.
7. **Validation** — `EXPECT_THROW` on non-finite/non-positive `dt`, bad `σ_a`/`σ_m`, non-finite
   measurement; runs in every build.

The weight is on groups 1–2 (the independent F check) and 5 (Joseph correctness/PSD), because
those are where a subtle error would otherwise flow silently into Phase K and every tracking
number after it.

## 5. Risks / watch items

| Risk | Mitigation |
| --- | --- |
| Common-mode F bug (shared truth/filter F) | hand-computed predict test, no `cv_transition_matrix` call (issue #6 / ADR-0006). |
| Non-PSD P from the textbook update under rounding | Joseph form + `symmetrize`; test asserts eigenvalues ≥ 0. |
| Gate fails **open** on a negative-definite S (`det(S) > 0` wrongly accepted `−I`, giving a negative d²) | SPD verified by **Cholesky** (`Eigen::LLT`) + symmetry check, shared by `gate_distance` and `update`; `d² = y·(S⁻¹y) ≥ 0`. Tested with `S = −I`, asymmetric S, and a negative-definite-P `update` (PR #11 H1). |
| NaN input silently poisons tracks and Phase K | throw `std::invalid_argument` on non-finite/invalid config/inputs (AC-7); the filter also requires `σ_m > 0`. |
| `σ_a` default too small → Phase K judges an overconfident filter | default is the spec's **5.0 m/s²**, not a placeholder (PR #11 M3); real tuning is M2 against the NIS target. |
| `Q`/variance vs std-dev confusion | `process_noise` takes `σ_a` (std-dev) and squares it; literal-matrix test catches a missing/extra square. |
| Transposed covariance predict (`Fᵀ·P·F`) or wrong gain scale | hand-computed predicted-covariance (`F·Fᵀ`) and update-state (`x = (11, 19.5, 1.1, −1.05)`) tests — mutation-verified to fail on those bugs (PR #11 M2). |
| UBSan `implicit-conversion` false positive inside Eigen's SSE `pabs`, hit by the `SelfAdjointEigenSolver` PSD check | `cmake/sanitizer-ignorelist.txt` scoped to Eigen paths, wired into `asan` and registered as a `CMAKE_CONFIGURE_DEPENDS` so edits force a rebuild — the third-party-scoped ignorelist Phase 0 planned for; our code stays instrumented. |
| clang-analyzer `security.ArrayBound` false positive inside Eigen's blocked GEMM, reached via the gain's `llt.solve()` | analyzer diagnostics ignore `HeaderFilterRegex`, so it leaked from the SYSTEM Eigen header as our error; disabled that **one** analyzer check in `.clang-tidy` with a documented rationale (the rest of `clang-analyzer-*` stays on). |

## 6. What this unblocks

With `types`, `motion`, `sensor`, and now `kalman`, **Phase K** can run its full
verification battery (NEES/NIS, convergence, covariance credibility) on the filter before any
association/tracker logic is built. After Phase K: `io`, `metrics` (RMSE), and `apps/radar_sim`.
