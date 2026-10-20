# M1 · `kalman` module — Tasks

## T1 — Header + source
- [ ] `include/radar/kalman.hpp`: `KalmanConfig{dt, sigma_a, sigma_m}`, `kGate99 = 9.21`,
      `kSigmaV0 = 150`, `MeasurementMatrix` (2×4); declarations for `process_noise`,
      `measurement_matrix`, `measurement_noise`, `predict`, `initiate_track`, `innovation`,
      `innovation_covariance`, `gate_distance`, `update`. Guard `RADAR_KALMAN_HPP`; `namespace
      radar`; include `<Eigen/Dense>`, `radar/types.hpp`.
- [ ] `src/kalman.cpp`: implement. `predict` reuses `motion::cv_transition_matrix` (ADR-0006).
      `update` is Joseph form + `linalg::symmetrize`. `gate_distance`/gain use `S.inverse()`
      with an SPD guard. Validate inputs → `throw std::invalid_argument`.

## T2 — Build wiring
- [ ] Add `kalman.cpp` to the `radar` library in `src/CMakeLists.txt`.
- [ ] Register `test_kalman` in `tests/CMakeLists.txt` via `radar_add_test`.

## T3 — Tests (careful; issue #6 is the headline)
- [ ] Model matrices vs **literal** spec (`process_noise`, `H`, `R`).
- [ ] **Predict with hand-computed values, NOT via `cv_transition_matrix`** (issue #6).
- [ ] `initiate_track` exact `x̂`, `P₀`.
- [ ] `innovation` / `innovation_covariance` / `gate_distance` (zero-y, `S=σ²I`, worked 2×2).
- [ ] Joseph update: `y=0` fixes `x` + shrinks `P`; posterior symmetric + PSD; trace drops;
      matches `(I−KH)P` on a benign case.
- [ ] Convergence smoke (AC-6): CV truth + sensor, error < raw noise in a few scans (fixed seed).
- [ ] Validation `EXPECT_THROW` (bad `dt`/`σ_a`/`σ_m`/measurement).

## T4 — Verify — ALL PASS
- [x] `release` → `ctest`; `ci` + `ci-clang` 0 warnings in our code, **50/50**; `asan` **52/52**;
      `clang-format` clean.
- [x] Issue #6 satisfied: `PredictAdvancesStateByHandComputedValues` checks predict against
      literal hand-computed numbers, never calling `cv_transition_matrix`.
- [x] Fixed during verify: `bugprone-easily-swappable-parameters` on `process_noise(dt,
      sigma_a)` (scoped `NOLINT` + rationale; a swap is caught by the literal-matrix test),
      `misc-const-correctness`, and `gate_distance` returning a 1×1 Eigen expr (`.value()`).
- [x] UBSan `implicit-conversion` false positive inside Eigen's SSE `pabs` (reached via the
      `SelfAdjointEigenSolver` PSD check) resolved with `cmake/sanitizer-ignorelist.txt` scoped
      to Eigen paths + wired into the `asan` preset — the third-party-scoped ignorelist Phase 0
      planned for. Our code stays instrumented.

## T5 — Ship

## PR #11 review follow-up (H1, M1–M3, Low)
All mutation-verified (the cited bug now fails the matching test):
- **H1 (gate fail-open)** — `gate_distance`/`update` used `det(S) > 0`, which accepts a 2×2
  negative-definite S (e.g. `−I`) and returns a negative d² inside the gate for any
  measurement. Replaced with a shared `spd_cholesky` helper (`Eigen::LLT` + symmetry check);
  `d² = y·(S⁻¹y) ≥ 0`. `update` now reuses `innovation`/`innovation_covariance` (no duplicated
  `y`/`S`) and the same helper. Tests: `S = −I`, asymmetric S, negative-definite-P `update`.
  *Mutation*: reverting to det-based accept fails `GateDistanceRejectsNonPositiveDefiniteS`.
- **M1 (convergence couldn't tell a filter from pass-through)** — `TrackRmseBeatsRawRmse` now
  checks the spec gate: RMS(est−truth)/RMS(z−truth) ≤ 0.5 over the tail. *Mutation*: `est = z`
  fails it (ratio 1.0).
- **M2 (no hand check of predicted cov / update state)** — added `PredictCovarianceMatchesHand
  Computed` (`F·Fᵀ`) and `UpdateStateMatchesHandComputed` (non-diagonal P → `x = (11, 19.5,
  1.1, −1.05)`, exercises the velocity gain). *Mutations*: `Fᵀ·P·F` and half-gain each fail.
- **M3** — `KalmanConfig::sigma_a` default `1.0 → 5.0` (spec).
- **Low** — filter requires `σ_m > 0` (clear error vs a downstream singular-S); the sanitizer
  ignorelist is now a `CMAKE_CONFIGURE_DEPENDS` so edits force a rebuild; `.clang-tidy` disables
  the single `clang-analyzer-security.ArrayBound` check (false positive in Eigen's blocked GEMM
  reached via `llt.solve`, with documented rationale).
- Verified: `ci`/`ci-clang` 54/54, `asan` 56/56, clang-format clean; all four mutations fail.
- [ ] Single clean commit, push, open PR (reference issue #6); CI gates.

## Exit criteria
All ACs in `requirements.md` met; `kalman.{hpp,cpp}` + `test_kalman.cpp` committed. This
unblocks **Phase K** (the exhaustive NEES/NIS filter-verification battery), which is the next
phase before any tracker logic.
