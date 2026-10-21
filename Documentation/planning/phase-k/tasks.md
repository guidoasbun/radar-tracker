# Phase K · Exhaustive Kalman verification battery — Tasks

## T1 — Branch + planning docs
- [x] Branch `phase-k` off `main`.
- [x] `requirements.md` — acceptance battery (AC-K1..K7), honest scope, MC config.
- [x] `design.md` — harness, statistics→code map, threshold theory, toolchain reuse.
- [x] `tasks.md` — this checklist.

## T2 — Implement `tests/test_phase_k.cpp`
- [ ] File-local MC harness: `run_filter(seed, k, burn_in) -> RunSamples`; one
      `std::mt19937_64` per run threaded through init + every scan's `sense`; truth via
      `propagate_cv`; filter via `initiate_track` / `predict` / `update`.
- [x] NIS from the **predicted** state (`innovation` + `innovation_covariance`), reusing
      `gate_distance(y, S)` as the exact `yᵀS⁻¹y`.
- [x] NEES from the **posterior** vs truth: `e.dot(est.P.llt().solve(e))`; assert ≥ 0.
- [x] Ensemble accumulation over M = 100 runs, K = 200 scans, burn-in 20, window 40.
- [x] `TEST(PhaseK, NisSingleRunInSpecBand)` — AC-K1, [1.5, 2.6].
- [x] `TEST(PhaseK, NisEnsembleMeanTight)` — AC-K2, [1.85, 2.15].
- [x] `TEST(PhaseK, NeesEnsembleMeanStateConsistent)` — AC-K3, [3.0, 4.5] over the window, keystone.
- [x] `TEST(PhaseK, CovarianceCredibilityVarianceMatch)` — AC-K4, ±15% per axis.
- [x] `TEST(PhaseK, ConvergenceOverMonteCarlo)` — AC-K5, mean ratio ≤ 0.5, every run ≤ 0.6.

## T3 — Build wiring
- [x] Register `test_phase_k` in `tests/CMakeLists.txt` via `radar_add_test`.

## T4 — Verify — ALL PASS
- [x] `release` → `ctest` green (5/5 Phase K, full suite clean).
- [x] `ci` (GCC) 59/59 + `ci-clang` 59/59, 0 warnings in our code. Fixed:
      `modernize-use-emplace` (two `push_back`→`emplace_back`), two clang-format aligns.
- [x] `asan` 61/61 green — Eigen `LLT::solve` NEES path clean under the existing
      `cmake/sanitizer-ignorelist.txt` + `.clang-tidy`; no new suppressions needed.
- [x] `clang-format --dry-run --Werror` clean.
- [x] Non-flakiness: fixed seeds ⇒ identical results across repeated preset runs.

## T5 — Mutation-verify (AC-K6) — perturbed production src/kalman.cpp, each reverted
Verified against the real `test_phase_k` (release build), not a probe:
- [x] Transposed covariance predict (`Fᵀ·P·F`) → **all 5 Phase K tests red** (NIS 4.37,
      NEES 96, credibility, convergence 0.76). Reverted.
- [x] `process_noise` `Q ×= 10` → **only NeesEnsembleMeanStateConsistent red**, measured
      `mean_nees = 2.7402 < 3.0` (NIS + convergence stay green — expected, NIS is
      position-innovation only). Reverted.
- [x] Half-gain `K` in `update` → **Phase K stays 100% green** (documented blind spot on
      clean CV), while `test_kalman::UpdateStateMatchesHandComputed` goes **red** — the
      unit-level deterministic catch the scope section relies on. Reverted.
- [x] After reverting all three: `src/kalman.cpp` has no diff vs HEAD; full suite 59/59
      green. Confirms NEES is the keystone (catches the Q over-inflation the position-only
      NIS cannot) and that the gain-scale blind spot is covered at the unit level.

## T6 — Ship
- [ ] Single clean commit on `phase-k`; push (sets upstream).
- [ ] Open PR; gated by the 4 required checks (gcc, clang, asan, gitleaks) green +
      up-to-date before merge. Phase K touches no `.github/workflows/*`, so the token's
      missing `workflow` scope is not hit.

## Exit criteria
All ACs in `requirements.md` met; `test_phase_k.cpp` committed and green across
`release`/`ci`/`ci-clang`/`asan`; all four mutations shown to turn the battery red and
reverted. This completes the M1 filter-verification gate and unblocks `io` / `metrics` /
`apps/radar_sim`.
