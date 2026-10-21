# Phase K · Exhaustive Kalman verification battery — Tasks

## T1 — Branch + planning docs
- [x] Branch `phase-k` off `main`.
- [x] `requirements.md` — acceptance battery (AC-K1..K7), honest scope, MC config.
- [x] `design.md` — harness, statistics→code map, threshold theory, toolchain reuse.
- [x] `tasks.md` — this checklist.

## T2 — Implement `tests/test_phase_k.cpp`
- [x] File-local MC harness: `run_filter(seed) -> RunSamples` (intervals from file-scope
      constants); one `std::mt19937_64` per run threaded through init + the per-scan
      process-noise draw + every scan's `sense`; **truth = `propagate_cv` + chol(Q_literal)·w**
      (model-matched); filter via `initiate_track` / `predict` / `update`.
- [x] NIS from the **predicted** state (`innovation` + `innovation_covariance`), reusing
      `gate_distance(y, S)` as the exact `yᵀS⁻¹y`; collected over the full post-burn-in tail.
- [x] NEES from the **posterior** vs truth through the SPD guard `nees()` (`Eigen::LLT`,
      returns `+∞` on non-SPD); collected over the full post-burn-in tail. AC-K3 asserts
      `all_nees_finite` + `min_nees ≥ 0` so a bad sample is reported under AC-K3.
- [x] Ensemble accumulation over M = 100 runs, K = 200 scans, burn-in 20 (single interval).
- [x] `TEST(PhaseK, NisSingleRunInSpecBand)` — AC-K1, [1.5, 2.6].
- [x] `TEST(PhaseK, NisEnsembleMeanTight)` — AC-K2, [1.85, 2.15].
- [x] `TEST(PhaseK, NeesEnsembleMeanStateConsistent)` — AC-K3, [3.5, 4.6] full tail, keystone.
- [x] `TEST(PhaseK, CovarianceCredibilityVarianceMatch)` — AC-K4, ±15% per axis.
- [x] `TEST(PhaseK, ConvergenceOverMonteCarlo)` — AC-K5, mean ratio ≤ 0.5, every run ≤ 0.6,
      and filtered tail position-error variance < `σ_m²`.
- [x] Guards: `static_assert`s on the interval constants; `#include <limits>`.

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
- [x] Single clean commit on `phase-k`; push (sets upstream).
- [x] Open PR #12; gated by the 4 required checks (gcc, clang, asan, gitleaks) — all green.

## T7 — PR #12 review rounds (addressed, each reverified green)
Round 1 (9 findings) — NEES LLT guard, per-sample finite/≥0 (NaN-safe), `<limits>`,
interval `static_assert`s, AC-K5 `tail_err_var < σ_m²`, NIS→full tail, no duplicate run-0
sim, doc reconciliation. All fixed (commit `ed7989c`).

Round 2 (re-review) — the NEES window was a transient, not χ²(4). **Model-matched truth**
now injects the filter's process noise into truth (`truth = propagate_cv + chol(Q)·w`), so
NEES is a genuine χ²(4) over the full tail and the window is dropped. Crucially, truth's Q is
an **independent literal** (not `process_noise`) — else a `Q` mutation is common-mode and
invisible (verified: the shared path let `Q ×0.1` pass). Re-mutated production `kalman`
against the new battery:
- [x] transposed covariance predict → all 5 red (NEES 172).
- [x] `Q ×10` (under-confident) → NEES 2.37 < 3.5, red.
- [x] `Q ×0.1` (over-confident) → NEES 12.49 > 4.6, red — **the mutation round-1's windowed
      design missed, now caught.**
- [x] half-gain → Phase K green (documented blind spot); `test_kalman` unit test red.
- [x] All reverted; `src/kalman.cpp` zero diff; both nits fixed (`n_tail` comment removed;
      `nees()` returns `+∞` instead of EXPECT-ing inside the shared ensemble builder).

## Exit criteria
All ACs in `requirements.md` met; `test_phase_k.cpp` committed and green across
`release`/`ci`/`ci-clang`/`asan`; the three production mutations in T5 (transposed-covariance
predict, `Q ×10`, half-gain) shown to turn the battery red (or, for half-gain, to be caught
at the unit level — the documented blind spot) and reverted. This completes the M1
filter-verification gate and unblocks `io` / `metrics` / `apps/radar_sim`.
