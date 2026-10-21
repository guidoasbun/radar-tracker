# Phase K · Exhaustive Kalman verification battery — Design

## 1. Shape

A single test translation unit, `tests/test_phase_k.cpp`, registered with
`radar_add_test(test_phase_k test_phase_k.cpp)` in `tests/CMakeLists.txt`. No production
code changes: Phase K only *reads* the existing `kalman`/`motion`/`sensor`/`types` API.
It links the same `radar::radar` + `GTest::gtest_main` + warnings/hardening targets as
every other test, so it inherits the full `ci`/`ci-clang`/`asan` treatment.

The battery is a set of GoogleTest `TEST(PhaseK, …)` cases backed by a small internal
Monte-Carlo harness (file-local `namespace {}`), one group of assertions per AC.

## 2. The Monte-Carlo harness

The core is one function that runs a single filter trajectory and returns the per-scan
samples needed by every statistic, so NIS/NEES/convergence all come from the *same*
runs (no redundant simulation):

```cpp
struct RunSamples {
  std::vector<double> nis;   // y^T S^-1 y over [burn-in, K)          (chi^2_2, full tail)
  std::vector<double> nees;  // e^T P^-1 e over [burn-in, burn-in+W)  (chi^2_4, window)
  std::vector<Eigen::Vector2d> pos_err;        // position error per window scan
  std::vector<Eigen::Vector2d> pos_var_claim;  // P(px,px), P(py,py) per window scan
  double tail_track_sq{0.0};  // sum |est_pos - truth_pos|^2 over the late tail
  double tail_raw_sq{0.0};    // sum |z - truth_pos|^2 over the late tail
};

RunSamples run_filter(std::uint64_t seed);  // intervals from file-scope constants
```

NEES goes through `nees_or_fail()`, which runs the production SPD guard (`Eigen::LLT` +
`info() == Success`) and asserts each sample is finite and ≥ 0 — a regression that makes `P`
indefinite fails loudly rather than feeding a meaningless solve into the band. NIS spans the
full post-burn-in tail (it does not drift); NEES/credibility span the window (NEES does).

Inside `run_filter`:
- **One** `std::mt19937_64 rng{seed}` for the whole run, threaded through the init
  `sense` and every scan's `sense`. This mirrors the real sim's "single scenario
  generator" contract and is what makes "runs are independent, a run is reproducible"
  literally true — not fresh generators mid-run.
- Truth: `Eigen::Vector4d` propagated by `motion::propagate_cv(truth, dt)` each scan.
  Initial truth is a straight-line CV target at aircraft speed (e.g. `v = (200, −100)`),
  which the existing `test_kalman` convergence smoke already uses — reused for continuity.
- Filter: `initiate_track` from the first measurement, then per scan
  `est = update(predict(est, cfg), z, cfg)`.
- Per scan, **before** the update, compute NIS from the *predicted* state
  (`innovation` / `innovation_covariance` on the prediction) — innovations are a property
  of the prediction, which is the textbook NIS. Compute NEES from the *posterior* (the
  filtered estimate) against truth.
- Collect tail samples (scan index ≥ `k_scans − tail`) for convergence, and a single
  fixed late scan's position error + claimed variance for credibility.

Ensemble: `for run in [0, M): acc += run_filter(kBaseSeed + run, K, burnIn)`, pooling
the per-scan vectors. Means are plain pooled averages.

## 3. The statistics, mapped to code

| Statistic | Definition | Code |
| --- | --- | --- |
| NIS | `yᵀ S⁻¹ y`, 2 DOF | `y = innovation(pred, z)`; `S = innovation_covariance(pred, cfg)`; `gate_distance(y, S)` **is exactly `yᵀS⁻¹y`** with the SPD guard — reuse it rather than re-inverting. Collected over the full post-burn-in tail. |
| NEES | `eᵀ P⁻¹ e`, 4 DOF | `e = est.x − truth`; `nees_or_fail(e, P)` runs `Eigen::LLT`, asserts `info() == Success` and each sample finite & ≥ 0, then returns `e.dot(llt.solve(e))`. Collected over the window. |
| convergence ratio | RMS(est−truth)/RMS(z−truth) | pooled `sqrt(Σ track_sq / Σ raw_sq)` over the late tail; plus the absolute check `tail_err_var < σ_m²` |
| credibility | Var(err) vs mean claimed `P_pos` | per-axis second moment of `pos_err` vs mean `pos_var_claim`, both over the window |

Reusing `gate_distance` for NIS is deliberate: it is the exact Mahalanobis form the
filter ships, including the Cholesky SPD guard, so the test measures the production
quantity, not a re-derivation that could silently differ. NEES uses the same LLT guard via
`nees_or_fail`, so an indefinite-`P` regression fails loudly rather than landing in-band.

## 4. Why these thresholds (theory + measurement)

- NIS ~ χ²(2): mean 2, variance 4. NEES ~ χ²(4): mean 4, variance 8.
- **Measurement intervals (deliberately different).** NIS is averaged over the **full
  post-burn-in tail** `[kBurnIn, kScans)` = `[20, 200)`; NEES and credibility over the
  **window** `[kBurnIn, kBurnIn + kWindow)` = `[20, 60)`. On perfectly-CV truth the filter
  reaches a steady state whose velocity-block covariance is slightly larger than the
  velocity error it achieves, so the full-tail NEES drifts below 4 to ~2.4 (benign, correct
  under-confidence), while NIS does **not** drift (full-tail 1.95 vs window 1.97). So NEES
  must be windowed to keep a principled χ²(4) band, but NIS can use the full tail — which
  matches the spec's "average over all post-burn-in scans" and catches a covariance bug
  that appears only late. Measured on the correct filter (M = 100): **NIS ≈ 1.95 (full
  tail), NEES ≈ 3.61 (window), posVarRatio ≈ 0.99, convergence ≈ 0.18.**
- **Sample counts:** NIS ≈ 100·180 = 18 000; NEES/credibility ≈ 100·40 = 4 000.
- **Bands, and why they are where they are** (every number measured, then
  mutation-verified — §7):
  - NIS ensemble **[1.85, 2.15]** — correct 1.95; the only mutation that moves it is the
    transposed predict (→4.37). Gain/Q leave NIS ≈ 1.9 because NIS sees only the position
    innovation; that is expected, not a weakness (NEES carries the covariance structure).
  - NEES ensemble **[3.0, 4.5]** — correct 3.61; catches the transposed predict (→96) and
    the 10×-`Q` over-inflation (→2.74 < 3.0). This is the keystone band.
  - Convergence **mean ≤ 0.5 / every run ≤ 0.6** — the spec M1 gate; correct 0.18,
    pass-through 1.0, transposed predict 0.76.
  - Credibility **ratio within ±15%/axis** — correct ~0.99; a gross structural bug
    (transposed predict) sends it to ~70.
- **Sampling spread vs band width.** If the `M·kWindow = 100·40 = 4000` window samples
- **Sampling spread vs band width.** If the samples were independent, the pooled-mean SE
  would be `sqrt(4/18000) ≈ 0.015` for NIS (full tail, 18 000 samples) and `sqrt(8/4000) ≈
  0.045` for NEES (window, 4 000 samples). Samples within a run are serially correlated, so
  the effective N is lower and the true SE larger; M = 100 keeps the between-run spread
  small enough that the bands (±0.15 NIS, −0.6/+0.9 NEES around the measured means) are many
  SEs wide — non-flaky, confirmed by repeated fixed-seed runs in the verify step.

## 5. Interaction with the toolchain (reused, not new)

Phase K hits the same Eigen code paths as `test_kalman` plus `LLT::solve` for NEES and
possibly `SelfAdjointEigenSolver` if used for any PSD assertion. Those are exactly the
paths the M1 kalman work already de-conflicted:

- **UBSan `implicit-conversion`** inside Eigen SSE packet math →
  `cmake/sanitizer-ignorelist.txt` (scoped to `_deps/eigen3-src` / `Eigen/`), wired into
  the `asan` preset and a `CMAKE_CONFIGURE_DEPENDS`. Reused as-is.
- **clang-analyzer `security.ArrayBound`** false positive in Eigen's blocked GEMM reached
  via `llt.solve()` → disabled (that one check only) in `.clang-tidy` with rationale.
  Reused as-is.

No new suppressions are expected. If a genuinely new Eigen path trips a sanitizer, it is
added to the **existing scoped** ignorelist (third-party paths only) with a comment — our
code stays fully instrumented. This is the mitigation the M1 design risk table committed
to.

## 6. Determinism & cost

- Fixed seeds (`kBaseSeed + run`), one generator per run; `std::normal_distribution`
  determinism is asserted only *within* one binary (libstdc++ vs libc++ differ), so the
  battery never relies on cross-implementation byte-equality — it relies on the χ²
  *distribution* holding, which is implementation-independent.
- Cost: 100 runs × 200 scans × (predict + update + one 4×4 LLT solve) ≈ a few ×10⁶
  flops — well under a second, even under `asan`'s `-O1`. Bounded and fixed, so CI time
  is stable.

## 7. Mutation resistance (verification obligation)

Per AC-K6, the discriminating power is demonstrated by perturbing the **production**
filter during verify and confirming the battery goes red, then reverting — not by a
test-side self-check (which would only exercise test arithmetic). Measured outcomes
(ensemble over the window; correct = NIS 1.97 / NEES 3.61 / conv 0.18):

| Mutation (reverted after) | NIS | NEES | conv | Result |
| --- | --- | --- | --- | --- |
| transposed covariance predict `Fᵀ·P·F` | 4.37 | 96.3 | 0.76 | **red everywhere** (K2/K3/K4/K5) |
| `process_noise` `Q ×= 10` | 1.96 | 2.74 | 0.23 | **red via NEES** (< 3.0) |
| `process_noise` `Q ×= 0.1` | 1.97 | 4.00 | 0.15 | borderline in-window; drifts redder past it |
| half-gain `K` | 1.97 | 3.70 | 0.15 | **not caught here** — unit test catches it |

The **half-gain blind spot** is real and stated plainly: on clean CV truth a half-gain
filter still tracks a straight line and stays nearly consistent, so no aggregate
statistic here flags it. It is caught deterministically by `test_kalman`'s
`UpdateStateMatchesHandComputed` (the PR #11 half-gain mutation), and it becomes
statistically visible only under a maneuver (M2 / CT scenario). Phase K's remit is
covariance-structure consistency + convergence; the bands are tuned to that and
documented accordingly.

## 8. What this unblocks

A statistically verified filter. After Phase K the remaining M1 steps — `io`, `metrics`
(RMSE), `apps/radar_sim`, and the README plot — build on a keystone whose consistency is
demonstrated, not assumed. M2 tuning (`σ_a` against NIS under CT maneuver mismatch) starts
from a filter known to be correctly *implemented*.
