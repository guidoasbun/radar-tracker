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
  std::vector<double> nis;   // y^T S^-1 y over [burn-in, K)   (chi^2_2, full tail)
  std::vector<double> nees;  // e^T P^-1 e over [burn-in, K)   (chi^2_4, full tail)
  std::vector<Eigen::Vector2d> pos_err;        // position error per post-burn-in scan
  std::vector<Eigen::Vector2d> pos_var_claim;  // P(px,px), P(py,py) per post-burn-in scan
  double tail_track_sq{0.0};  // sum |est_pos - truth_pos|^2 over the late tail
  double tail_raw_sq{0.0};    // sum |z - truth_pos|^2 over the late tail
};

RunSamples run_filter(std::uint64_t seed);  // intervals from file-scope constants
```

NEES goes through `nees(e, P)`, which runs the production SPD guard (`Eigen::LLT`) and
returns `+∞` if `info() != Success`; AC-K3 then asserts every sample was finite and the
minimum ≥ 0, so an indefinite `P` fails under AC-K3 rather than raising a gtest expectation
inside the shared ensemble builder (where it would be misreported). NIS, NEES, and
credibility all span the full post-burn-in tail — the model-matched truth makes both
statistics stationary there, so no window is needed.

Inside `run_filter`:
- **One** `std::mt19937_64 rng{seed}` for the whole run, threaded through the init `sense`,
  the per-scan process-noise draw `w`, and every scan's `sense` (fixed order: `w` before the
  sensor draw). This mirrors the real sim's single-generator contract and makes "runs
  independent, a run reproducible" literally true.
- **Truth obeys the filter's own model, sampled through the noise gain `G`:**
  `truth = propagate_cv(truth, dt)`, then per axis draw `a ~ N(0, σ_a²)` and add
  `a·[dt²/2, dt]` to `[pos, vel]`. This has covariance `σ_a²·G·Gᵀ` — exactly the filter's
  `Q` — with **no factorization**: `Q` is rank 2 / positive *semi*-definite (not SPD), so a
  `chol(Q)` does not exist in exact arithmetic (see §4). Sampling the acceleration through
  `G` is independent of `radar::process_noise` and holds for any `dt`/`σ_a`. Initial truth is
  a straight-line CV target at aircraft speed (`v = (200, −100)`).
- Filter: `initiate_track` from the first measurement, then per scan
  `est = update(predict(est, cfg), z, cfg)`.
- Per scan, **before** the update, compute NIS from the *predicted* state (`innovation` /
  `innovation_covariance`) — the textbook NIS. Compute NEES from the *posterior* vs truth.
- Collect NIS/NEES/pos_err/pos_var_claim over `[burn-in, K)`; accumulate the convergence
  sums over the late tail `[K−tail, K)`.

Ensemble: `for run in [0, M): acc += run_filter(kBaseSeed + run)`, pooling the per-scan
vectors. Means are plain pooled averages; run 0's single-run NIS mean is kept for AC-K1.

## 3. The statistics, mapped to code

| Statistic | Definition | Code |
| --- | --- | --- |
| NIS | `yᵀ S⁻¹ y`, 2 DOF | `y = innovation(pred, z)`; `S = innovation_covariance(pred, cfg)`; `gate_distance(y, S)` **is exactly `yᵀS⁻¹y`** with the SPD guard — reuse it rather than re-inverting. Collected over the full post-burn-in tail. |
| NEES | `eᵀ P⁻¹ e`, 4 DOF | `e = est.x − truth`; `nees(e, P)` runs `Eigen::LLT`, returns `+∞` if `info() != Success`, else `e.dot(llt.solve(e))`. Collected over the full post-burn-in tail; AC-K3 checks finiteness + `min ≥ 0`. |
| convergence ratio | RMS(est−truth)/RMS(z−truth) | pooled `sqrt(Σ track_sq / Σ raw_sq)` over the late tail; plus the absolute check `tail_err_var < σ_m²` |
| credibility | Var(err) vs mean claimed `P_pos` | per-axis second moment of `pos_err` vs mean `pos_var_claim`, both over the full post-burn-in tail |

Reusing `gate_distance` for NIS is deliberate: it is the exact Mahalanobis form the
filter ships, including the Cholesky SPD guard, so the test measures the production
quantity, not a re-derivation that could silently differ. NEES uses the same LLT guard via
`nees`, so an indefinite-`P` regression surfaces (as `+∞`) and fails AC-K3 rather than
landing in-band.

## 4. Why these thresholds (theory + measurement)

- NIS ~ χ²(2): mean 2, variance 4. NEES ~ χ²(4): mean 4, variance 8.
- **Model-matched truth → χ²(4) over the whole tail.** Because truth is driven by the
  filter's own `(F, Q)` dynamics (process noise injected, §2), the filter is correctly
  specified and NEES is a genuine χ²(4) statistic over the **full post-burn-in tail**
  `[kBurnIn, kScans)` — no window. NIS, NEES, and credibility all use that one interval.
  (The earlier noise-free-truth revision had to window NEES because a permanently
  under-confident filter made the full-tail NEES drift to ~2.4; injecting Q removed that
  drift. PR #12 review flagged the window as a transient, which it was.) Measured on the
  correct filter (M = 100): **NIS ≈ 1.99, NEES ≈ 3.93, posVarRatio ≈ 1.00, convergence ≈
  0.20.**
- **No factorization, and independence from `process_noise` (the crucial subtlety).** Truth's
  process noise is sampled per axis as `a·[dt²/2, dt]`, `a ~ N(0, σ_a²)`, giving covariance
  `σ_a²·G·Gᵀ`. That matrix is **rank 2 / positive semi-definite — not SPD** (each 2×2 axis
  block is a rank-1 outer product, determinant exactly 0), so a `chol(Q)` via `LLT` does not
  exist in exact arithmetic and only "succeeded" at `dt=0.1 / σ_a=5` by a +5.6e-17 rounding
  fluke (PR #12 re-review). Sampling the acceleration through `G` sidesteps that entirely and
  is also independent of `radar::process_noise`: a `Q`-scaling mutation then changes only the
  filter, a real truth/filter mismatch NEES catches. Routing truth through `process_noise`
  (or a hand-copied `Q` literal) would make the mutation common-mode (ADR-0006's F trap) —
  observed directly: with a shared-`Q` path `Q ×0.1` *passed*; through `G` it fails at NEES
  ~12.6.
- **Sample counts:** NIS, NEES, credibility each ≈ 100·180 = 18 000.
- **Bands, and why they are where they are** (every number measured, then
  mutation-verified — §7):
  - NIS ensemble **[1.85, 2.15]** — correct 1.99; the only mutation that moves it is the
    transposed predict (→3.56). Gain/Q leave NIS ≈ 2.0 because NIS sees only the position
    innovation; that is expected, not a weakness (NEES carries the covariance structure).
  - NEES ensemble **[3.5, 4.6]** — correct 3.93; catches the transposed predict (→172), the
    10×-`Q` under-inflation (→2.30 < 3.5) **and** the 0.1×-`Q` over-inflation (→12.6 > 4.6).
    This is the keystone band, and it now bites in **both** directions.
  - Convergence **mean ≤ 0.5 / every run ≤ 0.6** + `tail_err_var < σ_m²` — the spec M1 gate;
    correct 0.18, pass-through 1.0, transposed predict fails.
  - Credibility **ratio within ±15%/axis** — correct ~1.00; a gross structural bug
    (transposed predict) sends it to ~86.
- **Sampling spread vs band width.** If the ~18 000 samples were independent, the pooled-mean
  SE would be `sqrt(4/18000) ≈ 0.015` (NIS) and `sqrt(8/18000) ≈ 0.021` (NEES). Samples
  within a run are serially correlated, so the effective N is lower and the true SE larger;
  M = 100 keeps the between-run spread small enough that the bands (±0.15 NIS, −0.5/+0.6 NEES
  around the measured means) are many SEs wide — non-flaky, confirmed by repeated fixed-seed
  runs in the verify step.

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
(ensemble over the full post-burn-in tail; correct = NIS 1.99 / NEES 3.93 / conv 0.20):

| Mutation (reverted after) | NIS | NEES | Result |
| --- | --- | --- | --- |
| transposed covariance predict `Fᵀ·P·F` | 3.56 | 172 | **red everywhere** (K2/K3/K4/K5) |
| `process_noise` `Q ×= 10` (under-confident) | ~2.0 | 2.30 | **red via NEES** (< 3.5) |
| `process_noise` `Q ×= 0.1` (over-confident) | ~2.0 | 12.6 | **red via NEES** (> 4.6) |
| half-gain `K` | ~2.0 | ~4.0 | **not caught here** — unit test catches it |

Both `Q` directions are caught **because** truth is sampled through the independent G-vector model: a
`process_noise` mutation changes only the filter, creating a real truth/filter mismatch. (A
prior attempt that drove truth from `process_noise` directly made `Q ×0.1` *pass* — the
common-mode trap; sampling truth through the independent G-vector model is what fixed it. Recorded here so the
next reader does not "simplify" it back.)

The **half-gain blind spot** is real and stated plainly: a half-gain filter still tracks and
stays nearly consistent (NEES ~4.0), so no aggregate statistic here flags it. It is caught
deterministically by `test_kalman`'s `UpdateStateMatchesHandComputed` (the PR #11 half-gain
mutation), and becomes statistically visible only under a maneuver (M2 / CT scenario). Phase
K's remit is covariance-structure consistency + convergence; the bands are tuned to that.

## 8. What this unblocks

A statistically verified filter. After Phase K the remaining M1 steps — `io`, `metrics`
(RMSE), `apps/radar_sim`, and the README plot — build on a keystone whose consistency is
demonstrated, not assumed. M2 tuning (`σ_a` against NIS under CT maneuver mismatch) starts
from a filter known to be correctly *implemented*.
