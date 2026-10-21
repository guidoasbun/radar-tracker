# Phase K · Exhaustive Kalman verification battery — Requirements

## Purpose

`kalman` (M1) stops at *correct primitives with unit-level tests*. Phase K is the
deliberate, heavy-investment **statistical verification gate** that runs before any
tracker logic is built on top of the filter — the filter is the keystone the entire
project depends on, so it earns its own phase (see `../00-overview.md`, step 5 of M1).

The battery verifies that the filter is **statistically consistent**: its claimed
uncertainty (the covariance `P` and innovation covariance `S`) matches the errors it
actually makes, measured over fixed-seed Monte-Carlo runs that reuse `motion` (truth)
and `sensor` (Gaussian noise). The instruments are the standard filter-consistency
statistics: **NEES** (state consistency, χ²(4)), **NIS** (innovation consistency,
χ²(2)), **convergence** over an ensemble, and **covariance credibility**.

## What this battery is — and is not

Being honest about scope keeps the thresholds meaningful and sets up M2 correctly.

- **It IS** a *self-consistency + common-bug* battery. On matched CV-truth / CV-filter
  data with no model mismatch, a correctly implemented filter sits near the theoretical
  χ² means almost by construction. The discriminating power is therefore against
  **implementation bugs** that break the covariance structure: a transposed covariance
  predict (`Fᵀ·P·F`), a missing/mis-scaled `Q`, an `S` that omits `R`, a non-Joseph
  update that drifts non-PSD. Each such bug pushes NEES and/or NIS and/or the
  convergence ratio off its expected value by far more than the Monte-Carlo noise
  (measured in §AC-K6 / design.md §7).
- **A known blind spot (stated honestly):** a wrong *gain scale* (e.g. a half-gain `K`)
  is **not** caught by this battery on clean CV truth — a half-gain filter still tracks a
  straight line and stays nearly consistent (measured NEES ~3.7, convergence even
  slightly better). That bug is caught deterministically at the unit level by
  `test_kalman`'s `UpdateStateMatchesHandComputed` (PR #11 mutation log). The gain scale
  only becomes statistically visible under a *maneuver* (M2), which is the point of the
  CT scenario. Phase K's job is covariance-structure consistency + convergence, not gain
  scale, and the thresholds are tuned to that.
- **It is NOT** a tuning oracle. Because truth is CV and the filter is CV, `σ_a = 5`
  is not uniquely "correct" here — a range of `σ_a` looks consistent on position-only
  data. Tuning against the NIS target is an **M2** concern, where the maneuver scenario
  (CV filter vs CT truth) is the model mismatch that actually makes NIS deviate.
- **Keystone check:** **NEES** is weighted highest. It exercises the full 4×4 `P`,
  including the **velocity** block that the position-only NIS never sees directly. Rank
  of discriminating power: **NEES > convergence-ratio ≈ NIS > covariance credibility.**

## Form (decided — Option A)

A new `tests/test_phase_k.cpp`, registered via `radar_add_test`, running in the CI gate
(`release`, `ci`, `ci-clang`, `asan`). Consequences this imposes:

- **Fast:** bounded Monte-Carlo (M = 100 runs × K = 200 scans), sub-second even under
  `asan`.
- **Deterministic / no flakiness:** one `std::mt19937_64` **per run**, seeded
  `kBaseSeed + run_index`, threaded through the init measurement and every scan's
  `sense` (mirrors how the real sim threads one generator). Fixed seeds throughout.
- **Thresholds earn their width:** bands are set from χ² sampling theory with a
  deliberate margin for the serial correlation *within* a run (consecutive scans share
  state, so the effective sample size is below `M·K`); the chosen M makes the tight
  bands safe, verified non-flaky by repeated runs during the verify step.
- **Validation style:** `EXPECT_THROW`, never NDEBUG-guarded death tests.

## Monte-Carlo configuration

| Parameter | Value | Rationale |
| --- | --- | --- |
| `kBaseSeed` | 42 | spec default scenario seed |
| M (runs) | 100 | earns the tight NIS/NEES ensemble bands against within-run correlation |
| K (scans/run) | 200 | long enough for convergence + a long measured tail |
| burn-in | 20 scans | discard the one-point-init transient before measuring consistency |
| NIS interval | full post-burn-in tail `[burn-in, K)` = 180 scans | NIS does not drift (matches the spec's "average over all post-burn-in scans") |
| NEES window | 40 scans | `[burn-in, burn-in+40)` — where χ²(4) holds *before* benign steady-state under-confidence; NEES + credibility measured here (NEES, unlike NIS, drifts over the full tail) |
| convergence tail | 30 scans | late tail `[K−30, K)` for the RMSE ratio (spec M1 gate) |
| `dt` | 0.1 s | spec |
| `σ_a` | 5.0 m/s² | spec default (the one tuned knob; fixed here, tuned in M2) |
| `σ_m` | 50.0 m | spec; sensor and filter agree |
| truth | straight-line CV | `propagate_cv`; crossing the field at aircraft speed |

Samples per statistic (pooled across the M = 100 runs): **NIS ≈ M·(K−burn-in) = 100·180 =
18 000** (full post-burn-in tail); **NEES / credibility ≈ M·window = 100·40 = 4 000** (the
NEES window). The difference is deliberate — see the NIS/NEES interval rows above and
design.md §4.

## Acceptance criteria

### AC-K1 — NIS single-run time-average inside the spec band
One run (run 0 of the ensemble, reused — no re-simulation), the **time-averaged NIS** over
the **full post-burn-in tail** `[burn-in, K)` (~180 samples) lies in the spec's acceptance
band **[1.5, 2.6]** (mean target ~2.0, measurement dim = 2). Finite. This is the spec's own
"mean NIS over a run" instrument restated as a test.

### AC-K2 — NIS Monte-Carlo ensemble mean (tight consistency gate)
Over the ensemble, NIS pooled across the **full post-burn-in tail** of every run (~18 000
samples), the **ensemble-mean NIS** lies in **[1.85, 2.15]**. NIS ~ χ²(2), mean 2.0; the
pooled-mean sampling spread (with margin for within-run correlation, earned by M = 100)
keeps a correct filter comfortably inside, while a transposed-covariance bug lands well
outside (measured ~4.4). Using the full tail — not the NEES window — is deliberate: NIS does
not drift, so this both matches the spec and catches a covariance bug that appears only
after the window.

### AC-K3 — NEES state consistency (keystone)
Posterior (filtered) NEES per scan `e = x̂ − x_true`, `NEES = eᵀ·P⁻¹·e` ~ χ²(4), mean 4.0.
`P⁻¹·e` via `Eigen::LLT` — the **same SPD guard the production filter uses**: the test checks
`llt.info() == Success` and that **every** per-scan NEES is finite and ≥ 0, so a regression
that makes `P` indefinite (broken Joseph form, missing symmetrize) fails loudly instead of
feeding a meaningless solve into the band (not a post-hoc `std::min`, which cannot catch NaN). Measured over the **consistency window** `[burn-in, burn-in + 40)`, the
**ensemble-mean NEES** lies in **[3.0, 4.5]** (measured ~3.6). On perfectly-CV truth the
filter's covariance reaches a steady state whose velocity block is slightly larger than the
velocity error it achieves, so the *full-run-tail* NEES drifts below 4 (a benign, correct
under-confidence) — the window, not the whole tail, is where the χ²(4) expectation holds and
the principled band lives. This is the check a lying covariance (over/under-confident `P`,
especially the velocity block) fails — and the one the position-only NIS cannot make.

### AC-K4 — Covariance credibility (variance matching)
Over the consistency window across the ensemble, the **empirical per-axis position-error
variance** matches the filter's **mean claimed** `P` position variance to within ~15% (each
axis; measured ratio ~0.99). This is NEES restated in physical units — a reader-facing sanity
check that the filter's stated metres-of-uncertainty are real. (An explicit 1-σ
coverage-fraction assertion is intentionally **omitted**: at this sample size its binomial
spread is too wide to be both non-flaky and discriminating — variance matching carries the
same signal cleanly.)

### AC-K5 — Convergence over Monte-Carlo
Per run, the spec M1 ratio RMS(est−truth)/RMS(z−truth) over the converged tail. The
**ensemble-mean ratio ≤ 0.5** (the spec gate) and **every run ≤ 0.6** (no pathological
run). The filtered tail position-error variance is strictly below raw `σ_m²`. A
pass-through (`est = z`) gives ratio 1.0 and fails — the check that distinguishes a real
filter from a copy of the measurement.

### AC-K6 — Mutation-resistance (verification obligation, not a test)
The bands must be shown to **go red under a wrong filter**, verified during the Phase K
verify step by temporarily perturbing production `kalman` (each reverted after) and
recording which statistic fails. Measured outcomes (ensemble over the window; correct
filter = NIS 1.97 / NEES 3.61 / posVarRatio 0.99 / conv 0.18):

| Mutation (reverted after) | NIS | NEES | conv ratio | Caught by |
| --- | --- | --- | --- | --- |
| transposed covariance predict `Fᵀ·P·F` | 4.37 | 96.3 | 0.76 | **all** (K2, K3, K4, K5) |
| `process_noise` `Q ×= 10` | 1.96 | **2.74** | 0.23 | **K3** (NEES < 3.0) |
| `process_noise` `Q ×= 0.1` | 1.97 | 4.00 | 0.15 | borderline (drifts further out past the window) |
| half-gain `K` | 1.97 | 3.70 | 0.15 | **not here** — `test_kalman::UpdateStateMatchesHandComputed` (unit, deterministic) |

So the battery robustly catches the transposed-covariance bug (every statistic) and the
inflated-`Q` over-confidence/under-confidence via NEES; the gain-scale bug is a documented
blind spot on clean CV, caught at the unit level instead (see the scope section). A
hand-mis-scaled `S` asserted inside the test would only check test-side arithmetic and is
deliberately **not** included — the mutations are against *production* code.

### AC-K7 — Tooling + CI
- `clang-format --dry-run --Werror` clean; `ci`/`ci-clang` 0 warnings in our code.
- `tests/test_phase_k.cpp` registered via `radar_add_test`; green under `release`, `ci`,
  `ci-clang`, and `asan`.
- Eigen sanitizer/analyzer false positives reuse the existing
  `cmake/sanitizer-ignorelist.txt` (UBSan implicit-conversion) and `.clang-tidy`
  (`clang-analyzer-security.ArrayBound`) handling; no new suppressions unless a genuinely
  new Eigen path trips, documented if so.

## Out of scope (deferred)

- Tuning `σ_a` against NIS — M2, where CT-truth vs CV-filter mismatch makes it bite.
- Multi-target / clutter / gating-policy consistency — M2 (`association`/`tracker`).
- Predicted (vs filtered) NEES and smoothing consistency — not needed for the M1 gate.
