# Phase K · Exhaustive Kalman verification battery — Requirements

## Purpose

`kalman` (M1) stops at *correct primitives with unit-level tests*. Phase K is the
deliberate, heavy-investment **statistical verification gate** that runs before any
tracker logic is built on top of the filter — the filter is the keystone the entire
project depends on, so it earns its own phase (see `../00-overview.md`, step 5 of M1).

The battery verifies that the filter is **statistically consistent**: its claimed
uncertainty (the covariance `P` and innovation covariance `S`) matches the errors it
actually makes, measured over fixed-seed Monte-Carlo runs that reuse `motion` (truth) and
`sensor` (Gaussian noise). The instruments are the standard filter-consistency statistics:
**NEES** (state consistency, χ²(4)), **NIS** (innovation consistency, χ²(2)), **convergence**
over an ensemble, and **covariance credibility**.

**Model-matched truth (the design that makes NEES a real χ²(4) test).** The truth is
propagated with the filter's own process-noise model injected:

> truth<sub>k+1</sub> = F · truth<sub>k</sub> + chol(Q) · w,  w ~ N(0, I₄)

so truth and filter share the same (F, Q) dynamics and the filter is correctly specified.
Then posterior NEES = `eᵀ·P⁻¹·e` is genuinely χ²(4) (mean 4.0) over the **whole** post-burn-in
tail — a real steady-state consistency result, not a transient passing through a band. The
`chol(Q)` factor is built from an **independent, hand-written Q literal** (not
`radar::process_noise`): if truth drew its noise from the same function the filter predicts
with, a bug in `process_noise` would scale truth and filter identically and NEES could not
see it (the common-mode trap ADR-0006 names for F). An earlier revision propagated truth as
a noise-free straight line and measured NEES over a narrow early window where it happened to
pass ~3.6 on its way down to a steady ~2.4; PR #12 review correctly flagged that window as a
transient, not a χ²(4) result. Injecting Q is the fix.

## What this battery is — and is not

Being honest about scope keeps the thresholds meaningful and sets up M2 correctly.

- **It IS** a *consistency + common-bug* battery. With model-matched truth a correctly
  implemented filter sits at the theoretical χ² means (NEES ≈ 4.0, NIS ≈ 2.0). The
  discriminating power is against **implementation bugs** that break the covariance
  structure: a transposed covariance predict (`Fᵀ·P·F`), a **mis-scaled `Q` in either
  direction** (an over-confident `Q ×0.1` now fails as loudly as an under-confident `Q ×10`,
  because truth uses the independent literal Q), an `S` that omits `R`, a non-Joseph update
  that drifts non-PSD. Each such bug pushes NEES and/or NIS and/or the convergence ratio off
  its expected value by far more than the Monte-Carlo noise (measured in AC-K6 / design.md §7).
- **A known blind spot (stated honestly):** a wrong *gain scale* (e.g. a half-gain `K`) is
  **not** caught by this battery — a half-gain filter still tracks and stays nearly
  consistent (measured NEES ~4.0). That bug is caught deterministically at the unit level by
  `test_kalman`'s `UpdateStateMatchesHandComputed` (PR #11 mutation log). It only becomes
  statistically visible under a *maneuver* (M2 / CT scenario). Phase K's job is
  covariance-structure consistency + convergence, not gain scale, and the bands are tuned to
  that.
- **It is NOT** a tuning oracle. `σ_a = 5` is the spec default; with truth and filter sharing
  it by construction, the battery checks the filter is correctly *implemented*, not that
  `σ_a` is the right value for real data. Tuning against the NIS target is an **M2** concern,
  where CV filter vs CT truth is the mismatch that makes it bite.
- **Keystone check:** **NEES** is weighted highest. It exercises the full 4×4 `P`, including
  the **velocity** block that the position-only NIS never sees directly. Rank of
  discriminating power: **NEES > convergence-ratio ≈ NIS > covariance credibility.**

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
| consistency interval | full post-burn-in tail `[burn-in, K)` = 180 scans | NIS, NEES, and credibility — model-matched truth makes both statistics stationary here, so no window is needed |
| convergence tail | 30 scans | late tail `[K−30, K)` for the RMSE ratio (spec M1 gate) |
| `dt` | 0.1 s | spec |
| `σ_a` | 5.0 m/s² | spec default; drives BOTH the filter's Q and the truth's injected process noise |
| `σ_m` | 50.0 m | spec; sensor and filter agree |
| truth | CV + process noise | `propagate_cv(truth, dt) + chol(Q_literal)·w`, `w ~ N(0, I₄)` from the per-run RNG |

Samples per statistic, pooled across M = 100 runs ≈ M·(K−burn-in) = 100·180 = **18 000** for
NIS, NEES, and credibility alike (one interval now — the full post-burn-in tail).

## Acceptance criteria

### AC-K1 — NIS single-run time-average inside the spec band
One run (run 0 of the ensemble, reused — no re-simulation), the **time-averaged NIS** over
the **full post-burn-in tail** `[burn-in, K)` (~180 samples) lies in the spec's acceptance
band **[1.5, 2.6]** (mean target ~2.0, measured ~2.01). Finite. This is the spec's own "mean
NIS over a run" instrument restated as a test.

### AC-K2 — NIS Monte-Carlo ensemble mean (tight consistency gate)
Over the ensemble, NIS pooled across the full post-burn-in tail of every run (~18 000
samples), the **ensemble-mean NIS** lies in **[1.85, 2.15]**. NIS ~ χ²(2), mean 2.0 (measured
~2.01); a transposed-covariance predict pushes the pooled mean well outside (measured ~3.5).
NIS is position-innovation only, so a gain/`Q` bug barely moves it — that is by design; NEES
(AC-K3) carries those.

### AC-K3 — NEES state consistency (keystone)
Posterior (filtered) NEES per scan `e = x̂ − x_true`, `NEES = eᵀ·P⁻¹·e` ~ χ²(4), mean 4.0.
`P⁻¹·e` via `Eigen::LLT` — the **same SPD guard the production filter uses**: the helper
returns `+∞` on a non-SPD `P`, and AC-K3 asserts **every** sample was finite (`all_nees_finite`)
and the minimum is ≥ 0, so a regression that makes `P` indefinite (broken Joseph form, missing
symmetrize) fails loudly and is **reported under AC-K3** — not swallowed by a `std::min` (which
cannot catch NaN) nor misattributed to whichever test ran first. Measured over the **full
post-burn-in tail**, the **ensemble-mean NEES** lies in **[3.5, 4.6]** (measured ~4.03). Because
truth is model-matched (process noise injected), this is a genuine steady-state χ²(4) result
over the whole tail — no window. A lying covariance fails **both ways**: `Q ×10`
(under-confident) measures ~2.37 (< 3.5); `Q ×0.1` (over-confident) measures ~12.49 (> 4.6); a
transposed predict ~172. This is the check the position-only NIS cannot make — it sees the full
4×4 `P` including the velocity block.

### AC-K4 — Covariance credibility (variance matching)
Over the post-burn-in tail across the ensemble, the **empirical per-axis position-error
variance** matches the filter's **mean claimed** `P` position variance to within ~15% (each
axis; measured ratio ~1.03). This is NEES restated in physical units — a reader-facing sanity
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
recording which statistic fails. Measured outcomes (ensemble over the full post-burn-in
tail; correct filter = NIS 2.01 / NEES 4.03 / posVarRatio 1.03 / conv 0.18):

| Mutation (reverted after) | NIS | NEES | Caught by |
| --- | --- | --- | --- |
| transposed covariance predict `Fᵀ·P·F` | 3.50 | 172 | **all** (K2, K3, K4, K5) |
| `process_noise` `Q ×= 10` (under-confident) | ~2.0 | **2.37** | **K3** (NEES < 3.5) |
| `process_noise` `Q ×= 0.1` (over-confident) | ~2.0 | **12.49** | **K3** (NEES > 4.6) |
| half-gain `K` | ~2.0 | ~4.0 | **not here** — `test_kalman::UpdateStateMatchesHandComputed` (unit, deterministic) |

So the battery robustly catches the transposed-covariance bug (every statistic) and a
mis-scaled `Q` **in both directions** via NEES — the over-confident `Q ×0.1` that the earlier
windowed/noise-free design missed is now caught loudly (NEES 12.49), because truth draws its
process noise from the **independent literal Q** so a `process_noise` mutation creates a real
truth/filter mismatch instead of a common-mode scaling. The gain-scale bug is a documented
blind spot, caught at the unit level instead (see the scope section). A hand-mis-scaled `S`
asserted inside the test would only check test-side arithmetic and is deliberately **not**
included — the mutations are against *production* code.

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
