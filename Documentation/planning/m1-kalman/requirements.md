# M1 · `kalman` module — Requirements

## Purpose

The constant-velocity linear Kalman filter: the keystone of the project. Every tracking
result depends on it, so this module and its tests get the "careful" budget, and the
exhaustive verification lives in **Phase K** (the next phase), which this module unblocks.

`kalman` owns the filter *primitives* the spec enumerates: **predict**, **Joseph-form
update**, **innovation**, its **covariance S**, the **gate distance d²**, and **track
initialization**. It does **not** own the gating/association *policy* (which measurement is
assigned to which track — the greedy nearest-neighbour loop); that is `association`/`tracker`
in M2, built on the d² primitive here.

## Scope

In scope (M1):
- `KalmanConfig` carrying the tuning: `dt` (scan interval, s), `sigma_a` (process-noise
  acceleration std-dev, m/s²), `sigma_m` (measurement noise std-dev per axis, m).
- Model matrices:
  - `F` — reused from `motion::cv_transition_matrix(dt)` (single source of truth, ADR-0006).
  - `process_noise(dt, sigma_a) -> Eigen::Matrix4d` — the discrete white-noise-acceleration Q.
  - `measurement_matrix() -> Eigen::Matrix<double,2,4>` — H (selects position).
  - `measurement_noise(sigma_m) -> Eigen::Matrix2d` — R = diag(σ_m², σ_m²).
- `predict(const TrackState&, const KalmanConfig&) -> TrackState` — `x̂ = F·x̂`,
  `P = F·P·Fᵀ + Q`.
- `initiate_track(const Measurement&, const KalmanConfig&) -> TrackState` — one-point init:
  `x̂ = [z_x, z_y, 0, 0]ᵀ`, `P₀ = diag(σ_m², σ_m², σ_v0², σ_v0²)`, `σ_v0 = 150 m/s`.
- Innovation primitives (consumed by association/metrics):
  - `innovation(const TrackState&, const Measurement&) -> Eigen::Vector2d` — `y = z − H·x̂`.
  - `innovation_covariance(const TrackState&, const KalmanConfig&) -> Eigen::Matrix2d` —
    `S = H·P·Hᵀ + R`.
  - `gate_distance(const Eigen::Vector2d& y, const Eigen::Matrix2d& S) -> double` —
    Mahalanobis `d² = yᵀ·S⁻¹·y`.
- `update(const TrackState&, const Measurement&, const KalmanConfig&) -> TrackState` — the
  **Joseph-form** measurement update, symmetrized.
- The χ² gate threshold constant `kGate99 = 9.21` (2 DOF, 99%) for callers.

Out of scope / deferred:
- Gating/association **policy** (candidate sort, greedy assignment) — `association`/`tracker`, M2.
- Track lifecycle (M-of-N confirm, hit/miss counters, coasting/deletion) — `track`/`tracker`, M2.
- Two-point-differencing init upgrade — documented spec fallback, only if plots show slow
  convergence (M2+).
- Coordinated-turn process model — CT is deferred in `motion`; the filter stays CV even when
  truth maneuvers (that model mismatch is intentional, ADR-0006).

## Acceptance criteria

### AC-1 — Model matrices match the spec exactly
- `F == motion::cv_transition_matrix(dt)` (reused, not redefined).
- `process_noise(dt, σ_a)` equals `σ_a² · [[dt⁴/4,0,dt³/2,0],[0,dt⁴/4,0,dt³/2],
  [dt³/2,0,dt²,0],[0,dt³/2,0,dt²]]`, checked against a **literal** matrix built from the spec.
- `H` is the 2×4 `[[1,0,0,0],[0,1,0,0]]`; `R = diag(σ_m², σ_m²)`.

### AC-2 — Predict is `F·x` / `F·P·Fᵀ + Q`, verified independently (issue #6)
- `predict(state).x` advances position by `v·dt`, velocity unchanged — checked with
  **hand-computed** values (literal arithmetic), **not** by calling `cv_transition_matrix`.
  This is the common-mode guard from ADR-0006 / issue #6: a bug in `F` must not be able to hide
  by appearing on both the production and the test side.
- `predict(state).P` equals `F·P·Fᵀ + Q` and is symmetric; process noise strictly increases
  uncertainty (`P_pred − P` is PSD for the diagonal entries).

### AC-3 — Track initialization
`initiate_track(z)` yields `x̂ = [z_x, z_y, 0, 0]` and `P₀ = diag(σ_m², σ_m², 150², 150²)`
exactly.

### AC-4 — Innovation, S, and gate distance
- `innovation(state, z) == z − H·x̂` (exact).
- `innovation_covariance == H·P·Hᵀ + R` (the top-left 2×2 position block of P plus R).
- `gate_distance(y, S) == yᵀ·S⁻¹·y`: zero for `y == 0`; for `S = σ²·I`, `d² = ‖y‖²/σ²`
  (hand-checkable); non-negative and finite for any PSD `S`.

### AC-5 — Joseph update is correct and numerically sound
- For a measurement equal to the prediction (`y = 0`), the state is unchanged and `P` only
  shrinks (no spurious motion).
- The posterior `P` is **symmetric** (after `symmetrize`) and **positive semi-definite**
  (eigenvalues ≥ 0), and no larger than the prior in the position block (an update cannot
  increase certainty-weighted uncertainty).
- A single update with a finite-`P` prior and a real measurement reduces the position-block
  trace of `P` versus the prior.
- Cross-check: on a well-conditioned case the Joseph result matches the textbook
  `(I − K·H)·P` form to tolerance (they differ only in rounding, which is the point of Joseph).

### AC-6 — Convergence sanity (filter actually tracks)
Feeding a straight-line CV truth through a Gaussian sensor (reusing `motion` + `sensor`) and
running predict/update each scan, the estimate's position error settles below the raw
measurement noise within a handful of scans (a smoke-level check; the rigorous NEES/NIS battery
is Phase K).

### AC-7 — Invalid inputs rejected
Non-finite or non-positive `dt`, non-finite/negative `sigma_a` or `sigma_m`, and non-finite
measurement/state inputs are rejected (throw `std::invalid_argument`), consistent with the
`sensor`/`motion` precedent — a filter fed NaN silently poisons every downstream track.

### AC-8 — Tooling + tested
- `clang-format --dry-run --Werror` clean; `ci`/`ci-clang` 0 warnings in our code.
- `tests/test_kalman.cpp` registered via `radar_add_test`, green under `release`, `ci`,
  `ci-clang`, and `asan`.

## Note on Phase K (the next phase)

This module deliberately stops at *correct primitives with unit-level tests*. The heavy
statistical verification — NEES (state consistency), NIS (innovation consistency), filter
convergence over Monte-Carlo runs, covariance credibility — is **Phase K**, run before any
tracker logic is built on top. The AC-6 convergence check here is only a smoke test so the
module is self-evidently functional; it is not the Phase K gate.
