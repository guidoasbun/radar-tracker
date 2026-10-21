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
- `gate_distance(y, S)`: zero for `y == 0`; for `S = σ²·I`, `d² = ‖y‖²/σ²` (hand-checkable);
  non-negative by construction. S's positive-definiteness is verified by a **Cholesky
  (`Eigen::LLT`)** factorization, **not** `det(S) > 0` — a 2×2 negative-definite S (e.g. `−I`)
  also has positive determinant, which the old check accepted and returned a *negative* d²
  that passes the gate for any measurement (fail-open). `gate_distance` throws on a non-finite
  `y`, an asymmetric S, or a non-positive-definite S (tested with `S = −I` and an asymmetric S).

### AC-5 — Joseph update is correct and numerically sound
- `update` reuses `innovation` and `innovation_covariance` (one definition of `y` and `S`,
  shared with the gate) and the same Cholesky SPD guard — so a negative-definite prior `P`
  (hence negative-definite S) is rejected rather than producing a plausible-looking but corrupt
  posterior (tested).
- For a measurement equal to the prediction (`y = 0`), the state is unchanged and `P` only
  shrinks (no spurious motion).
- The posterior `P` is **symmetric** (after `symmetrize`) and **positive semi-definite**
  (eigenvalues ≥ 0); the position-block trace decreases versus the prior.
- **Hand-computed update state (M2):** with a non-diagonal prior `P` (position var 400, velocity
  var 100, position–velocity covariance 40 per axis), `σ_m = 20` (so `S = 800·I`), `x = (10, 20,
  1, −1)`, `z = (12, 19)`, the posterior is exactly `x = (11, 19.5, 1.1, −1.05)` (gains 0.5 /
  0.05). This exercises the **velocity** gain (the diagonal-P Joseph cross-check leaves it zero)
  and catches a wrong gain scale (e.g. a half-gain bug).
- Cross-check: on a well-conditioned diagonal case the Joseph result matches the textbook
  `(I − K·H)·P` form to tolerance (they differ only in rounding, the point of Joseph).

### AC-6 — Convergence: track RMSE ≤ 0.5 × raw RMSE (the spec's M1 gate)
Feeding a straight-line CV truth through a Gaussian sensor (reusing `motion` + `sensor`) and
running predict/update each scan, the **ratio** RMS(`est − truth`) / RMS(`z − truth`) over the
converged tail is `≤ 0.5` — the spec's actual M1 acceptance gate. Testing the ratio (not a loose
absolute bound) is what distinguishes a real filter from a pass-through: setting the estimate to
the raw measurement gives ratio 1.0 and fails. (The rigorous NEES/NIS battery is still Phase K.)

### AC-7 — Invalid inputs rejected
Non-finite or non-positive `dt`, non-finite/negative `sigma_a`, non-finite/**non-positive**
`sigma_m` (the filter requires `σ_m > 0`, unlike the sensor, so a zero does not surface later as
a confusing singular-S error), and non-finite measurement/state inputs are rejected (throw
`std::invalid_argument`), consistent with the `sensor`/`motion` precedent — a filter fed NaN
silently poisons every downstream track.

### AC-2b — Predicted covariance, hand-computed (M2)
With `P = I`, `dt = 0.5`, `σ_a = 0`, the predicted covariance is exactly
`F·Fᵀ = [[1.25,0,0.5,0],[0,1.25,0,0.5],[0.5,0,1,0],[0,0.5,0,1]]`, checked against a literal
matrix. This pins the order `F·P·Fᵀ` (a transposed `Fᵀ·P·F` bug puts the `1.25` in the velocity
block instead) — the covariance analogue of the issue-#6 hand-computed state check.

### AC-config — Default `σ_a` is the spec value
`KalmanConfig::sigma_a` defaults to **5.0 m/s²** (the spec parameter table and example
scenario), not a placeholder — an under-value would make Phase K's NEES/NIS judge an
overconfident filter. It remains the one genuinely tuned knob, refined in M2.

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
