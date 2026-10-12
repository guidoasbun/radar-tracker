# M1 · `motion` module — Requirements

## Purpose

Propagate a target's **true** state forward in time under the constant-velocity (CV)
model. This is the truth generator the simulator uses to advance each target by one scan
interval, and it is what the Phase K Monte-Carlo tests rely on to produce trajectories to
feed the sensor and filter. It also owns the CV state-transition matrix `F`, which the
`kalman` module reuses for its predict step so the filter's model and the truth model are
defined in exactly one place (that `kalman → motion` edge is a deliberate deviation from the
spec's module table — see ADR-0006).

CV is the whole of M1's motion. The coordinated-turn (CT) maneuver model is specified but
**deliberately deferred** — see "Deferred: CT" below — and must be completed before the
maneuver scenarios are exercised (M2+).

## Scope

In scope (M1):
- `cv_transition_matrix(dt) -> Eigen::Matrix4d` — the CV state-transition matrix `F` for a
  step of `dt` seconds, over the state `x = [p_x, p_y, v_x, v_y]`.
- `propagate_cv(const Eigen::Vector4d& x, double dt) -> Eigen::Vector4d` — advance a state
  vector one CV step: `p_{k+1} = p_k + v_k·dt`, velocity unchanged. For finite inputs, equal
  to `x_{k+1} = F·x_k`. It takes a **state vector, not a `TrackState`** — truth has no
  covariance, and `P` propagation (`F·P·Fᵀ + Q`) is the filter's job (see AC-5).
- A finite-input precondition on `propagate_cv` (`dt` and `x` finite), asserted in debug/ASan.
- No RNG, no noise: this is deterministic truth. (Measurement noise and process noise live
  in `sensor` and `kalman` respectively.)

Out of scope / deferred:
- **CT (coordinated turn)** — deferred but documented (see below).
- Process noise `Q` and measurement noise `R` — these belong to `kalman`/`sensor`, not to
  truth motion. `motion` only provides `F`.
- Maneuver-segment scheduling (which target turns, when, at what rate) — that is scenario/
  simulator wiring (M2), and depends on CT.
- Surveillance-square entry/exit culling — a simulator/sensor concern, not motion.

## Deferred: CT (coordinated turn) — must be completed later

The spec defines a CT model for maneuver segments. It is **not** implemented in M1, but is
recorded here so it is not forgotten and so the M1 interface leaves room for it:

> During a maneuver segment a target flies a coordinated turn at turn rate ω, applied as a
> rotation of the velocity followed by an Euler position step:
> `v_{k+1} = Rot(ω·dt)·v_k`, then `p_{k+1} = p_k + v_{k+1}·dt`
> where `Rot(θ) = [[cos θ, −sin θ], [sin θ, cos θ]]`.
> A target may have several maneuver segments, each with its own time window and turn rate.

Completion criteria for the deferred work (tracked for M2):
- `ct_propagate(const Eigen::Vector4d& x, double dt, double omega) -> Eigen::Vector4d`
  implementing the formula above (note the position step uses the **new** velocity
  `v_{k+1}`). Signature mirrors `propagate_cv` (state vector, not `TrackState`).
- `omega == 0` must reduce **exactly** (bit-for-bit `==`) to CV: `cos(0)==1`, `sin(0)==0`
  are exact, so the continuity test demands `==`, not `isApprox`.
- A maneuver-segment type (time window + turn rate) and the simulator logic that selects
  CV vs CT per target per scan.

## Acceptance criteria

### AC-1 — CV transition matrix is correct
`cv_transition_matrix(dt)` equals, to within floating-point tolerance:
```
[[1, 0, dt, 0],
 [0, 1, 0, dt],
 [0, 0, 1,  0],
 [0, 0, 0,  1]]
```
and `cv_transition_matrix(0)` is the identity. The test compares against a **literal** matrix
written from the spec (via `<<`), not the implementation's construction recipe, so it is an
independent check (see Deviations / common-mode note).

### AC-2 — Propagation matches the CV law
- `propagate_cv(x, dt)` leaves velocity unchanged and advances position by `v·dt`:
  `p' = p + v·dt`, `v' = v`.
- It is consistent with the matrix form: `propagate_cv(x, dt) == F·x` (same values to
  tolerance), where `F = cv_transition_matrix(dt)`.

### AC-3 — Degenerate, negative, and multi-step behaviour
- `dt == 0` is a no-op (state unchanged, exact equality).
- A target with zero velocity does not move (exact equality).
- `dt < 0` propagates backward; `propagate_cv(propagate_cv(x, dt), -dt)` returns to `x`.
- N successive steps of `dt` equal one step of `N·dt` in position (CV is linear and
  time-homogeneous): verified against a closed-form straight-line trajectory, including the
  direct path-length check `(p_N − p_0).norm() == ‖v‖·(N·dt)`.

### AC-4 — Deterministic, no allocation surprises
- No randomness; same input → same output.
- Fixed-size Eigen types only (no heap allocation in the hot path).

### AC-5 — Covariance handling is enforced by the signature
- `propagate_cv` advances the state vector `x` only. There is deliberately **no**
  `TrackState` overload: truth has no covariance, and `P` propagation (`F·P·Fᵀ + Q`,
  including the process noise `Q`) is the filter's job in `kalman`. This is enforced at
  compile time by a `static_assert` in the test that `propagate_cv` is not invocable with a
  `TrackState`, so the contract cannot silently drift (it is not merely documented).

### AC-6 — Finite-input precondition
- `propagate_cv` (and `cv_transition_matrix`) assert finite `dt` (and finite `x`) in
  debug/ASan builds; `release`/`ci` compile the asserts out (`-DNDEBUG`). `NDEBUG`-guarded
  death tests cover the abort. The boundary validation that keeps non-finite values from ever
  reaching here is `io`'s responsibility (see Deviations / forward notes).

### AC-7 — Tooling clean
- `clang-format --dry-run --Werror` passes on the new header, source, and test.
- `ci` (GCC `-Werror` + clang-tidy) and `ci-clang` build with 0 warnings in our code.

### AC-8 — Tested
- `tests/test_motion.cpp` is registered via `radar_add_test` and passes under `release`,
  `ci`, and `ci-clang`; the death tests additionally run under `asan`/`debug`.

## Deviations from the spec (recorded for the M2 implementer)

The spec's module table lists `motion | truth propagation (CV, CT) | M1`, but the M1
checklist only requires the CV model, and the spec's own `test_motion.cpp` contract lists a
CT check and a square-flagging check. This phase deviates deliberately in three places:

1. **CT deferred to M2.** The spec names CT as part of `motion`'s M1 scope, but the M1
   milestone gate only exercises straight-line (CV) targets. CT is specified and tracked
   above ("Deferred: CT") with completion criteria; its spec test ("CT at 3°/s returns to its
   heading after 120 s") is deferred with it. The spec is internally inconsistent here (table
   vs milestone checklist); we follow the milestone checklist.
2. **Surveillance-square entry/exit flagging moved out of `motion`.** The spec places
   "targets outside the square are flagged" in `test_motion`, but whether a target is inside
   the surveillance square is a *sensor/simulator* concern (it governs whether a detection is
   emitted), not a property of truth propagation. We assign that responsibility to
   `sensor`/`sim` (M1 sensor / M2), not `motion`. This is a genuine change of ownership, flagged
   here so it is not lost.
3. **`kalman → motion` dependency** for the shared CV `F`, which the spec's module table does
   not list — recorded and justified in **ADR-0006**, with its common-mode-`F` tradeoff and
   the independent-check mitigation (literal spec matrix here; hand-computed `F·x` in
   `test_kalman`).

**Forward note for `io` (M1).** `io` config parsing must reject non-finite or non-positive
`dt_s` and non-finite target states at the scenario boundary, so a malformed or hostile
scenario fails loudly instead of silently producing a NaN truth trajectory (which would
poison RMSE downstream). Record this as an acceptance criterion in the `io` planning docs.
