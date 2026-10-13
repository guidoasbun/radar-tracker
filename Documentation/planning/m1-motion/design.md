# M1 · `motion` module — Design

## 1. Shape of the module

`motion` is `include/radar/motion.hpp` + `src/motion.cpp`, added to `libradar`. It depends
only on `types` (for the state-vector index layout `kPx…kVy`) and Eigen. It has no RNG and
no state of its own — the functions are pure.

Two functions in M1, both CV:

```cpp
namespace radar {

// The CV state-transition matrix F for a step of dt seconds, over x = [px, py, vx, vy].
[[nodiscard]] Eigen::Matrix4d cv_transition_matrix(double dt) noexcept;

// Advance a state vector one CV step: p += v*dt, v unchanged. Equal to F*x for
// finite inputs. Precondition: dt and x finite (asserted in debug/ASan).
[[nodiscard]] Eigen::Vector4d propagate_cv(const Eigen::Vector4d& x, double dt) noexcept;

}  // namespace radar
```

## 2. Why these two functions, and this split of responsibility

- **`cv_transition_matrix(dt)` is the single source of truth for the CV `F`.** The simulator
  uses it to advance targets; `kalman`'s predict step uses the *same* `F` for
  `x̂ = F·x̂` and `P = F·P·Fᵀ + Q`. Defining `F` once in `motion` means the truth model and
  the filter's model can never silently diverge. This adds a `kalman → motion` dependency the
  spec's module table does not list; that deviation is recorded and justified in **ADR-0006**
  (and its common-mode tradeoff is in §6). "Single source of truth" is a **CV** claim only:
  CT cannot be expressed with this `F`, and once maneuvers land the filter stays CV while
  truth may be CT during a maneuver — that model mismatch is the *point* of the maneuver
  scenario, not a drift to "fix" by making the filter use CT truth.

- **`propagate_cv` operates on a plain `Eigen::Vector4d`, not a `TrackState`.** Truth
  propagation is exact — a true target has a state, not a covariance. Keeping the signature
  on the state vector:
  - keeps `motion` free of any covariance/`P` handling, so it cannot be confused with the
    filter's predict step;
  - makes the "truth has no uncertainty" invariant explicit in the type;
  - leaves `P` propagation (`F·P·Fᵀ + Q`) entirely to `kalman`, which is the only place
    that knows the process noise `Q`.

  The simulator holds each target's true state as a `Vector4d` (or constructs a `TrackState`
  with `x` set and `P` irrelevant); it calls `propagate_cv(target.x, dt)` each scan.

- **Process noise `Q` and measurement noise `R` are NOT here.** They are filter/sensor
  parameters, derived from `σ_a` and `σ_m`, and belong to `kalman`/`sensor`. `motion` is
  deterministic truth only. (See requirements AC-5.)

## 3. The CV model

From the spec: outside a maneuver a target flies constant velocity,
`p_{k+1} = p_k + v_k·Δt`, with velocity unchanged. In matrix form over
`x = [p_x, p_y, v_x, v_y]`:

```
      | 1 0 dt 0 |
F =   | 0 1 0 dt |      x_{k+1} = F · x_k
      | 0 0 1  0 |
      | 0 0 0  1 |
```

`propagate_cv` could be written either as the explicit `p += v·dt` or as `F·x`. We implement
it as `F·x` using `cv_transition_matrix(dt)` so the two APIs are provably consistent (a test
asserts they agree), and so there is exactly one place that encodes the CV law. The cost is a
4×4·4 multiply — negligible, and the matrix is a fixed-size Eigen type (no allocation).

`dt` is seconds as `double` (the project's time convention). `cv_transition_matrix(0)` is the
identity, so a zero step is a no-op; this falls out of the formula with no special-casing.

**Finite-input precondition.** Because `propagate_cv` forms the full product `F·x`, the zero
entries of `F` still multiply `x`, and `0 * NaN == NaN` / `0 * inf == NaN`. So a single
non-finite component of `x`, or a non-finite `dt`, spreads to components the explicit
`p += v·dt` would leave untouched (and a non-finite `dt` corrupts even a stationary target).
`propagate_cv` therefore has a precondition — `dt` and all of `x` finite — asserted in
debug/ASan builds (`-DNDEBUG` compiles the asserts out of `release`/`ci`). This is a caller
contract, not runtime validation: `io` must reject non-finite or non-positive `dt_s` and
non-finite target states at the scenario boundary so a malformed scenario fails loudly rather
than silently producing a NaN truth trajectory (and NaN-poisoned RMSE downstream). This
mirrors the P-finiteness forward note in `types.hpp`.

`dt < 0` is supported: it propagates backward in time, and a forward step followed by a
step of `-dt` returns to the start (tested). `dt == 0` is a no-op for finite `x`.

## 4. CT (coordinated turn) — deferred, documented

CT is **not** implemented in M1. It is specified and tracked (see `requirements.md`
"Deferred: CT") so it is completed before maneuver scenarios run (M2). The planned shape,
chosen now so the M1 interface is forward-compatible:

```cpp
// DEFERRED (M2): coordinated turn at turn rate omega (rad/s) for a step of dt.
//   v_{k+1} = Rot(omega*dt) * v_k ;  p_{k+1} = p_k + v_{k+1}*dt   (Euler step, new v)
//   Rot(theta) = [[cos, -sin], [sin, cos]]
// Must reduce exactly to propagate_cv when omega == 0.
// [[nodiscard]] Eigen::Vector4d ct_propagate(const Eigen::Vector4d& x, double dt,
//                                            double omega) noexcept;
```

Two correctness points to remember when it lands:
1. The position step uses the **post-rotation** velocity `v_{k+1}`, not `v_k`.
2. `omega == 0` must be **bit-for-bit** equal to `propagate_cv` (not merely within a
   tolerance): `cos(0) == 1.0` and `sin(0) == 0.0` exactly, so the rotation is exactly the
   identity. The future CT continuity test should therefore demand `==`, a stronger and free
   check.

We do **not** add a dead declaration or stub now (it would be untested surface and a
clang-tidy/`-Werror` liability); the design note above is the forward contract.

## 5. Verification approach

`tests/test_motion.cpp`:
1. **`F` structure (independent)** — `cv_transition_matrix(dt)` equals a **literal 4×4
   written out from the spec** via `<<` (not the implementation's Identity-plus-two-entries
   recipe, so the test checks `F` rather than repeating it); `cv_transition_matrix(0)` is
   identity.
2. **CV law** — `propagate_cv`: position advances by `v·dt`, velocity unchanged; a
   zero-velocity target does not move (exact: `(out.array() == in.array()).all()`).
3. **Matrix/function consistency** — `propagate_cv(x, dt)` equals `cv_transition_matrix(dt)·x`.
4. **Degenerate** — `dt == 0` is a no-op (exact equality).
5. **Multi-step closed form + path length** — `N` steps of `dt` equal one step of `N·dt`;
   check a straight-line target against analytic `p0 + v·(N·dt)`, and the spec's "path length
   = speed × time" directly as `(p_N − p_0).norm() == ‖v‖·t`.
6. **Negative `dt` round-trip** — step by `dt` then `−dt` returns to the start (documents that
   backward propagation is supported).
7. **Determinism** — same input twice gives **bit-for-bit** identical output (`==`, not
   `isApprox`).
8. **Finite-input precondition** — `NDEBUG`-guarded death tests (`MotionDeathTest`) assert
   that a non-finite `dt` or `x` aborts; they run under `debug`/`asan` where asserts are live
   and are compiled out of `release`/`ci`.
9. **No `TrackState` overload (compile-time)** — a `CvPropagatable` concept with a positive
   control (`CvPropagatable<Eigen::Vector4d>`) and a negative guard
   (`!CvPropagatable<TrackState>`), pinning the "state vector only, P is kalman's" contract.
   A concept (not `is_invocable_v<decltype(propagate_cv), …>`) is used because `decltype` of
   an overloaded function is ill-formed and would make a future overload a confusing compile
   error rather than a clear assertion.

## 6. Risks / watch items

| Risk | Mitigation |
| --- | --- |
| `F` here drifts from `kalman`'s predict `F` | `F` is defined only here; `kalman` consumes `cv_transition_matrix`. A test asserts `propagate_cv == F*x`. |
| **Common-mode `F` bug (truth and filter share `F`)** | Sharing `F` means a bug in it corrupts truth *and* filter identically, so Phase K NEES/NIS consistency checks can't catch it (simulated truth follows the same wrong law). Mitigation: verify `F` **independently** — `test_motion` compares against a literal spec matrix (not `cv_transition_matrix`), and `test_kalman` must check `predict == F·x` with **hand-computed** values, not another `cv_transition_matrix` call. Accepted in **ADR-0006**. |
| Non-finite `dt`/`x` silently poisons the trajectory | `F·x` spreads NaN/inf to all components; `propagate_cv` asserts finite inputs (debug/ASan), and `io` must reject non-finite/non-positive `dt_s` and non-finite states at the boundary (requirements "Deviations / forward notes"). |
| CT forgotten | Deferred work captured in `requirements.md` + §4 with completion criteria; `omega==0`-reduces-to-CV (exact `==`) is the pinned acceptance test. |
| Someone adds `Q` or noise here | AC-5 and §2 state process noise is `kalman`'s; `motion` stays deterministic. A compile-time `static_assert` forbids a `TrackState` overload. |
| Euler CT position-step subtlety (uses new v) | Documented explicitly in §4 so the future implementer gets it right. |
