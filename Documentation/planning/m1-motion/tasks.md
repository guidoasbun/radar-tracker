# M1 · `motion` module — Tasks

> **Status: ✅ complete — merged (PR #4).**

Ordered checklist; doubles as the execution record.

## T1 — Header + source
- [x] `include/radar/motion.hpp`: declare `cv_transition_matrix(double) -> Eigen::Matrix4d`
      and `propagate_cv(const Eigen::Vector4d&, double) -> Eigen::Vector4d`; guard
      `RADAR_MOTION_HPP`; `namespace radar`; include `<Eigen/Core>`.
- [x] `src/motion.cpp`: implement both; `propagate_cv` returns `cv_transition_matrix(dt) * x`.
- [x] Document the deferred CT contract in the header (comment only, no stub).

## T2 — Build wiring
- [x] Added `motion.cpp` to the `radar` library sources in `src/CMakeLists.txt`.
- [x] Registered `test_motion` in `tests/CMakeLists.txt` via `radar_add_test`.

## T3 — Tests
- [x] `tests/test_motion.cpp`: literal-spec-matrix `F` check (independent of impl) + identity
      at dt=0; CV law (p+=v*dt, v fixed); zero-velocity target static (exact); `propagate_cv
      == F*x`; dt=0 no-op (exact); multi-step vs closed form + direct path-length
      `(p_N−p_0).norm()==‖v‖·t`; negative-dt round-trip; determinism (exact `==`);
      `NDEBUG`-guarded non-finite death tests; compile-time no-`TrackState`-overload
      `static_assert`.

## T4 — Verify (maps to ACs) — ALL PASS
- [x] AC-1/2/3/8: `release` build → `ctest` passes.
- [x] AC-7: `ci` build (GCC `-Werror` + clang-tidy) → 0 warnings in our code.
- [x] AC-7: `clang-format --dry-run --Werror` clean.
- [x] AC-8: `ci-clang` (Clang 21) build → 0 warnings, tests pass.
- [x] AC-6/8: `asan` → death tests + all Motion.* pass.

## PR #4 review follow-up
Addressed the review on PR #4:
- **H1** finite-input precondition + `assert(isfinite(dt) && x.allFinite())`; header reworded
  "equal to `F·x` for finite inputs"; io boundary-validation recorded (Deviations / forward
  note). **H2** named indices (`f(kPx,kVx)=dt`), `motion.hpp` includes `types.hpp`, and the
  `F` test compares a literal spec matrix instead of repeating the impl recipe.
- **M1** ADR-0006 for the `kalman → motion` dependency. **M2** common-mode-`F` risk row +
  independent-check mitigation in design §6 (+ `test_kalman` must hand-compute `F·x`).
- **M3** requirements realigned to the `Vector4d` API (Scope, AC-2, Deferred CT); AC-5
  reworded as signature-enforced + compile-time `static_assert` that no `TrackState` overload
  exists. **M4** "Deviations from the spec" section (CT deferred; square-flagging ownership
  moved to sensor/sim; `kalman→motion`); direct path-length test added.
- **L1** determinism/no-op/zero-velocity use exact `==`. **L2** negative-dt documented +
  round-trip test + non-finite death tests. **L3** CT continuity reworded to bit-for-bit
  `==`. **L4** "single source of truth" scoped to CV + CT/CV intentional-mismatch note.
- **L5** (move `ci.yml` earlier) — noted for the author; a sequencing call, not changed here.

## Exit criteria
All ACs in `requirements.md` demonstrated by T4; `motion.{hpp,cpp}` + `test_motion.cpp`
committed. CT remains deferred with its contract recorded. Next module: `sensor`
(Gaussian noise only; P_D=1, no clutter).
