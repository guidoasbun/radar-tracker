# M1 · `types` module — Tasks

> **Status: ✅ complete — merged (PR #3).**

Ordered checklist; doubles as the execution record. Check off as completed.

## T1 — Header
- [x] Write `include/radar/types.hpp`: `Measurement`, `LabeledMeasurement`, `Scan`,
      `TrackStatus`, `TrackState`, and the `kPx/kPy/kVx/kVy` + `kClutterOrigin` constants.
- [x] Self-contained: include `<Eigen/Core>`, `<vector>`, `<cstdint>`, `<concepts>`,
      `<limits>`, `<type_traits>`; include-guarded; `radar` namespace. `TrackStatus` given a
      `std::uint8_t` base with explicit pinned values.
- [x] Origin-isolation `static_assert`s live in the header (hold with tests OFF).

## T2 — Build wiring
- [x] Header-only — no `src/types.cpp`. Reachable via `libradar`'s existing `include/`
      include path (no CMake change needed for the header itself).
- [x] Registered the test target in `tests/CMakeLists.txt` via `radar_add_test(test_types ...)`.

## T3 — Tests
- [x] `tests/test_types.cpp`: construction/defaults (incl. NaN `P`), pinned `TrackStatus`
      values, index-constant round-trip, `LabeledMeasurement` handoff, value-semantics
      traits, and a belt-and-suspenders structured-binding member-count guard.

## T4 — Verify (maps to requirements ACs) — ALL PASS
- [x] AC-1/7: `release` build → `ctest` → **14/14 pass** (9 new `Types.*` + 5 prior
      [2 smoke + 3 linalg]).
- [x] AC-1/6: `ci` build (GCC `-Werror` + clang-tidy) → **0 warnings in our code**, 14/14 pass.
- [x] AC-2 (negative tests): compiled Measurement mutations against the header guards;
      each fails the build (then reverted). Cases: `int origin` after `position` (sizeof);
      `int origin` before `position` (sizeof); `int target_id` added (sizeof); `bool
      is_clutter` added (sizeof); `position → Vector3d` (is_same); `position → Vector2f +
      float origin` of equal total size (is_same). sizeof + is_same are the real cover;
      `detail::HasOrigin` and the structured-binding test are extra.
- [x] AC-6: `clang-format --dry-run --Werror` on `types.hpp` + `test_types.cpp` → clean.
- [x] AC-4: `ci-clang` (Clang 21) build → **0 warnings in our code**, 14/14 pass.

## PR #3 review follow-up
Addressed a review on the initial M1 types PR:
- **B1** branch rebased `--onto origin/main` (post-squash) so the PR shows only the 6 new
  files, not the 34 pre-squash Phase-0 files. **B2** commit message fixed (was a literal
  `git commit -m "…` subject). **B3** CI: no `.github/` workflow yet (scheduled for the end
  of M1 per the overview); every "Verified" claim is a local `ci`/`ci-clang` run — called out
  in the PR body.
- **S1/S2** origin guards moved into the header and strengthened to check member *types*,
  `sizeof`, and a `HasOrigin` concept — the four leak variants the review compiled
  (origin at end / in the middle / `Vector3d` widening / time replaced by a struct) now all
  fail to build.
- **S3** dropped `Measurement::time_s` (duplicated `Scan::time_s`, not on the M3 wire,
  potential side channel). **S4** `TrackState::P` now defaults to NaN. **S5** `TrackStatus`
  values pinned `0..3` with `static_assert`.
- **S6/S7** recorded as forward notes (io/config id validation; net/codec non-finite
  rejection) in `design.md §5`.
- **D1–D6** doc/code mismatches fixed (test counts, enum snippet, trivially-copyable claim,
  implemented-guard description, AC-4 starting-status wording, copy-ctor comment).
- Nit: `<Eigen/Core>` instead of `<Eigen/Dense>`.

## Exit criteria
All ACs in `requirements.md` demonstrated by T4; `types.hpp` + `test_types.cpp`
committed. Next module: `motion` (CV truth propagation).
