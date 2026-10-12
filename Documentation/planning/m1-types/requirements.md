# M1 · `types` module — Requirements

## Purpose

Define the plain data structures shared across every module: measurements, scans,
and track state. The `types` module owns *data*, never *logic* (spec module table:
"plain structs shared everywhere; no logic"). It is the first M1 module because
`motion`, `sensor`, `kalman`, and `metrics` all depend on it.

The single most important property is **origin-tag isolation**: the `Measurement`
type handed to the tracker carries no information about whether it came from a target
or from clutter. Per the spec (sensor model, step 4): *"The `Measurement` type handed
to the tracker has no such field, so leakage is prevented by the type system, not by
discipline."* The hidden origin tag lives on a separate type used only by the
simulator (to record truth) and the scorer (to pair tracks with truth).

## Scope

In scope:
- `Measurement` — a noisy 2D position `(x, y)` in meters only; the scan time lives on
  `Scan::time_s`, not per measurement. No origin, no target id.
- `Scan` — one sensor sweep: a scan index, its timestamp, and the (already shuffled)
  list of `Measurement`s.
- `TrackStatus` — the track lifecycle enum (tentative / confirmed / coasting / deleted).
- `TrackState` — a filter estimate: state vector `[p_x, p_y, v_x, v_y]` and its
  4×4 covariance `P`.
- `LabeledMeasurement` — a `Measurement` plus the hidden origin tag (target id, or
  `-1` for clutter). Used only by `sensor`/`metrics`, never passed to the tracker.
- Named constants for the state-vector layout (indices for px, py, vx, vy) and the
  clutter sentinel origin value.

Out of scope (later modules own these):
- Any propagation, noise, filtering, gating, association, or scoring logic.
- Hit/miss counters and M-of-N bookkeeping — those live on `Track` (`track.hpp`, M2),
  which composes a `TrackState`. `types` only provides the estimate payload.
- Serialization / JSON / CSV mapping (`io`, M1 later).

## Acceptance criteria

### AC-1 — Builds everywhere
`include/radar/types.hpp` compiles cleanly as part of `libradar` and the test suite
under all presets, including `ci`/`ci-clang` with the full `-Werror` strict warning
set. The header is self-contained (includes what it uses; include-guarded).

### AC-2 — Origin isolation is enforced by the type system
- `Measurement` has no member that reveals origin (no target id, no `is_clutter`,
  no `origin`) and no per-detection time. This is asserted **at compile time in the
  header** (`static_assert`s on member *types*, `sizeof`, and a `!HasOrigin<Measurement>`
  concept check), so the guarantee holds in every build — including one with tests
  disabled — and a future edit that adds, moves, or widens a field to smuggle origin
  breaks the build. A test-only structured-binding guard adds a member-count check.
- `LabeledMeasurement` carries the origin tag (`HasOrigin<LabeledMeasurement>` asserted)
  and composes (not inherits leak-ably into) a plain `Measurement`, so the tracker-facing
  value can always be extracted without the tag.

### AC-3 — Correct units and layout
- `Measurement` position is meters in the East–North frame; the scan time is carried
  once on `Scan::time_s`, not per measurement. `TrackState` is
  `[p_x, p_y, v_x, v_y]` (meters, meters/second). The index constants
  (`kPx, kPy, kVx, kVy`) match this order and are covered by a test.
- Covariance `P` is `Eigen::Matrix4d`, matching the Kalman spec (store P as
  `Eigen::Matrix4d`).

### AC-4 — Sensible, explicit defaults
- Structs are aggregates with value-initialized defaults: zeroed `position` and state
  `x`; `P` defaults to **NaN** so an uninitialised covariance fails loudly rather than
  masquerading as a valid identity (see design S4); `kClutterOrigin` for `origin`.
  Defaults are covered by a construction test.
- `TrackStatus` enumerators have **explicit, pinned values** (`Tentative=0 … Deleted=3`)
  because they are serialized on the M3 wire and in CSV; the test checks the values. Note
  the starting status of a *track* is `Tentative`, but that lives on `Track` (M2); no type
  in this module has a `TrackStatus` member, so this AC only pins the enum values here.
- The clutter sentinel (`kClutterOrigin == -1`) matches the spec.

### AC-5 — No logic, no heap surprises
- The module introduces no functions with behavior beyond trivial constructors.
- `Measurement`/`TrackState` use Eigen fixed-size types (no heap allocation), per the
  spec's "fixed-size matrices allocate nothing on the heap" rationale.

### AC-6 — Tooling clean
- `clang-format --dry-run --Werror` passes on the new header and test.
- `clang-tidy` (via the `ci` per-target run) reports no warnings in the new code.

### AC-7 — Tested
- `tests/test_types.cpp` is registered with CTest via `radar_add_test` and passes
  under `release` and `ci`.
