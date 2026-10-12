# ADR-0006 — `kalman` reuses `motion`'s CV transition matrix (`kalman → motion`)

- **Status:** Accepted
- **Date:** 2026-10-12
- **Deciders:** project author; raised in PR #4 review (M1 motion)
- **Phase:** M1 (motion / kalman)

## Context

The spec's module-responsibility table lists dependencies as:

| Module | Depends on |
| --- | --- |
| motion | types |
| kalman | types |

i.e. `motion` and `kalman` are both meant to depend only on `types`, with no edge
between them.

The constant-velocity (CV) state-transition matrix `F` appears in two places:
- **truth propagation** in `motion` (the simulator advances targets by `x_{k+1} = F·x_k`), and
- the **Kalman predict** step in `kalman` (`x̂ = F·x̂`, `P = F·P·Fᵀ + Q`).

`F` is the same matrix in both. If each module defines its own `F`, the two can drift
apart under edits (e.g. a layout change, a `Δt` sign convention), and the drift would be
silent because each side would still be internally consistent.

The M1 `motion` module therefore exposes `cv_transition_matrix(dt)` and the M1 `kalman`
module is intended to call it, which introduces a `kalman → motion` dependency that the
spec's table does not list.

## Decision

Allow `kalman` to depend on `motion` for the CV transition matrix. `cv_transition_matrix`
in `motion` is the single definition of `F`; `kalman`'s predict step consumes it rather
than defining its own.

The one-way dependency rule from `00-overview.md` ("nothing from `tracker` downward
includes `io` or `net`") is **not** violated: `motion` is a pure, lower-level math module
with no I/O, and the new edge points *upward* from `kalman` to `motion`, not toward `io`/
`net`. The dependency graph remains acyclic.

## Consequences

- Positive: `F` is defined once; truth and filter cannot silently disagree on the CV law.
- Positive: `motion` stays free of covariance/process-noise concerns; `kalman` composes
  `F·P·Fᵀ + Q` itself.
- Negative / tradeoff — **common-mode `F` bug.** Because truth and filter share `F`, a bug
  in `F` affects both identically, so Phase K consistency checks (NEES/NIS) that compare
  the filter against simulated truth cannot catch it — the simulated truth would follow the
  same wrong law. This is accepted *on the condition* that `F` is verified by an
  **independent** check that does not call `cv_transition_matrix`:
  - `test_motion` compares `cv_transition_matrix` against a literal spec matrix (done), and
  - `test_kalman` must verify "predict equals `F·x` analytically" with hand-computed values,
    not another call to `cv_transition_matrix`.
  Recorded as a risk row in the motion `design.md §6`.
- Follow-up: when the coordinated-turn (CT) model lands, the filter stays CV while truth may
  be CT during maneuvers — an intentional model mismatch (that is the point of the maneuver
  scenarios), not a drift to fix. Noted in motion `design.md`.

## Alternatives considered

- **Keep `kalman` independent, duplicate `F`.** Rejected: two definitions of the same
  matrix invite silent drift, which is exactly what the shared definition prevents. The
  spec's table is a guideline for keeping the graph one-way and I/O-free, and a `kalman →
  motion` math edge does not break that intent.
- **Move `F` into `types`.** Rejected: `types` is "plain structs, no logic" (its own
  contract); a `dt`-parameterised matrix factory is behaviour, not a data type.
- **A new shared `linalg`/`model` module owning `F`.** Rejected as over-engineering for one
  4×4 matrix at M1; can revisit if more shared models appear.
