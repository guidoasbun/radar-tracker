# Radar Tracker — Planning Overview

This document is the map. It lists every milestone and phase, the order we execute
them in, and the documentation convention each phase follows. The authoritative
functional spec is `../Radar Tracker — Project Specification.md`; this folder tracks
*how* we build it, decision by decision.

## Documentation convention

Every phase gets its own folder under `planning/` with three artifacts:

| File | Purpose |
| --- | --- |
| `requirements.md` | What must be true when the phase is done — testable acceptance criteria. The "definition of done." |
| `design.md` | How we build it — structure, key decisions, and rationale. Light for small phases, detailed for algorithm-heavy ones. |
| `tasks.md` | Ordered checklist, checked off as we execute. Doubles as the record of what was done. |

Cross-cutting decisions that outlive a single phase are recorded as short
Architecture Decision Records (ADRs) under `../decisions/`.

Git history is the authoritative changelog; the spec and these docs capture intent
and rationale that commits alone do not.

## Phase and milestone map

The spec defines four milestones (M1–M4). We insert two extra phases that the spec
implies but does not name as discrete steps: **Phase 0** (scaffold/toolchain) before
M1, and **Phase K** (exhaustive Kalman verification) inside M1 after the `kalman`
module exists. Phase K is a deliberate, heavy-investment verification gate because
the filter is the keystone the entire project depends on.

| Phase | Name | Spec milestone | Gate (one-line definition of done) | Status |
| --- | --- | --- | --- | --- |
| 0 | Scaffold & toolchain | pre-M1 | Clean clone configures + builds + a trivial test passes under all presets | NEXT |
| M1 | Single target | M1 (v0.1) | `single.json` runs; track RMSE ≤ 0.5 × raw RMSE; plot in README; CI green | planned |
| K | Kalman verification | within M1 | Full six-group filter verification battery passes (see phase-K docs) | planned |
| M2 | Multi-target + clutter | M2 (v0.2) | baseline clears all thresholds; crossing keeps identities; dense_clutter ≤ 1 false track | planned |
| M3 | Two services over UDP | M3 (v0.3) | `docker compose up` leaves metrics.json; UDP tracks match in-process on same seed | planned |
| M4 | Live display (optional) | M4 (v0.4) | 60 fps with 10 tracks + λ=20; demo GIF in README | planned |

## Execution ordering within M1 (with Phase K)

Phase K's statistical/Monte-Carlo tests need the CV motion model and a Gaussian
sensor to generate data, so the M1 module order is:

1. `types` — shared structs
2. `motion` (CV) — truth propagation
3. `sensor` (noise only; P_D=1, no clutter) — enough to feed Monte-Carlo tests
4. `kalman` — predict, Joseph-form update, innovation, S, gate distance
5. **Phase K** — exhaustive filter verification battery (gate before building upward)
6. `io` (config + csv) — load scenario, write CSVs
7. `metrics` (RMSE only) — raw vs track error
8. `apps/radar_sim` — wire it together, run end-to-end, print RMSE
9. `tools/plot.py` + README figure
10. `.github/workflows/ci.yml` — build + test on push

## Working principles (carried from the spec)

- All logic lives in `libradar`; `apps/*` only parse args, load config, and wire modules.
- Dependency arrows point one way: nothing from `tracker` downward includes `io` or `net`.
- Every phase ends in something runnable or a passing test — no "big bang" integration.
- Thin vertical slices: one module → one test → one commit. Small blast radius.
- Spend the "careful" budget where the risk is: Phase K (filter), M2 (tuning + association), M3 (threading).

## Toolchain baseline (verified 2026-10-11, RHEL 10.2)

| Tool | Version present | Spec minimum | Notes |
| --- | --- | --- | --- |
| GCC (g++) | 14.3.1 | GCC 12 | primary compiler |
| Clang/Clang++ | 21.1.8 | Clang 15 | much newer; see ADR-0002 on `-Werror` |
| CMake | 3.31.8 | 3.25 | presets + SYSTEM keyword on FetchContent_Declare |
| Ninja | 1.11.1 | — | chosen generator; see ADR-0001 |
| clang-tidy / clang-format | 21.1.8 | — | available locally (M2) |
| Valgrind | 3.26.0 | — | nightly memcheck |
| gcov | 14.3.1 | — | coverage; lcov wrapper still needed (M2) |
| Python | 3.12.15 | 3.10 | matplotlib/pandas + pip not yet installed (needed end of M1) |
| Docker | not installed | — | needed at M3 only |

Open toolchain items, due before the phase that needs them:
- `pip3` + `matplotlib` + `pandas` — before the M1 plotting step.
- `lcov` — before the M2 coverage job.
- Docker + Compose v2 — before M3.
