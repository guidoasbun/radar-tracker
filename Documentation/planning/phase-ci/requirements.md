# Phase CI · GitHub Actions workflow — Requirements

## Purpose

Add `.github/workflows/ci.yml` so every push and pull request to `main` is built and
tested automatically, instead of relying on the author's local runs. This is step 10 in
the M1 execution order, **pulled forward** now (after `types` and `motion`) because:

- Two PR reviews (PR #4 L5, N2) flagged that every "Verified" claim rests on the author's
  machine, with nothing stopping a regression from merging before `ci.yml` lands.
- The module count is growing; the cost of a silent regression rises with each module.
- The `motion` finite-input asserts and death tests (H1) only run where asserts are live
  (`debug`/`asan`), and the `ci`/`ci-clang` presets inherit `-DNDEBUG` from `release`.
  Without a sanitizer/debug CI job, that protection is checked **nowhere** automatically
  (N2). This workflow closes that gap.

The README already promises (lines 73–74) that a **gitleaks CI job "which cannot be
bypassed" will be added with the M1 CI workflow**; this phase delivers it.

## Scope

In scope:
- A workflow triggered on `push` to `main` and on `pull_request` targeting `main`.
- A build+test matrix that mirrors the local gate:
  - `ci` preset — GCC, `-Werror` + clang-tidy (the strict gate).
  - `ci-clang` preset — Clang, same strict set.
  - `asan` preset — Clang ASan/UBSan, **runs `ctest`** so the asserts and death tests
    actually execute (the N2 fix). The spec mandates ASan/UBSan "on every push."
- A **gitleaks** job that scans the event's commit range on every push/PR, plus a weekly
  `schedule` + `workflow_dispatch` that scan **full history** (`--log-opts=--all`). It uses the
  same pinned, checksum-verified gitleaks binary as the local hook — independent of the local
  pre-commit hook and not skippable by contributors.
- FetchContent dependency caching so runs are reasonably fast after the first.

Out of scope / deferred:
- Coverage (lcov) — M2 (toolchain item not yet installed).
- Valgrind memcheck nightly — later.
- The plotting / Python job — arrives with the M1 plotting step.
- Release/packaging workflows — not part of v1.

## Acceptance criteria

### AC-1 — Triggers
The workflow runs on push to `main` and on every pull request targeting `main`.

### AC-2 — Strict build matrix mirrors local
Separate jobs build and test `ci` (GCC) and `ci-clang` (Clang); both must be
0-warnings-in-our-code (the presets already carry `-Werror` + clang-tidy) and
`ctest` must report 100% passed.

### AC-3 — Sanitizer job exercises the asserts (N2)
An `asan` job (Clang, per ADR-0004) builds and runs `ctest` under ASan/UBSan. Because the
`asan` preset is a Debug build (no `-DNDEBUG`), the `motion` finite-input asserts are live
and the `MotionDeathTest` cases execute. Any sanitizer finding or failed death test fails
the job.

### AC-4 — Checksum-verified secret scan, unbypassable once required
A gitleaks job runs the **same** pinned, checksum-verified gitleaks 8.30.1 binary as the local
hook (`sha256sum -c`, ADR-0005) with the tracked `.gitleaks.toml`. On push/PR it scans the
event's commit range; a weekly `schedule` and `workflow_dispatch` scan **full history**
(`--log-opts=--all`). It is server-side, so unlike the local hook it cannot be skipped with
`--no-verify`. A finding fails the job. **Caveat (H1):** the job only *blocks* a merge once it
is set as a required status check in branch protection — tracked separately, since that is a
repo setting, not workflow code.

### AC-5 — Valid and self-consistent
- The workflow is valid YAML and references only presets that exist in
  `CMakePresets.json` (`ci`, `ci-clang`, `asan`).
- Action versions are pinned (not floating `@master`).
- Runner is `ubuntu-24.04` (per ADR-0002: CI is the final arbiter; its GCC/Clang may
  differ from local, which is acceptable and intended).

### AC-6 — Docs updated
- `README.md` reflects that CI now runs the matrix + gitleaks on every push/PR (the
  "will be added" promise becomes "is enforced").
- `00-overview.md` step 10 is marked done / pulled forward.

## Notes on verification

GitHub Actions cannot be executed locally, so pre-merge verification is: (1) the YAML is
valid and references real presets; (2) the same `cmake --preset …` commands the workflow
runs are green locally; (3) the workflow then runs on its own PR, which is the real proof.
