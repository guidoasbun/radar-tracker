# Phase CI · GitHub Actions workflow — Tasks

## T1 — Planning docs
- [x] `requirements.md`, `design.md`, `tasks.md` under `planning/phase-ci/`.

## T2 — Workflow
- [x] `.github/workflows/ci.yml`: triggers (push `main` + PR), `permissions: contents: read`.
- [x] `build` matrix job — installs ninja + clang + clang-tidy; runs `cmake --preset {ci,
      ci-clang,asan}` → build → `ctest` (fail-fast:false so all three report).
- [x] `asan` entry runs `ctest` under Clang Debug, so asserts + death tests execute.
- [x] `gitleaks` job — full-history (`fetch-depth: 0`) scan with `.gitleaks.toml`, pinned
      `gitleaks/gitleaks-action@v2`.
- [x] FetchContent `_deps` caching (`actions/cache@v4`, keyed on `Dependencies.cmake`);
      actions pinned (`@v4`, `@v2`).

## T3 — Validate + docs
- [x] YAML valid (parsed); the three matrix presets all exist in `CMakePresets.json`.
- [x] Local parity: `ci` 24/24, `ci-clang` 24/24, `asan` **26/26** (incl. 2 death tests) —
      the same commands the jobs run, all green.
- [x] `README.md` — gitleaks-CI promise now "runs on every push/PR"; new "Continuous
      integration" section documents the matrix + unbypassable gitleaks.
- [x] `00-overview.md` — step 10 marked done / pulled forward with rationale.

## T4 — Ship
- [ ] Branch `ci-workflow`, commit, push `-u`, open PR.
- [ ] Confirm the workflow actually runs on the PR and is green (the real verification).

## Exit criteria
All ACs in `requirements.md` met: the workflow runs the GCC/Clang/asan matrix + the
unbypassable gitleaks job on push/PR, is green on its own PR, and the docs reflect that CI
is now enforced.
