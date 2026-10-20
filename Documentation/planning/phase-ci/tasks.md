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
- [x] Branch `ci-workflow`, commit, push, open PR #7.
- [x] Confirmed the workflow runs on PR #7 and all four checks are green (run 39470536608:
      gcc/clang/asan pass, asan ran both MotionDeathTest cases; gitleaks real scan, no leaks).

## T5 — PR #7 security-review fixes
- [x] **H2/M2/M1-action/L4** — replaced `gitleaks-action@v2` with the pinned,
      checksum-verified gitleaks 8.30.1 binary (`sha256sum -c`, same as the local hook);
      event-range scan on push/PR, `--log-opts=--all` on `schedule`+`workflow_dispatch`.
- [x] **M1** — `actions/checkout` + `actions/cache` pinned to SHAs (verified vs upstream v4);
      added `.github/dependabot.yml` (github-actions, weekly).
- [x] **M3** — install clang/clang-tidy **21** from apt.llvm.org via `update-alternatives` so
      CI's linter matches local (presets unchanged).
- [x] **M4** — `cancel-in-progress` only on `pull_request`.
- [x] **L1** — `persist-credentials: false` on both checkouts.
- [x] **L2** — cache only `_deps/*-src`; corrected the cache comment.
- [x] **L3** — `timeout-minutes` (30 build / 10 gitleaks).
- [x] **L5** — doc fixes: ADR-0005 status, design §1 job IDs / §2 scan range + H1 note / §3
      CMake + clang-21, requirements AC-4, README table.
- [x] **H1** — required the four checks in branch protection via `gh api` (strict + enforce_admins
      on; verified). The scan is now genuinely unbypassable. Caveat documented: checks match by
      display name, so keep matrix `name:` values and the branch-protection list in sync.

## Exit criteria
All ACs met and the PR #7 review addressed: the matrix + checksum-verified gitleaks run on
push/PR (full history on schedule), green on the PR, docs accurate. The scan is *unbypassable*
once H1's required checks are enabled in branch protection.
