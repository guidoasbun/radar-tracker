# Phase CI · GitHub Actions workflow — Tasks

> **Status: ✅ complete — merged (PR #7).**

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
- [x] **M3** — install clang/clang-tidy **21** from apt.llvm.org and point `/usr/bin/clang*`
      at them with `ln -sf` + a version **assertion** (see T6: `update-alternatives` silently
      failed). Presets unchanged.
- [x] **M4** — `cancel-in-progress` only on `pull_request`.
- [x] **L1** — `persist-credentials: false` on both checkouts.
- [x] **L2** — cache `_deps/*-src` **and** `_deps/*-subbuild` (corrected in T6; `*-src` alone
      was never reused); corrected the cache comment.
- [x] **L3** — `timeout-minutes` (30 build / 10 gitleaks).
- [x] **L5** — doc fixes: ADR-0005 status, design §1 job IDs / §2 scan range + H1 note / §3
      CMake + clang-21, requirements AC-4, README table.
- [x] **H1** — required the four checks in branch protection via `gh api` (strict + enforce_admins
      on; verified). The scan is now genuinely unbypassable. Caveat documented: checks match by
      display name, so keep matrix `name:` values and the branch-protection list in sync.

## T6 — follow-up review fixes (@ `037b64d`)
- [x] **M3 blocker** — the first M3 fix didn't take: `update-alternatives --install` doesn't
      override the runner's existing plain `/usr/bin/clang` symlink (warns, exits 0), so CI
      kept **clang 18** while docs claimed 21 (confirmed in the run log). Replaced with
      `ln -sf` and added version assertions (`clang --version | grep -q 'clang version 21.'`,
      same for clang-tidy) so a silent fallback now fails the job. Added `libclang-rt-21-dev`
      for the asan link.
- [x] **Cache never reused** — FetchContent's stamp files live in `_deps/*-subbuild`; caching
      `*-src` alone meant a hit still `rm -rf`+re-cloned. Now caches both `*-src` and
      `*-subbuild`, with the preset back in the key; comment corrected.
- [x] **Scan wording** — `gitleaks git` (no `--log-opts`) scans all commits reachable from
      HEAD, not "new commits only"; corrected the step echo, comment, ADR-0005 and design §2.
- [x] **LLVM apt key pinned** — fetched to its own keyring, fingerprint-checked against
      `6084F3CF…F7421`, referenced via `signed-by=` (not trusted repo-wide in `trusted.gpg.d`).
- [x] **ADR-0005 stale caveat** — "until then a red PR can still be merged" updated (required
      checks are on).
- [x] **Re-verify on CI** — the `clang (ci-clang preset)` job has run green on every PR since
      (PR #7 onward through PR #12), building with Clang + clang-tidy under `-Werror`; any
      clang-tidy findings on our code were fixed before the required checks went green.

## Exit criteria
All ACs met and the PR #7 review addressed: the matrix + checksum-verified gitleaks run on
push/PR (full history on schedule), green on the PR, docs accurate. The scan is *unbypassable*
once H1's required checks are enabled in branch protection.
