# Phase CI · GitHub Actions workflow — Design

## 1. Shape

One workflow file, `.github/workflows/ci.yml`, with four independent jobs that run in
parallel on `ubuntu-24.04`:

The workflow has two GitHub job IDs: `build` (a 3-way matrix) and `gitleaks`. The four
**check names** that appear on a PR (and that branch protection matches on) come from the
matrix `name:` values and the gitleaks job name:

| Check name (matrix `name:` / job) | Compiler | Preset | What it proves |
| --- | --- | --- | --- |
| `gcc (ci preset)` | GCC | `ci` | strict `-Werror` + clang-tidy build, tests pass (the gate) |
| `clang (ci-clang preset)` | Clang | `ci-clang` | same strict set under the other compiler |
| `asan (Clang, ASan/UBSan)` | Clang | `asan` | ASan/UBSan clean **and** asserts/death tests run (Debug, no `-DNDEBUG`) |
| `gitleaks (secret scan)` | — | — | checksum-verified secret scan |

The matrix entries run concurrently and `fail-fast: false` so one failing compiler still
reports the others. **Renaming any `name:` later changes the check name**, which will leave
PRs waiting on a now-missing required check until branch protection is updated to match
(the H1 required-checks caveat).

## 2. Key decisions

- **Mirror the presets, don't reinvent flags.** Each build job runs exactly
  `cmake --preset X && cmake --build --preset X && ctest --preset X`. CI must test the same
  thing developers run locally; duplicating compiler flags in YAML would let the two drift.
  This is the same single-source-of-truth principle used for the CV `F` matrix.

- **A sanitizer job that actually runs the tests (the N2 fix).** The `ci`/`ci-clang` presets
  inherit `-DNDEBUG` from `release`, so `assert(...)` is compiled out and the
  `MotionDeathTest` cases are `#ifndef NDEBUG`-skipped there. The `asan` preset is a Debug
  build, so running `ctest` under it is the only automated place the finite-input asserts
  and death tests execute. Without this job the H1 protection would be CI-invisible.

- **`asan` uses Clang (ADR-0004).** The build machine lacks GCC's sanitizer runtime; we
  standardize on Clang for parity with local. GitHub runners have both, but Clang keeps CI
  and local identical.

- **`ubuntu-24.04`, not the local RHEL versions (ADR-0002).** CI is the final arbiter of the
  matrix; its GCC/Clang may differ from the author's 14.3/21.1. That is intended — a second,
  independent toolchain is extra coverage. The `ci` preset's `-Werror` makes any
  version-specific warning a visible failure to fix, not silent drift.

- **gitleaks as a first-class job, checksum-verified, not a mutable action.** Earlier this
  used `gitleaks/gitleaks-action@v2`, which pulled an older default gitleaks (8.24.3) with no
  checksum step, required `GITHUB_TOKEN`, and is license-gated for organizations. We instead
  install the **same** pinned gitleaks 8.30.1 the local hook uses (`scripts/setup-hooks.sh`,
  ADR-0005), verifying the download with `sha256sum -c` against the identical pinned hash, and
  run `gitleaks git --redact --no-banner --config .gitleaks.toml`. This closes the version gap
  (CI now runs the same rules as local), the checksum gap, and the license dependency.
  - **Scan range (important).** On `push`/`pull_request` gitleaks scans only the event's
    commit range — a fast gate on new commits. Genuine **full-history** scanning (`--log-opts
    =--all`) runs on a weekly `schedule` and on `workflow_dispatch`. `fetch-depth: 0` makes the
    full history available for those runs. (This corrects the earlier claim that every run
    scanned full history.)
  - **"Unbypassable" has a precondition.** The job only *blocks* a merge once it is set as a
    **required status check** in branch protection; otherwise a red PR can still be merged.
    See the H1 note below. With `enforce_admins` already on, requiring the checks binds the
    owner too.

- **Dependency caching.** FetchContent downloads Eigen/GoogleTest/json/CLI11 on first
  configure. We cache only the fetched **source** trees (`build/<preset>/_deps/*-src`), not the
  per-preset build/subbuild dirs, keyed on `cmake/Dependencies.cmake`. On a pin change the key
  changes; `restore-keys` may seed from an older cache, after which FetchContent re-fetches the
  new SHA — so a stale cache is only slower, never wrong.

## H1 — required status checks (repo setting, now enabled)

Branch protection on `main` originally had **no** `required_status_checks` (verified via the
API), so despite `enforce_admins` and required PRs, a PR with a failing check could still be
merged — the "unbypassable" claim was not yet true. This is now fixed: the four checks
`gcc (ci preset)`, `clang (ci-clang preset)`, `asan (Clang, ASan/UBSan)` and
`gitleaks (secret scan)` are set as **required** (with `strict: true`, so the branch must be
up to date), and `enforce_admins` is on, so the gate binds the owner too. Because required
checks match by **display name**, renaming a matrix `name:` later leaves PRs stuck on
"Expected — waiting"; keep the names and the branch-protection list in sync. (Settings →
Branches → `main`, or `gh api …/branches/main/protection`.)

- **SHA-pinned actions and least privilege.** Third-party actions are pinned to full 40-char
  commit SHAs, not mutable `@vN` tags — a tag can be moved by the maintainer or an attacker who
  takes over the action repo (how the March 2025 `tj-actions/changed-files` compromise spread),
  so a tag is not a real pin. This matches ADR-0005's SHA-pinning of FetchContent dependencies.
  Each pin carries a trailing `# vN` comment for readability, and `.github/dependabot.yml`
  (github-actions ecosystem) opens PRs that bump the SHA and refresh the comment, so pins stay
  current. The SHAs were verified against the upstream release tags before pinning.
  `permissions: contents: read` at the workflow level; nothing needs write access.

## 3. Toolchain install on the runner

`ubuntu-24.04` ships GCC, Clang, and CMake, but not always Ninja or a recent-enough CMake
by default, and the `ci` presets require clang-tidy. The jobs install what they need:
- Ninja (`ninja-build`) for all build jobs.
- **Clang/clang-tidy 21** for every build job. The distro ships clang-tidy 18, which has
  fewer checks than the local 21 — so CI would silently enforce *less* than local (e.g.
  `readability-math-missing-parentheses`, added in tidy 19, caught a real issue in PR #4).
  We install `clang-21`/`clang-tidy-21` from the official LLVM apt repo (signed key) and point
  the unversioned `clang`/`clang++`/`clang-tidy` at them via `update-alternatives`, so the
  CMake presets need no changes and CI's linter matches local. (The GCC `ci` job also needs
  clang-tidy because the `ci` preset enables `RADAR_CLANG_TIDY`.)
- CMake ≥ 3.25 is required (for the `SYSTEM` FetchContent keyword); `ubuntu-24.04` ships 3.31,
  so no pin is needed.

## 4. Risks / watch items

| Risk | Mitigation |
| --- | --- |
| Runner GCC/Clang emit a warning local versions don't | That is the point of a second toolchain; `-Werror` surfaces it as a fixable failure (ADR-0002). |
| clang-tidy not on the runner → `ci` preset configure fails | Jobs `apt-get install clang-tidy`; pinned to the distro's version. |
| gitleaks action version drift | Pin the action to a tag. |
| First run slow (deps download) | `_deps` cache keyed on `Dependencies.cmake`. |
| ASan job flakes on a leak in a dependency, not our code | `asan` preset + `ASAN_OPTIONS` already tuned in `CMakePresets.json`; our code is allocation-free so findings would be real. |

## 5. Why pull this forward now (vs step 10)

The plan put `ci.yml` last, after the app runs end-to-end. But CI's value is proportional to
how much it guards, and it guards *future* commits — so the earlier it exists, the more it
protects. With randomness about to enter in `sensor` (where a seed/logic regression is easy
to miss) and the filter after that, locking the gate in now is cheap insurance. This is a
deviation from the documented ordering, recorded here and reflected in `00-overview.md`.
