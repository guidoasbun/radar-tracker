# ADR-0005 — Dependency integrity (SHA pins) and secret-protection posture

- **Status:** Accepted
- **Date:** 2026-10-11
- **Deciders:** @Guido Asbun
- **Phase:** 0 (Scaffold & toolchain), prompted by PR review

## Context

The initial scaffold pinned FetchContent dependencies to Git **tags** (`3.4.0`,
`v1.14.0`, ...) and leaned on a broad `.gitignore` for "security-first" secret
protection. A review raised two valid points:

1. Git tags are mutable. If an upstream maintainer (or a compromised account)
   force-pushes a tag, every fresh clone and CI run silently builds different code,
   with nothing to detect it.
2. `.gitignore` is not a security control — it only prevents accidental staging. It
   does not stop an already-tracked secret, and broad substring globs (`*secret*`,
   `*token*`) silently drop legitimate source files instead.

## Decision

1. **Pin every dependency to its full 40-character commit SHA**, with the
   corresponding tag noted in a comment. A SHA cannot change content without changing
   identity, so builds are reproducible and tamper-evident. `GIT_SHALLOW` is removed
   (a shallow clone of an arbitrary SHA is unreliable across servers), and
   `GIT_SUBMODULES ""` prevents a dependency from pulling unpinned submodule code.
2. **Treat `.gitignore` as hygiene, not security.** It is rewritten to anchor
   build/runtime patterns (`/build/`, `/out/`) and ignore only specific high-signal
   secret file types (`*.pem`, `*.key`, `.env*`, `id_rsa`, `*.tfstate`). Broad
   substring and bare-extension globs are removed so source files and test fixtures
   are never silently dropped.
3. **Delegate real secret protection** to tooling that actually inspects content:
   a gitleaks pre-commit hook and GitHub push protection / secret scanning on the
   public repo. **(Both are now implemented — see Status below.)**

## Consequences

- Positive:
  - Reproducible, tamper-evident dependency builds.
  - No risk of `.gitignore` hiding real source files or golden fixtures from commits.
  - Secret protection moves to mechanisms that can actually block a leak.
- Negative / tradeoffs:
  - SHA pins are less human-readable than tags (mitigated by the tag comment).
  - Bumping a dependency requires resolving the new SHA (a `git ls-remote` away).
- Implementation status:
  - ✅ gitleaks pre-commit hook: `.githooks/pre-commit` + `.gitleaks.toml` (anchored
    allowlist), activated via `scripts/setup-hooks.sh` (`core.hooksPath=.githooks`).
    gitleaks is installed from a checksum-verified release binary.
  - ✅ GitHub secret scanning + push protection enabled on the repo.
  - ⏳ Non-provider (generic) pattern scanning: **not yet enabled.** The REST API
    returns 200 but leaves `secret_scanning_non_provider_patterns=disabled` (the
    available token scope / account tier cannot toggle it). Enable it in the web UI:
    Settings → Code security → Secret scanning → "Scan for non-provider patterns".
  - ✅ A gitleaks CI job runs on every push and pull request (`.github/workflows/ci.yml`),
    installing the **same** pinned, checksum-verified gitleaks 8.30.1 binary this ADR
    specifies for the local hook (`sha256sum -c`), not a mutable action. Two caveats, both
    tracked: (a) it is only truly *unbypassable* once the four CI checks are set as
    **required status checks** in branch protection (see phase-ci design §2 / the H1 note) —
    until then a red PR can still be merged; (b) on push/PR it scans only the event's commit
    range, so a weekly `schedule` + `workflow_dispatch` run `gitleaks git --log-opts=--all`
    for the genuine full-history scan.
- Follow-ups:
  - Enable non-provider pattern scanning via the web UI (see above).
  - Consider switching to release-archive URLs + hash (`URL_HASH`) for faster, git-free
    fetches if CI clone time becomes a concern.

## Alternatives considered

- **Keep tag pins** — readable but mutable; rejected on integrity grounds.
- **Release archive + SHA-256 hash** — strongest and fastest; deferred as a possible
  future optimization since SHA-pinned git clones already give integrity + reproducibility.
- **Rely on `.gitignore` for secrets** — rejected; it is the wrong tool for the job.
