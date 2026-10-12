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
   public repo. (Enabling these is tracked as a follow-up task.)

## Consequences

- Positive:
  - Reproducible, tamper-evident dependency builds.
  - No risk of `.gitignore` hiding real source files or golden fixtures from commits.
  - Secret protection moves to mechanisms that can actually block a leak.
- Negative / tradeoffs:
  - SHA pins are less human-readable than tags (mitigated by the tag comment).
  - Bumping a dependency requires resolving the new SHA (a `git ls-remote` away).
  - gitleaks / push protection must be set up separately (follow-up).
- Follow-ups:
  - Add a gitleaks pre-commit hook and enable GitHub secret scanning + push protection.
  - Consider switching to release-archive URLs + hash (`URL_HASH`) for faster, git-free
    fetches if CI clone time becomes a concern.

## Alternatives considered

- **Keep tag pins** — readable but mutable; rejected on integrity grounds.
- **Release archive + SHA-256 hash** — strongest and fastest; deferred as a possible
  future optimization since SHA-pinned git clones already give integrity + reproducibility.
- **Rely on `.gitignore` for secrets** — rejected; it is the wrong tool for the job.
