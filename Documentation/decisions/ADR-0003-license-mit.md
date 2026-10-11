# ADR-0003 — License: MIT

- **Status:** Accepted
- **Date:** 2026-10-11
- **Deciders:** @Guido Asbun
- **Phase:** 0 (Scaffold & toolchain)

## Context

The spec's open questions list "License: MIT or Apache-2.0 — before the repository
goes public." The repository is a public portfolio project intended to demonstrate
engineering skill to employers; it has a single author and no external contributors.

## Decision

License the project under the **MIT License**, copyright "2026 Guido Asbun".
`LICENSE` added at the repository root.

## Consequences

- Positive:
  - Simplest, most permissive, most widely recognized license; minimal friction for
    anyone (including prospective employers) reading or reusing the code.
  - Standard choice for individual open-source showcase repositories.
- Negative / tradeoffs:
  - No explicit patent grant (Apache-2.0 provides one). Acceptable: this is estimation
    and tracking code with no novel patented methods, single-author, portfolio-oriented.
- Follow-ups:
  - Reference the license in `README.md`.
  - This resolves the spec's "License" open question.

## Alternatives considered

- **Apache-2.0** — adds an explicit patent grant and contributor terms; valuable for
  multi-contributor or corporate projects, but heavier and unnecessary for a
  single-author portfolio repo.
