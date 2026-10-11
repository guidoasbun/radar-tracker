# ADR-0001 — Use Ninja as the CMake generator

- **Status:** Accepted
- **Date:** 2026-10-11
- **Deciders:** @Guido Asbun
- **Phase:** 0 (Scaffold & toolchain)

## Context

The project uses CMake with `CMakePresets.json`. CMake can drive several build
backends; the two realistic choices on Linux are **Ninja** and **Unix Makefiles**.
The spec's technology table names Ninja explicitly ("CMake + Ninja"). The build
machine is RHEL 10.2, where GNU Make 4.4.1 was already present and Ninja was not.

The forcing question at Phase 0 was whether to install Ninja (one `sudo dnf install
ninja-build`) or configure the presets to use Make and avoid any install.

## Decision

Use **Ninja** as the generator for all presets. Ninja 1.11.1 was installed via
`sudo dnf install ninja-build` and verified at `/usr/bin/ninja`.

## Consequences

- Positive:
  - Matches the spec exactly; no divergence to explain to a reviewer.
  - Faster incremental builds and better parallelism than Make, which matters as the
    module count and FetchContent dependency build time grow.
  - Ninja's dependency tracking is more precise, reducing spurious rebuilds.
  - GitHub Actions `ubuntu-24.04` runners have Ninja available, so local and CI
    generators match.
- Negative / tradeoffs:
  - Requires a system package install (done, one-time).
- Follow-ups:
  - None. Presets hardcode `"generator": "Ninja"`.

## Alternatives considered

- **Unix Makefiles** — zero install, universally present. Rejected because it
  diverges from the spec, builds slower, and the install cost was trivial and already
  paid. Make remains a fallback if a future environment lacks Ninja, by overriding the
  preset generator.
