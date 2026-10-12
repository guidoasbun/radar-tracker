# ADR-0002 — Newer compilers than spec; scope `-Werror` to the `ci` preset

- **Status:** Accepted
- **Date:** 2026-10-11
- **Deciders:** @Guido Asbun
- **Phase:** 0 (Scaffold & toolchain)

## Context

The spec sets minimum compilers of **GCC 12** and **Clang 15**. The build machine
(RHEL 10.2) provides substantially newer toolchains:

- GCC (g++) **14.3.1**
- Clang/Clang++ **21.1.8**

Both exceed the minimums and fully support C++20, which is good. However, newer
compiler releases — Clang 21 in particular, six major versions beyond the spec's
assumed Clang 15 — add diagnostics and tighten existing ones. The spec also mandates
a strict warning set promoted to errors with `-Werror` in the `ci` preset:

```
-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
-Wnon-virtual-dtor -Wold-style-cast
```

With `-Werror` applied everywhere, a warning that only Clang 21 emits (and that the
spec author never saw on Clang 15) would block ordinary local development, not just CI.

## Decision

1. Adopt the installed GCC 14 and Clang 21 as the local matrix; keep the spec's
   minimums as the documented floor.
2. Apply the full warning set to our own targets (`libradar`, `apps`, `tests`) under
   **all** presets, but add **`-Werror` only in the `ci` preset** — not in `debug`.
3. Treat Clang-21-specific warnings as **real cleanups to fix**, not noise to silence
   wholesale. Any targeted suppression must be narrowly scoped (specific check, specific
   site) and carry an inline comment with the reason.
4. Mark fetched dependency headers (Eigen, etc.) as **SYSTEM** includes so their
   internal warnings never reach our `-Werror`.

## Consequences

- Positive:
  - Local dev stays productive; `debug` builds don't fail on a stray new-compiler warning.
  - The CI gate remains strict and honest — exactly what a defense reviewer inspects.
  - Fixing Clang 21's stricter diagnostics generally improves code quality.
  - Modern C++20 features are fully available.
- Negative / tradeoffs:
  - A warning can slip through local `debug` builds and only surface in the `ci`
    preset or on CI. Mitigation: run `cmake --preset ci` locally before pushing, which
    the Phase 0 verification steps already exercise.
  - Clang 21 may require more up-front cleanups than Clang 15 would have.
- Follow-ups:
  - Keep GitHub Actions on `ubuntu-24.04`; note its GCC/Clang versions may differ from
    local, so CI is the final arbiter for the matrix.

## Third-party warning handling (recorded suppressions)

Our strict `-Werror` set applies **only to our own targets** (`radar`, test
executables), never to fetched dependencies:

1. **SYSTEM includes.** Every dependency is declared with the `SYSTEM` keyword in
   `cmake/Dependencies.cmake` (requires CMake ≥ 3.25), so Eigen/GoogleTest/json/CLI11
   headers are consumed via `-isystem`. Their internal warnings therefore never reach
   our `-Werror` build. (This replaced an earlier, incorrect attempt to mark an ALIAS
   target's includes SYSTEM, which does not work — the root cause the PR review caught.)

2. **One scoped suppression — `-Wno-character-conversion` on the GoogleTest targets.**
   Clang 21 emits `-Wcharacter-conversion` while compiling GoogleTest's *own* sources
   (`gtest-printers`), where GoogleTest self-includes via a non-SYSTEM path. GoogleTest
   is not built with `-Werror`, so this is harmless, but it is silenced on the
   `gtest`/`gtest_main`/`gmock`/`gmock_main` targets only, guarded by a
   `check_cxx_compiler_flag`, in `tests/CMakeLists.txt`. It touches no code of ours.
   This is the scoped suppression this ADR commits to recording.

No suppressions are applied to our own code: `radar`, `linalg.cpp`,
`test_smoke.cpp`, and `test_linalg.cpp` (which includes `<Eigen/Dense>` and does matrix
operations) all compile warning-free under both GCC 14 and Clang 21 with the full
`-Werror` set.

## Alternatives considered

- **`-Werror` in every preset** — maximally strict but blocks local iteration on
  new-compiler warnings; rejected for developer-experience reasons. CI still enforces it.
- **Pin to the spec's exact GCC 12 / Clang 15** — would require installing older
  toolchains on RHEL 10; unnecessary since newer compilers are a superset for C++20 and
  catching their stricter warnings is a benefit, not a liability.
- **Blanket-disable the stricter Clang warnings** — defeats the purpose of the strict
  gate and would look evasive to a reviewer; rejected in favor of targeted, documented fixes.
