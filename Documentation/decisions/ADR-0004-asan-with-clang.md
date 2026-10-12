# ADR-0004 — Run the `asan` preset with Clang (not GCC)

- **Status:** Accepted
- **Date:** 2026-10-11
- **Deciders:** @Guido Asbun
- **Phase:** 0 (Scaffold & toolchain)

## Context

The `asan` preset runs the test suite under AddressSanitizer + UndefinedBehaviorSanitizer.
On the build machine (RHEL 10.2, GCC 14.3.1), linking a sanitized binary with GCC fails:

```
/usr/bin/ld: cannot find /usr/lib64/libasan.so.8.0.0: No such file or directory
```

Investigation showed GCC's `libasan.so` under `/usr/lib/gcc/.../14/` is only a 38-byte
linker script; the actual `libasan.so.8.0.0` runtime is not installed (RHEL ships it in
a separate package that is absent). By contrast, Clang 21's compiler-rt sanitizer
runtimes (`libclang_rt.asan*`) are fully installed, and a direct test confirmed Clang's
ASan/UBSan/LeakSanitizer compiles, links, runs, and correctly detects an injected leak.

## Decision

Pin the `asan` preset to **Clang** explicitly (`CMAKE_C_COMPILER=clang`,
`CMAKE_CXX_COMPILER=clang++`) so sanitizer runs are deterministic regardless of the
user's default `CC`/`CXX`. GCC remains the compiler for `debug`, `release`, and `ci`;
Clang additionally covers `ci` in the CI matrix. The two-compiler matrix the spec asks
for is preserved.

## Consequences

- Positive:
  - Sanitizer runs work immediately with no `sudo`/package changes.
  - Clang's sanitizer implementation is the reference one, with excellent diagnostics,
    and includes LeakSanitizer by default.
  - Deterministic: the preset forces Clang rather than depending on environment.
- Negative / tradeoffs:
  - Sanitizer coverage comes from Clang, not GCC. In practice ASan/UBSan findings are
    compiler-agnostic for our code, so this is a negligible loss.
  - A second toolchain (Clang) is required to run the `asan` preset locally. It is
    already installed and also used for the `ci` matrix, so no new dependency.
- Follow-ups:
  - If GCC-based sanitizers are ever wanted, `sudo dnf install libasan libubsan`
    provides the missing runtime; a GCC asan preset could then be added.
  - GitHub CI runners (`ubuntu-24.04`) have the GCC sanitizer runtime; the sanitizer
    job may use either compiler there. We standardize on Clang for parity with local.

## Alternatives considered

- **Install the GCC sanitizer runtime** (`sudo dnf install libasan libubsan`) — matches
  the spec's implied GCC usage but requires a privileged install and leaves the preset
  dependent on it. Deferred as an optional follow-up, not the default.
- **Drop the asan preset** — unacceptable; the spec mandates ASan/UBSan on every push.
