# ADR-0007 — `io` throws a dedicated `ConfigError`, not `std::invalid_argument`

- **Status:** Accepted
- **Date:** 2026-10-21
- **Deciders:** project author; raised while scoping the M1 `io` module
- **Phase:** M1 (`io`)

## Context

The pure algorithm modules (`motion`, `sensor`, `kalman`) throw `std::invalid_argument`
on invalid input. That is the right choice there: invalid input to those functions is a
**programmer error** (a caller passed a NaN `dt`, a negative `sigma_m`), caught in tests.

`io::load_scenario` is different. A bad scenario file is **user / operator input**, not a
programmer bug, and the spec's CLI contract treats it specially:

> Exit codes: 0 success, **1 bad arguments or config (the message names the JSON path)**,
> 2 I/O or network error.

So `apps/*` must be able to distinguish "the scenario file is bad — exit 1, name the path"
from "a genuine `std::invalid_argument` escaped a library function — that's a bug." If `io`
also threw `std::invalid_argument`, `main` could not tell the two apart with a single
`catch`, and a real library bug could be silently reported to the user as "bad config."

Issue #5 also requires `io` to reject non-finite/non-positive `dt_s`, non-finite target
states, bad `sigma_m`, negative/duplicate target ids, etc. — each needs a clear,
path-naming message.

## Decision

`io` throws a dedicated exception:

```cpp
class ConfigError : public std::runtime_error {
  using std::runtime_error::runtime_error;
};
```

Every invalid-scenario case (syntax error, missing key, wrong type, or a failed issue-#5
validation) throws `ConfigError` with a message that names the source and the offending
JSON path, e.g. `single.json: targets[1].vx0 is not finite`. No `nlohmann::json::exception`
escapes `io` — it is caught at the parse boundary and rethrown as `ConfigError`.

`ConfigError` derives from `std::runtime_error` (hence `std::exception`), so:
- `apps/*` catch `ConfigError` precisely and map it to **exit code 1**, printing the message;
- a caller that only catches `std::exception` still behaves correctly;
- a genuine `std::invalid_argument` from a lower module is **not** caught by the
  `ConfigError` handler, so a library bug is not misreported as bad config.

## Consequences

- Positive: `main` gets a clean, precise mapping from "bad scenario" to exit code 1 with a
  path-named message, exactly as the spec requires, without swallowing real bugs.
- Positive: the exception type documents intent — a `ConfigError` is always
  untrusted-input rejection, never an internal precondition violation.
- Negative / tradeoff: a second error convention in the codebase (`std::invalid_argument`
  in pure modules, `ConfigError` in `io`). Mitigated by the clear rule above — the split is
  *programmer error* vs *operator input*, which is a meaningful boundary, not noise. Recorded
  here and in the `io` requirements/design so the two conventions are intentional.
- Follow-up: `net` (M3) decoding of untrusted datagrams may want the same treatment (a
  dedicated decode error vs a programmer bug); revisit when `net` lands rather than
  generalising prematurely.

## Alternatives considered

- **Reuse `std::invalid_argument` (the pure-module convention).** Rejected: `main` cannot
  distinguish bad config from a library bug with one `catch`, and the spec's exit-code-1
  "name the JSON path" contract would have to be reconstructed by string-matching messages.
- **A broad `radar::Error` base for all modules.** Rejected as premature: only `io` (and
  later `net`) handle untrusted input; retrofitting the pure modules onto a new base is
  churn with no M1 benefit. Can introduce a hierarchy later if `net` wants a sibling.
- **Return a `std::expected`/error-code instead of throwing.** Rejected for M1: the rest of
  the codebase uses exceptions for invalid input (sensor/kalman/motion), and `load_scenario`
  is a once-per-run call, not a hot path; a throw keeps the convention uniform and the happy
  path clean. (An `expected`-based API could wrap this later without changing the validation.)
