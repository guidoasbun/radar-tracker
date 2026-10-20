# M1 · `sensor` module — Requirements

## Purpose

Turn true target positions into the noisy measurements the tracker sees. This is the
third M1 module: `kalman` is next, and **Phase K** (the filter-verification battery) needs a
Gaussian sensor plus the CV `motion` model to generate Monte-Carlo data. The spec's M1
scenario `single.json` is **1 CV target, P_D = 1.0, λ = 0** — so the *noise-only* slice of the
sensor (Gaussian measurement noise, every target detected, no clutter) is exactly what M1
needs. The richer sensor (missed detections, clutter, shuffling) is specified but
**deferred to M2**, documented below.

## Scope

In scope (M1, noise-only slice):
- A measurement function: given a set of true in-view targets (each an id + true 2D
  position) and a random generator, emit **one** `LabeledMeasurement` per target with
  `z = p + n`, `n ~ N(0, σ_m²·I)` (independent axes), `origin = target id`.
- `SensorConfig` carrying `σ_m` (measurement-noise standard deviation per axis, default 50 m).
- Explicit, caller-owned randomness: the function takes a `std::mt19937_64&` by reference.
  Per the spec, one generator is seeded from the scenario (default 42) and threaded through
  the whole run, so results reproduce exactly. The sensor holds **no** RNG state of its own.
- The tracker-facing handoff is `labeled.measurement` — a plain `Measurement`, no origin
  (type-system isolation from `types`).

Out of scope / **deferred to M2** (documented in "Deviations / deferred" below):
- `P_D < 1` Bernoulli detection trials (missed detections).
- Poisson(λ) clutter with uniform placement in the surveillance square.
- Shuffling the per-scan measurement list.
- Surveillance-square entry/exit culling (targets outside ±L produce no detections).

## Deferred to M2 — the full sensor model

The spec's per-scan sensor model is:
1. For every target **inside the square**, draw Bernoulli(`P_D`); on success emit `z = p + n`.
2. Draw clutter count from Poisson(`λ`); place each clutter point uniformly in the square.
3. **Shuffle** the measurement list so order carries no origin information.
4. Record the hidden origin tag (target id, or −1 for clutter) — only on `LabeledMeasurement`.

M1 implements step 1 with `P_D = 1` and the "inside the square" check trivially true (the
single target is in view), and step 4. Steps 2–3 and the Bernoulli trial and square culling
land in M2, alongside the multi-target `baseline.json`/`dense_clutter.json` scenarios that
actually exercise them. Defaults for the deferred knobs (for reference): `P_D = 0.9`,
`λ = 2`, surveillance half-width `L = 20 000 m`.

Note on ownership: surveillance-square culling is assigned to `sensor` (it governs whether a
detection is emitted), consistent with the deviation recorded in the `motion` requirements.

## Acceptance criteria

### AC-1 — Noise model is correct
For a target at true position `p`, a produced measurement `z` satisfies `z = p + n` with
`n` drawn from `N(0, σ_m²)` per axis, axes independent. Verified statistically (AC-5).

### AC-2 — One labelled measurement per target, origin preserved, order shuffled
With `P_D = 1` and N in-view targets, the function returns exactly N `LabeledMeasurement`s,
one per target with `origin` equal to its target's id. The list is **shuffled**, so its order
carries no origin information; tests therefore look a measurement up **by origin**, never by
index. Extracting `.measurement` yields a plain `Measurement` with only a position. (A 0/1-
element shuffle draws nothing from `rng`, so single-target runs reproduce bit-for-bit.)

### AC-3 — Reproducible, caller-owned RNG
Given the same generator state (same seed), the function produces **bit-for-bit identical**
measurements; a different seed produces different ones. The sensor stores no RNG state; the
caller owns the single `std::mt19937_64`.

### AC-4 — Degenerate noise
`σ_m = 0` yields `z = p` exactly (no randomness consumed beyond the distribution's defined
behaviour) — a clean no-noise baseline.

### AC-5 — Statistical properties (fixed seed, deterministic test)
Over a large sample (≥ 100 000 draws for a fixed target, fixed seed):
- empirical mean of `z − p` is ≈ 0 on each axis (within tolerance);
- sample standard deviation per axis ≈ `σ_m` (within tolerance);
- raw measurement error RMS `‖z − p‖` ≈ `σ_m·√2` (≈ 70.7 m at σ_m = 50), matching the spec's
  "about 71 m" raw RMSE;
- the x/y **covariance** `Cov(e_x, e_y)` ≈ 0 — the check that actually tests AC-1's
  "independent axes". A correlated-noise bug (e.g. `ny = nx`) leaves mean/std/RMSE unchanged but
  drives this toward `σ²`, so without it independence is untested. Verified by a mutation.
The test uses a fixed seed so it is deterministic in CI (no flakiness), with tolerances loose
enough for seed stability but tight enough to catch a wrong σ, a mean offset, or correlation.

### AC-6 — Invalid inputs are rejected at runtime (not just asserted)
`sense` **throws `std::invalid_argument`** if `σ_m` is non-finite or negative, or if any target
has `id < 0` (would collide with the clutter sentinel −1) or a non-finite position. A runtime
throw, not an `assert`, because a bad `σ_m` in a release build otherwise fails *open* to a
silent noise-free sensor; the `EXPECT_THROW` tests therefore run in **every** build, not only
where asserts are live. `io` also validates scenario-sourced values at the boundary (issue #5).

### AC-7 — Tooling + tested
- `clang-format --dry-run --Werror` clean; `ci`/`ci-clang` build 0 warnings in our code.
- `tests/test_sensor.cpp` registered via `radar_add_test`, green under `release`, `ci`,
  `ci-clang`, and `asan`.

## Deviations from the spec (recorded for the M2 implementer)

The spec's `test_sensor.cpp` contract lists three checks this M1 slice does **not** implement,
because the features they test are deferred to M2 (above). The *tests* are deferred with the
features, recorded here so they are not forgotten:

1. **"P_D = 0 yields no detections"** — needs the Bernoulli detection trial (`P_D < 1`), M2.
2. **"λ = 0 yields no clutter" (and λ > 0 yields ~λ clutter per scan)** — needs the Poisson
   clutter generator, M2.
3. **"measurement order is shuffled"** — the M1 sensor *does* shuffle (M3 from the PR review),
   but the spec's intent (clutter and detections interleaved, order non-informative) is only
   fully exercised once clutter exists (M2). M1 covers the detection-only shuffle via the
   look-up-by-origin tests.

What M1 *does* cover from the contract: the Gaussian noise statistics (mean/std/RMSE),
independence of axes (covariance), reproducibility, and input validation.
