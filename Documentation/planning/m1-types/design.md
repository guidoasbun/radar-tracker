# M1 · `types` module — Design

## 1. Shape of the module

`types` is a single header, `include/radar/types.hpp`, with no `.cpp`. Every type is a
plain aggregate struct (or a scoped enum). There is no behavior to compile, so a
translation unit is unnecessary; the header joins `libradar`'s public include tree and
is pulled in by the modules that need it. A dedicated test TU
(`tests/test_types.cpp`) gives the header a compiler and anchors the compile-time
guarantees.

Only dependency: Eigen (fixed-size `Vector2d`, `Vector4d`, `Matrix4d`, pulled via
`<Eigen/Core>`), consistent with the module table (`types` depends on Eigen) and the
"fixed-size, no heap" rationale in the stack section.

## 2. Types

### `Measurement` — the tracker-facing detection

```cpp
struct Measurement {
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};  // (x east, y north) in meters
};
```

Deliberately minimal: just the position. **No origin, no target id, no clutter flag,
and no per-detection time.** This is the central design decision of the module: the
spec requires that origin leakage be *impossible by construction*, not merely avoided
by convention. A tracker that only ever sees `Measurement` cannot cheat because the
information is not in the type.

**No per-measurement timestamp (S3).** An earlier draft carried a `double time_s` on
each `Measurement`. It was removed: every detection in a scan happens at the scan time,
which `Scan::time_s` already holds; the M3 UDP measurement item is 16 bytes (`x_m`,
`y_m`) with no per-measurement time; and a per-detection field the simulator could fill
differently for targets vs clutter is exactly the kind of side channel the spec wants
ruled out by the type system. Downstream code reads the time from the enclosing `Scan`.

### `LabeledMeasurement` — sim/scoring only

```cpp
struct LabeledMeasurement {
  Measurement measurement{};   // the exact value the tracker would receive
  int origin{kClutterOrigin};  // target id (>= 0), or kClutterOrigin (-1) for clutter
};
```

Composition, not inheritance: a `LabeledMeasurement` *has a* `Measurement`. The
simulator builds these (to record what it emitted), the scorer reads them (to pair
tracks with truth), and the handoff to the tracker is `labeled.measurement` — a clean
value with no tag. There is no slicing or implicit conversion that could carry the
origin across the boundary.

`kClutterOrigin = -1` matches the spec ("target id, or −1 for clutter").

### `Scan` — one sweep

```cpp
struct Scan {
  int index{0};                           // 0-based scan number k
  double time_s{0.0};                     // k * dt
  std::vector<Measurement> measurements;  // already shuffled; order carries no info
};
```

A `Scan` holds `Measurement`s (not labeled ones): it is exactly what crosses the
sim→tracker boundary (a function call in M1/M2, a UDP datagram in M3). The simulator's
*labeled* record is a separate parallel structure it keeps for truth/scoring; it does
not travel in a `Scan`.

### `TrackStatus` — lifecycle

```cpp
enum class TrackStatus : std::uint8_t {
  Tentative = 0,
  Confirmed = 1,
  Coasting = 2,
  Deleted = 3,
};
```

Scoped enum (no implicit int conversion, satisfies `-Werror`). Names and meanings from
the glossary: *tentative* (newly started), *confirmed* (proven by M-of-N), *coasting*
(confirmed but missed this scan, predicting only). `Deleted` is the terminal lifecycle
state used by `track`/`tracker` in M2; defined here so the enum is complete and shared.

**Fixed underlying type and values (S5).** The base is `std::uint8_t` to match the M3
`TRACKS` datagram's `status` field (a `uint8` on the wire) and the CSV encoding. Because
these values are serialized, the enumerators are given **explicit values** and pinned
with `static_assert` in the header: reordering them must not silently change the wire
protocol or file meaning. Forward note for M3: the decoder must reject status bytes
above `Deleted` (3) before casting.

### `TrackState` — a filter estimate

```cpp
struct TrackState {
  Eigen::Vector4d x{Eigen::Vector4d::Zero()};  // [px, py, vx, vy]
  Eigen::Matrix4d P{Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN())};
};
```

`x` is the state vector `[p_x, p_y, v_x, v_y]` (glossary). `P` is `Eigen::Matrix4d`,
exactly as the Kalman section mandates ("Store P as `Eigen::Matrix4d`").

**`P` defaults to NaN, not identity (S4).** An identity default is `1 m²` on the
diagonal — a *valid-looking but wrong* covariance. The harm is in the **velocity block**,
not the gate: the spec's starting velocity sigma is `150 m/s` (`σ_v² ≈ 22500`), whereas an
identity `P₀` claims `σ_v = 1 m/s`. The filter would firmly believe a brand-new track is
nearly stationary, so velocity converges slowly and tracks on fast (e.g. `250 m/s`) targets
lag. The *position* gate is barely affected: the innovation covariance is `S = HPHᵀ + R`
with `R = σ_m² = 2500 m²` dominating, so an identity `P₀` makes the position gate only
~1.4× too tight. (An earlier draft of this doc claimed ~50×; that was wrong — it ignored
`R`'s dominance in `S`.)

A NaN default does **not** by itself "fail loudly" — NaN only surfaces if code checks for
it (a `reject-if d2 > γ` gate would treat a NaN distance as *inside* the gate). What NaN
does is make an uninitialised `P` **detectable**: it propagates, so an explicit check turns
misuse into an error instead of a silently-wrong estimate. Forward note for `kalman`/
`association`: `assert(P.allFinite())` at the start of `predict`/`update`, and write the
gate as `accept-if (d2 <= γ)` so a NaN distance fails *closed* (reject) rather than open.
Setting a real `P₀` is the filter's job (`kalman`, M1).

`TrackState` is the *estimate payload only*: hit/miss counters, status, and M-of-N
history live on `Track` (M2), which composes a `TrackState`. Keeping status off
`TrackState` avoids duplicating lifecycle state in two places.

### State-vector index constants

```cpp
inline constexpr int kPx = 0;
inline constexpr int kPy = 1;
inline constexpr int kVx = 2;
inline constexpr int kVy = 3;
inline constexpr int kClutterOrigin = -1;
```

Named indices keep every downstream module (`motion`, `kalman`, `metrics`) referring to
the same layout rather than hardcoding `0..3`. `inline constexpr` so the header is ODR-safe
when included in multiple TUs.

## 3. Why these choices

- **Aggregates with brace-init defaults** over constructors: `types` must have no logic;
  aggregates give zero-cost value types with designated-initializer ergonomics (C++20).
- **Composition for `LabeledMeasurement`** over inheritance: prevents any inheritance-based
  slicing path that could smuggle the origin tag into tracker code, and keeps `Measurement`
  a flat, single-member value type. (Note: `Measurement`/`TrackState` are *not* trivially
  copyable — Eigen's fixed-size types have a user-provided copy constructor — but they are
  ordinary copyable value types, which is what the tests assert.)
- **Eigen fixed-size members**: no heap allocation; matches the aerospace-code rationale and
  the Kalman storage requirement.
- **Scoped enum**: no accidental int arithmetic, clean under `-Wconversion`/`-Werror`.

## 4. Origin-isolation guards (where and what)

### 4.1 In the header, so they hold in every build (S2)

The origin-isolation `static_assert`s live in `types.hpp` immediately after the structs,
**not** in the test. The requirement is that leakage is impossible "by the type system";
placing the asserts next to the struct is what makes that literally true for every
translation unit that includes the header, including a `-DRADAR_BUILD_TESTS=OFF` release
or Docker build where the test is never compiled.

### 4.2 Checks that pin the shape (S1)

The header asserts member **types**, not just member count, because a count-only check
misses realistic leaks (e.g. widening `position` to `Vector3d` and hiding origin in the
third component, or inserting `int origin` between fields):

```cpp
namespace detail {  // implementation detail of the guards, not public API
template <class T> concept HasOrigin = requires(T t) { t.origin; };
}

static_assert(std::is_same_v<decltype(Measurement::position), Eigen::Vector2d>);
static_assert(sizeof(Measurement) == sizeof(Eigen::Vector2d));  // nothing beyond position
static_assert(!detail::HasOrigin<Measurement>);                 // tracker-facing: no tag
static_assert(detail::HasOrigin<LabeledMeasurement>);           // sim/scoring: has the tag
static_assert(!std::is_base_of_v<Measurement, LabeledMeasurement>);  // composition only
```

The `TrackStatus` wire values are pinned the same way (`static_cast<uint8_t>(…) == 0..3`).

### 4.3 Checks that stay in the test

`tests/test_types.cpp` keeps a belt-and-suspenders **structured-binding guard** — a
non-`constexpr` function whose `const auto& [p] = m;` binding has exactly one name, so an
origin field inserted *anywhere* (front, middle, or back) fails to compile. It also keeps
the value-semantics traits (`is_copy_constructible`/`is_copy_assignable`).

### 4.4 Runtime behaviour the test covers

1. **Defaults** — zeroed `position`, zeroed state `x`, **NaN** `P`, `kClutterOrigin` default.
2. **Pinned `TrackStatus` values** — `0..3` as above.
3. **Index constants** — `kPx..kVy` map to the intended slots; writing `x(kVx)` lands in
   the velocity-x component.
4. **`LabeledMeasurement` handoff** — extracting `.measurement` yields a plain `Measurement`
   value-equal to the one put in, with no tag reachable.

## 5. Risks / watch items and forward notes

| Risk | Mitigation |
| --- | --- |
| A later edit adds/moves an origin field or widens `position` in `Measurement` | Header `static_assert`s on member types + `sizeof` + `!detail::HasOrigin`, plus the test's structured-binding guard (§4). |
| Index constants drift from the actual `x` layout | Test writes/reads each slot through the named index. |
| An uninitialised `P` silently used as a valid covariance | `P` defaults to NaN, which makes misuse *detectable* (propagates to an explicit `allFinite()` check); the gate is written `accept-if d2 <= γ` so NaN fails closed (S4). |
| Serialized `TrackStatus` values change under a reorder | Explicit values pinned by `static_assert` (S5). |

**Forward notes (not changes to `types` now):**
- **S6 — origin sentinel vs scenario ids.** `origin` is a plain `int`, so a scenario with
  a target `"id": -1` would make that target's detections score as clutter. `io/config`
  (M1) must reject target ids below 0 (and ids that would overflow when mapped to a
  `uint32` `track_id`). A small strong type for `origin` is an option if this proves fragile.
- **S7 — non-finite input.** `Measurement` will accept NaN/Inf positions. The M3
  `net/codec` must reject non-finite `float64` values before building a `Measurement`, or a
  single bad datagram will poison a track's covariance.

**Nit applied:** the header includes `<Eigen/Core>` (sufficient for the fixed-size
`Vector2d`/`Vector4d`/`Matrix4d`), not `<Eigen/Dense>`, so every file that includes
`types.hpp` compiles a smaller Eigen surface.
