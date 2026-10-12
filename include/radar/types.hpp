#ifndef RADAR_TYPES_HPP
#define RADAR_TYPES_HPP

#include <Eigen/Core>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

namespace radar {

/// State-vector layout: x = [p_x, p_y, v_x, v_y] in the East-North frame.
/// Named indices so every module (motion, kalman, metrics) refers to the same
/// layout instead of hardcoding 0..3. inline constexpr => ODR-safe across TUs.
inline constexpr int kPx = 0;  ///< position east (m)
inline constexpr int kPy = 1;  ///< position north (m)
inline constexpr int kVx = 2;  ///< velocity east (m/s)
inline constexpr int kVy = 3;  ///< velocity north (m/s)

/// Hidden origin sentinel: a measurement produced by clutter, not a target.
/// A non-negative origin is a target id. Used only by sensor/metrics, never by
/// the tracker (see LabeledMeasurement).
inline constexpr int kClutterOrigin = -1;

/// A single noisy 2D detection as seen by the tracker.
///
/// Deliberately minimal: just the position (meters, East-North). It has NO
/// origin, target id, clutter flag, or per-detection time. Per the spec, the
/// Measurement type handed to the tracker must not reveal where a detection
/// came from, so origin leakage is prevented by the type system rather than by
/// discipline. The hidden origin lives on LabeledMeasurement, which only the
/// simulator and scorer use.
///
/// There is intentionally no per-measurement timestamp: every detection in a
/// scan happens at the scan time, which Scan::time_s already carries, and the
/// M3 wire format has no per-measurement time field. A per-detection time would
/// duplicate Scan::time_s and give the simulator a channel it could fill
/// differently for targets vs clutter -- exactly the side channel the spec
/// rules out by construction.
struct Measurement {
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};  ///< (x east, y north) in meters
};

/// A Measurement paired with its hidden origin tag. Built by the simulator to
/// record what it emitted and read by the scorer to pair tracks with truth.
/// Never passed to the tracker: the tracker-facing value is `.measurement`,
/// a plain Measurement with no tag. Composition (not inheritance) guarantees
/// there is no slicing path that could carry the origin across the boundary.
struct LabeledMeasurement {
  Measurement measurement{};   ///< the exact value the tracker would receive
  int origin{kClutterOrigin};  ///< target id (>= 0), or kClutterOrigin (-1) for clutter
};

/// One sensor sweep. Holds plain Measurements (not labeled ones): this is exactly
/// what crosses the sim->tracker boundary (a function call in M1/M2, a UDP
/// datagram in M3). The list is already shuffled, so its order carries no
/// information about origin. The scan time is carried once here, not per
/// measurement.
struct Scan {
  int index{0};                           ///< 0-based scan number k (>= 0)
  double time_s{0.0};                     ///< k * dt (s)
  std::vector<Measurement> measurements;  ///< this sweep's detections, order-randomized
};

/// Track lifecycle status (glossary). Values are pinned explicitly because they
/// go on the wire in M3 (the TRACKS datagram's status is a uint8) and into CSV:
/// reordering the enumerators must not silently change the protocol or file
/// meaning. The M3 decoder must reject status bytes above Deleted before casting.
///  - Tentative: newly started, not yet proven.
///  - Confirmed: proven by M-of-N hits.
///  - Coasting:  confirmed but missed this scan, predicting only.
///  - Deleted:   terminal lifecycle state (used by track/tracker in M2).
enum class TrackStatus : std::uint8_t {
  Tentative = 0,
  Confirmed = 1,
  Coasting = 2,
  Deleted = 3,
};

/// A Kalman filter estimate: the state vector and its covariance. This is the
/// estimate payload only -- hit/miss counters, status, and M-of-N history live
/// on Track (M2), which composes a TrackState. P is stored as Eigen::Matrix4d
/// per the Kalman spec.
///
/// P defaults to NaN, not identity. An identity default (1 m^2 on the diagonal)
/// looks like a valid covariance but is wrong in a way that does not fail: its
/// real harm is the velocity block. The spec's starting velocity sigma is
/// 150 m/s, so sigma_v^2 ~ 22500; an identity P0 claims sigma_v = 1 m/s, i.e.
/// the filter would firmly believe a brand-new track is nearly stationary, so
/// velocity converges slowly and tracks on fast (e.g. 250 m/s) targets lag.
/// (The position gate is barely affected: the innovation covariance is
/// S = H P H^T + R with R = sigma_m^2 = 2500 m^2 dominating, so an identity P0
/// makes the position gate only ~1.4x too tight -- not the large factor an
/// earlier draft claimed.) A NaN default does not by itself "fail loudly" --
/// NaN only surfaces if code checks for it -- but it makes an uninitialised P
/// *detectable*: NaN propagates, and an explicit check turns misuse into an
/// error instead of a silently-wrong estimate.
///
/// Forward note for kalman/association (M1/M2): assert(P.allFinite()) at the
/// start of predict/update, and write the gate as accept-if (d2 <= gamma) so a
/// NaN distance fails closed (reject) rather than open. Setting a real P0 is
/// the filter's job (kalman, M1).
struct TrackState {
  Eigen::Vector4d x{Eigen::Vector4d::Zero()};  ///< [p_x, p_y, v_x, v_y]
  Eigen::Matrix4d P{
      Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN())};  ///< covariance
};

// ---------------------------------------------------------------------------
// Origin-isolation guarantees, enforced in the header so they hold in EVERY
// build (including -DRADAR_BUILD_TESTS=OFF), not only when tests compile. The
// requirement is that leakage is impossible "by the type system"; putting the
// asserts next to the struct is what makes that literally true for every TU
// that includes this header.
// ---------------------------------------------------------------------------

/// True iff T exposes an `origin` member. The hidden tag must live ONLY on
/// LabeledMeasurement, never on the tracker-facing Measurement. Internal
/// (radar::detail): it is an implementation detail of the guards below, not
/// public API.
namespace detail {
template <class T>
concept HasOrigin = requires(T t) { t.origin; };
}  // namespace detail

// Measurement must be exactly a single Eigen::Vector2d position -- no origin,
// no target id, no clutter flag, no per-detection time. These check member
// *types*, not just count, so widening `position` or swapping a field for a
// struct that smuggles an origin is caught too.
static_assert(std::is_same_v<decltype(Measurement::position), Eigen::Vector2d>,
              "Measurement::position must be an Eigen::Vector2d");
static_assert(sizeof(Measurement) == sizeof(Eigen::Vector2d),
              "Measurement must contain nothing beyond its 2D position");
static_assert(!detail::HasOrigin<Measurement>, "Measurement must not expose an origin");
static_assert(detail::HasOrigin<LabeledMeasurement>,
              "LabeledMeasurement must expose the origin tag");
static_assert(!std::is_base_of_v<Measurement, LabeledMeasurement>,
              "LabeledMeasurement must compose, not inherit, Measurement");

// TrackStatus wire/CSV values are fixed; pin them so a reorder is a build error.
static_assert(static_cast<std::uint8_t>(TrackStatus::Tentative) == 0);
static_assert(static_cast<std::uint8_t>(TrackStatus::Confirmed) == 1);
static_assert(static_cast<std::uint8_t>(TrackStatus::Coasting) == 2);
static_assert(static_cast<std::uint8_t>(TrackStatus::Deleted) == 3);

}  // namespace radar

#endif  // RADAR_TYPES_HPP
