// tests/test_phase_k.cpp — Phase K: the exhaustive Kalman filter-verification battery.
//
// The filter is the keystone the whole project depends on, so before any tracker
// logic is built on top of it we verify it is *statistically consistent*: its claimed
// uncertainty (P and S) matches the errors it actually makes, measured over fixed-seed
// Monte-Carlo runs that reuse motion (truth) + sensor (Gaussian noise).
//
// MODEL-MATCHED TRUTH (the design that makes NEES a real chi^2(4) test). The truth is
// propagated with the FILTER'S OWN process-noise model injected:
//
//     truth_{k+1} = F · truth_k + chol(Q) · w,   w ~ N(0, I4),
//
// where Q = process_noise(dt, sigma_a) and chol(Q) is its lower Cholesky factor. With
// truth and filter sharing the same (F, Q) dynamics, the filter is correctly specified,
// so posterior NEES = e^T P^-1 e is genuinely chi^2(4) (mean 4.0) over the WHOLE
// post-burn-in tail — not a transient that merely passes through a band. (An earlier
// revision propagated truth as a NOISE-FREE straight line; that made the filter
// permanently under-confident in velocity, so steady-state NEES settled near 2.4 and
// the ~3.6 seen early was just the initial-uncertainty transient on its way down. PR
// #12 review caught this; injecting Q is the fix.) NIS was ~2.0 either way because R
// dominates S.
//
// SCOPE (see Documentation/planning/phase-k/{requirements,design}.md). This is a
// self-consistency + common-bug battery, NOT a tuning oracle. The discriminating power
// is against covariance-STRUCTURE implementation bugs — a transposed covariance predict
// (F^T P F), a mis-scaled Q (both directions now caught), an S missing R — plus the
// convergence gate. It does NOT catch a wrong *gain scale*: a half-gain filter still
// tracks and stays nearly consistent (NEES ~4.0), so that bug is caught deterministically
// at the unit level by test_kalman's UpdateStateMatchesHandComputed (PR #11 mutation
// log). Bands were measured on the correct filter and mutation-verified — see design.md.
//
// INTERVALS: NIS, NEES, and credibility are all measured over the FULL post-burn-in
// tail [kBurnIn, kScans) (~18k samples) — the model-matched truth makes both statistics
// stationary there, so no window is needed. Convergence (AC-K5) uses the late tail
// [kScans-kTail, kScans), where the spec M1 RMSE ratio is defined.
//
// NEES is the keystone: it exercises the full 4x4 P including the velocity block that
// the position-only NIS never sees directly.
//
// DETERMINISM: one std::mt19937_64 per run, seeded kBaseSeed + run_index, threaded
// through the init measurement, the per-scan process-noise draw w, and every scan's
// sense() — one generator, fixed order, mirroring the real sim. The battery relies on
// the chi^2 DISTRIBUTION holding (implementation-independent), never on cross-library
// byte-equality of the normal distribution. Mutation resistance is a verify-step
// obligation (AC-K6), demonstrated by perturbing production kalman, not a test-side
// self-check.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include "radar/kalman.hpp"
#include "radar/motion.hpp"
#include "radar/sensor.hpp"
#include "radar/types.hpp"

namespace {

// ---- Monte-Carlo configuration (Documentation/planning/phase-k/requirements.md) ----
constexpr std::uint64_t kBaseSeed = 42;  // spec default scenario seed
constexpr int kRuns = 100;               // M: earns the tight NIS/NEES ensemble bands
constexpr int kScans = 200;              // K: convergence + a long measured tail
constexpr int kBurnIn = 20;              // discard the one-point-init transient
constexpr int kTail = 30;                // late-tail length for the convergence RMSE ratio

// Guard every division below (AC-K7 robustness): if the constants are retuned so an
// interval is empty or runs off the end, fail at compile time with a clear message
// instead of at runtime with an unhelpful NaN.
static_assert(kBurnIn >= 0 && kTail > 0, "intervals must be non-empty");
static_assert(kBurnIn < kScans, "post-burn-in tail must be non-empty");
static_assert(kTail <= kScans, "convergence tail must fit within the run");
static_assert(kRuns > 0, "need at least one Monte-Carlo run");

const radar::KalmanConfig kCfg{.dt = 0.1, .sigma_a = 5.0, .sigma_m = 50.0};  // spec
const radar::SensorConfig kSensor{.sigma_m = 50.0};                          // matches

// Process-noise covariance Q for the truth injection, built from the spec's
// white-noise-acceleration formula by HAND — deliberately NOT radar::process_noise().
// If truth drew its noise from the same function the filter predicts with, a bug in
// process_noise would scale truth and filter identically and NEES could not see it
// (the common-mode trap ADR-0006 names for F). Driving truth with an independent Q
// means a mis-scaled production Q (e.g. Q x0.1 / x10) creates a real truth/filter
// mismatch that NEES catches. For the CORRECT filter this literal equals
// process_noise(dt, sigma_a) exactly, so truth and a correct filter stay matched and
// NEES ~ chi^2(4). (This literal mirrors test_kalman's ProcessNoiseMatchesSpecLiteral.)
Eigen::Matrix4d truth_process_noise_literal() {
  const double dt = kCfg.dt;
  const double t2 = dt * dt;
  const double t3 = t2 * dt;
  const double t4 = t3 * dt;
  const double qa = kCfg.sigma_a * kCfg.sigma_a;
  Eigen::Matrix4d q = Eigen::Matrix4d::Zero();
  q(radar::kPx, radar::kPx) = t4 / 4.0;
  q(radar::kPy, radar::kPy) = t4 / 4.0;
  q(radar::kVx, radar::kVx) = t2;
  q(radar::kVy, radar::kVy) = t2;
  q(radar::kPx, radar::kVx) = t3 / 2.0;
  q(radar::kVx, radar::kPx) = t3 / 2.0;
  q(radar::kPy, radar::kVy) = t3 / 2.0;
  q(radar::kVy, radar::kPy) = t3 / 2.0;
  return qa * q;
}

// Posterior NEES = e^T P^-1 e, computed through the SAME SPD guard the production filter
// uses (LLT), not a raw solve. Returns +inf (not a throw) if P is not SPD, so the caller
// still records a sample that the finiteness check in AC-K3 will catch and attribute to
// that test — rather than raising a gtest expectation here, while the shared ensemble()
// is being built, where a failure would be misreported under whichever test ran first.
double nees(const Eigen::Vector4d& e, const Eigen::Matrix4d& p) {
  const Eigen::LLT<Eigen::Matrix4d> llt(p);
  if (llt.info() != Eigen::Success) {
    return std::numeric_limits<double>::infinity();  // indefinite P (Joseph/symmetrize bug)
  }
  return e.dot(llt.solve(e));
}

// Per-run samples. All statistics come from the SAME runs — no redundant simulation —
// so they are mutually consistent views of one trajectory. NIS/NEES/credibility span
// the full post-burn-in tail; convergence sums the late tail.
struct RunSamples {
  std::vector<double> nis;   // y^T S^-1 y over [kBurnIn, kScans)   (chi^2_2)
  std::vector<double> nees;  // e^T P^-1 e over [kBurnIn, kScans)   (chi^2_4)
  // Position error and the filter's claimed position variance per post-burn-in scan,
  // for the physical-units credibility check (AC-K4).
  std::vector<Eigen::Vector2d> pos_err;
  std::vector<Eigen::Vector2d> pos_var_claim;
  double tail_track_sq{0.0};  // sum |est_pos - truth_pos|^2 over the late tail
  double tail_raw_sq{0.0};    // sum |z - truth_pos|^2 over the late tail
};

// A single filter trajectory, fixed seed. Truth obeys the filter's own (F, Q) model.
RunSamples run_filter(std::uint64_t seed) {
  RunSamples out;
  std::mt19937_64 rng{seed};  // NOLINT(cert-msc32-c,cert-msc51-cpp): fixed test seed

  // Truth's process noise is the INDEPENDENT literal Q (not process_noise — see the
  // comment on truth_process_noise_literal: avoids the common-mode trap). Q from the
  // white-noise-acceleration model is SPD for dt > 0, so its Cholesky factor exists.
  const Eigen::Matrix4d q = truth_process_noise_literal();
  const Eigen::LLT<Eigen::Matrix4d> q_llt(q);
  EXPECT_EQ(q_llt.info(), Eigen::Success) << "literal truth Q must be SPD for chol(Q)";
  const Eigen::Matrix4d chol_q = q_llt.matrixL();
  std::normal_distribution<double> gauss(0.0, 1.0);

  // Straight-line CV target at aircraft speed; the process noise perturbs it each scan.
  Eigen::Vector4d truth{0.0, 0.0, 200.0, -100.0};

  const radar::TrueTarget t0{.id = 0, .position = truth.head<2>()};
  radar::TrackState est =
      radar::initiate_track(radar::sense({t0}, kSensor, rng)[0].measurement, kCfg);

  for (int k = 0; k < kScans; ++k) {
    // Truth propagation WITH the filter's process noise: truth = F*truth + chol(Q)*w.
    // Draw w from the same per-run rng (fixed order: w before the sensor draw).
    Eigen::Vector4d w;
    for (int i = 0; i < 4; ++i) {
      w(i) = gauss(rng);
    }
    truth = radar::propagate_cv(truth, kCfg.dt) + chol_q * w;

    const radar::TrueTarget t{.id = 0, .position = truth.head<2>()};
    const radar::Measurement z = radar::sense({t}, kSensor, rng)[0].measurement;

    // Predict, then NIS from the PREDICTED state (innovations are a property of the
    // prediction — the textbook NIS). gate_distance(y, S) is exactly y^T S^-1 y with
    // the production Cholesky SPD guard, so we measure the shipped quantity.
    const radar::TrackState pred = radar::predict(est, kCfg);
    const Eigen::Vector2d y = radar::innovation(pred, z);
    const Eigen::Matrix2d s = radar::innovation_covariance(pred, kCfg);
    const double nis = radar::gate_distance(y, s);

    // Update, then NEES from the POSTERIOR vs truth, through the SPD guard.
    est = radar::update(pred, z, kCfg);
    const Eigen::Vector4d e = est.x - truth;

    if (k >= kBurnIn) {
      out.nis.push_back(nis);
      out.nees.push_back(nees(e, est.P));
      out.pos_err.emplace_back(e.head<2>());
      out.pos_var_claim.emplace_back(est.P(radar::kPx, radar::kPx), est.P(radar::kPy, radar::kPy));
    }
    if (k >= kScans - kTail) {
      out.tail_track_sq += (est.x.head<2>() - truth.head<2>()).squaredNorm();
      out.tail_raw_sq += (z.position - truth.head<2>()).squaredNorm();
    }
  }
  return out;
}

double mean(const std::vector<double>& v) {
  EXPECT_FALSE(v.empty());
  double s = 0.0;
  for (const double x : v) {
    s += x;
  }
  return s / static_cast<double>(v.size());
}

// Pooled ensemble over M runs with distinct deterministic seeds. Keeps run 0's single-run
// NIS mean so AC-K1 need not re-simulate it (review finding #8), and the min NEES + a
// finiteness flag so AC-K3 — not the ensemble builder — reports a bad NEES sample.
struct Ensemble {
  double mean_nis{0.0};        // pooled over the full post-burn-in tail
  double mean_nees{0.0};       // pooled over the full post-burn-in tail
  double run0_mean_nis{0.0};   // single-run time-average, run 0 (AC-K1)
  double min_nees{0.0};        // smallest NEES sample seen (for AC-K3 >= 0 check)
  bool all_nees_finite{true};  // false if any NEES sample was non-finite (indefinite P)
  double mean_ratio{0.0};      // pooled RMS(est-truth)/RMS(z-truth) over the late tail
  double max_run_ratio{0.0};   // worst single-run ratio
  double tail_err_var{0.0};    // pooled per-axis filtered position-error variance (tail)
  Eigen::Vector2d emp_pos_var{Eigen::Vector2d::Zero()};     // empirical error variance (tail)
  Eigen::Vector2d mean_claim_var{Eigen::Vector2d::Zero()};  // mean claimed P_pos (tail)
};

Ensemble run_ensemble() {
  Ensemble en;
  double sum_nis = 0.0;
  double sum_nees = 0.0;
  double min_nees = std::numeric_limits<double>::infinity();
  bool all_finite = true;
  long long n_consistency = 0;
  double sum_track_sq = 0.0;
  double sum_raw_sq = 0.0;
  double max_ratio = 0.0;
  Eigen::Vector2d sum_err_sq{Eigen::Vector2d::Zero()};
  Eigen::Vector2d sum_claim{Eigen::Vector2d::Zero()};

  for (int r = 0; r < kRuns; ++r) {
    const RunSamples rs = run_filter(kBaseSeed + static_cast<std::uint64_t>(r));
    if (r == 0) {
      en.run0_mean_nis = mean(rs.nis);  // the AC-K1 single-run statistic, reused
    }
    for (const double v : rs.nis) {
      sum_nis += v;
    }
    for (const double v : rs.nees) {
      sum_nees += v;
      // Track finiteness and the running minimum explicitly: a plain std::min would
      // silently drop a NaN (NaN < x is false), so AC-K3 could not see it.
      if (!std::isfinite(v)) {
        all_finite = false;
      } else {
        min_nees = std::min(min_nees, v);
      }
    }
    for (const Eigen::Vector2d& e : rs.pos_err) {
      sum_err_sq += e.cwiseProduct(e);
    }
    for (const Eigen::Vector2d& c : rs.pos_var_claim) {
      sum_claim += c;
    }
    n_consistency += static_cast<long long>(rs.nees.size());

    sum_track_sq += rs.tail_track_sq;
    sum_raw_sq += rs.tail_raw_sq;
    const double run_ratio =
        std::sqrt(rs.tail_track_sq / rs.tail_raw_sq);  // per-run RMSE ratio over its tail
    max_ratio = std::max(max_ratio, run_ratio);
  }

  // All denominators are guaranteed positive by the static_asserts (non-empty intervals)
  // and kRuns > 0, so these divisions cannot produce a NaN from 0/0.
  const auto n = static_cast<double>(n_consistency);
  en.mean_nis = sum_nis / n;
  en.mean_nees = sum_nees / n;
  en.min_nees = min_nees;
  en.all_nees_finite = all_finite;
  en.mean_ratio = std::sqrt(sum_track_sq / sum_raw_sq);
  en.max_run_ratio = max_ratio;
  // Filtered position-error variance over the tail, pooled per axis. sum_track_sq sums
  // |err|^2 (BOTH axes) over kTail scans x kRuns runs, so the sample count is
  // (kTail x kRuns) vector samples = (kTail x kRuns x 2) scalar/axis samples.
  en.tail_err_var = sum_track_sq / (static_cast<double>(kTail) * kRuns * 2.0);
  // Mean position error over the tail is ~0 (unbiased filter), so the raw second moment
  // about zero is the empirical variance — exactly what the filter's P claims to predict.
  en.emp_pos_var = sum_err_sq / n;
  en.mean_claim_var = sum_claim / n;
  return en;
}

// Built once and shared by every TEST — the ensemble is deterministic (fixed seeds), so
// one Monte-Carlo run feeds all acceptance criteria (including AC-K1's single-run
// statistic, captured from run 0, so no simulation is repeated).
const Ensemble& ensemble() {
  static const Ensemble en = run_ensemble();
  return en;
}

}  // namespace

// ---- AC-K1 — NIS single-run time-average inside the spec band [1.5, 2.6] ----

TEST(PhaseK, NisSingleRunInSpecBand) {
  // Run 0's time-average over the full post-burn-in tail (~180 samples), the spec's
  // "mean NIS over a run" instrument. Reused from the ensemble — no re-simulation.
  const double mean_nis = ensemble().run0_mean_nis;
  EXPECT_TRUE(std::isfinite(mean_nis));
  EXPECT_GT(mean_nis, 1.5);  // spec acceptance band (mean target ~2.0; measured ~2.01)
  EXPECT_LT(mean_nis, 2.6);
}

// ---- AC-K2 — NIS Monte-Carlo ensemble mean, tight consistency gate [1.85, 2.15] ----

TEST(PhaseK, NisEnsembleMeanTight) {
  // NIS ~ chi^2_2, mean 2.0, pooled over the full post-burn-in tail across all runs
  // (~18k samples; measured ~2.01). Tight band: a transposed-covariance predict pushes
  // the pooled mean well outside (measured ~3.5). NIS is position-innovation only, so a
  // gain/Q bug barely moves it — that is by design; NEES (AC-K3) carries those.
  const double mean_nis = ensemble().mean_nis;
  EXPECT_GT(mean_nis, 1.85);
  EXPECT_LT(mean_nis, 2.15);
}

// ---- AC-K3 — NEES state consistency (keystone) ----

TEST(PhaseK, NeesEnsembleMeanStateConsistent) {
  // Posterior NEES = e^T P^-1 e ~ chi^2_4, mean 4.0 (measured ~4.03 over the full tail —
  // the model-matched truth makes this a genuine steady-state chi^2(4) result, not a
  // transient, see the file header). Band [3.5, 4.6] keeps the correct filter inside
  // while a lying covariance fails BOTH ways: a 10x-inflated Q (under-confident) measures
  // ~2.4 (<3.5); a 0.1x Q (over-confident) measures ~12.9 (>4.6); a transposed predict
  // measures ~168. This is the check the position-only NIS cannot make — it sees the full
  // 4x4 P including the velocity block.
  const Ensemble& en = ensemble();
  EXPECT_TRUE(en.all_nees_finite) << "a NEES sample was non-finite — indefinite posterior P";
  EXPECT_GE(en.min_nees, 0.0);  // non-negative by construction (P SPD)
  EXPECT_GT(en.mean_nees, 3.5);
  EXPECT_LT(en.mean_nees, 4.6);
}

// ---- AC-K4 — Covariance credibility: empirical variance ~ claimed P (±15%/axis) ----

TEST(PhaseK, CovarianceCredibilityVarianceMatch) {
  // NEES restated in physical units: the filter's stated metres-of-uncertainty must
  // match the errors it actually makes over the tail (measured ratio ~1.03/axis). No
  // explicit coverage-fraction assertion — at this sample size its binomial spread is
  // too wide to be both non-flaky and discriminating; variance matching carries the
  // same signal cleanly and catches the gross transposed-covariance bug (ratio ~79).
  const Ensemble& en = ensemble();
  for (int axis = 0; axis < 2; ++axis) {
    const double emp = en.emp_pos_var(axis);
    const double claim = en.mean_claim_var(axis);
    EXPECT_GT(claim, 0.0);
    EXPECT_NEAR(emp / claim, 1.0, 0.15)
        << "axis " << axis << ": empirical var " << emp << " vs claimed " << claim;
  }
}

// ---- AC-K5 — Convergence over Monte-Carlo + the filter actually reduces error ----

TEST(PhaseK, ConvergenceOverMonteCarlo) {
  // Spec M1 gate: RMS(est-truth)/RMS(z-truth) over the converged late tail <= 0.5. A
  // pass-through (est = z) gives ratio 1.0 and fails; a transposed covariance predict
  // degrades tracking and also fails. The per-run bound guards against a single
  // pathological run hiding behind a good mean.
  const Ensemble& en = ensemble();
  EXPECT_LT(en.mean_ratio, 0.5);
  EXPECT_LT(en.max_run_ratio, 0.6);
  // The filtered tail position-error variance is strictly below the raw per-axis
  // measurement variance sigma_m^2 — i.e. the filter genuinely reduces error, not just
  // relatively (review finding #3; the ratio alone would also pass a scaled-up raw).
  const double sigma_m2 = kCfg.sigma_m * kCfg.sigma_m;
  EXPECT_LT(en.tail_err_var, sigma_m2)
      << "filtered tail variance " << en.tail_err_var << " not below sigma_m^2 " << sigma_m2;
}
