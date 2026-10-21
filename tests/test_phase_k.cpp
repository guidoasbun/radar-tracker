// tests/test_phase_k.cpp — Phase K: the exhaustive Kalman filter-verification battery.
//
// The filter is the keystone the whole project depends on, so before any tracker
// logic is built on top of it we verify it is *statistically consistent*: its
// claimed uncertainty (P and S) matches the errors it actually makes, measured over
// fixed-seed Monte-Carlo runs that reuse motion (truth) + sensor (Gaussian noise).
//
// SCOPE (see Documentation/planning/phase-k/{requirements,design}.md). This is a
// self-consistency + common-bug battery, NOT a tuning oracle. On matched CV-truth /
// CV-filter data a correctly implemented filter sits near the chi^2 means, so the
// discriminating power is against covariance-STRUCTURE implementation bugs — a
// transposed covariance predict (F^T P F), a mis-scaled Q, an S missing R — plus the
// convergence gate. It does NOT catch a wrong *gain scale* on clean CV data (a
// half-gain filter still tracks a straight line and stays nearly consistent); that
// bug is caught deterministically at the unit level by test_kalman's
// UpdateStateMatchesHandComputed (PR #11 mutation log). The bands here were chosen
// from measured statistics and mutation-verified (AC-K6) — see design.md §4/§7.
//
// MEASUREMENT INTERVALS — deliberately different for NIS vs NEES:
//   * NIS  (AC-K1/K2): the FULL post-burn-in tail [kBurnIn, kScans) — ~18k samples —
//     matching the spec's "time average over all post-burn-in scans". NIS is the
//     position-innovation statistic and does NOT drift (measured ~1.95 over the whole
//     tail, ~1.97 over the early window), so the full tail is both correct and catches
//     a covariance bug that only appears late.
//   * NEES (AC-K3) + credibility (AC-K4): a WINDOW [kBurnIn, kBurnIn+kWindow). On
//     perfectly-CV truth the filter's covariance reaches a steady state whose velocity
//     block is slightly larger than the velocity error it achieves, so the FULL-tail
//     NEES drifts below the chi^2_4 mean of 4 to ~2.4 (a benign, correct
//     under-confidence). Early after burn-in the chi^2_4 expectation holds (~3.6), so
//     the window — not the whole tail — is where the principled NEES band lives.
//   * Convergence (AC-K5): the LATE tail [kScans-kTail, kScans), where the RMSE ratio
//     is what the spec M1 gate specifies.
//
// NEES is the keystone: it exercises the full 4x4 P including the velocity block that
// the position-only NIS never sees directly.
//
// DETERMINISM: one std::mt19937_64 per run, seeded kBaseSeed + run_index, threaded
// through the init measurement and every scan's sense() — mirrors the real sim's
// single-generator contract. The battery relies on the chi^2 DISTRIBUTION holding
// (implementation-independent), never on cross-library byte-equality of the normal
// distribution. Mutation resistance is a verify-step obligation (AC-K6), demonstrated
// by perturbing production kalman, not a test-side self-check.

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
constexpr int kWindow = 40;              // NEES/credibility window [kBurnIn, kBurnIn+kWindow)
constexpr int kTail = 30;                // late-tail length for the convergence RMSE ratio

// Guard every per-interval division below (AC-K7 robustness): if the constants are
// retuned so an interval is empty or runs off the end, fail at compile time with a
// clear message instead of at runtime with an unhelpful NaN.
static_assert(kBurnIn >= 0 && kWindow > 0 && kTail > 0, "intervals must be non-empty");
static_assert(kBurnIn + kWindow <= kScans, "NEES window must fit within the run");
static_assert(kBurnIn < kScans, "NIS post-burn-in tail must be non-empty");
static_assert(kTail <= kScans, "convergence tail must fit within the run");
static_assert(kRuns > 0, "need at least one Monte-Carlo run");

const radar::KalmanConfig kCfg{.dt = 0.1, .sigma_a = 5.0, .sigma_m = 50.0};  // spec
const radar::SensorConfig kSensor{.sigma_m = 50.0};                          // matches

// Posterior NEES = e^T P^-1 e, computed through the SAME SPD guard the production
// filter uses (LLT), not a raw solve. If a regression makes P indefinite (broken
// Joseph form, missing symmetrize), llt.info() != Success and we fail loudly rather
// than reading a meaningless number that might still land in-band (review finding #1).
double nees_or_fail(const Eigen::Vector4d& e, const Eigen::Matrix4d& p) {
  const Eigen::LLT<Eigen::Matrix4d> llt(p);
  EXPECT_EQ(llt.info(), Eigen::Success) << "posterior P is not SPD — Joseph/symmetrize regression";
  const double nees = e.dot(llt.solve(e));
  EXPECT_TRUE(std::isfinite(nees) && nees >= 0.0) << "NEES must be finite and non-negative";
  return nees;
}

// Per-run samples. All statistics come from the SAME runs — no redundant simulation —
// so they are mutually consistent views of one trajectory. NIS spans the full
// post-burn-in tail; NEES/credibility span the window; convergence sums the late tail.
struct RunSamples {
  std::vector<double> nis;   // y^T S^-1 y over [kBurnIn, kScans)            (chi^2_2)
  std::vector<double> nees;  // e^T P^-1 e over [kBurnIn, kBurnIn+kWindow)   (chi^2_4)
  // Position error and the filter's claimed position variance per window scan (AC-K4).
  std::vector<Eigen::Vector2d> pos_err;
  std::vector<Eigen::Vector2d> pos_var_claim;
  double tail_track_sq{0.0};  // sum |est_pos - truth_pos|^2 over the late tail
  double tail_raw_sq{0.0};    // sum |z - truth_pos|^2 over the late tail
};

// A single filter trajectory on a straight-line CV target, fixed seed.
RunSamples run_filter(std::uint64_t seed) {
  RunSamples out;
  std::mt19937_64 rng{seed};  // NOLINT(cert-msc32-c,cert-msc51-cpp): fixed test seed

  // Straight-line CV truth at aircraft speed (same seed target as the test_kalman
  // convergence smoke, reused for continuity).
  Eigen::Vector4d truth{0.0, 0.0, 200.0, -100.0};

  const radar::TrueTarget t0{.id = 0, .position = truth.head<2>()};
  radar::TrackState est =
      radar::initiate_track(radar::sense({t0}, kSensor, rng)[0].measurement, kCfg);

  for (int k = 0; k < kScans; ++k) {
    truth = radar::propagate_cv(truth, kCfg.dt);
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
      out.nis.push_back(nis);  // full post-burn-in tail
    }
    if (k >= kBurnIn && k < kBurnIn + kWindow) {
      out.nees.push_back(nees_or_fail(e, est.P));  // window (NEES drifts late — see header)
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

// Pooled ensemble over M runs with distinct deterministic seeds. Also keeps run 0's
// single-run NIS mean so AC-K1 need not re-simulate it (review finding #8).
struct Ensemble {
  double mean_nis{0.0};       // pooled over the full post-burn-in tail
  double mean_nees{0.0};      // pooled over the window
  double run0_mean_nis{0.0};  // single-run time-average, run 0 (AC-K1)
  double mean_ratio{0.0};     // pooled RMS(est-truth)/RMS(z-truth) over the late tail
  double max_run_ratio{0.0};  // worst single-run ratio
  double tail_err_var{0.0};   // pooled per-axis filtered position-error variance (tail)
  Eigen::Vector2d emp_pos_var{Eigen::Vector2d::Zero()};     // empirical error variance (window)
  Eigen::Vector2d mean_claim_var{Eigen::Vector2d::Zero()};  // mean claimed P_pos (window)
};

Ensemble run_ensemble() {
  Ensemble en;
  double sum_nis = 0.0;
  double sum_nees = 0.0;
  long long n_nis = 0;
  long long n_nees = 0;
  double sum_track_sq = 0.0;
  double sum_raw_sq = 0.0;
  double max_ratio = 0.0;
  long long n_tail = 0;
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
    }
    for (const Eigen::Vector2d& e : rs.pos_err) {
      sum_err_sq += e.cwiseProduct(e);
    }
    for (const Eigen::Vector2d& c : rs.pos_var_claim) {
      sum_claim += c;
    }
    n_nis += static_cast<long long>(rs.nis.size());
    n_nees += static_cast<long long>(rs.nees.size());

    sum_track_sq += rs.tail_track_sq;
    sum_raw_sq += rs.tail_raw_sq;
    n_tail += kTail;  // each run contributes kTail scans x 2 axes to the sums
    const double run_ratio =
        std::sqrt(rs.tail_track_sq / rs.tail_raw_sq);  // per-run RMSE ratio over its tail
    max_ratio = std::max(max_ratio, run_ratio);
  }

  // All denominators are guaranteed positive by the static_asserts (non-empty
  // intervals) and kRuns > 0, so these divisions cannot produce a NaN from 0/0.
  en.mean_nis = sum_nis / static_cast<double>(n_nis);
  en.mean_nees = sum_nees / static_cast<double>(n_nees);
  en.mean_ratio = std::sqrt(sum_track_sq / sum_raw_sq);
  en.max_run_ratio = max_ratio;
  // Filtered position-error variance over the tail, pooled across both axes and all
  // runs: sum_track_sq is sum of |err|^2 (both axes) over kTail scans x kRuns runs, so
  // dividing by (n_tail x 2 axes) gives the per-axis variance (AC-K5).
  en.tail_err_var = sum_track_sq / (static_cast<double>(n_tail) * 2.0);
  // Mean position error over the window is ~0 (unbiased filter), so the raw second
  // moment about zero is the empirical variance — exactly what the filter's P predicts.
  en.emp_pos_var = sum_err_sq / static_cast<double>(n_nees);
  en.mean_claim_var = sum_claim / static_cast<double>(n_nees);
  return en;
}

// Built once and shared by every TEST — the ensemble is deterministic (fixed seeds),
// so one Monte-Carlo run feeds all acceptance criteria (including AC-K1's single-run
// statistic, captured from run 0, so no simulation is repeated).
const Ensemble& ensemble() {
  static const Ensemble en = run_ensemble();
  return en;
}

}  // namespace

// ---- AC-K1 — NIS single-run time-average inside the spec band [1.5, 2.6] ----

TEST(PhaseK, NisSingleRunInSpecBand) {
  // Run 0's time-average over the FULL post-burn-in tail (~180 samples), the spec's
  // "mean NIS over a run" instrument. Reused from the ensemble — no re-simulation.
  const double mean_nis = ensemble().run0_mean_nis;
  EXPECT_TRUE(std::isfinite(mean_nis));
  EXPECT_GT(mean_nis, 1.5);  // spec acceptance band (mean target ~2.0; measured ~1.95)
  EXPECT_LT(mean_nis, 2.6);
}

// ---- AC-K2 — NIS Monte-Carlo ensemble mean, tight consistency gate [1.85, 2.15] ----

TEST(PhaseK, NisEnsembleMeanTight) {
  // NIS ~ chi^2_2, mean 2.0, pooled over the full post-burn-in tail across all runs
  // (~18k samples; measured ~1.95). Tight band: a transposed-covariance predict pushes
  // the pooled mean well outside (measured ~4.4). NIS is position-innovation only, so a
  // gain/Q bug barely moves it — that is by design; NEES (AC-K3) carries those.
  const double mean_nis = ensemble().mean_nis;
  EXPECT_GT(mean_nis, 1.85);
  EXPECT_LT(mean_nis, 2.15);
}

// ---- AC-K3 — NEES state consistency (keystone) ----

TEST(PhaseK, NeesEnsembleMeanStateConsistent) {
  // Posterior NEES = e^T P^-1 e ~ chi^2_4, mean 4.0; measured ~3.6 over the window
  // (mild, correct steady-state under-confidence pulls it below 4 over the full tail —
  // see the file header, which is why NEES is windowed while NIS is not). Band
  // [3.0, 4.5] keeps the correct filter inside while a lying covariance fails: a
  // transposed predict measures ~96 (>>4.5), a 10x-inflated Q measures ~2.7 (<3.0).
  // Per-sample finiteness/non-negativity and the SPD-of-P guard are enforced inside
  // nees_or_fail() for EVERY window sample (not a post-hoc min), so a NaN or an
  // indefinite P fails loudly rather than slipping through std::min.
  EXPECT_GT(ensemble().mean_nees, 3.0);
  EXPECT_LT(ensemble().mean_nees, 4.5);
}

// ---- AC-K4 — Covariance credibility: empirical variance ~ claimed P (±15%/axis) ----

TEST(PhaseK, CovarianceCredibilityVarianceMatch) {
  // NEES restated in physical units: the filter's stated metres-of-uncertainty must
  // match the errors it actually makes over the window (measured ratio ~0.99/axis).
  // No explicit coverage-fraction assertion — at this sample size its binomial spread
  // is too wide to be both non-flaky and discriminating; variance matching carries the
  // same signal cleanly and catches the gross transposed-covariance bug (ratio ~70).
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
  // Spec M1 gate: RMS(est-truth)/RMS(z-truth) over the converged late tail <= 0.5
  // (measured ~0.18). A pass-through (est = z) gives ratio 1.0 and fails; a transposed
  // covariance predict degrades tracking to ~0.76 and also fails. The per-run bound
  // guards against a single pathological run hiding behind a good mean.
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
