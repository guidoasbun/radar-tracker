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
// WINDOW: NIS/NEES/credibility are measured over a defined window just after burn-in,
// [kBurnIn, kBurnIn + kWindow). On perfectly-CV truth the filter's covariance reaches
// a steady state whose velocity block is slightly larger than the velocity error it
// actually achieves, so the FULL-run-tail NEES drifts below the chi^2_4 mean of 4 (a
// benign, correct under-confidence). Early after burn-in the chi^2_4 expectation holds
// (~3.6), so the window — not the whole tail — is where the principled band lives.
// Convergence (AC-K5) is measured over the LATE tail, where the RMSE ratio is what the
// spec M1 gate specifies.
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
constexpr int kWindow = 40;              // consistency window [kBurnIn, kBurnIn+kWindow)
constexpr int kTail = 30;                // late-tail length for the convergence RMSE ratio

const radar::KalmanConfig kCfg{.dt = 0.1, .sigma_a = 5.0, .sigma_m = 50.0};  // spec
const radar::SensorConfig kSensor{.sigma_m = 50.0};                          // matches

// Per-run samples. NIS/NEES/convergence/credibility all come from the SAME runs — no
// redundant simulation — so the statistics are mutually consistent views of one
// trajectory. Consistency samples (nis/nees/pos_err/pos_var_claim) are collected over
// the window; convergence sums over the late tail.
struct RunSamples {
  std::vector<double> nis;   // y^T S^-1 y per scan in the window (chi^2_2)
  std::vector<double> nees;  // e^T P^-1 e per scan in the window (chi^2_4)
  // Position error and the filter's claimed position variance, per scan in the window,
  // for the physical-units credibility check (AC-K4).
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

    // Update, then NEES from the POSTERIOR vs truth: e^T P^-1 e via LLT (P is SPD
    // after Joseph + symmetrize).
    est = radar::update(pred, z, kCfg);
    const Eigen::Vector4d e = est.x - truth;
    const double nees = e.dot(est.P.llt().solve(e));

    if (k >= kBurnIn && k < kBurnIn + kWindow) {
      out.nis.push_back(nis);
      out.nees.push_back(nees);
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

// Pooled ensemble over M runs with distinct deterministic seeds.
struct Ensemble {
  double mean_nis{0.0};
  double mean_nees{0.0};
  double min_nees{0.0};
  double mean_ratio{0.0};  // pooled RMS(est-truth)/RMS(z-truth) over the late tail
  double max_run_ratio{0.0};
  Eigen::Vector2d emp_pos_var{Eigen::Vector2d::Zero()};     // empirical error variance (window)
  Eigen::Vector2d mean_claim_var{Eigen::Vector2d::Zero()};  // mean claimed P_pos (window)
};

Ensemble run_ensemble() {
  Ensemble en;
  double sum_nis = 0.0;
  double sum_nees = 0.0;
  double min_nees = std::numeric_limits<double>::infinity();
  long long n_consistency = 0;
  double sum_track_sq = 0.0;
  double sum_raw_sq = 0.0;
  double max_ratio = 0.0;
  Eigen::Vector2d sum_err_sq{Eigen::Vector2d::Zero()};
  Eigen::Vector2d sum_claim{Eigen::Vector2d::Zero()};

  for (int r = 0; r < kRuns; ++r) {
    const RunSamples rs = run_filter(kBaseSeed + static_cast<std::uint64_t>(r));
    for (const double v : rs.nis) {
      sum_nis += v;
    }
    for (const double v : rs.nees) {
      sum_nees += v;
      min_nees = std::min(min_nees, v);
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

  const auto n = static_cast<double>(n_consistency);
  en.mean_nis = sum_nis / n;
  en.mean_nees = sum_nees / n;
  en.min_nees = min_nees;
  en.mean_ratio = std::sqrt(sum_track_sq / sum_raw_sq);
  en.max_run_ratio = max_ratio;
  // Mean position error over the window is ~0 (unbiased filter), so the raw second
  // moment about zero is the empirical variance — exactly what the filter's P predicts.
  en.emp_pos_var = sum_err_sq / n;
  en.mean_claim_var = sum_claim / n;
  return en;
}

// Built once and shared by every TEST — the ensemble is deterministic (fixed seeds),
// so one Monte-Carlo run feeds all acceptance criteria.
const Ensemble& ensemble() {
  static const Ensemble en = run_ensemble();
  return en;
}

}  // namespace

// ---- AC-K1 — NIS single-run time-average inside the spec band [1.5, 2.6] ----

TEST(PhaseK, NisSingleRunInSpecBand) {
  const RunSamples rs = run_filter(kBaseSeed);
  double sum = 0.0;
  for (const double v : rs.nis) {
    sum += v;
  }
  const double mean_nis = sum / static_cast<double>(rs.nis.size());
  // Spec: mean NIS ~2.0 (measurement dim), acceptance band 1.5-2.6. (Measured ~1.97.)
  EXPECT_TRUE(std::isfinite(mean_nis));
  EXPECT_GT(mean_nis, 1.5);
  EXPECT_LT(mean_nis, 2.6);
}

// ---- AC-K2 — NIS Monte-Carlo ensemble mean, tight consistency gate [1.85, 2.15] ----

TEST(PhaseK, NisEnsembleMeanTight) {
  // NIS ~ chi^2_2, mean 2.0 (measured ~1.97 over the ensemble window). Tight band: a
  // transposed-covariance predict pushes the pooled mean well outside (measured ~4.4).
  // NIS is position-innovation only, so a gain/Q bug barely moves it — that is by
  // design; NEES (AC-K3) and the unit tests carry those.
  const double mean_nis = ensemble().mean_nis;
  EXPECT_GT(mean_nis, 1.85);
  EXPECT_LT(mean_nis, 2.15);
}

// ---- AC-K3 — NEES state consistency (keystone) ----

TEST(PhaseK, NeesEnsembleMeanStateConsistent) {
  // Posterior NEES = e^T P^-1 e ~ chi^2_4, mean 4.0; measured ~3.6 over the window
  // (mild, correct steady-state under-confidence pulls it slightly below 4 — see the
  // file header). The band [3.0, 4.5] keeps the correct filter inside while a lying
  // covariance fails: a transposed predict measures ~96 (>>4.5), a 10x-inflated Q
  // measures ~2.7 (<3.0). This is the check the position-only NIS cannot make — it
  // sees the full 4x4 P including the velocity block.
  const Ensemble& en = ensemble();
  EXPECT_GE(en.min_nees, 0.0);  // non-negative by construction (P SPD)
  EXPECT_GT(en.mean_nees, 3.0);
  EXPECT_LT(en.mean_nees, 4.5);
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

// ---- AC-K5 — Convergence over Monte-Carlo: mean ratio <= 0.5, every run <= 0.6 ----

TEST(PhaseK, ConvergenceOverMonteCarlo) {
  // Spec M1 gate: RMS(est-truth)/RMS(z-truth) over the converged late tail <= 0.5
  // (measured ~0.18). A pass-through (est = z) gives ratio 1.0 and fails; a transposed
  // covariance predict degrades tracking to ~0.76 and also fails. The per-run bound
  // guards against a single pathological run hiding behind a good mean.
  const Ensemble& en = ensemble();
  EXPECT_LT(en.mean_ratio, 0.5);
  EXPECT_LT(en.max_run_ratio, 0.6);
}
