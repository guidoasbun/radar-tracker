// tests/test_io.cpp — unit tests for the io module (scenario config + CSV writers).
//
// The weight is on the issue-#5 validation battery: io is the untrusted-input boundary,
// so every malformed/hostile scenario must fail LOUDLY with a ConfigError (not a crash,
// not a silent NaN trajectory). These are runtime throws, so the EXPECT_THROW cases run
// in every build (ci/ci-clang included), not only where asserts are live. The CSV tests
// pin the exact spec format and locale-independence.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <sstream>
#include <string>

#include "radar/io/config.hpp"
#include "radar/io/csv.hpp"
#include "radar/types.hpp"

namespace {

// A minimal valid single-target scenario as a literal, so the validation battery can
// mutate one field at a time without temp files.
const char* const kValidScenario = R"({
  "dt_s": 0.1,
  "duration_s": 60,
  "seed": 42,
  "area_half_width_m": 20000,
  "sensor": { "sigma_m": 50.0, "p_detect": 0.9, "clutter_per_scan": 2.0 },
  "tracker": { "sigma_a": 5.0, "gate_chi2": 9.21, "confirm_m": 3, "confirm_n": 5, "delete_after_misses": 5 },
  "targets": [
    { "id": 1, "x0": -15000, "y0": -12000, "vx0": 250, "vy0": 180, "maneuvers": [] }
  ]
})";

}  // namespace

// ---- AC-1 — valid scenario parses to the expected struct (vs literals) ------

TEST(IoConfig, ValidScenarioParsesToExpectedStruct) {
  const radar::ScenarioConfig c = radar::parse_scenario(kValidScenario, "valid");

  EXPECT_DOUBLE_EQ(c.dt_s, 0.1);
  EXPECT_DOUBLE_EQ(c.duration_s, 60.0);
  EXPECT_EQ(c.seed, 42U);
  EXPECT_DOUBLE_EQ(c.area_half_width_m, 20000.0);

  EXPECT_DOUBLE_EQ(c.sensor.sigma_m, 50.0);
  EXPECT_DOUBLE_EQ(c.sensor.p_detect, 0.9);
  EXPECT_DOUBLE_EQ(c.sensor.clutter_per_scan, 2.0);

  EXPECT_DOUBLE_EQ(c.tracker.sigma_a, 5.0);
  EXPECT_DOUBLE_EQ(c.tracker.gate_chi2, 9.21);
  EXPECT_EQ(c.tracker.confirm_m, 3);
  EXPECT_EQ(c.tracker.confirm_n, 5);
  EXPECT_EQ(c.tracker.delete_after_misses, 5);

  ASSERT_EQ(c.targets.size(), 1U);
  EXPECT_EQ(c.targets[0].id, 1);
  EXPECT_DOUBLE_EQ(c.targets[0].x0, -15000.0);
  EXPECT_DOUBLE_EQ(c.targets[0].y0, -12000.0);
  EXPECT_DOUBLE_EQ(c.targets[0].vx0, 250.0);
  EXPECT_DOUBLE_EQ(c.targets[0].vy0, 180.0);
  EXPECT_TRUE(c.targets[0].maneuvers.empty());
}

TEST(IoConfig, OptionalBlocksFallBackToDefaults) {
  // Only the required fields present; sensor/tracker/seed absent -> defaults.
  const char* const minimal =
      R"({ "dt_s": 0.1, "targets": [ { "id": 0, "x0": 0, "y0": 0, "vx0": 1, "vy0": 2 } ] })";
  const radar::ScenarioConfig c = radar::parse_scenario(minimal, "minimal");
  EXPECT_DOUBLE_EQ(c.sensor.sigma_m, 50.0);  // struct default
  EXPECT_DOUBLE_EQ(c.tracker.sigma_a, 5.0);  // struct default
  EXPECT_EQ(c.seed, 42U);                    // struct default
  ASSERT_EQ(c.targets.size(), 1U);
  EXPECT_TRUE(c.targets[0].maneuvers.empty());
}

// ---- AC-2 — issue #5 validation battery (each throws ConfigError) -----------

TEST(IoConfig, RejectsNonFiniteOrNonPositiveDt) {
  EXPECT_THROW(
      (void)radar::parse_scenario(
          R"({ "dt_s": 0.0, "targets": [ { "id": 0, "x0": 0, "y0": 0, "vx0": 0, "vy0": 0 } ] })",
          "s"),
      radar::ConfigError);
  EXPECT_THROW(
      (void)radar::parse_scenario(
          R"({ "dt_s": -0.1, "targets": [ { "id": 0, "x0": 0, "y0": 0, "vx0": 0, "vy0": 0 } ] })",
          "s"),
      radar::ConfigError);
  // 1e400 overflows a double; nlohmann rejects it at parse time (out_of_range),
  // which parse_scenario turns into a ConfigError. Either way: rejected, not stored.
  EXPECT_THROW(
      (void)radar::parse_scenario(
          R"({ "dt_s": 1e400, "targets": [ { "id": 0, "x0": 0, "y0": 0, "vx0": 0, "vy0": 0 } ] })",
          "s"),
      radar::ConfigError);
}

TEST(IoConfig, RejectsNonFiniteTargetState) {
  // 1e400 overflows -> rejected at parse (out_of_range) -> ConfigError. (The field-level
  // std::isfinite guard in req_finite is defense-in-depth for any future non-finite path;
  // standard JSON cannot carry NaN/inf tokens or an overflowing literal past the parser.)
  EXPECT_THROW(
      (void)radar::parse_scenario(
          R"({ "dt_s": 0.1, "targets": [ { "id": 0, "x0": 1e400, "y0": 0, "vx0": 0, "vy0": 0 } ] })",
          "s"),
      radar::ConfigError);
  // overflow vy0
  EXPECT_THROW(
      (void)radar::parse_scenario(
          R"({ "dt_s": 0.1, "targets": [ { "id": 0, "x0": 0, "y0": 0, "vx0": 0, "vy0": 1e400 } ] })",
          "s"),
      radar::ConfigError);
}

TEST(IoConfig, RejectsBadDurationOrArea) {
  EXPECT_THROW(
      (void)radar::parse_scenario(
          R"({ "dt_s": 0.1, "duration_s": 0, "targets": [ { "id": 0, "x0": 0, "y0": 0, "vx0": 0, "vy0": 0 } ] })",
          "s"),
      radar::ConfigError);
  EXPECT_THROW(
      (void)radar::parse_scenario(
          R"({ "dt_s": 0.1, "area_half_width_m": -1, "targets": [ { "id": 0, "x0": 0, "y0": 0, "vx0": 0, "vy0": 0 } ] })",
          "s"),
      radar::ConfigError);
}

TEST(IoConfig, RejectsNegativeOrNonFiniteSigmaM) {
  EXPECT_THROW(
      (void)radar::parse_scenario(
          R"({ "dt_s": 0.1, "sensor": { "sigma_m": -1 }, "targets": [ { "id": 0, "x0": 0, "y0": 0, "vx0": 0, "vy0": 0 } ] })",
          "s"),
      radar::ConfigError);
  EXPECT_THROW(
      (void)radar::parse_scenario(
          R"({ "dt_s": 0.1, "sensor": { "sigma_m": 1e400 }, "targets": [ { "id": 0, "x0": 0, "y0": 0, "vx0": 0, "vy0": 0 } ] })",
          "s"),
      radar::ConfigError);  // overflow -> parse-time reject
}

TEST(IoConfig, RejectsNegativeTargetId) {
  // id = -1 collides with the clutter sentinel.
  EXPECT_THROW(
      (void)radar::parse_scenario(
          R"({ "dt_s": 0.1, "targets": [ { "id": -1, "x0": 0, "y0": 0, "vx0": 0, "vy0": 0 } ] })",
          "s"),
      radar::ConfigError);
}

TEST(IoConfig, RejectsDuplicateTargetIds) {
  EXPECT_THROW(
      (void)radar::parse_scenario(
          R"({ "dt_s": 0.1, "targets": [ { "id": 2, "x0": 0, "y0": 0, "vx0": 0, "vy0": 0 }, { "id": 2, "x0": 1, "y0": 1, "vx0": 0, "vy0": 0 } ] })",
          "s"),
      radar::ConfigError);
}

TEST(IoConfig, RejectsNonEmptyManeuvers) {
  // CT is deferred in M1; a non-empty maneuver list must be rejected.
  EXPECT_THROW(
      (void)radar::parse_scenario(
          R"({ "dt_s": 0.1, "targets": [ { "id": 0, "x0": 0, "y0": 0, "vx0": 0, "vy0": 0, "maneuvers": [ { "t_start_s": 1, "t_end_s": 2, "turn_rate_deg_s": 1 } ] } ] })",
          "s"),
      radar::ConfigError);
}

// ---- AC-3 — malformed / missing input is a clear ConfigError ----------------

TEST(IoConfig, RejectsInvalidJsonSyntax) {
  EXPECT_THROW((void)radar::parse_scenario("{ this is not json", "s"), radar::ConfigError);
}

TEST(IoConfig, RejectsMissingRequiredKey) {
  // dt_s missing.
  EXPECT_THROW((void)radar::parse_scenario(
                   R"({ "targets": [ { "id": 0, "x0": 0, "y0": 0, "vx0": 0, "vy0": 0 } ] })", "s"),
               radar::ConfigError);
  // targets missing.
  EXPECT_THROW((void)radar::parse_scenario(R"({ "dt_s": 0.1 })", "s"), radar::ConfigError);
}

TEST(IoConfig, RejectsWrongType) {
  // dt_s is a string.
  EXPECT_THROW(
      (void)radar::parse_scenario(
          R"({ "dt_s": "fast", "targets": [ { "id": 0, "x0": 0, "y0": 0, "vx0": 0, "vy0": 0 } ] })",
          "s"),
      radar::ConfigError);
  // targets is an object, not an array.
  EXPECT_THROW((void)radar::parse_scenario(R"({ "dt_s": 0.1, "targets": {} })", "s"),
               radar::ConfigError);
}

TEST(IoConfig, ErrorMessageNamesTheSource) {
  try {
    (void)radar::parse_scenario(R"({ "dt_s": -1, "targets": [] })", "myscenario.json");
    FAIL() << "expected ConfigError";
  } catch (const radar::ConfigError& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("myscenario.json"), std::string::npos) << msg;
    EXPECT_NE(msg.find("dt_s"), std::string::npos) << msg;
  }
}

TEST(IoConfig, MissingFileThrows) {
  EXPECT_THROW((void)radar::load_scenario("does/not/exist.json"), radar::ConfigError);
}

// ---- AC-4 — CSV writers produce the exact spec format (+ round-trip) --------

TEST(IoCsv, TruthHeaderAndRowExactFormat) {
  std::ostringstream os;
  radar::write_truth_header(os);
  radar::write_truth_row(os, 12.3, 7, Eigen::Vector4d{-3250.41, -9873.12, 248.9, 181.3});
  EXPECT_EQ(os.str(),
            "t_s,target_id,x_m,y_m,vx_mps,vy_mps\n"
            "12.300,7,-3250.410000,-9873.120000,248.900000,181.300000\n");
}

TEST(IoCsv, MeasHeaderAndRowExactFormat) {
  std::ostringstream os;
  radar::write_meas_header(os);
  radar::write_meas_row(os, 0.1, 3, Eigen::Vector2d{10.5, -20.25}, -1);
  EXPECT_EQ(os.str(),
            "t_s,meas_id,x_m,y_m,origin_id\n"
            "0.100,3,10.500000,-20.250000,-1\n");
}

TEST(IoCsv, ValuesRoundTripToTolerance) {
  // A written value re-parsed as a double recovers it within the 6-dp precision.
  std::ostringstream os;
  const double x = -15000.123456;
  radar::write_meas_row(os, 1.0, 0, Eigen::Vector2d{x, 0.0}, 5);
  // Column 3 (0-based 2) is x_m.
  const std::string row = os.str();
  const std::size_t c1 = row.find(',');
  const std::size_t c2 = row.find(',', c1 + 1);
  const std::size_t c3 = row.find(',', c2 + 1);
  const std::string x_field = row.substr(c2 + 1, c3 - c2 - 1);
  EXPECT_NEAR(std::stod(x_field), x, 1e-6);
}

TEST(IoCsv, WriterRestoresStreamState) {
  // put_fixed must not leave the stream in fixed/precision mode for later output.
  std::ostringstream os;
  os << std::scientific;
  const auto flags_before = os.flags();
  radar::write_meas_row(os, 1.0, 0, Eigen::Vector2d{1.0, 2.0}, 0);
  EXPECT_EQ(os.flags(), flags_before);
}

// ---- AC — the shipped scenarios/single.json loads and validates -------------

TEST(IoConfig, SingleScenarioFileLoads) {
  const std::filesystem::path path = std::filesystem::path(RADAR_SCENARIOS_DIR) / "single.json";
  const radar::ScenarioConfig c = radar::load_scenario(path);
  EXPECT_DOUBLE_EQ(c.dt_s, 0.1);
  ASSERT_EQ(c.targets.size(), 1U);
  EXPECT_EQ(c.targets[0].id, 1);
  EXPECT_TRUE(c.targets[0].maneuvers.empty());
}
