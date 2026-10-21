#include "radar/io/csv.hpp"

#include <array>
#include <charconv>
#include <ostream>
#include <system_error>

#include "radar/types.hpp"  // state-vector indices kPx..kVy

// src/io/csv.cpp — truth/meas CSV writers.
//
// Every numeric field is formatted with std::to_chars, which is locale-INDEPENDENT by
// construction (always a '.' decimal point, never a thousands separator) and allocation-
// free. This fixes two things over an iostream approach: (1) integer columns no longer
// depend on the stream's imbued locale — an id like 1000 can never print as "1,000" and
// split a column, even if the app calls std::locale::global; (2) there is no per-field
// imbue/restore of the stream locale (which copies the locale and fires callbacks every
// call). Time uses 3 decimals (ms at a 0.1 s scan); positions/velocities use 6 decimals
// (sub-micron), enough to round-trip a scenario's values faithfully.

namespace radar {

namespace {

constexpr int kTimePrecision = 3;
constexpr int kValuePrecision = 6;

// A to_chars buffer: 64 chars is ample for a fixed-notation double or any 32-bit int.
// The data()/data()+size() and (r.ptr - data()) forms are the canonical std::to_chars
// API; the pointer arithmetic is bounded by the fixed-size std::array and unavoidable here.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void put_double(std::ostream& os, double value, int precision) {
  std::array<char, 64> buf{};
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const std::to_chars_result r = std::to_chars(buf.data(), buf.data() + buf.size(), value,
                                               std::chars_format::fixed, precision);
  // to_chars only fails (ec == value_too_large) if the buffer is too small; 64 chars is
  // always enough for fixed notation at these precisions, so this never trips in practice.
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  os.write(buf.data(), r.ptr - buf.data());
}

void put_int(std::ostream& os, int value) {
  std::array<char, 16> buf{};
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const std::to_chars_result r = std::to_chars(buf.data(), buf.data() + buf.size(), value);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  os.write(buf.data(), r.ptr - buf.data());
}

}  // namespace

void write_truth_header(std::ostream& os) { os << "t_s,target_id,x_m,y_m,vx_mps,vy_mps\n"; }

// t_s/target_id swap is caught by the exact-format test; order mirrors the CSV columns.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void write_truth_row(std::ostream& os, double t_s, int target_id, const Eigen::Vector4d& state) {
  put_double(os, t_s, kTimePrecision);
  os << ',';
  put_int(os, target_id);
  os << ',';
  put_double(os, state(kPx), kValuePrecision);
  os << ',';
  put_double(os, state(kPy), kValuePrecision);
  os << ',';
  put_double(os, state(kVx), kValuePrecision);
  os << ',';
  put_double(os, state(kVy), kValuePrecision);
  os << '\n';
}

void write_meas_header(std::ostream& os) { os << "t_s,meas_id,x_m,y_m,origin_id\n"; }

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): see write_truth_row
void write_meas_row(std::ostream& os, double t_s, int meas_id, const Eigen::Vector2d& pos,
                    int origin_id) {
  put_double(os, t_s, kTimePrecision);
  os << ',';
  put_int(os, meas_id);
  os << ',';
  put_double(os, pos.x(), kValuePrecision);
  os << ',';
  put_double(os, pos.y(), kValuePrecision);
  os << ',';
  put_int(os, origin_id);
  os << '\n';
}

}  // namespace radar
