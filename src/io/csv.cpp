#include "radar/io/csv.hpp"

#include <array>
#include <charconv>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <system_error>

#include "radar/types.hpp"  // state-vector indices kPx..kVy

// src/io/csv.cpp — truth/meas CSV writers.
//
// Every numeric field is formatted with std::to_chars, which is locale-INDEPENDENT by
// construction (always a '.' decimal point, never a thousands separator) and allocation-
// free. This fixes two things over an iostream approach: (1) integer columns no longer
// depend on the stream's imbued locale — an id like 1000 can never print as "1,000" and
// split a column, even if the app calls std::locale::global; (2) there is no per-field
// imbue/restore of the stream locale. Time uses 3 decimals (ms at a 0.1 s scan);
// positions/velocities use 6 decimals (sub-micron), enough to round-trip faithfully.

namespace radar {

namespace {

constexpr int kTimePrecision = 3;
constexpr int kValuePrecision = 6;

// Worst-case width of a fixed-notation double: a sign, the integer part (up to
// max_exponent10 + 1 digits for the largest finite magnitude, ~309), the '.', the
// fractional digits (precision), and slack for a terminator. std::to_chars returns
// ec == value_too_large (and leaves the buffer unspecified) if it does not fit, so a
// too-small buffer would otherwise write garbage silently — sizing for the worst case
// plus checking ec (below) closes that (PR #14 re-review). 1.79e308 at 6 dp is ~316
// chars; max_exponent10 (308) + 64 = 372 bytes is comfortable headroom.
constexpr std::size_t kDoubleBufSize =
    static_cast<std::size_t>(std::numeric_limits<double>::max_exponent10) + 64;

// A to_chars buffer. The data()/data()+size() and (r.ptr - data()) forms are the
// canonical std::to_chars API; the pointer arithmetic is bounded by the fixed-size
// std::array.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void put_double(std::ostream& os, double value, int precision) {
  std::array<char, kDoubleBufSize> buf{};
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const std::to_chars_result r = std::to_chars(buf.data(), buf.data() + buf.size(), value,
                                               std::chars_format::fixed, precision);
  if (r.ec != std::errc{}) {
    // Only happens if the buffer is too small, which kDoubleBufSize rules out for any
    // finite double; throwing (rather than writing an unspecified buffer) guarantees a
    // loud failure instead of silently corrupt CSV if this invariant is ever broken.
    throw std::range_error("radar::io: could not format a double for CSV (value too large)");
  }
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  os.write(buf.data(), r.ptr - buf.data());
}

void put_int(std::ostream& os, int value) {
  // 16 chars holds any 32-bit int ("-2147483648" is 11); sized generously.
  std::array<char, 16> buf{};
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const std::to_chars_result r = std::to_chars(buf.data(), buf.data() + buf.size(), value);
  if (r.ec != std::errc{}) {
    throw std::range_error("radar::io: could not format an integer for CSV");
  }
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
