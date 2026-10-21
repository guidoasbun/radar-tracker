#include "radar/io/csv.hpp"

#include <locale>
#include <ostream>

#include "radar/types.hpp"  // state-vector indices kPx..kVy

// src/io/csv.cpp — truth/meas CSV writers.
//
// Formatting is deliberately explicit and locale-INDEPENDENT: each numeric field is
// written through a helper that imbues the classic ("C") locale and fixed precision on a
// local copy of the stream's formatting state, so the decimal point is always '.' and the
// output is byte-identical regardless of the host's LC_NUMERIC (a German locale would
// otherwise print "3,14" and corrupt the columns). Time uses 3 decimals (ms at a 0.1 s
// scan); positions/velocities use 6 decimals (sub-micron), enough to round-trip a
// scenario's values faithfully.

namespace radar {

namespace {

constexpr int kTimePrecision = 3;
constexpr int kValuePrecision = 6;

// Write one double with fixed precision under the classic locale, restoring the stream's
// previous formatting afterward so callers' stream state is untouched. value/precision are
// obvious at the few local call sites; a swap is caught by the exact-format tests.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void put_fixed(std::ostream& os, double value, int precision) {
  const std::ios_base::fmtflags flags = os.flags();
  const std::streamsize prec = os.precision();
  const std::locale loc = os.getloc();

  os.imbue(std::locale::classic());
  os.setf(std::ios_base::fixed, std::ios_base::floatfield);
  os.precision(precision);
  os << value;

  os.imbue(loc);
  os.precision(prec);
  os.flags(flags);
}

}  // namespace

void write_truth_header(std::ostream& os) { os << "t_s,target_id,x_m,y_m,vx_mps,vy_mps\n"; }

// t_s (double) and the id (int) are adjacent and implicitly convertible; a swap is caught
// immediately by the exact-format tests (the columns would be visibly wrong). The argument
// order mirrors the CSV column order, which is the natural call form.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void write_truth_row(std::ostream& os, double t_s, int target_id, const Eigen::Vector4d& state) {
  put_fixed(os, t_s, kTimePrecision);
  os << ',' << target_id << ',';
  put_fixed(os, state(kPx), kValuePrecision);
  os << ',';
  put_fixed(os, state(kPy), kValuePrecision);
  os << ',';
  put_fixed(os, state(kVx), kValuePrecision);
  os << ',';
  put_fixed(os, state(kVy), kValuePrecision);
  os << '\n';
}

void write_meas_header(std::ostream& os) { os << "t_s,meas_id,x_m,y_m,origin_id\n"; }

// See write_truth_row: t_s/meas_id swap is caught by the exact-format test.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void write_meas_row(std::ostream& os, double t_s, int meas_id, const Eigen::Vector2d& pos,
                    int origin_id) {
  put_fixed(os, t_s, kTimePrecision);
  os << ',' << meas_id << ',';
  put_fixed(os, pos.x(), kValuePrecision);
  os << ',';
  put_fixed(os, pos.y(), kValuePrecision);
  os << ',' << origin_id << '\n';
}

}  // namespace radar
