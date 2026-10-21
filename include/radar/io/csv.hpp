#ifndef RADAR_IO_CSV_HPP
#define RADAR_IO_CSV_HPP

#include <Eigen/Core>
#include <ostream>

namespace radar {

/// CSV writers for the two files radar_sim produces (spec "Data formats"). All output is
/// plain CSV with a header row, SI units, and deterministic fixed-precision formatting
/// that is locale-independent (always a '.' decimal point, regardless of LC_NUMERIC) so a
/// file is byte-identical across hosts. Writers take an std::ostream& so the same code
/// serves a file (the app) and an std::ostringstream (tests). Readers are deferred to
/// when metrics/radar_score need them (M1 writers-only; see io requirements / ADR notes).

/// truth.csv header: t_s,target_id,x_m,y_m,vx_mps,vy_mps
void write_truth_header(std::ostream& os);

/// One truth row: scan time, target id, and the 4-vector state [x, y, vx, vy].
void write_truth_row(std::ostream& os, double t_s, int target_id, const Eigen::Vector4d& state);

/// meas.csv header: t_s,meas_id,x_m,y_m,origin_id
void write_meas_header(std::ostream& os);

/// One measurement row: scan time, a per-file measurement id, the 2D position, and the
/// hidden origin (target id, or -1 for clutter). meas.csv is read only by radar_score /
/// plot.py, never by the tracker.
void write_meas_row(std::ostream& os, double t_s, int meas_id, const Eigen::Vector2d& pos,
                    int origin_id);

}  // namespace radar

#endif  // RADAR_IO_CSV_HPP
