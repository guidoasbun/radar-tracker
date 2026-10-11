#ifndef RADAR_VERSION_HPP
#define RADAR_VERSION_HPP

#include <string_view>

/// Radar Tracker public API root namespace.
namespace radar {

/// Returns the library version string (semantic version, e.g. "0.0.1").
/// Units: none. Phase 0 scaffold symbol used to exercise the build+link+test path.
[[nodiscard]] std::string_view version_string() noexcept;

}  // namespace radar

#endif  // RADAR_VERSION_HPP
