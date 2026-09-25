#pragma once

#include <cmath>
#include <cstdint>

namespace libgnss::native_leading_clock {

// Admission uses the four-second UTC window. Receiver-derived GPS duration
// may straddle that nominal boundary: allow one UTC-key tick (1 ms), plus
// the existing microsecond tolerance for GNSSTime subtraction. Do not round
// or change either timestamp. The app still requires continuous mapped IMU.
inline bool withinSpan(double gps_seconds, std::int64_t utc_milliseconds) {
    constexpr double gps_boundary_tolerance_s = 0.001 + 0.000001;
    return std::isfinite(gps_seconds) && gps_seconds > 0.0 &&
           gps_seconds <= 4.0 + gps_boundary_tolerance_s &&
           utc_milliseconds > 0 && utc_milliseconds <= 4000;
}

}  // namespace libgnss::native_leading_clock
