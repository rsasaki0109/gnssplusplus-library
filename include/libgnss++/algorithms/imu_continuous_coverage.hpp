#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace libgnss::imu_coverage {
struct Result { bool ok = false; double maximum_gap_s = 0; std::string error; };
// Times are relative to the first GNSS epoch. Require real paired samples on
// both sides of the complete graph interval; never certify an endpoint hold.
inline Result check(const std::vector<double>& times, double begin, double end,
                    double maximum_gap_s = 0.05) {
    Result r;
    if (times.size() < 2 || !std::isfinite(begin) || !std::isfinite(end) ||
        end <= begin || !std::isfinite(maximum_gap_s) || maximum_gap_s <= 0) {
        r.error = "invalid IMU coverage interval"; return r;
    }
    for (std::size_t i = 0; i < times.size(); ++i) {
        if (!std::isfinite(times[i]) || (i && times[i] <= times[i-1])) {
            r.error = "nonfinite or nonmonotonic mapped IMU time"; return r;
        }
    }
    if (times.front() > begin || times.back() < end) {
        r.error = "mapped IMU does not bracket the complete GNSS interval"; return r;
    }
    for (std::size_t i = 1; i < times.size(); ++i) {
        if (times[i] <= begin || times[i-1] >= end) continue;
        r.maximum_gap_s = std::max(r.maximum_gap_s, times[i] - times[i-1]);
    }
    if (r.maximum_gap_s > maximum_gap_s + 1e-9) {
        r.error = "mapped IMU sample gap exceeds continuity bound"; return r;
    }
    r.ok = true; return r;
}

struct BoundedTailResult {
    bool ok = false;
    bool real_samples_bracket_full_interval = false;
    double maximum_gap_s = 0.0;
    double trailing_measurement_hold_s = 0.0;
    std::string error;
};

// An explicit alternative policy for the existing integrator's terminal
// measurement hold, not a proof of full real-sample bracketing. The recovered
// leading interval must still be covered by real samples through real_end.
// This does not synthesize samples, alter timestamps, or hold output positions.
inline BoundedTailResult checkBoundedTail(
    const std::vector<double>& times, double begin, double end, double real_end,
    double maximum_tail_s = 0.05, double maximum_gap_s = 0.05) {
    BoundedTailResult r;
    if (!std::isfinite(real_end) || !std::isfinite(begin) ||
        !std::isfinite(end) || real_end <= begin || real_end > end ||
        !std::isfinite(maximum_tail_s) || maximum_tail_s < 0.0) {
        r.error = "invalid bounded-tail IMU coverage interval";
        return r;
    }
    // Validate the complete stream and the strictly required real interval
    // first. Missing leading data or a gap there cannot be waived by a tail.
    const auto leading = check(times, begin, real_end, maximum_gap_s);
    if (!leading.ok) { r.error = leading.error; return r; }
    const double supported_end = std::min(end, times.back());
    const auto supported = check(times, begin, supported_end, maximum_gap_s);
    if (!supported.ok) { r.error = supported.error; return r; }
    r.maximum_gap_s = supported.maximum_gap_s;
    r.trailing_measurement_hold_s = std::max(0.0, end - times.back());
    if (r.trailing_measurement_hold_s > maximum_tail_s + 1e-9) {
        r.error = "terminal IMU measurement hold exceeds bound";
        return r;
    }
    r.real_samples_bracket_full_interval = times.back() >= end;
    r.ok = true;
    return r;
}
} // namespace libgnss::imu_coverage
