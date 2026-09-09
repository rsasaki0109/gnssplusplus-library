#pragma once

/**
 * Small, side-effect-free pieces of the pinned taroz GNSS state contract.
 *
 * The public implementation keeps its historical ECEF correction state and
 * Eigen backend.  This header only exposes the part that can be ported
 * exactly without changing that state layout: the midpoint motion/clock
 * equations use the measured epoch interval, and a temporal edge is valid
 * only under the upstream 1.5 s continuity bound.
 */

#include <cmath>
#include <limits>

namespace libgnss_apps::upstream_state_contract {

constexpr double kTimeDifferenceThresholdSeconds = 1.5;

inline bool validTemporalInterval(double dt_seconds) {
    return std::isfinite(dt_seconds) && dt_seconds > 0.0 &&
           dt_seconds < kTimeDifferenceThresholdSeconds;
}

inline double motionResidual(double position_difference_m,
                             double previous_velocity_mps,
                             double current_velocity_mps,
                             double dt_seconds) {
    return position_difference_m -
           0.5 * (previous_velocity_mps + current_velocity_mps) * dt_seconds;
}

inline double clockResidual(double clock_difference_m,
                            double previous_drift_mps,
                            double current_drift_mps,
                            double dt_seconds) {
    return clock_difference_m -
           0.5 * (previous_drift_mps + current_drift_mps) * dt_seconds;
}

inline double legacyUnitIntervalMotionResidual(double position_difference_m,
                                               double previous_velocity_mps,
                                               double current_velocity_mps) {
    return motionResidual(position_difference_m,
                          previous_velocity_mps,
                          current_velocity_mps,
                          1.0);
}

inline double legacyUnitIntervalClockResidual(double clock_difference_m,
                                              double previous_drift_mps,
                                              double current_drift_mps) {
    return clockResidual(clock_difference_m,
                         previous_drift_mps,
                         current_drift_mps,
                         1.0);
}

}  // namespace libgnss_apps::upstream_state_contract
