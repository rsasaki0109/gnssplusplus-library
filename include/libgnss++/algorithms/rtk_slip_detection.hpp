#pragma once

#include <cmath>

namespace libgnss::rtk_slip_detection {

inline bool detectDopplerSlip(double previous_sd_phase_m,
                              double current_sd_phase_m,
                              double sd_range_rate_mps,
                              double dt_s,
                              double threshold_m) {
    if (!std::isfinite(previous_sd_phase_m) ||
        !std::isfinite(current_sd_phase_m) ||
        !std::isfinite(sd_range_rate_mps) ||
        !std::isfinite(dt_s) ||
        !std::isfinite(threshold_m) ||
        dt_s <= 0.0 ||
        threshold_m <= 0.0) {
        return false;
    }
    const double observed_delta_m = current_sd_phase_m - previous_sd_phase_m;
    const double predicted_delta_m = sd_range_rate_mps * dt_s;
    return std::abs(observed_delta_m - predicted_delta_m) > threshold_m;
}

inline double singleDifferenceRangeRateMps(double rover_doppler_hz,
                                           double base_doppler_hz,
                                           double wavelength_m) {
    if (!std::isfinite(rover_doppler_hz) ||
        !std::isfinite(base_doppler_hz) ||
        !std::isfinite(wavelength_m) ||
        wavelength_m <= 0.0) {
        return 0.0;
    }
    return -(rover_doppler_hz - base_doppler_hz) * wavelength_m;
}

inline double singleDifferenceCodeMinusPhaseM(double rover_code_m,
                                              double base_code_m,
                                              double rover_phase_cycles,
                                              double base_phase_cycles,
                                              double wavelength_m) {
    if (!std::isfinite(rover_code_m) ||
        !std::isfinite(base_code_m) ||
        !std::isfinite(rover_phase_cycles) ||
        !std::isfinite(base_phase_cycles) ||
        !std::isfinite(wavelength_m) ||
        wavelength_m <= 0.0) {
        return 0.0;
    }
    const double sd_code_m = rover_code_m - base_code_m;
    const double sd_phase_m = (rover_phase_cycles - base_phase_cycles) * wavelength_m;
    return sd_code_m - sd_phase_m;
}

inline bool detectCodeSlip(double previous_code_minus_phase_m,
                           double current_code_minus_phase_m,
                           double threshold_m) {
    if (!std::isfinite(previous_code_minus_phase_m) ||
        !std::isfinite(current_code_minus_phase_m) ||
        !std::isfinite(threshold_m) ||
        threshold_m <= 0.0) {
        return false;
    }
    return std::abs(current_code_minus_phase_m - previous_code_minus_phase_m) > threshold_m;
}

}  // namespace libgnss::rtk_slip_detection
