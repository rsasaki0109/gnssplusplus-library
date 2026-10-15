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

// ---------------------------------------------------------------------------
// Doppler slip test when the rover and the base advanced over different spans.
//
// The single-difference phase is rover(t_rover) - base(t_base). Between two
// detector calls it changes by
//     rover_range_rate * rover_span - base_range_rate * base_span,
// each receiver's range rate being -wavelength * Doppler. The legacy
// prediction, single-difference range rate * rover_span, equals that only when
// both spans are the same. They differ when the base epochs are not
// time-matched with the rover epochs and the two kinds alternate: a base epoch
// that keeps its own measurement time (a millisecond before the rover epoch)
// followed by one interpolated to the rover time advances 0.199 s / 0.201 s
// while the rover advances 0.2 s, about one cycle of false slip at a Doppler of
// 1 kHz.
// ---------------------------------------------------------------------------

// Spans (s) closer than this are the same interval. The legacy expression is
// used for them, so time-matched data keeps bit-identical results.
inline constexpr double kDopplerSpanMatchToleranceS = 1.0e-6;

// True when the base span is known and differs from the rover span by more than
// kDopplerSpanMatchToleranceS. An unknown base span (first call, not finite, not
// positive) is never "different": the legacy prediction applies.
inline bool dopplerSpansDiffer(double rover_span_s, double base_span_s) {
    return std::isfinite(rover_span_s) &&
           std::isfinite(base_span_s) &&
           base_span_s > 0.0 &&
           std::abs(rover_span_s - base_span_s) > kDopplerSpanMatchToleranceS;
}

// Predicted single-difference phase change (m) between two detector calls.
inline double predictedSingleDifferencePhaseChangeM(double rover_doppler_hz,
                                                    double base_doppler_hz,
                                                    double wavelength_m,
                                                    double rover_span_s,
                                                    double base_span_s) {
    if (!dopplerSpansDiffer(rover_span_s, base_span_s)) {
        // The very expression detectDopplerSlip() evaluates.
        return singleDifferenceRangeRateMps(rover_doppler_hz, base_doppler_hz, wavelength_m) *
               rover_span_s;
    }
    if (!std::isfinite(rover_doppler_hz) ||
        !std::isfinite(base_doppler_hz) ||
        !std::isfinite(wavelength_m) ||
        wavelength_m <= 0.0) {
        // No usable Doppler: the same convention as singleDifferenceRangeRateMps().
        return 0.0;
    }
    const double rover_range_rate_mps = -rover_doppler_hz * wavelength_m;
    const double base_range_rate_mps = -base_doppler_hz * wavelength_m;
    return rover_range_rate_mps * rover_span_s - base_range_rate_mps * base_span_s;
}

// detectDopplerSlip() with the rover and the base advancing over their own
// spans. When the spans match (or the base span is unknown) this calls
// detectDopplerSlip() with the legacy arguments, so the decision is the same
// in every bit.
inline bool detectDopplerSlipWithBaseSpan(double previous_sd_phase_m,
                                          double current_sd_phase_m,
                                          double rover_doppler_hz,
                                          double base_doppler_hz,
                                          double wavelength_m,
                                          double rover_span_s,
                                          double base_span_s,
                                          double threshold_m) {
    if (!dopplerSpansDiffer(rover_span_s, base_span_s)) {
        return detectDopplerSlip(previous_sd_phase_m,
                                 current_sd_phase_m,
                                 singleDifferenceRangeRateMps(
                                     rover_doppler_hz, base_doppler_hz, wavelength_m),
                                 rover_span_s,
                                 threshold_m);
    }
    // Same input guards as detectDopplerSlip(); the spans are finite here.
    if (!std::isfinite(previous_sd_phase_m) ||
        !std::isfinite(current_sd_phase_m) ||
        !std::isfinite(threshold_m) ||
        rover_span_s <= 0.0 ||
        threshold_m <= 0.0) {
        return false;
    }
    const double observed_delta_m = current_sd_phase_m - previous_sd_phase_m;
    const double predicted_delta_m = predictedSingleDifferencePhaseChangeM(
        rover_doppler_hz, base_doppler_hz, wavelength_m, rover_span_s, base_span_s);
    return std::abs(observed_delta_m - predicted_delta_m) > threshold_m;
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
