#pragma once
#include <libgnss++/core/constants.hpp>
#include <cmath>
#include <optional>

namespace libgnss::gps_triple_code {
// Diagnostic only. For P_i = common + (f1/fi)^2 * I + bias_i,
// this combination cancels common and I, not frequency-specific biases.
// Fix the L5 coefficient to one; no fitted coefficients or ambiguity states.
inline double l2Weight() {
    const double g2 = std::pow(constants::GPS_L1_FREQ/constants::GPS_L2_FREQ,2);
    const double g5 = std::pow(constants::GPS_L1_FREQ/constants::GPS_L5_FREQ,2);
    return (1.-g5)/(g2-1.);
}
// TGD-only component for L1 C/A, L2 P(Y), L5. This deliberately excludes
// L1/L5 ISC and receiver/code-specific effects; not a complete correction.
inline std::optional<double> tgdComponent(double tgd_seconds) {
    if (!std::isfinite(tgd_seconds)) return std::nullopt;
    const double g5=std::pow(constants::GPS_L1_FREQ/constants::GPS_L5_FREQ,2);
    const double value=(1.-g5)*constants::SPEED_OF_LIGHT*tgd_seconds;
    if (!std::isfinite(value)) return std::nullopt;
    return value;
}
inline std::optional<double> closure(double p1, double p2, double p5) {
    if (!std::isfinite(p1) || !std::isfinite(p2) || !std::isfinite(p5) ||
        p1<=0. || p2<=0. || p5<=0.) return std::nullopt;
    const double result = l2Weight()*(p2-p1)+(p5-p1);
    if (!std::isfinite(result)) return std::nullopt;
    return result;
}
}  // namespace libgnss::gps_triple_code
