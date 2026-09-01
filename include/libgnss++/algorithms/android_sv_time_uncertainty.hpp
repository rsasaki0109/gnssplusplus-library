#pragma once

#include <libgnss++/core/constants.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace libgnss::android_sv_time_uncertainty {

/**
 * @brief Convert Android's source-domain code timing uncertainty to metres.
 *
 * Android reports ReceivedSvTimeUncertaintyNanos as a one-sigma time-domain
 * quantity.  Keep the conversion literal: no fitted scale, offset, or upper
 * bound is applied.  Invalid/non-positive source values are represented as
 * NaN so callers can retain their existing measurement sigma.
 */
inline double metersFromNanoseconds(double uncertainty_nanos) {
    if (!std::isfinite(uncertainty_nanos) || uncertainty_nanos <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double metres = uncertainty_nanos * constants::SPEED_OF_LIGHT * 1.0e-9;
    return std::isfinite(metres) && metres > 0.0
               ? metres
               : std::numeric_limits<double>::quiet_NaN();
}

/**
 * @brief Apply the opt-in native Android uncertainty floor to one sigma.
 *
 * Existing positive finite sigma is preserved when the option is disabled or
 * when the source field is missing/invalid.  There is intentionally no clip
 * or tuning coefficient.
 */
inline double sigmaWithFloor(double existing_sigma_m,
                             double uncertainty_m,
                             bool enabled) {
    if (!enabled || !std::isfinite(existing_sigma_m) ||
        existing_sigma_m <= 0.0 || !std::isfinite(uncertainty_m) ||
        uncertainty_m <= 0.0) {
        return existing_sigma_m;
    }
    return std::max(existing_sigma_m, uncertainty_m);
}

}  // namespace libgnss::android_sv_time_uncertainty
