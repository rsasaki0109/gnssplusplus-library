#pragma once

#include <cmath>
#include <limits>

namespace libgnss::cn0_doppler_calibration {

// Fixed by the Phase58 four-route leave-one-route-out audit.  This is a
// source-level constant, not a runtime tuning knob: changing it requires a
// new frozen audit/implementation stage.
constexpr double kReferenceCn0DbHz = 40.0;
constexpr double kAlphaMpsAtReference = 0.7586783350728457;

/**
 * @brief Evaluate the fixed source-supported C/N0 Doppler sigma shape.
 *
 * The 20 dB denominator is the same source-supported monotonic shape used by
 * the existing upstream SNR contract.  This helper does not reproduce that
 * contract's p85 scale or /12 factor; it evaluates only the separately frozen
 * Phase58 closure-residual scale.
 */
inline double modelSigmaMps(double cn0_dbhz) {
    if (!std::isfinite(cn0_dbhz) || cn0_dbhz <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double sigma = kAlphaMpsAtReference * std::pow(
        10.0, -(cn0_dbhz - kReferenceCn0DbHz) / 20.0);
    return std::isfinite(sigma) && sigma > 0.0
               ? sigma
               : std::numeric_limits<double>::quiet_NaN();
}

/**
 * @brief Apply the fixed C/N0 sigma floor to one existing Doppler sigma.
 *
 * Missing/non-finite/non-positive C/N0 falls back to the existing sigma
 * exactly.  The option is explicit so disabled callers retain the historical
 * byte-compatible graph.  No upper clip or coefficient is applied.
 */
inline double sigmaWithFloor(double existing_sigma_mps,
                             double cn0_dbhz,
                             bool enabled) {
    if (!enabled || !std::isfinite(existing_sigma_mps) ||
        existing_sigma_mps <= 0.0) {
        return existing_sigma_mps;
    }
    const double model_sigma_mps = modelSigmaMps(cn0_dbhz);
    if (!std::isfinite(model_sigma_mps) || model_sigma_mps <= 0.0) {
        return existing_sigma_mps;
    }
    return existing_sigma_mps > model_sigma_mps ? existing_sigma_mps
                                                 : model_sigma_mps;
}

}  // namespace libgnss::cn0_doppler_calibration
