#pragma once

// Pure, truth-free first-order dual-frequency code contract.  The signal-bias
// Phase11 candidate does not form IFLC rows (because the raw paired-code
// separation is large and receiver-dependent), but this helper is kept as an
// executable oracle so any future IFLC path cannot silently get coefficient,
// variance, sign, or pair-eligibility semantics wrong.

#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/types.hpp>

#include <cmath>
#include <limits>
#include <optional>

namespace libgnss::dual_frequency {

enum class PairKind {
    Unsupported,
    GpsL1L5,
    GalileoE1E5a,
};

inline PairKind pairKind(GNSSSystem system, SignalType primary,
                         SignalType secondary) {
    if (system == GNSSSystem::GPS && primary == SignalType::GPS_L1CA &&
        secondary == SignalType::GPS_L5) {
        return PairKind::GpsL1L5;
    }
    if (system == GNSSSystem::Galileo && primary == SignalType::GAL_E1 &&
        secondary == SignalType::GAL_E5A) {
        return PairKind::GalileoE1E5a;
    }
    return PairKind::Unsupported;
}

struct IonosphereFreeCode {
    double alpha = std::numeric_limits<double>::quiet_NaN();
    double beta = std::numeric_limits<double>::quiet_NaN();
    double value_m = std::numeric_limits<double>::quiet_NaN();
    double sigma_m = std::numeric_limits<double>::quiet_NaN();
};

/// Form P_IF = alpha*P1 + beta*P2 and propagate independent code noise.
/// Frequencies are ordered explicitly by the caller (the primary frequency
/// must be higher than the secondary frequency); nonfinite/missing pairs are
/// rejected rather than silently falling back to a single-frequency value.
inline std::optional<IonosphereFreeCode> combine(double p1_m, double sigma1_m,
                                                  double f1_hz, double p2_m,
                                                  double sigma2_m,
                                                  double f2_hz) {
    if (!std::isfinite(p1_m) || !std::isfinite(p2_m) ||
        !std::isfinite(sigma1_m) || !std::isfinite(sigma2_m) ||
        !(sigma1_m > 0.0) || !(sigma2_m > 0.0) ||
        !std::isfinite(f1_hz) || !std::isfinite(f2_hz) ||
        !(f1_hz > f2_hz) || f2_hz <= 0.0) {
        return std::nullopt;
    }
    const double f1_squared = f1_hz * f1_hz;
    const double f2_squared = f2_hz * f2_hz;
    const double denominator = f1_squared - f2_squared;
    if (!std::isfinite(denominator) || denominator <= 0.0) {
        return std::nullopt;
    }
    const double alpha = f1_squared / denominator;
    const double beta = -f2_squared / denominator;
    const double variance = alpha * alpha * sigma1_m * sigma1_m +
                            beta * beta * sigma2_m * sigma2_m;
    const double value = alpha * p1_m + beta * p2_m;
    if (!std::isfinite(alpha) || !std::isfinite(beta) ||
        !std::isfinite(variance) || !(variance > 0.0) ||
        !std::isfinite(value)) {
        return std::nullopt;
    }
    return IonosphereFreeCode{alpha, beta, value, std::sqrt(variance)};
}

}  // namespace libgnss::dual_frequency
