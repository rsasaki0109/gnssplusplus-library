#pragma once

// Truth-free residual-ionosphere contract for the opt-in smartphone FGO lane.
// The ordinary Klobuchar/TGD corrections are applied before a pseudorange row
// reaches the graph.  This header only defines the deterministic coefficient
// for a remaining first-order vertical L1 ionosphere state; it does not read
// a trajectory, truth, or any receiver-specific calibration.

#include <libgnss++/core/constants.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace libgnss::residual_ionosphere {

// A standard thin-shell mapping is used instead of a data-fitted elevation
// curve.  The values are fixed physical constants: a 6371 km earth radius
// and a 350 km single-layer ionosphere shell.  This mapping is finite for all
// positive elevations and is intentionally shared by every signal family.
constexpr double kEarthRadiusM = 6'371'000.0;
constexpr double kShellHeightM = 350'000.0;

inline double mappingFactor(double elevation_rad) {
    if (!std::isfinite(elevation_rad) || elevation_rad <= 0.0 ||
        elevation_rad > 0.5 * 3.14159265358979323846) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double ratio = kEarthRadiusM / (kEarthRadiusM + kShellHeightM);
    const double cosine = std::cos(elevation_rad);
    const double denominator = 1.0 - ratio * ratio * cosine * cosine;
    if (!(denominator > 0.0) || !std::isfinite(denominator)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return 1.0 / std::sqrt(denominator);
}

inline double signalCoefficient(double elevation_rad, double frequency_hz) {
    const double mapping = mappingFactor(elevation_rad);
    if (!std::isfinite(mapping) || !(frequency_hz > 0.0) ||
        !std::isfinite(frequency_hz)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double ratio = constants::GPS_L1_FREQ / frequency_hz;
    const double coefficient = mapping * ratio * ratio;
    return std::isfinite(coefficient) && coefficient > 0.0
               ? coefficient
               : std::numeric_limits<double>::quiet_NaN();
}

inline double randomWalkSigma(double seconds,
                              double sigma_m_per_sqrt_s) {
    if (!std::isfinite(seconds) || seconds <= 0.0 ||
        !std::isfinite(sigma_m_per_sqrt_s) || sigma_m_per_sqrt_s <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return sigma_m_per_sqrt_s * std::sqrt(seconds);
}

inline bool finiteCoefficient(double coefficient) {
    return std::isfinite(coefficient) && coefficient > 0.0;
}

}  // namespace libgnss::residual_ionosphere
