#pragma once

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_utils.hpp>
#include <libgnss++/core/constants.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace libgnss::ppp_internal {

inline constexpr double kDefaultZenithDelayMeters = 2.3;

inline int filterIterationCount(bool madoca_per_frequency_update,
                                bool precise_products_loaded,
                                int configured_iterations) {
    if (madoca_per_frequency_update) {
        return 1;
    }
    return precise_products_loaded ? 3 : configured_iterations;
}

inline int perFrequencyArMinLockCount(bool madoca_per_frequency,
                                      bool ssr_products_loaded,
                                      int convergence_min_epochs) {
    // MADOCALIB gen_sat_sd() admits every currently valid phase pair.  Its
    // ambiguity search has no separate lock-count gate in coherent SSR mode.
    if (madoca_per_frequency) {
        return 0;
    }
    return ssr_products_loaded
        ? std::min(convergence_min_epochs, 10)
        : convergence_min_epochs;
}

inline bool applyGpsL5MeasurementErrorFactor(
    bool madoca_per_frequency,
    SignalType primary_signal,
    SignalType secondary_signal) {
    const auto is_l5 = [](SignalType signal) {
        return signal == SignalType::GPS_L5 || signal == SignalType::QZS_L5;
    };
    // The generic RTKLIB-compatible path de-weights L5. MADOCALIB's
    // ppp.c::varerr() does not: its per-frequency profile applies the same
    // elevation/system variance to L1, L2/L5, and the additional bands.
    return !madoca_per_frequency &&
           (is_l5(primary_signal) || is_l5(secondary_signal));
}

inline bool alwaysRestoreArTrialState(PPPProcessor::PPPConfig::ARMethod method) {
    // MADOCALIB runs per-frequency EWL/WL/N1 constraints on xp/Pp, a copy of
    // the float filter.  The trial is never committed to rtk->x/P, including
    // early exits after EWL conditioning but before a usable WL set exists.
    return method == PPPProcessor::PPPConfig::ARMethod::DD_PER_FREQ;
}

inline Vector3d recenterPostfitReceiverPosition(
    const Vector3d& corrected_receiver_position,
    const Vector3d& prior_filter_position,
    const Vector3d& updated_filter_position) {
    // Precise corrections materialize the antenna phase-centre position at
    // the epoch prior.  Only absolute positions should be recentered; a zero
    // vector means the measurement model must obtain its receiver position
    // from the filter state.
    if (corrected_receiver_position.norm() <= 1000.0) {
        return corrected_receiver_position;
    }
    return corrected_receiver_position +
           (updated_filter_position - prior_filter_position);
}

inline double madocaIonosphereScale(double frequency_hz) {
    if (!(frequency_hz > 0.0)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double ratio = constants::GPS_L1_FREQ / frequency_hz;
    return ratio * ratio;
}

inline double madocaIonosphereStateFromPrimaryMeters(
    double primary_frequency_ionosphere_m,
    double primary_frequency_hz) {
    const double primary_scale = madocaIonosphereScale(primary_frequency_hz);
    if (!std::isfinite(primary_scale) || !(primary_scale > 0.0)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return primary_frequency_ionosphere_m / primary_scale;
}

inline double madocaCorrectedCodeIonosphereStateMeters(
    double fallback_primary_ionosphere_m,
    double corrected_primary_code_m,
    double corrected_secondary_code_m,
    double primary_frequency_hz,
    double secondary_frequency_hz) {
    double primary_ionosphere_m = fallback_primary_ionosphere_m;
    if (primary_frequency_hz > 0.0 &&
        secondary_frequency_hz > 0.0 &&
        std::isfinite(corrected_primary_code_m) &&
        std::isfinite(corrected_secondary_code_m)) {
        const double ratio = primary_frequency_hz / secondary_frequency_hz;
        const double denominator = 1.0 - ratio * ratio;
        if (std::abs(denominator) > 1e-6) {
            primary_ionosphere_m =
                (corrected_primary_code_m - corrected_secondary_code_m) /
                denominator;
        }
    }
    return madocaIonosphereStateFromPrimaryMeters(
        primary_ionosphere_m, primary_frequency_hz);
}

inline double madocaCarrierIonosphereMeters(double phase_l1_m,
                                            double phase_l2_m,
                                            double frequency_l1_hz,
                                            double frequency_l2_hz) {
    const double scale_l1 = madocaIonosphereScale(frequency_l1_hz);
    const double scale_l2 = madocaIonosphereScale(frequency_l2_hz);
    if (!std::isfinite(scale_l1) || !std::isfinite(scale_l2)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double denominator = scale_l1 - scale_l2;
    if (std::abs(denominator) < 1e-12) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    // MADOCALIB udiono_ppp(): the estimated STEC state is referenced to the
    // fixed GPS L1 frequency, including for non-GPS primary signals.
    return -(phase_l1_m - phase_l2_m) / denominator;
}

inline double madocaIonosphereProcessVariance(double zenith_variance_per_second,
                                              double elevation_rad,
                                              double dt_seconds) {
    constexpr double kMinimumElevationRad = 5.0 * M_PI / 180.0;
    const double sin_elevation = std::sin(std::max(elevation_rad,
                                                   kMinimumElevationRad));
    return zenith_variance_per_second * std::abs(dt_seconds) /
           (sin_elevation * sin_elevation);
}

inline double initialTroposphereVariance(bool madoca_per_frequency,
                                         bool broadcast_model,
                                         double configured_variance) {
    // MADOCALIB ppp.c initializes estimated ZTD with VAR_ZTD=SQR(0.12).
    if (madoca_per_frequency) {
        return 0.12 * 0.12;
    }
    return broadcast_model ? configured_variance : 25.0;
}

inline std::string trimCopy(const std::string& text) {
    const auto is_not_space = [](unsigned char ch) {
        return !std::isspace(ch);
    };
    const auto first_it = std::find_if(text.begin(), text.end(), is_not_space);
    if (first_it == text.end()) {
        return "";
    }
    const auto last_it = std::find_if(text.rbegin(), text.rend(), is_not_space).base();
    return std::string(first_it, last_it);
}

inline std::string normalizeAntennaType(const std::string& antenna_type) {
    std::string normalized = trimCopy(antenna_type);
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return normalized;
}

inline bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

inline const char* signalFamilyName(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA: return "GPS_L1CA";
        case SignalType::GPS_L1P: return "GPS_L1P";
        case SignalType::GPS_L2P: return "GPS_L2P";
        case SignalType::GPS_L2C: return "GPS_L2C";
        case SignalType::GPS_L5: return "GPS_L5";
        case SignalType::GLO_L1CA: return "GLO_L1CA";
        case SignalType::GLO_L1P: return "GLO_L1P";
        case SignalType::GLO_L2CA: return "GLO_L2CA";
        case SignalType::GLO_L2P: return "GLO_L2P";
        case SignalType::GAL_E1: return "GAL_E1";
        case SignalType::GAL_E5A: return "GAL_E5A";
        case SignalType::GAL_E5B: return "GAL_E5B";
        case SignalType::GAL_E6: return "GAL_E6";
        case SignalType::BDS_B1I: return "BDS_B1I";
        case SignalType::BDS_B2I: return "BDS_B2I";
        case SignalType::BDS_B3I: return "BDS_B3I";
        case SignalType::BDS_B1C: return "BDS_B1C";
        case SignalType::BDS_B2A: return "BDS_B2A";
        case SignalType::QZS_L1CA: return "QZS_L1CA";
        case SignalType::QZS_L2C: return "QZS_L2C";
        case SignalType::QZS_L5: return "QZS_L5";
        case SignalType::SIGNAL_TYPE_COUNT: return "UNKNOWN";
    }
    return "UNKNOWN";
}

inline std::vector<SignalType> primarySignals(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return {SignalType::GPS_L1CA};
        case GNSSSystem::GLONASS: return {SignalType::GLO_L1CA, SignalType::GLO_L1P};
        case GNSSSystem::Galileo: return {SignalType::GAL_E1};
        case GNSSSystem::BeiDou: return {SignalType::BDS_B1I, SignalType::BDS_B1C};
        case GNSSSystem::QZSS: return {SignalType::QZS_L1CA};
        default: return {};
    }
}

inline std::vector<SignalType> secondarySignals(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return {SignalType::GPS_L2C, SignalType::GPS_L5};
        case GNSSSystem::GLONASS: return {SignalType::GLO_L2CA, SignalType::GLO_L2P};
        case GNSSSystem::Galileo: return {SignalType::GAL_E5A, SignalType::GAL_E5B, SignalType::GAL_E6};
        case GNSSSystem::BeiDou: return {SignalType::BDS_B2I, SignalType::BDS_B2A, SignalType::BDS_B3I};
        case GNSSSystem::QZSS: return {SignalType::QZS_L2C, SignalType::QZS_L5};
        default: return {};
    }
}

inline const Observation* findObservationForSignals(const ObservationData& obs,
                                                    const SatelliteId& sat,
                                                    const std::vector<SignalType>& candidates) {
    for (const auto signal : candidates) {
        const Observation* candidate = obs.getObservation(sat, signal);
        if (candidate == nullptr || !candidate->valid || !candidate->has_pseudorange) {
            continue;
        }
        if (candidate->pseudorange <= 0.0 || !std::isfinite(candidate->pseudorange)) {
            continue;
        }
        return candidate;
    }
    return nullptr;
}

inline const Observation* findCarrierObservationForSignals(const ObservationData& obs,
                                                           const SatelliteId& sat,
                                                           const std::vector<SignalType>& candidates) {
    return ppp_utils::findCarrierObservation(obs, sat, candidates);
}

inline Vector3d ssrRacToEcef(const Vector3d& position_ecef,
                             const Vector3d& velocity_ecef,
                             const Vector3d& rac_correction) {
    return ppp_utils::ssrRacToEcef(position_ecef, velocity_ecef, rac_correction);
}

inline double safeVariance(double variance, double floor_value) {
    if (!std::isfinite(variance) || variance <= 0.0) {
        return floor_value;
    }
    return std::max(variance, floor_value);
}

}  // namespace libgnss::ppp_internal
