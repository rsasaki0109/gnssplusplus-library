#pragma once

/**
 * @file observable_upstream_preprocessing.hpp
 * @brief Pure raw-observable rules ported from taroz/gsdc2023.
 *
 * This header deliberately contains no file, truth, coordinate, or MATLAB
 * dependency.  It is the small, testable part of ``exobs_residuals.m`` and
 * ``obserrmodel.m`` that can be applied to the native Android observations.
 * Base-station pseudorange compensation from ``correct_pseudorange.m`` is
 * intentionally not represented here because its required base observation
 * is outside the native raw+navigation contract.
 */

#include <libgnss++/core/observation.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace libgnss_apps::upstream {

enum class ObservationBand {
    L1,
    L5,
    Unknown,
};

inline ObservationBand bandForSignal(libgnss::SignalType signal) {
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA:
        case libgnss::SignalType::GLO_L1CA:
        case libgnss::SignalType::GAL_E1:
        case libgnss::SignalType::BDS_B1I:
        case libgnss::SignalType::BDS_B1C:
        case libgnss::SignalType::QZS_L1CA:
            return ObservationBand::L1;
        case libgnss::SignalType::GPS_L5:
        case libgnss::SignalType::GAL_E5A:
        case libgnss::SignalType::BDS_B2A:
        case libgnss::SignalType::QZS_L5:
            return ObservationBand::L5;
        default:
            return ObservationBand::Unknown;
    }
}

inline bool isL1(libgnss::SignalType signal) {
    return bandForSignal(signal) == ObservationBand::L1;
}

inline bool isL5(libgnss::SignalType signal) {
    return bandForSignal(signal) == ObservationBand::L5;
}

/**
 * Signal multiplier from sysfreq2sigtype.m/parameters.m:
 * L1, G1, E1, B1, L5, E5, B2a = .8, 1.5, .8, .8, .5, .5, .5.
 */
inline double signalTypeFactor(libgnss::SignalType signal) {
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA:
        case libgnss::SignalType::GAL_E1:
        case libgnss::SignalType::BDS_B1I:
        case libgnss::SignalType::BDS_B1C:
            return 0.8;
        case libgnss::SignalType::GLO_L1CA:
            return 1.5;
        case libgnss::SignalType::GPS_L5:
        case libgnss::SignalType::GAL_E5A:
        case libgnss::SignalType::BDS_B2A:
            return 0.5;
        default:
            return std::numeric_limits<double>::quiet_NaN();
    }
}

/**
 * MATLAB ``prctile`` interpolation used by prctile(S,85,"all").
 *
 * MATLAB places the percentile at ``0.5 + n*p/100`` (the midpoint-of-order
 * statistics convention), with endpoint clipping.  This is intentionally
 * different from NumPy's default ``(n-1)*p`` convention; retaining the
 * distinction matters for the small synthetic fixtures and for sparse
 * signal-family observations.
 */
inline double linearPercentile(std::vector<double> values, double percentile) {
    values.erase(std::remove_if(values.begin(), values.end(),
                                [](double value) {
                                    return !std::isfinite(value);
                                }),
                  values.end());
    if (values.empty() || !std::isfinite(percentile) || percentile < 0.0 ||
        percentile > 100.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(values.begin(), values.end());
    if (values.size() == 1U) return values.front();
    const double rank = 0.5 + percentile / 100.0 *
                                  static_cast<double>(values.size());
    if (rank <= 1.0) return values.front();
    if (rank >= static_cast<double>(values.size())) return values.back();
    const double lower_rank = std::floor(rank);
    const std::size_t lower = static_cast<std::size_t>(lower_rank - 1.0);
    const std::size_t upper = lower + 1U;
    const double fraction = rank - lower_rank;
    return values[lower] + fraction * (values[upper] - values[lower]);
}

inline double snrScale(double snr_dbhz, double percentile85_dbhz) {
    if (!std::isfinite(snr_dbhz) || !std::isfinite(percentile85_dbhz)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::pow(10.0, -(snr_dbhz - percentile85_dbhz) / 20.0);
}

struct SnrPercentiles {
    double l1_dbhz = std::numeric_limits<double>::quiet_NaN();
    double l5_dbhz = std::numeric_limits<double>::quiet_NaN();

    double forBand(ObservationBand band) const {
        return band == ObservationBand::L1 ? l1_dbhz
             : band == ObservationBand::L5 ? l5_dbhz
                                            : std::numeric_limits<double>::quiet_NaN();
    }
};

inline double snrPercentileSigma(libgnss::SignalType signal,
                                 double snr_dbhz,
                                 const SnrPercentiles& percentiles,
                                 char observable) {
    const double factor = signalTypeFactor(signal);
    const double scale = snrScale(snr_dbhz, percentiles.forBand(bandForSignal(signal)));
    if (!std::isfinite(scale)) return std::numeric_limits<double>::quiet_NaN();
    switch (observable) {
        case 'P':
            return scale * factor;       // P_sn_ratio = 1
        case 'D':
            return scale / 12.0;         // D_sn_ratio = 1/12
        case 'L':
            return scale * factor / 400.0;  // L_sn_ratio = 1/400
        default:
            return std::numeric_limits<double>::quiet_NaN();
    }
}

inline bool finitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

/** Exact dDP expression in exobs_residuals.m, with D in cycles/second. */
inline double pseudorangeDopplerDifference(double previous_pseudorange_m,
                                           double current_pseudorange_m,
                                           double previous_doppler_cycles_s,
                                           double current_doppler_cycles_s,
                                           double wavelength_m,
                                           double dt_s) {
    return -(previous_doppler_cycles_s + current_doppler_cycles_s) *
               wavelength_m * 0.5 * dt_s -
           (current_pseudorange_m - previous_pseudorange_m);
}

/** Exact dDL expression in exobs_residuals.m, with L in cycles. */
inline double carrierDopplerDifference(double previous_carrier_cycles,
                                       double current_carrier_cycles,
                                       double previous_doppler_cycles_s,
                                       double current_doppler_cycles_s,
                                       double wavelength_m,
                                       double dt_s,
                                       double carrier_offset_m = 0.0) {
    return -(previous_doppler_cycles_s + current_doppler_cycles_s) *
               wavelength_m * 0.5 * dt_s -
           (current_carrier_cycles - previous_carrier_cycles) * wavelength_m -
           carrier_offset_m;
}

inline double residualThreshold(ObservationBand band, char observable) {
    switch (observable) {
        case 'P':
            return band == ObservationBand::L1 ? 20.0 : 15.0;
        case 'D':
            return 3.0;
        case 'L':
            return 1.5;
        default:
            return std::numeric_limits<double>::quiet_NaN();
    }
}

inline double pairThreshold(ObservationBand band, char pair_kind) {
    switch (pair_kind) {
        case 'P':  // P-D difference
            return band == ObservationBand::L1 ? 40.0 : 20.0;
        case 'L':  // L-D difference
            return 1.5;
        default:
            return std::numeric_limits<double>::quiet_NaN();
    }
}

}  // namespace libgnss_apps::upstream
