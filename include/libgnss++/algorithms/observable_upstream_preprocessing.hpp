#pragma once

/**
 * @file observable_upstream_preprocessing.hpp
 * @brief Pure raw-observable rules ported from taroz/gsdc2023.
 *
 * This header deliberately contains no file, truth, coordinate, or MATLAB
 * dependency.  It is the small, reusable part of ``exobs_residuals.m`` and
 * ``obserrmodel.m`` that can be applied to native Android observations.
 * Base-station pseudorange compensation from ``correct_pseudorange.m`` is
 * intentionally not represented here because its required base observation
 * is outside the native raw+navigation contract.
 */

#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/signals.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace libgnss::observable_upstream {

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

/** Signal multiplier from sysfreq2sigtype.m/parameters.m. */
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
 * different from NumPy's default ``(n-1)*p`` convention.
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

inline SnrPercentiles collectSnrPercentiles(
    const std::vector<libgnss::ObservationData>& epochs,
    double percentile = 85.0) {
    std::vector<double> l1;
    std::vector<double> l5;
    for (const auto& epoch : epochs) {
        for (const auto& observation : epoch.observations) {
            const ObservationBand band = bandForSignal(observation.signal);
            if (band == ObservationBand::L1) {
                l1.push_back(observation.snr);
            } else if (band == ObservationBand::L5) {
                l5.push_back(observation.snr);
            }
        }
    }
    return {linearPercentile(std::move(l1), percentile),
            linearPercentile(std::move(l5), percentile)};
}

inline double snrPercentileSigma(libgnss::SignalType signal,
                                 double snr_dbhz,
                                 const SnrPercentiles& percentiles,
                                 char observable) {
    const double factor = signalTypeFactor(signal);
    const double scale = snrScale(snr_dbhz, percentiles.forBand(bandForSignal(signal)));
    if (!std::isfinite(scale)) return std::numeric_limits<double>::quiet_NaN();
    switch (observable) {
        case 'P': return scale * factor;       // P_sn_ratio = 1
        case 'D': return scale / 12.0;         // D_sn_ratio = 1/12
        case 'L': return scale * factor / 400.0;  // L_sn_ratio = 1/400
        default: return std::numeric_limits<double>::quiet_NaN();
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
        case 'P': return band == ObservationBand::L1 ? 20.0 : 15.0;
        case 'D': return 3.0;
        case 'L': return 1.5;
        default: return std::numeric_limits<double>::quiet_NaN();
    }
}

/**
 * Apply the upstream receiver-clock correction to a Doppler residual.
 *
 * ``gnsslog2obs.m`` stores ``dclk`` as c*DriftNanosPerSecond/1e9 in m/s,
 * while ``exobs_residuals.m`` evaluates ``resD-dclk/obs.dt``.  Keeping this
 * small equation in the library makes the unit contract explicit and lets
 * the opt-in native screen be tested without a solver or a coordinate.
 */
inline double dopplerResidualAfterReceiverClock(double residual_mps,
                                                double dclk_mps,
                                                double observation_interval_s) {
    if (!std::isfinite(residual_mps) || !std::isfinite(dclk_mps) ||
        !(observation_interval_s > 0.0) ||
        !std::isfinite(observation_interval_s)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return residual_mps - dclk_mps / observation_interval_s;
}

inline bool acceptsAbsoluteDopplerResidual(double residual_mps,
                                           double dclk_mps,
                                           double observation_interval_s,
                                           double threshold_mps = 3.0) {
    const double corrected = dopplerResidualAfterReceiverClock(
        residual_mps, dclk_mps, observation_interval_s);
    return std::isfinite(corrected) && std::isfinite(threshold_mps) &&
           threshold_mps > 0.0 && std::abs(corrected) <= threshold_mps;
}

inline double pairThreshold(ObservationBand band, char pair_kind) {
    switch (pair_kind) {
        case 'P': return band == ObservationBand::L1 ? 40.0 : 20.0;
        case 'L': return 1.5;
        default: return std::numeric_limits<double>::quiet_NaN();
    }
}

using ObservationKey = std::pair<libgnss::SatelliteId, libgnss::SignalType>;

struct EpochMask {
    std::set<ObservationKey> pseudorange;
    std::set<ObservationKey> carrier;
};

inline bool carrierSignOffsetDevice(const std::string& device_model) {
    return device_model == "sm-a205u" || device_model == "sm-a217m" ||
           device_model == "sm-a505g" || device_model == "sm-a600t" ||
           device_model == "sm-a505u";
}

inline bool pseudorangePairSkipDevice(const std::string& device_model) {
    return device_model == "sm-a205u" || device_model == "sm-a505u";
}

/**
 * Build the two-sided raw adjacent P-D/L-D rejection masks.  The masks are
 * keyed by the original raw epoch and observation identity, so callers can
 * apply them before factor construction without changing timestamps or
 * synthesizing measurements.  The equations and 1.5 s continuity bound are
 * the immutable upstream residual contract; invalid/unsupported pairs are
 * simply not eligible for a rejection.
 */
inline void applyAdjacentMasks(
    const std::vector<libgnss::ObservationData>& epochs,
    const std::string& device_model,
    std::vector<EpochMask>& masks,
    std::size_t& pseudorange_rejections,
    std::size_t& carrier_rejections,
    double max_gap_s = 1.5) {
    masks.assign(epochs.size(), EpochMask{});
    pseudorange_rejections = 0;
    carrier_rejections = 0;
    struct Previous {
        std::size_t epoch_index = 0;
        const libgnss::Observation* observation = nullptr;
        libgnss::GNSSTime time;
    };
    std::map<ObservationKey, Previous> previous;
    for (std::size_t current_index = 0; current_index < epochs.size();
         ++current_index) {
        const auto& epoch = epochs[current_index];
        for (const auto& observation : epoch.observations) {
            const ObservationKey key{observation.satellite, observation.signal};
            const auto previous_it = previous.find(key);
            if (previous_it != previous.end() &&
                previous_it->second.observation != nullptr) {
                const auto& prior = previous_it->second;
                const double dt = epoch.time - prior.time;
                const ObservationBand band = bandForSignal(observation.signal);
                const double wavelength = libgnss::signalWavelengthMeters(observation);
                if (dt > 0.0 && dt <= max_gap_s &&
                    band != ObservationBand::Unknown &&
                    finitePositive(wavelength)) {
                    const auto* old = prior.observation;
                    if (old->has_pseudorange && observation.has_pseudorange &&
                        old->pseudorange > 0.0 && observation.pseudorange > 0.0 &&
                        old->has_doppler && observation.has_doppler &&
                        std::isfinite(old->doppler) && std::isfinite(observation.doppler)) {
                        const double difference = pseudorangeDopplerDifference(
                            old->pseudorange, observation.pseudorange,
                            old->doppler, observation.doppler, wavelength, dt);
                        if (std::isfinite(difference) &&
                            std::abs(difference) > pairThreshold(band, 'P') &&
                            !pseudorangePairSkipDevice(device_model)) {
                            if (masks[prior.epoch_index].pseudorange.insert(key).second) {
                                ++pseudorange_rejections;
                            }
                            if (masks[current_index].pseudorange.insert(key).second) {
                                ++pseudorange_rejections;
                            }
                        }
                    }
                    if (old->has_carrier_phase && observation.has_carrier_phase &&
                        std::isfinite(old->carrier_phase) &&
                        std::isfinite(observation.carrier_phase) &&
                        old->has_doppler && observation.has_doppler &&
                        std::isfinite(old->doppler) && std::isfinite(observation.doppler)) {
                        const double offset = carrierSignOffsetDevice(device_model)
                                                   ? 1.117 : 0.0;
                        const double difference = carrierDopplerDifference(
                            old->carrier_phase, observation.carrier_phase,
                            old->doppler, observation.doppler, wavelength, dt, offset);
                        if (std::isfinite(difference) &&
                            std::abs(difference) > pairThreshold(band, 'L')) {
                            if (masks[prior.epoch_index].carrier.insert(key).second) {
                                ++carrier_rejections;
                            }
                            if (masks[current_index].carrier.insert(key).second) {
                                ++carrier_rejections;
                            }
                        }
                    }
                }
            }
            previous[key] = {current_index, &observation, epoch.time};
        }
    }
}

}  // namespace libgnss::observable_upstream

// The standalone application historically exposed this contract under an
// app namespace.  Keep that source-compatible alias while making the actual
// implementation a library API; no app implementation is duplicated.
namespace libgnss_apps {
namespace upstream = ::libgnss::observable_upstream;
}
