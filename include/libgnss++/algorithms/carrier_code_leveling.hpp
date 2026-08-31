#pragma once

// Raw-only causal carrier-code leveling for the smartphone research lane.
//
// This is deliberately a preprocessing contract rather than an estimator
// state.  A constant code bias for one satellite/signal is not identifiable
// from absolute pseudorange together with an epoch receiver clock; a free
// state would therefore add a gauge, not information.  The relative ADR
// increment within a continuous arc is observable, so this helper applies the
// standard causal Hatch recursion to the code field while leaving every other
// observable untouched.

#include <libgnss++/algorithms/observable_upstream_preprocessing.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/signals.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace libgnss::carrier_code_leveling {

struct Config {
    // The existing smartphone Hatch contract is one-Hz, 30-sample causal
    // smoothing.  This is a fixed physical/observability choice, not a
    // truth-selected hyperparameter.
    std::size_t window_samples = 30U;
    double max_gap_s = 1.5;
    SignalType signal = SignalType::GAL_E1;
    bool require_hardware_clock_counts = true;
    // Optional raw P-vs-ADR innovation reset.  The default preserves the
    // Phase 14 Hatch contract byte-for-byte.  When enabled, the threshold is
    // derived from the pinned upstream adjacent P-D pair rule; callers cannot
    // tune a second threshold through this API.
    bool enable_innovation_reset = false;
    // An explicit set is an opt-in primary-band extension.  The supported
    // multi-signal set is deliberately limited to GPS L1 C/A and Galileo E1;
    // an empty set preserves the historical single `signal` contract.
    std::vector<SignalType> signals;
};

struct SignalDiagnostics {
    SignalType signal = SignalType::GAL_E1;
    std::size_t target_rows = 0U;
    std::size_t eligible_rows = 0U;
    std::size_t smoothed_rows = 0U;
    std::size_t arcs_started = 0U;
    std::size_t updates = 0U;
    std::size_t reset_invalid_code = 0U;
    std::size_t reset_invalid_adr = 0U;
    std::size_t reset_adr = 0U;
    std::size_t reset_cycle_slip = 0U;
    std::size_t reset_missing_or_nonfinite_adr = 0U;
    std::size_t reset_gap = 0U;
    std::size_t reset_clock_discontinuity = 0U;
    std::size_t reset_innovation = 0U;
    double max_abs_innovation_accepted_m = 0.0;
    double max_abs_innovation_rejected_m = 0.0;
    double innovation_reset_threshold_m = 0.0;
};

struct Diagnostics {
    bool enabled = false;
    std::size_t input_epochs = 0U;
    std::size_t target_rows = 0U;
    std::size_t eligible_rows = 0U;
    std::size_t smoothed_rows = 0U;
    std::size_t arcs_started = 0U;
    std::size_t updates = 0U;
    std::size_t reset_invalid_code = 0U;
    std::size_t reset_invalid_adr = 0U;
    std::size_t reset_adr = 0U;
    std::size_t reset_cycle_slip = 0U;
    std::size_t reset_missing_or_nonfinite_adr = 0U;
    std::size_t reset_gap = 0U;
    std::size_t reset_clock_discontinuity = 0U;
    std::size_t reset_innovation = 0U;
    std::size_t rejected_nonfinite_level = 0U;
    double max_abs_level_adjustment_m = 0.0;
    double max_abs_phase_increment_m = 0.0;
    double max_abs_innovation_accepted_m = 0.0;
    double max_abs_innovation_rejected_m = 0.0;
    double innovation_reset_threshold_m = 0.0;
    std::vector<SignalDiagnostics> per_signal;
};

struct Result {
    ObservationSeries observations;
    Diagnostics diagnostics;
    bool ok = false;
    std::string error;
};

namespace detail {

struct ArcState {
    std::int64_t timestamp_ms = 0;
    double adr_m = 0.0;
    double smoothed_m = 0.0;
    std::size_t count = 0U;
    int clock_count = 0;
};

inline bool finitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

inline bool validAdrState(const Observation& observation) {
    // Android's ADR valid/reset/cycle-slip bits are preserved by the raw
    // loader as has_carrier_phase, loss_of_lock, and LLI.  Do not use a row
    // with an explicit loss marker even if a numeric carrier is present.
    return observation.valid && observation.has_carrier_phase &&
           !observation.loss_of_lock && observation.lli == 0U;
}

inline bool isPrimaryMultiSignal(SignalType signal) {
    return signal == SignalType::GPS_L1CA || signal == SignalType::GAL_E1;
}

}  // namespace detail

/**
 * Apply one fixed, causal Hatch recursion to the selected signal.
 *
 * `epoch_utc_time_millis` and `hardware_clock_counts` are parallel to
 * `input.epochs`.  Android mode supplies both directly from the raw CSV; the
 * helper rejects a missing/misaligned clock vector by default so a hardware
 * clock discontinuity cannot be smoothed across accidentally.  No truth,
 * receiver position, enriched satellite fields, or precomputed trajectory
 * is consulted.
 */
inline Result apply(const ObservationSeries& input,
                    const std::vector<std::int64_t>& epoch_utc_time_millis,
                    const std::vector<int>& hardware_clock_counts,
                    const Config& config = Config{}) {
    Result result;
    result.observations = input;
    result.diagnostics.input_epochs = input.epochs.size();
    result.diagnostics.enabled = true;
    if (config.window_samples == 0U || !(config.max_gap_s > 0.0) ||
        !std::isfinite(config.max_gap_s)) {
        result.error = "carrier-code leveling configuration is invalid";
        return result;
    }
    if (epoch_utc_time_millis.size() != input.epochs.size()) {
        result.error = "carrier-code leveling epoch UTC keys are misaligned";
        return result;
    }
    if (config.require_hardware_clock_counts &&
        hardware_clock_counts.size() != input.epochs.size()) {
        result.error = "carrier-code leveling requires one hardware clock count per epoch";
        return result;
    }

    const std::int64_t max_gap_ms = static_cast<std::int64_t>(
        std::llround(config.max_gap_s * 1000.0));
    if (max_gap_ms <= 0) {
        result.error = "carrier-code leveling gap is below one millisecond";
        return result;
    }

    std::vector<SignalType> selected_signals;
    if (config.signals.empty()) {
        selected_signals.push_back(config.signal);
    } else {
        for (const SignalType signal : config.signals) {
            if (!detail::isPrimaryMultiSignal(signal)) {
                result.error =
                    "carrier-code multi-signal selection only supports GPS_L1CA and GAL_E1";
                return result;
            }
            if (std::find(selected_signals.begin(), selected_signals.end(), signal) !=
                selected_signals.end()) {
                result.error = "carrier-code multi-signal selection contains a duplicate";
                return result;
            }
            selected_signals.push_back(signal);
        }
        if (selected_signals.empty()) {
            result.error = "carrier-code multi-signal selection is empty";
            return result;
        }
    }

    std::map<SignalType, std::size_t> signal_diagnostic_indices;
    for (const SignalType signal : selected_signals) {
        const double threshold = observable_upstream::pairThreshold(
            observable_upstream::bandForSignal(signal), 'P');
        if (config.enable_innovation_reset &&
            (!std::isfinite(threshold) || !(threshold > 0.0))) {
            result.error = "carrier-code innovation reset lacks an upstream P threshold";
            return result;
        }
        SignalDiagnostics signal_diagnostics;
        signal_diagnostics.signal = signal;
        signal_diagnostics.innovation_reset_threshold_m =
            config.enable_innovation_reset ? threshold : 0.0;
        signal_diagnostic_indices.emplace(signal,
                                          result.diagnostics.per_signal.size());
        result.diagnostics.per_signal.push_back(signal_diagnostics);
        if (result.diagnostics.per_signal.size() == 1U) {
            // Retain the historical aggregate field for single-signal callers.
            result.diagnostics.innovation_reset_threshold_m =
                signal_diagnostics.innovation_reset_threshold_m;
        }
    }

    using Key = std::pair<SatelliteId, SignalType>;
    std::map<Key, detail::ArcState> arcs;
    std::int64_t previous_epoch_timestamp_ms = 0;
    bool have_previous_epoch_timestamp = false;
    for (std::size_t epoch_index = 0U; epoch_index < input.epochs.size();
         ++epoch_index) {
        auto& epoch = result.observations.epochs[epoch_index];
        const std::int64_t timestamp_ms = epoch_utc_time_millis[epoch_index];
        if (have_previous_epoch_timestamp &&
            timestamp_ms <= previous_epoch_timestamp_ms) {
            result.error =
                "carrier-code leveling epoch UTC keys are not strictly increasing";
            return result;
        }
        previous_epoch_timestamp_ms = timestamp_ms;
        have_previous_epoch_timestamp = true;
        const int clock_count = hardware_clock_counts.empty()
                                    ? 0
                                    : hardware_clock_counts[epoch_index];
        for (auto& observation : epoch.observations) {
            const auto signal_diagnostic_it =
                signal_diagnostic_indices.find(observation.signal);
            if (signal_diagnostic_it == signal_diagnostic_indices.end()) continue;
            auto& signal_diagnostics =
                result.diagnostics.per_signal[signal_diagnostic_it->second];
            ++result.diagnostics.target_rows;
            ++signal_diagnostics.target_rows;
            const Key key{observation.satellite, observation.signal};

            if (!observation.valid || !observation.has_pseudorange ||
                !detail::finitePositive(observation.pseudorange)) {
                arcs.erase(key);
                ++result.diagnostics.reset_invalid_code;
                ++signal_diagnostics.reset_invalid_code;
                continue;
            }
            if (observation.loss_of_lock) {
                arcs.erase(key);
                ++result.diagnostics.reset_adr;
                ++signal_diagnostics.reset_adr;
                continue;
            }
            if ((observation.lli & 0x01U) != 0U) {
                arcs.erase(key);
                ++result.diagnostics.reset_cycle_slip;
                ++signal_diagnostics.reset_cycle_slip;
                continue;
            }
            const double wavelength = signalWavelengthMeters(observation);
            const double adr_m = observation.has_carrier_phase
                                     ? observation.carrier_phase * wavelength
                                     : std::numeric_limits<double>::quiet_NaN();
            if (!(wavelength > 0.0) || !std::isfinite(wavelength) ||
                !std::isfinite(adr_m) || std::abs(adr_m) >= 1.0e9) {
                arcs.erase(key);
                ++result.diagnostics.reset_missing_or_nonfinite_adr;
                ++signal_diagnostics.reset_missing_or_nonfinite_adr;
                continue;
            }
            ++result.diagnostics.eligible_rows;
            ++signal_diagnostics.eligible_rows;

            auto previous_it = arcs.find(key);
            if (previous_it != arcs.end()) {
                const detail::ArcState& previous = previous_it->second;
                const std::int64_t elapsed_ms = timestamp_ms - previous.timestamp_ms;
                if (elapsed_ms <= 0 || elapsed_ms > max_gap_ms) {
                    arcs.erase(previous_it);
                    ++result.diagnostics.reset_gap;
                    ++signal_diagnostics.reset_gap;
                    previous_it = arcs.end();
                } else if (config.require_hardware_clock_counts &&
                           clock_count != previous.clock_count) {
                    arcs.erase(previous_it);
                    ++result.diagnostics.reset_clock_discontinuity;
                    ++signal_diagnostics.reset_clock_discontinuity;
                    previous_it = arcs.end();
                }
            }

            if (previous_it == arcs.end()) {
                arcs[key] = detail::ArcState{timestamp_ms, adr_m,
                                             observation.pseudorange, 1U,
                                             clock_count};
                ++result.diagnostics.arcs_started;
                ++signal_diagnostics.arcs_started;
                continue;
            }

            const detail::ArcState previous = previous_it->second;
            const double phase_increment = adr_m - previous.adr_m;
            const std::size_t count =
                std::min(config.window_samples, previous.count + 1U);
            const double predicted = previous.smoothed_m + phase_increment;
            const double innovation = observation.pseudorange - predicted;
            if (!std::isfinite(innovation)) {
                arcs.erase(previous_it);
                ++result.diagnostics.rejected_nonfinite_level;
                ++result.diagnostics.reset_missing_or_nonfinite_adr;
                ++signal_diagnostics.reset_missing_or_nonfinite_adr;
                continue;
            }
            if (config.enable_innovation_reset &&
                std::abs(innovation) >
                    signal_diagnostics.innovation_reset_threshold_m) {
                // The raw code is the only safe emitted value after a
                // discontinuity: do not smooth across a suspected code/ADR
                // seam and do not synthesize a corrected coordinate.
                result.diagnostics.max_abs_innovation_rejected_m = std::max(
                    result.diagnostics.max_abs_innovation_rejected_m,
                    std::abs(innovation));
                ++result.diagnostics.reset_innovation;
                ++signal_diagnostics.reset_innovation;
                signal_diagnostics.max_abs_innovation_rejected_m = std::max(
                    signal_diagnostics.max_abs_innovation_rejected_m,
                    std::abs(innovation));
                ++result.diagnostics.arcs_started;
                ++signal_diagnostics.arcs_started;
                arcs[key] = detail::ArcState{timestamp_ms, adr_m,
                                             observation.pseudorange, 1U,
                                             clock_count};
                continue;
            }
            result.diagnostics.max_abs_innovation_accepted_m = std::max(
                result.diagnostics.max_abs_innovation_accepted_m,
                std::abs(innovation));
            signal_diagnostics.max_abs_innovation_accepted_m = std::max(
                signal_diagnostics.max_abs_innovation_accepted_m,
                std::abs(innovation));
            const double smoothed =
                (static_cast<double>(count - 1U) * predicted +
                 observation.pseudorange) /
                static_cast<double>(count);
            if (!detail::finitePositive(smoothed)) {
                arcs.erase(previous_it);
                ++result.diagnostics.rejected_nonfinite_level;
                ++result.diagnostics.reset_missing_or_nonfinite_adr;
                ++signal_diagnostics.reset_missing_or_nonfinite_adr;
                continue;
            }
            observation.pseudorange = smoothed;
            result.diagnostics.max_abs_level_adjustment_m = std::max(
                result.diagnostics.max_abs_level_adjustment_m,
                std::abs(smoothed - previous.smoothed_m - phase_increment));
            result.diagnostics.max_abs_phase_increment_m = std::max(
                result.diagnostics.max_abs_phase_increment_m,
                std::abs(phase_increment));
            ++result.diagnostics.smoothed_rows;
            ++result.diagnostics.updates;
            ++signal_diagnostics.smoothed_rows;
            ++signal_diagnostics.updates;
            arcs[key] = detail::ArcState{timestamp_ms, adr_m, smoothed, count,
                                         clock_count};
        }
    }
    result.ok = true;
    return result;
}

}  // namespace libgnss::carrier_code_leveling
