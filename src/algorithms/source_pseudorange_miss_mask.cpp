#include <libgnss++/algorithms/source_pseudorange_miss_mask.hpp>

#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>

#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>
#include <utility>

namespace libgnss::source_pseudorange_miss_mask {
namespace {

bool finiteTime(const GNSSTime& time) {
    return std::isfinite(time.tow) && time.week >= -10000 && time.week <= 100000;
}

double finiteMedian(std::vector<double> values) {
    values.erase(std::remove_if(values.begin(), values.end(),
                                [](double value) {
                                    return !std::isfinite(value);
                                }),
                 values.end());
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    return values.size() % 2U == 0U
               ? 0.5 * (values[middle - 1U] + values[middle])
               : values[middle];
}

double finitePercentile(std::vector<double> values, double percentile) {
    values.erase(std::remove_if(values.begin(), values.end(),
                                [](double value) {
                                    return !std::isfinite(value);
                                }),
                 values.end());
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    if (values.size() == 1U) return values.front();
    const double rank = 0.5 + percentile / 100.0 * values.size();
    if (rank <= 1.0) return values.front();
    if (rank >= static_cast<double>(values.size())) return values.back();
    const double lower_rank = std::floor(rank);
    const std::size_t lower = static_cast<std::size_t>(lower_rank - 1.0);
    const std::size_t upper = lower + 1U;
    return values[lower] + (rank - lower_rank) *
                               (values[upper] - values[lower]);
}

}  // namespace

static bool applyImpl(
    std::vector<FGOProcessor::PseudorangeFactor>& factors,
    const std::vector<FGOProcessor::EpochSeed>& epochs,
    const CanonicalHasStream& has_stream,
    const CanonicalCorrectionAt& correction_at,
    bool canonical_key_mode,
    Report& report) {
    report = Report{};
    report.canonical_key_mode = canonical_key_mode;
    report.original_adopted_rows = factors.size();
    report.callback_contract_valid = static_cast<bool>(has_stream) &&
                                     static_cast<bool>(correction_at);
    if (!report.callback_contract_valid) {
        report.failure = "source miss-mask callbacks are not both supplied";
        return false;
    }
    if (std::any_of(
            factors.begin(), factors.end(),
            [](const FGOProcessor::PseudorangeFactor& factor) {
                return factor.native_base_pseudorange_correction_applied;
            })) {
        report.correction_already_applied = true;
        report.failure =
            "source miss-mask correction was already applied to a factor";
        return false;
    }
    if (!factors.empty()) {
        report.correction_application_passes = 1U;
    }

    std::vector<FGOProcessor::PseudorangeFactor> retained;
    retained.reserve(factors.size());
    std::vector<double> retained_correction_abs_m;
    retained_correction_abs_m.reserve(factors.size());
    for (const auto& original_factor : factors) {
        // Keep all caller-owned factors intact until the final vector swap,
        // including when a later callback throws after earlier corrections.
        auto factor = original_factor;
        auto& signal_counts = report.signal_counts[factor.signal];
        ++signal_counts.original_adopted_rows;
        const bool exact_stream_matched = has_stream(
            factor.satellite, factor.signal,
            factor.has_glonass_frequency_channel,
            factor.glonass_frequency_channel);
        if (exact_stream_matched) {
            ++report.matched_exact_stream_rows;
            ++signal_counts.matched_exact_stream_rows;
        } else {
            ++report.dropped_missing_exact_stream_rows;
            ++signal_counts.dropped_missing_exact_stream_rows;
            continue;
        }
        if (factor.epoch_index >= epochs.size() ||
            !finiteTime(epochs[factor.epoch_index].time)) {
            ++report.dropped_out_of_domain_rows;
            ++signal_counts.dropped_out_of_domain_rows;
            continue;
        }
        double correction_m = std::numeric_limits<double>::quiet_NaN();
        if (!correction_at(epochs[factor.epoch_index].time, factor.satellite,
                           factor.signal,
                           factor.has_glonass_frequency_channel,
                           factor.glonass_frequency_channel, correction_m)) {
            ++report.dropped_out_of_domain_rows;
            ++signal_counts.dropped_out_of_domain_rows;
            continue;
        }
        if (!std::isfinite(correction_m)) {
            ++report.dropped_nonfinite_correction_rows;
            ++signal_counts.dropped_nonfinite_correction_rows;
            continue;
        }
        const double corrected =
            base_pseudorange_compensation::subtractCorrection(
                factor.corrected_pseudorange_m, correction_m);
        if (!std::isfinite(corrected)) {
            ++report.dropped_nonfinite_correction_rows;
            ++signal_counts.dropped_nonfinite_correction_rows;
            continue;
        }
        factor.corrected_pseudorange_m = corrected;
        factor.native_base_pseudorange_correction_applied = true;
        if (exact_stream_matched) {
            ++report.finite_correction_rows_among_matched;
            ++signal_counts.finite_correction_rows_among_matched;
        }
        ++signal_counts.corrected_rows;
        retained_correction_abs_m.push_back(std::abs(correction_m));
        retained.push_back(std::move(factor));
    }

    // Do not mutate the caller's factor vector until every conservation
    // predicate below has passed.  This is the application half of the
    // Phase126 compound transaction: a late accounting failure cannot expose
    // a partially corrected or partially filtered graph.
    report.retained_finite_pc_rows = retained.size();
    report.corrected_rows = report.retained_finite_pc_rows;
    for (auto& [signal, signal_counts] : report.signal_counts) {
        signal_counts.retained_finite_pc_rows = signal_counts.corrected_rows;
        signal_counts.factor_count_consistent =
            signal_counts.original_adopted_rows ==
            signal_counts.retained_finite_pc_rows +
                signal_counts.dropped_missing_exact_stream_rows +
                signal_counts.dropped_out_of_domain_rows +
                signal_counts.dropped_nonfinite_correction_rows;
    }
    const std::size_t dropped =
        report.dropped_missing_exact_stream_rows +
        report.dropped_out_of_domain_rows +
        report.dropped_nonfinite_correction_rows;
    report.factor_count_consistent =
        report.original_adopted_rows == report.retained_finite_pc_rows + dropped;
    report.signal_count_consistent = std::all_of(
        report.signal_counts.begin(), report.signal_counts.end(),
        [](const auto& item) { return item.second.factor_count_consistent; });
    report.factor_count_consistent =
        report.factor_count_consistent && report.signal_count_consistent;
    if (report.original_adopted_rows > 0U) {
        report.retained_over_original_fraction =
            static_cast<double>(report.retained_finite_pc_rows) /
            static_cast<double>(report.original_adopted_rows);
    }
    // The report denominator is retained rows, not all adopted rows.  Every
    // retained row was checked for a finite pc before it entered the vector.
    report.retained_finite_pc_fraction =
        report.retained_finite_pc_rows > 0U ? 1.0 : 0.0;
    report.correction_abs_p50_m = finiteMedian(retained_correction_abs_m);
    report.correction_abs_p95_m = finitePercentile(
        retained_correction_abs_m, 95.0);
    report.correction_abs_max_m = retained_correction_abs_m.empty()
                                      ? std::numeric_limits<double>::quiet_NaN()
                                      : *std::max_element(
                                            retained_correction_abs_m.begin(),
                                            retained_correction_abs_m.end());
    if (!report.factor_count_consistent) {
        report.failure = "source miss-mask factor accounting mismatch";
        return false;
    }
    factors.swap(retained);
    return true;
}

bool apply(std::vector<FGOProcessor::PseudorangeFactor>& factors,
           const std::vector<FGOProcessor::EpochSeed>& epochs,
           const HasStream& has_stream,
           const CorrectionAt& correction_at,
           Report& report) {
    CanonicalHasStream canonical_has_stream;
    if (has_stream) {
        canonical_has_stream =
            [has_stream](const SatelliteId& satellite, SignalType signal, bool,
                         int) { return has_stream(satellite, signal); };
    }
    CanonicalCorrectionAt canonical_correction_at;
    if (correction_at) {
        canonical_correction_at =
            [correction_at](const GNSSTime& time, const SatelliteId& satellite,
                            SignalType signal, bool, int, double& correction_m) {
                return correction_at(time, satellite, signal, correction_m);
            };
    }
    return applyImpl(factors, epochs, canonical_has_stream,
                     canonical_correction_at, false, report);
}

bool applyCanonical(std::vector<FGOProcessor::PseudorangeFactor>& factors,
                    const std::vector<FGOProcessor::EpochSeed>& epochs,
                    const CanonicalHasStream& has_stream,
                    const CanonicalCorrectionAt& correction_at,
                    Report& report) {
    return applyImpl(factors, epochs, has_stream, correction_at, true, report);
}

}  // namespace libgnss::source_pseudorange_miss_mask
