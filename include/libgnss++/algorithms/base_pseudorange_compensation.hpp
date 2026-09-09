#pragma once

/**
 * @file base_pseudorange_compensation.hpp
 * @brief Truth-free native port of the published base pseudorange correction.
 *
 * The implementation owns only an in-memory copy of one already selected base
 * observation stream.  It does not know about files, truth, WLS solutions, or
 * an optimizer.  Callers can therefore apply the resulting correction only
 * to the adopted undifferenced pseudorange factors, leaving the SPP seed and
 * all carrier/Doppler factors untouched.
 */

#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/glonass_provenance.hpp>
#include <libgnss++/algorithms/phase131_canonical_correction_key.hpp>

#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace libgnss::base_pseudorange_compensation {

using ObservationKey = std::pair<SatelliteId, SignalType>;

struct Config {
    Vector3d base_position_ecef = Vector3d::Zero();
    // Phase126 source-complete mode is an all-or-nothing compound operator.
    // It uses the official geodist/Sagnac representation and the official
    // no-explicit-TGD/BGD resPc equation.  The default remains the historical
    // native operator below.
    bool source_complete = false;
    bool use_source_epoch_states = false; // Requires source_complete; default off.
    bool use_source_fgo_frequency_slots = false; // Source FTYPE L1/L5 slots only.
    bool use_dense_epoch_smoothing = false; // Preserve NaN epoch slots; source states only.
    bool approximate_position_present = false;
    bool antenna_delta_present = false;
    bool station_reference_verified = false;
    bool antenna_reference_is_approx_position = false;
    Vector3d antenna_delta_enu = Vector3d::Zero();
    double expected_interval_s = 0.0;
    std::size_t moving_mean_samples = 0U;
    // Keep the base residual model identical to the ordinary native FGO
    // pseudorange model.  These are explicit instead of implicit so the
    // caller can pin the candidate's correction contract in telemetry.
    bool use_ionosphere_model = true;
    bool use_troposphere_model = true;
    bool use_signal_specific_galileo_group_delay = false;
    // Phase127 is a strict provenance/admission layer composed with the
    // Phase126 source-complete operator.  It does not alter the residual
    // equation or the correction stream.
    bool use_phase127_glonass_channel_provenance = false;
    // Phase128 parser/admission overlay.  Absent/valid-empty headers still
    // permit the exact time-valid broadcast geph fallback; malformed labels
    // remain fail-closed.
    bool use_phase128_glonass_provenance_parser_admission = false;
    // Phase129 local-miss overlay.  An uncertified GLONASS row is omitted
    // from this correction stream with the same reason ledger used by the
    // rover factor builder; no uncorrected/zero/fallback row is admitted.
    bool use_phase129_glonass_local_miss_mask = false;
    // Phase131 opt-in: resolve correction streams by the source-defined
    // physical frequency family instead of literal SignalType/track-code
    // identity.  The estimator's typed signal remains unchanged and the
    // default is off.
    bool use_phase131_canonical_correction_band_key = false;
    GlonassFrequencyChannelHeaderStatus
        glonass_frequency_channel_header_status =
            GlonassFrequencyChannelHeaderStatus::Absent;
    std::vector<GlonassFrequencyChannelEntry>
        glonass_frequency_channel_entries;
    std::size_t glonass_frequency_channel_malformed_entries = 0U;
};

struct Diagnostics {
    bool source_epoch_states_requested = false;
    std::size_t source_epoch_states_built = 0;
    std::size_t source_frequency_rows_excluded = 0;
    bool enabled = false;
    bool source_complete = false;
    bool station_reference_verified = false;
    bool official_no_explicit_tgd_bgd = false;
    bool phase127_enabled = false;
    bool phase128_enabled = false;
    bool phase129_enabled = false;
    bool phase129_configuration_valid = true;
    std::string phase128_header_status = "absent";
    std::size_t phase128_canonical_records = 0U;
    std::size_t phase128_canonical_rejected_records = 0U;
    std::size_t phase127_glonass_rows = 0U;
    std::size_t phase127_accepted_rows = 0U;
    std::size_t phase127_header_primary_rows = 0U;
    std::size_t phase127_ephemeris_fallback_rows = 0U;
    std::size_t phase127_header_entries_seen = 0U;
    std::size_t phase127_header_duplicate_entries = 0U;
    std::size_t phase127_header_conflict_entries = 0U;
    std::size_t phase127_header_malformed_entries = 0U;
    std::size_t phase127_ephemeris_candidates = 0U;
    std::size_t phase127_ephemeris_ties = 0U;
    std::size_t phase127_ephemeris_duplicate_entries = 0U;
    std::size_t phase127_ephemeris_conflict_entries = 0U;
    std::size_t phase127_query_time_coverage_gaps = 0U;
    std::size_t phase127_invalid_channels = 0U;
    std::map<std::string, std::size_t> phase127_failure_counts;
    // Phase129 counts only provenance failures admitted as explicit local
    // misses.  No sample is inserted for these rows; source-complete
    // integrity failures continue to populate `failure` and abort the build.
    std::size_t phase129_glonass_local_miss_rows = 0U;
    std::map<std::string, std::size_t> phase129_glonass_local_miss_counts;
    std::size_t phase129_glonass_local_miss_streams = 0U;
    bool phase129_glonass_row_count_consistent = true;
    std::string phase129_configuration_failure;
    // Phase131 canonical correction-key admission telemetry.  This selector
    // changes only the correction join boundary; no factor/state/equation is
    // changed.
    bool phase131_enabled = false;
    bool phase131_configuration_valid = true;
    std::string phase131_configuration_failure;
    std::size_t phase131_canonical_rows = 0U;
    std::size_t phase131_canonical_rejected_rows = 0U;
    std::size_t phase131_unknown_band_rows = 0U;
    std::size_t phase131_canonical_key_conflicts = 0U;
    std::size_t phase131_canonical_duplicate_rows = 0U;
    std::size_t phase131_canonical_streams = 0U;
    std::size_t phase131_canonical_selected_streams = 0U;
    std::size_t phase131_canonical_merged_streams = 0U;
    std::map<std::string, std::size_t> phase131_failure_counts;
    std::size_t sagnac_evaluations = 0U;
    std::size_t source_complete_signal_rows = 0U;
    bool built = false;
    std::string failure;
    double base_interval_s = std::numeric_limits<double>::quiet_NaN();
    std::size_t base_epochs = 0U;
    std::size_t base_observation_rows = 0U;
    std::size_t matching_streams = 0U;
    std::size_t matched_base_rows = 0U;
    std::size_t finite_base_residual_rows = 0U;
    std::size_t smoothed_rows = 0U;
    std::size_t interpolated_rows = 0U;
    std::size_t interpolation_misses = 0U;
    std::size_t adopted_pseudorange_rows = 0U;
    std::size_t adopted_rows_corrected = 0U;
    double correction_abs_p50_m = std::numeric_limits<double>::quiet_NaN();
    double correction_abs_p95_m = std::numeric_limits<double>::quiet_NaN();
    double correction_abs_max_m = std::numeric_limits<double>::quiet_NaN();
    std::size_t moving_mean_samples = 0U;
};

/**
 * @brief In-memory same-satellite/same-signal base correction stream.
 */
class Model {
public:
    /**
     * Build finite base residual streams from raw RINEX observations and
     * broadcast navigation.  `base_epochs` must have been read by the caller
     * from the exact caller-declared sealed raw member; this method performs
     * no I/O.
     */
    bool build(const ObservationSeries& base_epochs,
               const NavigationData& nav,
               const Config& config);

    /**
     * Interpolate one correction in-domain.  Returns false for a missing
     * stream, an out-of-domain time, or a non-finite result; no extrapolation
     * or endpoint hold is performed.
     */
    bool correctionAt(const GNSSTime& time,
                      const SatelliteId& satellite,
                      SignalType signal,
                      double& correction_m) const;

    /**
     * @brief Return whether a finite base residual stream was built for an
     * exact satellite/signal key.
     *
     * This is deliberately separate from correctionAt(): callers use it to
     * account for exact source-key matching before the in-domain interpolation
     * and finiteness gates.  It does not perform any extrapolation or I/O.
     */
    bool hasStream(const SatelliteId& satellite, SignalType signal) const;

    // Diagnostic only: median of finite samples in the already-built exact
    // stream (including finite dense-grid smoothing outputs). No I/O, no
    // estimator mutation, no canonical aliasing. Output unchanged on failure.
    bool streamMedian(const SatelliteId& satellite, SignalType signal,
                      double& median_m) const;

    /** Return whether a finite source-defined canonical stream exists. */
    bool hasCanonicalStream(const SatelliteId& satellite,
                            SignalType signal,
                            bool has_glonass_frequency_channel,
                            int glonass_frequency_channel) const;

    /** Interpolate one canonical physical-frequency stream in-domain. */
    bool correctionAtCanonical(const GNSSTime& time,
                               const SatelliteId& satellite,
                               SignalType signal,
                               bool has_glonass_frequency_channel,
                               int glonass_frequency_channel,
                               double& correction_m) const;

    const Diagnostics& diagnostics() const { return diagnostics_; }

    // Read-only exact-key domain metadata for distinguishing time misses
    // from numerical interpolation failures. Does not expose corrections.
    bool streamTimeDomain(const SatelliteId& satellite, SignalType signal,
                          GNSSTime& first, GNSSTime& last) const {
        const auto found = streams_.find({satellite,signal});
        if (found == streams_.end() || found->second.empty()) return false;
        first = found->second.front().time;
        last = found->second.back().time;
        return true;
    }

private:
    struct Sample {
        GNSSTime time;
        double residual_m = 0.0;
    };

    std::map<ObservationKey, std::vector<Sample>> streams_;
    std::map<phase131_canonical::Key, std::vector<Sample>>
        canonical_streams_;
    Diagnostics diagnostics_;
};

/**
 * MATLAB smoothdata(...,"movmean",N) uses a centered window and shrinks the
 * window at either endpoint.  This helper is public so the exact edge policy
 * can be covered independently of a navigation file in focused tests.
 */
std::vector<double> centeredMovingMean(const std::vector<double>& values,
                                       std::size_t window_samples);

/** Exact source sign: positive base pc is subtracted from rover P. */
inline double subtractCorrection(double pseudorange_m, double correction_m) {
    return pseudorange_m - correction_m;
}

}  // namespace libgnss::base_pseudorange_compensation
