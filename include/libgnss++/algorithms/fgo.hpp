#pragma once

#include <libgnss++/algorithms/fgo_config.hpp>
#include <libgnss++/algorithms/doppler_velocity_wls.hpp>
#include <libgnss++/algorithms/raw_p_seed.hpp>
#include <libgnss++/algorithms/observable_upstream_preprocessing.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/core/types.hpp>
#include <libgnss++/io/imu.hpp>

#include <cstddef>
#include <cstdint>
#include <array>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace libgnss {

/**
 * @brief Batch pseudorange factor-graph optimizer.
 *
 * This is the native Eigen backend for the GNSS FGO pipeline. It keeps the
 * factor/problem representation explicit so a GTSAM backend can be added later
 * without changing callers that prepare GNSS factors from RINEX/navigation data.
 */
class FGOProcessor {
public:
    // FGOConfig lives in <libgnss++/algorithms/fgo_config.hpp> (fgo::Config);
    // the alias preserves the historical FGOProcessor::FGOConfig spelling.
    using FGOConfig = fgo::Config;

    // Official source-parity receiver clock state.  Components are ordered
    // exactly as sysfreq2sigtype.m: C[0] shared base/GPS-L1 clock, C[1..3]
    // GLO/GAL/BDS L1, and C[4..6] GPS/GAL/BDS L5-class components.  All
    // entries are metres; D remains metres/second.
    using EpochClockBiasComponentsM = std::array<double, 7>;
    static constexpr std::size_t kEpochClockBiasComponentCount = 7;

    struct EpochSeed {
        GNSSTime time;
        Vector3d position_ecef = Vector3d::Zero();
        double receiver_clock_bias_m = 0.0;
        // The historical SPP seed stores receiver_clock_bias_m in seconds
        // until an opt-in raw bridge normalizes it.  Keep the marker explicit
        // so a bridge never guesses units from magnitude.
        bool receiver_clock_bias_is_meters = false;
        // Optional raw Android receiver clock drift [m/s], copied from the
        // input epoch when DriftNanosPerSecond is available.  NaN means that
        // no raw drift was supplied; no estimator may infer it from a
        // coordinate for an upstream residual-screen candidate.
        double receiver_clock_drift_mps =
            std::numeric_limits<double>::quiet_NaN();
        // Exact source identity copied from ObservationData before an epoch
        // can be filtered out of the retained problem.  The source-meter
        // clock-state candidate uses these fields for a strict raw-D lookup;
        // it never matches by nearest time or by retained-vector position.
        std::size_t raw_source_index = std::numeric_limits<std::size_t>::max();
        std::int64_t raw_utc_time_millis = -1;
        // True only when position_ecef came from a valid SPP solve at this
        // epoch; false for last-valid/header fallbacks.
        bool fresh_spp_solution = false;
        // True when position_ecef was held from the most recent valid SPP
        // solve.  This remains false for the raw receiver-seed fallback.
        bool last_valid_spp_hold = false;
    };

    struct ObservationModelDebug {
        double raw_pseudorange_m = 0.0;
        double raw_carrier_m = 0.0;
        double satellite_clock_m = 0.0;
        double ionosphere_delay_m = 0.0;
        double troposphere_delay_m = 0.0;
        double group_delay_m = 0.0;
        double corrected_pseudorange_m = 0.0;
        double corrected_carrier_m = 0.0;
        double geometric_range_m = 0.0;
        double elevation_rad = 0.0;
        double azimuth_rad = 0.0;
        bool has_doppler_residual = false;
        double doppler_residual_mps = 0.0;
        double doppler_measured_range_rate_mps = 0.0;
        double doppler_satellite_range_rate_mps = 0.0;
        double doppler_satellite_clock_drift_mps = 0.0;
        bool doppler_uses_rotated_satellite_state = false;
        // Raw rover-receiver SNR/CN0 [dB-Hz] for this observation (Observation::snr
        // at the point this model_debug was built). Added for the sat-badness
        // EWMA down-weighting port's elevation/SNR penalty terms (see
        // FGOConfig::use_sat_badness_downweight); unused elsewhere. 0.0 when the
        // source Observation carried no SNR.
        double snr_dbhz = 0.0;
    };

    struct PseudorangeFactor {
        std::size_t epoch_index = 0;
        SatelliteId satellite;
        SignalType signal = SignalType::GPS_L1CA;
        GNSSSystem clock_group = GNSSSystem::GPS;
        Vector3d satellite_position_ecef = Vector3d::Zero();
        // Unrotated broadcast state at the source transmit-time query.  The
        // historical factor consumes satellite_position_ecef (already
        // earth-rotation corrected); Phase135 uses this provenance field to
        // form the official single-Sagnac fixed-LOS geometry.  It is never
        // consulted when the Phase135 selector is disabled.
        Vector3d source_satellite_position_ecef = Vector3d::Zero();
        bool source_satellite_position_available = false;
        double corrected_pseudorange_m = 0.0;
        double sigma_m = 1.0;
        // Raw CN0/SNR carried with the factor for truth-free quality
        // diagnostics.  The value is never populated from enriched
        // receiver/satellite coordinate columns.
        double snr_dbhz = 0.0;
        // Truth-free pre-fit residual against the native SPP seed, used only
        // by the opt-in upstream global residual mask.  It is not an
        // optimizer state and remains zero for legacy factors.
        double upstream_seed_residual_m = 0.0;
        double elevation_rad = 0.0;
        // Coefficient for the optional shared vertical L1 residual-ionosphere
        // state [m].  It is precomputed from the raw-row signal frequency and
        // seed elevation; zero means the row cannot participate in that
        // opt-in candidate.  The field is diagnostic-only when the candidate
        // is disabled.
        double residual_ionosphere_coefficient = 0.0;
        // Source-exact Android ReceivedSvTimeUncertaintyNanos provenance.
        // The floor is applied only when the corresponding opt-in config is
        // enabled; these fields are never populated from truth or WLS data.
        double android_sv_time_uncertainty_floor_m = 0.0;
        bool android_sv_time_uncertainty_available = false;
        bool android_sv_time_uncertainty_floor_applied = false;
        // Truth-free provenance guard for the opt-in native raw-base
        // pseudorange correction.  It is not consumed by any factor or
        // solver; the source miss-mask uses it only to reject a second
        // correction pass on an already corrected factor vector.
        bool native_base_pseudorange_correction_applied = false;
        // Phase131 correction-join provenance.  The typed estimator signal is
        // retained; these fields carry only the exact Phase127/128-certified
        // GLONASS channel needed by the canonical base stream key.
        bool has_glonass_frequency_channel = false;
        int glonass_frequency_channel = 0;
    };

    struct TimeDifferencedCarrierFactor {
        std::size_t previous_epoch_index = 0;
        std::size_t current_epoch_index = 0;
        SatelliteId satellite;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d previous_satellite_position_ecef = Vector3d::Zero();
        Vector3d current_satellite_position_ecef = Vector3d::Zero();
        Vector3d previous_source_satellite_position_ecef = Vector3d::Zero();
        Vector3d current_source_satellite_position_ecef = Vector3d::Zero();
        bool source_satellite_positions_available = false;
        double delta_carrier_m = 0.0;
        double sigma_m = 0.03;
        double dt_s = 0.0;
        // Same-run carrier geometry, not a lookup in accepted code factors.
        // Zero indicates unavailable in legacy/DD builders. No solver uses
        // these until an explicit joint residual-ionosphere lane is enabled.
        double previous_residual_ionosphere_coefficient = 0.0;
        double current_residual_ionosphere_coefficient = 0.0;
        double previous_source_adr_uncertainty_m = 0.0;
        double current_source_adr_uncertainty_m = 0.0;
    };

    struct SingleDifferenceDopplerFactor {
        std::size_t epoch_index = 0;
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d los = Vector3d::Zero();
        double residual_mps = 0.0;
        double sigma_mps = 0.2;
        double elevation_rad = 0.0;
    };

    /**
     * @brief Receiver-only (undifferenced) Doppler factor.
     *
     * The measured range-rate residual is prepared from the rover
     * observation and broadcast satellite state.  Unlike the
     * SingleDifferenceDopplerFactor this row has no base/reference satellite
     * and therefore remains usable in a no-base phone graph.
     */
    struct UndifferencedDopplerFactor {
        std::size_t epoch_index = 0;
        std::size_t previous_epoch_index =
            std::numeric_limits<std::size_t>::max();
        SatelliteId satellite;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d los = Vector3d::Zero();
        double residual_mps = 0.0;
        double sigma_mps = 0.2;
        double elevation_rad = 0.0;
        double dt_s = 0.0;
        // Broadcast satellite state and carrier wavelength used to prepare
        // this row.  These fields are diagnostic provenance for the raw
        // measurement-contract audit; the factor equation continues to use
        // only LOS, residual, and the known range-rate/clock terms below.
        Vector3d satellite_position_ecef = Vector3d::Zero();
        Vector3d satellite_velocity_ecef = Vector3d::Zero();
        Vector3d source_satellite_position_ecef = Vector3d::Zero();
        Vector3d source_satellite_velocity_ecef = Vector3d::Zero();
        bool source_satellite_state_available = false;
        double wavelength_m = 0.0;
        double measured_range_rate_mps = 0.0;
        double satellite_range_rate_mps = 0.0;
        double satellite_clock_drift_mps = 0.0;
        bool includes_receiver_clock_drift = false;
        bool uses_rotated_satellite_state = false;
        // Phase58 C/N0 model provenance.  These fields are populated only
        // when the opt-in calibration is enabled and are diagnostic; the
        // factor equation still uses residual_mps and sigma_mps.
        double cn0_doppler_model_sigma_mps = 0.0;
        bool cn0_doppler_model_sigma_available = false;
        bool cn0_doppler_sigma_floor_applied = false;
    };

    struct SingleDifferenceTdcpFactor {
        std::size_t previous_epoch_index = 0;
        std::size_t current_epoch_index = 0;
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d previous_los = Vector3d::Zero();
        Vector3d los = Vector3d::Zero();
        double delta_carrier_m = 0.0;
        double sigma_m = 0.003;
        double elevation_rad = 0.0;
        std::size_t target_ambiguity_index = 0;
        std::size_t reference_ambiguity_index = 0;
        int arc_length_epochs = 0;
        double dt_s = 0.0;
        bool has_doppler_witness = false;
        double previous_sd_doppler_mps = 0.0;
        double current_sd_doppler_mps = 0.0;
        double previous_sd_doppler_sigma_mps = 0.0;
        double current_sd_doppler_sigma_mps = 0.0;
    };

    struct AmbiguityState {
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        std::size_t segment_index = 0;
        double wavelength_m = 0.0;
        double initial_ambiguity_m = 0.0;
        bool is_double_difference = false;
    };

    struct CarrierPhaseFactor {
        std::size_t epoch_index = 0;
        std::size_t ambiguity_index = 0;
        SatelliteId satellite;
        GNSSSystem clock_group = GNSSSystem::GPS;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d satellite_position_ecef = Vector3d::Zero();
        double corrected_pseudorange_m = 0.0;
        double corrected_carrier_m = 0.0;
        double wavelength_m = 0.0;
        double sigma_m = 0.01;
        double elevation_rad = 0.0;
        bool has_carrier_phase = true;
        bool loss_of_lock = false;
        bool has_doppler_residual = false;
        double doppler_residual_mps = 0.0;
        double doppler_sigma_mps = 0.2;
        Vector3d los = Vector3d::Zero();
        ObservationModelDebug model_debug;
    };

    struct DoubleDifferencePseudorangeFactor {
        std::size_t epoch_index = 0;
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d rover_satellite_position_ecef = Vector3d::Zero();
        Vector3d rover_reference_position_ecef = Vector3d::Zero();
        Vector3d base_satellite_position_ecef = Vector3d::Zero();
        Vector3d base_reference_position_ecef = Vector3d::Zero();
        Vector3d base_position_ecef = Vector3d::Zero();
        double observed_dd_pseudorange_m = 0.0;
        double sigma_m = 1.0;
        double elevation_rad = 0.0;
        ObservationModelDebug rover_satellite_model;
        ObservationModelDebug rover_reference_model;
        ObservationModelDebug base_satellite_model;
        ObservationModelDebug base_reference_model;
    };

    struct DoubleDifferenceCarrierFactor {
        std::size_t epoch_index = 0;
        std::size_t ambiguity_index = 0;
        std::size_t reference_ambiguity_index = 0;
        bool use_ambiguity_difference = true;
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d rover_satellite_position_ecef = Vector3d::Zero();
        Vector3d rover_reference_position_ecef = Vector3d::Zero();
        Vector3d base_satellite_position_ecef = Vector3d::Zero();
        Vector3d base_reference_position_ecef = Vector3d::Zero();
        Vector3d base_position_ecef = Vector3d::Zero();
        double observed_dd_carrier_m = 0.0;
        double sigma_m = 0.02;
        double elevation_rad = 0.0;
        ObservationModelDebug rover_satellite_model;
        ObservationModelDebug rover_reference_model;
        ObservationModelDebug base_satellite_model;
        ObservationModelDebug base_reference_model;
    };

    struct AmbiguityBetweenFactor {
        std::size_t previous_epoch_index = 0;
        std::size_t current_epoch_index = 0;
        std::size_t previous_ambiguity_index = 0;
        std::size_t current_ambiguity_index = 0;
        SatelliteId satellite;
        SignalType signal = SignalType::GPS_L1CA;
        double sigma_m = 0.001;
    };

    // Phase116 read-only ordinary-TDCP incidence accounting.  These counters
    // are populated only when FGOConfig::use_carrier_tdcp_incidence_diagnostic
    // is enabled.  They describe the existing same-satellite/same-signal
    // factor builder and never add a factor or an ambiguity state.
    struct TdcpSignalDiagnostics {
        std::size_t carrier_rows_seen = 0;
        std::size_t carrier_phase_rows = 0;
        std::size_t retained_carrier_rows = 0;
        std::size_t missing_wavelength = 0;
        std::size_t nonfinite_measurements = 0;
        std::size_t candidate_pairs = 0;
        std::size_t accepted_pairs = 0;
        std::size_t rejected_gap = 0;
        std::size_t rejected_clock_discontinuity = 0;
        std::size_t rejected_missing_previous = 0;
        std::size_t rejected_loss_of_lock = 0;
        std::size_t rejected_nonfinite = 0;
        std::size_t rejected_code_phase_jump = 0;
        // Phase117-only fail-closed weighting metadata rejection.  This is
        // post-admission and therefore does not alter the native pair
        // predicate; valid source metadata keeps the factor count unchanged.
        std::size_t rejected_invalid_weight = 0;
    };

    struct FGOProblemDiagnostics {
        // A summary-boundary snapshot used by the native application to copy
        // the already-computed Phase131 base-report telemetry into the
        // top-level FGO diagnostics object.  This is deliberately a plain
        // value object: it has no estimator authority and must not be used
        // while constructing observations, factors, values, or solver
        // settings.
        struct Phase131DiagnosticsSnapshot {
            bool enabled = false;
            bool configuration_valid = true;
            std::string configuration_failure;
            std::size_t canonical_rows = 0U;
            std::size_t canonical_rejected_rows = 0U;
            std::size_t unknown_band_rows = 0U;
            std::size_t canonical_key_conflicts = 0U;
            std::size_t canonical_duplicate_rows = 0U;
            std::size_t canonical_streams = 0U;
            std::size_t canonical_selected_streams = 0U;
            std::size_t canonical_merged_streams = 0U;
            std::map<std::string, std::size_t> failure_counts;

            // The native report does not have a separate resolver counter;
            // canonical rows plus rejected rows is its exact attempt
            // accounting.  Keep the derived value explicit so the wrapper
            // does not have to reconstruct it from a second object.
            std::size_t canonicalization_attempt_rows = 0U;
            std::size_t resolver_call_count = 0U;

            // Source-exact correction/miss conservation copied from the
            // same base report.  These fields are diagnostic only and do not
            // authorize a second correction pass.
            bool source_miss_mask_enabled = false;
            bool source_miss_mask_canonical_key_mode = false;
            std::string source_miss_mask_matching_key = "(satellite,signal)";
            std::size_t original_adopted_pseudorange_rows = 0U;
            std::size_t retained_finite_pc_pseudorange_rows = 0U;
            std::size_t dropped_missing_exact_stream_rows = 0U;
            std::size_t dropped_out_of_domain_rows = 0U;
            std::size_t dropped_nonfinite_correction_rows = 0U;
            std::size_t matched_factor_rows = 0U;
            std::size_t finite_correction_rows_among_matched = 0U;
            std::size_t source_model_build_count = 0U;
            std::size_t correction_application_pass_count = 0U;
            std::size_t corrected_rows = 0U;
            bool pseudorange_factor_count_consistent = false;
            bool signal_count_consistent = false;
            bool applied = false;
            bool correction_applied_exactly_once = false;
            bool duplicate_correction_rejected = false;
        };

        // Copy, rather than accumulate, the one native base-report snapshot.
        // The monotonic count makes an accidental second synchronization
        // fail closed and gives the summary a directly testable exactly-once
        // invariant.  Selector-off callers leave this count at zero, so the
        // historical summary remains byte/field compatible.
        bool synchronizePhase131Diagnostics(
            const Phase131DiagnosticsSnapshot& snapshot) {
            if (phase131_diagnostics_bridge_sync_count != 0U) {
                return false;
            }
            phase131_canonical_correction_band_key_enabled = snapshot.enabled;
            phase131_configuration_valid = snapshot.configuration_valid;
            phase131_configuration_failure = snapshot.configuration_failure;
            phase131_canonical_rows = snapshot.canonical_rows;
            phase131_canonical_rejected_rows = snapshot.canonical_rejected_rows;
            phase131_unknown_band_rows = snapshot.unknown_band_rows;
            phase131_canonical_key_conflicts = snapshot.canonical_key_conflicts;
            phase131_canonical_duplicate_rows = snapshot.canonical_duplicate_rows;
            phase131_canonical_streams = snapshot.canonical_streams;
            phase131_canonical_selected_streams =
                snapshot.canonical_selected_streams;
            phase131_canonical_merged_streams = snapshot.canonical_merged_streams;
            phase131_failure_counts = snapshot.failure_counts;
            phase131_canonicalization_attempt_rows =
                snapshot.canonicalization_attempt_rows;
            phase131_resolver_call_count = snapshot.resolver_call_count;
            phase131_source_miss_mask_enabled =
                snapshot.source_miss_mask_enabled;
            phase131_source_miss_mask_canonical_key_mode =
                snapshot.source_miss_mask_canonical_key_mode;
            phase131_source_miss_mask_matching_key =
                snapshot.source_miss_mask_matching_key;
            phase131_original_adopted_pseudorange_rows =
                snapshot.original_adopted_pseudorange_rows;
            phase131_retained_finite_pc_pseudorange_rows =
                snapshot.retained_finite_pc_pseudorange_rows;
            phase131_dropped_missing_exact_stream_rows =
                snapshot.dropped_missing_exact_stream_rows;
            phase131_dropped_out_of_domain_rows =
                snapshot.dropped_out_of_domain_rows;
            phase131_dropped_nonfinite_correction_rows =
                snapshot.dropped_nonfinite_correction_rows;
            phase131_matched_factor_rows = snapshot.matched_factor_rows;
            phase131_finite_correction_rows_among_matched =
                snapshot.finite_correction_rows_among_matched;
            phase131_source_model_build_count = snapshot.source_model_build_count;
            phase131_correction_application_pass_count =
                snapshot.correction_application_pass_count;
            phase131_corrected_rows = snapshot.corrected_rows;
            phase131_pseudorange_factor_count_consistent =
                snapshot.pseudorange_factor_count_consistent;
            phase131_signal_count_consistent = snapshot.signal_count_consistent;
            phase131_applied = snapshot.applied;
            phase131_correction_applied_exactly_once =
                snapshot.correction_applied_exactly_once;
            phase131_duplicate_correction_rejected =
                snapshot.duplicate_correction_rejected;
            phase131_diagnostics_bridge_sync_count = 1U;
            return true;
        }

        std::size_t phase131_diagnostics_bridge_sync_count = 0U;
        std::size_t phase131_canonicalization_attempt_rows = 0U;
        std::size_t phase131_resolver_call_count = 0U;
        bool phase131_source_miss_mask_enabled = false;
        bool phase131_source_miss_mask_canonical_key_mode = false;
        std::string phase131_source_miss_mask_matching_key =
            "(satellite,signal)";
        std::size_t phase131_original_adopted_pseudorange_rows = 0U;
        std::size_t phase131_retained_finite_pc_pseudorange_rows = 0U;
        std::size_t phase131_dropped_missing_exact_stream_rows = 0U;
        std::size_t phase131_dropped_out_of_domain_rows = 0U;
        std::size_t phase131_dropped_nonfinite_correction_rows = 0U;
        std::size_t phase131_matched_factor_rows = 0U;
        std::size_t phase131_finite_correction_rows_among_matched = 0U;
        std::size_t phase131_source_model_build_count = 0U;
        std::size_t phase131_correction_application_pass_count = 0U;
        std::size_t phase131_corrected_rows = 0U;
        bool phase131_pseudorange_factor_count_consistent = false;
        bool phase131_signal_count_consistent = false;
        bool phase131_applied = false;
        bool phase131_correction_applied_exactly_once = false;
        bool phase131_duplicate_correction_rejected = false;

        std::size_t input_epochs = 0;
        std::size_t seeded_epochs = 0;
        std::size_t skipped_epochs_without_seed = 0;
        // Truth-free SPP anchor replay diagnostics.  These are populated only
        // for FGOConfig::use_quality_anchor_initialization.
        bool quality_anchor_initialization_enabled = false;
        bool quality_anchor_selected = false;
        std::size_t quality_anchor_index = std::numeric_limits<std::size_t>::max();
        std::size_t quality_anchor_candidates = 0;
        std::size_t quality_anchor_forward_valid_epochs = 0;
        std::size_t quality_anchor_backward_valid_epochs = 0;
        std::size_t quality_anchor_fallback_epochs = 0;
        int quality_anchor_satellites = 0;
        double quality_anchor_gdop = std::numeric_limits<double>::quiet_NaN();
        double quality_anchor_normalized_residual_rms =
            std::numeric_limits<double>::quiet_NaN();
        // Truth-free fallback-seed quality-anchor recovery diagnostics.  The
        // normal quality-anchor reconnaissance is authoritative and always
        // precedes this opt-in retry; these fields are zero/false when the
        // recovery flag is disabled or not triggered.
        bool quality_anchor_recovery_enabled = false;
        bool quality_anchor_recovery_triggered = false;
        bool quality_anchor_recovery_selected = false;
        std::size_t quality_anchor_normal_candidates = 0;
        std::size_t quality_anchor_recovery_candidates = 0;
        std::size_t quality_anchor_recovery_anchor_index =
            std::numeric_limits<std::size_t>::max();
        int quality_anchor_recovery_anchor_satellites = 0;
        double quality_anchor_recovery_anchor_gdop =
            std::numeric_limits<double>::quiet_NaN();
        double quality_anchor_recovery_anchor_normalized_residual_rms =
            std::numeric_limits<double>::quiet_NaN();
        std::size_t quality_anchor_recovery_replay_valid_epochs = 0;
        std::size_t quality_anchor_recovery_replay_invalid_epochs = 0;
        // Deliberately fixed false: recovery is an initializer only and does
        // not bypass factor-level elevation/geometry selection globally.
        bool sentinel_factor_bypass = false;
        // Phase127 strict GLONASS FCN provenance telemetry.  These fields are
        // admission metadata only: a failed source proof stops the selector-on
        // build before its row reaches a factor or correction stream, except
        // for the Phase129 explicit local-miss overlay; default remains
        // untouched.
        bool phase127_glonass_channel_provenance_enabled = false;
        bool phase128_glonass_provenance_parser_admission_enabled = false;
        // Phase129 keeps the same exact-query provenance ledger, but changes
        // only an uncertified GLONASS row's local admission decision.  It
        // never retains raw/zero-corrected data or changes graph topology.
        bool phase129_glonass_local_miss_mask_enabled = false;
        bool phase129_configuration_valid = true;
        std::string phase129_configuration_failure;
        std::string phase128_header_status = "absent";
        std::size_t phase128_canonical_records = 0;
        std::size_t phase128_canonical_rejected_records = 0;
        std::size_t phase127_glonass_rows = 0;
        std::size_t phase127_accepted_rows = 0;
        std::size_t phase127_header_primary_rows = 0;
        std::size_t phase127_ephemeris_fallback_rows = 0;
        std::size_t phase127_header_entries_seen = 0;
        std::size_t phase127_header_duplicate_entries = 0;
        std::size_t phase127_header_conflict_entries = 0;
        std::size_t phase127_header_malformed_entries = 0;
        std::size_t phase127_ephemeris_candidates = 0;
        std::size_t phase127_ephemeris_ties = 0;
        std::size_t phase127_ephemeris_duplicate_entries = 0;
        std::size_t phase127_ephemeris_conflict_entries = 0;
        std::size_t phase127_query_time_coverage_gaps = 0;
        std::size_t phase127_invalid_channels = 0;
        std::map<std::string, std::size_t> phase127_failure_counts;
        std::string phase127_failure;
        // Phase129 local-miss and shared-vector conservation telemetry.  The
        // counters describe the existing pseudorange factor population only;
        // no additional factor/state is created.
        std::size_t phase129_glonass_local_miss_rows = 0;
        std::map<std::string, std::size_t> phase129_glonass_local_miss_counts;
        std::size_t phase129_glonass_factor_rows_dropped = 0;
        std::size_t phase129_glonass_factor_rows_retained = 0;
        bool phase129_glonass_factor_count_consistent = true;
        bool phase129_glonass_row_count_consistent = true;
        // Phase131 physical-frequency correction-key telemetry.  This is an
        // opt-in admission boundary and must not alter graph topology,
        // factors, equations, units, or solver settings.
        bool phase131_canonical_correction_band_key_enabled = false;
        bool phase131_configuration_valid = true;
        std::string phase131_configuration_failure;
        std::size_t phase131_canonical_rows = 0;
        std::size_t phase131_canonical_rejected_rows = 0;
        std::size_t phase131_unknown_band_rows = 0;
        std::size_t phase131_canonical_key_conflicts = 0;
        std::size_t phase131_canonical_duplicate_rows = 0;
        std::size_t phase131_canonical_streams = 0;
        std::size_t phase131_canonical_selected_streams = 0;
        std::size_t phase131_canonical_merged_streams = 0;
        std::map<std::string, std::size_t> phase131_failure_counts;
        // Phase135 compound fixed-initial-geometry affine-family admission
        // witness. This is diagnostic-only and leaves the legacy problem
        // builder unchanged when the selector is disabled.
        bool phase135_official_affine_measurement_family_enabled = false;
        bool phase135_configuration_valid = true;
        std::string phase135_configuration_failure;
        std::size_t sparse_epochs_retained = 0;
        std::size_t sparse_empty_epochs_retained = 0;
        std::size_t double_difference_matched_base_epochs = 0;
        std::size_t double_difference_interpolated_base_epochs = 0;
        std::size_t double_difference_candidate_pairs = 0;
        std::size_t double_difference_rejected_no_base_epoch = 0;
        std::size_t double_difference_rejected_no_reference = 0;
        std::size_t tdcp_candidate_pairs = 0;
        std::size_t tdcp_rejected_gap = 0;
        std::size_t tdcp_rejected_clock_discontinuity = 0;
        std::size_t tdcp_rejected_missing_previous = 0;
        std::size_t tdcp_rejected_loss_of_lock = 0;
        std::size_t tdcp_rejected_invalid_measurement = 0;
        std::size_t tdcp_rejected_code_phase_jump = 0;
        // Phase117 official SNR/type sigma could not be formed in native
        // metres (missing/nonfinite SNR, percentile, type, or wavelength).
        // No legacy sigma fallback is allowed when this candidate is on.
        std::size_t tdcp_rejected_invalid_weight = 0;
        std::map<SignalType, TdcpSignalDiagnostics>
            tdcp_signal_diagnostics;
        std::size_t residual_ionosphere_invalid_coefficients = 0;
        std::size_t source_rover_epoch_states_built = 0;
        std::size_t source_rover_missing_ephemeris_satellite_epochs = 0;
        std::size_t residual_ionosphere_candidate_rows = 0;
        std::size_t code_minus_carrier_jump_resets = 0;       ///< CMC screening: arc breaks forced
        std::size_t geometry_free_cycle_slip_resets = 0;      ///< confirmed geometry-free band resets
        std::size_t code_minus_carrier_level_exclusions = 0;  ///< CMC screening: (sat,signal) epochs excluded
        std::size_t cmc_ref_avoided_count = 0;  ///< cmc_aware_reference_selection: references changed away from a CMC-excluded candidate
        // Raw upstream residual/SNR quality contract diagnostics.  These are
        // populated only when FGOConfig::use_upstream_observable_quality is
        // enabled and are otherwise zero, preserving the default graph.
        double upstream_snr_l1_dbhz =
            std::numeric_limits<double>::quiet_NaN();
        double upstream_snr_l5_dbhz =
            std::numeric_limits<double>::quiet_NaN();
        std::size_t upstream_pseudorange_candidates = 0;
        std::size_t upstream_doppler_candidates = 0;
        std::size_t upstream_pseudorange_factors = 0;
        std::size_t upstream_doppler_factors = 0;
        std::size_t upstream_pd_pair_rejections = 0;
        std::size_t upstream_ld_pair_rejections = 0;
        std::size_t upstream_doppler_residual_rejections = 0;
        std::size_t upstream_pseudorange_residual_rejections = 0;
        std::size_t upstream_absolute_doppler_candidates = 0;
        std::size_t upstream_absolute_doppler_factors = 0;
        std::size_t upstream_absolute_doppler_rejections = 0;
        std::size_t upstream_absolute_doppler_missing_clock = 0;
        double upstream_absolute_doppler_max_abs_corrected_residual = 0.0;
        // Source-specific Galileo E1 group-delay selection diagnostics. These
        // remain zero for the legacy/default path.
        std::size_t galileo_e1_fnav_group_delay_rows = 0;
        std::size_t galileo_e1_inav_group_delay_rows = 0;
        std::size_t galileo_e1_group_delay_source_fallback_rows = 0;
        std::size_t galileo_e1_group_delay_invalid_rows = 0;
        // Native Android ReceivedSvTimeUncertaintyNanos sigma-floor telemetry
        // over the final FGO pseudorange-factor population after existing
        // masks.  These remain zero/false when the option is disabled.
        bool native_android_sv_time_uncertainty_sigma_floor_enabled = false;
        std::size_t native_android_sv_time_uncertainty_rows_applied = 0;
        std::size_t native_android_sv_time_uncertainty_rows_fallback = 0;
        std::size_t native_android_sv_time_uncertainty_factors_affected = 0;
        double native_android_sv_time_uncertainty_floor_min_m = 0.0;
        double native_android_sv_time_uncertainty_floor_median_m = 0.0;
        double native_android_sv_time_uncertainty_floor_p95_m = 0.0;
        double native_android_sv_time_uncertainty_floor_max_m = 0.0;
        // Phase58 raw Android C/N0/Doppler calibration telemetry over the
        // final adopted undifferenced FGO Doppler population.  These remain
        // zero/false when the explicit opt-in is disabled.
        bool native_cn0_doppler_calibration_enabled = false;
        std::size_t native_cn0_doppler_calibration_candidate_rows = 0;
        std::size_t native_cn0_doppler_calibration_finite_cn0_rows = 0;
        std::size_t native_cn0_doppler_calibration_fallback_rows = 0;
        std::size_t native_cn0_doppler_calibration_factors_affected = 0;
        double native_cn0_doppler_calibration_alpha_mps = 0.0;
        double native_cn0_doppler_calibration_reference_cn0_dbhz = 0.0;
        double native_cn0_doppler_calibration_model_sigma_min_mps = 0.0;
        double native_cn0_doppler_calibration_model_sigma_median_mps = 0.0;
        double native_cn0_doppler_calibration_model_sigma_p95_mps = 0.0;
        double native_cn0_doppler_calibration_model_sigma_max_mps = 0.0;
    };

    // --- Phase 2 milestone 2b: IMU preintegration inputs ---
    //
    // Continuous-time IMU noise densities + bias random-walk for the GTSAM
    // PreintegrationCombinedParams. Sigmas, not covariances; the backend
    // squares them. Defaults are a reasonable consumer/industrial MEMS grade
    // (order of the tokyo low-cost preset) and can be overridden by the caller.
    // Defaults are the (deliberately conservative) values validated on tokyo1
    // in milestone 2b; final tuning is deferred to 2c/2e.
    struct ImuNoiseParams {
        double accel_noise_sigma = 1.0e-1;      ///< accel white noise [m/s^2/sqrt(Hz)]
        double gyro_noise_sigma = 1.0e-2;       ///< gyro white noise [rad/s/sqrt(Hz)]
        double accel_bias_rw_sigma = 1.0e-2;    ///< accel bias random walk [m/s^3/sqrt(Hz)]
        double gyro_bias_rw_sigma = 1.0e-3;     ///< gyro bias random walk [rad/s^2/sqrt(Hz)]
        double integration_sigma = 1.0e-2;      ///< integration uncertainty [m/s/sqrt(Hz)]
        double gravity_mps2 = 9.80665;          ///< local gravity magnitude
    };

    // Everything the GTSAM backend needs to add IMU factors, computed by the
    // caller (harness): the local nav (ENU) frame definition, the per-sample
    // IMU stream already remapped to body FLU with gyro in rad/s, the initial
    // navigation state (from Stage-1 alignment) and its prior sigmas, and the
    // preintegration noise. The nav frame is ENU (Z-up); gravity points to -Z.
    struct ImuInput {
        bool valid = false;
        // Local ENU nav-frame origin: pose translations are expressed as the
        // body/IMU origin in this ENU frame, and ecef_T_nav maps them to ECEF.
        Vector3d nav_origin_ecef = Vector3d::Zero();
        double nav_origin_lat_rad = 0.0;
        double nav_origin_lon_rad = 0.0;
        // Time-sorted IMU samples, already body-FLU (ImuAxisConvention applied)
        // with gyro converted to rad/s. The backend preintegrates the samples
        // falling in each [epoch[i].time, epoch[i+1].time) interval.
        std::vector<ImuSample> samples_body_flu;
        // Initial navigation state (first epoch), nav = ENU frame. Attitude is
        // the body->nav rotation from Stage-1 static leveling + heading align.
        Matrix3d init_attitude_body_to_nav = Matrix3d::Identity();
        // Same-run native heading seeds, keyed to the exact problem epochs.
        // No serialized positioning input; consumed only by explicit selector.
        std::vector<Matrix3d> epoch_heading_attitudes_body_to_nav;
        std::vector<GNSSTime> epoch_heading_attitude_times;
        Vector3d init_velocity_nav = Vector3d::Zero();
        // Optional raw GNSS-first ENU velocity sequence used by the upstream
        // stationary-stop gate.  It is populated only by the Android
        // GNSS-first handoff; an empty vector makes the backend use its normal
        // per-epoch graph seeds.  This is not a truth or file-derived state.
        std::vector<Vector3d> stop_velocity_seeds_nav;
        Vector3d init_accel_bias = Vector3d::Zero();
        Vector3d init_gyro_bias = Vector3d::Zero();
        // First-state prior sigmas (gauge/anchor for the IMU chain).
        double init_attitude_sigma_roll_pitch_rad = 0.02;
        double init_attitude_sigma_yaw_rad = 0.5;
        double init_velocity_sigma_mps = 0.5;
        double init_accel_bias_sigma = 0.05;
        double init_gyro_bias_sigma = 0.01;
        ImuNoiseParams noise;
    };

    /**
     * @brief A truth-free native PDC state used as an optional initializer.
     *
     * This is intentionally a value object, not a coordinate-file interface.
     * The smartphone bridge constructs it in memory from the same
     * PseudorangeFactor/UndifferencedDopplerFactor rows consumed by FGO.  A
     * backend may reject an individual component by its `has_*` flag. The
     * P/D measurements are not duplicated as priors; the production/default
     * graph remains unchanged when the bridge config flag is false.
     */
    // State-only handoff from the in-process native PDC solve. This is an
    // initializer/diagnostic record, not an additional graph factor: the same
    // raw P/D observations remain owned by the normal FGO graph.
    struct NativePdcStateSeed {
        std::size_t epoch_index = 0;
        Vector3d position_ecef = Vector3d::Zero();
        Vector3d velocity_ecef_mps = Vector3d::Zero();
        std::array<double, 5> clock_bias_m = {0.0, 0.0, 0.0, 0.0, 0.0};
        double clock_rate_mps = 0.0;
        double position_sigma_m = 0.0;
        double velocity_sigma_mps = 0.0;
        double clock_sigma_m = 0.0;
        double clock_rate_sigma_mps = 0.0;
        int pseudorange_rows = 0;
        int doppler_rows = 0;
        int rank = 0;
        double condition_number = std::numeric_limits<double>::infinity();
        double normalized_pseudorange_rms =
            std::numeric_limits<double>::infinity();
        bool has_position = false;
        bool has_velocity = false;
        bool has_clock = false;
        bool has_clock_rate = false;
    };

    struct FGOProblem {
        std::vector<EpochSeed> epochs;
        ImuInput imu;  ///< Milestone 2b IMU inputs (valid only when populated).
        std::vector<bool> clock_jumps;
        // Diagnostic copy of the signed common GPS pseudorange change used by
        // the legacy positive-only clock_jumps detector, plus its support.
        // Neither field is consumed by an optimizer.
        std::vector<double> gps_common_pseudorange_delta_m;
        std::vector<int> gps_common_pseudorange_delta_satellites;
        std::vector<PseudorangeFactor> pseudorange_factors;
        // Ephemeral, pre-residual-mask population. No serialization/input
        // interface; created only by the explicitly selected raw builder.
        std::vector<PseudorangeFactor> native_pseudorange_remasking_pool;
        // Complete same-run factors passing temporal-candidate and baseline
        // residual gates, excluded from ordinary graph and median estimation.
        std::vector<PseudorangeFactor> native_code_edge_readmission_pool;
        std::vector<TimeDifferencedCarrierFactor> tdcp_factors;
        std::vector<UndifferencedDopplerFactor> undifferenced_doppler_factors;
        // Same-run corrected ECEF D rows, explicitly remapped to these
        // epochs. Never populate from optimized or saved velocity results.
        std::vector<UndifferencedDopplerFactor> native_phase213_main_doppler_rows;
        std::vector<doppler_velocity_wls::Estimate>
            doppler_velocity_wls_estimates;
        std::vector<SingleDifferenceDopplerFactor> single_difference_doppler_factors;
        std::vector<SingleDifferenceTdcpFactor> single_difference_tdcp_factors;
        std::vector<AmbiguityState> ambiguity_states;
        std::vector<CarrierPhaseFactor> carrier_observations;
        std::vector<CarrierPhaseFactor> double_difference_pseudorange_observations;
        std::vector<CarrierPhaseFactor> double_difference_reference_observations;
        std::vector<CarrierPhaseFactor> carrier_phase_factors;
        std::vector<DoubleDifferencePseudorangeFactor> double_difference_pseudorange_factors;
        std::vector<DoubleDifferenceCarrierFactor> double_difference_carrier_factors;
        // DD carrier rows dropped BEFORE reaching double_difference_carrier_
        // factors above by a build-time exclusion (currently: CMC sustained-
        // multipath level exclusion, code_minus_carrier_level_threshold_m,
        // when NOT running in code_minus_carrier_level_pseudorange_only
        // mode -- that mode keeps the DD-CP factor and only excludes the
        // DD-PR factor, so nothing needs to be retained here for it).  NEVER
        // added to the solved graph; consumed only by the surplus-satellite
        // independent integrity validation (FGOConfig::use_surplus_
        // satellite_validation) as its "observations excluded from the fix"
        // pool. ambiguity_index on these entries is a sentinel
        // (std::numeric_limits<std::size_t>::max()) -- there is no
        // AmbiguityState for an arc that was never solved for; the surplus
        // validator looks up wavelength from `signal` instead.
        std::vector<DoubleDifferenceCarrierFactor> excluded_double_difference_carrier_factors;
        std::vector<AmbiguityBetweenFactor> ambiguity_between_factors;
        // Optional in-memory PDC state bridge.  Empty unless a caller has
        // explicitly enabled and populated the research-only bridge.
        std::vector<NativePdcStateSeed> native_pdc_state_seeds;
        // Phase164 dedicated GNSS-only handoff. Entries are produced by the
        // same-run raw-P adapter; no serialized seed is valid here.
        std::vector<raw_p_seed::RawPNoDopplerSeed>
            native_raw_p_no_doppler_seeds;
        // Phase93 same-run GNSS-first clock-drift handoff.  When the
        // source-meter C0D handoff selector is active for the main graph, this
        // contains exactly one optimized GNSS-first D_i [m/s] per retained
        // epoch.  It is an initializer only; the main graph still owns the
        // C0/D factors and must fail closed when coverage is missing/nonfinite.
        std::vector<double>
            native_source_clock_c0d_gnss_first_d_handoff_mps;
        // Phase101 same-run handoff of the complete optimized official C_i
        // vector.  Entries are exact retained-epoch order and metres; an
        // active candidate must reject missing, nonfinite, or misaligned
        // coverage rather than falling back to scalar/global ISB states.
        std::vector<EpochClockBiasComponentsM>
            native_source_clock_c0d_gnss_first_c_handoff_m;
        // Phase114 same-run direct-WLS main seed.  These vectors are populated
        // only by the explicit raw entry-point adapter after exact retained
        // source-key validation.  Position/scalar-clock remain in `epochs`;
        // C7 and D are copied here so the backend cannot accidentally select
        // the GNSS-first result, a global ISB, or a raw/zero fallback.
        std::vector<Vector3d>
            native_direct_wls_ephemeral_velocity_ecef_mps;
        std::vector<double>
            native_direct_wls_ephemeral_d_handoff_mps;
        std::vector<EpochClockBiasComponentsM>
            native_direct_wls_ephemeral_c_handoff_m;
        FGOProblemDiagnostics diagnostics;
    };

    struct GeometryFreeSlipShadowEpoch {
        int event_pairs = 0;
        double max_jump_m = 0.0;
        int tainted_ambiguities = 0;
        int doppler_event_signals = 0;
        double doppler_max_innovation_m = 0.0;
        int doppler_isolated_event_pairs = 0;
        std::set<std::pair<SatelliteId, SignalType>> event_satellite_signals;
    };

    enum class TemporalCarrierShadowClassification {
        Clean = 0,
        WitnessedOutlier = 1,
        UnexplainedOutlier = 2,
    };

    /// Diagnostic-only classification of one receiver-clock-free temporal
    /// carrier difference.  None of these fields has estimator authority.
    struct TemporalCarrierShadowFactorDiagnostics {
        SingleDifferenceTdcpFactor factor;
        double residual_m = 0.0;
        double normalized_residual = 0.0;
        bool residual_outlier = false;
        bool doppler_evaluated = false;
        double doppler_innovation_signed_m = 0.0;
        double doppler_innovation_m = 0.0;
        double doppler_innovation_sigma_m = 0.0;
        double normalized_doppler_innovation = 0.0;
        bool doppler_outlier = false;
        bool doppler_calibration_evaluated = false;
        double doppler_bias_m = 0.0;
        double doppler_calibrated_scale_m = 0.0;
        double doppler_centered_innovation_m = 0.0;
        double doppler_calibrated_score = 0.0;
        bool doppler_calibrated_outlier = false;
        bool geometry_free_witness = false;
        bool carrier_hold_witness = false;
        bool carrier_fde_witness = false;
        TemporalCarrierShadowClassification classification =
            TemporalCarrierShadowClassification::Clean;
    };

    enum class PredictedDdprQualityAction {
        Unavailable = 0,
        Keep = 1,
        Downweight = 2,
    };

    /// Causal, diagnostic-only temporal quality check for one DD pseudorange
    /// row. The previous position is the already-solved prior epoch and the
    /// current position is the pre-solve IMU prediction. No field has
    /// estimator authority.
    struct PredictedDdprQualityFactorDiagnostics {
        std::size_t previous_epoch_index = 0;
        std::size_t current_epoch_index = 0;
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        double dt_s = 0.0;
        int pair_age_epochs = 0;
        double measured_ddpr_change_m = 0.0;
        bool doppler_evaluated = false;
        double doppler_predicted_change_m = 0.0;
        double doppler_innovation_m = 0.0;
        double doppler_innovation_sigma_m = 0.0;
        double normalized_doppler_innovation = 0.0;
        bool imu_geometry_evaluated = false;
        double imu_predicted_change_m = 0.0;
        double previous_predicted_ddpr_residual_m = 0.0;
        double current_predicted_ddpr_residual_m = 0.0;
        double imu_innovation_m = 0.0;
        double imu_innovation_sigma_m = 0.0;
        double normalized_imu_innovation = 0.0;
        double elevation_rad = 0.0;
        double target_snr_dbhz = 0.0;
        double reference_snr_dbhz = 0.0;
        PredictedDdprQualityAction proposed_action =
            PredictedDdprQualityAction::Unavailable;
    };

    /// Causal, diagnostic-only scalar bias prediction for one DD pseudorange
    /// pair. `prior_bias_m` is formed solely from earlier rows. The current
    /// residual is assimilated only after this diagnostic has been formed.
    struct PredictedDdprBiasStateDiagnostics {
        std::size_t previous_epoch_index = 0;
        std::size_t current_epoch_index = 0;
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        double dt_s = 0.0;
        int pair_age_epochs = 0;
        int prior_updates = 0;
        bool continuity_reset = false;
        bool prediction_usable = false;
        bool update_applied = false;
        bool update_clipped = false;
        double raw_residual_m = 0.0;
        double prior_bias_m = 0.0;
        double prior_sigma_m = 0.0;
        double corrected_residual_m = 0.0;
        double measurement_sigma_m = 0.0;
        double innovation_sigma_m = 0.0;
        double normalized_innovation = 0.0;
        double applied_innovation_m = 0.0;
        double posterior_bias_m = 0.0;
        double posterior_sigma_m = 0.0;
    };

    struct SatelliteQuarantineWitnessDiagnostics {
        std::size_t epoch_index = 0;
        SatelliteId satellite;
        double postfit_ddpr_residual_m = 0.0;
        double epoch_median_postfit_ddpr_residual_m = 0.0;
        bool postfit_gross = false;
        bool doppler_evaluated = false;
        bool doppler_outlier = false;
        bool imu_evaluated = false;
        bool imu_outlier = false;
        int temporal_support_pairs = 0;
        bool quarantine_candidate = false;
    };

    // Phase96 read-only root-cause telemetry.  These records deliberately
    // describe the graph and the already-pinned LM trial surface only; they
    // do not expose a solution and are empty unless the explicit opt-in
    // configuration flag is enabled.
    struct FGOPhase96FactorFamilyDiagnostics {
        std::string family;
        std::size_t factor_count = 0;
        std::size_t finite_factor_count = 0;
        std::size_t nonfinite_factor_count = 0;
        double initial_cost = 0.0;
    };

    struct FGOPhase96VariableFamilyNormDiagnostics {
        std::string family;
        std::string variable_bucket;
        std::size_t contribution_count = 0;
        std::size_t finite_contribution_count = 0;
        std::size_t nonfinite_contribution_count = 0;
        double gradient_l2_norm = 0.0;
        double normal_diagonal_l2_norm = 0.0;
        double normal_diagonal_min =
            std::numeric_limits<double>::quiet_NaN();
        double normal_diagonal_max =
            std::numeric_limits<double>::quiet_NaN();
    };

    struct FGOPhase96LmTrialDiagnostics {
        std::size_t trial_index = 0;
        std::size_t outer_iteration = 0;
        double lambda = std::numeric_limits<double>::quiet_NaN();
        double old_linearized_cost =
            std::numeric_limits<double>::quiet_NaN();
        double new_linearized_cost =
            std::numeric_limits<double>::quiet_NaN();
        double predicted_reduction =
            std::numeric_limits<double>::quiet_NaN();
        double candidate_nonlinear_cost =
            std::numeric_limits<double>::quiet_NaN();
        double actual_reduction =
            std::numeric_limits<double>::quiet_NaN();
        double model_fidelity = std::numeric_limits<double>::quiet_NaN();
        bool candidate_finite = false;
        bool linear_system_solved = false;
        std::string linear_system_status;
        std::string rejection_reason;
    };

    struct FGOPhase96ExceptionDiagnostics {
        std::string stage;
        std::string classification;
        std::string type;
        std::string message;
        std::size_t count = 1;
    };

    struct FGOPhase96MainDiagnostics {
        bool enabled = false;
        bool attempted = false;
        bool graph_observed = false;
        bool initial_linearization_observed = false;
        bool trial_trace_complete = false;
        std::size_t trial_limit = 10;
        std::size_t graph_factor_count = 0;
        std::size_t graph_value_count = 0;
        double graph_initial_cost =
            std::numeric_limits<double>::quiet_NaN();
        double initial_cost = std::numeric_limits<double>::quiet_NaN();
        double final_cost = std::numeric_limits<double>::quiet_NaN();
        std::size_t accepted_outer_iterations = 0;
        std::string terminal_branch;
        double factor_family_cost_sum =
            std::numeric_limits<double>::quiet_NaN();
        std::vector<FGOPhase96FactorFamilyDiagnostics> factor_families;
        std::vector<FGOPhase96VariableFamilyNormDiagnostics>
            variable_family_norms;
        std::vector<FGOPhase96LmTrialDiagnostics> lm_trials;
        std::vector<FGOPhase96ExceptionDiagnostics> exceptions;
    };

    // Phase97 read-only singular-system telemetry.  Key references retain the
    // exact GTSAM integer key and its Symbol decomposition; no coordinate or
    // optimized-state value is carried in these records.
    struct FGOPhase97KeyReference {
        std::uint64_t numeric_key = 0;
        char symbol_character = 0;
        std::uint64_t symbol_index = 0;
    };

    struct FGOPhase97FamilyDegreeDiagnostics {
        std::string family;
        std::size_t factor_degree = 0;
    };

    struct FGOPhase97KeyDiagnostics {
        FGOPhase97KeyReference key;
        std::string variable_bucket;
        std::string value_type;
        std::size_t value_dimension = 0;
        bool value_present = false;
        std::size_t factor_degree = 0;
        std::size_t prior_factor_degree = 0;
        std::size_t component_id = std::numeric_limits<std::size_t>::max();
        std::size_t linearized_contribution_count = 0;
        std::size_t finite_linearized_contribution_count = 0;
        std::size_t nonfinite_linearized_contribution_count = 0;
        double gradient_l2_norm = std::numeric_limits<double>::quiet_NaN();
        double normal_diagonal_l2_norm =
            std::numeric_limits<double>::quiet_NaN();
        double normal_diagonal_min = std::numeric_limits<double>::quiet_NaN();
        double normal_diagonal_max = std::numeric_limits<double>::quiet_NaN();
        std::size_t exact_zero_normal_diagonal_count = 0;
        std::size_t near_zero_normal_diagonal_count = 0;
        bool exact_zero_column = false;
        bool near_zero_column = false;
        // Aggregate keyed linearized vectors.  These contain Jacobian-derived
        // gradient/normal entries only, never a state estimate.
        std::vector<double> gradient;
        std::vector<double> normal_diagonal;
        std::vector<FGOPhase97FamilyDegreeDiagnostics> family_degrees;
    };

    struct FGOPhase97FactorDiagnostics {
        std::size_t graph_index = 0;
        std::string concrete_factor_type;
        std::string family;
        std::size_t factor_dimension = 0;
        bool finite_error = false;
        bool is_prior_or_anchor = false;
        std::vector<FGOPhase97KeyReference> keys;
    };

    struct FGOPhase97ComponentDiagnostics {
        std::size_t component_id = 0;
        std::size_t key_count = 0;
        std::size_t factor_count = 0;
        bool anchored = false;
        std::vector<FGOPhase97KeyReference> keys;
    };

    struct FGOPhase97RankDiagnostics {
        bool attempted = false;
        bool rank_known = false;
        bool nullity_known = false;
        bool finite = true;
        std::size_t row_count = 0;
        std::size_t column_count = 0;
        std::size_t rank = 0;
        std::size_t nullity = 0;
        std::string method;
        std::string status;
        double threshold = std::numeric_limits<double>::quiet_NaN();
        std::vector<FGOPhase97KeyReference> nullspace_attribution;
    };

    struct FGOPhase97LmTrialDiagnostics {
        std::size_t trial_index = 0;
        std::size_t outer_iteration = 0;
        double lambda = std::numeric_limits<double>::quiet_NaN();
        double old_linearized_cost =
            std::numeric_limits<double>::quiet_NaN();
        double new_linearized_cost =
            std::numeric_limits<double>::quiet_NaN();
        double predicted_reduction =
            std::numeric_limits<double>::quiet_NaN();
        double candidate_nonlinear_cost =
            std::numeric_limits<double>::quiet_NaN();
        double actual_reduction = std::numeric_limits<double>::quiet_NaN();
        double model_fidelity = std::numeric_limits<double>::quiet_NaN();
        bool candidate_finite = false;
        bool linear_system_solved = false;
        std::string linear_system_status;
        std::string rejection_reason;
        bool nearby_variable_available = false;
        FGOPhase97KeyReference nearby_variable;
        std::string nearby_variable_status;
    };

    struct FGOPhase97MainDiagnostics {
        bool enabled = false;
        bool attempted = false;
        bool graph_observed = false;
        bool initial_linearization_observed = false;
        bool diagnostic_complete = false;
        std::size_t graph_factor_count = 0;
        std::size_t graph_value_count = 0;
        std::size_t graph_value_dimension = 0;
        double graph_initial_cost =
            std::numeric_limits<double>::quiet_NaN();
        double initial_cost = std::numeric_limits<double>::quiet_NaN();
        double final_cost = std::numeric_limits<double>::quiet_NaN();
        std::size_t accepted_outer_iterations = 0;
        std::size_t trial_limit = 10;
        bool trial_trace_complete = false;
        std::string terminal_branch;
        std::size_t missing_factor_key_count = 0;
        std::size_t duplicate_key_reference_count = 0;
        std::size_t value_type_mismatch_count = 0;
        std::size_t empty_factor_count = 0;
        std::size_t isolated_value_key_count = 0;
        std::size_t connected_component_count = 0;
        std::size_t exact_zero_column_count = 0;
        std::size_t near_zero_column_count = 0;
        double near_zero_threshold = 1.0e-12;
        std::string nearby_variable_capture_status =
            "not-observed";
        std::vector<FGOPhase97KeyReference> nearby_variables;
        std::vector<FGOPhase97KeyReference> missing_factor_keys;
        std::vector<FGOPhase97KeyReference> duplicate_key_reference_keys;
        std::vector<FGOPhase97KeyReference> value_type_mismatch_keys;
        std::string ordering_context = "initial Values::keys() ascending";
        std::vector<FGOPhase97KeyDiagnostics> keys;
        std::vector<FGOPhase97FactorDiagnostics> factors;
        std::vector<FGOPhase97ComponentDiagnostics> components;
        std::vector<FGOPhase97RankDiagnostics> rank_decompositions;
        std::vector<FGOPhase97LmTrialDiagnostics> lm_trials;
        std::vector<FGOPhase96ExceptionDiagnostics> exceptions;
    };

    // Phase98 read-only solver-boundary telemetry.  This compact sidecar
    // records only metadata from the existing pinned LM invocation and an
    // actual IndeterminantLinearSystemException when that exception reaches
    // the integration boundary.  It intentionally carries no Values,
    // coordinates, residuals, or solution rows.
    struct FGOPhase98IndeterminateExceptionDiagnostics {
        std::string stage;
        double lambda = std::numeric_limits<double>::quiet_NaN();
        std::string solver_type;
        std::string solver_branch;
        std::string elimination_function;
        std::string ordering_type;
        bool explicit_ordering_present = false;
        std::size_t ordering_size = 0;
        std::string ordering_digest;
        bool diagonal_damping = false;
        bool nearby_variable_available = false;
        FGOPhase97KeyReference nearby_variable;
        std::string nearby_variable_status = "nearby_variable_unavailable";
        std::string exception_type;
        std::string exception_message;
    };

    struct FGOPhase98SolverDiagnostics {
        bool enabled = false;
        bool attempted = false;
        bool exception_captured = false;
        std::string solver_type;
        std::string solver_branch;
        std::string elimination_function;
        std::string ordering_type;
        bool explicit_ordering_present = false;
        std::size_t ordering_size = 0;
        std::string ordering_digest;
        bool diagonal_damping = false;
        std::size_t existing_lm_trial_limit = 10;
        std::vector<FGOPhase98IndeterminateExceptionDiagnostics>
            indeterminate_exceptions;
    };

    // Phase143 native-authoritative termination telemetry.  This object is a
    // scalar sidecar for one unchanged GTSAM LM call; it deliberately carries
    // no Values, residual rows, coordinates, or solution output.  The
    // termination_branch is an enum-valued string from the frozen set rather
    // than a free-form wrapper interpretation.
    struct FGOPhase143TerminationDiagnostics {
        bool selector_enabled = false;
        std::string stage = "disabled";
        std::size_t configured_max_iterations = 0;
        std::size_t effective_max_iterations = 0;
        bool attempted = false;
        std::size_t attempted_outer_iterations = 0;
        std::size_t accepted_outer_iterations = 0;
        std::size_t rejected_outer_iterations = 0;
        std::size_t total_inner_lambda_attempts = 0;
        // Native trace accounting is retained even when the strict
        // completeness contract rejects the solve.  These are diagnostic
        // counters only; they never reconstruct or promote a solution.
        std::size_t parsed_trial_count = 0;
        std::size_t native_inner_iterations = 0;
        std::size_t expected_trial_count = 0;
        double initial_cost = std::numeric_limits<double>::quiet_NaN();
        double final_cost = std::numeric_limits<double>::quiet_NaN();
        bool costs_finite = false;
        bool strict_cost_decrease = false;
        std::string termination_branch;
        double relative_error_tolerance =
            std::numeric_limits<double>::quiet_NaN();
        double absolute_error_tolerance =
            std::numeric_limits<double>::quiet_NaN();
        double error_tolerance = std::numeric_limits<double>::quiet_NaN();
        double initial_lambda = std::numeric_limits<double>::quiet_NaN();
        double final_lambda = std::numeric_limits<double>::quiet_NaN();
        double maximum_lambda = std::numeric_limits<double>::quiet_NaN();
        double lambda_factor = std::numeric_limits<double>::quiet_NaN();
        double lambda_lower_bound = std::numeric_limits<double>::quiet_NaN();
        double lambda_upper_bound = std::numeric_limits<double>::quiet_NaN();
        double min_model_fidelity =
            std::numeric_limits<double>::quiet_NaN();
        bool diagonal_damping = false;
        bool use_fixed_lambda_factor = false;
        std::string linear_solver;
        std::string elimination;
        std::string ordering_type;
        bool explicit_ordering_present = false;
        bool no_fallback = true;
        bool termination_trace_complete = false;
        bool configuration_valid = true;
        std::string configuration_failure;
    };

    struct FGODiagnostics {
        int iterations = 0;
        bool converged = false;
        std::size_t epochs = 0;
        std::size_t sparse_epochs_retained = 0;
        std::size_t sparse_empty_epochs_retained = 0;
        std::size_t pseudorange_factors = 0;
        std::size_t receiver_signal_bias_factors = 0;
        std::size_t receiver_signal_bias_states = 0;
        std::size_t residual_ionosphere_factors = 0;
        std::size_t residual_ionosphere_states = 0;
        std::size_t residual_ionosphere_resets = 0;
        std::size_t residual_ionosphere_invalid_coefficients = 0;
        double residual_ionosphere_max_abs_m = 0.0;
        double residual_ionosphere_rms_m = 0.0;
        double residual_ionosphere_min_coefficient = 0.0;
        double residual_ionosphere_max_coefficient = 0.0;
        /// TDCP measurements present in the backend-independent problem.
        std::size_t tdcp_factors = 0;
        /// TDCP residual rows/factors actually inserted by the selected backend.
        std::size_t tdcp_factors_inserted = 0;
        std::size_t tdcp_only_affine_factors_inserted = 0;
        std::size_t epoch_heading_attitude_seeds_inserted = 0;
        std::size_t first_imu_bias_priors_inserted = 0;
        std::size_t first_imu_bias_priors_omitted = 0;
        std::size_t first_imu_velocity_priors_inserted = 0;
        std::size_t first_imu_velocity_priors_omitted = 0;
        std::size_t relative_height_pairs_selected = 0;
        std::size_t relative_height_factors_inserted = 0;
        // Phase118 official route-Type Huber-k metadata.  These fields are
        // provenance only; the selected threshold is applied only to ordinary
        // TDCP factors and never changes sigma, equations, admission, or any
        // other robust kernel.
        bool official_tdcp_huber_k_enabled = false;
        std::string official_tdcp_setting_type;
        double official_tdcp_huber_threshold_sigma = 4.0;
        // Phase184 dedicated Phase171 source Type mapping.  This is kept
        // separate from the Phase118 provenance fields above.
        bool native_phase184_source_tdcp_huber_k_enabled = false;
        std::string native_phase184_tdcp_setting_type;
        // Phase120 source-parity metadata.  This flag reports only the
        // ordinary TDCP measurement preparation selector; it does not imply
        // that standalone carrier, pseudorange, or double-difference values
        // were changed.
        bool official_tdcp_resl_atmosphere_cancellation_enabled = false;
        // Phase143 native-authoritative LM termination report.  It is
        // populated only when the Phase143 selector is enabled; selector-off
        // diagnostics retain the historical schema and values.
        FGOPhase143TerminationDiagnostics
            native_phase143_termination;
        // Phase135 compound fixed-initial-geometry affine-family witness.
        // These counters are populated only by the opt-in GTSAM adapter;
        // selector-off diagnostics retain their historical values.
        bool phase135_official_affine_measurement_family_enabled = false;
        bool phase135_configuration_valid = true;
        std::string phase135_configuration_failure;
        std::size_t phase135_pseudorange_factors_inserted = 0;
        std::size_t phase135_doppler_factors_inserted = 0;
        std::size_t phase135_tdcp_factors_inserted = 0;
        std::size_t phase135_pose3_x_bridge_factors = 0;
        std::size_t phase135_geometry_rows_validated = 0;
        std::string phase135_geometry_representation = "disabled";
        // Phase141 native-authoritative schema witnesses.  These are
        // populated by the already-selected affine insertion transaction;
        // they do not participate in factor construction or optimization.
        bool phase135_transactional = false;
        bool phase135_fixed_initial_geometry = false;
        bool phase135_finite_jacobians = false;
        bool phase135_single_sagnac_representation = false;
        bool phase135_pseudorange_key_order_exact = false;
        bool phase135_pseudorange_finite_values = false;
        bool phase135_pseudorange_source_geometry_same_path = false;
        bool phase135_doppler_key_order_exact = false;
        bool phase135_doppler_finite_values = false;
        bool phase135_doppler_source_geometry_same_path = false;
        bool phase135_tdcp_key_order_exact = false;
        bool phase135_tdcp_finite_values = false;
        bool phase135_tdcp_source_geometry_same_path = false;
        std::size_t phase135_legacy_pseudorange_factor_count = 0;
        std::size_t phase135_legacy_doppler_factor_count = 0;
        std::size_t phase135_legacy_tdcp_factor_count = 0;
        bool phase135_pose3_x_bridge_keys_exact = false;
        // Phase138 ordinary-TDCP measurement-constant witness.  The
        // correction is populated only by the opt-in Phase135 adapter; it
        // never changes the TDCP pair ledger, factor Jacobians, or count.
        bool phase138_affine_tdcp_anchor_range_constant_enabled = false;
        bool phase138_configuration_valid = true;
        std::string phase138_configuration_failure;
        std::size_t phase138_tdcp_range_constants_validated = 0;
        std::size_t phase138_tdcp_measurements_adjusted = 0;
        std::size_t phase138_adjustment_application_passes = 0;
        bool phase138_adjusted_exactly_once = false;
        bool phase138_factor_count_unchanged = true;
        std::string phase138_measurement_equation = "disabled";
        std::string phase138_geometry_representation = "disabled";
        bool phase138_same_endpoint_epoch_and_satellite_state = false;
        bool phase138_same_satellite_state = false;
        bool phase138_finite_adjusted_measurements = false;
        bool phase138_no_raw_or_zero_fallback = false;
        bool phase138_transactional = false;
        std::size_t phase138_legacy_tdcp_factor_count = 0;
        std::size_t undifferenced_doppler_factors = 0;
        /// Undifferenced Doppler rows actually inserted by the selected backend.
        /// This can differ from undifferenced_doppler_factors when a backend
        /// rejects a non-finite/invalid row or when a feature path is disabled.
        std::size_t undifferenced_doppler_factors_inserted = 0;
        /// Raw-observable quality residual diagnostics (candidate-only).
        double upstream_pseudorange_normalized_rms = 0.0;
        double upstream_doppler_normalized_rms = 0.0;
        // Truth-free per-epoch Doppler WLS initialization diagnostics.
        std::size_t doppler_velocity_wls_valid_epochs = 0;
        std::size_t doppler_velocity_wls_propagated_epochs = 0;
        std::size_t doppler_velocity_wls_rejected_epochs = 0;
        double doppler_velocity_wls_max_condition_number = 0.0;
        double doppler_velocity_wls_max_normalized_rms = 0.0;
        double doppler_velocity_wls_max_velocity_norm_mps = 0.0;
        double doppler_velocity_wls_max_clock_rate_abs_mps = 0.0;
        std::size_t single_difference_doppler_factors = 0;
        /// Satellite-single-difference TDCP measurements present in the problem.
        std::size_t single_difference_tdcp_factors = 0;
        /// Satellite-single-difference TDCP rows/factors actually inserted by the backend.
        std::size_t single_difference_tdcp_factors_inserted = 0;
        std::size_t carrier_phase_factors = 0;
        std::size_t double_difference_pseudorange_factors = 0;
        std::size_t double_difference_carrier_factors = 0;
        std::size_t ambiguity_states = 0;
        std::size_t ambiguity_fix_candidates = 0;
        std::size_t lambda_ambiguity_candidates = 0;
        std::size_t lambda_ambiguity_used_candidates = 0;
        std::size_t lambda_ambiguity_attempts = 0;
        /// Ambiguity keys removed before LAMBDA because they left the active smoother.
        std::size_t lambda_stale_candidates_filtered = 0;
        /// Exceptions while requesting the active pose/ambiguity joint marginal.
        std::size_t lambda_joint_marginal_failures = 0;
        std::size_t integer_constrained_reoptimization_attempts = 0;
        std::size_t integer_constrained_reoptimization_accepts = 0;
        std::size_t integer_constrained_reoptimization_rejects = 0;
        std::size_t imu_aided_ratio_accepts = 0;
        std::size_t imu_aided_ratio_rejects = 0;
        std::size_t fixed_history_dr_accepts = 0;
        std::size_t fixed_history_dr_rejects = 0;
        std::size_t fixed_history_dr_surplus_overrides = 0;  ///< surplus_validation_overrides_history_dr flipped a DR-rejected candidate to FIXED
        std::size_t fixed_history_dr_surplus_override_capped = 0;  ///< surplus_validation_overrides_history_dr_max_consecutive blocked an otherwise-qualifying override
        std::size_t ddpr_anchor_validation_accepts = 0;
        std::size_t ddpr_anchor_validation_rejects = 0;
        std::size_t fixed_postfit_validation_accepts = 0;
        std::size_t fixed_postfit_validation_rejects = 0;
        std::size_t external_doppler_dr_accepts = 0;
        std::size_t external_doppler_dr_rejects = 0;
        std::size_t external_doppler_dr_unavailable = 0;
        // --- Surplus-satellite independent integrity validation (use_surplus_satellite_validation) ---
        std::size_t surplus_validation_attempts = 0;   ///< LAMBDA attempts where the test rendered a verdict
        std::size_t surplus_validation_passes = 0;
        std::size_t surplus_validation_fails = 0;
        std::size_t surplus_validation_insufficient_surplus = 0;  ///< too few surplus sats at every fallback level
        std::size_t surplus_validation_rescued_epochs = 0;  ///< epochs FIXED only because this test passed a relaxed-ratio candidate
        std::size_t surplus_validation_separation_rejects = 0;  ///< surplus-pass rescues rejected by the existing fixed-vs-float/IMU separation aperture
        std::size_t surplus_validation_quality_rejects = 0;  ///< surplus-pass rescues rejected by the independent geometry/code-quality floor
        std::size_t surplus_validation_vetoed_epochs = 0;   ///< established-ratio fixes demoted by surplus_validation_veto_high_ratio_fails
        std::size_t surplus_validation_fallback_level_histogram[6] = {0, 0, 0, 0, 0, 0};  ///< index = deciding fallback level (0=GQEBR .. 5=GQ)
        // --- Below-floor low-count AR rescue (use_low_count_ambiguity_resolution) ---
        std::size_t low_count_ambiguity_attempts = 0;  ///< LAMBDA attempts made only because this knob lowered the floor
        std::size_t low_count_ambiguity_fix_accepted = 0;  ///< of which accepted (surplus pass [+ ratio floor])
        std::size_t fixed_ambiguities = 0;
        std::size_t tdcp_candidate_pairs = 0;
        std::size_t tdcp_rejected_gap = 0;
        std::size_t tdcp_rejected_missing_previous = 0;
        std::size_t tdcp_rejected_loss_of_lock = 0;
        std::size_t tdcp_rejected_code_phase_jump = 0;
        std::size_t double_difference_matched_base_epochs = 0;
        std::size_t double_difference_interpolated_base_epochs = 0;
        std::size_t double_difference_candidate_pairs = 0;
        std::size_t double_difference_rejected_no_base_epoch = 0;
        std::size_t double_difference_rejected_no_reference = 0;
        std::size_t motion_factors = 0;
        // Source-exact ClockFactor_CCDD C0/D telemetry.  These counters are
        // populated by the GTSAM Pose3+IMU candidate and remain zero when the
        // candidate is disabled.  The parity scope is one active C0/D row,
        // not the full seven-component source clock vector.
        bool native_source_clock_c0d_factor_enabled = false;
        // Phase164 dedicated no-D graph numerical-gauge provenance.  These
        // fields never imply that an unobserved C7 component was measured.
        bool native_raw_p_no_doppler_graph_enabled = false;
        double native_raw_p_no_doppler_unobserved_clock_gauge_sigma_m =
            std::numeric_limits<double>::quiet_NaN();
        std::size_t native_raw_p_no_doppler_unobserved_clock_gauge_components =
            0;
        // Phase171 GNSS-first raw-P+D staging provenance.  The velocity and
        // Doppler LOS rows remain ECEF in this Point3 graph; the main IMU
        // graph has a separate ENU handoff and keeps generic D empty.
        bool native_raw_p_ecef_doppler_gnss_first_enabled = false;
        std::size_t native_raw_p_ecef_doppler_unobserved_clock_gauge_components =
            0;
        // Phase171 reuses the same explicit weak numerical gauge for C7
        // components absent from the retained P rows in the Pose3+IMU main
        // graph.  It is a numerical gauge, never a measured ISB observation.
        bool native_phase171_no_doppler_imu_main_enabled = false;
        double native_phase171_unobserved_clock_gauge_sigma_m =
            std::numeric_limits<double>::quiet_NaN();
        std::size_t native_phase171_unobserved_clock_gauge_components = 0;
        // Phase201 source-inclusive-forward IMU schedule telemetry.  These
        // fields are populated only by the dedicated Phase171 opt-in branch;
        // the legacy preceding-delta/tail schedule does not write them.
        bool native_phase201_source_inclusive_forward_imu_schedule_enabled =
            false;
        bool native_phase201_source_inclusive_forward_imu_schedule_attempted =
            false;
        bool native_phase201_source_inclusive_forward_imu_schedule_configuration_valid =
            true;
        std::size_t native_phase201_intervals = 0;
        std::size_t native_phase201_intervals_with_samples = 0;
        std::size_t native_phase201_inserted_samples = 0;
        std::size_t native_phase201_invalid_sample_count = 0;
        std::size_t native_phase201_nonfinite_dt_count = 0;
        std::size_t native_phase201_nonpositive_dt_count = 0;
        std::size_t native_phase201_empty_interval_count = 0;
        double native_phase201_integrated_duration_min_s =
            std::numeric_limits<double>::quiet_NaN();
        double native_phase201_integrated_duration_max_s =
            std::numeric_limits<double>::quiet_NaN();
        double native_phase201_gnss_interval_duration_min_s =
            std::numeric_limits<double>::quiet_NaN();
        double native_phase201_gnss_interval_duration_max_s =
            std::numeric_limits<double>::quiet_NaN();
        double native_phase201_duration_error_min_s =
            std::numeric_limits<double>::quiet_NaN();
        double native_phase201_duration_error_max_s =
            std::numeric_limits<double>::quiet_NaN();
        double native_phase201_duration_error_max_abs_s =
            std::numeric_limits<double>::quiet_NaN();
        std::string native_phase201_configuration_failure;
        bool native_phase205_bias_density_enabled = false;
        bool native_phase209_separate_imu_enabled = false;
        bool native_phase213_main_doppler_enabled = false;
        bool native_phase217_main_motion_enabled = false;
        std::size_t native_phase217_main_motion_factors = 0;
        std::size_t native_phase217_main_motion_gap_skips = 0;
        std::size_t native_phase213_main_doppler_factors = 0;
        std::size_t native_phase209_motion_factors = 0;
        std::size_t native_phase209_bias_factors = 0;
        std::size_t native_phase209_inclusive_samples = 0;
        std::size_t native_phase205_bias_density_intervals = 0;
        std::size_t native_phase205_bias_density_samples = 0;
        double native_phase205_bias_density_scale_min = 0.0;
        double native_phase205_bias_density_scale_max = 0.0;
        // Phase92 opt-in state-unit parity.  C_i and global ISB_i are metres
        // inside the active GTSAM batch graph; D_i remains metres/second.
        // Public PositionSolution::receiver_clock_bias remains seconds after
        // one explicit division by C_LIGHT at the output boundary.
        bool native_source_clock_c0d_meter_state_parity_enabled = false;
        // Phase101 official epoch-local C-vector topology witness.  The
        // candidate uses seven metre components per retained epoch and never
        // inserts the legacy global `i` ISB keys.
        bool native_source_clock_c0d_epoch_vector_parity_enabled = false;
        std::size_t native_source_clock_c0d_epoch_vector_dimension = 0;
        std::size_t native_source_clock_c0d_epoch_vector_state_count = 0;
        std::size_t native_source_clock_c0d_epoch_vector_handoff_count = 0;
        std::size_t native_source_clock_c0d_global_isb_state_count = 0;
        std::size_t native_source_clock_c0d_factor_count = 0;
        // Phase88 active-solve diagnostics. These fields remain at their
        // defaults unless the opt-in diagnostic flag is enabled alongside
        // the source C0/D factor.
        bool native_source_clock_c0d_active_solve_diagnostic_enabled = false;
        bool native_source_clock_c0d_active_solve_attempted = false;
        std::size_t native_source_clock_c0d_accepted_outer_iterations = 0;
        std::size_t native_source_clock_c0d_total_inner_lambda_attempts = 0;
        std::size_t native_source_clock_c0d_indeterminate_linear_solve_count = 0;
        std::size_t native_source_clock_c0d_unsuccessful_model_step_count = 0;
        std::size_t native_source_clock_c0d_small_cost_change_stop_count = 0;
        std::size_t native_source_clock_c0d_maximum_lambda_stop_count = 0;
        double native_source_clock_c0d_initial_lambda = 0.0;
        double native_source_clock_c0d_maximum_lambda = 0.0;
        double native_source_clock_c0d_final_lambda = 0.0;
        double native_source_clock_c0d_max_whitened_clock_column_norm = 0.0;
        double native_source_clock_c0d_max_whitened_drift_column_norm = 0.0;
        double native_source_clock_c0d_conditioning_proxy = 0.0;
        bool native_source_clock_c0d_active_solve_finite_costs = false;
        bool native_source_clock_c0d_termination_trace_complete = false;
        std::string native_source_clock_c0d_termination_branch_reason;
        // Phase91 raw Android receiver-clock-drift D_i initializer. These
        // fields cover only the finite retained EpochSeed sequence; exact
        // cross-stream epoch identity is reported by the native entry point.
        bool native_source_clock_c0d_raw_drift_d_initializer_enabled = false;
        bool native_source_clock_c0d_raw_drift_d_initializer_attempted = false;
        bool native_source_clock_c0d_raw_drift_d_initializer_coverage_valid = false;
        std::size_t native_source_clock_c0d_raw_drift_d_initializer_epoch_count = 0;
        std::size_t native_source_clock_c0d_raw_drift_d_initializer_finite_count = 0;
        std::size_t native_source_clock_c0d_raw_drift_d_initializer_nonfinite_count = 0;
        double native_source_clock_c0d_raw_drift_d_initializer_min_mps =
            std::numeric_limits<double>::quiet_NaN();
        double native_source_clock_c0d_raw_drift_d_initializer_max_mps =
            std::numeric_limits<double>::quiet_NaN();
        std::string native_source_clock_c0d_raw_drift_d_initializer_failure;
        std::size_t native_source_clock_c0d_clock_jump_skips = 0;
        std::size_t native_source_clock_c0d_gap_skips = 0;
        std::size_t native_source_clock_c0d_invalid_dt_skips = 0;
        std::size_t native_source_clock_c0d_phone_exclusion_skips = 0;
        double native_source_clock_c0d_dt_min_s = 0.0;
        double native_source_clock_c0d_dt_max_s = 0.0;
        std::size_t native_source_clock_c0d_legacy_between_factor_count = 0;
        std::size_t ambiguity_between_factors = 0;
        std::size_t robust_pseudorange_factors = 0;
        std::size_t robust_carrier_phase_factors = 0;
        std::size_t robust_double_difference_pseudorange_factors = 0;
        std::size_t robust_double_difference_carrier_factors = 0;
        std::size_t robust_tdcp_factors = 0;
        std::size_t ddpr_gnc_evaluated_epochs = 0;
        std::size_t ddpr_gnc_factors = 0;
        std::size_t ddpr_gnc_downweighted_factors = 0;
        std::size_t ddpr_gnc_counterfactual_attempts = 0;
        std::size_t ddpr_gnc_counterfactual_successes = 0;
        std::size_t candidate_integrity_witness_evaluated = 0;
        std::size_t candidate_integrity_witness_passes = 0;
        std::size_t satellite_quarantine_witness_satellites = 0;
        std::size_t satellite_quarantine_candidates = 0;
        std::size_t selective_arc_restart_detection_epochs = 0;
        std::size_t selective_arc_restart_candidate_satellites = 0;
        std::size_t selective_arc_restart_candidate_pairs = 0;
        std::size_t selective_arc_restart_applied_arcs = 0;
        std::size_t selective_arc_restart_skipped_fixed = 0;
        std::size_t selective_arc_restart_skipped_thrash = 0;
        std::size_t selective_arc_restart_skipped_cap = 0;
        std::size_t selective_arc_restart_skipped_no_arc = 0;
        std::size_t graph_factors = 0;
        std::size_t graph_values = 0;
        std::size_t native_pdc_position_seeds = 0;
        std::size_t native_pdc_velocity_seeds = 0;
        std::size_t native_pdc_clock_seeds = 0;
        std::size_t native_pdc_clock_rate_seeds = 0;
        std::size_t imu_intervals = 0;  ///< 2b: CombinedImuFactors added between epochs
        std::size_t smoother_max_window_vars = 0;  ///< 2c: peak in-window variable count
        std::size_t smoother_updates = 0;          ///< 2c: number of smoother.update() calls
        std::size_t smoother_recovery_epochs = 0;  ///< 2e: epochs re-anchored after an indeterminate update
        std::size_t nhc_epochs = 0;   ///< 2d: epochs an NHC factor was applied
        std::size_t zupt_epochs = 0;  ///< 2d: epochs a ZUPT prior was applied
        std::size_t upstream_stop_epochs = 0;
        std::size_t upstream_stop_velocity_factors = 0;
        std::size_t upstream_stop_pose_factors = 0;
        std::size_t upstream_stop_imu_samples = 0;
        // Per-detected-epoch accounting for the upstream stop gate.  These
        // counters are scalar diagnostics only; they do not alter the gate
        // or provide a fallback factor/seed.  In particular, a graph
        // velocity is counted as a fallback only when the exact same-run
        // stop-seed vector is unavailable or nonfinite for that epoch.
        std::size_t upstream_stop_velocity_key_missing_epochs = 0;
        std::size_t upstream_stop_seed_unavailable_epochs = 0;
        std::size_t upstream_stop_seed_nonfinite_epochs = 0;
        std::size_t upstream_stop_graph_velocity_fallback_epochs = 0;
        std::size_t upstream_stop_graph_velocity_nonfinite_epochs = 0;
        std::size_t upstream_stop_speed_nonfinite_epochs = 0;
        std::size_t upstream_stop_speed_evaluated_epochs = 0;
        std::size_t upstream_stop_speed_gate_accepted_epochs = 0;
        std::size_t upstream_stop_speed_gate_rejected_epochs = 0;
        double upstream_stop_speed_min_mps = 0.0;
        double upstream_stop_speed_max_mps = 0.0;
        double upstream_stop_acceleration_std_threshold_mps2 = 0.0;
        double upstream_stop_gyro_std_threshold_radps = 0.0;
        std::size_t ambiguity_hold_epochs = 0;  ///< 2e: epochs FIXED via held (not fresh) integers
        std::size_t ambiguity_hold_arcs = 0;    ///< 2e: distinct arcs pinned at their integer
        std::size_t quality_gated_epochs = 0;   ///< epochs where the quality gates suppressed fixing
        std::size_t code_minus_carrier_jump_resets = 0;       ///< CMC screening: arc breaks forced
        std::size_t geometry_free_cycle_slip_resets = 0;      ///< confirmed geometry-free band resets
        std::size_t code_minus_carrier_level_exclusions = 0;  ///< CMC screening: (sat,signal) epochs excluded
        std::size_t cmc_ref_avoided_count = 0;  ///< cmc_aware_reference_selection: references changed away from a CMC-excluded candidate
        // --- CP-hold / sanity FSM diagnostics (use_cp_hold_recovery) ---
        std::size_t cp_hold_triggers = 0;         ///< times CP-hold was (re)engaged/extended
        std::size_t cp_hold_epochs_held = 0;      ///< cumulative epochs with carrier suppressed
        std::size_t cp_hold_anchor_releases = 0;
        std::size_t selective_cp_hold_downweighted_factors = 0;
        std::size_t sanity_mass_resets = 0;       ///< persist-path (3 consecutive bad) resets
        std::size_t sanity_fast_resets = 0;       ///< catastrophic fast-path resets
        std::size_t sanity_pose_replacements = 0; ///< epochs where the reported pose was IMU-predicted
        std::size_t sanity_multipath_skips = 0;   ///< bad epochs skipped as single-satellite multipath
        std::size_t sanity_gdop_skips = 0;        ///< persist-eligible resets skipped for weak geometry
        std::size_t ambiguity_generation_bumps = 0;  ///< total per-arc generation bumps (fresh symbols)
        std::size_t ambiguity_generation_bumps_hold = 0;
        std::size_t ambiguity_generation_bumps_fde = 0;
        std::size_t ambiguity_generation_bumps_reset = 0;
        std::size_t ambiguity_generation_bumps_warm_reset = 0;
        std::size_t ambiguity_continuous_unfix_resets = 0;
        std::size_t ambiguity_continuous_unfix_anchor_allows = 0;
        std::size_t ambiguity_continuous_unfix_anchor_skips = 0;
        std::size_t ambiguity_generation_bumps_stale_pin = 0;
        // --- Stale-pin invalidation diagnostics (use_stale_pin_invalidation) ---
        std::size_t stale_pin_invalidations = 0;  ///< pinned arcs released per-arc at a trigger epoch
        // --- Fix plausibility demotion diagnostics ---
        std::size_t fix_plausibility_demotions = 0;  ///< FIXED epochs demoted to FLOAT by general or intrinsic GF integrity guards
        std::size_t geometry_free_fix_guard_demotions = 0;  ///< low-redundancy GF-reset fixes rejected by gross SPP disagreement
        std::size_t fix_plausibility_anchor_demotions = 0;  ///< of which via the DDPR-LS anchor gap
        std::size_t fix_plausibility_anchor_gross_gated = 0;  ///< anchor-gap evaluations skipped by the gross-offender gate (fix_demote_anchor_gross)
        std::size_t fix_plausibility_hold_skips = 0;  ///< fix-and-hold pinnings skipped on implausible epochs
        std::size_t fix_plausibility_surplus_reprieves = 0;  ///< demotions skipped because fix_demote_surplus_crosscheck's verdict passed
        std::size_t fix_plausibility_spp_model_reprieves = 0;  ///< residual-only demotions skipped by fresh-SPP/model agreement
        // --- Exception recovery diagnostics (use_solve_exception_recovery) ---
        std::size_t solve_exception_recoveries = 0;   ///< loose-prior retries that succeeded
        std::size_t solve_exception_warm_resets = 0;  ///< full smoother re-creations
        // --- DDPR-LS anchor diagnostics (use_ddpr_anchor) ---
        std::size_t ddpr_anchor_solves = 0;         ///< mini DDPR-LS solve attempts (any of the 3 call sites)
        std::size_t ddpr_anchor_successes = 0;      ///< of which trusted (n>=min_factors, res_rms<=max)
        std::size_t ddpr_anchor_gated_resets_skipped = 0;  ///< diagnostic-only: gate would have rejected the reset (persist path; the reset still fires -- see use_ddpr_anchor comment)
        std::size_t ddpr_anchor_gated_resets_allowed = 0;  ///< diagnostic-only: gate would have accepted the reset
        std::size_t ddpr_anchored_warm_resets = 0;  ///< exception recoveries that used the DDPR anchor (vs the IMU-seeded fallback)
        std::size_t ddpr_anchor_bootstrap_prior_epochs = 0;  ///< epochs an anchor bootstrap translation prior was actually added
        // --- FDE diagnostics (use_fde) ---
        std::size_t fde_pseudorange_rejections = 0;  ///< total DD PR factors removed
        std::size_t fde_carrier_rejections = 0;      ///< total DD CP factors removed
        std::size_t fde_carrier_quarantines = 0;     ///< gross DD CP ambiguities excluded from AR only
        std::size_t fde_safeguard_skips = 0;         ///< epochs where the reject-fraction safeguard aborted FDE
        std::size_t fde_epochs = 0;                  ///< epochs where >=1 factor was actually removed
        // --- Sat-badness EWMA down-weighting diagnostics (use_sat_badness_downweight) ---
        std::size_t sat_badness_downweighted_factors = 0;  ///< DD PR/CP factors whose sigma was inflated (bad_pair>0 and its scale>0)
        double sat_badness_max_score_seen = 0.0;            ///< max bad_pair score observed across the run
        std::size_t float_rejected_seed_position_divergence = 0;
        std::size_t float_rejected_position_jump = 0;
        bool fixed_solution = false;
        bool lambda_ambiguity_fix_solved = false;
        bool lambda_ambiguity_fix_used = false;
        bool partial_lambda_ambiguity_fix_used = false;
        double initial_cost = 0.0;
        double final_cost = 0.0;
        // Phase96 main-graph diagnostics are opt-in and remain empty for the
        // legacy/default path.  No solution or accuracy fields are included.
        FGOPhase96MainDiagnostics native_source_clock_c0d_phase96_main;
        // Phase97 singular-system diagnostics are independently opt-in.  The
        // sidecar contains graph structure and read-only linearization data,
        // never a solution or accuracy estimate.
        FGOPhase97MainDiagnostics
            native_source_clock_c0d_phase97_singular_system;
        // Phase98 solver-boundary metadata is independently opt-in.  The
        // default remains disabled and this sidecar never exposes a
        // solution or changes the solver path.
        FGOPhase98SolverDiagnostics
            native_source_clock_c0d_phase98_solver_rank;
        // Phase99 solver selection is an opt-in branch witness.  The strings
        // describe the exact solver selected for the GTSAM batch invocation;
        // they do not expose Values, solution rows, or accuracy.
        bool native_source_clock_c0d_phase99_main_multifrontal_qr_solver_requested =
            false;
        bool native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected =
            false;
        std::string selected_linear_solver_type = "not-applicable";
        std::string selected_solver_branch = "not-applicable";
        std::string selected_elimination_function = "not-applicable";
        double processing_time_ms = 0.0;
        double epoch_lambda_processing_time_ms = 0.0;
        double epoch_lambda_setup_time_ms = 0.0;
        double epoch_lambda_factorization_time_ms = 0.0;
        double epoch_lambda_covariance_solve_time_ms = 0.0;
        double epoch_lambda_search_time_ms = 0.0;
        double epoch_lambda_fixed_output_time_ms = 0.0;
        double epoch_lambda_debug_record_time_ms = 0.0;
        double postprocessing_time_ms = 0.0;
        double total_processing_time_ms = 0.0;
        double last_update_norm_m = 0.0;
        double residual_rms_m = 0.0;
        double tdcp_residual_rms_m = 0.0;
        std::size_t tdcp_frequency_residual_states = 0;
        std::size_t tdcp_frequency_residual_factors = 0;
        std::size_t tdcp_frequency_residual_priors = 0;
        std::size_t optimized_imu_bias_count = 0;
        std::size_t nominal_p_information_epochs = 0;
        std::size_t nominal_p_information_rank_deficient_epochs = 0;
        double nominal_p_information_min_eigenvalue_per_m2 = 0.0;
        double optimized_accel_bias_max_norm_mps2 = 0.0;
        double optimized_gyro_bias_max_norm_radps = 0.0;
        double undifferenced_doppler_residual_rms_mps = 0.0;
        double single_difference_doppler_residual_rms_mps = 0.0;
        double single_difference_tdcp_residual_rms_m = 0.0;
        double carrier_phase_residual_rms_m = 0.0;
        double double_difference_pseudorange_residual_rms_m = 0.0;
        double double_difference_carrier_residual_rms_m = 0.0;
        double fixed_ambiguity_residual_rms_cycles = 0.0;
        double lambda_ambiguity_ratio = 0.0;
    };

    struct AmbiguityEstimate {
        SatelliteId satellite;
        SignalType signal = SignalType::GPS_L1CA;
        std::size_t segment_index = 0;
        double wavelength_m = 0.0;
        double ambiguity_m = 0.0;
        double ambiguity_cycles = 0.0;
        int fixed_cycles = 0;
        double fixed_ambiguity_m = 0.0;
        double fix_residual_cycles = 0.0;
        bool is_fixed = false;
        bool fixed_by_lambda = false;
    };

    struct LambdaDebugEntry {
        std::size_t epoch_index = 0;
        GNSSTime time;
        bool solved = false;
        bool fixed_epoch = false;
        double ratio = 0.0;
        int candidate_count = 0;
        int row = 0;
        int col = 0;
        int local_index = 0;
        int other_local_index = 0;
        SatelliteId satellite;
        SatelliteId other_satellite;
        double ambiguity_float = 0.0;
        double fixed_ambiguity = 0.0;
        double covariance = 0.0;
        double position_covariance_x = 0.0;
        double position_covariance_y = 0.0;
        double position_covariance_z = 0.0;
    };

    struct CostTraceEntry {
        std::string phase;
        int local_iteration = 0;
        int global_iteration = 0;
        double cost = 0.0;
        double absolute_decrease = 0.0;
        double relative_decrease = 0.0;
        double update_norm = 0.0;
        bool converged = false;
    };

    /// Terminal ambiguity-resolution state for one epoch.  This is kept
    /// separate from the reported FIX/FLOAT status so a FLOAT epoch says
    /// exactly which integrity gate stopped it.
    enum class AmbiguityResolutionOutcome : int {
        NotAttempted = 0,
        Disabled = 1,
        SmootherFailure = 2,
        QualityGateRejected = 3,
        NoCandidates = 4,
        InsufficientCandidates = 5,
        MarginalFailure = 6,
        LambdaSearchFailed = 7,
        RatioRejected = 8,
        ImuApertureRejected = 9,
        FixedHistoryRejected = 10,
        PostfitRejected = 11,
        Fixed = 12,
        SurplusValidationRejected = 13,
        LowCountRejected = 14,  ///< below-floor attempt (use_low_count_ambiguity_resolution) failed its surplus/ratio gate
        IntegerConstrainedReoptimizationRejected = 15,
    };

    /// Terminal reason why one satellite/signal DD carrier row did or did not
    /// reach the per-epoch LAMBDA candidate set.  This is diagnostic-only:
    /// the backend records decisions already made by the existing pipeline.
    enum class AmbiguityCandidateDisposition : int {
        LambdaEligible = 0,
        BuildTimeExcluded = 1,
        CarrierHoldSuppressed = 2,
        CarrierHoldQuarantined = 3,
        OneBandPerSatelliteExcluded = 4,
        ConstellationExcluded = 5,
        PreviousResidualGateExcluded = 6,
        FdeExcluded = 7,
        StaleSmootherKeyExcluded = 8,
        EpochQualityGateExcluded = 9,
        AmbiguityResolutionDisabled = 10,
    };

    /// Satellite/signal-level trace for one DD carrier row in one epoch.
    struct AmbiguityCandidateTrace {
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::SIGNAL_TYPE_COUNT;
        std::size_t ambiguity_index = std::numeric_limits<std::size_t>::max();
        AmbiguityCandidateDisposition disposition =
            AmbiguityCandidateDisposition::LambdaEligible;
    };

    /// One counterfactual leave-one-target-satellite-out LAMBDA trial.
    /// Populated only by monitor_ratio_impact_partial_ar and never consumed
    /// by the estimator.
    struct RatioImpactTrialTrace {
        SatelliteId excluded_satellite;
        int excluded_ambiguities = 0;
        double excluded_max_variance_cycles2 = 0.0;
        double excluded_max_fractional_cycles = 0.0;
        double excluded_ddpr_residual_m = 0.0;
        bool candidate_available = false;
        double ratio = 0.0;
        int fixed_ambiguities = 0;
        Vector3d candidate_position_ecef = Vector3d::Zero();
        double float_separation_m = 0.0;
        double imu_separation_m = 0.0;
    };

    /// Two constellation-disjoint reduced LAMBDA candidates. Populated only
    /// by monitor_disjoint_constellation_ar and never consumed by the
    /// estimator.
    struct DisjointConstellationArShadow {
        bool evaluated = false;
        std::uint64_t partition_a_system_mask = 0;
        std::uint64_t partition_b_system_mask = 0;
        int partition_a_ambiguities = 0;
        int partition_b_ambiguities = 0;
        double partition_a_ratio = 0.0;
        double partition_b_ratio = 0.0;
        double partition_a_bootstrapped_success_rate = 0.0;
        double partition_b_bootstrapped_success_rate = 0.0;
        bool partition_a_ratio_passed = false;
        bool partition_b_ratio_passed = false;
        bool partition_a_candidate_available = false;
        bool partition_b_candidate_available = false;
        Vector3d partition_a_position_ecef = Vector3d::Zero();
        Vector3d partition_b_position_ecef = Vector3d::Zero();
        double partition_separation_m = 0.0;
        double partition_a_primary_separation_m = 0.0;
        double partition_b_primary_separation_m = 0.0;
    };

    /// Counterfactual two-stage multi-frequency AR result. Populated only by
    /// monitor_conditional_multiband_ar and never consumed by the estimator.
    struct ConditionalMultibandArShadow {
        bool evaluated = false;
        int primary_ambiguities = 0;
        int secondary_ambiguities = 0;
        double primary_ratio = 0.0;
        double secondary_ratio = 0.0;
        double primary_bootstrapped_success_rate = 0.0;
        double secondary_bootstrapped_success_rate = 0.0;
        bool primary_ratio_passed = false;
        bool secondary_ratio_passed = false;
        bool candidate_available = false;
        Vector3d candidate_position_ecef = Vector3d::Zero();
        double float_separation_m = 0.0;
        double imu_separation_m = 0.0;
    };

    /// Counterfactual LAMBDA result using only ambiguity arcs whose integer
    /// candidate has persisted across multiple consecutive epochs.
    struct MultiEpochArShadow {
        bool evaluated = false;
        int persistent_ambiguities = 0;
        int minimum_support_epochs = 0;
        double ratio = 0.0;
        double bootstrapped_success_rate = 0.0;
        bool history_integers_agree = false;
        bool ratio_passed = false;
        bool candidate_available = false;
        Vector3d candidate_position_ecef = Vector3d::Zero();
        double float_separation_m = 0.0;
        double imu_separation_m = 0.0;
        // Held-out carrier witness: rows whose ambiguity is not part of the
        // persistent LAMBDA subset are checked at the candidate position.
        // A missing independent pool leaves evaluated=false (fail closed).
        bool surplus_validation_evaluated = false;
        bool surplus_validation_pass = false;
        int surplus_validation_fallback_level = -1;
        int surplus_validation_surplus_used = 0;
        // GICI-style constrained graph-cost witness. The active fixed-lag
        // graph is independently batch-refined with this candidate imposed
        // as tight ambiguity priors, then scored using the original graph
        // without those priors. Diagnostic-only; never updates the smoother.
        bool graph_cost_evaluated = false;
        bool graph_cost_pass = false;
        int graph_cost_factor_count = 0;
        double graph_cost_before = 0.0;
        double graph_cost_after = 0.0;
    };

    /// One diagnostic-only DDPR GNC weight at the current epoch's optimized
    /// pose. These rows are never consumed by the estimator.
    struct DdprGncFactorTrace {
        SatelliteId satellite;
        SatelliteId reference_satellite;
        SignalType signal = SignalType::GPS_L1CA;
        double residual_m = 0.0;
        double sigma_m = 0.0;
        double normalized_residual = 0.0;
        double weight = 1.0;
    };

    /// One selective arc-restart candidate/action row. Populated only when
    /// use_selective_arc_restart (active) or
    /// monitor_selective_arc_restart_candidates (monitor-only) is enabled.
    struct SelectiveArcRestartTrace {
        std::size_t detection_epoch = 0;
        SatelliteId satellite;
        bool is_reference = false;
        std::size_t ambiguity_index = 0;
        bool detected = false;
        bool applied = false;
        double postfit_residual_m = 0.0;
        double epoch_median_postfit_residual_m = 0.0;
        double normalized_doppler_innovation = 0.0;
        double normalized_imu_innovation = 0.0;
    };

    /// Per-epoch fixed-lag integrity state.  These values expose why an epoch
    /// did or did not fix, rather than only reporting the final FIX/FLOAT label.
    struct FGOEpochDiagnostics {
        GNSSTime time;
        /// Latest observation time used for the reported position minus its
        /// epoch time. Smoothed positions must not masquerade as causal input.
        double solution_latency_s = std::numeric_limits<double>::quiet_NaN();
        std::uint64_t solution_reset_generation = 0;
        AmbiguityResolutionOutcome ar_outcome =
            AmbiguityResolutionOutcome::NotAttempted;
        double ddpr_rms_m = 0.0;
        bool ddpr_gnc_evaluated = false;
        int ddpr_gnc_factor_count = 0;
        int ddpr_gnc_stages = 0;
        double ddpr_gnc_initial_mu = 0.0;
        double ddpr_gnc_final_mu = 0.0;
        double ddpr_gnc_min_weight = 1.0;
        double ddpr_gnc_mean_weight = 1.0;
        double ddpr_gnc_effective_factor_count = 0.0;
        int ddpr_gnc_downweighted_factors = 0;
        double ddpr_gnc_weighted_rms_m = 0.0;
        std::vector<DdprGncFactorTrace> ddpr_gnc_factor_trace;
        bool ddpr_gnc_counterfactual_evaluated = false;
        bool ddpr_gnc_counterfactual_succeeded = false;
        int ddpr_gnc_counterfactual_factor_count = 0;
        int ddpr_gnc_counterfactual_stages = 0;
        double ddpr_gnc_counterfactual_cost_before = 0.0;
        double ddpr_gnc_counterfactual_cost_after = 0.0;
        double ddpr_gnc_counterfactual_ddpr_rms_before_m = 0.0;
        double ddpr_gnc_counterfactual_ddpr_rms_after_m = 0.0;
        Vector3d ddpr_gnc_counterfactual_position_ecef = Vector3d::Zero();
        double ddpr_gnc_counterfactual_float_separation_m = 0.0;
        bool ddpr_gnc_counterfactual_lambda_evaluated = false;
        int ddpr_gnc_counterfactual_lambda_ambiguities = 0;
        double ddpr_gnc_counterfactual_lambda_ratio = 0.0;
        bool ddpr_gnc_counterfactual_lambda_ratio_pass = false;
        bool candidate_integrity_witness_evaluated = false;
        bool candidate_integrity_anchor_available = false;
        int candidate_integrity_anchor_factors = 0;
        double candidate_integrity_anchor_rms_m = 0.0;
        double candidate_integrity_anchor_separation_m = 0.0;
        bool candidate_integrity_anchor_pass = false;
        bool candidate_integrity_imu_pass = false;
        bool candidate_integrity_doppler_available = false;
        bool candidate_integrity_doppler_pass = false;
        bool candidate_integrity_carrier_pass = false;
        bool candidate_integrity_composite_pass = false;
        double sd_doppler_rms_mps = 0.0;
        int clock_resilient_tdcp_factors = 0;
        double clock_resilient_tdcp_rms_m = 0.0;
        double clock_resilient_tdcp_max_abs_m = 0.0;
        int clock_resilient_tdcp_clean = 0;
        int clock_resilient_tdcp_witnessed_outliers = 0;
        int clock_resilient_tdcp_unexplained_outliers = 0;
        double gdop = 0.0;
        int num_satellites = 0;
        int sd_doppler_factors = 0;
        int ambiguity_candidates = 0;
        int lambda_attempts = 0;
        int lambda_selected_stage = -1;  ///< PPC cascade: 0=GQEBR ... 5=GQ
        double ambiguity_variance_median_cycles2 = 0.0;
        double ambiguity_variance_max_cycles2 = 0.0;
        /// Median |float ambiguity - nearest integer| over the epoch's LAMBDA
        /// candidates (cycles). A small variance with a large fractional part
        /// indicates an integer bias rather than a weak estimate.
        double ambiguity_fractional_median_cycles = 0.0;
        double imu_pose_correction_m = 0.0;
        // Read-only copy of the builder's independent current-epoch SPP
        // seed. This exposes an absolute-code witness for AR analysis while
        // keeping all estimator and FIX/FLOAT decisions unchanged.
        bool fresh_spp_solution = false;
        Vector3d spp_seed_position_ecef = Vector3d::Zero();
        // Last position candidate produced by a successful LAMBDA search in
        // this epoch, even when a later integrity/ratio decision leaves the
        // epoch FLOAT. Diagnostic-only; never feeds the estimator.
        bool lambda_candidate_available = false;
        Vector3d lambda_candidate_position_ecef = Vector3d::Zero();
        int lambda_candidate_fixed_ambiguities = 0;
        double lambda_candidate_ratio = 0.0;
        // Covariance-only quality diagnostics from the same Top-K LAMBDA
        // search that produced lambda_candidate_position_ecef. These fields
        // are monitor-only and never participate in FIX/FLOAT decisions.
        double lambda_candidate_bsr = 0.0;
        double lambda_candidate_bsr_qscale2 = 0.0;
        double lambda_candidate_bsr_qscale4 = 0.0;
        double lambda_candidate_bsr_qscale8 = 0.0;
        double lambda_candidate_bsr_qscale16 = 0.0;
        bool lambda_candidate_ffrt_table_supported = false;
        bool lambda_candidate_ffrt_accepts_any = false;
        double lambda_candidate_ffrt_min_ratio = 0.0;
        bool lambda_candidate_ffrt_pass = false;
        // Temporal-consensus shadow for the last LAMBDA candidate in this
        // epoch. Ambiguity indices are stable only within an unchanged arc,
        // so overlap automatically excludes restarted/referenced arcs. The
        // shadow never changes acceptance, hold, or graph state.
        int lambda_candidate_integer_overlap = 0;
        int lambda_candidate_integer_agreements = 0;
        double lambda_candidate_integer_agreement_fraction = 0.0;
        int lambda_candidate_integer_consensus_streak = 0;
        // Diagnostic-only geometry-free cycle-slip shadow. It analyzes the
        // rover/base single-difference carrier phases already present in the
        // DD factors, but never changes ambiguity arcs or graph factors.
        int gf_slip_shadow_event_pairs = 0;
        double gf_slip_shadow_max_jump_m = 0.0;
        int gf_slip_shadow_tainted_ambiguities = 0;
        int doppler_slip_shadow_event_signals = 0;
        double doppler_slip_shadow_max_innovation_m = 0.0;
        int gf_doppler_shadow_isolated_pairs = 0;
        ConditionalMultibandArShadow conditional_multiband_ar_shadow;
        MultiEpochArShadow multiepoch_ar_shadow;
        bool ratio_impact_evaluated = false;
        int ratio_impact_trials = 0;
        double ratio_impact_best_ratio = 0.0;
        int ratio_impact_best_fixed_ambiguities = 0;
        Vector3d ratio_impact_best_position_ecef = Vector3d::Zero();
        double ratio_impact_best_float_separation_m = 0.0;
        double ratio_impact_best_imu_separation_m = 0.0;
        std::vector<RatioImpactTrialTrace> ratio_impact_trial_trace;
        DisjointConstellationArShadow disjoint_constellation_ar_shadow;
        bool ddpr_anchor_evaluated = false;
        bool ddpr_anchor_bootstrap_prior_applied = false;
        int ddpr_anchor_active_factors = 0;
        double ddpr_anchor_residual_rms_m = 0.0;
        Vector3d ddpr_anchor_position_ecef = Vector3d::Zero();
        double fixed_float_separation_m = 0.0;
        double fixed_imu_prediction_separation_m = 0.0;
        // --- "c2" DR-gate bypass counterfactual (see FGOConfig::
        // surplus_validation_overrides_history_dr). Monitor fields below are
        // populated whenever a surplus-passed candidate's RAW fixed-history-
        // DR verdict (before any reprieve/override) would reject it -- cheap
        // and unconditional, independent of whether the override knob is
        // enabled -- so callers can score the would-be fix against
        // reference.csv even when the epoch is left FLOAT. ---
        bool dr_bypass_candidate_evaluated = false;  ///< true = this epoch had a surplus-passed candidate blocked by the raw DR verdict
        Vector3d dr_bypass_candidate_position_ecef = Vector3d::Zero();  ///< that candidate's fixed antenna ECEF position
        bool dr_bypass_applied = false;  ///< surplus_validation_overrides_history_dr actually accepted this candidate
        double fixed_postfit_ddcp_rms_m = 0.0;
        double fixed_postfit_ddcp_max_normalized = 0.0;
        double fixed_postfit_ddcp_chi2_per_dof = 0.0;
        int fixed_postfit_ddcp_factors = 0;
        double effective_ratio_threshold = 0.0;
        bool integer_constrained_reoptimization_evaluated = false;
        bool integer_constrained_reoptimization_pass = false;
        double integer_constrained_base_cost_before = 0.0;
        double integer_constrained_base_cost_after = 0.0;
        double external_dr_separation_m = 0.0;
        double external_dr_mahalanobis2 = 0.0;
        int external_dr_age_epochs = -1;
        bool external_doppler_velocity_valid = false;
        Vector3d external_doppler_velocity_ecef_mps = Vector3d::Zero();
        bool external_dr_evaluated = false;
        bool external_dr_accepted = false;
        bool external_dr_rejected = false;
        // Monitor-only NHC/ZUPT gate inputs and decisions. These values are
        // computed before the current epoch is optimized and never feed AR.
        int motion_constraint_imu_samples = 0;
        double motion_constraint_accel_std_mps2 = 0.0;
        double motion_constraint_gyro_std_radps = 0.0;
        double motion_constraint_gyro_median_radps = 0.0;
        double motion_constraint_yaw_rate_radps = 0.0;
        double motion_constraint_seed_speed_mps = 0.0;
        bool zupt_candidate = false;
        bool zupt_applied = false;
        bool nhc_candidate = false;
        bool nhc_applied = false;
        bool carrier_hold_active = false;
        bool imu_aperture_accepted = false;
        bool imu_aperture_rejected = false;
        // --- Surplus-satellite independent integrity validation ---
        bool surplus_validation_evaluated = false;  ///< false = insufficient surplus sats at every fallback level
        bool surplus_validation_pass = false;
        bool surplus_validation_used_for_rescue = false;  ///< this test flipped a relaxed-ratio candidate to FIXED
        bool surplus_validation_used_for_veto = false;    ///< this test demoted an established-ratio fix
        int surplus_validation_fallback_level = -1;  ///< 0=GQEBR .. 5=GQ, -1 = not evaluated
        int surplus_validation_surplus_used = 0;     ///< surplus satellites in the deciding pool
        // --- Below-floor low-count AR rescue (use_low_count_ambiguity_resolution) ---
        bool low_count_ar_attempted = false;  ///< this epoch's LAMBDA attempt only happened because the floor was lowered
        bool low_count_ar_used = false;       ///< this epoch's FIXED label came from the low-count rescue path
        int carrier_factors_available = 0;
        int carrier_factors_added = 0;
        int carrier_factors_suppressed_hold = 0;
        // Candidate attrition funnel.  ambiguity_candidates retains its
        // historical meaning (after one-band/constellation filtering, before
        // runtime residual/FDE/stale-key filtering).
        int ambiguity_candidates_after_hold = 0;
        int ambiguity_candidates_final = 0;
        int ambiguity_candidates_excluded_build_time = 0;
        int ambiguity_candidates_excluded_hold = 0;
        int ambiguity_candidates_excluded_one_band = 0;
        int ambiguity_candidates_excluded_constellation = 0;
        int ambiguity_candidates_excluded_previous_residual = 0;
        int ambiguity_candidates_excluded_fde = 0;
        int ambiguity_candidates_excluded_stale = 0;
        std::vector<AmbiguityCandidateTrace> ambiguity_candidate_trace;
        int ambiguity_generation_bumps_hold = 0;
        int ambiguity_generation_bumps_fde = 0;
        int ambiguity_generation_bumps_reset = 0;
        int ambiguity_generation_bumps_warm_reset = 0;
        int ambiguity_generation_bumps_stale_pin = 0;
        // --- Active selective arc restart (use_selective_arc_restart) ---
        bool selective_arc_restart_armed = false;
        int selective_arc_restart_candidate_satellites = 0;
        int selective_arc_restart_candidate_pairs = 0;
        int selective_arc_restart_applied_arcs = 0;
        int selective_arc_restart_skipped_fixed = 0;
        int selective_arc_restart_skipped_thrash = 0;
        int selective_arc_restart_skipped_cap = 0;
        bool selective_arc_restart_monitor_only = false;
        std::vector<SelectiveArcRestartTrace> selective_arc_restart_trace;
    };

    struct FGOResult {
        struct TdcpFrequencyCorrection {
            std::size_t previous_epoch_index = 0;
            std::size_t current_epoch_index = 0;
            SatelliteId satellite;
            SignalType signal = SignalType::GPS_L1CA;
            // Subtract from prediction-minus-measurement residual [m].
            double alpha_slant_change_m = 0.0;
        };
        // Same-run diagnostic handoff only; empty when disabled. Never an
        // input to a later solver invocation or serialized seed trajectory.
        std::vector<TdcpFrequencyCorrection> tdcp_frequency_corrections;
        Solution solution;
        FGODiagnostics diagnostics;
        std::vector<AmbiguityEstimate> ambiguity_estimates;
        std::vector<std::set<SatelliteId>> ambiguity_candidate_satellites_by_epoch;
        std::vector<std::set<SatelliteId>> ambiguity_reference_satellites_by_epoch;
        std::vector<std::map<SatelliteId, double>> ambiguity_estimate_cycles_by_epoch;
        std::vector<Vector3d> epoch_velocities_ecef_mps;
        // Phase93 optimized receiver clock-drift states [m/s], exported in
        // retained GNSS-first/source order when the source-meter C0D graph is
        // active.  Missing or nonfinite state export is a fail-closed result;
        // no raw/zero/WLS/interpolated fallback is permitted by that selector.
        std::vector<double> epoch_clock_drift_mps;
        // Phase101 optimized official seven-component receiver C_i vectors
        // [m], exported in exact retained epoch order.  Empty on legacy and
        // scalar-C paths; no CSV/accuracy output is implied by this field.
        std::vector<EpochClockBiasComponentsM>
            epoch_clock_bias_components_m;
        // Static receiver secondary-signal code-bias estimates [m], populated
        // only by the opt-in signal-bias backend path.
        std::map<std::pair<GNSSSystem, SignalType>, double>
            receiver_signal_bias_estimates_m;
        // Per-epoch vertical L1 residual-ionosphere estimates [m], populated
        // only by the opt-in raw candidate.
        std::vector<double> residual_ionosphere_estimates_m;
        std::vector<LambdaDebugEntry> lambda_debug_entries;
        std::vector<CostTraceEntry> cost_trace_entries;
        std::vector<FGOEpochDiagnostics> epoch_diagnostics;
        std::vector<TemporalCarrierShadowFactorDiagnostics>
            temporal_carrier_shadow_factors;
        std::vector<PredictedDdprQualityFactorDiagnostics>
            predicted_ddpr_quality_factors;
        std::vector<PredictedDdprBiasStateDiagnostics>
            predicted_ddpr_bias_state_factors;
        std::vector<SatelliteQuarantineWitnessDiagnostics>
            satellite_quarantine_witnesses;
        std::vector<SelectiveArcRestartTrace> selective_arc_restart_trace;
        // Milestone 2b (populated only by the GTSAM IMU-coupled path):
        // per-epoch estimated attitude as [roll, pitch, heading] in degrees
        // (body FLU -> nav ENU; heading is clockwise from North) and estimated
        // velocity in the ENU nav frame [m/s].
        std::vector<Vector3d> epoch_attitude_rpy_deg;
        // Exact GTSAM Rot3::rpy() values [roll, pitch, yaw] in radians.  This
        // is kept separate from the display/course convention above so raw
        // post-processing ports cannot accidentally use a heading remap.
        std::vector<Vector3d> epoch_attitude_rpy_rad;
        std::vector<Vector3d> epoch_velocity_nav_mps;
    };

    FGOProcessor() = default;
    explicit FGOProcessor(const FGOConfig& config) : config_(config) {}

    const FGOConfig& getConfig() const { return config_; }
    void setConfig(const FGOConfig& config) { config_ = config; }

    static std::vector<GeometryFreeSlipShadowEpoch>
    analyzeGeometryFreeSlipShadow(const FGOProblem& problem,
                                  double threshold_m = 0.05,
                                  double max_gap_s = 1.5);

    static std::vector<SingleDifferenceTdcpFactor>
    buildClockResilientTemporalCarrierShadow(const FGOProblem& problem,
                                             double sigma_m = 0.003,
                                             double max_gap_s = 1.5);

    /// Evaluate and classify a pre-built temporal-carrier shadow against a
    /// supplied per-epoch ECEF trajectory. This routine is solver-independent
    /// so a frozen solution CSV can replay diagnostics without rerunning GTSAM.
    static std::vector<TemporalCarrierShadowFactorDiagnostics>
    classifyClockResilientTemporalCarrierShadow(
        const FGOProblem& problem,
        const std::vector<SingleDifferenceTdcpFactor>& factors,
        const std::vector<Vector3d>& epoch_positions_ecef,
        std::vector<FGOEpochDiagnostics>& epoch_diagnostics,
        const std::vector<GeometryFreeSlipShadowEpoch>* geometry_free_shadow =
            nullptr,
        const std::vector<std::set<std::size_t>>*
            fde_rejected_ambiguities_by_epoch = nullptr);

    /// Compare temporal DD pseudorange changes with receiver-clock-free DD
    /// Doppler and a one-step predicted antenna trajectory. `previous_*`
    /// supplies the causal solved pose at k-1; `predicted_*` supplies the
    /// pre-solve pose prediction at k. The result is monitor-only.
    static std::vector<PredictedDdprQualityFactorDiagnostics>
    analyzePredictedDdprQualityShadow(
        const FGOProblem& problem,
        const std::vector<Vector3d>& previous_solution_positions_ecef,
        const std::vector<Vector3d>& predicted_positions_ecef,
        double doppler_sigma_mps = 0.2,
        double normalized_outlier_threshold = 5.0,
        double max_gap_s = 1.5);

    /// Predict persistent pair-specific DD pseudorange bias from prior causal
    /// predicted-DDPR residual rows. The current row never predicts itself.
    static std::vector<PredictedDdprBiasStateDiagnostics>
    analyzePredictedDdprBiasStateShadow(
        const std::vector<PredictedDdprQualityFactorDiagnostics>& quality_rows,
        double process_noise_m_sqrt_s = 0.25,
        double initial_sigma_m = 5.0,
        double min_measurement_sigma_m = 0.5,
        double robust_update_sigma = 3.0,
        int min_prior_updates = 2);

    FGOProblem buildPseudorangeProblem(const std::vector<ObservationData>& epochs,
                                       const NavigationData& nav) const;

    FGOProblem buildDoubleDifferenceProblem(
        const std::vector<ObservationData>& rover_epochs,
        const std::vector<ObservationData>& base_epochs,
        const NavigationData& nav,
        const Vector3d& base_position_ecef) const;

    FGOResult optimize(const std::vector<ObservationData>& epochs,
                       const NavigationData& nav) const;

    FGOResult optimize(const std::vector<ObservationData>& rover_epochs,
                       const std::vector<ObservationData>& base_epochs,
                       const NavigationData& nav,
                       const Vector3d& base_position_ecef) const;

    FGOResult optimizeProblem(const FGOProblem& problem) const;

private:
    FGOConfig config_;
};

}  // namespace libgnss
