#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <libgnss++/algorithms/integrity_consensus.hpp>
#include <libgnss++/algorithms/nlos_weights.hpp>
#include <libgnss++/algorithms/float_trust_policy.hpp>
#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/rtk_validation.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/gnss.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/solution_writer.hpp>
#include <libgnss++/models/troposphere.hpp>

#include "cli_toml_config.hpp"
#include "rtk_base_epoch_align.hpp"

namespace {

using libgnss_apps::kExactTimeToleranceSeconds;
constexpr double kDefaultFloatVsSppGuardMeters = 30.0;
constexpr double kDefaultNonFixedJumpGuardMeters = 8.0;
constexpr double kDefaultVerticalStepGuardMeters = 1.0;
constexpr double kDefaultNonFixDriftGuardMaxAnchorGapSeconds = 120.0;
constexpr double kDefaultNonFixDriftGuardMaxAnchorSpeedMps = 1.0;
constexpr double kDefaultNonFixDriftGuardMaxResidualMeters = 30.0;
constexpr double kDefaultNonFixDriftGuardMinHorizontalResidualMeters = 0.0;
constexpr int kDefaultNonFixDriftGuardMinSegmentEpochs = 20;
constexpr int kDefaultNonFixDriftGuardMaxSegmentEpochs = 0;
constexpr double kDefaultSppHeightStepGuardMinMeters = 30.0;
constexpr double kDefaultSppHeightStepGuardMaxRateMps = 4.0;
constexpr double kDefaultFloatBridgeTailGuardMaxAnchorGapSeconds = 120.0;
constexpr double kDefaultFloatBridgeTailGuardMinAnchorSpeedMps = 0.4;
constexpr double kDefaultFloatBridgeTailGuardMaxAnchorSpeedMps = 1.0;
constexpr double kDefaultFloatBridgeTailGuardMaxResidualMeters = 12.0;
constexpr int kDefaultFloatBridgeTailGuardMinSegmentEpochs = 20;
constexpr double kDefaultFixedBridgeBurstGuardMaxAnchorGapSeconds = 30.0;
constexpr double kDefaultFixedBridgeBurstGuardMinBoundaryGapSeconds = 1.0;
constexpr double kDefaultFixedBridgeBurstGuardMaxResidualMeters = 20.0;
constexpr int kDefaultFixedBridgeBurstGuardMaxSegmentEpochs = 12;

enum class ModeChoice {
    AUTO,
    KINEMATIC,
    STATIC,
    MOVING_BASE
};

enum class IonoChoice {
    AUTO,
    OFF,
    IFLC,
    EST
};

enum class GlonassARChoice {
    OFF,
    ON,
    AUTOCAL
};

enum class RTKTuningPreset {
    NONE,
    SURVEY,
    LOW_COST,
    MOVING_BASE,
    ODAIBA
};

struct SolveConfig {
    std::string data_dir;
    std::string rover_obs_path;
    std::string base_obs_path;
    std::string nav_path;
    std::string output_pos_path = "output/rtk_solution.pos";
    std::string output_kml_path = "output/rtk_solution.kml";
    bool write_kml = true;
    bool enable_base_interpolation = true;
    bool verbose = false;
    bool base_position_override = false;
    Eigen::Vector3d base_position_ecef = Eigen::Vector3d::Zero();
    ModeChoice mode = ModeChoice::AUTO;
    IonoChoice iono = IonoChoice::AUTO;
    libgnss::io::SolutionWriter::Format output_format = libgnss::io::SolutionWriter::Format::POS;
    double max_baseline_length_m = 20000.0;
    bool prefer_trusted_seed = false;
    bool use_doppler_float_seed = false;
    double doppler_float_seed_max_age_s = 6.0;
    std::string rover_seed_pos_path;
    std::string diagnostics_csv_path;
    // GNSS-only causal MultiSD FGO shadow. The rolling window ends at the
    // current RTK epoch and never changes RTK output/filter state.
    std::string multisd_fgo_shadow_csv_path;
    int multisd_fgo_shadow_window_epochs = 25;
    int multisd_fgo_shadow_min_epochs = 10;
    int multisd_fgo_shadow_holdout_offset = 2;
    int multisd_fgo_shadow_top_k = 4;
    double multisd_fgo_shadow_max_seed_separation_m = 0.5;
    int multisd_fgo_shadow_validation_history_epochs = 3;
    double multisd_fgo_shadow_min_carrier_fraction = 0.75;
    int multisd_fgo_shadow_min_fixed_ambiguities = 6;
    int multisd_fgo_shadow_holdout_satellites = 4;
    double rtk_update_outlier_threshold = 0.0;
    bool student_t_rtk_front_end = false;
    double ratio_threshold = 3.0;
    bool enable_satellite_count_ratio_threshold = false;
    bool enable_ar_filter = false;
    bool has_ar_filter_override = false;
    double ar_filter_margin = 0.25;
    int min_satellites_for_ar = 5;
    int min_subset_pairs_for_ar = 4;
    int max_subset_drop_steps_for_ar = 6;
    int min_subset_sats_for_ar = 0;
    int min_subset_systems_for_ar = 0;
    int min_subset_frequencies_for_ar = 0;
    int min_subset_dual_frequency_sats_for_ar = 0;
    double min_full_ratio_for_subset_ar = 0.0;
    double low_ratio_guard_threshold = 0.0;
    int low_ratio_min_fixed_ambiguities = 0;
    double low_count_rescue_ratio_threshold = 0.0;
    int low_count_rescue_min_fixed_ambiguities = 4;
    double low_count_rescue_max_history_speed_mps = 0.0;
    int min_hold_count = 5;
    double hold_ratio_threshold = 2.0;
    double elevation_mask_deg = 15.0;
    double process_noise_position = 1e-4;
    double process_noise_ambiguity = 1e-8;
    double process_noise_iono = 1e-4;
    double carrier_phase_sigma = 0.002;
    bool enable_snr_weighting = false;
    double snr_reference_dbhz = 45.0;
    double snr_max_variance_scale = 25.0;
    double snr_min_baseline_m = 0.0;
    double cycle_slip_threshold = 0.05;
    double doppler_slip_threshold = 0.20;
    double code_slip_threshold = 5.0;
    bool use_dynamic_slip_threshold_floor = true;
    bool enable_adaptive_dynamic_slip_thresholds = false;
    int adaptive_dynamic_slip_nonfix_count = 3;
    int adaptive_dynamic_slip_hold_epochs = 10;
    bool process_noise_position_set = false;
    bool process_noise_ambiguity_set = false;
    bool process_noise_iono_set = false;
    bool carrier_phase_sigma_set = false;
    bool enable_glonass = true;
    bool enable_beidou = true;
    bool enable_l5 = false;  // Phase 18 Step 3: opt-in L5 measurement collection
    GlonassARChoice glonass_ar = GlonassARChoice::OFF;
    double glonass_icb_l1_m_per_mhz = 0.0;
    double glonass_icb_l2_m_per_mhz = 0.0;
    int skip_epochs = 0;
    int max_epochs = -1;
    std::string debug_epoch_log_path;
    bool enable_kinematic_post_filter = true;
    bool enable_nonfix_drift_guard = true;
    double nonfix_drift_guard_max_anchor_gap_s = kDefaultNonFixDriftGuardMaxAnchorGapSeconds;
    double nonfix_drift_guard_max_anchor_speed_mps = kDefaultNonFixDriftGuardMaxAnchorSpeedMps;
    double nonfix_drift_guard_max_residual_m = kDefaultNonFixDriftGuardMaxResidualMeters;
    double nonfix_drift_guard_min_horizontal_residual_m =
        kDefaultNonFixDriftGuardMinHorizontalResidualMeters;
    int nonfix_drift_guard_min_segment_epochs = kDefaultNonFixDriftGuardMinSegmentEpochs;
    int nonfix_drift_guard_max_segment_epochs = kDefaultNonFixDriftGuardMaxSegmentEpochs;
    bool enable_spp_height_step_guard = true;
    double spp_height_step_guard_min_m = kDefaultSppHeightStepGuardMinMeters;
    double spp_height_step_guard_max_rate_mps = kDefaultSppHeightStepGuardMaxRateMps;
    bool enable_float_bridge_tail_guard = true;
    double float_bridge_tail_guard_max_anchor_gap_s = kDefaultFloatBridgeTailGuardMaxAnchorGapSeconds;
    double float_bridge_tail_guard_min_anchor_speed_mps = kDefaultFloatBridgeTailGuardMinAnchorSpeedMps;
    double float_bridge_tail_guard_max_anchor_speed_mps = kDefaultFloatBridgeTailGuardMaxAnchorSpeedMps;
    double float_bridge_tail_guard_max_residual_m = kDefaultFloatBridgeTailGuardMaxResidualMeters;
    int float_bridge_tail_guard_min_segment_epochs = kDefaultFloatBridgeTailGuardMinSegmentEpochs;
    bool enable_fixed_bridge_burst_guard = false;
    double fixed_bridge_burst_guard_max_anchor_gap_s = kDefaultFixedBridgeBurstGuardMaxAnchorGapSeconds;
    double fixed_bridge_burst_guard_min_boundary_gap_s = kDefaultFixedBridgeBurstGuardMinBoundaryGapSeconds;
    double fixed_bridge_burst_guard_max_residual_m = kDefaultFixedBridgeBurstGuardMaxResidualMeters;
    int fixed_bridge_burst_guard_max_segment_epochs = kDefaultFixedBridgeBurstGuardMaxSegmentEpochs;
    RTKTuningPreset preset = RTKTuningPreset::NONE;
    bool ratio_threshold_set = false;
    bool ar_filter_margin_set = false;
    bool min_satellites_for_ar_set = false;
    bool min_subset_sats_for_ar_set = false;
    bool min_subset_systems_for_ar_set = false;
    bool min_subset_frequencies_for_ar_set = false;
    bool min_subset_dual_frequency_sats_for_ar_set = false;
    bool min_hold_count_set = false;
    bool hold_ratio_threshold_set = false;
    bool max_position_jump_min_m_set = false;
    bool max_position_jump_rate_mps_set = false;
    bool min_full_ratio_for_subset_ar_set = false;
    libgnss::RTKProcessor::RTKConfig::ARPolicy ar_policy =
        libgnss::RTKProcessor::RTKConfig::ARPolicy::EXTENDED;
    double max_hold_divergence_m = 0.0;
    double max_position_jump_m = 5.0;
    double max_fixed_anchor_age_s = 0.0;
    double max_fixed_doppler_consensus_m = 0.0;
    double max_position_jump_min_m = 0.0;
    double max_position_jump_rate_mps = 0.0;
    double max_float_spp_divergence_m = 0.0;
    double max_float_prefit_residual_rms_m = 0.0;
    double max_float_prefit_residual_max_m = 0.0;
    int max_float_prefit_residual_reset_streak = 3;
    bool max_float_prefit_residual_rms_m_set = false;
    bool max_float_prefit_residual_max_m_set = false;
    bool max_float_prefit_residual_reset_streak_set = false;
    double min_float_prefit_residual_trusted_jump_m = 0.0;
    double max_fixed_prefit_residual_rms_m = 0.0;
    int min_fixed_prefit_outliers = 0;
    double max_fixed_overconfidence_covariance_trace_m2 = 0.0;
    int fixed_prefit_reset_streak = 1;
    bool fixed_prefit_quarantine_only = false;
    double max_update_nis_per_observation = 0.0;
    double max_fixed_update_nis_per_observation = 0.0;
    double max_fixed_update_post_residual_rms_m = 0.0;
    double max_fixed_update_gate_ratio = 0.0;
    double min_fixed_update_gate_baseline_m = 0.0;
    double max_fixed_update_gate_baseline_m = 0.0;
    double min_fixed_update_gate_speed_mps = 0.0;
    double max_fixed_update_gate_speed_mps = 0.0;
    double max_fixed_update_secondary_gate_ratio = 0.0;
    double min_fixed_update_secondary_gate_baseline_m = 0.0;
    double max_fixed_update_secondary_gate_baseline_m = 0.0;
    double min_fixed_update_secondary_gate_speed_mps = 0.0;
    double max_fixed_update_secondary_gate_speed_mps = 0.0;
    double demote_fixed_status_nis_per_observation = 0.0;
    double demote_fixed_status_post_residual_rms_m = 0.0;
    double demote_fixed_status_max_ratio = 0.0;
    double demote_fixed_status_gate_ratio = 0.0;
    int demote_fixed_status_min_satellites = 0;
    int demote_fixed_status_low_satellite_ceiling = 0;
    double demote_fixed_status_low_satellite_max_ratio = 0.0;
    double min_demote_fixed_status_baseline_m = 0.0;
    double max_demote_fixed_status_baseline_m = 0.0;
    bool enable_realtime_fix_integrity = false;
    // Fix 1 (agent/realtime-fix-integrity follow-up): opt-in, off by
    // default. Ported field-for-field from the frozen offline external
    // audit and safe there (Shinjuku-Trimble: 22 caught / 0 harmed), but
    // net-harmful on the plain KF PPC baseline (nagoya2: 80 caught / 177
    // harmed) -- see output/ppc_realtime_fix_integrity_matrix.md's "After
    // fix" section. Recommended for low-FIX-rate receivers matching the
    // audited offline external policy.
    bool enable_integrity_base_gate = false;
    std::string integrity_shadow_csv_path;
    std::string integrity_log_path;
    double integrity_shadow_match_tolerance_s = 0.25;
    double integrity_shadow_max_age_s = 1.0;
    double integrity_shadow_max_gdop = 4.0;
    double integrity_shadow_max_ddpr_rms_m = 40.0;
    int integrity_shadow_min_satellites = 8;
    double integrity_shadow_default_covariance_trace_m2 = 4.0;
    // Health must bound position accuracy, not just availability -- see
    // libgnss::ShadowEstimateHealthGate's doc comment. A missing/unpopulated
    // (non-positive) covariance trace disables demotion authority unless
    // integrity_shadow_assume_default_covariance_trace is explicitly set.
    double integrity_shadow_max_covariance_trace_m2 = 4.0;
    bool integrity_shadow_assume_default_covariance_trace = false;
    bool integrity_shadow_require_fixed_status = true;
    int max_consecutive_float_for_reset = 0;
    int max_consecutive_nonfix_for_reset = 0;
    double max_postfix_residual_rms = 0.0;
    bool enable_wide_lane_ar = false;
    bool wide_lane_ar_set = false;
    double wide_lane_acceptance_threshold = 0.25;
    bool wide_lane_acceptance_threshold_set = false;
    bool enable_wlnl_fallback = false;
    bool enable_bsr_guided_decimation = false;
    int bsr_guided_worst_axes = 3;
    int bsr_guided_max_drop_steps = 6;
    int lambda_candidate_shadow_count = 0;
    double lambda_src_par_shadow_success_rate = 0.0;
    double lambda_src_par_shadow_covariance_scale = 1.0;
    int lambda_satellite_par_shadow_max_drop_steps = 0;
    double lambda_satellite_par_shadow_covariance_scale = 16.0;
    bool enable_lambda_satellite_par_shadow_quality_diverse = false;
    bool enable_lambda_l1_l5_wlnl_shadow = false;
    bool enable_lambda_l1_l5_wlnl_causal_arcs = false;
    bool enable_safe_fix_shadow_state_machine = false;
    bool enable_safe_fix_robust_consensus_shadow = false;
    bool require_safe_fix_independent_failure_budget = false;
    bool enable_library_fixed_quality_gate = false;
    double library_fixed_quality_max_covariance_trace_m2 = 0.00025;
    double library_fixed_quality_max_nis_per_observation = 10.0;
    int library_fixed_quality_min_strong_observations = 28;
    double library_fixed_quality_strong_max_nis_per_observation = 1.0;
    bool enable_safe_float_continuity = false;
    double safe_float_continuity_max_age_s = 6.0;
    bool enable_safe_availability_fallback = false;
    // WP7: NLOS/multipath measurement weighting. Empty path / OFF mode
    // (both defaults) mean the feature is entirely inert.
    std::string nlos_weights_csv_path;
    libgnss::nlos_weights::NlosWeightMode nlos_weight_mode =
        libgnss::nlos_weights::NlosWeightMode::OFF;
    double nlos_two_tier_los_threshold = 0.5;
    double nlos_two_tier_sigma_inflation = 3.0;
    double nlos_continuous_los_prob_floor = 0.05;
    double nlos_tow_tolerance_s = 0.05;
    // WP8: hard exclusion mode threshold/safety-guard. No effect unless
    // --nlos-weight-mode exclude.
    double nlos_exclude_threshold = 0.5;
    int nlos_min_sats = 5;
    // WP9: float-filter trust/reset policy. LEGACY (default) is
    // bit-identical to pre-WP9 behavior.
    libgnss::float_trust_policy::FloatTrustPolicy float_trust_policy =
        libgnss::float_trust_policy::FloatTrustPolicy::LEGACY;
    double trust_lapse_qpos_m2_per_s = 10.0;
    bool trust_gate_nlos_relax = false;
    // WP10: lapse-gated trust policy + optional NLOS-fraction trigger.
    // No effect unless --float-trust-policy lapse-gated.
    double trust_lapse_gate_s = 5.0;
    double trust_lapse_gate_nlos_frac = -1.0;
    // WP10 (WP8 rec 2): AR-acceptance-only min-LOS-satellites gate.
    // <= 0 (default) disables it; requires --nlos-weights.
    int nlos_min_los_sats = 0;
    // Phase 2a: CMC-aware DD reference-satellite selection with hysteresis
    // (RTKConfig::cmc_aware_reference_selection). Off by default; see the
    // config field's doc comment in rtk.hpp for the full algorithm.
    bool cmc_aware_reference_selection = false;
    double cmc_ref_level_m = 0.75;
    int cmc_ref_switch_epochs = 3;
    double cmc_ref_return_min_elev_deg = 5.0;
    double cmc_ref_switch_max_elev_drop_deg = 10.0;
    double cmc_ref_switch_min_elev_deg = 30.0;
    // navi.776 A2: innovation-based adaptive measurement variance
    // (RTKConfig::enable_adaptive_measurement_noise). Off by default; see
    // the config field's doc comment in rtk.hpp.
    bool rtk_adaptive_noise = false;
    double rtk_adaptive_noise_alpha_phase = 0.9;
    double rtk_adaptive_noise_alpha_code = 0.5;
    double rtk_adaptive_noise_min_scale = 0.25;
    double rtk_adaptive_noise_max_scale = 25.0;
    double rtk_adaptive_noise_max_baseline_m = 0.0;
    // Self-reference-free integer validation. This is independently opt-in;
    // it does not require the IMU tight-coupling path.
    bool cp_pr_fixed_gate = false;
    double cp_pr_fixed_gate_threshold_m = 10.0;
    int cp_pr_fixed_gate_min_pairs = 4;
    int cp_pr_fixed_gate_max_bad_pairs = 1;
    int cp_pr_fixed_gate_escalation_epochs = 2;
    double ddpr_anchor_fde_threshold_m = 5.0;
    int ddpr_anchor_max_fde_removals = 3;
};

using libgnss_apps::timeDiffSeconds;
using libgnss_apps::interpolateBaseEpoch;

bool shouldDemoteFixedStatus(const SolveConfig& config,
                             const libgnss::PositionSolution& solution) {
    if (!solution.isFixed()) {
        return false;
    }

    if (config.demote_fixed_status_min_satellites > 0 &&
        solution.num_satellites < config.demote_fixed_status_min_satellites) {
        return true;
    }

    if (config.demote_fixed_status_low_satellite_ceiling > 0 &&
        config.demote_fixed_status_low_satellite_max_ratio > 0.0 &&
        solution.num_satellites <= config.demote_fixed_status_low_satellite_ceiling &&
        std::isfinite(solution.ratio) &&
        solution.ratio <= config.demote_fixed_status_low_satellite_max_ratio) {
        return true;
    }

    if (std::isfinite(config.demote_fixed_status_max_ratio) &&
        config.demote_fixed_status_max_ratio > 0.0 &&
        std::isfinite(solution.ratio) &&
        solution.ratio <= config.demote_fixed_status_max_ratio) {
        return true;
    }

    const bool nis_enabled =
        std::isfinite(config.demote_fixed_status_nis_per_observation) &&
        config.demote_fixed_status_nis_per_observation > 0.0;
    const bool post_rms_enabled =
        std::isfinite(config.demote_fixed_status_post_residual_rms_m) &&
        config.demote_fixed_status_post_residual_rms_m > 0.0;
    if (!nis_enabled && !post_rms_enabled) {
        return false;
    }

    if (config.demote_fixed_status_gate_ratio > 0.0) {
        if (!std::isfinite(solution.ratio) ||
            solution.ratio > config.demote_fixed_status_gate_ratio) {
            return false;
        }
    }
    if (config.min_demote_fixed_status_baseline_m > 0.0 &&
        solution.baseline_length < config.min_demote_fixed_status_baseline_m) {
        return false;
    }
    if (config.max_demote_fixed_status_baseline_m > 0.0 &&
        solution.baseline_length > config.max_demote_fixed_status_baseline_m) {
        return false;
    }

    if (nis_enabled &&
        std::isfinite(solution.rtk_update_normalized_innovation_squared_per_observation) &&
        solution.rtk_update_normalized_innovation_squared_per_observation >
            config.demote_fixed_status_nis_per_observation) {
        return true;
    }
    if (post_rms_enabled &&
        std::isfinite(solution.rtk_update_post_suppression_residual_rms_m) &&
        solution.rtk_update_post_suppression_residual_rms_m >
            config.demote_fixed_status_post_residual_rms_m) {
        return true;
    }
    return false;
}

std::vector<std::string> parseCsvFields(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        if (ch == '"') {
            if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
                field.push_back('"');
                ++i;
            } else {
                quoted = !quoted;
            }
        } else if (ch == ',' && !quoted) {
            fields.push_back(field);
            field.clear();
        } else {
            field.push_back(ch);
        }
    }
    fields.push_back(field);
    return fields;
}

std::optional<double> parseFiniteDouble(const std::string& text) {
    if (text.empty()) return std::nullopt;
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

class IntegrityShadowTimeline {
public:
    struct Config {
        double match_tolerance_s = 0.25;
        libgnss::ShadowEstimateHealthGate::Config health;
    };

    explicit IntegrityShadowTimeline(Config config)
        : config_(config), health_gate_(config.health) {}

    // Diagnostics for reporting how often the shadow qualifies as demotion
    // authority under the configured health gate.
    std::uint64_t lookups() const { return lookups_; }
    std::uint64_t healthyLookups() const { return healthy_lookups_; }

    bool load(const std::string& path) {
        std::ifstream input(path);
        if (!input) return false;
        std::string line;
        if (!std::getline(input, line)) return false;
        const auto header = parseCsvFields(line);
        std::map<std::string, std::size_t> columns;
        for (std::size_t i = 0; i < header.size(); ++i) columns[header[i]] = i;
        for (const char* required : {"tow", "status", "x_ecef_m", "y_ecef_m", "z_ecef_m"}) {
            if (columns.find(required) == columns.end()) return false;
        }

        auto field = [&](const std::vector<std::string>& row,
                         const char* name) -> std::string {
            const auto it = columns.find(name);
            return it != columns.end() && it->second < row.size()
                ? row[it->second]
                : std::string{};
        };
        while (std::getline(input, line)) {
            const auto row = parseCsvFields(line);
            const auto tow = parseFiniteDouble(field(row, "tow"));
            const auto x = parseFiniteDouble(field(row, "x_ecef_m"));
            const auto y = parseFiniteDouble(field(row, "y_ecef_m"));
            const auto z = parseFiniteDouble(field(row, "z_ecef_m"));
            if (!tow || !x || !y || !z) continue;
            Sample sample;
            sample.tow_s = *tow;
            sample.position_ecef = Eigen::Vector3d(*x, *y, *z);
            const std::string status = field(row, "status");
            sample.status_fixed = status == "FIXED" || status == "4";
            sample.status_present = sample.status_fixed || status == "FLOAT" ||
                status == "3";
            sample.gdop = parseFiniteDouble(field(row, "gdop"));
            sample.ddpr_rms_m = parseFiniteDouble(field(row, "ddpr_rms_m"));
            if (const auto nsat = parseFiniteDouble(field(row, "nsat"))) {
                sample.num_satellites = static_cast<int>(*nsat);
            }
            sample.covariance_trace_m2 = parseFiniteDouble(
                field(row, "position_covariance_trace_m2"));
            if (const auto generation = parseFiniteDouble(
                    field(row, "reset_generation"))) {
                sample.reset_generation = static_cast<std::uint64_t>(
                    std::max(0.0, *generation));
            }
            samples_.push_back(std::move(sample));
        }
        std::sort(samples_.begin(), samples_.end(), [](const Sample& lhs, const Sample& rhs) {
            return lhs.tow_s < rhs.tow_s;
        });
        return !samples_.empty();
    }

    libgnss::RealtimeFixIntegrityGate::IndependentEstimate lookup(
        const libgnss::GNSSTime& time) const {
        libgnss::RealtimeFixIntegrityGate::IndependentEstimate output;
        if (samples_.empty()) return output;
        // Never consume a future shadow estimate.  The CSV may have been
        // generated offline, but the gate must behave exactly like a live
        // KF/FGO subscriber at this epoch.
        const auto after = std::upper_bound(
            samples_.begin(), samples_.end(), time.tow,
            [](double tow, const Sample& sample) { return tow < sample.tow_s; });
        if (after == samples_.begin()) return output;
        const Sample* best = &*std::prev(after);
        const double age_s = time.tow - best->tow_s;
        if (age_s > config_.match_tolerance_s) return output;

        output.present = true;
        output.age_s = age_s;
        output.reset_generation = best->reset_generation;
        output.estimate.valid = best->position_ecef.allFinite();
        output.estimate.position_ecef = best->position_ecef;

        libgnss::ShadowEstimateHealthGate::Sample health_sample;
        health_sample.status_fixed = best->status_fixed;
        health_sample.status_present = best->status_present;
        health_sample.gdop = best->gdop;
        health_sample.ddpr_rms_m = best->ddpr_rms_m;
        health_sample.num_satellites = best->num_satellites;
        health_sample.covariance_trace_m2 = best->covariance_trace_m2;
        health_sample.age_s = age_s;
        const auto health = health_gate_.evaluate(health_sample);
        output.estimate.covariance_trace_m2 = health.covariance_trace_m2;

        ++lookups_;
        if (health.healthy) ++healthy_lookups_;
        return output;
    }

private:
    struct Sample {
        double tow_s = 0.0;
        bool status_fixed = false;
        bool status_present = false;
        Eigen::Vector3d position_ecef = Eigen::Vector3d::Zero();
        std::optional<double> gdop;
        std::optional<double> ddpr_rms_m;
        std::optional<int> num_satellites;
        std::optional<double> covariance_trace_m2;
        std::uint64_t reset_generation = 0;
    };

    Config config_;
    libgnss::ShadowEstimateHealthGate health_gate_;
    std::vector<Sample> samples_;
    mutable std::uint64_t lookups_ = 0;
    mutable std::uint64_t healthy_lookups_ = 0;
};

const char* integrityStateName(libgnss::IntegrityConsensusManager::State state) {
    using State = libgnss::IntegrityConsensusManager::State;
    switch (state) {
        case State::NORMAL: return "NORMAL";
        case State::SUSPECT: return "SUSPECT";
        case State::QUARANTINE: return "QUARANTINE";
        case State::RECOVERY: return "RECOVERY";
    }
    return "UNKNOWN";
}

class IntegrityTelemetryWriter {
public:
    bool open(const std::string& path) {
        if (path.empty()) return true;
        output_.open(path);
        if (!output_) return false;
        output_ << "gps_week,tow,status,state,reasons,allow_fixed,request_primary_reset,"
                   "promote_joint_anchor,disagreement_m,aperture_m,recovery_streak,"
                   "primary_suspect,hard_primary_suspect,independent_present,"
                   "independent_valid,independent_age_s,primary_covariance_trace_m2,"
                   "independent_covariance_trace_m2,independent_reset_generation,"
                   "residual_streak_match,residual_streak_demoted,residual_spike_demoted,"
                   "consensus_demoted,output_demoted,output_latency_epochs,"
                   "base_confidence_demoted\n";
        return true;
    }

    void write(const libgnss::RealtimeFixIntegrityGate::Emission& emission) {
        if (!output_) return;
        const auto& solution = emission.solution;
        const auto& telemetry = emission.telemetry;
        output_ << solution.time.week << ',' << std::fixed << std::setprecision(3)
                << solution.time.tow << ',' << static_cast<int>(solution.status) << ','
                << integrityStateName(telemetry.consensus.state) << ','
                << telemetry.consensus.reasons << ','
                << telemetry.consensus.allow_fixed << ','
                << telemetry.consensus.request_primary_reset << ','
                << telemetry.consensus.promote_joint_anchor << ',';
        writeNumber(telemetry.consensus.disagreement_m);
        output_ << ',';
        writeNumber(telemetry.consensus.aperture_m);
        output_ << ',' << telemetry.consensus.recovery_streak << ','
                << telemetry.primary_suspect << ','
                << telemetry.hard_primary_suspect << ','
                << telemetry.independent_present << ','
                << telemetry.independent_valid << ',';
        writeNumber(telemetry.independent_age_s);
        output_ << ',';
        writeNumber(telemetry.primary_covariance_trace_m2);
        output_ << ',';
        writeNumber(telemetry.independent_covariance_trace_m2);
        output_ << ',' << telemetry.independent_reset_generation << ','
                << telemetry.residual_streak_match << ','
                << telemetry.residual_streak_demoted << ','
                << telemetry.residual_spike_demoted << ','
                << telemetry.consensus_demoted << ','
                << telemetry.output_demoted << ','
                << telemetry.output_latency_epochs << ','
                << telemetry.base_confidence_demoted << '\n';
    }

private:
    void writeNumber(double value) {
        if (std::isfinite(value)) output_ << std::setprecision(9) << value;
    }

    std::ofstream output_;
};

std::string modeChoiceString(ModeChoice mode) {
    switch (mode) {
        case ModeChoice::AUTO:
            return "auto";
        case ModeChoice::KINEMATIC:
            return "kinematic";
        case ModeChoice::STATIC:
            return "static";
        case ModeChoice::MOVING_BASE:
            return "moving-base";
    }
    return "unknown";
}

std::string ionoChoiceString(IonoChoice iono) {
    switch (iono) {
        case IonoChoice::AUTO:
            return "auto";
        case IonoChoice::OFF:
            return "off";
        case IonoChoice::IFLC:
            return "iflc";
        case IonoChoice::EST:
            return "est";
    }
    return "unknown";
}

std::string glonassARChoiceString(GlonassARChoice choice) {
    switch (choice) {
        case GlonassARChoice::OFF:
            return "off";
        case GlonassARChoice::ON:
            return "on";
        case GlonassARChoice::AUTOCAL:
            return "autocal";
    }
    return "off";
}

std::string outputFormatString(libgnss::io::SolutionWriter::Format format) {
    switch (format) {
        case libgnss::io::SolutionWriter::Format::POS:
            return "pos";
        case libgnss::io::SolutionWriter::Format::LLH:
            return "llh";
        case libgnss::io::SolutionWriter::Format::XYZ:
            return "xyz";
    }
    return "pos";
}

// PPC pipeline 互換の診断 CSV ライター。 my-branch と同じ列構成を維持し、
// develop に存在しないフィールド (alt_lambda_*, glonass_icb_*, cascade_*) は
// 0/NaN/空でスタブ出力 (forward-compat: cascade branch では実値が入る)。
struct EpochDiagnostics {
    int epoch_index = 0;
    int gps_week = 0;
    double tow = 0.0;
    bool exact_base = false;
    bool interpolated_base = false;
    bool initial_valid = false;
    int initial_status = 0;
    int initial_sats = 0;
    double initial_ratio = 0.0;
    double initial_pdop = 999.9;
    double initial_baseline_m = 0.0;
    double initial_residual_rms = 0.0;
    double initial_residual_abs_max = 0.0;
    int initial_update_rows = 0;
    int initial_suppressed_outliers = 0;
    bool final_valid = false;
    int final_status = 0;
    int final_sats = 0;
    double final_ratio = 0.0;
    double final_pdop = 999.9;
    double final_baseline_m = 0.0;
    double final_residual_rms = 0.0;
    double final_residual_abs_max = 0.0;
    int final_update_rows = 0;
    int final_suppressed_outliers = 0;
    bool spp_valid = false;
    int spp_sats = 0;
    double spp_pdop = 999.9;
    double candidate_vs_spp_m = std::numeric_limits<double>::quiet_NaN();
    double candidate_jump_m = std::numeric_limits<double>::quiet_NaN();
    double last_output_dt_s = std::numeric_limits<double>::quiet_NaN();
    double nonfixed_jump_guard_m = std::numeric_limits<double>::quiet_NaN();
    double drift_from_fixed_m = std::numeric_limits<double>::quiet_NaN();
    double height_from_fixed_m = std::numeric_limits<double>::quiet_NaN();
    double dt_since_fixed_s = std::numeric_limits<double>::quiet_NaN();
    double fixed_drift_guard_m = std::numeric_limits<double>::quiet_NaN();
    double fixed_height_guard_m = std::numeric_limits<double>::quiet_NaN();
    bool spp_replacement_used = false;
    bool output_added = false;
    std::string rejection_reason = "none";
    // cascade fields (develop port: stub. cascade branch では PositionSolution
    // から populate される。 PPC pipeline の 70-col schema に対する forward-compat 拡張。)
    bool initial_cascade_used = false;
    int initial_cascade_wl_attempted = 0;
    int initial_cascade_wl_fixed = 0;
    int initial_cascade_n1_fixed = 0;
    double initial_cascade_wl_acceptance_rate = std::numeric_limits<double>::quiet_NaN();
    int initial_cascade_l5_pairs_attempted = 0;
    int initial_cascade_l5_pairs_fixed = 0;
    int initial_cascade_l5_cross_validation_rejects = 0;
    bool final_cascade_used = false;
    int final_cascade_wl_attempted = 0;
    int final_cascade_wl_fixed = 0;
    int final_cascade_n1_fixed = 0;
    double final_cascade_wl_acceptance_rate = std::numeric_limits<double>::quiet_NaN();
    int final_cascade_l5_pairs_attempted = 0;
    int final_cascade_l5_pairs_fixed = 0;
    int final_cascade_l5_cross_validation_rejects = 0;
};

void writeDiagnosticsHeader(std::ostream& out) {
    out << "epoch_index,gps_week,tow,exact_base,interpolated_base,"
        << "initial_valid,initial_status,initial_sats,initial_ratio,initial_pdop,"
        << "initial_baseline_m,initial_residual_rms,initial_residual_abs_max,"
        << "initial_update_rows,initial_suppressed_outliers,"
        << "initial_has_glonass_icb_diag,initial_glonass_icb_l1,initial_glonass_icb_l2,"
        << "initial_glonass_icb_l1_sigma,initial_glonass_icb_l2_sigma,"
        << "initial_glonass_icb_l1_rows,initial_glonass_icb_l2_rows,"
        << "initial_has_alt_lambda,initial_alt_lambda_rank,initial_alt_lambda_cost_ratio,"
        << "initial_alt_lambda_x,initial_alt_lambda_y,initial_alt_lambda_z,"
        << "initial_alt_lambda_dx,initial_alt_lambda_dy,initial_alt_lambda_dz,"
        << "initial_alt_lambda_dnorm,initial_alt_lambda_candidates,"
        << "initial_cascade_used,initial_cascade_wl_attempted,initial_cascade_wl_fixed,"
        << "initial_cascade_n1_fixed,initial_cascade_wl_acceptance_rate,"
        << "initial_cascade_l5_pairs_attempted,initial_cascade_l5_pairs_fixed,"
        << "initial_cascade_l5_cross_validation_rejects,"
        << "final_valid,final_status,final_sats,final_ratio,final_pdop,"
        << "final_baseline_m,final_residual_rms,final_residual_abs_max,"
        << "final_update_rows,final_suppressed_outliers,"
        << "final_has_glonass_icb_diag,final_glonass_icb_l1,final_glonass_icb_l2,"
        << "final_glonass_icb_l1_sigma,final_glonass_icb_l2_sigma,"
        << "final_glonass_icb_l1_rows,final_glonass_icb_l2_rows,"
        << "final_has_alt_lambda,final_alt_lambda_rank,final_alt_lambda_cost_ratio,"
        << "final_alt_lambda_x,final_alt_lambda_y,final_alt_lambda_z,"
        << "final_alt_lambda_dx,final_alt_lambda_dy,final_alt_lambda_dz,"
        << "final_alt_lambda_dnorm,final_alt_lambda_candidates,"
        << "final_cascade_used,final_cascade_wl_attempted,final_cascade_wl_fixed,"
        << "final_cascade_n1_fixed,final_cascade_wl_acceptance_rate,"
        << "final_cascade_l5_pairs_attempted,final_cascade_l5_pairs_fixed,"
        << "final_cascade_l5_cross_validation_rejects,"
        << "spp_valid,spp_sats,spp_pdop,candidate_vs_spp_m,candidate_jump_m,"
        << "last_output_dt_s,nonfixed_jump_guard_m,drift_from_fixed_m,"
        << "height_from_fixed_m,dt_since_fixed_s,fixed_drift_guard_m,"
        << "fixed_height_guard_m,spp_replacement_used,output_added,rejection_reason\n";
}

// develop 用 fill: alt_lambda / glonass_icb は develop に存在せずスタブ
void fillSolutionDiagnostics(const libgnss::PositionSolution& solution,
                             bool& valid, int& status, int& sats,
                             double& ratio, double& pdop, double& baseline_m,
                             double& residual_rms, double& residual_abs_max,
                             int& update_rows, int& suppressed_outliers) {
    valid = solution.isValid();
    status = static_cast<int>(solution.status);
    sats = solution.num_satellites;
    ratio = solution.ratio;
    pdop = solution.pdop;
    baseline_m = solution.baseline_length;
    residual_rms = solution.rtk_update_post_suppression_residual_rms_m > 0.0
                       ? solution.rtk_update_post_suppression_residual_rms_m
                       : solution.residual_rms;
    residual_abs_max = solution.rtk_update_post_suppression_residual_max_m;
    update_rows = solution.rtk_update_observations;
    suppressed_outliers = solution.rtk_update_suppressed_outliers;
}

void writeDiagnosticsRow(std::ostream& out, const EpochDiagnostics& d) {
    const char* nan_str = "nan";
    out << d.epoch_index << ','
        << d.gps_week << ','
        << std::fixed << std::setprecision(3) << d.tow << ','
        << (d.exact_base ? 1 : 0) << ','
        << (d.interpolated_base ? 1 : 0) << ','
        << (d.initial_valid ? 1 : 0) << ','
        << d.initial_status << ','
        << d.initial_sats << ','
        << std::setprecision(6) << d.initial_ratio << ','
        << d.initial_pdop << ','
        << d.initial_baseline_m << ','
        << d.initial_residual_rms << ','
        << d.initial_residual_abs_max << ','
        << d.initial_update_rows << ','
        << d.initial_suppressed_outliers << ','
        // initial glonass_icb (stub: develop に無い)
        << "0," << nan_str << "," << nan_str << "," << nan_str << "," << nan_str
        << ",0,0,"
        // initial alt_lambda (stub)
        << "0,0," << nan_str << "," << nan_str << "," << nan_str << "," << nan_str
        << "," << nan_str << "," << nan_str << "," << nan_str << "," << nan_str << ",,"
        // initial cascade
        << (d.initial_cascade_used ? 1 : 0) << ','
        << d.initial_cascade_wl_attempted << ','
        << d.initial_cascade_wl_fixed << ','
        << d.initial_cascade_n1_fixed << ','
        << d.initial_cascade_wl_acceptance_rate << ','
        << d.initial_cascade_l5_pairs_attempted << ','
        << d.initial_cascade_l5_pairs_fixed << ','
        << d.initial_cascade_l5_cross_validation_rejects << ','
        << (d.final_valid ? 1 : 0) << ','
        << d.final_status << ','
        << d.final_sats << ','
        << d.final_ratio << ','
        << d.final_pdop << ','
        << d.final_baseline_m << ','
        << d.final_residual_rms << ','
        << d.final_residual_abs_max << ','
        << d.final_update_rows << ','
        << d.final_suppressed_outliers << ','
        // final glonass_icb (stub)
        << "0," << nan_str << "," << nan_str << "," << nan_str << "," << nan_str
        << ",0,0,"
        // final alt_lambda (stub)
        << "0,0," << nan_str << "," << nan_str << "," << nan_str << "," << nan_str
        << "," << nan_str << "," << nan_str << "," << nan_str << "," << nan_str << ",,"
        // final cascade
        << (d.final_cascade_used ? 1 : 0) << ','
        << d.final_cascade_wl_attempted << ','
        << d.final_cascade_wl_fixed << ','
        << d.final_cascade_n1_fixed << ','
        << d.final_cascade_wl_acceptance_rate << ','
        << d.final_cascade_l5_pairs_attempted << ','
        << d.final_cascade_l5_pairs_fixed << ','
        << d.final_cascade_l5_cross_validation_rejects << ','
        << (d.spp_valid ? 1 : 0) << ','
        << d.spp_sats << ','
        << d.spp_pdop << ','
        << d.candidate_vs_spp_m << ','
        << d.candidate_jump_m << ','
        << d.last_output_dt_s << ','
        << d.nonfixed_jump_guard_m << ','
        << d.drift_from_fixed_m << ','
        << d.height_from_fixed_m << ','
        << d.dt_since_fixed_s << ','
        << d.fixed_drift_guard_m << ','
        << d.fixed_height_guard_m << ','
        << (d.spp_replacement_used ? 1 : 0) << ','
        << (d.output_added ? 1 : 0) << ','
        << d.rejection_reason << '\n';
}

class EpochDebugWriter {
public:
    bool open(const std::string& path) {
        if (path.empty()) {
            return true;
        }
        file_.open(path);
        if (!file_.is_open()) {
            return false;
        }
        file_ << "gps_week,tow,status,num_sats,ratio,baseline_m,"
              << "ar_attempted,input_pair_count,pair_count,max_ambiguity_variance,"
              << "effective_ratio_threshold,ratio_satellite_count,min_subset_pair_count,min_full_ratio_for_subset_ar,"
              << "subset_candidates_evaluated,subset_candidates_rejected_by_full_ratio,"
              << "subset_candidates_rejected_by_diversity,"
              << "bsr_guided_candidates_evaluated,bsr_guided_candidates_accepted,"
              << "wide_lane_total,wide_lane_fixed,"
              << "wide_lane_rejected,wide_lane_min_distance,wide_lane_max_distance,"
              << "gf_slip_count,gf_slip_l1l5_count,"
              << "doppler_slip_l1_count,doppler_slip_l2_count,doppler_slip_l5_count,"
              << "code_slip_l1_count,code_slip_l2_count,code_slip_l5_count,"
              << "lli_slip_l1_count,lli_slip_l2_count,lli_slip_l5_count,"
              << "ambiguity_reset_l1_count,ambiguity_reset_l2_count,"
              << "ambiguity_reset_l5_count,"
              << "adaptive_dynamic_slip_active,consecutive_nonfix_before_bias_update,"
              << "adaptive_dynamic_slip_hold_remaining,"
              << "full_lambda_solved,full_ratio,"
              << "lambda_shadow_attempted,lambda_shadow_solved,"
              << "lambda_shadow_runtime_ms,"
              << "lambda_shadow_candidate_count,lambda_shadow_bsr,"
              << "lambda_shadow_bsr_qscale2,lambda_shadow_bsr_qscale4,"
              << "lambda_shadow_bsr_qscale8,lambda_shadow_bsr_qscale16,"
              << "lambda_shadow_best_cost,lambda_shadow_second_cost,"
              << "lambda_shadow_best_mass,lambda_shadow_effective_candidates,"
              << "lambda_shadow_best_second_disagreements,"
              << "lambda_shadow_ffrt_table_supported,"
              << "lambda_shadow_ffrt_accepts_any,lambda_shadow_ffrt_passed,"
              << "lambda_shadow_ffrt_min_ratio,"
              << "lambda_shadow_best_ecef_x,lambda_shadow_best_ecef_y,"
              << "lambda_shadow_best_ecef_z,"
              << "lambda_shadow_best_correction_x,"
              << "lambda_shadow_best_correction_y,"
              << "lambda_shadow_best_correction_z,"
              << "lambda_shadow_second_ecef_x,lambda_shadow_second_ecef_y,"
              << "lambda_shadow_second_ecef_z,"
              << "lambda_shadow_second_correction_x,"
              << "lambda_shadow_second_correction_y,"
              << "lambda_shadow_second_correction_z,";
        for (int candidate = 0; candidate < 8; ++candidate) {
            const int ordinal = candidate + 1;
            file_ << "lambda_shadow_candidate_" << ordinal << "_cost,"
                  << "lambda_shadow_candidate_" << ordinal << "_ecef_x,"
                  << "lambda_shadow_candidate_" << ordinal << "_ecef_y,"
                  << "lambda_shadow_candidate_" << ordinal << "_ecef_z,";
        }
        file_ << "lambda_shadow_second_position_delta_m,"
              << "lambda_shadow_position_spread_max_m,"
              << "lambda_src_par_shadow_attempted,"
              << "lambda_src_par_shadow_solved,"
              << "lambda_src_par_shadow_subset_size,"
              << "lambda_src_par_shadow_bsr,"
              << "lambda_src_par_shadow_ratio,"
              << "lambda_src_par_shadow_ffrt_min_ratio,"
              << "lambda_src_par_shadow_ffrt_passed,"
              << "lambda_src_par_shadow_best_ecef_x,"
              << "lambda_src_par_shadow_best_ecef_y,"
              << "lambda_src_par_shadow_best_ecef_z,"
              << "lambda_src_par_shadow_best_correction_x,"
              << "lambda_src_par_shadow_best_correction_y,"
              << "lambda_src_par_shadow_best_correction_z,"
              << "lambda_src_par_shadow_second_position_delta_m,"
              << "lambda_src_par_shadow_runtime_ms,"
              << "lambda_satellite_par_shadow_attempted,"
              << "lambda_satellite_par_shadow_subsets_evaluated,"
              << "lambda_satellite_par_shadow_solved,"
              << "lambda_satellite_par_shadow_subset_size,"
              << "lambda_satellite_par_shadow_dropped_satellites,"
              << "lambda_satellite_par_shadow_bsr,"
              << "lambda_satellite_par_shadow_ratio,"
              << "lambda_satellite_par_shadow_ffrt_min_ratio,"
              << "lambda_satellite_par_shadow_ffrt_passed,"
              << "lambda_satellite_par_shadow_best_ecef_x,"
              << "lambda_satellite_par_shadow_best_ecef_y,"
              << "lambda_satellite_par_shadow_best_ecef_z,"
              << "lambda_satellite_par_shadow_best_correction_x,"
              << "lambda_satellite_par_shadow_best_correction_y,"
              << "lambda_satellite_par_shadow_best_correction_z,"
              << "lambda_satellite_par_shadow_second_position_delta_m,"
              << "lambda_satellite_par_shadow_runtime_ms,"
              << "lambda_l1_l5_wlnl_shadow_attempted,"
              << "lambda_l1_l5_wlnl_shadow_pair_count,"
              << "lambda_l1_l5_wlnl_shadow_wl_bsr,"
              << "lambda_l1_l5_wlnl_shadow_wl_ratio,"
              << "lambda_l1_l5_wlnl_shadow_wl_ffrt_min_ratio,"
              << "lambda_l1_l5_wlnl_shadow_wl_ffrt_passed,"
              << "lambda_l1_l5_wlnl_shadow_mw_disagreements,"
              << "lambda_l1_l5_wlnl_shadow_raw_mw_disagreements,"
              << "lambda_l1_l5_wlnl_shadow_causal_arc_ready_pairs,"
              << "lambda_l1_l5_wlnl_shadow_causal_arc_resets,"
              << "lambda_l1_l5_wlnl_shadow_nl_bsr,"
              << "lambda_l1_l5_wlnl_shadow_nl_ratio,"
              << "lambda_l1_l5_wlnl_shadow_nl_ffrt_min_ratio,"
              << "lambda_l1_l5_wlnl_shadow_nl_ffrt_passed,"
              << "lambda_l1_l5_wlnl_shadow_best_ecef_x,"
              << "lambda_l1_l5_wlnl_shadow_best_ecef_y,"
              << "lambda_l1_l5_wlnl_shadow_best_ecef_z,"
              << "lambda_l1_l5_wlnl_shadow_runtime_ms,"
              << "safe_fix_shadow_enabled,safe_fix_shadow_state,"
              << "safe_fix_shadow_declared_fixed,"
              << "safe_fix_shadow_candidate_accepted,"
              << "safe_fix_shadow_held,safe_fix_shadow_revoked,"
              << "safe_fix_shadow_strong_acquisition,"
              << "safe_fix_shadow_change_point_acquisition,"
              << "safe_fix_shadow_acquisition_streak,"
              << "safe_fix_shadow_hold_epochs,"
              << "safe_fix_shadow_independent_consensus_delta_m,"
              << "safe_fix_shadow_independent_source_families,"
              << "safe_fix_shadow_joint_failure_probability,"
              << "safe_fix_shadow_failure_budget_passed,"
              << "inertial_fix_evidence_available,"
              << "inertial_fix_evidence_healthy_anchor,"
              << "inertial_fix_evidence_passed,"
              << "inertial_fix_evidence_time_error_s,"
              << "inertial_fix_evidence_position_delta_m,"
              << "inertial_fix_evidence_nis_per_dimension,"
              << "safe_float_continuity_attempted,"
              << "safe_float_continuity_used,"
              << "safe_float_continuity_solver_gap_anchor,"
              << "safe_float_continuity_anchor_age_s,"
              << "safe_float_continuity_velocity_age_s,"
              << "selected_fixed,selected_ratio,"
              << "selected_pair_count,selected_distinct_sats,selected_distinct_systems,"
              << "selected_distinct_frequencies,selected_dual_frequency_sats,"
              << "selected_fixed_ambiguities,selected_used_subset,"
              << "selected_reference_satellites,prior_held_integer_count,"
              << "prior_held_pair_count,prior_consecutive_fix_count,"
              << "prior_tracked_ambiguity_count,"
              << "used_wlnl_fallback,validation_attempted,validation_passed,"
              << "cp_pr_gate_evaluated,cp_pr_gate_rejected,cp_pr_gate_escalated,"
              << "cp_pr_gate_checked_pairs,cp_pr_gate_bad_pairs,"
              << "cp_pr_gate_rms_m,cp_pr_gate_max_m,"
              << "postfix_residual_rms,fixed_float_jump_m,fixed_candidate_x_m,"
              << "fixed_candidate_y_m,fixed_candidate_z_m,"
              << "fixed_candidate_float_separation_m,fixed_candidate_history_jump_m,"
              << "fixed_candidate_history_dt_s,fixed_candidate_doppler_consensus_distance_m,"
              << "low_count_rescue_evaluated,"
              << "low_count_rescue_passed,post_validation_rejected,"
              << "final_fixed_applied,hold_fix_attempted,hold_fix_applied,"
              << "hold_fix_candidate_pairs,hold_fix_matched_pairs,hold_fix_jump_m,"
              << "hold_fix_float_divergence_m,hold_fix_reject_reason,"
              << "library_fixed_quality_gate_enabled,"
              << "library_fixed_quality_gate_passed,"
              << "library_fixed_quality_gate_safe_shadow_branch,"
              << "library_fixed_quality_gate_covariance_branch,"
              << "library_fixed_quality_gate_strong_innovation_branch,"
              << "library_fixed_quality_gate_demoted,"
              << "reject_reason,ar_skip_reason,"
              << "float_update_observation_count,float_update_prefit_residual_rms_m,"
              << "float_update_post_suppression_residual_rms_m,"
              << "float_update_nis_per_observation,float_update_suppressed_outliers,"
              << "float_update_student_t_downweighted_rows,"
              << "float_update_student_t_minimum_weight,"
              << "float_update_student_t_mean_weight,"
              << "float_position_covariance_trace_m2\n";
        return true;
    }

    void write(const libgnss::PositionSolution& solution,
               const libgnss::RTKProcessor::EpochDebugTelemetry& telemetry) {
        if (!file_.is_open()) {
            return;
        }
        file_ << solution.time.week << ","
              << std::fixed << std::setprecision(3) << solution.time.tow << ","
              << static_cast<int>(solution.status) << ","
              << solution.num_satellites << ",";
        writeNumber(solution.ratio);
        file_ << ",";
        writeNumber(solution.baseline_length);
        file_ << ","
              << telemetry.ar_attempted << ","
              << telemetry.input_pair_count << ","
              << telemetry.pair_count << ",";
        writeNumber(telemetry.max_ambiguity_variance);
        file_ << ",";
        writeNumber(telemetry.effective_ratio_threshold);
        file_ << ',' << telemetry.ratio_satellite_count;
        file_ << ","
              << telemetry.min_subset_pair_count << ","
              << telemetry.min_full_ratio_for_subset_ar << ","
              << telemetry.subset_candidates_evaluated << ","
              << telemetry.subset_candidates_rejected_by_full_ratio << ","
              << telemetry.subset_candidates_rejected_by_diversity << ","
              << telemetry.bsr_guided_candidates_evaluated << ","
              << telemetry.bsr_guided_candidates_accepted << ","
              << telemetry.wide_lane_total << ","
              << telemetry.wide_lane_fixed << ","
              << telemetry.wide_lane_rejected << ",";
        writeNumber(telemetry.wide_lane_min_distance);
        file_ << ",";
        writeNumber(telemetry.wide_lane_max_distance);
        file_ << ","
              << telemetry.gf_slip_count << ","
              << telemetry.gf_slip_l1l5_count << ","
              << telemetry.doppler_slip_l1_count << ","
              << telemetry.doppler_slip_l2_count << ","
              << telemetry.doppler_slip_l5_count << ","
              << telemetry.code_slip_l1_count << ","
              << telemetry.code_slip_l2_count << ","
              << telemetry.code_slip_l5_count << ","
              << telemetry.lli_slip_l1_count << ","
              << telemetry.lli_slip_l2_count << ","
              << telemetry.lli_slip_l5_count << ","
              << telemetry.ambiguity_reset_l1_count << ","
              << telemetry.ambiguity_reset_l2_count << ","
              << telemetry.ambiguity_reset_l5_count << ","
              << telemetry.adaptive_dynamic_slip_active << ","
              << telemetry.consecutive_nonfix_before_bias_update << ","
              << telemetry.adaptive_dynamic_slip_hold_remaining << ","
              << telemetry.full_lambda_solved << ",";
        writeNumber(telemetry.full_ratio);
        file_ << ","
              << telemetry.lambda_shadow_attempted << ","
              << telemetry.lambda_shadow_solved << ",";
        writeNumber(telemetry.lambda_shadow_runtime_ms);
        file_ << ","
              << telemetry.lambda_shadow_candidate_count << ",";
        writeNumber(telemetry.lambda_shadow_bsr);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_bsr_qscale2);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_bsr_qscale4);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_bsr_qscale8);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_bsr_qscale16);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_best_cost);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_second_cost);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_best_mass);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_effective_candidates);
        file_ << ","
              << telemetry.lambda_shadow_best_second_disagreements << ","
              << telemetry.lambda_shadow_ffrt_table_supported << ","
              << telemetry.lambda_shadow_ffrt_accepts_any << ","
              << telemetry.lambda_shadow_ffrt_passed << ",";
        writeNumber(telemetry.lambda_shadow_ffrt_min_ratio);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_best_ecef_x);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_best_ecef_y);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_best_ecef_z);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_best_correction_x);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_best_correction_y);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_best_correction_z);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_second_ecef_x);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_second_ecef_y);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_second_ecef_z);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_second_correction_x);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_second_correction_y);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_second_correction_z);
        file_ << ",";
        for (int candidate = 0; candidate < 8; ++candidate) {
            writeNumber(
                telemetry.lambda_shadow_candidate_costs(candidate));
            file_ << ",";
            for (int axis = 0; axis < 3; ++axis) {
                writeNumber(
                    telemetry.lambda_shadow_candidate_ecef_m(
                        axis, candidate));
                file_ << ",";
            }
        }
        writeNumber(telemetry.lambda_shadow_second_position_delta_m);
        file_ << ",";
        writeNumber(telemetry.lambda_shadow_position_spread_max_m);
        file_ << ","
              << telemetry.lambda_src_par_shadow_attempted << ","
              << telemetry.lambda_src_par_shadow_solved << ","
              << telemetry.lambda_src_par_shadow_subset_size << ",";
        writeNumber(telemetry.lambda_src_par_shadow_bsr);
        file_ << ",";
        writeNumber(telemetry.lambda_src_par_shadow_ratio);
        file_ << ",";
        writeNumber(telemetry.lambda_src_par_shadow_ffrt_min_ratio);
        file_ << ","
              << telemetry.lambda_src_par_shadow_ffrt_passed << ",";
        writeNumber(telemetry.lambda_src_par_shadow_best_ecef_x);
        file_ << ",";
        writeNumber(telemetry.lambda_src_par_shadow_best_ecef_y);
        file_ << ",";
        writeNumber(telemetry.lambda_src_par_shadow_best_ecef_z);
        file_ << ",";
        writeNumber(telemetry.lambda_src_par_shadow_best_correction_x);
        file_ << ",";
        writeNumber(telemetry.lambda_src_par_shadow_best_correction_y);
        file_ << ",";
        writeNumber(telemetry.lambda_src_par_shadow_best_correction_z);
        file_ << ",";
        writeNumber(
            telemetry.lambda_src_par_shadow_second_position_delta_m);
        file_ << ",";
        writeNumber(telemetry.lambda_src_par_shadow_runtime_ms);
        file_ << ","
              << telemetry.lambda_satellite_par_shadow_attempted << ","
              << telemetry.lambda_satellite_par_shadow_subsets_evaluated << ","
              << telemetry.lambda_satellite_par_shadow_solved << ","
              << telemetry.lambda_satellite_par_shadow_subset_size << ","
              << telemetry.lambda_satellite_par_shadow_dropped_satellites
              << ",";
        writeNumber(telemetry.lambda_satellite_par_shadow_bsr);
        file_ << ",";
        writeNumber(telemetry.lambda_satellite_par_shadow_ratio);
        file_ << ",";
        writeNumber(telemetry.lambda_satellite_par_shadow_ffrt_min_ratio);
        file_ << ","
              << telemetry.lambda_satellite_par_shadow_ffrt_passed << ",";
        writeNumber(telemetry.lambda_satellite_par_shadow_best_ecef_x);
        file_ << ",";
        writeNumber(telemetry.lambda_satellite_par_shadow_best_ecef_y);
        file_ << ",";
        writeNumber(telemetry.lambda_satellite_par_shadow_best_ecef_z);
        file_ << ",";
        writeNumber(
            telemetry.lambda_satellite_par_shadow_best_correction_x);
        file_ << ",";
        writeNumber(
            telemetry.lambda_satellite_par_shadow_best_correction_y);
        file_ << ",";
        writeNumber(
            telemetry.lambda_satellite_par_shadow_best_correction_z);
        file_ << ",";
        writeNumber(
            telemetry
                .lambda_satellite_par_shadow_second_position_delta_m);
        file_ << ",";
        writeNumber(telemetry.lambda_satellite_par_shadow_runtime_ms);
        file_ << ","
              << telemetry.lambda_l1_l5_wlnl_shadow_attempted << ","
              << telemetry.lambda_l1_l5_wlnl_shadow_pair_count << ",";
        writeNumber(telemetry.lambda_l1_l5_wlnl_shadow_wl_bsr);
        file_ << ",";
        writeNumber(telemetry.lambda_l1_l5_wlnl_shadow_wl_ratio);
        file_ << ",";
        writeNumber(
            telemetry.lambda_l1_l5_wlnl_shadow_wl_ffrt_min_ratio);
        file_ << ","
              << telemetry.lambda_l1_l5_wlnl_shadow_wl_ffrt_passed << ","
              << telemetry.lambda_l1_l5_wlnl_shadow_mw_disagreements << ","
              << telemetry
                     .lambda_l1_l5_wlnl_shadow_raw_mw_disagreements
              << ","
              << telemetry
                     .lambda_l1_l5_wlnl_shadow_causal_arc_ready_pairs
              << ","
              << telemetry.lambda_l1_l5_wlnl_shadow_causal_arc_resets
              << ",";
        writeNumber(telemetry.lambda_l1_l5_wlnl_shadow_nl_bsr);
        file_ << ",";
        writeNumber(telemetry.lambda_l1_l5_wlnl_shadow_nl_ratio);
        file_ << ",";
        writeNumber(
            telemetry.lambda_l1_l5_wlnl_shadow_nl_ffrt_min_ratio);
        file_ << ","
              << telemetry.lambda_l1_l5_wlnl_shadow_nl_ffrt_passed << ",";
        writeNumber(telemetry.lambda_l1_l5_wlnl_shadow_best_ecef_x);
        file_ << ",";
        writeNumber(telemetry.lambda_l1_l5_wlnl_shadow_best_ecef_y);
        file_ << ",";
        writeNumber(telemetry.lambda_l1_l5_wlnl_shadow_best_ecef_z);
        file_ << ",";
        writeNumber(telemetry.lambda_l1_l5_wlnl_shadow_runtime_ms);
        file_ << ","
              << telemetry.safe_fix_shadow_enabled << ","
              << telemetry.safe_fix_shadow_state << ","
              << telemetry.safe_fix_shadow_declared_fixed << ","
              << telemetry.safe_fix_shadow_candidate_accepted << ","
              << telemetry.safe_fix_shadow_held << ","
              << telemetry.safe_fix_shadow_revoked << ","
              << telemetry.safe_fix_shadow_strong_acquisition << ","
              << telemetry.safe_fix_shadow_change_point_acquisition
              << ","
              << telemetry.safe_fix_shadow_acquisition_streak << ","
              << telemetry.safe_fix_shadow_hold_epochs << ",";
        writeNumber(
            telemetry
                .safe_fix_shadow_independent_consensus_delta_m);
        file_ << ","
              << telemetry
                     .safe_fix_shadow_independent_source_families
              << ",";
        writeNumber(
            telemetry.safe_fix_shadow_joint_failure_probability);
        file_ << ","
              << telemetry.safe_fix_shadow_failure_budget_passed
              << ","
              << telemetry.inertial_fix_evidence_available
              << ","
              << telemetry.inertial_fix_evidence_healthy_anchor
              << ","
              << telemetry.inertial_fix_evidence_passed
              << ",";
        writeNumber(telemetry.inertial_fix_evidence_time_error_s);
        file_ << ",";
        writeNumber(
            telemetry.inertial_fix_evidence_position_delta_m);
        file_ << ",";
        writeNumber(
            telemetry.inertial_fix_evidence_nis_per_dimension);
        file_
              << ","
              << telemetry.safe_float_continuity_attempted << ","
              << telemetry.safe_float_continuity_used << ","
              << telemetry.safe_float_continuity_solver_gap_anchor
              << ",";
        writeNumber(telemetry.safe_float_continuity_anchor_age_s);
        file_ << ",";
        writeNumber(telemetry.safe_float_continuity_velocity_age_s);
        file_ << ","
              << telemetry.selected_fixed << ",";
        writeNumber(telemetry.selected_ratio);
        file_ << ","
              << telemetry.selected_pair_count << ","
              << telemetry.selected_distinct_sats << ","
              << telemetry.selected_distinct_systems << ","
              << telemetry.selected_distinct_frequencies << ","
              << telemetry.selected_dual_frequency_sats << ","
              << telemetry.selected_fixed_ambiguities << ","
              << telemetry.selected_used_subset << ","
              << telemetry.selected_reference_satellites << ","
              << telemetry.prior_held_integer_count << ","
              << telemetry.prior_held_pair_count << ","
              << telemetry.prior_consecutive_fix_count << ","
              << telemetry.prior_tracked_ambiguity_count << ","
              << telemetry.used_wlnl_fallback << ","
              << telemetry.validation_attempted << ","
              << telemetry.validation_passed << ","
              << telemetry.cp_pr_gate_evaluated << ","
              << telemetry.cp_pr_gate_rejected << ","
              << telemetry.cp_pr_gate_escalated << ","
              << telemetry.cp_pr_gate_checked_pairs << ","
              << telemetry.cp_pr_gate_bad_pairs << ",";
        writeNumber(telemetry.cp_pr_gate_rms_m);
        file_ << ",";
        writeNumber(telemetry.cp_pr_gate_max_m);
        file_ << ",";
        writeNumber(telemetry.postfix_residual_rms);
        file_ << ",";
        writeNumber(telemetry.fixed_float_jump_m);
        file_ << ",";
        if (telemetry.fixed_candidate_position_valid) {
            writeNumber(telemetry.fixed_candidate_position_ecef.x());
        }
        file_ << ",";
        if (telemetry.fixed_candidate_position_valid) {
            writeNumber(telemetry.fixed_candidate_position_ecef.y());
        }
        file_ << ",";
        if (telemetry.fixed_candidate_position_valid) {
            writeNumber(telemetry.fixed_candidate_position_ecef.z());
        }
        file_ << ",";
        writeNumber(telemetry.fixed_candidate_float_separation_m);
        file_ << ",";
        writeNumber(telemetry.fixed_candidate_history_jump_m);
        file_ << ",";
        writeNumber(telemetry.fixed_candidate_history_dt_s);
        file_ << ",";
        writeNumber(telemetry.fixed_candidate_doppler_consensus_distance_m);
        file_ << ","
              << telemetry.low_count_rescue_evaluated << ","
              << telemetry.low_count_rescue_passed << ","
              << telemetry.post_validation_rejected << ","
              << telemetry.final_fixed_applied << ","
              << telemetry.hold_fix_attempted << ","
              << telemetry.hold_fix_applied << ","
              << telemetry.hold_fix_candidate_pairs << ","
              << telemetry.hold_fix_matched_pairs << ",";
        writeNumber(telemetry.hold_fix_jump_m);
        file_ << ",";
        writeNumber(telemetry.hold_fix_float_divergence_m);
        file_ << ","
              << telemetry.hold_fix_reject_reason << ","
              << telemetry.library_fixed_quality_gate_enabled << ","
              << telemetry.library_fixed_quality_gate_passed << ","
              << telemetry
                     .library_fixed_quality_gate_safe_shadow_branch
              << ","
              << telemetry.library_fixed_quality_gate_covariance_branch
              << ","
              << telemetry
                     .library_fixed_quality_gate_strong_innovation_branch
              << ","
              << telemetry.library_fixed_quality_gate_demoted << ","
              << telemetry.reject_reason << ","
              << libgnss::RTKProcessor::arSkipReasonToString(telemetry.ar_skip_reason) << ","
              << telemetry.float_update_observation_count << ",";
        writeNumber(telemetry.float_update_prefit_residual_rms_m);
        file_ << ",";
        writeNumber(telemetry.float_update_post_suppression_residual_rms_m);
        file_ << ",";
        writeNumber(telemetry.float_update_nis_per_observation);
        file_ << ","
              << telemetry.float_update_suppressed_outliers << ","
              << telemetry.float_update_student_t_downweighted_rows
              << ",";
        writeNumber(
            telemetry.float_update_student_t_minimum_weight);
        file_ << ",";
        writeNumber(telemetry.float_update_student_t_mean_weight);
        file_ << ",";
        writeNumber(telemetry.float_position_covariance_trace_m2);
        file_ << "\n";
        file_.flush();
    }

private:
    void writeNumber(double value) {
        if (!std::isfinite(value)) {
            return;
        }
        file_ << std::setprecision(6) << value;
    }

    std::ofstream file_;
};

void printUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name << " --data-dir <dir> [options]\n"
        << "       " << program_name
        << " --rover <file> --base <file> --nav <file> [options]\n\n"
        << "Batch RTK post-processing. Prefer a preset or TOML config for repeatable runs.\n\n"
        << "Inputs:\n"
        << "  --data-dir <dir>        Load rover.obs, base.obs, and navigation.nav\n"
        << "  --rover <file>          Rover RINEX observation file\n"
        << "  --base <file>           Base RINEX observation file\n"
        << "  --nav <file>            Navigation RINEX file\n\n"
        << "Common options:\n"
        << "  --config <path>         Load flat TOML defaults; CLI options override them\n"
        << "  --preset <name>         survey|low-cost|moving-base|odaiba\n"
        << "  --mode <name>           auto|kinematic|static|moving-base\n"
        << "  --iono <name>           auto|off|iflc|est\n"
        << "  --ratio <value>         Ambiguity ratio threshold\n"
        << "  --max-epochs <n>        Stop after n epochs (0 = no limit)\n"
        << "  --out <file>            Solution output (default: output/rtk_solution.pos)\n"
        << "  --kml <file>            Optional KML output path\n"
        << "  --no-kml                Disable KML output\n"
        << "  --verbose               Print per-epoch progress\n\n"
        << "Help:\n"
        << "  -h, --help              Show this everyday-use help\n"
        << "  --help-advanced         Show every tuning, experiment, and diagnostic option\n\n"
        << "Example:\n"
        << "  " << program_name
        << " --data-dir <run-dir> --preset low-cost --out output/rtk.pos\n";
}

void printAdvancedUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name << " [options]\n"
        << "  --config <path>           Load flat TOML defaults from [gnss_solve]; CLI wins\n"
        << "  --data-dir <dir>           Use <dir>/rover.obs, base.obs, navigation.nav\n"
        << "  --rover <file>             Rover RINEX observation file\n"
        << "  --base <file>              Base RINEX observation file\n"
        << "  --nav <file>               Navigation RINEX file\n"
        << "  --out <file>               Output solution file (default: output/rtk_solution.pos)\n"
        << "  --kml <file>               Write KML output (default: output/rtk_solution.kml)\n"
        << "  --no-kml                   Disable KML output\n"
        << "  --format <pos|llh|xyz>     Output text format (default: pos)\n"
        << "  --mode <auto|kinematic|static|moving-base>\n"
        << "                             Position mode (default: auto)\n"
        << "  --iono <auto|off|iflc|est> Ionosphere option (default: auto)\n"
        << "  --ratio <value|sat-count>  Fixed or satellite-count-aware Ratio threshold\n"
        << "  --preset <survey|low-cost|moving-base|odaiba>\n"
        << "                             Apply a named RTK tuning preset\n"
        << "  --arfilter                 Require extra ratio margin for subset AR fixes\n"
        << "  --no-arfilter              Disable subset AR filter margin even if a preset enables it\n"
        << "  --arfilter-margin <v>      Extra ratio margin for --arfilter (default: 0.25)\n"
        << "  --min-ar-sats <n>          Minimum satellites for AR (default: 5)\n"
        << "  --min-subset-ar-pairs <n>  Minimum DD pairs for subset AR (default: 4)\n"
        << "  --max-subset-ar-drop-steps <n>\n"
        << "                             Max worst-variance DD pairs dropped for subset AR (default: 6)\n"
        << "  --min-subset-ar-sats <n>   Minimum distinct satellites for subset AR\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-subset-ar-systems <n>\n"
        << "                             Minimum distinct constellations for subset AR\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-subset-ar-freqs <n>  Minimum distinct frequencies for subset AR\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-subset-ar-dual-freq-sats <n>\n"
        << "                             Minimum dual-frequency satellites for subset AR\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-full-ratio-for-subset-ar <v>\n"
        << "                             Require full-set LAMBDA ratio before subset AR\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-hold-count <n>       Consecutive fixes before hold ambiguity is allowed (default: 5)\n"
        << "  --hold-ratio-threshold <v> Ratio threshold used while hold ambiguity is active (default: 2.0)\n"
        << "  --elevation-mask-deg <v>   Elevation mask in degrees (default: 15)\n"
        << "  --process-noise-position <v>\n"
        << "                             KF process noise for position state, m^2/s (default: 1e-4)\n"
        << "  --process-noise-ambiguity <v>\n"
        << "                             KF process noise for DD ambiguity state, cycles^2/s (default: 1e-8)\n"
        << "  --process-noise-iono <v>   KF process noise for DD ionosphere state, m^2/s (default: 1e-4)\n"
        << "  --carrier-phase-sigma <v>  Carrier phase observation sigma in m (default: 0.002)\n"
        << "  --rtk-snr-weighting        Inflate RTK observation variance for low SNR links (default: off)\n"
        << "  --rtk-snr-reference-dbhz <v>\n"
        << "                             SNR with no RTK variance inflation (default: 45)\n"
        << "  --rtk-snr-max-variance-scale <v>\n"
        << "                             Max RTK low-SNR variance inflation (default: 25)\n"
        << "  --rtk-snr-min-baseline <m>\n"
        << "                             Apply RTK SNR weighting only above this baseline length\n"
        << "                             (default: 0, disabled)\n"
        << "  --cycle-slip-threshold <m> Geometry-free L1/L2 slip threshold (default: 0.05)\n"
        << "  --doppler-slip-threshold <m>\n"
        << "                             Doppler-predicted phase slip threshold (default: 0.20)\n"
        << "  --code-slip-threshold <m>  Code-minus-phase slip threshold (default: 5.0)\n"
        << "  --strict-dynamic-slip-thresholds\n"
        << "                             Use configured slip thresholds in dynamic RTK without protective floors\n"
        << "  --adaptive-dynamic-slip-thresholds\n"
        << "                             Drop dynamic slip floors only after a non-FIX streak\n"
        << "  --adaptive-dynamic-slip-nonfix-count <n>\n"
        << "                             Non-FIX epochs before adaptive slip thresholds activate (default: 3)\n"
        << "  --adaptive-dynamic-slip-hold-epochs <n>\n"
        << "                             Epochs to keep adaptive slip thresholds after activation (default: 10)\n"
        << "  --no-glonass               Disable GLONASS in RTK carrier processing\n"
        << "  --no-beidou                Disable BeiDou in RTK carrier processing\n"
        << "  --glonass-ar <off|on|autocal> GLONASS ambiguity resolution mode (default: off)\n"
        << "  --glonass-icb-l1 <m/MHz>   GLONASS L1 inter-channel bias slope (default: 0)\n"
        << "  --glonass-icb-l2 <m/MHz>   GLONASS L2 inter-channel bias slope (default: 0)\n"
        << "  --ar-policy <extended|demo5-continuous>\n"
        << "                             AR policy gate (default: extended)\n"
        << "                             demo5-continuous disables relaxed-hold-ratio,\n"
        << "                             subset/partial AR fallback, hold-fix fallback,\n"
        << "                             and Q regularization (raw Q passed to LAMBDA)\n"
        << "  --max-hold-div <v>         Max hold fix divergence from float in meters\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-pos-jump <v>         Max AR fix jump from last fixed pos in meters\n"
        << "                             (default: 5.0; pass 0 to disable, additional to history check)\n"
        << "  --max-fixed-anchor-age <s> Expire the last-FIX jump reference after s seconds\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-fixed-doppler-consensus <m>\n"
        << "                             Max FIX distance from Doppler-integrated track\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-pos-jump-min <v>     Min adaptive AR fix jump in meters (default: 0)\n"
        << "  --max-pos-jump-rate <v>    Max adaptive AR fix jump rate in m/s\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-float-spp-div <v>    Max FLOAT divergence from same-epoch SPP in meters\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-float-prefit-rms <v> Max FLOAT prefit DD residual RMS before state reset\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-float-prefit-max <v> Max FLOAT prefit DD residual magnitude before state reset\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-float-prefit-reset-streak <n>\n"
        << "                             Consecutive high-residual FLOAT epochs before reset (default: 3)\n"
        << "  --min-float-prefit-trusted-jump <v>\n"
        << "                             Require high-residual FLOAT to be this far from last trusted pos\n"
        << "                             before reset (default: 0, disabled)\n"
        << "  --max-fixed-prefit-rms <v> Max FIX prefit DD residual RMS for wrong-basin reset\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-fixed-prefit-outliers <n>\n"
        << "                             Required suppressed outliers for wrong-basin reset\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-fixed-overconfidence-cov-trace <v>\n"
        << "                             Require FLOAT position covariance trace <= v m^2\n"
        << "                             for wrong-basin reset (default: 0, disabled)\n"
        << "  --fixed-prefit-reset-streak <n>\n"
        << "                             Consecutive suspect FIX candidates before reset (default: 1)\n"
        << "  --fixed-prefit-quarantine-only\n"
        << "                             Clear hold and emit FLOAT without resetting the filter state\n"
        << "  --max-update-nis-per-obs <v>\n"
        << "                             Reject DD Kalman update when NIS/active observation exceeds v\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-fixed-update-nis-per-obs <v>\n"
        << "                             Reject only FIX candidates when update NIS/active observation exceeds v\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-fixed-update-post-rms <v>\n"
        << "                             Reject only FIX candidates when update post residual RMS exceeds v\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-fixed-update-gate-ratio <v>\n"
        << "                             Apply fixed-update gates only when AR ratio <= v\n"
        << "                             (default: 0, unconditional when gates are enabled)\n"
        << "  --min-fixed-update-gate-baseline <m>\n"
        << "                             Apply fixed-update gates only above this baseline length\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-fixed-update-gate-baseline <m>\n"
        << "                             Apply fixed-update gates only below this baseline length\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-fixed-update-gate-speed <m/s>\n"
        << "                             Apply fixed-update gates only above this candidate speed\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-fixed-update-gate-speed <m/s>\n"
        << "                             Apply fixed-update gates only below this candidate speed\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-fixed-update-secondary-gate-ratio <v>\n"
        << "                             Optional second fixed-update gate window AR ratio cap\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-fixed-update-secondary-gate-baseline <m>\n"
        << "                             Optional second fixed-update gate window baseline floor\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-fixed-update-secondary-gate-baseline <m>\n"
        << "                             Optional second fixed-update gate window baseline ceiling\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-fixed-update-secondary-gate-speed <m/s>\n"
        << "                             Optional second fixed-update gate window speed floor\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-fixed-update-secondary-gate-speed <m/s>\n"
        << "                             Optional second fixed-update gate window speed ceiling\n"
        << "                             (default: 0, disabled)\n"
        << "  --demote-fixed-status-nis-per-obs <v>\n"
        << "                             Output FIX as FLOAT when update NIS/active observation exceeds v\n"
        << "                             (default: 0, disabled)\n"
        << "  --demote-fixed-status-post-rms <v>\n"
        << "                             Output FIX as FLOAT when update post residual RMS exceeds v\n"
        << "                             (default: 0, disabled)\n"
        << "  --demote-fixed-status-max-ratio <v>\n"
        << "                             Output FIX as FLOAT when AR ratio is <= v\n"
        << "                             (default: 0, disabled)\n"
        << "  --demote-fixed-status-gate-ratio <v>\n"
        << "                             Apply fixed-status demotion only when AR ratio <= v\n"
        << "                             (default: 0, unconditional when demotion is enabled)\n"
        << "  --demote-fixed-status-min-satellites <n>\n"
        << "                             Output FIX as FLOAT when fewer than n satellites are used\n"
        << "                             (default: 0, disabled)\n"
        << "  --demote-fixed-status-low-satellite-ceiling <n>\n"
        << "                             With the paired max-ratio option, demote at <= n satellites\n"
        << "                             (default: 0, disabled)\n"
        << "  --demote-fixed-status-low-satellite-max-ratio <v>\n"
        << "                             With the paired ceiling, demote when AR ratio is <= v\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-demote-fixed-status-baseline <m>\n"
        << "                             Apply fixed-status demotion only above this baseline length\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-demote-fixed-status-baseline <m>\n"
        << "                             Apply fixed-status demotion only below this baseline length\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-postfix-rms <v>      Max L1 post-fix phase residual RMS in meters\n"
        << "                             (default: 0, disabled)\n"
        << "  --enable-wide-lane-ar      Enable MW wide-lane AR pre-step (default: off)\n"
        << "  --no-wide-lane-ar          Disable MW wide-lane AR, overriding presets\n"
        << "  --wide-lane-threshold <v>  WL float->int threshold in cycles (default: 0.25)\n"
        << "  --enable-wlnl-fallback     Enable MW WL/NL fallback after LAMBDA fails\n"
        << "  --enable-bsr-decimation    Enable BSR-guided partial AR decimation\n"
        << "                             (eigendecomposition-driven drop subsets)\n"
        << "                             alongside the variance-based progressive\n"
        << "                             drop family. Default: off.\n"
        << "  --lambda-shadow-candidates <n>\n"
        << "                             Record top-K MLAMBDA/BSR diagnostics without\n"
        << "                             changing FIX decisions (0=off, 2..32)\n"
        << "  --lambda-src-par-shadow-success-rate <p>\n"
        << "                             Shadow-only SRC partial-AR success threshold\n"
        << "                             (0=off; requires lambda shadow candidates)\n"
        << "  --lambda-src-par-shadow-covariance-scale <s>\n"
        << "                             Positive SRC/FFRT covariance inflation\n"
        << "                             (default: 1)\n"
        << "  --lambda-satellite-par-shadow-max-drops <n>\n"
        << "                             Shadow-only satellite-group sequential PAR\n"
        << "                             (0=off; requires lambda shadow candidates)\n"
        << "  --lambda-satellite-par-shadow-covariance-scale <s>\n"
        << "                             Positive satellite-PAR FFRT covariance\n"
        << "                             inflation (default: 16)\n"
        << "  --lambda-satellite-par-shadow-quality-diverse\n"
        << "                             Add deduplicated variance/residual/fractional/\n"
        << "                             elevation/SNR/azimuth satellite-drop paths\n"
        << "  --lambda-l1-l5-wlnl-shadow\n"
        << "                             Shadow-only L1/L5 WL->NL cascade with\n"
        << "                             per-stage scale-16 FFRT and MW agreement;\n"
        << "                             never changes filter/output decisions\n"
        << "  --lambda-l1-l5-wlnl-causal-arcs\n"
        << "                             Validate fresh WL searches with causal MW\n"
        << "                             arc smoothing keyed by satellite, frequency,\n"
        << "                             and reference generation (default: off)\n"
        << "  --safe-fix-shadow-state-machine\n"
        << "                             Run default-off FIX/hold/revoke telemetry;\n"
        << "                             never changes output status or filter state\n"
        << "  --safe-fix-robust-consensus-shadow\n"
        << "                             Enable the experimental default-off robust\n"
        << "                             full/satellite-PAR consensus state machine\n"
        << "                             (ratio>=1.4, pairs>=12, 3x/2cm acquisition,\n"
        << "                             strong/change-point shortcuts, no hold)\n"
        << "  --safe-fix-independent-failure-budget\n"
        << "                             Require >=2 distinct source families and\n"
        << "                             joint fixed-failure probability <=2e-6;\n"
        << "                             full and satellite-PAR count as one family\n"
        << "  --library-fixed-quality-gate\n"
        << "                             Demote library FIX unless safe-shadow or a\n"
        << "                             structural quality branch passes (default: off)\n"
        << "  --library-fixed-quality-max-cov-trace <m^2>\n"
        << "                             Covariance-branch float ECEF trace ceiling\n"
        << "                             (default: 0.00025)\n"
        << "  --library-fixed-quality-max-nis <v>\n"
        << "                             Covariance-branch NIS/observation ceiling\n"
        << "                             (default: 10)\n"
        << "  --library-fixed-quality-min-observations <n>\n"
        << "                             Strong-innovation observation floor\n"
        << "                             (default: 28)\n"
        << "  --library-fixed-quality-strong-max-nis <v>\n"
        << "                             Strong-innovation NIS/observation ceiling\n"
        << "                             (default: 1)\n"
        << "  --safe-float-continuity\n"
        << "                             Emit bounded Doppler-propagated FLOAT output\n"
        << "                             during short no-solution gaps (default: off)\n"
        << "  --safe-float-continuity-max-age <seconds>\n"
        << "                             Trusted-anchor and velocity age limit\n"
        << "                             (default: 6)\n"
        << "  --safe-availability-fallback\n"
        << "                             Preserve epoch availability after fail-closed\n"
        << "                             RTK rejection using independent SPP, then an\n"
        << "                             explicit PROPAGATED estimate with inflated\n"
        << "                             covariance; never supplies FIX authority\n"
        << "  --bsr-worst-axes <n>       Number of largest-eigenvalue Qb axes to score\n"
        << "                             per-pair loadings against (default: 3)\n"
        << "  --bsr-max-drops <n>        Max pairs to drop progressively in BSR-guided\n"
        << "                             decimation (default: 6)\n"
        << "  --nlos-weights <csv>       WP7: per-epoch per-satellite LOS/NLOS weight CSV\n"
        << "                             (columns: tow,sat,los_prob; also accepts the\n"
        << "                             tow,epoch_idx,prn,is_los,... contract emitted by\n"
        << "                             build_per_epoch_nlos_csv.py). Requires\n"
        << "                             --nlos-weight-mode to have any effect.\n"
        << "  --nlos-weight-mode <off|two-tier|continuous|exclude>\n"
        << "                             Sigma-inflation (or WP8 hard exclusion) mapping for\n"
        << "                             NLOS satellites (default: off — bit-identical to no\n"
        << "                             NLOS weighting)\n"
        << "  --nlos-two-tier-threshold <v>\n"
        << "                             los_prob below this is treated as NLOS in\n"
        << "                             two-tier mode (default: 0.5)\n"
        << "  --nlos-two-tier-inflation <v>\n"
        << "                             Sigma multiplier for NLOS satellites in two-tier\n"
        << "                             mode (default: 3.0)\n"
        << "  --nlos-continuous-floor <v>\n"
        << "                             Floor on los_prob before the 1/sqrt(...) sigma\n"
        << "                             mapping in continuous mode (default: 0.05)\n"
        << "  --nlos-tow-tolerance <s>   Tow-matching tolerance for the NLOS weight CSV\n"
        << "                             (default: 0.05)\n"
        << "  --nlos-exclude-threshold <v>\n"
        << "                             WP8: los_prob below this is dropped from DD\n"
        << "                             formation entirely in exclude mode (default: 0.5)\n"
        << "  --nlos-min-sats <n>        WP8: exclude-mode safety guard -- skip exclusion for\n"
        << "                             an epoch (keep all satellites) if it would leave\n"
        << "                             fewer than this many in the candidate set (default: 5)\n"
        << "  --float-trust-policy <legacy|cv-predict|scaled-reset|lapse-gated>\n"
        << "                             WP9/WP10: policy for resetPositionToSPP() once the\n"
        << "                             previous epoch failed to refresh position trust.\n"
        << "                             legacy (default): unconditional wide reset to\n"
        << "                             900 m^2/axis, reseeded from SPP -- bit-identical to\n"
        << "                             pre-WP9 behavior. cv-predict: propagate the previous\n"
        << "                             position with a constant-velocity predict and grow\n"
        << "                             covariance at --trust-lapse-qpos instead of jumping\n"
        << "                             to 900. scaled-reset: keep the SPP reseed but scale\n"
        << "                             the reset variance with time-since-trust instead of\n"
        << "                             using a flat 900 (from the very first lapsed epoch).\n"
        << "                             lapse-gated (WP10): scaled-reset's law, but only once\n"
        << "                             the *continuous* trust lapse exceeds --trust-lapse-\n"
        << "                             gate-s seconds (or --trust-lapse-gate-nlos-frac\n"
        << "                             triggers) -- below the gate, behaves exactly like\n"
        << "                             legacy (bit-identical), fixing scaled-reset's WP9\n"
        << "                             regression on short/benign lapses. All non-legacy\n"
        << "                             policies are capped at 900 m^2/axis (converge to\n"
        << "                             legacy under a long drought)\n"
        << "  --trust-lapse-qpos <v>     WP9/WP10: process-noise rate (m^2/s) used by\n"
        << "                             cv-predict (linear growth per second) and\n"
        << "                             scaled-reset/lapse-gated (quadratic growth in\n"
        << "                             time-since-trust). No effect when --float-trust-policy\n"
        << "                             legacy (default: 10.0; WP9/WP10's run1 winner is 0.1)\n"
        << "  --trust-lapse-gate-s <s>   WP10: seconds of continuous trust lapse required\n"
        << "                             before --float-trust-policy lapse-gated switches from\n"
        << "                             legacy to the scaled-reset law (default: 5.0). No\n"
        << "                             effect unless --float-trust-policy lapse-gated\n"
        << "  --trust-lapse-gate-nlos-frac <f>\n"
        << "                             WP10 optional second trigger for lapse-gated: when\n"
        << "                             set (>= 0) and --nlos-weights is loaded, ALSO switch\n"
        << "                             to the scaled-reset law (regardless of lapse length)\n"
        << "                             on epochs whose NLOS-flagged satellite fraction\n"
        << "                             exceeds this value (one-epoch-lagged; default: off/\n"
        << "                             disabled). No effect unless --float-trust-policy\n"
        << "                             lapse-gated\n"
        << "  --trust-gate-nlos-relax    WP9 optional lever: relax rememberSolution()'s FLOAT\n"
        << "                             trust-refresh jump gate 2x when >50% of this epoch's\n"
        << "                             tracked satellites are NLOS-flagged per --nlos-weights\n"
        << "                             (default: off; also requires --nlos-weights to have\n"
        << "                             any effect, independent of --nlos-weight-mode)\n"
        << "  --nlos-min-los-sats <n>    WP10: AR-acceptance-only gate (never touches the\n"
        << "                             float-KF update) -- require at least n LOS-flagged\n"
        << "                             satellites (per --nlos-weights) among the AR\n"
        << "                             candidate set before attempting/accepting a fix\n"
        << "                             (default: 0, disabled). Requires --nlos-weights\n"
        << "  --cmc-ref                  Phase 2a: CMC-aware DD reference-satellite selection\n"
        << "                             with hysteresis (default: off). Switches the DD\n"
        << "                             reference away from a code-minus-carrier-suspect\n"
        << "                             satellite only after --cmc-ref-switch-epochs\n"
        << "                             consecutive suspect epochs, and back only after the\n"
        << "                             same streak of non-suspect epochs plus an elevation\n"
        << "                             margin (see rtk.hpp's cmc_aware_reference_selection\n"
        << "                             doc comment for the full algorithm)\n"
        << "  --cmc-ref-level <m>        CMC suspect-classification deviation threshold in\n"
        << "                             meters (default: 0.75). No effect without --cmc-ref\n"
        << "  --cmc-ref-switch-epochs <k>\n"
        << "                             Consecutive suspect/non-suspect epochs required before\n"
        << "                             switching away/back (default: 3). No effect without\n"
        << "                             --cmc-ref\n"
        << "  --cmc-ref-return-min-elev <deg>\n"
        << "                             Minimum elevation margin (degrees) a candidate must\n"
        << "                             clear over the current reference before a switch-back\n"
        << "                             is considered (default: 5.0). No effect without\n"
        << "                             --cmc-ref\n"
        << "  --cmc-ref-switch-max-elev-drop <deg>\n"
        << "                             Elevation-quality gate on switch-away only: only\n"
        << "                             switch away from a suspect reference if the best\n"
        << "                             replacement's elevation is within this many degrees\n"
        << "                             below it (default: 10.0). No effect without --cmc-ref\n"
        << "  --cmc-ref-switch-min-elev <deg>\n"
        << "                             Companion absolute floor: the switch-away replacement\n"
        << "                             must also be above this elevation (default: 30.0). No\n"
        << "                             effect without --cmc-ref\n"
        << "  --rtk-adaptive-noise       Innovation-based adaptive measurement variance\n"
        << "                             (navi.776): per-satellite EWMA of v^2-HPH'-ref_var\n"
        << "                             replaces the model DD satellite variance, clamped\n"
        << "                             relative to it (default: off)\n"
        << "  --rtk-adaptive-noise-alpha-phase <a>\n"
        << "                             Carrier-phase EWMA memory factor (default: 0.9)\n"
        << "  --rtk-adaptive-noise-alpha-code <a>\n"
        << "                             Pseudorange EWMA memory factor (default: 0.5)\n"
        << "  --rtk-adaptive-noise-min-scale <s>\n"
        << "                             Adapted variance floor as a fraction of the model\n"
        << "                             variance (default: 0.25)\n"
        << "  --rtk-adaptive-noise-max-scale <s>\n"
        << "                             Adapted variance ceiling as a multiple of the model\n"
        << "                             variance (default: 25.0)\n"
        << "  --rtk-adaptive-noise-max-baseline <m>\n"
        << "                             Only adapt while the float baseline is at or below\n"
        << "                             this many meters (default: 0 = no gate). Long\n"
        << "                             baselines feed DD iono error into the adaptation\n"
        << "  --cp-pr-fixed-gate         Validate an integer candidate with independent\n"
        << "                             DD code-vs-carrier innovations before accepting it\n"
        << "                             (default: off; does not require tight coupling)\n"
        << "  --cp-pr-fixed-gate-threshold <m>\n"
        << "                             Per-pair innovation threshold (default: 10.0)\n"
        << "  --cp-pr-fixed-gate-min-pairs <n>\n"
        << "                             Minimum checked pairs (default: 4)\n"
        << "  --cp-pr-fixed-gate-max-bad-pairs <n>\n"
        << "                             Allowed pairs above threshold (default: 1)\n"
        << "  --cp-pr-fixed-gate-escalation-epochs <n>\n"
        << "                             Consecutive vetoes before DDPR anchor solve (default: 2)\n"
        << "  --ddpr-anchor-fde-threshold <m>\n"
        << "                             DDPR anchor leave-one-out threshold (default: 5.0)\n"
        << "  --ddpr-anchor-max-fde-removals <n>\n"
        << "                             DDPR anchor maximum removals (default: 3)\n"
        << "  --low-ratio-guard-threshold <ratio>\n"
        << "                             Below this ratio, require a strongly constrained\n"
        << "                             integer candidate (default: 0, disabled)\n"
        << "  --low-ratio-min-fixed-ambiguities <n>\n"
        << "                             Required DD integers below the guard threshold\n"
        << "                             (default: 0, disabled)\n"
        << "  --low-count-rescue-ratio <ratio>\n"
        << "                             Rescue a small integer subset only above this\n"
        << "                             strong LAMBDA ratio (default: 0, disabled)\n"
        << "  --low-count-rescue-min-fixed <n>\n"
        << "                             Minimum integers for rescue (default: 4)\n"
        << "  --low-count-rescue-max-history-speed <m/s>\n"
        << "                             Maximum implied speed from previous FIX\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-consec-float-reset <n>\n"
        << "                             Reset ambiguity state after n consecutive float epochs\n"
        << "                             (default: 0, disabled; e.g. 10 for aggressive urban reconvergence)\n"
        << "  --max-consec-nonfix-reset <n>\n"
        << "                             Reset ambiguity state after n consecutive non-FIX epochs\n"
        << "                             including FLOAT/SPP/no-solution (default: 0, disabled)\n"
        << "  --no-nonfix-drift-guard   Disable low-speed non-FIX segment drift rejection\n"
        << "  --nonfix-drift-max-anchor-gap <s>\n"
        << "                             Max FIX-to-FIX gap for non-FIX drift guard (default: 120)\n"
        << "  --nonfix-drift-max-anchor-speed <m/s>\n"
        << "                             Max FIX-anchor speed treated as stationary (default: 1.0)\n"
        << "  --nonfix-drift-max-residual <m>\n"
        << "                             Reject non-FIX epochs farther than this from FIX-anchor bridge\n"
        << "                             in low-speed segments (default: 30)\n"
        << "  --nonfix-drift-min-horizontal-residual <m>\n"
        << "                             Require this horizontal bridge residual before rejecting\n"
        << "                             a non-FIX epoch (default: 0, disabled)\n"
        << "  --nonfix-drift-min-segment-epochs <n>\n"
        << "                             Minimum bounded non-FIX segment length to inspect (default: 20)\n"
        << "  --nonfix-drift-max-segment-epochs <n>\n"
        << "                             Maximum bounded non-FIX segment length to inspect\n"
        << "                             (default: 0, disabled)\n"
        << "  --no-spp-height-step-guard\n"
        << "                             Disable SPP vertical spike rejection\n"
        << "  --spp-height-step-min <m>  Minimum SPP height jump rejected (default: 30)\n"
        << "  --spp-height-step-rate <m/s>\n"
        << "                             Rate-scaled SPP height jump limit (default: 4)\n"
        << "  --float-bridge-tail-guard Enable slow FLOAT bridge-tail rejection (default: on)\n"
        << "  --no-float-bridge-tail-guard\n"
        << "                             Disable slow FLOAT bridge-tail rejection\n"
        << "  --float-bridge-tail-max-anchor-gap <s>\n"
        << "                             Max FIX-to-FIX gap for FLOAT bridge-tail guard (default: 120)\n"
        << "  --float-bridge-tail-min-anchor-speed <m/s>\n"
        << "                             Min horizontal FIX-anchor speed for FLOAT bridge-tail guard (default: 0.4)\n"
        << "  --float-bridge-tail-max-anchor-speed <m/s>\n"
        << "                             Max horizontal FIX-anchor speed for FLOAT bridge-tail guard (default: 1.0)\n"
        << "  --float-bridge-tail-max-residual <m>\n"
        << "                             Reject FLOAT epochs farther than this from FIX-anchor bridge\n"
        << "                             in slow bounded segments (default: 12)\n"
        << "  --float-bridge-tail-min-segment-epochs <n>\n"
        << "                             Minimum bounded FLOAT-tail segment length to inspect (default: 20)\n"
        << "  --fixed-bridge-burst-guard\n"
        << "                             Enable short FIX burst bridge-residual rejection (default: off)\n"
        << "  --no-fixed-bridge-burst-guard\n"
        << "                             Disable short FIX burst bridge-residual rejection\n"
        << "  --fixed-bridge-burst-max-anchor-gap <s>\n"
        << "                             Max FIX-anchor gap for fixed-burst guard (default: 30)\n"
        << "  --fixed-bridge-burst-min-boundary-gap <s>\n"
        << "                             Min gap around a short FIX burst (default: 1)\n"
        << "  --fixed-bridge-burst-max-residual <m>\n"
        << "                             Reject burst FIX epochs farther than this from FIX bridge (default: 20)\n"
        << "  --fixed-bridge-burst-max-segment-epochs <n>\n"
        << "                             Max short FIX segment length to inspect (default: 12)\n"
        << "  --max-baseline-m <v>       Max baseline length in meters (default: 20000)\n"
        << "  --base-ecef <x> <y> <z>    Override base ECEF position in meters\n"
        << "  --skip-epochs <n>          Skip the first n rover epochs before solving\n"
        << "  --max-epochs <n>           Stop after n rover epochs\n"
        << "  --debug-epoch-log <csv>    Write per-epoch AR/debug telemetry CSV\n"
        << "  --prefer-trusted-seed      Prefer recent trusted/fixed solution as seed\n"
        << "  --doppler-float-seed       Propagate trusted position with Doppler velocity\n"
        << "                             during short FLOAT gaps (default: off)\n"
        << "  --doppler-float-seed-max-age <s>\n"
        << "                             Maximum trusted-anchor age (default: 6)\n"
        << "  --rover-seed-pos <file>    Inject ECEF seed positions from .pos file per epoch\n"
        << "  --diagnostics-csv <file>   Write per-epoch RTK candidate diagnostics CSV (PPC pipeline format)\n"
        << "  --multisd-fgo-shadow-csv <file>\n"
        << "                             GNSS-only causal rolling-window MultiSD shadow telemetry\n"
        << "  --multisd-fgo-shadow-window <epochs>\n"
        << "                             Rolling window ending at current epoch (default: 25)\n"
        << "  --multisd-fgo-shadow-min-epochs <epochs>\n"
        << "                             Warm-up before shadow solve (default: 10)\n"
        << "  --multisd-fgo-shadow-holdout-offset <n>\n"
        << "                             Deterministic disjoint partition offset (default: 2)\n"
        << "  --multisd-fgo-shadow-top-k <n>\n"
        << "                             Bounded integer hypotheses, 2..32 (default: 4)\n"
        << "  --multisd-fgo-shadow-max-seed-separation <m>\n"
        << "                             GNSS-only RTK/FGO aperture; 0 disables (default: 0.5)\n"
        << "  --multisd-fgo-shadow-validation-history <epochs>\n"
        << "                             Causal held-out carrier span (default: 3)\n"
        << "  --multisd-fgo-shadow-min-carrier-fraction <0..1>\n"
        << "                             Required held-out rows (default: 0.75)\n"
        << "  --multisd-fgo-shadow-min-fixed-ambiguities <n>\n"
        << "                             MultiSD PAR dimension floor (default: 6)\n"
        << "  --multisd-fgo-shadow-holdout-satellites <n>\n"
        << "                             Fully disjoint satellites (default: 4)\n"
        << "  --realtime-fix-integrity   Gate FIX output with bounded-latency residual checks\n"
        << "                             (default: off; maximum latency: 7 epochs)\n"
        << "  --integrity-base-gate      Also enable the frozen offline low-satellite/ratio\n"
        << "                             confidence gate (default: off). Recommended for\n"
        << "                             low-FIX-rate receivers matching the audited offline\n"
        << "                             external policy (UrbanNav Shinjuku-Trimble: 22 caught,\n"
        << "                             0 harmed); on the plain KF PPC baseline it over-demotes\n"
        << "                             (net-harmful on at least one PPC run) -- see\n"
        << "                             output/ppc_realtime_fix_integrity_matrix.md.\n"
        << "  --no-integrity-base-gate   Disable the base confidence gate (default)\n"
        << "  --integrity-shadow-csv <file>\n"
        << "                             Add causal KF/FGO position consensus from an epoch CSV;\n"
        << "                             implies --realtime-fix-integrity\n"
        << "  --integrity-shadow-match-tolerance <s>\n"
        << "                             Max age of a shadow sample to match an epoch (default: 0.25)\n"
        << "  --integrity-shadow-max-age <s>\n"
        << "                             Max shadow sample age accepted as healthy (default: 1.0)\n"
        << "  --integrity-shadow-max-gdop <v>\n"
        << "                             Max shadow GDOP accepted as healthy (default: 4.0)\n"
        << "  --integrity-shadow-max-ddpr-rms <m>\n"
        << "                             Max shadow DD-prefit RMS accepted as healthy (default: 40.0)\n"
        << "  --integrity-shadow-min-satellites <n>\n"
        << "                             Min shadow satellite count accepted as healthy (default: 8)\n"
        << "  --integrity-shadow-max-covariance-trace <m^2>\n"
        << "                             Max shadow position_covariance_trace_m2 accepted as\n"
        << "                             demotion authority (default: 4.0)\n"
        << "  --integrity-shadow-assume-default-covariance-trace\n"
        << "                             Opt in to substituting --integrity-shadow-default-covariance-trace\n"
        << "                             when the shadow CSV's own trace is missing/non-positive\n"
        << "                             (default: off -- a missing/unpopulated trace disables\n"
        << "                             demotion authority rather than silently granting it)\n"
        << "  --integrity-shadow-default-covariance-trace <m^2>\n"
        << "                             Fallback trace used only with --integrity-shadow-assume-\n"
        << "                             default-covariance-trace (default: 4.0)\n"
        << "  --integrity-shadow-require-fixed-status\n"
        << "                             Require shadow status FIXED for demotion authority (default: on)\n"
        << "  --integrity-shadow-allow-float-status\n"
        << "                             Allow shadow status FLOAT to act as demotion authority too\n"
        << "                             (default: off; FLOAT shadow samples were found badly diverged\n"
        << "                             on PPC tokyo1 despite passing GDOP/DDPR-RMS/nsat/age)\n"
        << "  --integrity-log <file>     Write real-time integrity decisions and latency CSV\n"
        << "  --rtk-update-outlier-threshold <v>\n"
        << "                             Outlier rejection threshold for RTK measurement update (default: 30.0)\n"
        << "  --student-t-rtk-front-end  Enable Student-t(nu=4) DD covariance inflation (default: off)\n"
        << "  --no-kinematic-post-filter Disable the kinematic output post-filter\n"
        << "  --no-base-interp           Require exact rover/base epoch alignment\n"
        << "  --verbose                  Print per-epoch progress summary\n"
        << "  -h, --help                 Show concise everyday-use help\n"
        << "  --help-advanced            Show this complete option reference\n";
}

[[noreturn]] void argumentError(const std::string& message, const char* program_name) {
    std::cerr << "Argument error: " << message << "\n\n";
    printUsage(program_name);
    std::exit(1);
}

ModeChoice parseModeChoice(const std::string& value, const char* program_name) {
    if (value == "auto") return ModeChoice::AUTO;
    if (value == "kinematic") return ModeChoice::KINEMATIC;
    if (value == "static") return ModeChoice::STATIC;
    if (value == "moving-base") return ModeChoice::MOVING_BASE;
    argumentError("unsupported --mode value: " + value, program_name);
}

IonoChoice parseIonoChoice(const std::string& value, const char* program_name) {
    if (value == "auto") return IonoChoice::AUTO;
    if (value == "off") return IonoChoice::OFF;
    if (value == "iflc") return IonoChoice::IFLC;
    if (value == "est") return IonoChoice::EST;
    argumentError("unsupported --iono value: " + value, program_name);
}

GlonassARChoice parseGlonassARChoice(const std::string& value, const char* program_name) {
    if (value == "off") return GlonassARChoice::OFF;
    if (value == "on") return GlonassARChoice::ON;
    if (value == "autocal") return GlonassARChoice::AUTOCAL;
    argumentError("unsupported --glonass-ar value: " + value, program_name);
}

libgnss::nlos_weights::NlosWeightMode parseNlosWeightMode(const std::string& value,
                                                           const char* program_name) {
    if (value == "off") return libgnss::nlos_weights::NlosWeightMode::OFF;
    if (value == "two-tier") return libgnss::nlos_weights::NlosWeightMode::TWO_TIER;
    if (value == "continuous") return libgnss::nlos_weights::NlosWeightMode::CONTINUOUS;
    if (value == "exclude") return libgnss::nlos_weights::NlosWeightMode::EXCLUDE;
    argumentError("unsupported --nlos-weight-mode value: " + value, program_name);
}

libgnss::float_trust_policy::FloatTrustPolicy parseFloatTrustPolicy(const std::string& value,
                                                                     const char* program_name) {
    if (value == "legacy") return libgnss::float_trust_policy::FloatTrustPolicy::LEGACY;
    if (value == "cv-predict") return libgnss::float_trust_policy::FloatTrustPolicy::CV_PREDICT;
    if (value == "scaled-reset") return libgnss::float_trust_policy::FloatTrustPolicy::SCALED_RESET;
    if (value == "lapse-gated") return libgnss::float_trust_policy::FloatTrustPolicy::LAPSE_GATED;
    argumentError("unsupported --float-trust-policy value: " + value, program_name);
}

RTKTuningPreset parseRTKTuningPreset(const std::string& value, const char* program_name) {
    if (value == "survey") return RTKTuningPreset::SURVEY;
    if (value == "low-cost") return RTKTuningPreset::LOW_COST;
    if (value == "moving-base") return RTKTuningPreset::MOVING_BASE;
    if (value == "odaiba") return RTKTuningPreset::ODAIBA;
    argumentError("unsupported --preset value: " + value, program_name);
}

void applyRTKTuningPreset(SolveConfig& config) {
    switch (config.preset) {
        case RTKTuningPreset::NONE:
            return;
        case RTKTuningPreset::SURVEY:
            if (!config.ratio_threshold_set) config.ratio_threshold = 3.0;
            if (!config.has_ar_filter_override) config.enable_ar_filter = false;
            if (!config.ar_filter_margin_set) config.ar_filter_margin = 0.25;
            if (!config.min_satellites_for_ar_set) config.min_satellites_for_ar = 5;
            if (!config.min_hold_count_set) config.min_hold_count = 5;
            if (!config.hold_ratio_threshold_set) config.hold_ratio_threshold = 2.0;
            return;
        case RTKTuningPreset::LOW_COST:
            if (!config.ratio_threshold_set) config.ratio_threshold = 3.0;
            if (!config.has_ar_filter_override) config.enable_ar_filter = true;
            if (!config.ar_filter_margin_set) config.ar_filter_margin = 0.35;
            if (!config.min_satellites_for_ar_set) config.min_satellites_for_ar = 6;
            if (!config.min_hold_count_set) config.min_hold_count = 8;
            if (!config.hold_ratio_threshold_set) config.hold_ratio_threshold = 2.5;
            // Motion-aware fixed-position jump gate for kinematic rovers. The
            // static 5 m jump-from-last-fix limit starves fixes once the rover
            // drives away from the last accepted fix (the limit never grows, so
            // every correct fix past ~5 m of travel is rejected and the
            // last-fixed anchor freezes). Use a velocity-scaled limit instead.
            if (!config.max_position_jump_rate_mps_set)
                config.max_position_jump_rate_mps = 30.0;
            if (!config.max_position_jump_min_m_set)
                config.max_position_jump_min_m = 5.0;
            // Require the full-constellation LAMBDA ratio to be non-degenerate
            // before admitting a subset-AR fix; subset blunders show full_ratio
            // ~1 (the rest of the constellation disagrees) yet pass the subset
            // ratio locally.
            if (!config.min_full_ratio_for_subset_ar_set)
                config.min_full_ratio_for_subset_ar = 1.5;
            // Float-divergence recovery: in urban canyons the float filter loses
            // lock and coasts tens of metres for minutes (almost half of float
            // epochs end up >10 m off). Reset the ambiguity state and re-seed
            // from SPP once the prefit DD residual stays large for several
            // epochs. Thresholds are deliberately loose (gross divergence only)
            // so good tracking is never reset — tighter values regress the
            // already-converged runs while looser ones still recover the bad
            // windows.
            if (!config.max_float_prefit_residual_rms_m_set)
                config.max_float_prefit_residual_rms_m = 4.0;
            if (!config.max_float_prefit_residual_max_m_set)
                config.max_float_prefit_residual_max_m = 10.0;
            if (!config.max_float_prefit_residual_reset_streak_set)
                config.max_float_prefit_residual_reset_streak = 5;
            return;
        case RTKTuningPreset::MOVING_BASE:
            if (!config.ratio_threshold_set) config.ratio_threshold = 2.8;
            if (!config.has_ar_filter_override) config.enable_ar_filter = true;
            if (!config.ar_filter_margin_set) config.ar_filter_margin = 0.20;
            if (!config.min_satellites_for_ar_set) config.min_satellites_for_ar = 6;
            if (!config.min_hold_count_set) config.min_hold_count = 8;
            if (!config.hold_ratio_threshold_set) config.hold_ratio_threshold = 2.4;
            return;
        case RTKTuningPreset::ODAIBA:
            if (!config.ratio_threshold_set) config.ratio_threshold = 3.0;
            if (!config.has_ar_filter_override) config.enable_ar_filter = true;
            if (!config.ar_filter_margin_set) config.ar_filter_margin = 0.35;
            if (!config.min_satellites_for_ar_set) config.min_satellites_for_ar = 6;
            if (!config.min_hold_count_set) config.min_hold_count = 8;
            if (!config.hold_ratio_threshold_set) config.hold_ratio_threshold = 2.5;
            if (!config.wide_lane_ar_set) config.enable_wide_lane_ar = true;
            if (!config.wide_lane_acceptance_threshold_set) {
                config.wide_lane_acceptance_threshold = 0.12;
            }
            return;
    }
}

libgnss::io::SolutionWriter::Format parseOutputFormat(const std::string& value,
                                                      const char* program_name) {
    if (value == "pos") return libgnss::io::SolutionWriter::Format::POS;
    if (value == "llh") return libgnss::io::SolutionWriter::Format::LLH;
    if (value == "xyz") return libgnss::io::SolutionWriter::Format::XYZ;
    argumentError("unsupported --format value: " + value, program_name);
}

SolveConfig parseArguments(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--help-advanced") {
            printAdvancedUsage(argv[0]);
            std::exit(0);
        }
    }

    const libgnss_apps::TomlCliSchema schema{
        "gnss_solve",
        {
            {"--base-interp", ""},
            {"--beidou", ""},
            {"--glonass", ""},
            {"--kinematic-post-filter", ""},
            {"--kml", ""},
            {"--wide-lane-ar", "--enable-wide-lane-ar"},
        },
        {
            {"--arfilter", "--no-arfilter"},
            {"--base-interp", "--no-base-interp"},
            {"--beidou", "--no-beidou"},
            {"--fixed-bridge-burst-guard", "--no-fixed-bridge-burst-guard"},
            {"--float-bridge-tail-guard", "--no-float-bridge-tail-guard"},
            {"--glonass", "--no-glonass"},
            {"--integrity-base-gate", "--no-integrity-base-gate"},
            {"--kinematic-post-filter", "--no-kinematic-post-filter"},
            {"--kml", "--no-kml"},
            {"--nonfix-drift-guard", "--no-nonfix-drift-guard"},
            {"--spp-height-step-guard", "--no-spp-height-step-guard"},
            {"--enable-wide-lane-ar", "--no-wide-lane-ar"},
            {"--wide-lane-ar", "--no-wide-lane-ar"},
        },
        {},
    };
    std::vector<std::string> expanded_arguments =
        libgnss_apps::expandTomlConfigArguments(argc, argv, schema);
    std::vector<char*> expanded_argv;
    expanded_argv.reserve(expanded_arguments.size());
    for (auto& argument : expanded_arguments) {
        expanded_argv.push_back(argument.data());
    }
    argc = static_cast<int>(expanded_argv.size());
    argv = expanded_argv.data();

    SolveConfig config;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--multisd-fgo-shadow-csv" && i + 1 < argc) {
            config.multisd_fgo_shadow_csv_path = argv[++i];
            continue;
        }
        if (arg == "--multisd-fgo-shadow-window" && i + 1 < argc) {
            config.multisd_fgo_shadow_window_epochs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--multisd-fgo-shadow-min-epochs" && i + 1 < argc) {
            config.multisd_fgo_shadow_min_epochs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--multisd-fgo-shadow-holdout-offset" && i + 1 < argc) {
            config.multisd_fgo_shadow_holdout_offset = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--multisd-fgo-shadow-top-k" && i + 1 < argc) {
            config.multisd_fgo_shadow_top_k = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--multisd-fgo-shadow-max-seed-separation" &&
            i + 1 < argc) {
            config.multisd_fgo_shadow_max_seed_separation_m =
                std::stod(argv[++i]);
            continue;
        }
        if (arg == "--multisd-fgo-shadow-validation-history" &&
            i + 1 < argc) {
            config.multisd_fgo_shadow_validation_history_epochs =
                std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--multisd-fgo-shadow-min-carrier-fraction" &&
            i + 1 < argc) {
            config.multisd_fgo_shadow_min_carrier_fraction =
                std::stod(argv[++i]);
            continue;
        }
        if (arg == "--multisd-fgo-shadow-min-fixed-ambiguities" &&
            i + 1 < argc) {
            config.multisd_fgo_shadow_min_fixed_ambiguities =
                std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--multisd-fgo-shadow-holdout-satellites" &&
            i + 1 < argc) {
            config.multisd_fgo_shadow_holdout_satellites =
                std::stoi(argv[++i]);
            continue;
        }
        // Phase 2a CMC-aware reference selection flags: kept as STANDALONE
        // ifs (each ending in `continue`) rather than joining the else-if
        // chain below, which already sits at MSVC's C1061 nested-block
        // limit (gnss_solve is clang-only to build anyway, but there is no
        // reason to push the chain any deeper).
        if (arg == "--cmc-ref") {
            config.cmc_aware_reference_selection = true;
            continue;
        }
        if (arg == "--cmc-ref-level" && i + 1 < argc) {
            config.cmc_ref_level_m = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--demote-fixed-status-low-satellite-ceiling" && i + 1 < argc) {
            config.demote_fixed_status_low_satellite_ceiling = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--demote-fixed-status-low-satellite-max-ratio" && i + 1 < argc) {
            config.demote_fixed_status_low_satellite_max_ratio = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--max-fixed-prefit-rms" && i + 1 < argc) {
            config.max_fixed_prefit_residual_rms_m = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--max-fixed-anchor-age" && i + 1 < argc) {
            config.max_fixed_anchor_age_s = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--max-fixed-doppler-consensus" && i + 1 < argc) {
            config.max_fixed_doppler_consensus_m = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--min-fixed-prefit-outliers" && i + 1 < argc) {
            config.min_fixed_prefit_outliers = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--max-fixed-overconfidence-cov-trace" && i + 1 < argc) {
            config.max_fixed_overconfidence_covariance_trace_m2 = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--fixed-prefit-reset-streak" && i + 1 < argc) {
            config.fixed_prefit_reset_streak = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--fixed-prefit-quarantine-only") {
            config.fixed_prefit_quarantine_only = true;
            continue;
        }
        if (arg == "--cmc-ref-switch-epochs" && i + 1 < argc) {
            config.cmc_ref_switch_epochs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--cmc-ref-return-min-elev" && i + 1 < argc) {
            config.cmc_ref_return_min_elev_deg = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--cmc-ref-switch-max-elev-drop" && i + 1 < argc) {
            config.cmc_ref_switch_max_elev_drop_deg = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--cmc-ref-switch-min-elev" && i + 1 < argc) {
            config.cmc_ref_switch_min_elev_deg = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--rtk-adaptive-noise") {
            config.rtk_adaptive_noise = true;
            continue;
        }
        if (arg == "--rtk-adaptive-noise-alpha-phase" && i + 1 < argc) {
            config.rtk_adaptive_noise_alpha_phase = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--rtk-adaptive-noise-alpha-code" && i + 1 < argc) {
            config.rtk_adaptive_noise_alpha_code = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--rtk-adaptive-noise-min-scale" && i + 1 < argc) {
            config.rtk_adaptive_noise_min_scale = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--rtk-adaptive-noise-max-scale" && i + 1 < argc) {
            config.rtk_adaptive_noise_max_scale = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--rtk-adaptive-noise-max-baseline" && i + 1 < argc) {
            config.rtk_adaptive_noise_max_baseline_m = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--cp-pr-fixed-gate") {
            config.cp_pr_fixed_gate = true;
            continue;
        }
        if (arg == "--cp-pr-fixed-gate-threshold" && i + 1 < argc) {
            config.cp_pr_fixed_gate_threshold_m = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--cp-pr-fixed-gate-min-pairs" && i + 1 < argc) {
            config.cp_pr_fixed_gate_min_pairs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--cp-pr-fixed-gate-max-bad-pairs" && i + 1 < argc) {
            config.cp_pr_fixed_gate_max_bad_pairs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--cp-pr-fixed-gate-escalation-epochs" && i + 1 < argc) {
            config.cp_pr_fixed_gate_escalation_epochs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--ddpr-anchor-fde-threshold" && i + 1 < argc) {
            config.ddpr_anchor_fde_threshold_m = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--ddpr-anchor-max-fde-removals" && i + 1 < argc) {
            config.ddpr_anchor_max_fde_removals = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--realtime-fix-integrity") {
            config.enable_realtime_fix_integrity = true;
            continue;
        }
        if (arg == "--integrity-base-gate") {
            config.enable_integrity_base_gate = true;
            continue;
        }
        if (arg == "--no-integrity-base-gate") {
            config.enable_integrity_base_gate = false;
            continue;
        }
        if (arg == "--integrity-shadow-csv" && i + 1 < argc) {
            config.integrity_shadow_csv_path = argv[++i];
            config.enable_realtime_fix_integrity = true;
            continue;
        }
        if (arg == "--integrity-shadow-match-tolerance" && i + 1 < argc) {
            config.integrity_shadow_match_tolerance_s = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--integrity-shadow-max-age" && i + 1 < argc) {
            config.integrity_shadow_max_age_s = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--integrity-shadow-max-gdop" && i + 1 < argc) {
            config.integrity_shadow_max_gdop = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--integrity-shadow-max-ddpr-rms" && i + 1 < argc) {
            config.integrity_shadow_max_ddpr_rms_m = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--integrity-shadow-min-satellites" && i + 1 < argc) {
            config.integrity_shadow_min_satellites = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--integrity-shadow-max-covariance-trace" && i + 1 < argc) {
            config.integrity_shadow_max_covariance_trace_m2 = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--integrity-shadow-default-covariance-trace" && i + 1 < argc) {
            config.integrity_shadow_default_covariance_trace_m2 = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--integrity-shadow-assume-default-covariance-trace") {
            config.integrity_shadow_assume_default_covariance_trace = true;
            continue;
        }
        if (arg == "--integrity-shadow-require-fixed-status") {
            config.integrity_shadow_require_fixed_status = true;
            continue;
        }
        if (arg == "--integrity-shadow-allow-float-status") {
            config.integrity_shadow_require_fixed_status = false;
            continue;
        }
        if (arg == "--integrity-log" && i + 1 < argc) {
            config.integrity_log_path = argv[++i];
            config.enable_realtime_fix_integrity = true;
            continue;
        }
        if (arg == "--low-ratio-guard-threshold" && i + 1 < argc) {
            config.low_ratio_guard_threshold = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--low-ratio-min-fixed-ambiguities" && i + 1 < argc) {
            config.low_ratio_min_fixed_ambiguities = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--low-count-rescue-ratio" && i + 1 < argc) {
            config.low_count_rescue_ratio_threshold = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--low-count-rescue-min-fixed" && i + 1 < argc) {
            config.low_count_rescue_min_fixed_ambiguities = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--low-count-rescue-max-history-speed" && i + 1 < argc) {
            config.low_count_rescue_max_history_speed_mps = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--float-bridge-tail-guard") {
            config.enable_float_bridge_tail_guard = true;
            continue;
        }
        if (arg == "--no-float-bridge-tail-guard") {
            config.enable_float_bridge_tail_guard = false;
            continue;
        }
        if (arg == "--float-bridge-tail-max-anchor-gap" && i + 1 < argc) {
            config.float_bridge_tail_guard_max_anchor_gap_s = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--float-bridge-tail-min-anchor-speed" && i + 1 < argc) {
            config.float_bridge_tail_guard_min_anchor_speed_mps = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--float-bridge-tail-max-anchor-speed" && i + 1 < argc) {
            config.float_bridge_tail_guard_max_anchor_speed_mps = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--float-bridge-tail-max-residual" && i + 1 < argc) {
            config.float_bridge_tail_guard_max_residual_m = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--float-bridge-tail-min-segment-epochs" && i + 1 < argc) {
            config.float_bridge_tail_guard_min_segment_epochs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--max-baseline-m" && i + 1 < argc) {
            config.max_baseline_length_m = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--base-ecef" && i + 3 < argc) {
            config.base_position_ecef = Eigen::Vector3d(
                std::stod(argv[++i]), std::stod(argv[++i]), std::stod(argv[++i]));
            config.base_position_override = true;
            continue;
        }
        if (arg == "--skip-epochs" && i + 1 < argc) {
            config.skip_epochs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--max-epochs" && i + 1 < argc) {
            config.max_epochs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--debug-epoch-log" && i + 1 < argc) {
            config.debug_epoch_log_path = argv[++i];
            continue;
        }
        // Keep these late-chain options standalone as well. Besides making
        // the parser easier to extend, this keeps MSVC below C1061's nested
        // else-if limit.
        if (arg == "--fixed-bridge-burst-guard") {
            config.enable_fixed_bridge_burst_guard = true;
            continue;
        }
        if (arg == "--no-fixed-bridge-burst-guard") {
            config.enable_fixed_bridge_burst_guard = false;
            continue;
        }
        if (arg == "--fixed-bridge-burst-max-anchor-gap" && i + 1 < argc) {
            config.fixed_bridge_burst_guard_max_anchor_gap_s = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--fixed-bridge-burst-min-boundary-gap" && i + 1 < argc) {
            config.fixed_bridge_burst_guard_min_boundary_gap_s = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--fixed-bridge-burst-max-residual" && i + 1 < argc) {
            config.fixed_bridge_burst_guard_max_residual_m = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--fixed-bridge-burst-max-segment-epochs" && i + 1 < argc) {
            config.fixed_bridge_burst_guard_max_segment_epochs = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--data-dir" && i + 1 < argc) {
            config.data_dir = argv[++i];
            continue;
        }
        if (arg == "--rover" && i + 1 < argc) {
            config.rover_obs_path = argv[++i];
            continue;
        }
        if (arg == "--base" && i + 1 < argc) {
            config.base_obs_path = argv[++i];
            continue;
        }
        if (arg == "--nav" && i + 1 < argc) {
            config.nav_path = argv[++i];
            continue;
        }
        if (arg == "--out" && i + 1 < argc) {
            config.output_pos_path = argv[++i];
            continue;
        }
        if (arg == "--kml" && i + 1 < argc) {
            config.output_kml_path = argv[++i];
            config.write_kml = true;
            continue;
        }
        if (arg == "--no-kml") {
            config.write_kml = false;
            continue;
        }
        if (arg == "--format" && i + 1 < argc) {
            config.output_format = parseOutputFormat(argv[++i], argv[0]);
            continue;
        }
        if (arg == "--mode" && i + 1 < argc) {
            config.mode = parseModeChoice(argv[++i], argv[0]);
            continue;
        }
        if (arg == "--iono" && i + 1 < argc) {
            config.iono = parseIonoChoice(argv[++i], argv[0]);
            continue;
        }
        if (arg == "--ratio" && i + 1 < argc) {
            const std::string ratio_value = argv[++i];
            config.enable_satellite_count_ratio_threshold = ratio_value == "sat-count";
            config.ratio_threshold = config.enable_satellite_count_ratio_threshold
                ? 3.0
                : std::stod(ratio_value);
            config.ratio_threshold_set = true;
            continue;
        }
        if (arg == "--preset" && i + 1 < argc) {
            config.preset = parseRTKTuningPreset(argv[++i], argv[0]);
            continue;
        }
        if (arg == "--arfilter") {
            config.enable_ar_filter = true;
            config.has_ar_filter_override = true;
            continue;
        }
        if (arg == "--no-arfilter") {
            config.enable_ar_filter = false;
            config.has_ar_filter_override = true;
            continue;
        }
        if (arg == "--arfilter-margin" && i + 1 < argc) {
            config.ar_filter_margin = std::stod(argv[++i]);
            config.ar_filter_margin_set = true;
            continue;
        }
        if (arg == "--min-ar-sats" && i + 1 < argc) {
            config.min_satellites_for_ar = std::stoi(argv[++i]);
            config.min_satellites_for_ar_set = true;
            continue;
        }
        if (arg == "--min-subset-ar-pairs" && i + 1 < argc) {
            config.min_subset_pairs_for_ar = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--max-subset-ar-drop-steps" && i + 1 < argc) {
            config.max_subset_drop_steps_for_ar = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "--min-subset-ar-sats" && i + 1 < argc) {
            config.min_subset_sats_for_ar = std::stoi(argv[++i]);
            config.min_subset_sats_for_ar_set = true;
            continue;
        }
        if (arg == "--min-subset-ar-systems" && i + 1 < argc) {
            config.min_subset_systems_for_ar = std::stoi(argv[++i]);
            config.min_subset_systems_for_ar_set = true;
            continue;
        }
        if (arg == "--min-subset-ar-freqs" && i + 1 < argc) {
            config.min_subset_frequencies_for_ar = std::stoi(argv[++i]);
            config.min_subset_frequencies_for_ar_set = true;
            continue;
        }
        if (arg == "--min-subset-ar-dual-freq-sats" && i + 1 < argc) {
            config.min_subset_dual_frequency_sats_for_ar = std::stoi(argv[++i]);
            config.min_subset_dual_frequency_sats_for_ar_set = true;
            continue;
        }
        if (arg == "--min-full-ratio-for-subset-ar" && i + 1 < argc) {
            config.min_full_ratio_for_subset_ar = std::stod(argv[++i]);
            config.min_full_ratio_for_subset_ar_set = true;
            continue;
        }
        if (arg == "--min-hold-count" && i + 1 < argc) {
            config.min_hold_count = std::stoi(argv[++i]);
            config.min_hold_count_set = true;
            continue;
        }
        if (arg == "--hold-ratio-threshold" && i + 1 < argc) {
            config.hold_ratio_threshold = std::stod(argv[++i]);
            config.hold_ratio_threshold_set = true;
            continue;
        }
        if (arg == "--elevation-mask-deg" && i + 1 < argc) {
            config.elevation_mask_deg = std::stod(argv[++i]);
            continue;
        }
        if (arg == "--process-noise-position" && i + 1 < argc) {
            config.process_noise_position = std::stod(argv[++i]);
            config.process_noise_position_set = true;
        } else if (arg == "--process-noise-ambiguity" && i + 1 < argc) {
            config.process_noise_ambiguity = std::stod(argv[++i]);
            config.process_noise_ambiguity_set = true;
        } else if (arg == "--process-noise-iono" && i + 1 < argc) {
            config.process_noise_iono = std::stod(argv[++i]);
            config.process_noise_iono_set = true;
        } else if (arg == "--carrier-phase-sigma" && i + 1 < argc) {
            config.carrier_phase_sigma = std::stod(argv[++i]);
            config.carrier_phase_sigma_set = true;
        } else if (arg == "--rtk-snr-weighting") {
            config.enable_snr_weighting = true;
        } else if (arg == "--rtk-snr-reference-dbhz" && i + 1 < argc) {
            config.snr_reference_dbhz = std::stod(argv[++i]);
        } else if (arg == "--rtk-snr-max-variance-scale" && i + 1 < argc) {
            config.snr_max_variance_scale = std::stod(argv[++i]);
        } else if (arg == "--rtk-snr-min-baseline" && i + 1 < argc) {
            config.snr_min_baseline_m = std::stod(argv[++i]);
        } else if (arg == "--cycle-slip-threshold" && i + 1 < argc) {
            config.cycle_slip_threshold = std::stod(argv[++i]);
        } else if (arg == "--doppler-slip-threshold" && i + 1 < argc) {
            config.doppler_slip_threshold = std::stod(argv[++i]);
        } else if (arg == "--code-slip-threshold" && i + 1 < argc) {
            config.code_slip_threshold = std::stod(argv[++i]);
        } else if (arg == "--strict-dynamic-slip-thresholds") {
            config.use_dynamic_slip_threshold_floor = false;
        } else if (arg == "--adaptive-dynamic-slip-thresholds") {
            config.enable_adaptive_dynamic_slip_thresholds = true;
        } else if (arg == "--adaptive-dynamic-slip-nonfix-count" && i + 1 < argc) {
            config.adaptive_dynamic_slip_nonfix_count = std::stoi(argv[++i]);
        } else if (arg == "--adaptive-dynamic-slip-hold-epochs" && i + 1 < argc) {
            config.adaptive_dynamic_slip_hold_epochs = std::stoi(argv[++i]);
        } else if (arg == "--no-glonass") {
            config.enable_glonass = false;
        } else if (arg == "--no-beidou") {
            config.enable_beidou = false;
        } else if (arg == "--enable-l5") {
            // Phase 18 Step 3: opt-in L5 measurement collection. Default off — when off,
            // L5 obs are still placed in the L2 slot (legacy fallback for L5-only receivers).
            // When on, L5-class obs are collected into a dedicated SatelliteData.l5_* slot
            // and the L2 slot only carries true L2 signals. Step 4+ will add filter
            // measurement updates / cycle slip handling for L5.
            config.enable_l5 = true;
        } else if (arg == "--glonass-ar" && i + 1 < argc) {
            config.glonass_ar = parseGlonassARChoice(argv[++i], argv[0]);
        } else if (arg == "--glonass-icb-l1" && i + 1 < argc) {
            config.glonass_icb_l1_m_per_mhz = std::stod(argv[++i]);
        } else if (arg == "--glonass-icb-l2" && i + 1 < argc) {
            config.glonass_icb_l2_m_per_mhz = std::stod(argv[++i]);
        } else if (arg == "--ar-policy" && i + 1 < argc) {
            const std::string policy_str = argv[++i];
            if (policy_str == "extended") {
                config.ar_policy = libgnss::RTKProcessor::RTKConfig::ARPolicy::EXTENDED;
            } else if (policy_str == "demo5-continuous") {
                config.ar_policy = libgnss::RTKProcessor::RTKConfig::ARPolicy::DEMO5_CONTINUOUS;
            } else {
                argumentError("unsupported --ar-policy value: " + policy_str, argv[0]);
            }
        } else if (arg == "--max-hold-div" && i + 1 < argc) {
            config.max_hold_divergence_m = std::stod(argv[++i]);
        } else if (arg == "--max-pos-jump" && i + 1 < argc) {
            config.max_position_jump_m = std::stod(argv[++i]);
        } else if (arg == "--max-pos-jump-min" && i + 1 < argc) {
            config.max_position_jump_min_m = std::stod(argv[++i]);
            config.max_position_jump_min_m_set = true;
        } else if (arg == "--max-pos-jump-rate" && i + 1 < argc) {
            config.max_position_jump_rate_mps = std::stod(argv[++i]);
            config.max_position_jump_rate_mps_set = true;
        } else if (arg == "--max-float-spp-div" && i + 1 < argc) {
            config.max_float_spp_divergence_m = std::stod(argv[++i]);
        } else if (arg == "--max-float-prefit-rms" && i + 1 < argc) {
            config.max_float_prefit_residual_rms_m = std::stod(argv[++i]);
            config.max_float_prefit_residual_rms_m_set = true;
        } else if (arg == "--max-float-prefit-max" && i + 1 < argc) {
            config.max_float_prefit_residual_max_m = std::stod(argv[++i]);
            config.max_float_prefit_residual_max_m_set = true;
        } else if (arg == "--max-float-prefit-reset-streak" && i + 1 < argc) {
            config.max_float_prefit_residual_reset_streak = std::stoi(argv[++i]);
            config.max_float_prefit_residual_reset_streak_set = true;
        } else if (arg == "--min-float-prefit-trusted-jump" && i + 1 < argc) {
            config.min_float_prefit_residual_trusted_jump_m = std::stod(argv[++i]);
        } else if (arg == "--max-update-nis-per-obs" && i + 1 < argc) {
            config.max_update_nis_per_observation = std::stod(argv[++i]);
        } else if (arg == "--max-fixed-update-nis-per-obs" && i + 1 < argc) {
            config.max_fixed_update_nis_per_observation = std::stod(argv[++i]);
        } else if (arg == "--max-fixed-update-post-rms" && i + 1 < argc) {
            config.max_fixed_update_post_residual_rms_m = std::stod(argv[++i]);
        } else if (arg == "--max-fixed-update-gate-ratio" && i + 1 < argc) {
            config.max_fixed_update_gate_ratio = std::stod(argv[++i]);
        } else if (arg == "--min-fixed-update-gate-baseline" && i + 1 < argc) {
            config.min_fixed_update_gate_baseline_m = std::stod(argv[++i]);
        } else if (arg == "--max-fixed-update-gate-baseline" && i + 1 < argc) {
            config.max_fixed_update_gate_baseline_m = std::stod(argv[++i]);
        } else if (arg == "--min-fixed-update-gate-speed" && i + 1 < argc) {
            config.min_fixed_update_gate_speed_mps = std::stod(argv[++i]);
        } else if (arg == "--max-fixed-update-gate-speed" && i + 1 < argc) {
            config.max_fixed_update_gate_speed_mps = std::stod(argv[++i]);
        } else if (arg == "--max-fixed-update-secondary-gate-ratio" && i + 1 < argc) {
            config.max_fixed_update_secondary_gate_ratio = std::stod(argv[++i]);
        } else if (arg == "--min-fixed-update-secondary-gate-baseline" && i + 1 < argc) {
            config.min_fixed_update_secondary_gate_baseline_m = std::stod(argv[++i]);
        } else if (arg == "--max-fixed-update-secondary-gate-baseline" && i + 1 < argc) {
            config.max_fixed_update_secondary_gate_baseline_m = std::stod(argv[++i]);
        } else if (arg == "--min-fixed-update-secondary-gate-speed" && i + 1 < argc) {
            config.min_fixed_update_secondary_gate_speed_mps = std::stod(argv[++i]);
        } else if (arg == "--max-fixed-update-secondary-gate-speed" && i + 1 < argc) {
            config.max_fixed_update_secondary_gate_speed_mps = std::stod(argv[++i]);
        } else if (arg == "--demote-fixed-status-nis-per-obs" && i + 1 < argc) {
            config.demote_fixed_status_nis_per_observation = std::stod(argv[++i]);
        } else if (arg == "--demote-fixed-status-post-rms" && i + 1 < argc) {
            config.demote_fixed_status_post_residual_rms_m = std::stod(argv[++i]);
        } else if (arg == "--demote-fixed-status-max-ratio" && i + 1 < argc) {
            config.demote_fixed_status_max_ratio = std::stod(argv[++i]);
        } else if (arg == "--demote-fixed-status-gate-ratio" && i + 1 < argc) {
            config.demote_fixed_status_gate_ratio = std::stod(argv[++i]);
        } else if (arg == "--demote-fixed-status-min-satellites" && i + 1 < argc) {
            config.demote_fixed_status_min_satellites = std::stoi(argv[++i]);
        } else if (arg == "--min-demote-fixed-status-baseline" && i + 1 < argc) {
            config.min_demote_fixed_status_baseline_m = std::stod(argv[++i]);
        } else if (arg == "--max-demote-fixed-status-baseline" && i + 1 < argc) {
            config.max_demote_fixed_status_baseline_m = std::stod(argv[++i]);
        } else if (arg == "--max-postfix-rms" && i + 1 < argc) {
            config.max_postfix_residual_rms = std::stod(argv[++i]);
        } else if (arg == "--enable-wide-lane-ar") {
            config.enable_wide_lane_ar = true;
            config.wide_lane_ar_set = true;
        } else if (arg == "--no-wide-lane-ar") {
            config.enable_wide_lane_ar = false;
            config.wide_lane_ar_set = true;
        } else if (arg == "--wide-lane-threshold" && i + 1 < argc) {
            config.wide_lane_acceptance_threshold = std::stod(argv[++i]);
            config.wide_lane_acceptance_threshold_set = true;
        } else if (arg == "--enable-wlnl-fallback") {
            config.enable_wlnl_fallback = true;
        } else if (arg == "--enable-bsr-decimation") {
            config.enable_bsr_guided_decimation = true;
        } else if (arg == "--bsr-worst-axes" && i + 1 < argc) {
            config.bsr_guided_worst_axes = std::stoi(argv[++i]);
        } else if (arg == "--bsr-max-drops" && i + 1 < argc) {
            config.bsr_guided_max_drop_steps = std::stoi(argv[++i]);
        } else if (arg == "--nlos-weights" && i + 1 < argc) {
            config.nlos_weights_csv_path = argv[++i];
        } else if (arg == "--nlos-weight-mode" && i + 1 < argc) {
            config.nlos_weight_mode = parseNlosWeightMode(argv[++i], argv[0]);
        } else if (arg == "--nlos-two-tier-threshold" && i + 1 < argc) {
            config.nlos_two_tier_los_threshold = std::stod(argv[++i]);
        } else if (arg == "--nlos-two-tier-inflation" && i + 1 < argc) {
            config.nlos_two_tier_sigma_inflation = std::stod(argv[++i]);
        } else if (arg == "--nlos-continuous-floor" && i + 1 < argc) {
            config.nlos_continuous_los_prob_floor = std::stod(argv[++i]);
        } else if (arg == "--nlos-tow-tolerance" && i + 1 < argc) {
            config.nlos_tow_tolerance_s = std::stod(argv[++i]);
        } else if (arg == "--nlos-exclude-threshold" && i + 1 < argc) {
            config.nlos_exclude_threshold = std::stod(argv[++i]);
        } else if (arg == "--nlos-min-sats" && i + 1 < argc) {
            config.nlos_min_sats = std::stoi(argv[++i]);
        } else if (arg == "--float-trust-policy" && i + 1 < argc) {
            config.float_trust_policy = parseFloatTrustPolicy(argv[++i], argv[0]);
        } else if (arg == "--trust-lapse-qpos" && i + 1 < argc) {
            config.trust_lapse_qpos_m2_per_s = std::stod(argv[++i]);
        } else if (arg == "--trust-lapse-gate-s" && i + 1 < argc) {
            config.trust_lapse_gate_s = std::stod(argv[++i]);
        } else if (arg == "--trust-lapse-gate-nlos-frac" && i + 1 < argc) {
            config.trust_lapse_gate_nlos_frac = std::stod(argv[++i]);
        } else if (arg == "--trust-gate-nlos-relax") {
            config.trust_gate_nlos_relax = true;
        } else if (arg == "--nlos-min-los-sats" && i + 1 < argc) {
            config.nlos_min_los_sats = std::stoi(argv[++i]);
        } else if (arg == "--max-consec-float-reset" && i + 1 < argc) {
            config.max_consecutive_float_for_reset = std::stoi(argv[++i]);
        } else if (arg == "--max-consec-nonfix-reset" && i + 1 < argc) {
            config.max_consecutive_nonfix_for_reset = std::stoi(argv[++i]);
        } else if (arg == "--no-nonfix-drift-guard") {
            config.enable_nonfix_drift_guard = false;
        } else if (arg == "--nonfix-drift-max-anchor-gap" && i + 1 < argc) {
            config.nonfix_drift_guard_max_anchor_gap_s = std::stod(argv[++i]);
        } else if (arg == "--nonfix-drift-max-anchor-speed" && i + 1 < argc) {
            config.nonfix_drift_guard_max_anchor_speed_mps = std::stod(argv[++i]);
        } else if (arg == "--nonfix-drift-max-residual" && i + 1 < argc) {
            config.nonfix_drift_guard_max_residual_m = std::stod(argv[++i]);
        } else if (arg == "--nonfix-drift-min-horizontal-residual" && i + 1 < argc) {
            config.nonfix_drift_guard_min_horizontal_residual_m = std::stod(argv[++i]);
        } else if (arg == "--nonfix-drift-min-segment-epochs" && i + 1 < argc) {
            config.nonfix_drift_guard_min_segment_epochs = std::stoi(argv[++i]);
        } else if (arg == "--nonfix-drift-max-segment-epochs" && i + 1 < argc) {
            config.nonfix_drift_guard_max_segment_epochs = std::stoi(argv[++i]);
        } else if (arg == "--no-spp-height-step-guard") {
            config.enable_spp_height_step_guard = false;
        } else if (arg == "--spp-height-step-min" && i + 1 < argc) {
            config.spp_height_step_guard_min_m = std::stod(argv[++i]);
        } else if (arg == "--spp-height-step-rate" && i + 1 < argc) {
            config.spp_height_step_guard_max_rate_mps = std::stod(argv[++i]);
        } else if (arg == "--lambda-shadow-candidates" && i + 1 < argc) {
            config.lambda_candidate_shadow_count = std::stoi(argv[++i]);
        } else if (arg == "--lambda-src-par-shadow-success-rate" &&
                   i + 1 < argc) {
            config.lambda_src_par_shadow_success_rate =
                std::stod(argv[++i]);
        } else if (arg == "--lambda-src-par-shadow-covariance-scale" &&
                   i + 1 < argc) {
            config.lambda_src_par_shadow_covariance_scale =
                std::stod(argv[++i]);
        } else if (arg == "--lambda-satellite-par-shadow-max-drops" &&
                   i + 1 < argc) {
            config.lambda_satellite_par_shadow_max_drop_steps =
                std::stoi(argv[++i]);
        } else if (
            arg == "--lambda-satellite-par-shadow-covariance-scale" &&
            i + 1 < argc) {
            config.lambda_satellite_par_shadow_covariance_scale =
                std::stod(argv[++i]);
        } else if (
            arg ==
            "--lambda-satellite-par-shadow-quality-diverse") {
            config
                .enable_lambda_satellite_par_shadow_quality_diverse =
                true;
        } else if (arg == "--lambda-l1-l5-wlnl-shadow") {
            config.enable_lambda_l1_l5_wlnl_shadow = true;
        } else if (arg == "--lambda-l1-l5-wlnl-causal-arcs") {
            config.enable_lambda_l1_l5_wlnl_causal_arcs = true;
        } else if (arg == "--safe-fix-shadow-state-machine") {
            config.enable_safe_fix_shadow_state_machine = true;
        } else if (arg == "--safe-fix-robust-consensus-shadow") {
            config.enable_safe_fix_shadow_state_machine = true;
            config.enable_safe_fix_robust_consensus_shadow = true;
            config.lambda_candidate_shadow_count =
                std::max(config.lambda_candidate_shadow_count, 2);
            config.lambda_satellite_par_shadow_max_drop_steps =
                std::max(
                    config.lambda_satellite_par_shadow_max_drop_steps,
                    8);
        } else if (
            arg == "--safe-fix-independent-failure-budget") {
            config.require_safe_fix_independent_failure_budget =
                true;
        } else if (arg == "--library-fixed-quality-gate") {
            config.enable_library_fixed_quality_gate = true;
        } else if (
            arg == "--library-fixed-quality-max-cov-trace" &&
            i + 1 < argc) {
            config.library_fixed_quality_max_covariance_trace_m2 =
                std::stod(argv[++i]);
        } else if (
            arg == "--library-fixed-quality-max-nis" &&
            i + 1 < argc) {
            config.library_fixed_quality_max_nis_per_observation =
                std::stod(argv[++i]);
        } else if (
            arg == "--library-fixed-quality-min-observations" &&
            i + 1 < argc) {
            config.library_fixed_quality_min_strong_observations =
                std::stoi(argv[++i]);
        } else if (
            arg == "--library-fixed-quality-strong-max-nis" &&
            i + 1 < argc) {
            config
                .library_fixed_quality_strong_max_nis_per_observation =
                std::stod(argv[++i]);
        } else if (arg == "--safe-float-continuity") {
            config.enable_safe_float_continuity = true;
        } else if (
            arg == "--safe-float-continuity-max-age" &&
            i + 1 < argc) {
            config.safe_float_continuity_max_age_s =
                std::stod(argv[++i]);
        } else if (arg == "--safe-availability-fallback") {
            config.enable_safe_availability_fallback = true;
        } else if (arg == "--prefer-trusted-seed") {
            config.prefer_trusted_seed = true;
        } else if (arg == "--doppler-float-seed") {
            config.use_doppler_float_seed = true;
        } else if (arg == "--doppler-float-seed-max-age" && i + 1 < argc) {
            config.doppler_float_seed_max_age_s = std::stod(argv[++i]);
        } else if (arg == "--rover-seed-pos" && i + 1 < argc) {
            config.rover_seed_pos_path = argv[++i];
        } else if (arg == "--diagnostics-csv" && i + 1 < argc) {
            config.diagnostics_csv_path = argv[++i];
        } else if (arg == "--rtk-update-outlier-threshold" && i + 1 < argc) {
            config.rtk_update_outlier_threshold = std::stod(argv[++i]);
        } else if (arg == "--student-t-rtk-front-end") {
            config.student_t_rtk_front_end = true;
        } else if (arg == "--no-kinematic-post-filter") {
            config.enable_kinematic_post_filter = false;
        } else if (arg == "--no-base-interp") {
            config.enable_base_interpolation = false;
        } else if (arg == "--verbose") {
            config.verbose = true;
        } else {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }

    if (!config.data_dir.empty()) {
        if (config.rover_obs_path.empty()) config.rover_obs_path = config.data_dir + "/rover.obs";
        if (config.base_obs_path.empty()) config.base_obs_path = config.data_dir + "/base.obs";
        if (config.nav_path.empty()) config.nav_path = config.data_dir + "/navigation.nav";
    }

    applyRTKTuningPreset(config);

    if (config.rover_obs_path.empty() || config.base_obs_path.empty() || config.nav_path.empty()) {
        argumentError("provide --data-dir or all of --rover, --base, and --nav", argv[0]);
    }
    if (config.min_satellites_for_ar < 4) {
        argumentError("--min-ar-sats must be >= 4", argv[0]);
    }
    if (config.min_subset_pairs_for_ar < 4) {
        argumentError("--min-subset-ar-pairs must be >= 4", argv[0]);
    }
    if (config.max_subset_drop_steps_for_ar < 0) {
        argumentError("--max-subset-ar-drop-steps must be >= 0", argv[0]);
    }
    if (config.min_subset_sats_for_ar < 0) {
        argumentError("--min-subset-ar-sats must be >= 0", argv[0]);
    }
    if (config.min_subset_systems_for_ar < 0) {
        argumentError("--min-subset-ar-systems must be >= 0", argv[0]);
    }
    if (config.min_subset_frequencies_for_ar < 0) {
        argumentError("--min-subset-ar-freqs must be >= 0", argv[0]);
    }
    if (config.min_subset_dual_frequency_sats_for_ar < 0) {
        argumentError("--min-subset-ar-dual-freq-sats must be >= 0", argv[0]);
    }
    if (config.min_full_ratio_for_subset_ar < 0.0) {
        argumentError("--min-full-ratio-for-subset-ar must be >= 0", argv[0]);
    }
    if (config.ar_filter_margin < 0.0) {
        argumentError("--arfilter-margin must be >= 0", argv[0]);
    }
    if (config.nlos_weight_mode != libgnss::nlos_weights::NlosWeightMode::OFF &&
        config.nlos_weights_csv_path.empty()) {
        argumentError("--nlos-weight-mode requires --nlos-weights <csv>", argv[0]);
    }
    if (config.nlos_two_tier_los_threshold < 0.0 || config.nlos_two_tier_los_threshold > 1.0) {
        argumentError("--nlos-two-tier-threshold must be in [0, 1]", argv[0]);
    }
    if (config.nlos_two_tier_sigma_inflation < 1.0) {
        argumentError("--nlos-two-tier-inflation must be >= 1", argv[0]);
    }
    if (config.nlos_continuous_los_prob_floor <= 0.0 || config.nlos_continuous_los_prob_floor > 1.0) {
        argumentError("--nlos-continuous-floor must be in (0, 1]", argv[0]);
    }
    if (config.nlos_tow_tolerance_s < 0.0) {
        argumentError("--nlos-tow-tolerance must be >= 0", argv[0]);
    }
    if (config.nlos_exclude_threshold < 0.0 || config.nlos_exclude_threshold > 1.0) {
        argumentError("--nlos-exclude-threshold must be in [0, 1]", argv[0]);
    }
    if (config.nlos_min_sats < 0) {
        argumentError("--nlos-min-sats must be >= 0", argv[0]);
    }
    if (config.trust_lapse_qpos_m2_per_s < 0.0) {
        argumentError("--trust-lapse-qpos must be >= 0", argv[0]);
    }
    if (config.trust_lapse_gate_s < 0.0) {
        argumentError("--trust-lapse-gate-s must be >= 0", argv[0]);
    }
    if (config.trust_lapse_gate_nlos_frac > 1.0) {
        argumentError("--trust-lapse-gate-nlos-frac must be <= 1 (or negative to disable)", argv[0]);
    }
    if (config.nlos_min_los_sats < 0) {
        argumentError("--nlos-min-los-sats must be >= 0", argv[0]);
    }
    if (config.cmc_ref_switch_epochs < 1) {
        argumentError("--cmc-ref-switch-epochs must be >= 1", argv[0]);
    }
    if (config.cmc_ref_return_min_elev_deg < 0.0) {
        argumentError("--cmc-ref-return-min-elev must be >= 0", argv[0]);
    }
    if (config.cmc_ref_switch_max_elev_drop_deg < 0.0) {
        argumentError("--cmc-ref-switch-max-elev-drop must be >= 0", argv[0]);
    }
    if (config.cmc_ref_switch_min_elev_deg < 0.0) {
        argumentError("--cmc-ref-switch-min-elev must be >= 0", argv[0]);
    }
    if (config.rtk_adaptive_noise_alpha_phase < 0.0 || config.rtk_adaptive_noise_alpha_phase > 1.0) {
        argumentError("--rtk-adaptive-noise-alpha-phase must be in [0, 1]", argv[0]);
    }
    if (config.rtk_adaptive_noise_alpha_code < 0.0 || config.rtk_adaptive_noise_alpha_code > 1.0) {
        argumentError("--rtk-adaptive-noise-alpha-code must be in [0, 1]", argv[0]);
    }
    if (config.rtk_adaptive_noise_min_scale <= 0.0) {
        argumentError("--rtk-adaptive-noise-min-scale must be > 0", argv[0]);
    }
    if (config.rtk_adaptive_noise_max_scale < config.rtk_adaptive_noise_min_scale) {
        argumentError("--rtk-adaptive-noise-max-scale must be >= min scale", argv[0]);
    }
    if (config.cp_pr_fixed_gate_threshold_m <= 0.0) {
        argumentError("--cp-pr-fixed-gate-threshold must be > 0", argv[0]);
    }
    if (config.cp_pr_fixed_gate_min_pairs < 1) {
        argumentError("--cp-pr-fixed-gate-min-pairs must be >= 1", argv[0]);
    }
    if (config.cp_pr_fixed_gate_max_bad_pairs < 0) {
        argumentError("--cp-pr-fixed-gate-max-bad-pairs must be >= 0", argv[0]);
    }
    if (config.cp_pr_fixed_gate_escalation_epochs < 1) {
        argumentError("--cp-pr-fixed-gate-escalation-epochs must be >= 1", argv[0]);
    }
    if (config.ddpr_anchor_fde_threshold_m <= 0.0) {
        argumentError("--ddpr-anchor-fde-threshold must be > 0", argv[0]);
    }
    if (config.ddpr_anchor_max_fde_removals < 0) {
        argumentError("--ddpr-anchor-max-fde-removals must be >= 0", argv[0]);
    }
    if (config.low_ratio_guard_threshold < 0.0) {
        argumentError("--low-ratio-guard-threshold must be >= 0", argv[0]);
    }
    if (config.low_ratio_min_fixed_ambiguities < 0) {
        argumentError("--low-ratio-min-fixed-ambiguities must be >= 0", argv[0]);
    }
    if (config.low_count_rescue_ratio_threshold < 0.0) {
        argumentError("--low-count-rescue-ratio must be >= 0", argv[0]);
    }
    if (config.low_count_rescue_min_fixed_ambiguities < 1) {
        argumentError("--low-count-rescue-min-fixed must be >= 1", argv[0]);
    }
    if (config.low_count_rescue_max_history_speed_mps < 0.0) {
        argumentError("--low-count-rescue-max-history-speed must be >= 0", argv[0]);
    }
    if (config.min_hold_count < 0) {
        argumentError("--min-hold-count must be >= 0", argv[0]);
    }
    if (config.hold_ratio_threshold <= 0.0) {
        argumentError("--hold-ratio-threshold must be > 0", argv[0]);
    }
    if (config.elevation_mask_deg < 0.0 || config.elevation_mask_deg >= 90.0) {
        argumentError("--elevation-mask-deg must be in [0, 90)", argv[0]);
    }
    if (config.snr_reference_dbhz <= 0.0) {
        argumentError("--rtk-snr-reference-dbhz must be > 0", argv[0]);
    }
    if (config.snr_max_variance_scale < 1.0) {
        argumentError("--rtk-snr-max-variance-scale must be >= 1", argv[0]);
    }
    if (config.snr_min_baseline_m < 0.0) {
        argumentError("--rtk-snr-min-baseline must be >= 0", argv[0]);
    }
    if (config.cycle_slip_threshold <= 0.0) {
        argumentError("--cycle-slip-threshold must be > 0", argv[0]);
    }
    if (config.doppler_slip_threshold <= 0.0) {
        argumentError("--doppler-slip-threshold must be > 0", argv[0]);
    }
    if (config.code_slip_threshold <= 0.0) {
        argumentError("--code-slip-threshold must be > 0", argv[0]);
    }
    if (config.adaptive_dynamic_slip_nonfix_count < 1) {
        argumentError("--adaptive-dynamic-slip-nonfix-count must be >= 1", argv[0]);
    }
    if (config.adaptive_dynamic_slip_hold_epochs < 0) {
        argumentError("--adaptive-dynamic-slip-hold-epochs must be >= 0", argv[0]);
    }
    if (config.max_baseline_length_m <= 0.0) {
        argumentError("--max-baseline-m must be > 0", argv[0]);
    }
    if (config.max_position_jump_m < 0.0) {
        argumentError("--max-pos-jump must be >= 0", argv[0]);
    }
    if (config.max_fixed_anchor_age_s < 0.0) {
        argumentError("--max-fixed-anchor-age must be >= 0", argv[0]);
    }
    if (config.max_fixed_doppler_consensus_m < 0.0) {
        argumentError("--max-fixed-doppler-consensus must be >= 0", argv[0]);
    }
    if (config.max_position_jump_min_m < 0.0) {
        argumentError("--max-pos-jump-min must be >= 0", argv[0]);
    }
    if (config.max_position_jump_rate_mps < 0.0) {
        argumentError("--max-pos-jump-rate must be >= 0", argv[0]);
    }
    if (config.max_float_spp_divergence_m < 0.0) {
        argumentError("--max-float-spp-div must be >= 0", argv[0]);
    }
    if (config.max_float_prefit_residual_rms_m < 0.0) {
        argumentError("--max-float-prefit-rms must be >= 0", argv[0]);
    }
    if (config.max_float_prefit_residual_max_m < 0.0) {
        argumentError("--max-float-prefit-max must be >= 0", argv[0]);
    }
    if (config.max_float_prefit_residual_reset_streak < 1) {
        argumentError("--max-float-prefit-reset-streak must be >= 1", argv[0]);
    }
    if (config.min_float_prefit_residual_trusted_jump_m < 0.0) {
        argumentError("--min-float-prefit-trusted-jump must be >= 0", argv[0]);
    }
    if (config.max_update_nis_per_observation < 0.0) {
        argumentError("--max-update-nis-per-obs must be >= 0", argv[0]);
    }
    if (config.max_fixed_update_nis_per_observation < 0.0) {
        argumentError("--max-fixed-update-nis-per-obs must be >= 0", argv[0]);
    }
    if (config.max_fixed_update_post_residual_rms_m < 0.0) {
        argumentError("--max-fixed-update-post-rms must be >= 0", argv[0]);
    }
    if (config.max_fixed_update_gate_ratio < 0.0) {
        argumentError("--max-fixed-update-gate-ratio must be >= 0", argv[0]);
    }
    if (config.min_fixed_update_gate_baseline_m < 0.0) {
        argumentError("--min-fixed-update-gate-baseline must be >= 0", argv[0]);
    }
    if (config.max_fixed_update_gate_baseline_m < 0.0) {
        argumentError("--max-fixed-update-gate-baseline must be >= 0", argv[0]);
    }
    if (config.min_fixed_update_gate_speed_mps < 0.0) {
        argumentError("--min-fixed-update-gate-speed must be >= 0", argv[0]);
    }
    if (config.max_fixed_update_gate_speed_mps < 0.0) {
        argumentError("--max-fixed-update-gate-speed must be >= 0", argv[0]);
    }
    if (config.max_fixed_update_secondary_gate_ratio < 0.0) {
        argumentError("--max-fixed-update-secondary-gate-ratio must be >= 0", argv[0]);
    }
    if (config.min_fixed_update_secondary_gate_baseline_m < 0.0) {
        argumentError("--min-fixed-update-secondary-gate-baseline must be >= 0", argv[0]);
    }
    if (config.max_fixed_update_secondary_gate_baseline_m < 0.0) {
        argumentError("--max-fixed-update-secondary-gate-baseline must be >= 0", argv[0]);
    }
    if (config.min_fixed_update_secondary_gate_speed_mps < 0.0) {
        argumentError("--min-fixed-update-secondary-gate-speed must be >= 0", argv[0]);
    }
    if (config.max_fixed_update_secondary_gate_speed_mps < 0.0) {
        argumentError("--max-fixed-update-secondary-gate-speed must be >= 0", argv[0]);
    }
    if (config.demote_fixed_status_nis_per_observation < 0.0) {
        argumentError("--demote-fixed-status-nis-per-obs must be >= 0", argv[0]);
    }
    if (config.demote_fixed_status_post_residual_rms_m < 0.0) {
        argumentError("--demote-fixed-status-post-rms must be >= 0", argv[0]);
    }
    if (config.demote_fixed_status_max_ratio < 0.0) {
        argumentError("--demote-fixed-status-max-ratio must be >= 0", argv[0]);
    }
    if (config.demote_fixed_status_gate_ratio < 0.0) {
        argumentError("--demote-fixed-status-gate-ratio must be >= 0", argv[0]);
    }
    if (config.max_fixed_prefit_residual_rms_m < 0.0) {
        argumentError("--max-fixed-prefit-rms must be >= 0", argv[0]);
    }
    if (config.min_fixed_prefit_outliers < 0) {
        argumentError("--min-fixed-prefit-outliers must be >= 0", argv[0]);
    }
    if (config.max_fixed_overconfidence_covariance_trace_m2 < 0.0) {
        argumentError("--max-fixed-overconfidence-cov-trace must be >= 0", argv[0]);
    }
    if (config.fixed_prefit_reset_streak < 1) {
        argumentError("--fixed-prefit-reset-streak must be >= 1", argv[0]);
    }
    if ((config.max_fixed_prefit_residual_rms_m > 0.0) !=
        (config.min_fixed_prefit_outliers > 0)) {
        argumentError(
            "--max-fixed-prefit-rms and --min-fixed-prefit-outliers must be used together",
            argv[0]);
    }
    if (config.fixed_prefit_quarantine_only &&
        config.max_fixed_prefit_residual_rms_m <= 0.0) {
        argumentError(
            "--fixed-prefit-quarantine-only requires the fixed-prefit RMS/outlier gate",
            argv[0]);
    }
    if (config.demote_fixed_status_min_satellites < 0) {
        argumentError("--demote-fixed-status-min-satellites must be >= 0", argv[0]);
    }
    if (config.demote_fixed_status_low_satellite_ceiling < 0) {
        argumentError("--demote-fixed-status-low-satellite-ceiling must be >= 0", argv[0]);
    }
    if (config.demote_fixed_status_low_satellite_max_ratio < 0.0) {
        argumentError("--demote-fixed-status-low-satellite-max-ratio must be >= 0", argv[0]);
    }
    if ((config.demote_fixed_status_low_satellite_ceiling > 0) !=
        (config.demote_fixed_status_low_satellite_max_ratio > 0.0)) {
        argumentError(
            "--demote-fixed-status-low-satellite-ceiling and "
            "--demote-fixed-status-low-satellite-max-ratio must be used together",
            argv[0]);
    }
    if (config.min_demote_fixed_status_baseline_m < 0.0) {
        argumentError("--min-demote-fixed-status-baseline must be >= 0", argv[0]);
    }
    if (config.max_demote_fixed_status_baseline_m < 0.0) {
        argumentError("--max-demote-fixed-status-baseline must be >= 0", argv[0]);
    }
    if (config.nonfix_drift_guard_max_anchor_gap_s <= 0.0) {
        argumentError("--nonfix-drift-max-anchor-gap must be > 0", argv[0]);
    }
    if (config.nonfix_drift_guard_max_anchor_speed_mps < 0.0) {
        argumentError("--nonfix-drift-max-anchor-speed must be >= 0", argv[0]);
    }
    if (config.nonfix_drift_guard_max_residual_m <= 0.0) {
        argumentError("--nonfix-drift-max-residual must be > 0", argv[0]);
    }
    if (config.nonfix_drift_guard_min_horizontal_residual_m < 0.0) {
        argumentError("--nonfix-drift-min-horizontal-residual must be >= 0", argv[0]);
    }
    if (config.nonfix_drift_guard_min_segment_epochs < 1) {
        argumentError("--nonfix-drift-min-segment-epochs must be >= 1", argv[0]);
    }
    if (config.nonfix_drift_guard_max_segment_epochs < 0) {
        argumentError("--nonfix-drift-max-segment-epochs must be >= 0", argv[0]);
    }
    if (config.spp_height_step_guard_min_m <= 0.0) {
        argumentError("--spp-height-step-min must be > 0", argv[0]);
    }
    if (config.spp_height_step_guard_max_rate_mps < 0.0) {
        argumentError("--spp-height-step-rate must be >= 0", argv[0]);
    }
    if (config.float_bridge_tail_guard_max_anchor_gap_s <= 0.0) {
        argumentError("--float-bridge-tail-max-anchor-gap must be > 0", argv[0]);
    }
    if (config.float_bridge_tail_guard_min_anchor_speed_mps < 0.0) {
        argumentError("--float-bridge-tail-min-anchor-speed must be >= 0", argv[0]);
    }
    if (config.float_bridge_tail_guard_max_anchor_speed_mps <
        config.float_bridge_tail_guard_min_anchor_speed_mps) {
        argumentError("--float-bridge-tail-max-anchor-speed must be >= min anchor speed", argv[0]);
    }
    if (config.float_bridge_tail_guard_max_residual_m <= 0.0) {
        argumentError("--float-bridge-tail-max-residual must be > 0", argv[0]);
    }
    if (config.float_bridge_tail_guard_min_segment_epochs < 1) {
        argumentError("--float-bridge-tail-min-segment-epochs must be >= 1", argv[0]);
    }
    if (config.fixed_bridge_burst_guard_max_anchor_gap_s <= 0.0) {
        argumentError("--fixed-bridge-burst-max-anchor-gap must be > 0", argv[0]);
    }
    if (config.fixed_bridge_burst_guard_min_boundary_gap_s < 0.0) {
        argumentError("--fixed-bridge-burst-min-boundary-gap must be >= 0", argv[0]);
    }
    if (config.fixed_bridge_burst_guard_max_residual_m <= 0.0) {
        argumentError("--fixed-bridge-burst-max-residual must be > 0", argv[0]);
    }
    if (config.fixed_bridge_burst_guard_max_segment_epochs < 1) {
        argumentError("--fixed-bridge-burst-max-segment-epochs must be >= 1", argv[0]);
    }
    if (config.skip_epochs < 0) {
        argumentError("--skip-epochs must be >= 0", argv[0]);
    }
    if (config.lambda_candidate_shadow_count != 0 &&
        (config.lambda_candidate_shadow_count < 2 ||
         config.lambda_candidate_shadow_count > 32)) {
        argumentError(
            "--lambda-shadow-candidates must be 0 or in [2, 32]",
            argv[0]);
    }
    if (!std::isfinite(config.lambda_src_par_shadow_success_rate) ||
        config.lambda_src_par_shadow_success_rate < 0.0 ||
        config.lambda_src_par_shadow_success_rate > 1.0) {
        argumentError(
            "--lambda-src-par-shadow-success-rate must be in [0, 1]",
            argv[0]);
    }
    if (!std::isfinite(config.lambda_src_par_shadow_covariance_scale) ||
        config.lambda_src_par_shadow_covariance_scale <= 0.0) {
        argumentError(
            "--lambda-src-par-shadow-covariance-scale must be > 0",
            argv[0]);
    }
    if (config.lambda_src_par_shadow_success_rate > 0.0 &&
        config.lambda_candidate_shadow_count < 2) {
        argumentError(
            "--lambda-src-par-shadow-success-rate requires "
            "--lambda-shadow-candidates >= 2",
            argv[0]);
    }
    if (config.lambda_satellite_par_shadow_max_drop_steps < 0 ||
        config.lambda_satellite_par_shadow_max_drop_steps > 32) {
        argumentError(
            "--lambda-satellite-par-shadow-max-drops must be in [0, 32]",
            argv[0]);
    }
    if (!std::isfinite(
            config.lambda_satellite_par_shadow_covariance_scale) ||
        config.lambda_satellite_par_shadow_covariance_scale <= 0.0) {
        argumentError(
            "--lambda-satellite-par-shadow-covariance-scale must be > 0",
            argv[0]);
    }
    if (config.lambda_satellite_par_shadow_max_drop_steps > 0 &&
        config.lambda_candidate_shadow_count < 2) {
        argumentError(
            "--lambda-satellite-par-shadow-max-drops requires "
            "--lambda-shadow-candidates >= 2",
            argv[0]);
    }
    if (
        config.enable_lambda_satellite_par_shadow_quality_diverse &&
        config.lambda_satellite_par_shadow_max_drop_steps <= 0) {
        argumentError(
            "--lambda-satellite-par-shadow-quality-diverse requires "
            "--lambda-satellite-par-shadow-max-drops > 0",
            argv[0]);
    }
    if (config.enable_lambda_l1_l5_wlnl_causal_arcs &&
        !config.enable_lambda_l1_l5_wlnl_shadow) {
        argumentError(
            "--lambda-l1-l5-wlnl-causal-arcs requires "
            "--lambda-l1-l5-wlnl-shadow",
            argv[0]);
    }
    if (config.enable_safe_fix_shadow_state_machine &&
        config.lambda_candidate_shadow_count < 2) {
        argumentError(
            "--safe-fix-shadow-state-machine requires "
            "--lambda-shadow-candidates >= 2",
            argv[0]);
    }
    if (config.enable_library_fixed_quality_gate &&
        !config.enable_safe_fix_robust_consensus_shadow) {
        argumentError(
            "--library-fixed-quality-gate requires "
            "--safe-fix-robust-consensus-shadow",
            argv[0]);
    }
    if (config.require_safe_fix_independent_failure_budget &&
        (!config.enable_safe_fix_robust_consensus_shadow ||
         !config.enable_lambda_l1_l5_wlnl_shadow)) {
        argumentError(
            "--safe-fix-independent-failure-budget requires "
            "--safe-fix-robust-consensus-shadow and "
            "--lambda-l1-l5-wlnl-shadow",
            argv[0]);
    }
    if (!std::isfinite(
            config.library_fixed_quality_max_covariance_trace_m2) ||
        config.library_fixed_quality_max_covariance_trace_m2 <= 0.0) {
        argumentError(
            "--library-fixed-quality-max-cov-trace must be > 0",
            argv[0]);
    }
    if (!std::isfinite(
            config.library_fixed_quality_max_nis_per_observation) ||
        config.library_fixed_quality_max_nis_per_observation <= 0.0) {
        argumentError(
            "--library-fixed-quality-max-nis must be > 0",
            argv[0]);
    }
    if (config.library_fixed_quality_min_strong_observations < 1) {
        argumentError(
            "--library-fixed-quality-min-observations must be >= 1",
            argv[0]);
    }
    if (!std::isfinite(
            config
                .library_fixed_quality_strong_max_nis_per_observation) ||
        config
                .library_fixed_quality_strong_max_nis_per_observation <=
            0.0) {
        argumentError(
            "--library-fixed-quality-strong-max-nis must be > 0",
            argv[0]);
    }
    if (
        !std::isfinite(config.safe_float_continuity_max_age_s) ||
        config.safe_float_continuity_max_age_s <= 0.0) {
        argumentError(
            "--safe-float-continuity-max-age must be > 0",
            argv[0]);
    }
    if (!std::isfinite(config.doppler_float_seed_max_age_s) ||
        config.doppler_float_seed_max_age_s <= 0.0) {
        argumentError("--doppler-float-seed-max-age must be > 0", argv[0]);
    }
    if (config.multisd_fgo_shadow_window_epochs < 2) {
        argumentError("--multisd-fgo-shadow-window must be >= 2", argv[0]);
    }
    if (config.multisd_fgo_shadow_min_epochs < 2 ||
        config.multisd_fgo_shadow_min_epochs >
            config.multisd_fgo_shadow_window_epochs) {
        argumentError(
            "--multisd-fgo-shadow-min-epochs must be in [2, window]",
            argv[0]);
    }
    if (config.multisd_fgo_shadow_holdout_offset < 0) {
        argumentError(
            "--multisd-fgo-shadow-holdout-offset must be non-negative",
            argv[0]);
    }
    if (config.multisd_fgo_shadow_top_k < 2 ||
        config.multisd_fgo_shadow_top_k > 32) {
        argumentError("--multisd-fgo-shadow-top-k must be in [2, 32]",
                      argv[0]);
    }
    if (!std::isfinite(
            config.multisd_fgo_shadow_max_seed_separation_m) ||
        config.multisd_fgo_shadow_max_seed_separation_m < 0.0) {
        argumentError(
            "--multisd-fgo-shadow-max-seed-separation must be >= 0",
            argv[0]);
    }
    if (config.multisd_fgo_shadow_validation_history_epochs < 1) {
        argumentError(
            "--multisd-fgo-shadow-validation-history must be >= 1",
            argv[0]);
    }
    if (!std::isfinite(config.multisd_fgo_shadow_min_carrier_fraction) ||
        config.multisd_fgo_shadow_min_carrier_fraction <= 0.0 ||
        config.multisd_fgo_shadow_min_carrier_fraction > 1.0) {
        argumentError(
            "--multisd-fgo-shadow-min-carrier-fraction must be in (0, 1]",
            argv[0]);
    }
    if (config.multisd_fgo_shadow_min_fixed_ambiguities < 2 ||
        config.multisd_fgo_shadow_min_fixed_ambiguities > 16) {
        argumentError(
            "--multisd-fgo-shadow-min-fixed-ambiguities must be in [2, 16]",
            argv[0]);
    }
    if (config.multisd_fgo_shadow_holdout_satellites < 2 ||
        config.multisd_fgo_shadow_holdout_satellites > 16) {
        argumentError(
            "--multisd-fgo-shadow-holdout-satellites must be in [2, 16]",
            argv[0]);
    }

    return config;
}

bool pathLooksStatic(const std::string& text) {
    return text.find("short_baseline") != std::string::npos ||
        text.find("static") != std::string::npos;
}

bool pathLooksShortBaseline(const std::string& text) {
    return text.find("short_baseline") != std::string::npos;
}

libgnss::RTKProcessor::RTKConfig::PositionMode resolvePositionMode(const SolveConfig& config) {
    if (config.mode == ModeChoice::KINEMATIC) {
        return libgnss::RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    }
    if (config.mode == ModeChoice::STATIC) {
        return libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC;
    }
    if (config.mode == ModeChoice::MOVING_BASE) {
        return libgnss::RTKProcessor::RTKConfig::PositionMode::MOVING_BASE;
    }

    const std::string hint = config.data_dir + " " + config.rover_obs_path + " " + config.base_obs_path;
    if (pathLooksStatic(hint)) {
        return libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC;
    }
    return libgnss::RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
}

libgnss::RTKProcessor::RTKConfig::IonoOpt resolveIonoOpt(const SolveConfig& config,
                                                         libgnss::RTKProcessor::RTKConfig::PositionMode mode) {
    if (config.iono == IonoChoice::OFF) {
        return libgnss::RTKProcessor::RTKConfig::IonoOpt::OFF;
    }
    if (config.iono == IonoChoice::IFLC) {
        return libgnss::RTKProcessor::RTKConfig::IonoOpt::IFLC;
    }
    if (config.iono == IonoChoice::EST) {
        return libgnss::RTKProcessor::RTKConfig::IonoOpt::EST;
    }

    const std::string hint = config.data_dir + " " + config.rover_obs_path + " " + config.base_obs_path;
    const bool short_baseline = pathLooksShortBaseline(hint);
    if (mode == libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC && !short_baseline) {
        return libgnss::RTKProcessor::RTKConfig::IonoOpt::IFLC;
    }
    return libgnss::RTKProcessor::RTKConfig::IonoOpt::OFF;
}

std::string positionModeString(libgnss::RTKProcessor::RTKConfig::PositionMode mode) {
    switch (mode) {
        case libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC:
            return "static";
        case libgnss::RTKProcessor::RTKConfig::PositionMode::KINEMATIC:
            return "kinematic";
        case libgnss::RTKProcessor::RTKConfig::PositionMode::MOVING_BASE:
            return "moving-base";
    }
    return "kinematic";
}

std::string ionoOptString(libgnss::RTKProcessor::RTKConfig::IonoOpt iono) {
    switch (iono) {
        case libgnss::RTKProcessor::RTKConfig::IonoOpt::OFF:
            return "off";
        case libgnss::RTKProcessor::RTKConfig::IonoOpt::IFLC:
            return "iflc";
        case libgnss::RTKProcessor::RTKConfig::IonoOpt::EST:
            return "est";
    }
    return "off";
}

bool writeSolutions(const SolveConfig& config, const libgnss::Solution& solution) {
    libgnss::io::SolutionWriter writer;
    if (!writer.open(config.output_pos_path, config.output_format)) {
        std::cerr << "Error: failed to open output file: " << config.output_pos_path << std::endl;
        return false;
    }

    for (const auto& epoch_solution : solution.solutions) {
        writer.writeEpoch(epoch_solution);
    }
    writer.close();

    if (config.write_kml && !solution.writeKML(config.output_kml_path)) {
        std::cerr << "Error: failed to write KML output: " << config.output_kml_path << std::endl;
        return false;
    }
    return true;
}

double roundedTowKey(double tow) {
    return std::round(tow * 10.0) / 10.0;
}

libgnss::FGOProcessor::FGOConfig makeGnssOnlyMultiSdShadowConfig(
    const SolveConfig& solve_config) {
    libgnss::FGOProcessor::FGOConfig config;
    config.backend = libgnss::FGOBackend::Eigen;
    config.max_iterations = 12;
    config.pseudorange_sigma_m = 4.0;
    config.motion_sigma_m = 100.0;
    config.clock_motion_sigma_m = 600.0;
    config.tdcp_sigma_m = 0.05;
    config.carrier_phase_sigma_m = 0.02;
    config.double_difference_pseudorange_sigma_m = 1.0;
    config.double_difference_carrier_sigma_m = 0.02;
    config.ambiguity_prior_sigma_m = 2000.0;
    config.fixed_ambiguity_sigma_m = 0.005;
    config.pseudorange_huber_threshold_sigma = 3.0;
    config.carrier_phase_huber_threshold_sigma = 3.0;
    config.tdcp_huber_threshold_sigma = 3.0;
    config.max_tdcp_gap_s = 1.5;
    config.base_epoch_match_tolerance_s = 0.02;
    config.base_interpolation_max_gap_s = 1.2;
    config.tdcp_code_phase_jump_threshold_m = 6.0;
    config.min_elevation_deg = 15.0;
    config.use_spp_seed = false;
    config.use_double_difference_factors = true;
    config.use_single_difference_doppler_factors = true;
    config.use_single_difference_tdcp_factors = true;
    config.use_velocity_states = true;
    config.use_velocity_motion_factors = true;
    config.fix_ambiguities = true;
    config.use_lambda_ambiguity_fix = true;
    config.lambda_ratio_threshold = 1.5;
    config.min_fixed_ambiguities =
        solve_config.multisd_fgo_shadow_min_fixed_ambiguities;
    config.max_lambda_ambiguities = 16;
    config.lambda_top_k_candidates = solve_config.multisd_fgo_shadow_top_k;
    config.parallelize_lambda_hypotheses = true;
    config.use_multisd_ambiguities = true;
    config.use_multisd_disjoint_validation = true;
    config.multisd_validation_holdout_satellites =
        solve_config.multisd_fgo_shadow_holdout_satellites;
    config.multisd_validation_holdout_offset =
        solve_config.multisd_fgo_shadow_holdout_offset;
    config.multisd_validation_aperture_cycles = 0.15;
    config.multisd_validation_history_epochs =
        solve_config.multisd_fgo_shadow_validation_history_epochs;
    config.multisd_validation_min_carrier_fraction =
        solve_config.multisd_fgo_shadow_min_carrier_fraction;
    config.multisd_validation_max_ddpr_rms_m = 3.0;
    config.multisd_validation_max_fixed_float_separation_m = 0.0;
    config.multisd_validation_max_seed_separation_m =
        solve_config.multisd_fgo_shadow_max_seed_separation_m;
    return config;
}

void writeOptionalCsvNumber(std::ostream& output, double value) {
    if (std::isfinite(value)) {
        output << value;
    }
}

std::map<double, Eigen::Vector3d> loadSeedPositions(const std::string& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("failed to open rover seed .pos: " + path);
    }
    std::map<double, Eigen::Vector3d> out;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '%') continue;
        std::istringstream iss(line);
        double week = 0.0;
        double tow = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!(iss >> week >> tow >> x >> y >> z)) continue;
        Eigen::Vector3d pos(x, y, z);
        if (std::isfinite(tow) && pos.allFinite() && pos.norm() > 1e6) {
            out[roundedTowKey(tow)] = pos;
        }
    }
    if (out.empty()) {
        throw std::runtime_error("rover seed .pos contained no usable ECEF rows: " + path);
    }
    return out;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const SolveConfig config = parseArguments(argc, argv);

        const std::map<double, Eigen::Vector3d> rover_seed_positions =
            config.rover_seed_pos_path.empty()
                ? std::map<double, Eigen::Vector3d>{}
                : loadSeedPositions(config.rover_seed_pos_path);

        libgnss::RTKProcessor rtk_processor;
        libgnss::SPPProcessor spp_processor;
        libgnss::RTKProcessor::RTKConfig rtk_config;
        rtk_config.prefer_trusted_position_seed = config.prefer_trusted_seed;
        rtk_config.use_doppler_float_seed = config.use_doppler_float_seed;
        rtk_config.doppler_float_seed_max_age_s = config.doppler_float_seed_max_age_s;
        rtk_config.prefer_rover_position_seed =
            config.prefer_trusted_seed || !rover_seed_positions.empty();
        if (config.rtk_update_outlier_threshold > 0.0) {
            rtk_config.outlier_threshold = config.rtk_update_outlier_threshold;
        }
        rtk_config.max_baseline_length = config.max_baseline_length_m;
        rtk_config.ar_mode = libgnss::RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
        rtk_config.ratio_threshold = config.ratio_threshold;
        rtk_config.ambiguity_ratio_threshold = config.ratio_threshold;
        rtk_config.enable_satellite_count_ratio_threshold =
            config.enable_satellite_count_ratio_threshold;
        rtk_config.hold_ambiguity_ratio_threshold = config.hold_ratio_threshold;
        rtk_config.enable_ar_filter = config.enable_ar_filter;
        rtk_config.ar_filter_margin = config.ar_filter_margin;
        rtk_config.nlos_weight_mode = config.nlos_weight_mode;
        rtk_config.nlos_two_tier_los_threshold = config.nlos_two_tier_los_threshold;
        rtk_config.nlos_two_tier_sigma_inflation = config.nlos_two_tier_sigma_inflation;
        rtk_config.nlos_continuous_los_prob_floor = config.nlos_continuous_los_prob_floor;
        rtk_config.nlos_tow_tolerance_s = config.nlos_tow_tolerance_s;
        rtk_config.nlos_exclude_threshold = config.nlos_exclude_threshold;
        rtk_config.nlos_min_sats = config.nlos_min_sats;
        rtk_config.float_trust_policy = config.float_trust_policy;
        rtk_config.trust_lapse_qpos_m2_per_s = config.trust_lapse_qpos_m2_per_s;
        rtk_config.trust_gate_nlos_relax = config.trust_gate_nlos_relax;
        rtk_config.trust_lapse_gate_s = config.trust_lapse_gate_s;
        rtk_config.trust_lapse_gate_nlos_frac = config.trust_lapse_gate_nlos_frac;
        rtk_config.nlos_min_los_sats = config.nlos_min_los_sats;
        rtk_config.cmc_aware_reference_selection = config.cmc_aware_reference_selection;
        rtk_config.cmc_ref_level_m = config.cmc_ref_level_m;
        rtk_config.cmc_ref_switch_epochs = config.cmc_ref_switch_epochs;
        rtk_config.cmc_ref_return_min_elev_deg = config.cmc_ref_return_min_elev_deg;
        rtk_config.cmc_ref_switch_max_elev_drop_deg = config.cmc_ref_switch_max_elev_drop_deg;
        rtk_config.cmc_ref_switch_min_elev_deg = config.cmc_ref_switch_min_elev_deg;
        rtk_config.enable_adaptive_measurement_noise = config.rtk_adaptive_noise;
        rtk_config.adaptive_noise_alpha_phase = config.rtk_adaptive_noise_alpha_phase;
        rtk_config.adaptive_noise_alpha_code = config.rtk_adaptive_noise_alpha_code;
        rtk_config.adaptive_noise_min_variance_scale = config.rtk_adaptive_noise_min_scale;
        rtk_config.adaptive_noise_max_variance_scale = config.rtk_adaptive_noise_max_scale;
        rtk_config.adaptive_noise_max_baseline_m = config.rtk_adaptive_noise_max_baseline_m;
        rtk_config.enable_cp_pr_fixed_gate = config.cp_pr_fixed_gate;
        rtk_config.cp_pr_fixed_gate_threshold_m = config.cp_pr_fixed_gate_threshold_m;
        rtk_config.cp_pr_fixed_gate_min_pairs = config.cp_pr_fixed_gate_min_pairs;
        rtk_config.cp_pr_fixed_gate_max_bad_pairs = config.cp_pr_fixed_gate_max_bad_pairs;
        rtk_config.cp_pr_fixed_gate_escalation_epochs =
            config.cp_pr_fixed_gate_escalation_epochs;
        rtk_config.ddpr_anchor_fde_threshold_m = config.ddpr_anchor_fde_threshold_m;
        rtk_config.ddpr_anchor_max_fde_removals = config.ddpr_anchor_max_fde_removals;
        rtk_config.low_ratio_guard_threshold = config.low_ratio_guard_threshold;
        rtk_config.low_ratio_min_fixed_ambiguities =
            config.low_ratio_min_fixed_ambiguities;
        rtk_config.low_count_rescue_ratio_threshold =
            config.low_count_rescue_ratio_threshold;
        rtk_config.low_count_rescue_min_fixed_ambiguities =
            config.low_count_rescue_min_fixed_ambiguities;
        rtk_config.low_count_rescue_max_history_speed_mps =
            config.low_count_rescue_max_history_speed_mps;
        rtk_config.min_satellites_for_ar = config.min_satellites_for_ar;
        rtk_config.min_subset_pairs_for_ar = config.min_subset_pairs_for_ar;
        rtk_config.max_subset_drop_steps_for_ar = config.max_subset_drop_steps_for_ar;
        rtk_config.min_subset_sats_for_ar = config.min_subset_sats_for_ar;
        rtk_config.min_subset_systems_for_ar = config.min_subset_systems_for_ar;
        rtk_config.min_subset_frequencies_for_ar = config.min_subset_frequencies_for_ar;
        rtk_config.min_subset_dual_frequency_sats_for_ar =
            config.min_subset_dual_frequency_sats_for_ar;
        rtk_config.min_full_ratio_for_subset_ar = config.min_full_ratio_for_subset_ar;
        rtk_config.min_hold_count = config.min_hold_count;
        rtk_config.elevation_mask = config.elevation_mask_deg * M_PI / 180.0;
        if (config.process_noise_position_set) {
            rtk_config.process_noise_position = config.process_noise_position;
        }
        if (config.process_noise_ambiguity_set) {
            rtk_config.process_noise_ambiguity = config.process_noise_ambiguity;
        }
        if (config.process_noise_iono_set) {
            rtk_config.process_noise_iono = config.process_noise_iono;
        }
        if (config.carrier_phase_sigma_set) {
            rtk_config.carrier_phase_sigma = config.carrier_phase_sigma;
        }
        rtk_config.enable_snr_weighting = config.enable_snr_weighting;
        rtk_config.snr_reference_dbhz = config.snr_reference_dbhz;
        rtk_config.snr_max_variance_scale = config.snr_max_variance_scale;
        rtk_config.snr_min_baseline_m = config.snr_min_baseline_m;
        rtk_config.cycle_slip_threshold = config.cycle_slip_threshold;
        rtk_config.doppler_slip_threshold = config.doppler_slip_threshold;
        rtk_config.code_slip_threshold = config.code_slip_threshold;
        rtk_config.use_dynamic_slip_threshold_floor = config.use_dynamic_slip_threshold_floor;
        rtk_config.enable_adaptive_dynamic_slip_thresholds =
            config.enable_adaptive_dynamic_slip_thresholds;
        rtk_config.adaptive_dynamic_slip_nonfix_count =
            config.adaptive_dynamic_slip_nonfix_count;
        rtk_config.adaptive_dynamic_slip_hold_epochs =
            config.adaptive_dynamic_slip_hold_epochs;
        rtk_config.position_mode = resolvePositionMode(config);
        rtk_config.ionoopt = resolveIonoOpt(config, rtk_config.position_mode);
        rtk_config.enable_glonass = config.enable_glonass;
        rtk_config.enable_beidou = config.enable_beidou;
        rtk_config.enable_l5 = config.enable_l5;  // Phase 18 Step 3
        rtk_config.glonass_ar_mode =
            config.glonass_ar == GlonassARChoice::AUTOCAL
                ? libgnss::RTKProcessor::RTKConfig::GlonassARMode::AUTOCAL
                : (config.glonass_ar == GlonassARChoice::ON
                       ? libgnss::RTKProcessor::RTKConfig::GlonassARMode::ON
                       : libgnss::RTKProcessor::RTKConfig::GlonassARMode::OFF);
        rtk_config.glonass_icb_l1_m_per_mhz = config.glonass_icb_l1_m_per_mhz;
        rtk_config.glonass_icb_l2_m_per_mhz = config.glonass_icb_l2_m_per_mhz;
        rtk_config.ar_policy = config.ar_policy;
        rtk_config.max_hold_divergence_m = config.max_hold_divergence_m;
        rtk_config.max_position_jump_m = config.max_position_jump_m;
        rtk_config.max_fixed_anchor_age_s = config.max_fixed_anchor_age_s;
        rtk_config.max_fixed_doppler_consensus_m = config.max_fixed_doppler_consensus_m;
        rtk_config.max_position_jump_min_m = config.max_position_jump_min_m;
        rtk_config.max_position_jump_rate_mps = config.max_position_jump_rate_mps;
        rtk_config.max_float_spp_divergence_m = config.max_float_spp_divergence_m;
        rtk_config.max_float_prefit_residual_rms_m =
            config.max_float_prefit_residual_rms_m;
        rtk_config.max_float_prefit_residual_max_m =
            config.max_float_prefit_residual_max_m;
        rtk_config.max_float_prefit_residual_reset_streak =
            config.max_float_prefit_residual_reset_streak;
        rtk_config.min_float_prefit_residual_trusted_jump_m =
            config.min_float_prefit_residual_trusted_jump_m;
        rtk_config.max_fixed_prefit_residual_rms_m =
            config.max_fixed_prefit_residual_rms_m;
        rtk_config.min_fixed_prefit_outliers = config.min_fixed_prefit_outliers;
        rtk_config.max_fixed_overconfidence_covariance_trace_m2 =
            config.max_fixed_overconfidence_covariance_trace_m2;
        rtk_config.fixed_prefit_reset_streak = config.fixed_prefit_reset_streak;
        rtk_config.fixed_prefit_quarantine_only = config.fixed_prefit_quarantine_only;
        rtk_config.max_update_nis_per_observation = config.max_update_nis_per_observation;
        rtk_config.student_t_front_end.enabled =
            config.student_t_rtk_front_end;
        rtk_config.max_fixed_update_nis_per_observation =
            config.max_fixed_update_nis_per_observation;
        rtk_config.max_fixed_update_post_residual_rms_m =
            config.max_fixed_update_post_residual_rms_m;
        rtk_config.max_fixed_update_gate_ratio = config.max_fixed_update_gate_ratio;
        rtk_config.min_fixed_update_gate_baseline_m =
            config.min_fixed_update_gate_baseline_m;
        rtk_config.max_fixed_update_gate_baseline_m =
            config.max_fixed_update_gate_baseline_m;
        rtk_config.min_fixed_update_gate_speed_mps =
            config.min_fixed_update_gate_speed_mps;
        rtk_config.max_fixed_update_gate_speed_mps =
            config.max_fixed_update_gate_speed_mps;
        rtk_config.max_fixed_update_secondary_gate_ratio =
            config.max_fixed_update_secondary_gate_ratio;
        rtk_config.min_fixed_update_secondary_gate_baseline_m =
            config.min_fixed_update_secondary_gate_baseline_m;
        rtk_config.max_fixed_update_secondary_gate_baseline_m =
            config.max_fixed_update_secondary_gate_baseline_m;
        rtk_config.min_fixed_update_secondary_gate_speed_mps =
            config.min_fixed_update_secondary_gate_speed_mps;
        rtk_config.max_fixed_update_secondary_gate_speed_mps =
            config.max_fixed_update_secondary_gate_speed_mps;
        rtk_config.max_consecutive_float_for_reset = config.max_consecutive_float_for_reset;
        rtk_config.max_consecutive_nonfix_for_reset = config.max_consecutive_nonfix_for_reset;
        rtk_config.max_postfix_residual_rms = config.max_postfix_residual_rms;
        rtk_config.enable_wide_lane_ar = config.enable_wide_lane_ar;
        rtk_config.wide_lane_acceptance_threshold = config.wide_lane_acceptance_threshold;
        rtk_config.enable_wlnl_fallback = config.enable_wlnl_fallback;
        rtk_config.enable_bsr_guided_decimation = config.enable_bsr_guided_decimation;
        rtk_config.bsr_guided_worst_axes = config.bsr_guided_worst_axes;
        rtk_config.bsr_guided_max_drop_steps = config.bsr_guided_max_drop_steps;
        rtk_config.lambda_candidate_shadow_count =
            config.lambda_candidate_shadow_count;
        rtk_config.lambda_src_par_shadow_success_rate =
            config.lambda_src_par_shadow_success_rate;
        rtk_config.lambda_src_par_shadow_covariance_scale =
            config.lambda_src_par_shadow_covariance_scale;
        rtk_config.lambda_satellite_par_shadow_max_drop_steps =
            config.lambda_satellite_par_shadow_max_drop_steps;
        rtk_config.lambda_satellite_par_shadow_covariance_scale =
            config.lambda_satellite_par_shadow_covariance_scale;
        rtk_config.lambda_satellite_par_shadow_quality_diverse =
            config
                .enable_lambda_satellite_par_shadow_quality_diverse;
        rtk_config.lambda_l1_l5_wlnl_shadow =
            config.enable_lambda_l1_l5_wlnl_shadow;
        rtk_config.lambda_l1_l5_wlnl_causal_arc_smoothing =
            config.enable_lambda_l1_l5_wlnl_causal_arcs;
        rtk_config.safe_fix_shadow_state_machine.enabled =
            config.enable_safe_fix_shadow_state_machine;
        rtk_config.safe_fix_shadow_state_machine
            .require_independent_failure_budget =
            config.require_safe_fix_independent_failure_budget;
        rtk_config.library_fixed_quality_gate.enabled =
            config.enable_library_fixed_quality_gate;
        rtk_config.library_fixed_quality_gate
            .require_independent_failure_budget =
            config.require_safe_fix_independent_failure_budget;
        rtk_config.library_fixed_quality_gate
            .maximum_float_position_covariance_trace_m2 =
            config.library_fixed_quality_max_covariance_trace_m2;
        rtk_config.library_fixed_quality_gate
            .maximum_covariance_branch_nis_per_observation =
            config.library_fixed_quality_max_nis_per_observation;
        rtk_config.library_fixed_quality_gate
            .minimum_strong_innovation_observations =
            config.library_fixed_quality_min_strong_observations;
        rtk_config.library_fixed_quality_gate
            .maximum_strong_innovation_nis_per_observation =
            config
                .library_fixed_quality_strong_max_nis_per_observation;
        if (config.enable_safe_fix_robust_consensus_shadow) {
            rtk_config.safe_fix_shadow_state_machine
                .acquisition_streak_epochs = 3;
            rtk_config.safe_fix_shadow_state_machine
                .maximum_acquisition_correction_jump_m = 0.02;
            rtk_config.safe_fix_shadow_state_machine
                .maximum_hold_epochs = 0;
            rtk_config.safe_fix_shadow_state_machine
                .minimum_absolute_ratio = 1.4;
            rtk_config.safe_fix_shadow_state_machine
                .maximum_independent_consensus_delta_m = 0.10;
            rtk_config.safe_fix_shadow_state_machine
                .allow_strong_instant_acquisition = true;
            rtk_config.safe_fix_shadow_state_machine
                .allow_change_point_acquisition = true;
            rtk_config.safe_fix_shadow_minimum_pairs = 12;
            rtk_config
                .safe_fix_shadow_maximum_second_position_delta_m =
                0.25;
            rtk_config.safe_fix_shadow_maximum_nis_per_observation =
                3.0;
        }
        rtk_config.safe_float_continuity.enabled =
            config.enable_safe_float_continuity;
        rtk_config.safe_float_continuity.maximum_anchor_age_s =
            config.safe_float_continuity_max_age_s;
        rtk_config.safe_float_continuity.maximum_velocity_age_s =
            config.safe_float_continuity_max_age_s;
        if (!config.nlos_weights_csv_path.empty()) {
            auto table = std::make_shared<libgnss::nlos_weights::NlosWeightTable>(
                libgnss::nlos_weights::loadNlosWeightsCsv(config.nlos_weights_csv_path));
            std::cout << "Loaded NLOS weights: " << config.nlos_weights_csv_path
                      << " (" << table->by_tow.size() << " distinct epochs)" << std::endl;
            rtk_processor.setNlosWeightTable(std::move(table));
        }
        rtk_processor.setRTKConfig(rtk_config);
        libgnss::SPPProcessor::SPPConfig spp_config;
        spp_config.use_multi_constellation = true;
        spp_config.enable_glonass = config.enable_glonass;
        spp_config.enable_beidou = config.enable_beidou;
        spp_processor.setSPPConfig(spp_config);
        EpochDebugWriter debug_writer;
        if (!debug_writer.open(config.debug_epoch_log_path)) {
            std::cerr << "Error: failed to open debug epoch log: "
                      << config.debug_epoch_log_path << std::endl;
            return 1;
        }

        std::ofstream diagnostics_csv;
        if (!config.diagnostics_csv_path.empty()) {
            diagnostics_csv.open(config.diagnostics_csv_path);
            if (!diagnostics_csv.is_open()) {
                std::cerr << "Error: failed to open diagnostics CSV: "
                          << config.diagnostics_csv_path << std::endl;
                return 1;
            }
            writeDiagnosticsHeader(diagnostics_csv);
        }

        const libgnss::FGOProcessor multisd_shadow_processor(
            makeGnssOnlyMultiSdShadowConfig(config));
        std::vector<libgnss::ObservationData> multisd_shadow_rover_window;
        std::vector<libgnss::ObservationData> multisd_shadow_base_window;
        multisd_shadow_rover_window.reserve(
            static_cast<std::size_t>(config.multisd_fgo_shadow_window_epochs));
        multisd_shadow_base_window.reserve(
            static_cast<std::size_t>(config.multisd_fgo_shadow_window_epochs));
        std::ofstream multisd_shadow_csv;
        if (!config.multisd_fgo_shadow_csv_path.empty()) {
            multisd_shadow_csv.open(config.multisd_fgo_shadow_csv_path);
            if (!multisd_shadow_csv.is_open()) {
                std::cerr << "Error: failed to open MultiSD FGO shadow CSV: "
                          << config.multisd_fgo_shadow_csv_path << std::endl;
                return 1;
            }
            multisd_shadow_csv
                << "epoch_index,gps_week,tow,window_epochs,rtk_status,"
                   "shadow_status,shadow_fixed,lambda_ratio,fixed_ambiguities,"
                   "validation_evaluated,validation_pass,holdout_satellites,"
                   "hypotheses_passed,hypotheses_evaluated,selected_rank,"
                   "carrier_passed,carrier_used,max_integer_distance_cycles,"
                   "ddpr_rms_m,x,y,z,rtk_position_delta_m,build_runtime_ms,"
                   "optimize_wall_ms,optimizer_cpu_ms,runtime_ms,"
                   "normal_state_size,cuda_selected,cuda_attempts,"
                   "cuda_successes,cuda_fallbacks,cuda_runtime_ms,"
                   "fixed_float_separation_m,seed_separation_m\n";
        }

        std::unique_ptr<IntegrityShadowTimeline> integrity_shadow;
        if (!config.integrity_shadow_csv_path.empty()) {
            IntegrityShadowTimeline::Config shadow_config;
            shadow_config.match_tolerance_s = config.integrity_shadow_match_tolerance_s;
            shadow_config.health.max_age_s = config.integrity_shadow_max_age_s;
            shadow_config.health.max_gdop = config.integrity_shadow_max_gdop;
            shadow_config.health.max_ddpr_rms_m = config.integrity_shadow_max_ddpr_rms_m;
            shadow_config.health.min_satellites = config.integrity_shadow_min_satellites;
            shadow_config.health.max_covariance_trace_m2 =
                config.integrity_shadow_max_covariance_trace_m2;
            shadow_config.health.assume_default_covariance_trace =
                config.integrity_shadow_assume_default_covariance_trace;
            shadow_config.health.default_covariance_trace_m2 =
                config.integrity_shadow_default_covariance_trace_m2;
            shadow_config.health.require_fixed_status =
                config.integrity_shadow_require_fixed_status;
            integrity_shadow = std::make_unique<IntegrityShadowTimeline>(shadow_config);
            if (!integrity_shadow->load(config.integrity_shadow_csv_path)) {
                std::cerr << "Error: failed to load integrity shadow CSV: "
                          << config.integrity_shadow_csv_path << std::endl;
                return 1;
            }
        }

        std::unique_ptr<libgnss::RealtimeFixIntegrityGate> integrity_gate;
        IntegrityTelemetryWriter integrity_writer;
        if (config.enable_realtime_fix_integrity) {
            libgnss::RealtimeFixIntegrityGate::Config integrity_config;
            integrity_config.enable_consensus = integrity_shadow != nullptr;
            integrity_config.enable_base_confidence_policy =
                config.enable_integrity_base_gate;
            integrity_gate =
                std::make_unique<libgnss::RealtimeFixIntegrityGate>(integrity_config);
            if (!integrity_writer.open(config.integrity_log_path)) {
                std::cerr << "Error: failed to open integrity log: "
                          << config.integrity_log_path << std::endl;
                return 1;
            }
        }

        std::cout << "libgnss++ post-process solver" << std::endl;
        std::cout << "  rover: " << config.rover_obs_path << std::endl;
        std::cout << "  base: " << config.base_obs_path << std::endl;
        std::cout << "  nav: " << config.nav_path << std::endl;
        std::cout << "  out: " << config.output_pos_path << " (" << outputFormatString(config.output_format)
                  << ")" << std::endl;
        if (config.write_kml) {
            std::cout << "  kml: " << config.output_kml_path << std::endl;
        }
        std::cout << "  mode: " << positionModeString(rtk_config.position_mode)
                  << " (requested " << modeChoiceString(config.mode) << ")" << std::endl;
        std::cout << "  iono: " << ionoOptString(rtk_config.ionoopt)
                  << " (requested " << ionoChoiceString(config.iono) << ")" << std::endl;
        std::cout << "  carrier constellations: GLONASS "
                  << (rtk_config.enable_glonass ? "on" : "off")
                  << ", BeiDou " << (rtk_config.enable_beidou ? "on" : "off")
                  << ", L5 " << (rtk_config.enable_l5 ? "on" : "off") << std::endl;
        std::cout << "  GLONASS AR: " << glonassARChoiceString(config.glonass_ar)
                  << " (L1 ICB " << config.glonass_icb_l1_m_per_mhz
                  << " m/MHz, L2 ICB " << config.glonass_icb_l2_m_per_mhz << " m/MHz)"
                  << std::endl;
        std::cout << "  base interpolation: "
                  << (config.enable_base_interpolation ? "enabled" : "disabled") << std::endl;
        if (integrity_gate) {
            std::cout << "  real-time FIX integrity: enabled (max latency "
                      << integrity_gate->maxOutputLatencyEpochs() << " epochs, consensus "
                      << (integrity_shadow ? "enabled" : "disabled") << ')' << std::endl;
        }

        libgnss::io::RINEXReader rover_reader;
        libgnss::io::RINEXReader base_reader;
        libgnss::io::RINEXReader nav_reader;

        if (!rover_reader.open(config.rover_obs_path)) {
            std::cerr << "Error: cannot open rover observation file" << std::endl;
            return 1;
        }
        libgnss::io::RINEXReader::RINEXHeader rover_header;
        if (!rover_reader.readHeader(rover_header)) {
            std::cerr << "Error: failed to read rover observation header" << std::endl;
            return 1;
        }

        if (!base_reader.open(config.base_obs_path)) {
            std::cerr << "Error: cannot open base observation file" << std::endl;
            return 1;
        }
        libgnss::io::RINEXReader::RINEXHeader base_header;
        if (!base_reader.readHeader(base_header)) {
            std::cerr << "Error: failed to read base observation header" << std::endl;
            return 1;
        }

        if (!nav_reader.open(config.nav_path)) {
            std::cerr << "Error: cannot open navigation file" << std::endl;
            return 1;
        }
        libgnss::NavigationData nav_data;
        if (!nav_reader.readNavigationData(nav_data)) {
            std::cerr << "Error: failed to read navigation data" << std::endl;
            return 1;
        }

        Eigen::Vector3d base_position = Eigen::Vector3d::Zero();
        if (config.base_position_override) {
            base_position = config.base_position_ecef;
            if (base_header.approximate_position.norm() > 0.0) {
                const double override_delta =
                    (config.base_position_ecef - base_header.approximate_position).norm();
                if (override_delta > 1000.0) {
                    std::cerr << "Warning: --base-ecef differs from RINEX header by "
                              << override_delta << " m; this may severely degrade RTK alignment."
                              << std::endl;
                }
            }
        } else if (base_header.approximate_position.norm() > 0.0) {
            base_position = base_header.approximate_position;
        } else {
            std::cerr << "Error: base position unavailable. Use --base-ecef to override." << std::endl;
            return 1;
        }
        rtk_processor.setBasePosition(base_position);

        libgnss::Solution solution;
        libgnss::ObservationData rover_obs;
        libgnss::ObservationData base_obs;
        libgnss::ObservationData previous_base_obs;
        bool has_previous_base = false;
        bool rover_ok = rover_reader.readObservationEpoch(rover_obs);
        bool base_ok = base_reader.readObservationEpoch(base_obs);

        if (!rover_ok) {
            std::cerr << "Error: no rover epochs available" << std::endl;
            return 1;
        }
        if (!base_ok) {
            std::cerr << "Error: no base epochs available" << std::endl;
            return 1;
        }

        if (rover_header.approximate_position.norm() > 0.0) {
            rover_obs.receiver_position = rover_header.approximate_position;
        } else {
            rover_obs.receiver_position = base_position + Eigen::Vector3d(3000.0, 0.0, 0.0);
        }

        int skipped_initial_epochs = 0;
        while (rover_ok && skipped_initial_epochs < config.skip_epochs) {
            const Eigen::Vector3d saved_rover_pos = rover_obs.receiver_position;
            rover_ok = rover_reader.readObservationEpoch(rover_obs);
            if (rover_ok) {
                rover_obs.receiver_position = saved_rover_pos;
            }
            skipped_initial_epochs++;
        }
        if (!rover_ok) {
            std::cerr << "Error: skip-epochs exhausted the rover observation stream" << std::endl;
            return 1;
        }

        int processed_rover_epochs = 0;
        int valid_solution_count = 0;
        int fixed_solution_count = 0;
        int exact_base_epochs = 0;
        int interpolated_base_epochs = 0;
        int skipped_rover_epochs = 0;
        int nonfix_drift_guard_inspected_segments = 0;
        int nonfix_drift_guard_rejected_segments = 0;
        int nonfix_drift_guard_rejected_epochs = 0;
        int spp_height_step_guard_rejected_epochs = 0;
        int float_bridge_tail_guard_inspected_segments = 0;
        int float_bridge_tail_guard_rejected_segments = 0;
        int float_bridge_tail_guard_rejected_epochs = 0;
        int fixed_bridge_burst_guard_inspected_segments = 0;
        int fixed_bridge_burst_guard_rejected_segments = 0;
        int fixed_bridge_burst_guard_rejected_epochs = 0;
        int kinematic_postfilter_continuity_epochs = 0;
        int immediate_guard_continuity_epochs = 0;
        int availability_spp_fallback_epochs = 0;
        int availability_propagated_fallback_epochs = 0;
        int availability_postfilter_reinserted_epochs = 0;
        std::size_t multisd_shadow_attempts = 0;
        std::size_t multisd_shadow_valid = 0;
        std::size_t multisd_shadow_fixed = 0;
        double multisd_shadow_total_runtime_ms = 0.0;
        libgnss::PositionSolution last_fixed_output;
        bool have_last_fixed_output = false;
        libgnss::PositionSolution last_guard_output;
        bool have_last_guard_output = false;

        const auto record_output = [&](const libgnss::PositionSolution& output_solution) {
            if (!output_solution.isValid()) return;
            solution.addSolution(output_solution);
            ++valid_solution_count;
            if (output_solution.isFixed()) ++fixed_solution_count;
            if (config.verbose &&
                (valid_solution_count <= 5 || valid_solution_count % 100 == 0)) {
                std::cout << "epoch " << valid_solution_count
                          << " tow=" << std::fixed << std::setprecision(3)
                          << output_solution.time.tow
                          << " status=" << static_cast<int>(output_solution.status)
                          << " sats=" << output_solution.num_satellites
                          << " ratio=" << std::setprecision(2) << output_solution.ratio
                          << std::endl;
            }
        };

        const auto record_integrity_emissions = [&](const auto& emissions) {
            for (const auto& emission : emissions) {
                integrity_writer.write(emission);
                record_output(emission.solution);
            }
        };
        libgnss::PositionSolution last_real_output;
        bool have_last_real_output = false;

        while (rover_ok) {
            if (config.max_epochs > 0 && processed_rover_epochs >= config.max_epochs) {
                break;
            }

            libgnss::ObservationData lower_base_obs = previous_base_obs;
            bool have_lower_base = has_previous_base;

            while (base_ok && timeDiffSeconds(base_obs.time, rover_obs.time) < -kExactTimeToleranceSeconds) {
                lower_base_obs = base_obs;
                have_lower_base = true;
                previous_base_obs = base_obs;
                has_previous_base = true;

                libgnss::ObservationData next_base_obs;
                base_ok = base_reader.readObservationEpoch(next_base_obs);
                if (base_ok) {
                    base_obs = std::move(next_base_obs);
                }
            }

            libgnss::ObservationData aligned_base_obs;
            const double exact_dt = base_ok ? std::abs(timeDiffSeconds(base_obs.time, rover_obs.time))
                                            : std::numeric_limits<double>::infinity();
            bool have_aligned_base = false;
            bool used_interpolated_base = false;

            if (base_ok && exact_dt <= kExactTimeToleranceSeconds) {
                aligned_base_obs = base_obs;
                exact_base_epochs++;
                have_aligned_base = true;
            } else if (config.enable_base_interpolation && base_ok && have_lower_base &&
                       timeDiffSeconds(rover_obs.time, lower_base_obs.time) >= -kExactTimeToleranceSeconds &&
                       timeDiffSeconds(base_obs.time, rover_obs.time) >= -kExactTimeToleranceSeconds &&
                       interpolateBaseEpoch(lower_base_obs, base_obs, rover_obs.time,
                                            base_position, nav_data, aligned_base_obs)) {
                interpolated_base_epochs++;
                have_aligned_base = true;
                used_interpolated_base = true;
            }

            if (!have_aligned_base) {
                skipped_rover_epochs++;
                const Eigen::Vector3d saved_rover_pos = rover_obs.receiver_position;
                rover_ok = rover_reader.readObservationEpoch(rover_obs);
                if (rover_ok) {
                    rover_obs.receiver_position = saved_rover_pos;
                }
                processed_rover_epochs++;
                continue;
            }

            if (!rover_seed_positions.empty()) {
                const auto seed_it =
                    rover_seed_positions.find(roundedTowKey(rover_obs.time.tow));
                if (seed_it != rover_seed_positions.end()) {
                    rover_obs.receiver_position = seed_it->second;
                }
            }

            auto pos_solution = rtk_processor.processRTKEpoch(rover_obs, aligned_base_obs, nav_data);

            // GNSS-only, strictly causal shadow: the window contains only
            // observations at or before this RTK epoch, and only its latest
            // solution is recorded. It has no authority over RTK output or
            // state. RTK's current estimate is used solely as the geometry
            // seed (never reference truth).
            if (multisd_shadow_csv.is_open()) {
                libgnss::ObservationData shadow_rover = rover_obs;
                if (pos_solution.isValid() &&
                    pos_solution.position_ecef.allFinite()) {
                    shadow_rover.receiver_position =
                        pos_solution.position_ecef;
                    shadow_rover.receiver_clock_bias =
                        pos_solution.receiver_clock_bias;
                }
                multisd_shadow_rover_window.push_back(
                    std::move(shadow_rover));
                multisd_shadow_base_window.push_back(aligned_base_obs);
                const std::size_t maximum_window =
                    static_cast<std::size_t>(
                        config.multisd_fgo_shadow_window_epochs);
                if (multisd_shadow_rover_window.size() > maximum_window) {
                    multisd_shadow_rover_window.erase(
                        multisd_shadow_rover_window.begin());
                    multisd_shadow_base_window.erase(
                        multisd_shadow_base_window.begin());
                }

                if (multisd_shadow_rover_window.size() >=
                    static_cast<std::size_t>(
                        config.multisd_fgo_shadow_min_epochs)) {
                    const auto shadow_start =
                        std::chrono::steady_clock::now();
                    const auto shadow_problem =
                        multisd_shadow_processor.buildDoubleDifferenceProblem(
                            multisd_shadow_rover_window,
                            multisd_shadow_base_window,
                            nav_data,
                            base_position);
                    const auto shadow_optimize_start =
                        std::chrono::steady_clock::now();
                    const auto shadow_result =
                        multisd_shadow_processor.optimizeProblem(shadow_problem);
                    const auto shadow_end =
                        std::chrono::steady_clock::now();
                    const double shadow_build_runtime_ms =
                        std::chrono::duration<double, std::milli>(
                            shadow_optimize_start - shadow_start)
                            .count();
                    const double shadow_optimize_wall_ms =
                        std::chrono::duration<double, std::milli>(
                            shadow_end - shadow_optimize_start)
                            .count();
                    const double shadow_runtime_ms =
                        std::chrono::duration<double, std::milli>(
                            shadow_end - shadow_start)
                            .count();
                    ++multisd_shadow_attempts;
                    multisd_shadow_total_runtime_ms += shadow_runtime_ms;

                    const libgnss::PositionSolution* shadow_latest = nullptr;
                    if (!shadow_result.solution.solutions.empty()) {
                        const auto& candidate =
                            shadow_result.solution.solutions.back();
                        if (std::abs(candidate.time - rover_obs.time) <=
                            kExactTimeToleranceSeconds) {
                            shadow_latest = &candidate;
                        }
                    }
                    if (shadow_latest && shadow_latest->isValid()) {
                        ++multisd_shadow_valid;
                        if (shadow_latest->isFixed()) {
                            ++multisd_shadow_fixed;
                        }
                    }

                    const auto& diagnostics = shadow_result.diagnostics;
                    multisd_shadow_csv
                        << processed_rover_epochs << ','
                        << rover_obs.time.week << ','
                        << std::setprecision(12) << rover_obs.time.tow << ','
                        << multisd_shadow_rover_window.size() << ','
                        << static_cast<int>(pos_solution.status) << ','
                        << (shadow_latest
                                ? static_cast<int>(shadow_latest->status)
                                : static_cast<int>(libgnss::SolutionStatus::NONE))
                        << ','
                        << (shadow_latest && shadow_latest->isFixed() ? 1 : 0)
                        << ',';
                    writeOptionalCsvNumber(
                        multisd_shadow_csv,
                        shadow_latest ? shadow_latest->ratio
                                      : diagnostics.lambda_ambiguity_ratio);
                    multisd_shadow_csv << ','
                        << (shadow_latest
                                ? shadow_latest->num_fixed_ambiguities
                                : 0)
                        << ','
                        << (diagnostics.multisd_validation_evaluated ? 1 : 0)
                        << ','
                        << (diagnostics.multisd_validation_pass ? 1 : 0)
                        << ','
                        << diagnostics.multisd_validation_holdout_satellites
                        << ','
                        << diagnostics.multisd_validation_hypotheses_passed
                        << ','
                        << diagnostics.multisd_validation_hypotheses_evaluated
                        << ','
                        << diagnostics.multisd_validation_selected_rank
                        << ','
                        << diagnostics.multisd_validation_carrier_passed
                        << ','
                        << diagnostics.multisd_validation_carrier_used
                        << ',';
                    writeOptionalCsvNumber(
                        multisd_shadow_csv,
                        diagnostics
                            .multisd_validation_max_integer_distance_cycles);
                    multisd_shadow_csv << ',';
                    writeOptionalCsvNumber(
                        multisd_shadow_csv,
                        diagnostics.multisd_validation_ddpr_rms_m);
                    multisd_shadow_csv << ',';
                    if (shadow_latest) {
                        multisd_shadow_csv
                            << std::setprecision(12)
                            << shadow_latest->position_ecef.x() << ','
                            << shadow_latest->position_ecef.y() << ','
                            << shadow_latest->position_ecef.z() << ',';
                        if (pos_solution.isValid()) {
                            writeOptionalCsvNumber(
                                multisd_shadow_csv,
                                (shadow_latest->position_ecef -
                                 pos_solution.position_ecef)
                                    .norm());
                        }
                    } else {
                        multisd_shadow_csv << ",,,";
                    }
                    multisd_shadow_csv
                        << ',' << shadow_build_runtime_ms
                        << ',' << shadow_optimize_wall_ms
                        << ',' << diagnostics.processing_time_ms
                        << ',' << shadow_runtime_ms
                        << ',' << diagnostics.dense_normal_state_size
                        << ',' << (diagnostics.cuda_dense_solver_selected ? 1 : 0)
                        << ',' << diagnostics.cuda_dense_solve_attempts
                        << ',' << diagnostics.cuda_dense_solve_successes
                        << ',' << diagnostics.cuda_dense_solve_fallbacks
                        << ',' << diagnostics.cuda_dense_solve_time_ms
                        << ',';
                    writeOptionalCsvNumber(
                        multisd_shadow_csv,
                        diagnostics
                            .multisd_validation_fixed_float_separation_m);
                    multisd_shadow_csv << ',';
                    writeOptionalCsvNumber(
                        multisd_shadow_csv,
                        diagnostics.multisd_validation_seed_separation_m);
                    multisd_shadow_csv
                        << '\n';
                }
            }
            const libgnss::PositionSolution* last_output =
                have_last_guard_output ? &last_guard_output : nullptr;
            const bool have_last_output = last_output != nullptr;
            bool immediate_guard_continuity_used = false;
            bool availability_fallback_used = false;
            const auto replace_rejected_with_continuity =
                [&](const libgnss::PositionSolution& rejected) {
                    if (!rtk_config.safe_float_continuity.enabled) {
                        return false;
                    }
                    libgnss::safe_float_continuity::Result continuity;
                    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
                    auto continuity_config =
                        rtk_config.safe_float_continuity;
                    const auto try_anchor =
                        [&](const libgnss::PositionSolution& anchor,
                            double maximum_age_s) {
                            const double anchor_age_s =
                                rejected.time - anchor.time;
                            const bool use_current_velocity =
                                rejected.has_velocity &&
                                rejected.velocity_ecef.allFinite();
                            if (!use_current_velocity &&
                                !anchor.has_velocity) {
                                return false;
                            }
                            velocity =
                                use_current_velocity
                                    ? rejected.velocity_ecef
                                    : anchor.velocity_ecef;
                            auto candidate_config =
                                rtk_config.safe_float_continuity;
                            candidate_config.maximum_anchor_age_s =
                                std::min(
                                    candidate_config
                                        .maximum_anchor_age_s,
                                    maximum_age_s);
                            continuity =
                                libgnss::safe_float_continuity::propagate(
                                    candidate_config,
                                    anchor.position_ecef,
                                    anchor_age_s,
                                    velocity,
                                    use_current_velocity
                                        ? 0.0
                                        : anchor_age_s);
                            if (continuity.valid) {
                                continuity_config = candidate_config;
                            }
                            return continuity.valid;
                        };
                    // Prefer a validated FIXED anchor for the configured
                    // outage horizon. Fall back to a real solver output only
                    // for an isolated sub-second guard rejection.
                    bool propagated =
                        have_last_fixed_output &&
                        try_anchor(
                            last_fixed_output,
                            continuity_config.maximum_anchor_age_s);
                    if (!propagated && have_last_real_output) {
                        propagated = try_anchor(
                            last_real_output,
                            continuity_config
                                .maximum_solver_gap_anchor_age_s);
                    }
                    if (!propagated) {
                        return false;
                    }
                    pos_solution = libgnss::PositionSolution{};
                    pos_solution.time = rejected.time;
                    pos_solution.status =
                        libgnss::SolutionStatus::FLOAT;
                    pos_solution.position_ecef =
                        continuity.position_ecef;
                    pos_solution.position_geodetic =
                        libgnss::spp_utils::ecefToGeodetic(
                            pos_solution.position_ecef);
                    pos_solution.position_covariance =
                        Eigen::Matrix3d::Identity() *
                        continuity.position_variance_m2;
                    pos_solution.velocity_ecef = velocity;
                    pos_solution.velocity_covariance =
                        Eigen::Matrix3d::Identity() *
                        std::pow(
                            continuity_config.velocity_sigma_mps,
                            2.0);
                    pos_solution.has_velocity = true;
                    pos_solution.num_satellites = 4;
                    pos_solution.ratio = 0.0;
                    pos_solution.num_fixed_ambiguities = 0;
                    immediate_guard_continuity_used = true;
                    ++immediate_guard_continuity_epochs;
                    return true;
                };
            const auto jump_from_last_output = [&](const libgnss::PositionSolution& candidate) {
                if (!have_last_output || !candidate.isValid()) {
                    return std::numeric_limits<double>::infinity();
                }
                return (candidate.position_ecef - last_output->position_ecef).norm();
            };
            const auto max_nonfixed_jump = [&]() {
                if (!have_last_output) {
                    return kDefaultNonFixedJumpGuardMeters;
                }
                double dt = pos_solution.time - last_output->time;
                if (!std::isfinite(dt) || dt <= 0.0) {
                    dt = 1.0;
                }
                return std::max(kDefaultNonFixedJumpGuardMeters, 25.0 * dt);
            };
            // A raw solver gap used to bypass the same bounded FLOAT-only
            // continuity path that handles rejected jumps and height steps.
            // Route it through that fail-closed path as well: propagation
            // still requires a trusted finite anchor, recent velocity, and
            // the configured maximum age, and can never declare FIX.
            if (!pos_solution.isValid()) {
                auto rejected = pos_solution;
                rejected.time = rover_obs.time;
                replace_rejected_with_continuity(rejected);
            }
            if (
                !pos_solution.isValid() &&
                config.enable_safe_availability_fallback) {
                auto spp_fallback =
                    spp_processor.processEpoch(rover_obs, nav_data);
                if (spp_fallback.isValid() &&
                    spp_fallback.position_ecef.allFinite()) {
                    spp_fallback.status =
                        libgnss::SolutionStatus::SPP;
                    pos_solution = spp_fallback;
                    availability_fallback_used = true;
                    ++availability_spp_fallback_epochs;
                } else {
                    const libgnss::PositionSolution*
                        availability_anchor =
                            last_output != nullptr &&
                                    last_output
                                        ->position_ecef
                                        .allFinite()
                                ? last_output
                                : nullptr;
                    Eigen::Vector3d anchor_position =
                        Eigen::Vector3d::Zero();
                    double anchor_variance_m2 = 1e6;
                    double dt = 0.2;
                    if (availability_anchor != nullptr) {
                        anchor_position =
                            availability_anchor->position_ecef;
                        const double candidate_dt =
                            rover_obs.time -
                            availability_anchor->time;
                        if (
                            std::isfinite(candidate_dt) &&
                            candidate_dt > 0.0) {
                            dt = candidate_dt;
                        }
                        const double candidate_variance =
                            availability_anchor
                                ->position_covariance
                                .trace() /
                            3.0;
                        if (
                            std::isfinite(candidate_variance) &&
                            candidate_variance >= 25.0) {
                            anchor_variance_m2 =
                                candidate_variance;
                        } else {
                            anchor_variance_m2 = 25.0;
                        }
                    } else if (
                        rover_obs.receiver_position.allFinite() &&
                        rover_obs.receiver_position.norm() > 1e6) {
                        // This is an input/header or explicitly supplied
                        // runtime seed, never audit truth. It is exposed only
                        // as high-variance PROPAGATED availability.
                        anchor_position =
                            rover_obs.receiver_position;
                    } else {
                        anchor_position =
                            Eigen::Vector3d::Constant(
                                std::numeric_limits<double>::quiet_NaN());
                    }

                    Eigen::Vector3d velocity =
                        Eigen::Vector3d::Zero();
                    bool have_bounded_velocity = false;
                    if (
                        availability_anchor != nullptr &&
                        availability_anchor->has_velocity &&
                        availability_anchor
                            ->velocity_ecef.allFinite() &&
                        availability_anchor
                                ->velocity_ecef.norm() <= 60.0) {
                        velocity =
                            availability_anchor->velocity_ecef;
                        have_bounded_velocity = true;
                    } else if (
                        availability_anchor == last_output &&
                        last_output != nullptr &&
                        solution.solutions.size() >= 2) {
                        const auto& previous =
                            solution.solutions[
                                solution.solutions.size() - 2];
                        const double velocity_dt =
                            last_output->time - previous.time;
                        if (
                            previous.position_ecef.allFinite() &&
                            std::isfinite(velocity_dt) &&
                            velocity_dt > 0.0 &&
                            velocity_dt <= 2.0) {
                            const Eigen::Vector3d candidate_velocity =
                                (last_output->position_ecef -
                                 previous.position_ecef) /
                                velocity_dt;
                            if (
                                candidate_velocity.allFinite() &&
                                candidate_velocity.norm() <= 60.0) {
                                velocity = candidate_velocity;
                                have_bounded_velocity = true;
                            }
                        }
                    }
                    pos_solution =
                        libgnss::PositionSolution{};
                    pos_solution.time = rover_obs.time;
                    pos_solution.status =
                        libgnss::SolutionStatus::PROPAGATED;
                    pos_solution.position_ecef =
                        anchor_position + velocity * dt;
                    pos_solution.position_geodetic =
                        libgnss::spp_utils::ecefToGeodetic(
                            pos_solution.position_ecef);
                    const double growth_sigma_m = 25.0 * dt;
                    pos_solution.position_covariance =
                        Eigen::Matrix3d::Identity() *
                        (anchor_variance_m2 +
                         growth_sigma_m * growth_sigma_m);
                    pos_solution.velocity_ecef = velocity;
                    pos_solution.velocity_covariance =
                        Eigen::Matrix3d::Identity() * 625.0;
                    pos_solution.has_velocity =
                        have_bounded_velocity;
                    pos_solution.num_satellites = 0;
                    pos_solution.ratio = 0.0;
                    pos_solution.num_fixed_ambiguities = 0;
                    availability_fallback_used =
                        pos_solution.isValid();
                    if (availability_fallback_used) {
                        ++availability_propagated_fallback_epochs;
                    }
                }
            }
            if (used_interpolated_base && pos_solution.status == libgnss::SolutionStatus::FLOAT) {
                auto spp_solution = spp_processor.processEpoch(rover_obs, nav_data);
                if (spp_solution.isValid()) {
                    const double float_vs_spp = (pos_solution.position_ecef - spp_solution.position_ecef).norm();
                    if (std::isfinite(float_vs_spp) && float_vs_spp > kDefaultFloatVsSppGuardMeters) {
                        spp_solution.status = libgnss::SolutionStatus::SPP;
                        const double float_jump = jump_from_last_output(pos_solution);
                        const double spp_jump = jump_from_last_output(spp_solution);
                        const double jump_guard = max_nonfixed_jump();
                        const bool spp_is_plausible =
                            std::isfinite(spp_jump) &&
                            spp_jump <= jump_guard &&
                            (!std::isfinite(float_jump) || spp_jump + 3.0 < float_jump);
                        if (spp_is_plausible) {
                            pos_solution = spp_solution;
                        }
                    }
                }
            }

            if (!availability_fallback_used &&
                pos_solution.isValid() &&
                pos_solution.status != libgnss::SolutionStatus::FIXED) {
                const double candidate_jump = jump_from_last_output(pos_solution);
                if (std::isfinite(candidate_jump) && candidate_jump > max_nonfixed_jump()) {
                    const auto rejected = pos_solution;
                    if (!replace_rejected_with_continuity(rejected)) {
                        pos_solution = libgnss::PositionSolution{};
                        pos_solution.time = rover_obs.time;
                        pos_solution.status = libgnss::SolutionStatus::NONE;
                    }
                }
            }

            if (!availability_fallback_used &&
                pos_solution.isValid() &&
                pos_solution.status != libgnss::SolutionStatus::FIXED &&
                !immediate_guard_continuity_used &&
                have_last_fixed_output) {
                double dt_since_fixed = pos_solution.time - last_fixed_output.time;
                if (std::isfinite(dt_since_fixed) && dt_since_fixed > 0.0 && dt_since_fixed <= 5.0) {
                    const double drift_from_fixed =
                        (pos_solution.position_ecef - last_fixed_output.position_ecef).norm();
                    const double height_from_fixed = std::abs(
                        pos_solution.position_geodetic.height -
                        last_fixed_output.position_geodetic.height);
                    const double max_fixed_drift = std::max(12.0, 15.0 * dt_since_fixed);
                    const double max_height_drift = std::max(6.0, 3.0 * dt_since_fixed);
                    if (drift_from_fixed > max_fixed_drift ||
                        height_from_fixed > max_height_drift) {
                        const auto rejected = pos_solution;
                        if (!replace_rejected_with_continuity(rejected)) {
                            pos_solution =
                                libgnss::PositionSolution{};
                            pos_solution.time = rover_obs.time;
                            pos_solution.status =
                                libgnss::SolutionStatus::NONE;
                        }
                    }
                }
            }

            // A solver result can be valid at the raw boundary and then be
            // rejected by the jump/height guards above. Preserve the same
            // no-FIX availability contract at this final boundary as well.
            if (
                !pos_solution.isValid() &&
                config.enable_safe_availability_fallback) {
                Eigen::Vector3d anchor_position =
                    Eigen::Vector3d::Constant(
                        std::numeric_limits<double>::quiet_NaN());
                double dt = 0.2;
                double anchor_variance_m2 = 1e6;
                if (
                    last_output != nullptr &&
                    last_output->position_ecef.allFinite()) {
                    anchor_position = last_output->position_ecef;
                    const double candidate_dt =
                        rover_obs.time - last_output->time;
                    if (
                        std::isfinite(candidate_dt) &&
                        candidate_dt > 0.0) {
                        dt = candidate_dt;
                    }
                    const double candidate_variance =
                        last_output->position_covariance.trace() /
                        3.0;
                    if (
                        std::isfinite(candidate_variance) &&
                        candidate_variance >= 25.0) {
                        anchor_variance_m2 =
                            candidate_variance;
                    }
                } else if (
                    rover_obs.receiver_position.allFinite() &&
                    rover_obs.receiver_position.norm() > 1e6) {
                    anchor_position =
                        rover_obs.receiver_position;
                }
                if (anchor_position.allFinite()) {
                    pos_solution =
                        libgnss::PositionSolution{};
                    pos_solution.time = rover_obs.time;
                    pos_solution.status =
                        libgnss::SolutionStatus::PROPAGATED;
                    pos_solution.position_ecef =
                        anchor_position;
                    pos_solution.position_geodetic =
                        libgnss::spp_utils::ecefToGeodetic(
                            anchor_position);
                    const double growth_sigma_m = 25.0 * dt;
                    pos_solution.position_covariance =
                        Eigen::Matrix3d::Identity() *
                        (anchor_variance_m2 +
                         growth_sigma_m * growth_sigma_m);
                    pos_solution.velocity_covariance =
                        Eigen::Matrix3d::Identity() * 625.0;
                    pos_solution.has_velocity = false;
                    pos_solution.num_satellites = 0;
                    pos_solution.ratio = 0.0;
                    pos_solution.num_fixed_ambiguities = 0;
                    availability_fallback_used = true;
                    ++availability_propagated_fallback_epochs;
                }
            }

            const auto feedback_solution = pos_solution;
            rtk_processor.applyLibraryFixedQualityGate(pos_solution);
            if (shouldDemoteFixedStatus(config, pos_solution)) {
                pos_solution.status = libgnss::SolutionStatus::FLOAT;
            }

            auto gated_feedback_solution = feedback_solution;
            bool integrity_reset_requested = false;
            if (integrity_gate) {
                libgnss::RealtimeFixIntegrityGate::EpochInput integrity_input;
                integrity_input.primary = pos_solution;
                if (integrity_shadow) {
                    integrity_input.independent = integrity_shadow->lookup(pos_solution.time);
                }
                auto integrity_update = integrity_gate->push(std::move(integrity_input));
                if (integrity_update.current.output_demoted) {
                    pos_solution.status = libgnss::SolutionStatus::FLOAT;
                    if (gated_feedback_solution.isFixed()) {
                        gated_feedback_solution.status = libgnss::SolutionStatus::FLOAT;
                    }
                }
                integrity_reset_requested =
                    integrity_update.current.consensus.request_primary_reset ||
                    integrity_update.current.residual_streak_demoted ||
                    integrity_update.current.residual_spike_demoted ||
                    integrity_update.current.base_confidence_demoted;
                record_integrity_emissions(integrity_update.emitted);
            }

            debug_writer.write(pos_solution, rtk_processor.getLastDebugTelemetry());

            if (diagnostics_csv.is_open()) {
                EpochDiagnostics diag;
                diag.epoch_index = static_cast<int>(processed_rover_epochs);
                diag.gps_week = rover_obs.time.week;
                diag.tow = rover_obs.time.tow;
                diag.exact_base = !used_interpolated_base;
                diag.interpolated_base = used_interpolated_base;
                fillSolutionDiagnostics(pos_solution,
                    diag.initial_valid, diag.initial_status, diag.initial_sats,
                    diag.initial_ratio, diag.initial_pdop, diag.initial_baseline_m,
                    diag.initial_residual_rms, diag.initial_residual_abs_max,
                    diag.initial_update_rows, diag.initial_suppressed_outliers);
                fillSolutionDiagnostics(pos_solution,
                    diag.final_valid, diag.final_status, diag.final_sats,
                    diag.final_ratio, diag.final_pdop, diag.final_baseline_m,
                    diag.final_residual_rms, diag.final_residual_abs_max,
                    diag.final_update_rows, diag.final_suppressed_outliers);
                diag.output_added = pos_solution.isValid();
                writeDiagnosticsRow(diagnostics_csv, diag);
            }

            if (integrity_reset_requested) {
                rtk_processor.reset();
                have_last_fixed_output = false;
            }

            if (!integrity_gate) {
                record_output(pos_solution);
            }
            if (pos_solution.isValid()) {
                last_guard_output = pos_solution;
                have_last_guard_output = true;
                if (gated_feedback_solution.isFixed()) {
                    last_fixed_output = gated_feedback_solution;
                if (!immediate_guard_continuity_used &&
                    !availability_fallback_used &&
                    !rtk_processor.getLastDebugTelemetry()
                         .safe_float_continuity_used) {
                    last_real_output = feedback_solution;
                    have_last_real_output =
                        feedback_solution.isValid();
                }
                    have_last_fixed_output = true;
                }
            }

            const Eigen::Vector3d saved_rover_pos = rover_obs.receiver_position;
            rover_ok = rover_reader.readObservationEpoch(rover_obs);
            if (rover_ok) {
                const bool trusted_spp_seed =
                    pos_solution.status == libgnss::SolutionStatus::SPP &&
                    pos_solution.num_satellites >= 7;
                if (gated_feedback_solution.isFixed()) {
                    rover_obs.receiver_position = gated_feedback_solution.position_ecef;
                } else if (trusted_spp_seed) {
                    rover_obs.receiver_position = pos_solution.position_ecef;
                } else {
                    rover_obs.receiver_position = saved_rover_pos;
                }
            }
            processed_rover_epochs++;
        }

        if (integrity_gate) {
            record_integrity_emissions(integrity_gate->flush());
        }
        const std::vector<libgnss::PositionSolution>
            availability_source_solutions =
                config.enable_safe_availability_fallback
                    ? solution.solutions
                    : std::vector<libgnss::PositionSolution>{};

        if (config.enable_nonfix_drift_guard &&
            rtk_config.position_mode != libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC &&
            !solution.isEmpty()) {
            libgnss::rtk_validation::NonFixedDriftGuardConfig guard_config;
            guard_config.max_anchor_gap_s = config.nonfix_drift_guard_max_anchor_gap_s;
            guard_config.max_anchor_speed_mps = config.nonfix_drift_guard_max_anchor_speed_mps;
            guard_config.max_residual_m = config.nonfix_drift_guard_max_residual_m;
            guard_config.min_horizontal_residual_m =
                config.nonfix_drift_guard_min_horizontal_residual_m;
            guard_config.min_segment_epochs = config.nonfix_drift_guard_min_segment_epochs;
            guard_config.max_segment_epochs = config.nonfix_drift_guard_max_segment_epochs;
            auto guard_result = libgnss::rtk_validation::filterNonFixedStationaryDrift(
                solution.solutions,
                guard_config);
            nonfix_drift_guard_inspected_segments = guard_result.inspected_segments;
            nonfix_drift_guard_rejected_segments = guard_result.rejected_segments;
            nonfix_drift_guard_rejected_epochs = guard_result.rejected_epochs;
            solution.solutions = std::move(guard_result.solutions);
        }

        if (config.enable_spp_height_step_guard &&
            rtk_config.position_mode != libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC &&
            !solution.isEmpty()) {
            libgnss::rtk_validation::SppHeightStepGuardConfig guard_config;
            guard_config.min_step_m = config.spp_height_step_guard_min_m;
            guard_config.max_rate_mps = config.spp_height_step_guard_max_rate_mps;
            auto guard_result = libgnss::rtk_validation::filterSppHeightSteps(
                solution.solutions,
                guard_config);
            spp_height_step_guard_rejected_epochs = guard_result.rejected_epochs;
            solution.solutions = std::move(guard_result.solutions);
        }

        if (config.enable_float_bridge_tail_guard &&
            rtk_config.position_mode != libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC &&
            !solution.isEmpty()) {
            libgnss::rtk_validation::FloatBridgeTailGuardConfig guard_config;
            guard_config.max_anchor_gap_s = config.float_bridge_tail_guard_max_anchor_gap_s;
            guard_config.min_anchor_speed_mps =
                config.float_bridge_tail_guard_min_anchor_speed_mps;
            guard_config.max_anchor_speed_mps =
                config.float_bridge_tail_guard_max_anchor_speed_mps;
            guard_config.max_residual_m = config.float_bridge_tail_guard_max_residual_m;
            guard_config.min_segment_epochs =
                config.float_bridge_tail_guard_min_segment_epochs;
            auto guard_result = libgnss::rtk_validation::filterFloatBridgeTail(
                solution.solutions,
                guard_config);
            float_bridge_tail_guard_inspected_segments = guard_result.inspected_segments;
            float_bridge_tail_guard_rejected_segments = guard_result.rejected_segments;
            float_bridge_tail_guard_rejected_epochs = guard_result.rejected_epochs;
            solution.solutions = std::move(guard_result.solutions);
        }

        if (config.enable_fixed_bridge_burst_guard &&
            rtk_config.position_mode != libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC &&
            !solution.isEmpty()) {
            libgnss::rtk_validation::FixedBridgeBurstGuardConfig guard_config;
            guard_config.max_anchor_gap_s = config.fixed_bridge_burst_guard_max_anchor_gap_s;
            guard_config.min_boundary_gap_s = config.fixed_bridge_burst_guard_min_boundary_gap_s;
            guard_config.max_residual_m = config.fixed_bridge_burst_guard_max_residual_m;
            guard_config.max_segment_epochs = config.fixed_bridge_burst_guard_max_segment_epochs;
            auto guard_result = libgnss::rtk_validation::filterFixedBridgeBursts(
                solution.solutions,
                guard_config);
            fixed_bridge_burst_guard_inspected_segments = guard_result.inspected_segments;
            fixed_bridge_burst_guard_rejected_segments = guard_result.rejected_segments;
            fixed_bridge_burst_guard_rejected_epochs = guard_result.rejected_epochs;
            solution.solutions = std::move(guard_result.solutions);
        }

        if (config.enable_kinematic_post_filter &&
            rtk_config.position_mode != libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC &&
            !solution.isEmpty()) {
            // Single-epoch drop on height-step jumps. The previous "suppress
            // until N consecutive FIXED" cascade was removed because, with
            // PR #34's state-restore in validateFixedSolution and PR #35's
            // --max-pos-jump=5.0 default, wrong-FIX is already rejected at
            // the AR layer. A wrong-FIX occasionally still slipped past and
            // became `last_kept`, after which the cascade silently dropped
            // many subsequent valid epochs (truth validation: 92.1% -> 47.8%
            // coverage). Drop only the offending epoch and leave `last_kept`
            // pointing at the prior trusted anchor.
            libgnss::Solution filtered_solution;
            filtered_solution.solutions.reserve(solution.solutions.size());
            bool have_kept_fixed = false;
            const libgnss::PositionSolution* last_kept = nullptr;
            for (const auto& epoch_solution : solution.solutions) {
                if (!epoch_solution.isValid()) {
                    continue;
                }

                if (last_kept != nullptr && have_kept_fixed) {
                    double dt = epoch_solution.time - last_kept->time;
                    if (!std::isfinite(dt) || dt <= 0.0) {
                        dt = 1.0;
                    }
                    const double height_step = std::abs(
                        epoch_solution.position_geodetic.height -
                        last_kept->position_geodetic.height);
                    const double max_height_step =
                        std::max(kDefaultVerticalStepGuardMeters, 4.0 * dt);
                    if (height_step > max_height_step) {
                        const auto continuity =
                            libgnss::safe_float_continuity::propagate(
                                rtk_config.safe_float_continuity,
                                last_kept->position_ecef,
                                dt,
                                last_kept->velocity_ecef,
                                dt);
                        if (
                            last_kept->has_velocity &&
                            continuity.valid) {
                            auto replacement = *last_kept;
                            replacement.time = epoch_solution.time;
                            replacement.status =
                                libgnss::SolutionStatus::FLOAT;
                            replacement.position_ecef =
                                continuity.position_ecef;
                            replacement.position_geodetic =
                                libgnss::spp_utils::ecefToGeodetic(
                                    replacement.position_ecef);
                            replacement.position_covariance =
                                Eigen::Matrix3d::Identity() *
                                continuity.position_variance_m2;
                            replacement.num_satellites = 4;
                            replacement.ratio = 0.0;
                            replacement.num_fixed_ambiguities = 0;
                            filtered_solution.addSolution(replacement);
                            ++kinematic_postfilter_continuity_epochs;
                        }
                        // Never advance the trusted post-filter anchor with a
                        // propagated replacement.
                        continue;
                    }
                }

                filtered_solution.addSolution(epoch_solution);
                last_kept = &filtered_solution.solutions.back();
                if (epoch_solution.isFixed()) {
                    have_kept_fixed = true;
                }
            }
            solution = std::move(filtered_solution);
        }

        if (
            config.enable_safe_availability_fallback &&
            !availability_source_solutions.empty()) {
            const auto epoch_key =
                [](const libgnss::PositionSolution& epoch_solution) {
                    return std::make_pair(
                        epoch_solution.time.week,
                        static_cast<long long>(std::llround(
                            epoch_solution.time.tow * 1000.0)));
                };
            std::set<std::pair<int, long long>> kept_epochs;
            for (const auto& epoch_solution : solution.solutions) {
                kept_epochs.insert(epoch_key(epoch_solution));
            }
            for (const auto& source : availability_source_solutions) {
                if (kept_epochs.count(epoch_key(source)) != 0) {
                    continue;
                }
                auto degraded = source;
                degraded.status =
                    source.status ==
                            libgnss::SolutionStatus::PROPAGATED
                        ? libgnss::SolutionStatus::PROPAGATED
                        : libgnss::SolutionStatus::SPP;
                degraded.position_covariance +=
                    Eigen::Matrix3d::Identity() * 10000.0;
                degraded.ratio = 0.0;
                degraded.num_fixed_ambiguities = 0;
                if (
                    degraded.status ==
                    libgnss::SolutionStatus::PROPAGATED) {
                    degraded.num_satellites = 0;
                } else {
                    degraded.num_satellites =
                        std::max(4, degraded.num_satellites);
                }
                solution.addSolution(degraded);
                kept_epochs.insert(epoch_key(degraded));
                ++availability_postfilter_reinserted_epochs;
            }
            std::sort(
                solution.solutions.begin(),
                solution.solutions.end(),
                [](const libgnss::PositionSolution& left,
                   const libgnss::PositionSolution& right) {
                    if (left.time.week != right.time.week) {
                        return left.time.week < right.time.week;
                    }
                    return left.time.tow < right.time.tow;
                });
        }

        if (solution.isEmpty()) {
            std::cerr << "Error: no valid solutions were produced" << std::endl;
            return 1;
        }
        if (!writeSolutions(config, solution)) {
            return 1;
        }

        Eigen::Vector3d mean_pos = Eigen::Vector3d::Zero();
        int mean_count = 0;
        for (const auto& epoch_solution : solution.solutions) {
            if (epoch_solution.isValid()) {
                mean_pos += epoch_solution.position_ecef;
                mean_count++;
            }
        }
        if (mean_count > 0) {
            mean_pos /= static_cast<double>(mean_count);
        }

        const auto stats = solution.calculateStatistics(mean_pos);
        std::cout << "\nSummary" << std::endl;
        std::cout << "  total solutions: " << solution.size() << std::endl;
        std::cout << "  valid solutions: " << stats.valid_solutions << std::endl;
        std::cout << "  fixed solutions: " << stats.fixed_solutions << std::endl;
        std::cout << "  fix rate: " << std::fixed << std::setprecision(2)
                  << stats.fix_rate * 100.0 << "%" << std::endl;
        if (rtk_config.position_mode == libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC) {
            std::cout << "  RMS horizontal (self-consistency): "
                      << stats.rms_horizontal << " m" << std::endl;
            std::cout << "  RMS vertical (self-consistency): "
                      << stats.rms_vertical << " m" << std::endl;
        } else {
            std::cout << "  self-consistency metrics: omitted in dynamic mode; "
                      << "use reference-based comparison for accuracy." << std::endl;
        }
        std::cout << "  exact base epochs: " << exact_base_epochs << std::endl;
        std::cout << "  interpolated base epochs: " << interpolated_base_epochs << std::endl;
        std::cout << "  skipped rover epochs: " << skipped_rover_epochs << std::endl;
        if (config.cmc_aware_reference_selection) {
            const auto cmc_ref_diag = rtk_processor.getCmcReferenceDiagnostics();
            std::cout << "  CMC-aware reference selection: enabled"
                      << " suspect_epochs=" << cmc_ref_diag.suspect_epoch_count
                      << " switches=" << cmc_ref_diag.switch_count << std::endl;
        }
        if (rtk_config.position_mode != libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC) {
            std::cout << "  non-FIX drift guard: "
                      << (config.enable_nonfix_drift_guard ? "enabled" : "disabled")
                      << " inspected_segments=" << nonfix_drift_guard_inspected_segments
                      << " rejected_segments=" << nonfix_drift_guard_rejected_segments
                      << " rejected_epochs=" << nonfix_drift_guard_rejected_epochs
                      << std::endl;
            std::cout << "  SPP height-step guard: "
                      << (config.enable_spp_height_step_guard ? "enabled" : "disabled")
                      << " rejected_epochs=" << spp_height_step_guard_rejected_epochs
                      << std::endl;
            std::cout << "  FLOAT bridge-tail guard: "
                      << (config.enable_float_bridge_tail_guard ? "enabled" : "disabled")
                      << " inspected_segments=" << float_bridge_tail_guard_inspected_segments
                      << " rejected_segments=" << float_bridge_tail_guard_rejected_segments
                      << " rejected_epochs=" << float_bridge_tail_guard_rejected_epochs
                      << std::endl;
        std::cout << "  fixed bridge-burst guard: "
                      << (config.enable_fixed_bridge_burst_guard ? "enabled" : "disabled")
                      << " inspected_segments=" << fixed_bridge_burst_guard_inspected_segments
                      << " rejected_segments=" << fixed_bridge_burst_guard_rejected_segments
                  << " rejected_epochs=" << fixed_bridge_burst_guard_rejected_epochs
                  << std::endl;
        std::cout << "  kinematic post-filter FLOAT continuity epochs: "
                  << kinematic_postfilter_continuity_epochs << std::endl;
        std::cout << "  immediate guard FLOAT continuity epochs: "
                  << immediate_guard_continuity_epochs << std::endl;
        std::cout << "  safe availability SPP fallback epochs: "
                  << availability_spp_fallback_epochs << std::endl;
        std::cout << "  safe availability propagated epochs: "
                  << availability_propagated_fallback_epochs
                  << std::endl;
        std::cout << "  safe availability post-filter reinsertions: "
                  << availability_postfilter_reinserted_epochs
                  << std::endl;
        }
        if (rtk_config.position_mode == libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC &&
            rover_header.approximate_position.norm() > 0.0 && mean_count > 0) {
            std::cout << "  header vs mean diff: "
                      << (mean_pos - rover_header.approximate_position).norm() << " m" << std::endl;
        }
        if (integrity_shadow) {
            const auto lookups = integrity_shadow->lookups();
            const auto healthy = integrity_shadow->healthyLookups();
            std::cout << "  integrity shadow health: " << healthy << '/' << lookups;
            if (lookups > 0) {
                std::cout << " (" << std::fixed << std::setprecision(2)
                          << (100.0 * static_cast<double>(healthy) /
                              static_cast<double>(lookups))
                          << "% qualified as demotion authority)";
            }
            std::cout << std::endl;
        }
        if (multisd_shadow_csv.is_open()) {
            std::cout << "  GNSS-only MultiSD shadow: attempts="
                      << multisd_shadow_attempts
                      << " valid=" << multisd_shadow_valid
                      << " fixed=" << multisd_shadow_fixed;
            if (multisd_shadow_attempts > 0) {
                std::cout << " mean_runtime_ms="
                          << (multisd_shadow_total_runtime_ms /
                              static_cast<double>(multisd_shadow_attempts));
            }
            std::cout << " csv=" << config.multisd_fgo_shadow_csv_path
                      << std::endl;
        }
        std::cout << "  output written: " << config.output_pos_path << std::endl;
        if (config.write_kml) {
            std::cout << "  KML written: " << config.output_kml_path << std::endl;
        }

        return 0;
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error (invalid_argument): " << e.what() << std::endl;
        return 1;
    } catch (const std::out_of_range& e) {
        std::cerr << "Error (out_of_range): " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
