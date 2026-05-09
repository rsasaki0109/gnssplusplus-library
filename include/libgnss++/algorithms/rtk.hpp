#pragma once

#include "../core/processor.hpp"
#include "../core/observation.hpp"
#include "../core/navigation.hpp"
#include "../core/solution.hpp"
#include "rtk_measurement.hpp"
#include "rtk_selection.hpp"
#include "rtk_slip_detection.hpp"
#include "rtk_validation.hpp"
#include "spp.hpp"
#include <Eigen/Dense>
#include <limits>
#include <mutex>
#include <set>
#include <string>

namespace libgnss {

/**
 * @brief Real-Time Kinematic (RTK) processor
 *
 * Implements RTK positioning using carrier phase measurements
 * with ambiguity resolution for centimeter-level accuracy.
 *
 * State vector: [baseline(3), N1_SD_sat1, N1_SD_sat2, ..., N2_SD_sat1, ...]
 * SD (single-difference) ambiguities per satellite.
 * DD formed only in the observation model (H matrix maps SD states).
 * RTKLIB-compatible approach.
 */
class RTKProcessor : public ProcessorBase {
public:
    struct RTKConfig {
        double max_baseline_length = 50000.0;
        double min_baseline_length = 0.1;

        enum class AmbiguityResolutionMode {
            OFF = 0,
            CONTINUOUS = 1,
            INSTANTANEOUS = 2,
            PARTIAL = 3
        };
        enum class GlonassARMode {
            OFF = 0,
            ON = 1,
            AUTOCAL = 2
        };
        AmbiguityResolutionMode ar_mode = AmbiguityResolutionMode::CONTINUOUS;
        GlonassARMode glonass_ar_mode = GlonassARMode::OFF;

        double ratio_threshold = 3.0;
        double ambiguity_ratio_threshold = 3.0;
        double validation_threshold = 0.15;
        bool enable_ar_filter = false;
        double ar_filter_margin = 0.25;
        int min_satellites_for_ar = 5;
        int min_lock_count = 5;
        int min_hold_count = 5;   // RTKLIB minfix: consecutive fixes before holdamb
        double hold_ambiguity_ratio_threshold = 2.0;

        // Ionosphere option
        enum class IonoOpt {
            OFF = 0,       // no ionosphere correction (L1/L2 separate)
            IFLC = 1,      // ionosphere-free linear combination
            EST = 2        // estimate DD ionosphere states in the KF
        };
        IonoOpt ionoopt = IonoOpt::OFF;

        // Position mode
        enum class PositionMode {
            STATIC = 0,    // position constant (add process noise)
            KINEMATIC = 1, // position reset each epoch (RTKLIB kinematic)
            MOVING_BASE = 2 // kinematic relative baseline against a time-varying base position
        };
        PositionMode position_mode = PositionMode::KINEMATIC;

        // Initial-position seed sources (default off preserves legacy SPP-only seed).
        // When prefer_trusted_position_seed is set, kinematic re-seeds prefer the
        // most recent trusted/fixed solution (within a 1s window) over an SPP fix.
        // When prefer_rover_position_seed is set, the rover RINEX header position
        // (if magnitude > 1e6 m, i.e. real ECEF) is used before SPP.
        bool prefer_trusted_position_seed = false;
        bool prefer_rover_position_seed = false;

        // Kalman filter parameters
        double process_noise_position = 1e-4;       // m^2/s for static baseline
        double process_noise_ambiguity = 1e-8;    // cycles^2/s (very small - ambiguities are constant)
        double process_noise_iono = 1e-4;         // m^2/s for DD ionosphere states
        int kf_iterations = 4;                       // more iterations for position convergence

        // Measurement noise
        double pseudorange_sigma = 0.3;           // m (eratio = pseudorange_sigma / carrier_phase_sigma = 150)
        double carrier_phase_sigma = 0.002;       // m (instrument noise)
        bool enable_snr_weighting = false;        // Opt-in C/N0-based variance inflation
        double snr_reference_dbhz = 45.0;         // No inflation at or above this SNR
        double snr_max_variance_scale = 25.0;     // Clamp low-SNR variance inflation
        double snr_min_baseline_m = 0.0;          // Optional baseline-length floor for SNR weighting

        // Quality control
        bool enable_cycle_slip_detection = true;
        double cycle_slip_threshold = 0.05;
        bool enable_doppler_slip_detection = true;
        double doppler_slip_threshold = 0.20;
        bool enable_code_slip_detection = true;
        double code_slip_threshold = 5.0;
        bool use_dynamic_slip_threshold_floor = true;
        bool enable_adaptive_dynamic_slip_thresholds = false;
        int adaptive_dynamic_slip_nonfix_count = 3;
        int adaptive_dynamic_slip_hold_epochs = 10;
        bool enable_outlier_detection = true;
        double outlier_threshold = 3.0;

        // Elevation mask
        double elevation_mask = 15.0 * M_PI / 180.0;  // 15 degrees (RTKLIB default)

        // Incremental multi-GNSS rollout switches.
        bool enable_glonass = true;
        bool enable_beidou = true;
        double glonass_icb_l1_m_per_mhz = 0.0;
        double glonass_icb_l2_m_per_mhz = 0.0;

        // Phase 18 Step 3: enable L5 measurement collection.
        // When false (default), collectSatelliteData populates only L1/L2 — L5 obs are ignored.
        // When true, L5-class obs (GPS_L5/QZS_L5/GAL_E5A/BDS_B2A) are populated into
        // SatelliteData.l5_* fields. Step 4+ will add residual/Jacobian/cycle-slip handling.
        bool enable_l5 = false;

        /// AR policy gate.
        /// EXTENDED (default): all hold/subset/fallback/regularization extras active.
        /// DEMO5_CONTINUOUS:   demo5-equivalent simple AR — no relaxed hold ratio,
        ///                     no subset/partial AR fallback, no hold-fix fallback,
        ///                     no Q regularization (raw Q passed to LAMBDA).
        enum class ARPolicy { EXTENDED, DEMO5_CONTINUOUS };
        ARPolicy ar_policy = ARPolicy::EXTENDED;

        /// Minimum DD ambiguity pairs allowed for subset/partial AR candidates.
        /// 4 (default) preserves the existing partial AR search behavior.
        int min_subset_pairs_for_ar = 4;

        /// Maximum number of worst-variance DD pairs to drop while searching
        /// progressive subset AR candidates. 6 preserves existing behavior.
        int max_subset_drop_steps_for_ar = 6;

        /// Minimum distinct non-reference satellites required for subset AR.
        /// 0 (default) disables the gate and preserves existing subset behavior.
        int min_subset_sats_for_ar = 0;

        /// Minimum distinct constellations required for subset AR.
        /// 0 (default) disables the gate and preserves existing subset behavior.
        int min_subset_systems_for_ar = 0;

        /// Minimum distinct frequencies required for subset AR.
        /// 0 (default) disables the gate and preserves existing subset behavior.
        int min_subset_frequencies_for_ar = 0;

        /// Minimum non-reference satellites represented on at least two frequencies.
        /// 0 (default) disables the gate and preserves existing subset behavior.
        int min_subset_dual_frequency_sats_for_ar = 0;

        /// Minimum full-set LAMBDA ratio required before accepting subset AR.
        /// 0 (default) disables the gate and preserves existing subset behavior.
        double min_full_ratio_for_subset_ar = 0.0;

        /// Max hold fix divergence from float baseline in meters.
        /// 0 (default) disables the check — existing behavior preserved.
        double max_hold_divergence_m = 0.0;

        /// Max AR fix position jump from last fixed position in meters.
        /// Applied in addition to the history-based exceedsFixHistoryJump check.
        /// Default 5.0m: rejects wrong-FIX with implausible position jumps.
        /// Set to 0 to disable.
        double max_position_jump_m = 5.0;

        /// Adaptive max AR fix jump from last fixed position.
        /// When max_position_jump_rate_mps > 0, accepted jump is
        /// max(max_position_jump_min_m, max_position_jump_rate_mps * dt).
        /// Disabled by default to preserve existing behavior.
        double max_position_jump_min_m = 0.0;
        double max_position_jump_rate_mps = 0.0;

        /// Max FLOAT solution divergence from same-epoch SPP in meters.
        /// When > 0, FLOAT epochs farther than this from the current SPP
        /// solution fall back to SPP/no-solution. 0 (default) disables the
        /// diagnostic gate and preserves existing behavior.
        double max_float_spp_divergence_m = 0.0;

        /// Max accepted FLOAT prefit DD residual RMS in meters.
        /// When > 0, high-residual FLOAT epochs are still reported as FLOAT,
        /// but ambiguity states are reset for the next epoch's reacquisition.
        /// 0 (default) disables the diagnostic gate.
        double max_float_prefit_residual_rms_m = 0.0;

        /// Max accepted FLOAT prefit DD residual magnitude in meters.
        /// When > 0, high-residual FLOAT epochs are still reported as FLOAT,
        /// but ambiguity states are reset for the next epoch's reacquisition.
        /// 0 (default) disables the diagnostic gate.
        double max_float_prefit_residual_max_m = 0.0;

        /// Number of consecutive high-residual FLOAT epochs before resetting
        /// ambiguity states. Used only when a FLOAT prefit residual threshold
        /// is enabled. Defaults to 3 to avoid reacting to isolated multipath
        /// residual spikes.
        int max_float_prefit_residual_reset_streak = 3;

        /// Minimum FLOAT position jump from the last trusted FIX/FLOAT state
        /// before a high-prefit-residual reset is allowed. 0 (default)
        /// preserves the residual-only behavior when the residual gate is
        /// enabled.
        double min_float_prefit_residual_trusted_jump_m = 0.0;

        /// Reject a whole RTK DD Kalman update when normalized innovation
        /// squared divided by active observations exceeds this threshold.
        /// 0 (default) disables the update gate.
        double max_update_nis_per_observation = 0.0;

        /// Reject only FIXED ambiguity candidates when the preceding RTK DD
        /// update NIS divided by active observations exceeds this threshold.
        /// FLOAT output is retained; 0 (default) disables the fixed-only gate.
        double max_fixed_update_nis_per_observation = 0.0;

        /// Reject only FIXED ambiguity candidates when the preceding RTK DD
        /// update post-suppression residual RMS exceeds this threshold.
        /// FLOAT output is retained; 0 (default) disables the fixed-only gate.
        double max_fixed_update_post_residual_rms_m = 0.0;

        /// Apply fixed-update diagnostic gates only to low-ratio FIX candidates.
        /// When > 0, max_fixed_update_nis_per_observation and
        /// max_fixed_update_post_residual_rms_m are checked only if the AR ratio
        /// is finite and <= this threshold. 0 (default) preserves legacy behavior.
        double max_fixed_update_gate_ratio = 0.0;

        /// Apply fixed-update diagnostic gates only inside an optional baseline
        /// length window. 0 (default) disables each side of the window.
        double min_fixed_update_gate_baseline_m = 0.0;
        double max_fixed_update_gate_baseline_m = 0.0;

        /// Apply fixed-update diagnostic gates only inside an optional speed
        /// window, estimated from the previous accepted solution to the current
        /// fixed candidate. 0 (default) disables each side of the window.
        double min_fixed_update_gate_speed_mps = 0.0;
        double max_fixed_update_gate_speed_mps = 0.0;

        /// Optional secondary fixed-update gate window. When any secondary
        /// window field is enabled, fixed-update diagnostic gates apply if
        /// either the primary window above or this secondary window passes.
        double max_fixed_update_secondary_gate_ratio = 0.0;
        double min_fixed_update_secondary_gate_baseline_m = 0.0;
        double max_fixed_update_secondary_gate_baseline_m = 0.0;
        double min_fixed_update_secondary_gate_speed_mps = 0.0;
        double max_fixed_update_secondary_gate_speed_mps = 0.0;

        /// Reset ambiguity state after N consecutive float epochs (aggressive reconvergence).
        /// 0 (default) disables the check — existing behavior preserved.
        int max_consecutive_float_for_reset = 0;

        /// Reset ambiguity state after N consecutive non-FIX epochs.
        /// Counts FLOAT, SPP fallback, and no-solution epochs, so urban dropouts
        /// can force a clean ambiguity reacquisition even when FLOAT streaks are
        /// broken by fallback epochs. 0 (default) disables the check.
        int max_consecutive_nonfix_for_reset = 0;

        /// Max L1 post-fix phase residual RMS in meters.
        /// Computed after the LAMBDA fix is obtained, using the fixed DD ambiguities.
        /// 0 (default) disables the check — existing behavior preserved.
        double max_postfix_residual_rms = 0.0;

        /// Enable MW wide-lane AR pre-step.
        /// When true, fix wide-lane integers per L1/L2 pair before the N1/N2 LAMBDA
        /// search and inject the fixed WL integers as hard constraints into the
        /// search problem (head_state, dd_float, Qb, Qab).
        /// false (default) disables — existing behavior preserved.
        bool enable_wide_lane_ar = false;

        /// Threshold for accepting a wide-lane float as an integer (cycles).
        /// |WL_float - round(WL_float)| < threshold → fix. 0.25 is the worktree
        /// default; only referenced when enable_wide_lane_ar = true.
        double wide_lane_acceptance_threshold = 0.25;

        /// Enable MW wide-lane / narrow-lane fallback after ordinary LAMBDA
        /// fails for non-IFLC runs. IFLC keeps its historical fallback path.
        /// This reuses wide_lane_acceptance_threshold for MW and NL acceptance.
        bool enable_wlnl_fallback = false;

        /// Enable BSR-guided partial AR decimation alongside the existing
        /// variance-based progressive drop subsets. BSR-guided drops use a
        /// self-adjoint eigendecomposition of the ambiguity covariance Qb
        /// to identify pairs loading on the least-informative directions
        /// (largest eigenvalues, lowest integer-rounding success rate per
        /// Teunissen 1998), and drops them greedily.
        /// false (default) preserves existing partial AR behavior.
        bool enable_bsr_guided_decimation = false;

        /// Number of largest-eigenvalue Qb axes used to score per-pair
        /// loadings for BSR-guided decimation.
        int bsr_guided_worst_axes = 3;

        /// Max pairs to drop progressively in BSR-guided decimation.
        int bsr_guided_max_drop_steps = 6;
    };

    /// Reason why AR was silently skipped or failed in resolveAmbiguities().
    /// NONE means AR either succeeded or is not yet attempted this epoch.
    enum class ARSkipReason {
        NONE = 0,
        FILTER_NOT_INIT,
        ESTIMATED_IONO_MODE,
        DD_PAIRS_LT_4_BEFORE_VAR_FILTER,
        DD_PAIRS_LT_4_AFTER_VAR_FILTER,
        LAMBDA_FAILED,
        RATIO_COMPUTATION_FAILED,
    };

    /// Convert ARSkipReason to a short ASCII string suitable for CSV output.
    static const char* arSkipReasonToString(ARSkipReason reason) {
        switch (reason) {
            case ARSkipReason::NONE:                           return "none";
            case ARSkipReason::FILTER_NOT_INIT:                return "filter_not_init";
            case ARSkipReason::ESTIMATED_IONO_MODE:            return "estimated_iono_mode";
            case ARSkipReason::DD_PAIRS_LT_4_BEFORE_VAR_FILTER: return "dd_lt4_before_var";
            case ARSkipReason::DD_PAIRS_LT_4_AFTER_VAR_FILTER:  return "dd_lt4_after_var";
            case ARSkipReason::LAMBDA_FAILED:                  return "lambda_failed";
            case ARSkipReason::RATIO_COMPUTATION_FAILED:       return "ratio_computation_failed";
            default:                                           return "unknown";
        }
    }

    struct EpochDebugTelemetry {
        bool ar_attempted = false;
        int input_pair_count = 0;
        int pair_count = 0;
        double max_ambiguity_variance = std::numeric_limits<double>::quiet_NaN();
        double effective_ratio_threshold = std::numeric_limits<double>::quiet_NaN();
        int min_subset_pair_count = 0;
        double min_full_ratio_for_subset_ar = std::numeric_limits<double>::quiet_NaN();
        int subset_candidates_evaluated = 0;
        int subset_candidates_rejected_by_full_ratio = 0;
        int subset_candidates_rejected_by_diversity = 0;
        // Subset of subset_candidates_evaluated that came from the
        // BSR-guided decimation family (eigendecomposition-driven). The
        // accepted counter only ticks when this family adopts a candidate
        // that improves on the variance-based best (preferCandidate true).
        int bsr_guided_candidates_evaluated = 0;
        int bsr_guided_candidates_accepted = 0;

        int wide_lane_total = 0;
        int wide_lane_fixed = 0;
        int wide_lane_rejected = 0;
        double wide_lane_min_distance = std::numeric_limits<double>::quiet_NaN();
        double wide_lane_max_distance = std::numeric_limits<double>::quiet_NaN();

        int gf_slip_count = 0;
        int doppler_slip_l1_count = 0;
        int doppler_slip_l2_count = 0;
        int doppler_slip_l5_count = 0;  // Phase 18 Step 5
        int code_slip_l1_count = 0;
        int code_slip_l2_count = 0;
        int code_slip_l5_count = 0;  // Phase 18 Step 5
        int lli_slip_l1_count = 0;
        int lli_slip_l2_count = 0;
        int lli_slip_l5_count = 0;  // Phase 18 Step 5
        int ambiguity_reset_l1_count = 0;
        int ambiguity_reset_l2_count = 0;
        int ambiguity_reset_l5_count = 0;  // Phase 18 Step 5
        int gf_slip_l1l5_count = 0;       // Phase 18 Step 5: GF combination L1-L5
        bool adaptive_dynamic_slip_active = false;
        int consecutive_nonfix_before_bias_update = 0;
        int adaptive_dynamic_slip_hold_remaining = 0;

        bool full_lambda_solved = false;
        double full_ratio = std::numeric_limits<double>::quiet_NaN();
        bool selected_fixed = false;
        double selected_ratio = std::numeric_limits<double>::quiet_NaN();
        int selected_pair_count = 0;
        int selected_distinct_sats = 0;
        int selected_distinct_systems = 0;
        int selected_distinct_frequencies = 0;
        int selected_dual_frequency_sats = 0;
        int selected_fixed_ambiguities = 0;
        bool selected_used_subset = false;
        bool used_wlnl_fallback = false;

        bool validation_attempted = false;
        bool validation_passed = false;
        double postfix_residual_rms = std::numeric_limits<double>::quiet_NaN();
        double fixed_float_jump_m = std::numeric_limits<double>::quiet_NaN();
        bool post_validation_rejected = false;
        bool final_fixed_applied = false;
        std::string reject_reason;
        ARSkipReason ar_skip_reason{ARSkipReason::NONE};
    };

    RTKProcessor();
    explicit RTKProcessor(const RTKConfig& rtk_config);
    ~RTKProcessor() override = default;

    // ProcessorBase interface
    bool initialize(const ProcessorConfig& config) override;
    PositionSolution processEpoch(const ObservationData& rover_obs,
                                const NavigationData& nav) override;
    ProcessorStats getStats() const override;
    void reset() override;

    PositionSolution processRTKEpoch(const ObservationData& rover_obs,
                                   const ObservationData& base_obs,
                                   const NavigationData& nav);

    void setBasePosition(const Vector3d& base_position) {
        base_position_ = base_position;
        base_position_known_ = true;
    }

    void setRTKConfig(const RTKConfig& config);
    const RTKConfig& getRTKConfig() const { return rtk_config_; }
    const EpochDebugTelemetry& getLastDebugTelemetry() const { return debug_telemetry_; }

public:
    bool lambdaMethod(const VectorXd& float_ambiguities,
                     const MatrixXd& covariance_matrix,
                     VectorXd& fixed_ambiguities,
                     double& success_rate);

    static int ambiguityStateIndex(const SatelliteId& sat, int freq) { return IB(sat, freq); }
    static int ionoStateIndex(const SatelliteId& sat) { return II(sat); }

private:
    RTKConfig rtk_config_;
    SPPProcessor spp_processor_;
    EpochDebugTelemetry debug_telemetry_;

    Vector3d base_position_;
    bool base_position_known_ = false;

    // Fixed-size state: [pos(3), glo_hw_bias(2), iono(MAXSAT), N1(MAXSAT), N2(MAXSAT), N5(MAXSAT)]
    // Phase 18 Step 2 (2026-05-09): N5 freq slot reserved (IB(sat, freq=2)).
    // Currently N5 entries stay 0 / unconfirmed; populated by Steps 3+ when L5 enabled.
    // Existing L1/L2-only path is unchanged: N5 slots default to 0 with zero covariance.
    // Satellite slots are system-aware so G01/E01/Q01 do not collide.
    static constexpr int BASE_STATES = 3;
    static constexpr int GLO_HWBIAS_STATES = 2;
    static constexpr int SATS_PER_SYSTEM = 64;
    static constexpr int SUPPORTED_SYSTEM_BLOCKS = 6;
    static constexpr int MAXSAT = SATS_PER_SYSTEM * SUPPORTED_SYSTEM_BLOCKS;
    static constexpr int REAL_STATES = BASE_STATES + GLO_HWBIAS_STATES;
    static constexpr int IONO_STATES = MAXSAT;
    static constexpr int FREQ_SLOTS = 3;  // L1, L2, L5 (Phase 18 Step 2)
    static constexpr int NX = REAL_STATES + IONO_STATES + MAXSAT * FREQ_SLOTS;  // total state size

    static int systemSlotBase(GNSSSystem system) {
        switch (system) {
            case GNSSSystem::GPS: return 0 * SATS_PER_SYSTEM;
            case GNSSSystem::GLONASS: return 1 * SATS_PER_SYSTEM;
            case GNSSSystem::Galileo: return 2 * SATS_PER_SYSTEM;
            case GNSSSystem::BeiDou: return 3 * SATS_PER_SYSTEM;
            case GNSSSystem::QZSS: return 4 * SATS_PER_SYSTEM;
            case GNSSSystem::NavIC: return 5 * SATS_PER_SYSTEM;
            default: return 0;
        }
    }

    static int satelliteSlot(const SatelliteId& sat) {
        int prn = sat.prn;
        if (prn < 1) prn = 1;
        if (prn > SATS_PER_SYSTEM) prn = SATS_PER_SYSTEM;
        return systemSlotBase(sat.system) + (prn - 1);
    }

    static int IL(int freq) {
        return BASE_STATES + freq;
    }

    static int II(const SatelliteId& sat) {
        return REAL_STATES + satelliteSlot(sat);
    }

    static int IB(const SatelliteId& sat, int freq) {
        return REAL_STATES + IONO_STATES + freq * MAXSAT + satelliteSlot(sat);
    }

    struct RTKState {
        VectorXd state;
        MatrixXd covariance;
        // Legacy index maps (now use IB() directly)
        std::map<SatelliteId, int> iono_indices;
        std::map<SatelliteId, int> n1_indices;
        std::map<SatelliteId, int> n2_indices;
        // Phase 18 Step 2: N5 ambiguity index map (parallel to n1/n2_indices).
        // Populated only when L5 measurements present (Step 3+); remains empty for L1/L2-only path.
        std::map<SatelliteId, int> n5_indices;
        int next_state_idx = BASE_STATES;
    };

    RTKState filter_state_;
    bool filter_initialized_ = false;

    // Lock count per satellite per frequency (for AR eligibility)
    std::map<SatelliteId, int> lock_count_l1_;
    std::map<SatelliteId, int> lock_count_l2_;
    // Phase 18 Step 2: L5 lock count (parallel to L1/L2). Stays empty in L1/L2-only path.
    std::map<SatelliteId, int> lock_count_l5_;

    // Reference satellite tracking
    SatelliteId current_ref_satellite_;
    bool has_ref_satellite_ = false;

    // Fixed solution baseline (from LAMBDA)
    Vector3d fixed_baseline_;
    bool has_fixed_solution_ = false;

    // Last validated fixed position (for position reset in next epoch)
    Vector3d last_fixed_position_ = Vector3d::Zero();
    bool has_last_fixed_position_ = false;
    GNSSTime last_fixed_time_;
    bool has_last_fixed_time_ = false;
    Vector3d last_solution_position_ = Vector3d::Zero();
    bool has_last_solution_position_ = false;
    Vector3d last_trusted_position_ = Vector3d::Zero();
    bool has_last_trusted_position_ = false;
    Vector3d fixed_update_gate_previous_position_ = Vector3d::Zero();
    GNSSTime fixed_update_gate_previous_time_;
    bool has_fixed_update_gate_previous_solution_ = false;

    // Consecutive fix tracking for holdamb
    int consecutive_fix_count_ = 0;

    // Consecutive float tracking for ambiguity auto-reset gate
    int consecutive_float_count_ = 0;

    // Consecutive non-FIX tracking for ambiguity auto-reset gate
    int consecutive_nonfix_count_ = 0;

    // Consecutive high-residual FLOAT tracking for residual-aware reacquisition
    int consecutive_high_float_residual_count_ = 0;

    // Remaining epochs for adaptive dynamic slip thresholds after a non-FIX streak.
    int adaptive_dynamic_slip_hold_count_ = 0;

    // Last fix data for holdamb
    struct DDPair {
        SatelliteId ref_sat;
        int ref_idx;
        int sat_idx;
        SatelliteId sat;
        int freq;
    };
    struct HoldStateSnapshot {
        Vector3d last_fixed_position = Vector3d::Zero();
        bool has_last_fixed_position = false;
        GNSSTime last_fixed_time;
        bool has_last_fixed_time = false;
        std::vector<DDPair> dd_pairs;
        std::vector<int> best_subset;
        VectorXd dd_fixed;
        double ar_ratio = 0.0;
        int num_fixed_ambiguities = 0;

        bool hasHeldIntegers() const { return dd_fixed.size() > 0; }
    };
    std::vector<DDPair> last_dd_pairs_;
    std::vector<int> last_best_subset_;
    VectorXd last_dd_fixed_;
    double last_ar_ratio_ = 0.0;
    int last_num_fixed_ambiguities_ = 0;

    // Epoch tracking
    GNSSTime last_epoch_time_;
    bool has_last_epoch_ = false;
    GNSSTime last_trusted_time_;
    bool has_last_trusted_time_ = false;

    // Statistics
    mutable std::mutex stats_mutex_;
    size_t total_epochs_processed_ = 0;
    size_t fixed_solutions_ = 0;
    size_t float_solutions_ = 0;

    struct RTKUpdateDiagnostics {
        int iterations = 0;
        int observation_count = 0;
        int phase_observation_count = 0;
        int code_observation_count = 0;
        int suppressed_outliers = 0;
        double prefit_residual_rms_m = 0.0;
        double prefit_residual_max_m = 0.0;
        double post_suppression_residual_rms_m = 0.0;
        double post_suppression_residual_max_m = 0.0;
        double normalized_innovation_squared = 0.0;
        double normalized_innovation_squared_per_observation = 0.0;
        bool rejected_by_innovation_gate = false;
    };
    RTKUpdateDiagnostics current_update_diagnostics_;

    // Satellite data for current epoch
    struct SatelliteData {
        SatelliteId satellite;
        SignalType l1_signal = SignalType::GPS_L1CA;
        SignalType l2_signal = SignalType::GPS_L2C;
        SignalType l5_signal = SignalType::GPS_L5;
        double l1_wavelength = 0.0;
        double l2_wavelength = 0.0;
        double l5_wavelength = 0.0;
        double l1_frequency_hz = 0.0;
        double l2_frequency_hz = 0.0;
        double l5_frequency_hz = 0.0;
        // L1
        double rover_l1_phase = 0.0;  // cycles
        double rover_l1_code = 0.0;   // meters
        double rover_l1_doppler = 0.0; // Hz
        double rover_l1_snr = 0.0;     // dB-Hz
        double base_l1_phase = 0.0;
        double base_l1_code = 0.0;
        double base_l1_doppler = 0.0; // Hz
        double base_l1_snr = 0.0;     // dB-Hz
        bool has_l1 = false;
        bool has_l1_doppler = false;
        int l1_lli = 0;
        // L2
        double rover_l2_phase = 0.0;
        double rover_l2_code = 0.0;
        double rover_l2_doppler = 0.0;
        double rover_l2_snr = 0.0;     // dB-Hz
        double base_l2_phase = 0.0;
        double base_l2_code = 0.0;
        double base_l2_doppler = 0.0;
        double base_l2_snr = 0.0;     // dB-Hz
        bool has_l2 = false;
        bool has_l2_doppler = false;
        int l2_lli = 0;
        // L5 (Phase 18 — populated when --enable-l5 set; default off, no effect on L1/L2 path)
        double rover_l5_phase = 0.0;
        double rover_l5_code = 0.0;
        double rover_l5_doppler = 0.0;
        double rover_l5_snr = 0.0;
        double base_l5_phase = 0.0;
        double base_l5_code = 0.0;
        double base_l5_doppler = 0.0;
        double base_l5_snr = 0.0;
        bool has_l5 = false;
        bool has_l5_doppler = false;
        int l5_lli = 0;
        // Geometry
        Vector3d sat_pos;          // satellite position for rover (RTKLIB satposs from rover PR)
        Vector3d sat_pos_base;     // satellite position for base (RTKLIB satposs from base PR)
        double elevation = 0.0;       // elevation from rover position
        double base_elevation = 0.0;   // elevation from base position
        bool has_ephemeris = false;
    };

    // Current epoch satellite data (cached for use across functions)
    std::map<SatelliteId, SatelliteData> current_sat_data_;
    std::map<SatelliteId, double> gf_l1l2_history_;
    std::map<SatelliteId, double> gf_l1l5_history_;  // Phase 18 Step 5: GF L1-L5 combination
    std::map<SatelliteId, double> doppler_phase_history_l1_m_;
    std::map<SatelliteId, double> doppler_phase_history_l2_m_;
    std::map<SatelliteId, double> doppler_phase_history_l5_m_;  // Phase 18 Step 5
    std::map<SatelliteId, double> code_phase_history_l1_m_;
    std::map<SatelliteId, double> code_phase_history_l2_m_;
    std::map<SatelliteId, double> code_phase_history_l5_m_;  // Phase 18 Step 5

    /**
     * Collect all satellite data for an epoch (L1+L2 for rover and base)
     */
    std::map<SatelliteId, SatelliteData> collectSatelliteData(
        const ObservationData& rover_obs,
        const ObservationData& base_obs,
        const NavigationData& nav);

    /**
     * Select reference satellite (highest elevation with L1+L2)
     */
    SatelliteId selectReferenceSatellite(
        const std::map<SatelliteId, SatelliteData>& sat_data);

    /**
     * Handle reference satellite change: no-op for SD parameterization
     */
    void handleReferenceSatelliteChange(const SatelliteId& new_ref,
                                        const std::map<SatelliteId, SatelliteData>& sat_data);

    /**
     * Initialize filter
     */
    bool initializeFilter(const ObservationData& rover_obs,
                        const ObservationData& base_obs,
                        const NavigationData& nav);

    /**
     * Update SD biases (RTKLIB udbias): initialize new, reset slipped
     */
    void updateBias(const std::map<SatelliteId, SatelliteData>& sat_data, double dt_s);

    /**
     * Predict state (kinematic: reset position variance)
     */
    void predictState(double dt);

    /**
     * Kinematic position reset: SPP position with large variance (RTKLIB udpos)
     */
    void resetPositionToSPP(const ObservationData& rover_obs, const NavigationData& nav);

    /**
     * Reset ambiguity states after N consecutive float epochs (experimental gate).
     * No-op when max_consecutive_float_for_reset == 0 (default).
     */
    void handleConsecutiveFloatReset(const ObservationData& rover_obs,
                                     const NavigationData& nav);
    void resetAmbiguityStatesForReacquisition(const ObservationData& rover_obs,
                                              const NavigationData& nav);
    bool floatResidualExceedsReacquisitionGate() const;
    bool floatResidualTrustedJumpPassesGate(
        const PositionSolution& float_solution,
        const Vector3d& saved_last_trusted_position,
        bool saved_has_last_trusted,
        const GNSSTime& saved_last_trusted_time,
        bool saved_has_last_trusted_time) const;
    bool shouldResetAfterFloatResidualGate(
        const PositionSolution& float_solution,
        const Vector3d& saved_last_trusted_position,
        bool saved_has_last_trusted,
        const GNSSTime& saved_last_trusted_time,
        bool saved_has_last_trusted_time);
    void recordFixedEpoch();
    void recordFloatEpoch(const ObservationData& rover_obs, const NavigationData& nav);
    void recordFallbackEpoch(const ObservationData& rover_obs, const NavigationData& nav);

    /**
     * Full KF update with DD observation model mapping to SD states
     */
    bool updateFilter(const std::map<SatelliteId, SatelliteData>& sat_data);
    std::vector<rtk_measurement::MeasurementBlock> buildMeasurementBlocks(
        const std::map<SatelliteId, SatelliteData>& sat_data) const;

    // Keep old signature for compatibility
    bool updateFilter(const ObservationData& rover_obs,
                    const ObservationData& base_obs,
                    const NavigationData& nav);

    /**
     * Resolve ambiguities: SD->DD transform + LAMBDA (RTKLIB resamb_LAMBDA)
     */
    bool resolveAmbiguities();
    bool resolveAmbiguities(std::vector<DDPair> dd_pairs);

    PositionSolution generateSolution(const GNSSTime& time,
                                    SolutionStatus status,
                                    int num_satellites);
    void rememberSolution(const PositionSolution& solution);

    void updateStatistics(SolutionStatus status) const;

    /**
     * Validate fixed solution with post-fit residual check (RTKLIB valpos)
     */
    bool validateFixedSolution(const std::map<SatelliteId, SatelliteData>& sat_data,
                               const GNSSTime& current_time);

    /**
     * Hold ambiguities after consecutive fixes (RTKLIB holdamb)
     */
    void applyHoldAmbiguity();

    /**
     * Try to produce a fixed solution using held DD integers when LAMBDA fails
     */
    bool tryHoldFix(const std::map<SatelliteId, SatelliteData>& sat_data,
                    const GNSSTime& time, int n_sats, PositionSolution& solution);
    HoldStateSnapshot captureHoldState() const;
    void restoreHoldState(const HoldStateSnapshot& snapshot);

    /**
     * Increment lock counts after KF update (RTKLIB: lock++ after filter)
     */
    void incrementLockCounts(const std::map<SatelliteId, SatelliteData>& sat_data);

    /**
     * Get or create SD ambiguity state index for a satellite
     */
    int getOrCreateN1Index(const SatelliteId& sat, double initial_value);
    int getOrCreateN2Index(const SatelliteId& sat, double initial_value);
    int getOrCreateN5Index(const SatelliteId& sat, double initial_value);  // Phase 18 Step 4
    int getOrCreateIonoIndex(const SatelliteId& sat, double initial_value);

    bool selectSystemReferenceSatellite(const std::map<SatelliteId, SatelliteData>& sat_data,
                                        GNSSSystem system,
                                        int min_lock_count,
                                        SatelliteId& ref_sat) const;
    std::vector<rtk_selection::SatelliteSelectionData> buildSelectionSnapshot(
        const std::map<SatelliteId, SatelliteData>& sat_data) const;
    std::vector<DDPair> buildDoubleDifferencePairs(
        const std::map<SatelliteId, SatelliteData>& sat_data,
        int min_lock_count) const;

    /**
     * Expand state vector to accommodate new states
     */
    void expandState(int new_size);

    /**
     * Remove satellite from state
     */
    void removeSatelliteFromState(const SatelliteId& sat);
    void syncSPPConfig();
    void updateGlonassHardwareBias(double dt);

    /**
     * RTKLIB varerr: SD measurement error variance
     * var = 2.0 * (a^2 + b^2/sin^2(el))
     */
    double varerr(double elevation, bool is_phase, double snr_dbhz = 0.0) const;

    // Legacy stubs kept for interface compatibility
    struct DoubleDifference {
        SatelliteId reference_satellite;
        SatelliteId satellite;
        SignalType signal;
        double pseudorange_dd = 0.0;
        double carrier_phase_dd = 0.0;
        double geometric_range = 0.0;
        Vector3d unit_vector = Vector3d::Zero();
        double elevation = 0.0;
        double variance = 0.0;
        bool valid = false;
    };
    struct AmbiguityInfo {
        double float_value = 0.0;
        double fixed_value = 0.0;
        int lock_count = 0;
        bool is_fixed = false;
        double last_phase = 0.0;
        GNSSTime last_time;
    };
    std::map<SatelliteId, std::map<SignalType, AmbiguityInfo>> ambiguity_states_;
    std::vector<DoubleDifference> formDoubleDifferences(
        const ObservationData& rover_obs, const ObservationData& base_obs,
        const NavigationData& nav);
    void detectCycleSlips(const ObservationData& rover_obs,
                        const ObservationData& base_obs);
    bool resolveAmbiguities(int dummy);
    struct LAMBDAResult {
        VectorXd fixed_ambiguities;
        double ratio = 0.0;
        bool success = false;
    };
    LAMBDAResult solveLAMBDA(const VectorXd& float_ambiguities,
                           const MatrixXd& ambiguity_covariance);
    bool validateAmbiguityResolution(const VectorXd& fixed_ambiguities,
                                   const VectorXd& float_ambiguities,
                                   const MatrixXd& covariance,
                                   double ratio);
    void updateAmbiguityStates(const std::vector<DoubleDifference>& double_diffs);
    Vector3d calculateBaseline() const;
    VectorXd calculateResiduals(const std::vector<DoubleDifference>& measurements,
                              const Vector3d& baseline) const;
    MatrixXd formMeasurementMatrix(const std::vector<DoubleDifference>& measurements,
                                 const NavigationData& nav,
                                 const GNSSTime& time) const;
    MatrixXd calculateMeasurementWeights(const std::vector<DoubleDifference>& measurements) const;
    bool hasSufficientSatellites(const std::vector<DoubleDifference>& measurements) const;
    void resetAmbiguity(const SatelliteId& satellite, SignalType signal);
    bool applyFixedAmbiguities(const VectorXd& fixed_n1,
                               const VectorXd& fixed_n2,
                               const std::map<SatelliteId, SatelliteData>& sat_data);
    void solvePositionWithAmbiguities(const std::map<SatelliteId, SatelliteData>& sat_data);
    bool trySingleEpochAR(const std::map<SatelliteId, SatelliteData>& sat_data);
    double elevationWeight(double elevation) const;
};

namespace rtk_utils {
    double calculateIonosphereFree(double l1_measurement, double l2_measurement,
                                 double f1, double f2);
    double calculateWideLane(double l1_phase, double l2_phase,
                           double l1_range, double l2_range,
                           double f1, double f2);
    double calculateNarrowLane(double l1_phase, double l2_phase, double f1, double f2);
    MatrixXd decorrelateMatrix(const MatrixXd& covariance);
    VectorXd integerLeastSquares(const VectorXd& float_solution,
                               const MatrixXd& covariance_matrix);
    double calculateBaselineLength(const Vector3d& baseline);
    bool checkBaselineConstraints(const Vector3d& baseline,
                                double min_length, double max_length);
}

} // namespace libgnss
