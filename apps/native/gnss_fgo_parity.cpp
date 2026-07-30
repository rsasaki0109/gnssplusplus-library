// Real-data Eigen-vs-GTSAM FGO parity harness (Phase 1 validation).
//
// Loads a real tokyo/nagoya PPC run (rover.obs + base.obs + base.nav) exactly
// the way apps/native/gnss_fgo.cpp does, materializes ONE production FGOProblem via
// FGOProcessor::buildDoubleDifferenceProblem(), then runs that identical
// problem through BOTH FGOProcessor backends (FGOBackend::Eigen and
// FGOBackend::GTSAM) and reports:
//   (a) per-epoch ECEF float-solution delta (max / RMS),
//   (b) fixed-solution / fix agreement with LAMBDA enabled on both backends.
//
// This is an app (not a unit test) because it needs the real dataset files.
// It only builds when the library was configured with GTSAM (otherwise the
// GTSAM backend silently falls back to Eigen and the comparison is trivial).

#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/fusion/fusion_initialization.hpp>
#include <libgnss++/io/imu.hpp>
#include <libgnss++/io/rinex.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

struct Args {
    std::string rover_path;
    std::string base_path;
    std::string nav_path;
    int max_epochs = 0;      // 0 = all
    int start_epoch = 0;     // rover input epochs to skip before replay
    int max_iters = 0;       // 0 = use preset default
    bool no_pr_factors = false;  // pure double-difference path (clock-free)
    bool no_robust = false;      // disable Huber robust loss (convexity check)
    bool float_only = false;     // skip the LAMBDA/Marginals fix comparison
    std::string imu_path;        // milestone 2b: imu.csv -> IMU-coupled Pose3 run
    std::string ref_path;        // optional reference.csv for attitude sanity
    double fixed_lag_s = 0.0;    // milestone 2c: >0 enables the fixed-lag smoother
    bool fixed_lag_qr = false;   // rank-tolerant iSAM2 elimination
    bool use_nhc = false;        // milestone 2d
    bool use_zupt = false;       // milestone 2d
    bool use_hold = false;       // milestone 2e: fix-and-hold
    bool no_held_fix_label = false;  // keep hold priors; require fresh AR for FIX output
    double elev_mask_deg = -1.0; // milestone 2e: >0 overrides preset elevation mask
    double snr_mask_dbhz = -1.0; // milestone 2e: >0 sets an SNR mask
    bool diag = false;           // milestone 2e: Eigen-vs-GTSAM apples fix-rate + freq analysis
    bool single_freq = false;    // multi-freq control: force L1/E1/B1-only DD (single-frequency baseline)
    bool multi_freq = false;     // force multi-frequency DD on (library default is now OFF)
    bool sd_doppler = false;     // add single-difference Doppler velocity factors
    bool postfit_gate = false;
    bool adaptive_ratio = false;
    int postfit_min_n = -1;
    double postfit_rms = -1.0;
    double postfit_max_norm = -1.0;
    double postfit_chi2 = -1.0;
    bool external_dr = false;
    int external_dr_max_age = -1;
    double external_dr_chi2 = -1.0;
    double external_dr_reset_ratio = -1.0;
    bool oneband = false;        // restrict LAMBDA to one ambiguity per satellite
    bool no_oneband = false;     // MF-AR ablation: disable one-band-per-satellite LAMBDA restriction
    bool no_band_sigma = false;  // MF-AR ablation: disable per-band (secondary) sigma de-weighting
    bool no_code_align = false;  // MF hygiene ablation: disable secondary-band code alignment
    bool dd_resid = false;       // per-signal DD residuals at the reference trajectory (no solve)
    bool partial_ar = false;     // MF-AR step 2: partial AR in the fixed-lag LAMBDA
    bool integer_constrained_reoptimization = false;
    double integer_constrained_prior_sigma_cycles = -1.0;
    double integer_constrained_cost_tolerance = -1.0;
    int integer_constrained_max_iterations = 0;
    bool no_gal_ar = false;      // MF-AR step 2: exclude Galileo arcs from LAMBDA / hold
    double ratio_threshold = 0.0;  // >0: override lambda_ratio_threshold
    bool imu_ratio_aperture = false;
    double imu_ratio_relaxed = -1.0;
    double imu_ratio_float_sep = -1.0;
    double imu_ratio_pred_sep = -1.0;
    bool fixed_history_dr = false;
    int fixed_history_dr_window = 0;
    double fixed_history_dr_sep = -1.0;
    bool anchor_aided_validation = false;
    bool constellation_par = false;
    bool residual_par = false;
    bool variance_par = false;
    bool gici_par = false;          // GICI-style strict PAR + continuous-unfix reacquisition profile
    double par_max_std = -1.0;
    double hold_ratio = 0.0;       // >0: override ambiguity_hold_ratio_threshold
    int hold_min = 0;              // >0: override ambiguity_hold_min_fixed
    int min_fixed = 0;             // >0: override min_fixed_ambiguities (partial-AR floor)
    bool gates = false;            // per-epoch quality gates (reference gate.py/postfit.py port)
    double gate_res = -1.0;        // >=0: override gate_ddpr_res_max_m (0 disables)
    double gate_sat_res = -1.0;    // >=0: override gate_per_sat_res_max_m (0 disables)
    int gate_min_sat = 0;          // >0: override gate_min_satellites
    double gate_gdop = 0.0;        // >0: override gate_gdop_max
    bool cmc = false;              // Code-Minus-Carrier multipath screening (slip_detect.py port)
    bool cmc_level_pr_only = false; // sustained CMC excludes DD code, not carrier continuity
    double cmc_jump = -1.0;        // >=0: override code_minus_carrier_jump_threshold_m
    double cmc_level = -1.0;       // >=0: override code_minus_carrier_level_threshold_m
    int cmc_warmup = 0;            // >0: override code_minus_carrier_warmup_epochs
    double cmc_alpha = -1.0;       // >=0: override code_minus_carrier_baseline_alpha
    bool cmc_ref = false;          // enable cmc_aware_reference_selection (needs --cmc)
    bool cp_hold = false;                  // CP-hold / sanity FSM (validation/postfit.py + recovery.py port)
    bool cp_hold_float_recovery = false;   // keep downweighted float carrier during hold
    bool cp_hold_anchor_release = false;   // trusted DDPR anchor releases hold
    bool cp_hold_keep_imu_chain = false;   // ambiguity reset must not discard nav-state continuity
    bool selective_cp_hold = false;        // suppress only residual-implicated carrier pairs during a hold
    double cp_hold_res = -1.0;             // >=0: override cp_hold_main_residual_threshold_m
    double cp_hold_catastrophic = -1.0;    // >=0: override cp_hold_catastrophic_threshold_m
    double cp_hold_fast_worst_sat = -1.0;  // >=0: override cp_hold_fast_worst_satellite_min_m
    int cp_hold_persist = 0;               // >0: override cp_hold_persist_epochs
    int cp_hold_epochs_n = 0;              // >0: override cp_hold_epochs
    double cp_hold_release_res = -1.0;     // >=0: override cp_hold_release_threshold_m
    int cp_hold_release_n = 0;             // >0: override cp_hold_release_count
    double cp_hold_pose_replace = -1.0;    // >=0: override cp_hold_pose_replace_threshold_m
    double cp_hold_gdop = -1.0;            // >=0: override cp_hold_max_gdop
    bool exc_recovery = false;             // solve-exception recovery (recovery.py handle_solve_exception port)
    bool ddpr_anchor = false;              // DDPR-LS anchor (ls_solvers.py + anchor stages of postfit/recovery/stage.py)
    double ddpr_anchor_max_res = -1.0;     // >=0: override ddpr_anchor_max_residual_m
    double ddpr_anchor_fde = -1.0;         // >=0: override ddpr_anchor_fde_threshold_m
    int ddpr_anchor_min_n = 0;             // >0: override ddpr_anchor_min_factors
    int ddpr_anchor_boot_epochs = -1;      // >=0: override ddpr_anchor_bootstrap_epochs
    double ddpr_anchor_boot_sigma = -1.0;  // >=0: override ddpr_anchor_bootstrap_sigma_m
    int ddpr_anchor_boot_after_mass = -1;  // 0/1: override cp_hold_bootstrap_after_mass_reset
    bool fde = false;              // GICI-style FDE (validation/postfit.py apply_fde port)
    bool fde_pr_only = false;      // PPC-style: exclude pseudorange outliers, preserve carrier arcs
    bool fde_cp_quarantine = false; // keep gross CP factors but exclude their ambiguities from AR
    double fde_pr = -1.0;          // >=0: override fde_pseudorange_threshold_m
    double fde_cp = -1.0;          // >=0: override fde_carrier_threshold_m
    double fde_frac = -1.0;        // >=0: override fde_max_rejected_fraction
    int fde_iters = 0;             // >0: override fde_max_iterations
    bool sat_badness = false;          // sat-badness EWMA down-weighting (sat_quality.py port)
    double sat_badness_cp_scale = -1.0;   // >=0: override sat_badness_carrier_sigma_scale
    double sat_badness_pr_scale = -1.0;   // >=0: override sat_badness_pseudorange_sigma_scale
    double sat_badness_ddpr_thresh = -1.0; // >=0: override sat_badness_ddpr_threshold_m
    // CLAMPED-variant overrides (see FGOConfig's sat_badness_residual_clamp_m /
    // sat_badness_score_cap / sat_badness_cppr_decay for the full rationale).
    double sat_badness_clamp = -1.0;      // >=0: override sat_badness_residual_clamp_m (0 = no clamp = faithful)
    double sat_badness_cap = -1.0;        // >=0: override sat_badness_score_cap (0 = no cap = faithful)
    double sat_badness_cppr_decay = -1.0; // >=0: override sat_badness_cppr_decay (1.0 = never decays = faithful)
    bool varerr = false;           // elevation-dependent DD sigma (RTKLIB-demo5 varerr port)
    double varerr_a = -1.0;        // >=0: override elevation_sigma_err_a_m
    double varerr_b = -1.0;        // >=0: override elevation_sigma_err_b_m
    double varerr_eratio = -1.0;   // >=0: override elevation_sigma_pseudorange_ratio
    // H1: IMU noise-grade alignment vs the reference's IMU_PRESETS['tactical']
    // (config.py ~line 32). -1 = leave buildImuInput()'s current default.
    double imu_accel_noise = -1.0;  // >=0: override accel_noise_sigma [m/s^2/sqrt(Hz)]
    double imu_gyro_noise = -1.0;   // >=0: override gyro_noise_sigma [rad/s/sqrt(Hz)]
    double imu_accel_bias = -1.0;   // >=0: override accel_bias_rw_sigma [m/s^3/sqrt(Hz)]
    double imu_gyro_bias = -1.0;    // >=0: override gyro_bias_rw_sigma [rad/s^2/sqrt(Hz)]
    bool imu_preset_tactical = false;  // convenience: apply all four tactical-grade values at once
    // Reference parity: IMU integration-covariance value semantics + per-
    // epoch inflation (FGOConfig::imu_integration_covariance /
    // use_imu_integration_covariance_inflation, fgo.hpp) and gravity
    // (ImuNoiseParams::gravity_mps2, currently hardcoded 9.80665 in
    // buildImuInput() vs the reference's utils/imu.py:39 9.81).
    double integ_cov = -1.0;       // >=0: override config.imu_integration_covariance (reference imu_integ_cov; 1e-3 is the reference value)
    bool integ_cov_inflate = false;  // enable use_imu_integration_covariance_inflation
    double integ_cov_max = -1.0;   // >=0: override config.imu_integration_covariance_max (reference imu_integ_cov_max, default 0.5)
    double gravity_mps2 = -1.0;    // >=0: override problem.imu.noise.gravity_mps2 (reference 9.81; ours defaults 9.80665)
    bool fixed_err_hist = false;      // print FIXED-epoch horizontal-error histogram + worst offenders
    double partial_ar_frac = -1.0;    // >=0: override fixed_lag_partial_lambda_min_fraction (default 0.7)
    double hold_sigma = -1.0;         // >=0: override ambiguity_hold_sigma_cycles (default 1e-3 cycles)
    bool stale_pin = false;           // M1: enable use_stale_pin_invalidation
    double stale_pin_res = -1.0;      // >=0: override stale_pin_per_sat_residual_m (default 2.0)
    int stale_pin_age = -1;           // >=0: override stale_pin_min_hold_age_epochs (default 0)
    bool fix_demote = false;          // M2: enable use_fix_plausibility_demotion
    double fix_demote_dist = -1.0;    // >=0: override fix_demote_distance_m (default 5.0)
    bool fix_demote_anchor = false;   // M2b: enable fix_demote_use_ddpr_anchor (needs --ddpr-anchor)
    double fix_demote_anchor_dist = -1.0;  // >=0: override fix_demote_anchor_distance_m (default 3.0)
    double fix_demote_anchor_res = -1.0;   // >=0: override fix_demote_anchor_trust_res_m (0 = FSM gate)
    double fix_demote_res = -1.0;          // >=0: override fix_demote_res_m (0 = off)
    int fix_demote_posthold = -1;          // >=0: override fix_demote_posthold_epochs (0 = off)
    double fix_demote_res_rel = -1.0;      // >=0: override fix_demote_res_rel (0 = off)
    bool leaky_persist = false;             // C1: enable use_cp_hold_leaky_persist
    double leaky_persist_decay = -1.0;      // >=0: override cp_hold_persist_decay (default 1.0)
    bool fix_demote_anchor_gross = false;   // C2: enable fix_demote_anchor_gross
    double fix_demote_anchor_gross_ratio = -1.0;  // >=0: override fix_demote_anchor_gross_ratio (default 10.0)
    double fix_demote_anchor_gross_abs = -1.0;    // >=0: override fix_demote_anchor_gross_abs_m (default 20.0)
    bool fix_demote_surplus_crosscheck = false;   // enable fix_demote_surplus_crosscheck (needs --fix-demote + --surplus-validation)
    bool surplus_overrides_dr = false;   // "c2" lever: enable surplus_validation_overrides_history_dr (needs --fixed-history-dr + --surplus-validation)
    int surplus_overrides_dr_min_used = 0;  // >0: override surplus_validation_overrides_history_dr_min_surplus_used (default 0 = no extra floor)
    int surplus_overrides_dr_max_consec = 0;  // >0: override surplus_validation_overrides_history_dr_max_consecutive (default 0 = no cap)
    // Surplus-satellite independent integrity validation (see FGOConfig::use_surplus_satellite_validation)
    bool surplus_validation = false;
    bool surplus_validation_monitor = false;
    bool surplus_validation_veto = false;
    double surplus_validation_veto_ratio_ceiling = -1.0;
    double surplus_validation_veto_min_ddpr = -1.0;
    int surplus_validation_min_n = 0;          // >0: override surplus_validation_min_surplus_satellites (default 2)
    double surplus_validation_ap_lt1 = -1.0;   // >=0: override surplus_validation_aperture_pdop_lt1_cycles (default 0.1)
    double surplus_validation_ap_1to2 = -1.0;  // >=0: override surplus_validation_aperture_pdop_1to2_cycles (default 0.2)
    double surplus_validation_ap_gt2 = -1.0;   // >=0: override surplus_validation_aperture_pdop_gt2_cycles (default 0.3)
    bool surplus_validation_majority = false;  // use majority aggregation instead of require-all
    double surplus_validation_majority_frac = -1.0;  // >=0: override surplus_validation_majority_fraction (default 0.5)
    // Below-floor low-count AR rescue (see FGOConfig::use_low_count_ambiguity_resolution)
    bool low_count_ar = false;
    int low_count_ar_min = -1;        // >0: override low_count_min_candidates (default 4)
    double low_count_ar_ratio = -1.0; // >=0: override low_count_min_ratio (default 1.5); use --low-count-ratio 0 for "surplus alone"
    std::string dump_csv_path;  // debug: per-epoch CSV dump (tow/status/E-N-U err/position) for plotting
    // Opt-in FGOProblem cache (skips repeated RINEX parse + problem build
    // across validation runs on the SAME inputs/config). Default-off; when
    // empty, behavior is byte-identical to the pre-cache harness. See
    // ProblemCacheFingerprint / loadProblemCache / writeProblemCache below.
    std::string problem_cache_path;
};

Args parseArgs(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        // Keep standalone switches outside the very long else-if parser;
        // MSVC otherwise exceeds its block-nesting limit (C1061).
        if (a == "--problem-cache" && i + 1 < argc) {
            args.problem_cache_path = argv[++i];
            continue;
        }
        if (a == "--fixed-lag-qr") {
            args.fixed_lag_qr = true;
            continue;
        }
        if (a == "--no-held-fix-label") {
            args.no_held_fix_label = true;
            continue;
        }
        if (a == "--gici-par") {
            args.gici_par = true;
            continue;
        }
        if (a == "--integer-constrained-reoptimization") {
            args.integer_constrained_reoptimization = true;
            continue;
        }
        if (a == "--integer-constrained-prior-sigma" && i + 1 < argc) {
            args.integer_constrained_prior_sigma_cycles = std::stod(argv[++i]);
            continue;
        }
        if (a == "--integer-constrained-cost-tolerance" && i + 1 < argc) {
            args.integer_constrained_cost_tolerance = std::stod(argv[++i]);
            continue;
        }
        if (a == "--integer-constrained-max-iterations" && i + 1 < argc) {
            args.integer_constrained_max_iterations = std::stoi(argv[++i]);
            continue;
        }
        if (a == "--cp-hold-keep-imu-chain") {
            args.cp_hold_keep_imu_chain = true;
            continue;
        }
        if (a == "--sd-doppler") {
            args.sd_doppler = true;
            continue;
        }
        if (a == "--postfit-gate") {
            args.postfit_gate = true;
            continue;
        }
        if (a == "--surplus-validation") {
            args.surplus_validation = true;
            continue;
        }
        if (a == "--surplus-validation-monitor") {
            args.surplus_validation = true;
            args.surplus_validation_monitor = true;
            continue;
        }
        if (a == "--surplus-validation-veto") {
            args.surplus_validation = true;
            args.surplus_validation_veto = true;
            continue;
        }
        if (a == "--surplus-validation-veto-ratio-ceiling" && i + 1 < argc) {
            args.surplus_validation_veto_ratio_ceiling = std::stod(argv[++i]);
            continue;
        }
        if (a == "--surplus-validation-veto-min-ddpr" && i + 1 < argc) {
            args.surplus_validation_veto_min_ddpr = std::stod(argv[++i]);
            continue;
        }
        if (a == "--surplus-validation-min-n" && i + 1 < argc) {
            args.surplus_validation_min_n = std::stoi(argv[++i]);
            continue;
        }
        if (a == "--surplus-validation-aperture-lt1" && i + 1 < argc) {
            args.surplus_validation_ap_lt1 = std::stod(argv[++i]);
            continue;
        }
        if (a == "--surplus-validation-aperture-1to2" && i + 1 < argc) {
            args.surplus_validation_ap_1to2 = std::stod(argv[++i]);
            continue;
        }
        if (a == "--surplus-validation-aperture-gt2" && i + 1 < argc) {
            args.surplus_validation_ap_gt2 = std::stod(argv[++i]);
            continue;
        }
        if (a == "--surplus-validation-majority") {
            args.surplus_validation_majority = true;
            continue;
        }
        if (a == "--surplus-validation-majority-frac" && i + 1 < argc) {
            args.surplus_validation_majority_frac = std::stod(argv[++i]);
            continue;
        }
        if (a == "--fix-demote-surplus-crosscheck") {
            args.fix_demote_surplus_crosscheck = true;
            continue;
        }
        if (a == "--cmc-ref") {
            args.cmc_ref = true;
            continue;
        }
        if (a == "--surplus-overrides-dr") {
            args.surplus_overrides_dr = true;
            continue;
        }
        if (a == "--surplus-overrides-dr-min-used" && i + 1 < argc) {
            args.surplus_overrides_dr_min_used = std::stoi(argv[++i]);
            continue;
        }
        if (a == "--surplus-overrides-dr-max-consec" && i + 1 < argc) {
            args.surplus_overrides_dr_max_consec = std::stoi(argv[++i]);
            continue;
        }
        if (a == "--low-count-ar") {
            args.low_count_ar = true;
            continue;
        }
        if (a == "--low-count-min" && i + 1 < argc) {
            args.low_count_ar_min = std::stoi(argv[++i]);
            continue;
        }
        if (a == "--low-count-ratio" && i + 1 < argc) {
            args.low_count_ar_ratio = std::stod(argv[++i]);
            continue;
        }
        if (a == "--adaptive-ratio") {
            args.adaptive_ratio = true;
            continue;
        }
        if (a == "--external-dr") {
            args.external_dr = true;
            continue;
        }
        if (a == "--external-dr-max-age" && i + 1 < argc) {
            args.external_dr_max_age = std::stoi(argv[++i]);
            continue;
        }
        if (a == "--external-dr-chi2" && i + 1 < argc) {
            args.external_dr_chi2 = std::stod(argv[++i]);
            continue;
        }
        if (a == "--external-dr-reset-ratio" && i + 1 < argc) {
            args.external_dr_reset_ratio = std::stod(argv[++i]);
            continue;
        }
        if (a == "--postfit-min-n" && i + 1 < argc) {
            args.postfit_min_n = std::stoi(argv[++i]);
            continue;
        }
        if (a == "--postfit-rms" && i + 1 < argc) {
            args.postfit_rms = std::stod(argv[++i]);
            continue;
        }
        if (a == "--postfit-max-norm" && i + 1 < argc) {
            args.postfit_max_norm = std::stod(argv[++i]);
            continue;
        }
        if (a == "--postfit-chi2" && i + 1 < argc) {
            args.postfit_chi2 = std::stod(argv[++i]);
            continue;
        }
        if (a == "--start-epoch" && i + 1 < argc) {
            args.start_epoch = std::max(0, std::stoi(argv[++i]));
            continue;
        }
        if (a == "--rover" && i + 1 < argc) {
            args.rover_path = argv[++i];
        } else if (a == "--base" && i + 1 < argc) {
            args.base_path = argv[++i];
        } else if (a == "--nav" && i + 1 < argc) {
            args.nav_path = argv[++i];
        } else if (a == "--max-epochs" && i + 1 < argc) {
            args.max_epochs = std::stoi(argv[++i]);
        } else if (a == "--max-iters" && i + 1 < argc) {
            args.max_iters = std::stoi(argv[++i]);
        } else if (a == "--no-pr") {
            args.no_pr_factors = true;
        } else if (a == "--no-robust") {
            args.no_robust = true;
        } else if (a == "--float-only") {
            args.float_only = true;
        } else if (a == "--imu" && i + 1 < argc) {
            args.imu_path = argv[++i];
        } else if (a == "--ref" && i + 1 < argc) {
            args.ref_path = argv[++i];
        } else if (a == "--fixed-lag" && i + 1 < argc) {
            args.fixed_lag_s = std::stod(argv[++i]);
        } else if (a == "--nhc") {
            args.use_nhc = true;
        } else if (a == "--zupt") {
            args.use_zupt = true;
        } else if (a == "--hold") {
            args.use_hold = true;
        } else if (a == "--elev-mask" && i + 1 < argc) {
            args.elev_mask_deg = std::stod(argv[++i]);
        } else if (a == "--snr-mask" && i + 1 < argc) {
            args.snr_mask_dbhz = std::stod(argv[++i]);
        } else if (a == "--diag") {
            args.diag = true;
        } else if (a == "--single-freq") {
            args.single_freq = true;
        } else if (a == "--multi-freq") {
            args.multi_freq = true;
        } else if (a == "--one-band-per-sat") {
            args.oneband = true;
        } else if (a == "--no-oneband") {
            args.no_oneband = true;
        } else if (a == "--no-band-sigma") {
            args.no_band_sigma = true;
        } else if (a == "--no-code-align") {
            args.no_code_align = true;
        } else if (a == "--dd-resid") {
            args.dd_resid = true;
        } else if (a == "--partial-ar") {
            args.partial_ar = true;
        } else if (a == "--no-gal-ar") {
            args.no_gal_ar = true;
        } else if (a == "--ratio" && i + 1 < argc) {
            args.ratio_threshold = std::stod(argv[++i]);
        } else if (a == "--imu-ratio-aperture") {
            args.imu_ratio_aperture = true;
        } else if (a == "--imu-ratio-relaxed" && i + 1 < argc) {
            args.imu_ratio_relaxed = std::stod(argv[++i]);
        } else if (a == "--imu-ratio-float-sep" && i + 1 < argc) {
            args.imu_ratio_float_sep = std::stod(argv[++i]);
        } else if (a == "--imu-ratio-pred-sep" && i + 1 < argc) {
            args.imu_ratio_pred_sep = std::stod(argv[++i]);
        } else if (a == "--fixed-history-dr") {
            args.fixed_history_dr = true;
        } else if (a == "--fixed-history-dr-window" && i + 1 < argc) {
            args.fixed_history_dr_window = std::stoi(argv[++i]);
        } else if (a == "--fixed-history-dr-sep" && i + 1 < argc) {
            args.fixed_history_dr_sep = std::stod(argv[++i]);
        } else if (a == "--anchor-aided-validation") {
            args.anchor_aided_validation = true;
        } else if (a == "--constellation-par") {
            args.constellation_par = true;
        } else if (a == "--residual-par") {
            args.residual_par = true;
        } else if (a == "--variance-par") {
            args.variance_par = true;
        } else if (a == "--par-max-std" && i + 1 < argc) {
            args.par_max_std = std::stod(argv[++i]);
        } else if (a == "--hold-ratio" && i + 1 < argc) {
            args.hold_ratio = std::stod(argv[++i]);
        } else if (a == "--hold-min" && i + 1 < argc) {
            args.hold_min = std::stoi(argv[++i]);
        } else if (a == "--min-fixed" && i + 1 < argc) {
            args.min_fixed = std::stoi(argv[++i]);
        } else if (a == "--gates") {
            args.gates = true;
        } else if (a == "--gate-res" && i + 1 < argc) {
            args.gate_res = std::stod(argv[++i]);
        } else if (a == "--gate-sat-res" && i + 1 < argc) {
            args.gate_sat_res = std::stod(argv[++i]);
        } else if (a == "--gate-gdop" && i + 1 < argc) {
            args.gate_gdop = std::stod(argv[++i]);
        } else if (a == "--gate-min-sat" && i + 1 < argc) {
            args.gate_min_sat = std::stoi(argv[++i]);
        } else if (a == "--cmc") {
            args.cmc = true;
        } else if (a == "--cmc-level-pr-only") {
            args.cmc = true;
            args.cmc_level_pr_only = true;
        } else if (a == "--cmc-jump" && i + 1 < argc) {
            args.cmc_jump = std::stod(argv[++i]);
        } else if (a == "--cmc-level" && i + 1 < argc) {
            args.cmc_level = std::stod(argv[++i]);
        } else if (a == "--cmc-warmup" && i + 1 < argc) {
            args.cmc_warmup = std::stoi(argv[++i]);
        } else if (a == "--cmc-alpha" && i + 1 < argc) {
            args.cmc_alpha = std::stod(argv[++i]);
        } else if (a == "--cp-hold") {
            args.cp_hold = true;
        } else if (a == "--cp-hold-float-recovery") {
            args.cp_hold = true;
            args.cp_hold_float_recovery = true;
        } else if (a == "--cp-hold-anchor-release") {
            args.cp_hold = true;
            args.cp_hold_anchor_release = true;
        } else if (a == "--cp-hold-selective") {
            args.selective_cp_hold = true;
        } else if (a == "--cp-hold-res" && i + 1 < argc) {
            args.cp_hold_res = std::stod(argv[++i]);
        } else if (a == "--cp-hold-catastrophic" && i + 1 < argc) {
            args.cp_hold_catastrophic = std::stod(argv[++i]);
        } else if (a == "--cp-hold-fast-worst-sat" && i + 1 < argc) {
            args.cp_hold_fast_worst_sat = std::stod(argv[++i]);
        } else if (a == "--cp-hold-persist" && i + 1 < argc) {
            args.cp_hold_persist = std::stoi(argv[++i]);
        } else if (a == "--cp-hold-epochs" && i + 1 < argc) {
            args.cp_hold_epochs_n = std::stoi(argv[++i]);
        } else if (a == "--cp-hold-release-res" && i + 1 < argc) {
            args.cp_hold_release_res = std::stod(argv[++i]);
        } else if (a == "--cp-hold-release-count" && i + 1 < argc) {
            args.cp_hold_release_n = std::stoi(argv[++i]);
        } else if (a == "--cp-hold-pose-replace" && i + 1 < argc) {
            args.cp_hold_pose_replace = std::stod(argv[++i]);
        } else if (a == "--cp-hold-gdop" && i + 1 < argc) {
            args.cp_hold_gdop = std::stod(argv[++i]);
        } else if (a == "--exc-recovery") {
            args.exc_recovery = true;
        } else if (a == "--ddpr-anchor") {
            args.ddpr_anchor = true;
        } else if (a == "--ddpr-anchor-max-res" && i + 1 < argc) {
            args.ddpr_anchor_max_res = std::stod(argv[++i]);
        } else if (a == "--ddpr-anchor-fde" && i + 1 < argc) {
            args.ddpr_anchor_fde = std::stod(argv[++i]);
        } else if (a == "--ddpr-anchor-min-n" && i + 1 < argc) {
            args.ddpr_anchor_min_n = std::stoi(argv[++i]);
        } else if (a == "--ddpr-anchor-boot-epochs" && i + 1 < argc) {
            args.ddpr_anchor_boot_epochs = std::stoi(argv[++i]);
        } else if (a == "--ddpr-anchor-boot-sigma" && i + 1 < argc) {
            args.ddpr_anchor_boot_sigma = std::stod(argv[++i]);
        } else if (a == "--ddpr-anchor-boot-after-mass" && i + 1 < argc) {
            args.ddpr_anchor_boot_after_mass = std::stoi(argv[++i]);
        } else if (a == "--fde") {
            args.fde = true;
        } else if (a == "--fde-pr-only") {
            args.fde = true;
            args.fde_pr_only = true;
        } else if (a == "--fde-cp-quarantine") {
            args.fde = true;
            args.fde_cp_quarantine = true;
        } else if (a == "--fde-pr" && i + 1 < argc) {
            args.fde_pr = std::stod(argv[++i]);
        } else if (a == "--fde-cp" && i + 1 < argc) {
            args.fde_cp = std::stod(argv[++i]);
        } else if (a == "--fde-frac" && i + 1 < argc) {
            args.fde_frac = std::stod(argv[++i]);
        } else if (a == "--fde-iters" && i + 1 < argc) {
            args.fde_iters = std::stoi(argv[++i]);
        } else if (a == "--sat-badness") {
            args.sat_badness = true;
        } else if (a == "--sat-badness-cp-scale" && i + 1 < argc) {
            args.sat_badness_cp_scale = std::stod(argv[++i]);
        } else if (a == "--sat-badness-pr-scale" && i + 1 < argc) {
            args.sat_badness_pr_scale = std::stod(argv[++i]);
        } else if (a == "--sat-badness-ddpr-thresh" && i + 1 < argc) {
            args.sat_badness_ddpr_thresh = std::stod(argv[++i]);
        } else if (a == "--sat-badness-clamp" && i + 1 < argc) {
            args.sat_badness_clamp = std::stod(argv[++i]);
        } else if (a == "--sat-badness-cap" && i + 1 < argc) {
            args.sat_badness_cap = std::stod(argv[++i]);
        } else if (a == "--sat-badness-cppr-decay" && i + 1 < argc) {
            args.sat_badness_cppr_decay = std::stod(argv[++i]);
        } else if (a == "--varerr") {
            args.varerr = true;
        } else if (a == "--varerr-a" && i + 1 < argc) {
            args.varerr_a = std::stod(argv[++i]);
        } else if (a == "--varerr-b" && i + 1 < argc) {
            args.varerr_b = std::stod(argv[++i]);
        } else if (a == "--varerr-eratio" && i + 1 < argc) {
            args.varerr_eratio = std::stod(argv[++i]);
        } else if (a == "--imu-accel-noise" && i + 1 < argc) {
            args.imu_accel_noise = std::stod(argv[++i]);
        } else if (a == "--imu-gyro-noise" && i + 1 < argc) {
            args.imu_gyro_noise = std::stod(argv[++i]);
        } else if (a == "--imu-accel-bias" && i + 1 < argc) {
            args.imu_accel_bias = std::stod(argv[++i]);
        } else if (a == "--imu-gyro-bias" && i + 1 < argc) {
            args.imu_gyro_bias = std::stod(argv[++i]);
        } else if (a == "--imu-preset-tactical") {
            args.imu_preset_tactical = true;
        } else if (a == "--integ-cov" && i + 1 < argc) {
            args.integ_cov = std::stod(argv[++i]);
        } else if (a == "--integ-cov-inflate") {
            args.integ_cov_inflate = true;
        } else if (a == "--integ-cov-max" && i + 1 < argc) {
            args.integ_cov_max = std::stod(argv[++i]);
        } else if (a == "--gravity" && i + 1 < argc) {
            args.gravity_mps2 = std::stod(argv[++i]);
        } else if (a == "--fixed-err-hist") {
            args.fixed_err_hist = true;
        } else if (a == "--partial-ar-frac" && i + 1 < argc) {
            args.partial_ar_frac = std::stod(argv[++i]);
        } else if (a == "--hold-sigma" && i + 1 < argc) {
            args.hold_sigma = std::stod(argv[++i]);
        } else if (a == "--stale-pin") {
            args.stale_pin = true;
        } else if (a == "--stale-pin-res" && i + 1 < argc) {
            args.stale_pin_res = std::stod(argv[++i]);
        } else if (a == "--stale-pin-age" && i + 1 < argc) {
            args.stale_pin_age = std::stoi(argv[++i]);
        } else if (a == "--fix-demote") {
            args.fix_demote = true;
        } else if (a == "--fix-demote-dist" && i + 1 < argc) {
            args.fix_demote_dist = std::stod(argv[++i]);
        } else if (a == "--fix-demote-anchor") {
            args.fix_demote_anchor = true;
        } else if (a == "--fix-demote-anchor-dist" && i + 1 < argc) {
            args.fix_demote_anchor_dist = std::stod(argv[++i]);
        } else if (a == "--fix-demote-anchor-res" && i + 1 < argc) {
            args.fix_demote_anchor_res = std::stod(argv[++i]);
        } else if (a == "--fix-demote-res" && i + 1 < argc) {
            args.fix_demote_res = std::stod(argv[++i]);
        } else if (a == "--fix-demote-posthold" && i + 1 < argc) {
            args.fix_demote_posthold = std::stoi(argv[++i]);
        } else if (a == "--fix-demote-res-rel" && i + 1 < argc) {
            args.fix_demote_res_rel = std::stod(argv[++i]);
        } else if (a == "--leaky-persist") {
            args.leaky_persist = true;
        } else if (a == "--leaky-persist-decay" && i + 1 < argc) {
            args.leaky_persist_decay = std::stod(argv[++i]);
        } else if (a == "--fix-demote-anchor-gross") {
            args.fix_demote_anchor_gross = true;
        } else if (a == "--fix-demote-anchor-gross-ratio" && i + 1 < argc) {
            args.fix_demote_anchor_gross_ratio = std::stod(argv[++i]);
        } else if (a == "--fix-demote-anchor-gross-abs" && i + 1 < argc) {
            args.fix_demote_anchor_gross_abs = std::stod(argv[++i]);
        } else if (a == "--dump-csv" && i + 1 < argc) {
            args.dump_csv_path = argv[++i];
        } else {
            std::cerr << "Unknown/incomplete arg: " << a << "\n";
            std::exit(2);
        }
    }
    if (args.rover_path.empty() || args.base_path.empty() || args.nav_path.empty()) {
        std::cerr << "Usage: gnss_fgo_parity --rover <rover.obs> --base <base.obs> "
                     "--nav <base.nav> [--max-epochs N]\n";
        std::exit(2);
    }
    return args;
}

// Mirrors apps/native/gnss_fgo.cpp's "real-data-float" preset for the DD RTK path so
// the FGOProblem we build matches a realistic production configuration.
libgnss::FGOProcessor::FGOConfig makeRealDataDdConfig() {
    libgnss::FGOProcessor::FGOConfig config;
    config.max_iterations = 12;
    config.pseudorange_sigma_m = 4.0;
    config.motion_sigma_m = 100.0;
    config.clock_motion_sigma_m = 600.0;
    config.tdcp_sigma_m = 0.05;
    config.carrier_phase_sigma_m = 0.02;
    config.double_difference_pseudorange_sigma_m = 1.0;
    config.double_difference_carrier_sigma_m = 0.02;
    config.ambiguity_prior_sigma_m = 2000.0;
    config.pseudorange_huber_threshold_sigma = 3.0;
    config.carrier_phase_huber_threshold_sigma = 3.0;
    config.tdcp_huber_threshold_sigma = 3.0;
    config.max_tdcp_gap_s = 1.5;
    config.min_elevation_deg = 15.0;
    config.use_robust_loss = true;
    config.use_double_difference_factors = true;
    config.use_pseudorange_factors = true;
    config.use_carrier_phase_factors = false;
    config.use_tdcp_factors = true;
    config.use_ambiguity_priors = true;
    config.fix_ambiguities = false;
    // Per-epoch LAMBDA on this urban multipath data tops out around ratio ~2.5;
    // the default 3.0 gate fixes nothing on EITHER backend's per-epoch path
    // (Eigen only "fixes" via its lenient global nearest-integer snap). Use 2.0
    // so both backends' per-epoch LAMBDA actually engage and can be compared.
    config.lambda_ratio_threshold = 2.0;
    config.use_inter_system_biases = true;
    return config;
}

// Maps every CLI knob onto FGOConfig. Extracted out of main() so the
// --problem-cache fingerprint (which snapshots this struct's raw bytes) can
// be computed BEFORE the expensive RINEX parse + buildDoubleDifferenceProblem
// call, without duplicating this arg-to-config mapping logic.
libgnss::FGOProcessor::FGOConfig buildFgoConfig(const Args& args) {
    libgnss::FGOProcessor::FGOConfig config = makeRealDataDdConfig();
    config.report_held_ambiguities_as_fixed = !args.no_held_fix_label;
    if (args.max_iters > 0) {
        config.max_iterations = args.max_iters;
    }
    if (args.no_robust) {
        config.use_robust_loss = false;
    }
    if (args.no_pr_factors) {
        // Pure DD path: no undifferenced pseudorange factors, so there are no
        // receiver-clock / inter-system-bias states at all -- this isolates the
        // clock-free double-difference RTK path (the Phase-1 target) from the
        // undifferenced-pseudorange modeling differences between backends.
        config.use_pseudorange_factors = false;
    }
    // Milestone 2e lever 2: elevation / SNR masks on the DD observations
    // (applied at problem-build time -- attacks deep-urban multipath float).
    if (args.elev_mask_deg > 0.0) {
        config.min_elevation_deg = args.elev_mask_deg;
    }
    if (args.snr_mask_dbhz > 0.0) {
        config.min_snr_dbhz = args.snr_mask_dbhz;
        config.double_difference_reference_min_snr_dbhz = args.snr_mask_dbhz;
        config.double_difference_base_min_snr_dbhz = args.snr_mask_dbhz;
    }
    // Multi-freq control: --single-freq forces the front-end back to L1/E1/B1I
    // only (the pre-change behaviour) so the SAME harness invocation yields a
    // single-frequency baseline for apples-to-apples comparison.
    if (args.single_freq) {
        config.use_multi_frequency_double_difference = false;
    }
    if (args.multi_freq) {
        config.use_multi_frequency_double_difference = true;
    }
    if (args.sd_doppler) {
        config.use_single_difference_doppler_factors = true;
    }
    if (args.postfit_gate) {
        config.use_fixed_hypothesis_postfit_validation = true;
    }
    if (args.adaptive_ratio) {
        config.use_satellite_count_adaptive_ratio = true;
    }
    if (args.external_dr) {
        config.use_external_doppler_dr_validation = true;
    }
    if (args.external_dr_max_age > 0) {
        config.external_doppler_dr_max_age_epochs = args.external_dr_max_age;
    }
    if (args.external_dr_chi2 >= 0.0) {
        config.external_doppler_dr_chi2_threshold = args.external_dr_chi2;
    }
    if (args.external_dr_reset_ratio >= 0.0) {
        config.external_doppler_dr_reset_min_ratio = args.external_dr_reset_ratio;
    }
    if (args.postfit_min_n > 0) {
        config.fixed_postfit_min_factors = args.postfit_min_n;
    }
    if (args.postfit_rms >= 0.0) {
        config.fixed_postfit_max_rms_m = args.postfit_rms;
    }
    if (args.postfit_max_norm >= 0.0) {
        config.fixed_postfit_max_normalized_residual = args.postfit_max_norm;
    }
    if (args.postfit_chi2 >= 0.0) {
        config.fixed_postfit_max_chi2_per_dof = args.postfit_chi2;
    }
    if (args.oneband) {
        config.double_difference_lambda_one_band_per_satellite = true;
    }
    if (args.no_oneband) {
        config.double_difference_lambda_one_band_per_satellite = false;
    }
    if (args.no_band_sigma) {
        config.double_difference_secondary_carrier_sigma_scale = 1.0;
        config.double_difference_secondary_pseudorange_sigma_scale = 1.0;
    }
    if (args.no_code_align) {
        config.use_double_difference_secondary_code_alignment = false;
    }
    if (args.partial_ar) {
        config.use_fixed_lag_partial_lambda = true;
    }
    if (args.integer_constrained_reoptimization) {
        config.use_integer_constrained_reoptimization = true;
    }
    if (args.integer_constrained_prior_sigma_cycles > 0.0) {
        config.integer_constrained_prior_sigma_cycles =
            args.integer_constrained_prior_sigma_cycles;
    }
    if (args.integer_constrained_cost_tolerance >= 0.0) {
        config.integer_constrained_cost_abs_tolerance =
            args.integer_constrained_cost_tolerance;
    }
    if (args.integer_constrained_max_iterations > 0) {
        config.integer_constrained_max_iterations =
            args.integer_constrained_max_iterations;
    }
    if (args.partial_ar_frac >= 0.0) {
        config.fixed_lag_partial_lambda_min_fraction = args.partial_ar_frac;
    }
    if (args.hold_sigma >= 0.0) {
        config.ambiguity_hold_sigma_cycles = args.hold_sigma;
    }
    if (args.stale_pin) {
        config.use_stale_pin_invalidation = true;
    }
    if (args.stale_pin_res >= 0.0) {
        config.stale_pin_per_sat_residual_m = args.stale_pin_res;
    }
    if (args.stale_pin_age >= 0) {
        config.stale_pin_min_hold_age_epochs = args.stale_pin_age;
    }
    if (args.fix_demote) {
        config.use_fix_plausibility_demotion = true;
    }
    if (args.fix_demote_dist >= 0.0) {
        config.fix_demote_distance_m = args.fix_demote_dist;
    }
    if (args.fix_demote_anchor) {
        config.fix_demote_use_ddpr_anchor = true;
    }
    if (args.fix_demote_anchor_dist >= 0.0) {
        config.fix_demote_anchor_distance_m = args.fix_demote_anchor_dist;
    }
    if (args.fix_demote_anchor_res >= 0.0) {
        config.fix_demote_anchor_trust_res_m = args.fix_demote_anchor_res;
    }
    if (args.fix_demote_res >= 0.0) {
        config.fix_demote_res_m = args.fix_demote_res;
    }
    if (args.fix_demote_posthold >= 0) {
        config.fix_demote_posthold_epochs = args.fix_demote_posthold;
    }
    if (args.fix_demote_res_rel >= 0.0) {
        config.fix_demote_res_rel = args.fix_demote_res_rel;
    }
    if (args.fix_demote_surplus_crosscheck) {
        config.fix_demote_surplus_crosscheck = true;
    }
    if (args.leaky_persist) {
        config.use_cp_hold_leaky_persist = true;
    }
    if (args.leaky_persist_decay >= 0.0) {
        config.cp_hold_persist_decay = args.leaky_persist_decay;
    }
    if (args.fix_demote_anchor_gross) {
        config.fix_demote_anchor_gross = true;
    }
    if (args.fix_demote_anchor_gross_ratio >= 0.0) {
        config.fix_demote_anchor_gross_ratio = args.fix_demote_anchor_gross_ratio;
    }
    if (args.fix_demote_anchor_gross_abs >= 0.0) {
        config.fix_demote_anchor_gross_abs_m = args.fix_demote_anchor_gross_abs;
    }
    if (args.surplus_validation) {
        config.use_surplus_satellite_validation = true;
    }
    if (args.surplus_validation_monitor) {
        config.surplus_validation_monitor_only = true;
    }
    if (args.surplus_validation_veto) {
        config.surplus_validation_veto_high_ratio_fails = true;
    }
    if (args.surplus_validation_veto_ratio_ceiling >= 0.0) {
        config.surplus_validation_veto_ratio_ceiling =
            args.surplus_validation_veto_ratio_ceiling;
    }
    if (args.surplus_validation_veto_min_ddpr >= 0.0) {
        config.surplus_validation_veto_min_ddpr_rms_m =
            args.surplus_validation_veto_min_ddpr;
    }
    if (args.surplus_validation_min_n > 0) {
        config.surplus_validation_min_surplus_satellites = args.surplus_validation_min_n;
    }
    if (args.surplus_validation_ap_lt1 >= 0.0) {
        config.surplus_validation_aperture_pdop_lt1_cycles = args.surplus_validation_ap_lt1;
    }
    if (args.surplus_validation_ap_1to2 >= 0.0) {
        config.surplus_validation_aperture_pdop_1to2_cycles = args.surplus_validation_ap_1to2;
    }
    if (args.surplus_validation_ap_gt2 >= 0.0) {
        config.surplus_validation_aperture_pdop_gt2_cycles = args.surplus_validation_ap_gt2;
    }
    if (args.surplus_validation_majority) {
        config.surplus_validation_require_all = false;
    }
    if (args.surplus_validation_majority_frac >= 0.0) {
        config.surplus_validation_majority_fraction = args.surplus_validation_majority_frac;
    }
    if (args.low_count_ar) {
        config.use_low_count_ambiguity_resolution = true;
    }
    if (args.low_count_ar_min > 0) {
        config.low_count_min_candidates = args.low_count_ar_min;
    }
    if (args.low_count_ar_ratio >= 0.0) {
        config.low_count_min_ratio = args.low_count_ar_ratio;
    }
    if (args.no_gal_ar) {
        config.exclude_galileo_ambiguity_fixing = true;
    }
    if (args.ratio_threshold > 0.0) {
        config.lambda_ratio_threshold = args.ratio_threshold;
    }
    if (args.imu_ratio_aperture) {
        config.use_imu_aided_ratio_aperture = true;
    }
    if (args.imu_ratio_relaxed >= 0.0) {
        config.imu_aided_relaxed_ratio_threshold = args.imu_ratio_relaxed;
    }
    if (args.imu_ratio_float_sep >= 0.0) {
        config.imu_aided_max_float_separation_m = args.imu_ratio_float_sep;
    }
    if (args.imu_ratio_pred_sep >= 0.0) {
        config.imu_aided_max_prediction_separation_m = args.imu_ratio_pred_sep;
    }
    if (args.fixed_history_dr) {
        config.use_fixed_history_dr_validation = true;
    }
    if (args.fixed_history_dr_window > 0) {
        config.fixed_history_dr_window_epochs = args.fixed_history_dr_window;
    }
    if (args.fixed_history_dr_sep >= 0.0) {
        config.fixed_history_dr_max_separation_m = args.fixed_history_dr_sep;
    }
    if (args.surplus_overrides_dr) {
        config.surplus_validation_overrides_history_dr = true;
    }
    if (args.surplus_overrides_dr_min_used > 0) {
        config.surplus_validation_overrides_history_dr_min_surplus_used =
            args.surplus_overrides_dr_min_used;
    }
    if (args.surplus_overrides_dr_max_consec > 0) {
        config.surplus_validation_overrides_history_dr_max_consecutive =
            args.surplus_overrides_dr_max_consec;
    }
    if (args.anchor_aided_validation) {
        config.use_ddpr_anchor_aided_validation = true;
    }
    if (args.constellation_par) {
        config.use_constellation_ranked_partial_ar = true;
    }
    if (args.residual_par) {
        config.use_residual_screened_partial_ar = true;
    }
    if (args.variance_par) {
        config.use_variance_ranked_partial_ar = true;
    }
    if (args.gici_par) {
        config.use_fixed_lag_partial_lambda = true;
        config.use_variance_ranked_partial_ar = true;
        config.use_continuous_unfix_ambiguity_reset = true;
        // The Tokyo1 audit isolated every wrong fix in a near-threshold
        // ratio/poor-carrier-postfit cluster. Validate only that aperture
        // fringe, preserving the high-ratio fixes that remain trustworthy
        // even when urban DD code residuals are large.
        config.use_fixed_hypothesis_postfit_validation = true;
        config.fixed_postfit_validate_normal_ratio = true;
        config.fixed_postfit_normal_ratio_ceiling = 3.2;
        if (args.postfit_chi2 < 0.0) {
            config.fixed_postfit_max_chi2_per_dof = 1.5;
        }
        if (args.partial_ar_frac < 0.0) {
            config.fixed_lag_partial_lambda_min_fraction = 0.9;
        }
        if (args.ratio_threshold <= 0.0) config.lambda_ratio_threshold = 3.0;
        if (args.min_fixed <= 0) config.min_fixed_ambiguities = 6;
        if (args.par_max_std < 0.0) config.partial_ar_max_std_cycles = 0.25;
    }
    if (args.par_max_std >= 0.0) {
        config.partial_ar_max_std_cycles = args.par_max_std;
    }
    if (args.hold_ratio > 0.0) {
        config.ambiguity_hold_ratio_threshold = args.hold_ratio;
    }
    if (args.hold_min > 0) {
        config.ambiguity_hold_min_fixed = args.hold_min;
    }
    if (args.min_fixed > 0) {
        config.min_fixed_ambiguities = args.min_fixed;
    }
    if (args.gates) {
        config.use_epoch_quality_gates = true;
    }
    if (args.gate_res >= 0.0) {
        config.gate_ddpr_res_max_m = args.gate_res;
    }
    if (args.gate_sat_res >= 0.0) {
        config.gate_per_sat_res_max_m = args.gate_sat_res;
    }
    if (args.gate_gdop > 0.0) {
        config.gate_gdop_max = args.gate_gdop;
    }
    if (args.gate_min_sat > 0) {
        config.gate_min_satellites = args.gate_min_sat;
    }
    if (args.cmc) {
        config.use_code_minus_carrier_screening = true;
    }
    if (args.cmc_jump >= 0.0) {
        config.code_minus_carrier_jump_threshold_m = args.cmc_jump;
    }
    if (args.cmc_level >= 0.0) {
        config.code_minus_carrier_level_threshold_m = args.cmc_level;
    }
    config.code_minus_carrier_level_pseudorange_only = args.cmc_level_pr_only;
    if (args.cmc_warmup > 0) {
        config.code_minus_carrier_warmup_epochs = args.cmc_warmup;
    }
    if (args.cmc_alpha >= 0.0) {
        config.code_minus_carrier_baseline_alpha = args.cmc_alpha;
    }
    if (args.cmc_ref) {
        config.cmc_aware_reference_selection = true;
    }
    if (args.cp_hold) {
        config.use_cp_hold_recovery = true;
    }
    if (args.cp_hold_float_recovery) {
        config.use_cp_hold_float_recovery = true;
    }
    if (args.cp_hold_anchor_release) {
        config.use_cp_hold_anchor_release = true;
    }
    if (args.cp_hold_keep_imu_chain) {
        config.cp_hold_break_imu_chain = false;
    }
    if (args.selective_cp_hold) {
        config.use_selective_cp_hold = true;
    }
    if (args.cp_hold_res >= 0.0) {
        config.cp_hold_main_residual_threshold_m = args.cp_hold_res;
    }
    if (args.cp_hold_catastrophic >= 0.0) {
        config.cp_hold_catastrophic_threshold_m = args.cp_hold_catastrophic;
    }
    if (args.cp_hold_fast_worst_sat >= 0.0) {
        config.cp_hold_fast_worst_satellite_min_m = args.cp_hold_fast_worst_sat;
    }
    if (args.cp_hold_persist > 0) {
        config.cp_hold_persist_epochs = args.cp_hold_persist;
    }
    if (args.cp_hold_epochs_n > 0) {
        config.cp_hold_epochs = args.cp_hold_epochs_n;
    }
    if (args.cp_hold_release_res >= 0.0) {
        config.cp_hold_release_threshold_m = args.cp_hold_release_res;
    }
    if (args.cp_hold_release_n > 0) {
        config.cp_hold_release_count = args.cp_hold_release_n;
    }
    if (args.cp_hold_pose_replace >= 0.0) {
        config.cp_hold_pose_replace_threshold_m = args.cp_hold_pose_replace;
    }
    if (args.cp_hold_gdop >= 0.0) {
        config.cp_hold_max_gdop = args.cp_hold_gdop;
    }
    if (args.exc_recovery) {
        config.use_solve_exception_recovery = true;
    }
    if (args.ddpr_anchor) {
        config.use_ddpr_anchor = true;
    }
    if (args.ddpr_anchor_max_res >= 0.0) {
        config.ddpr_anchor_max_residual_m = args.ddpr_anchor_max_res;
    }
    if (args.ddpr_anchor_fde >= 0.0) {
        config.ddpr_anchor_fde_threshold_m = args.ddpr_anchor_fde;
    }
    if (args.ddpr_anchor_min_n > 0) {
        config.ddpr_anchor_min_factors = args.ddpr_anchor_min_n;
    }
    if (args.ddpr_anchor_boot_epochs >= 0) {
        config.ddpr_anchor_bootstrap_epochs = args.ddpr_anchor_boot_epochs;
    }
    if (args.ddpr_anchor_boot_sigma >= 0.0) {
        config.ddpr_anchor_bootstrap_sigma_m = args.ddpr_anchor_boot_sigma;
    }
    if (args.ddpr_anchor_boot_after_mass >= 0) {
        config.cp_hold_bootstrap_after_mass_reset = (args.ddpr_anchor_boot_after_mass != 0);
    }
    if (args.fde) {
        config.use_fde = true;
    }
    config.fde_pseudorange_only = args.fde_pr_only;
    config.fde_carrier_quarantine = args.fde_cp_quarantine;
    if (args.fde_pr >= 0.0) {
        config.fde_pseudorange_threshold_m = args.fde_pr;
    }
    if (args.fde_cp >= 0.0) {
        config.fde_carrier_threshold_m = args.fde_cp;
    }
    if (args.fde_frac >= 0.0) {
        config.fde_max_rejected_fraction = args.fde_frac;
    }
    if (args.fde_iters > 0) {
        config.fde_max_iterations = args.fde_iters;
    }
    if (args.sat_badness) {
        config.use_sat_badness_downweight = true;
    }
    if (args.sat_badness_cp_scale >= 0.0) {
        config.sat_badness_carrier_sigma_scale = args.sat_badness_cp_scale;
    }
    if (args.sat_badness_pr_scale >= 0.0) {
        config.sat_badness_pseudorange_sigma_scale = args.sat_badness_pr_scale;
    }
    if (args.sat_badness_ddpr_thresh >= 0.0) {
        config.sat_badness_ddpr_threshold_m = args.sat_badness_ddpr_thresh;
    }
    if (args.sat_badness_clamp >= 0.0) {
        config.sat_badness_residual_clamp_m = args.sat_badness_clamp;
    }
    if (args.sat_badness_cap >= 0.0) {
        config.sat_badness_score_cap = args.sat_badness_cap;
    }
    if (args.sat_badness_cppr_decay >= 0.0) {
        config.sat_badness_cppr_decay = args.sat_badness_cppr_decay;
    }
    if (args.varerr) {
        config.use_elevation_dependent_sigma = true;
    }
    if (args.varerr_a >= 0.0) {
        config.elevation_sigma_err_a_m = args.varerr_a;
    }
    if (args.varerr_b >= 0.0) {
        config.elevation_sigma_err_b_m = args.varerr_b;
    }
    if (args.varerr_eratio >= 0.0) {
        config.elevation_sigma_pseudorange_ratio = args.varerr_eratio;
    }
    if (args.integ_cov >= 0.0) {
        config.imu_integration_covariance = args.integ_cov;
    }
    if (args.integ_cov_inflate) {
        config.use_imu_integration_covariance_inflation = true;
    }
    if (args.integ_cov_max >= 0.0) {
        config.imu_integration_covariance_max = args.integ_cov_max;
    }
    return config;
}

std::vector<libgnss::ObservationData> loadEpochs(const std::string& path,
                                                 int max_epochs,
                                                 const libgnss::Vector3d& fixed_position,
                                                 bool assign_fixed_position,
                                                 int start_epoch = 0) {
    libgnss::io::RINEXReader reader;
    std::vector<libgnss::ObservationData> epochs;
    if (!reader.open(path)) {
        std::cerr << "Error: cannot open " << path << "\n";
        std::exit(1);
    }
    libgnss::io::RINEXReader::RINEXHeader header;
    if (!reader.readHeader(header)) {
        std::cerr << "Error: cannot read header " << path << "\n";
        std::exit(1);
    }
    libgnss::ObservationData epoch;
    int input_index = 0;
    while (reader.readObservationEpoch(epoch)) {
        if (input_index++ < start_epoch) {
            continue;
        }
        if (assign_fixed_position) {
            epoch.receiver_position = fixed_position;
        } else if (header.approximate_position.norm() > 0.0) {
            epoch.receiver_position = header.approximate_position;
        }
        epochs.push_back(epoch);
        if (max_epochs > 0 && static_cast<int>(epochs.size()) >= max_epochs) {
            break;
        }
    }
    return epochs;
}

struct FloatParity {
    std::size_t compared_epochs = 0;
    double max_delta_m = 0.0;
    double rms_delta_m = 0.0;
    std::size_t nonfinite_eigen = 0;
    std::size_t nonfinite_gtsam = 0;
    std::size_t status_disagreements = 0;
};

const char* statusName(libgnss::SolutionStatus s) {
    switch (s) {
        case libgnss::SolutionStatus::NONE: return "NONE";
        case libgnss::SolutionStatus::SPP: return "SPP";
        case libgnss::SolutionStatus::FLOAT: return "FLOAT";
        case libgnss::SolutionStatus::FIXED: return "FIXED";
        default: return "?";
    }
}

FloatParity compareFloat(const libgnss::FGOProcessor::FGOResult& e,
                         const libgnss::FGOProcessor::FGOResult& g) {
    FloatParity fp;
    const std::size_t n = std::min(e.solution.solutions.size(), g.solution.solutions.size());
    double sum_sq = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const auto& es = e.solution.solutions[i];
        const auto& gs = g.solution.solutions[i];
        if (es.status != gs.status) {
            ++fp.status_disagreements;
        }
        const bool ef = es.position_ecef.allFinite();
        const bool gf = gs.position_ecef.allFinite();
        if (!ef) ++fp.nonfinite_eigen;
        if (!gf) ++fp.nonfinite_gtsam;
        // Only compare epochs where BOTH produced a usable (non-NONE) finite fix.
        if (!ef || !gf || es.status == libgnss::SolutionStatus::NONE ||
            gs.status == libgnss::SolutionStatus::NONE) {
            continue;
        }
        const double d = (es.position_ecef - gs.position_ecef).norm();
        fp.max_delta_m = std::max(fp.max_delta_m, d);
        sum_sq += d * d;
        ++fp.compared_epochs;
    }
    if (fp.compared_epochs > 0) {
        fp.rms_delta_m = std::sqrt(sum_sq / static_cast<double>(fp.compared_epochs));
    }
    return fp;
}

std::size_t countFixedEpochs(const libgnss::FGOProcessor::FGOResult& r) {
    std::size_t n = 0;
    for (const auto& s : r.solution.solutions) {
        if (s.status == libgnss::SolutionStatus::FIXED) {
            ++n;
        }
    }
    return n;
}

// Tokyo IMU->antenna lever arm, body FLU (docs/imu_fusion.md, project memory);
// used only when the caller asks for the milestone-2a Pose3 GTSAM path.
constexpr double kTokyoLeverArmX = 0.31;
constexpr double kTokyoLeverArmY = 0.0;
constexpr double kTokyoLeverArmZ = 0.55;

libgnss::FGOProcessor::FGOResult run(const libgnss::FGOProcessor::FGOProblem& problem,
                                     libgnss::FGOProcessor::FGOConfig config,
                                     libgnss::FGOBackend backend,
                                     bool use_lambda,
                                     double& seconds,
                                     bool use_pose3 = false,
                                     bool use_imu = false,
                                     double fixed_lag_s = 0.0,
                                      bool use_nhc = false,
                                      bool use_zupt = false,
                                      bool use_hold = false,
                                      bool fixed_lag_qr = false) {
    config.backend = backend;
    config.use_lambda_ambiguity_fix = use_lambda;
    config.fix_ambiguities = use_lambda;
    // Drive BOTH backends through their per-epoch LAMBDA path (fixed-position
    // output) so the fix comparison is apples-to-apples: the native backend's
    // per-epoch fixing and the GTSAM backend's per-epoch fixing both mark
    // epochs FIXED and snap positions to the fixed ambiguities.
    config.use_epoch_lambda_fixed_output = use_lambda;
    config.collect_lambda_debug = false;
    if (use_pose3) {
        // Milestone 2a (docs/gtsam_backend_design.md): key the GTSAM rover
        // state as a body Pose3 + lever arm instead of a bare Point3.
        config.use_pose3_state = true;
        config.pose3_lever_arm_body_m =
            libgnss::Vector3d(kTokyoLeverArmX, kTokyoLeverArmY, kTokyoLeverArmZ);
    }
    // Milestone 2b: enable IMU tight coupling (the problem must already carry
    // a valid ImuInput; the backend checks problem.imu.valid).
    config.use_imu = use_imu;
    // Milestone 2c: enable the incremental fixed-lag smoother.
    if (fixed_lag_s > 0.0) {
        config.use_fixed_lag_smoother = true;
        config.fixed_lag_smoother_lag_s = fixed_lag_s;
        config.fixed_lag_use_qr_factorization = fixed_lag_qr;
    }
    // Milestone 2d: NHC / ZUPT pseudo-measurements.
    config.use_nhc = use_nhc;
    config.use_zupt = use_zupt;
    // Milestone 2e: fix-and-hold ambiguity resolution.
    config.use_ambiguity_hold = use_hold;
    libgnss::FGOProcessor processor(config);
    const auto t0 = std::chrono::high_resolution_clock::now();
    auto result = processor.optimizeProblem(problem);
    const auto t1 = std::chrono::high_resolution_clock::now();
    seconds = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
    return result;
}

// Per-epoch ECEF delta between two results restricted to epochs both mark FIXED.
struct FixedDelta {
    std::size_t both_fixed = 0;
    double max_delta_m = 0.0;
    double rms_delta_m = 0.0;
};

FixedDelta compareFixedPositions(const libgnss::FGOProcessor::FGOResult& e,
                                 const libgnss::FGOProcessor::FGOResult& g) {
    FixedDelta fd;
    const std::size_t n = std::min(e.solution.solutions.size(), g.solution.solutions.size());
    double sum_sq = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const auto& es = e.solution.solutions[i];
        const auto& gs = g.solution.solutions[i];
        if (es.status != libgnss::SolutionStatus::FIXED ||
            gs.status != libgnss::SolutionStatus::FIXED ||
            !es.position_ecef.allFinite() || !gs.position_ecef.allFinite()) {
            continue;
        }
        const double d = (es.position_ecef - gs.position_ecef).norm();
        fd.max_delta_m = std::max(fd.max_delta_m, d);
        sum_sq += d * d;
        ++fd.both_fixed;
    }
    if (fd.both_fixed > 0) {
        fd.rms_delta_m = std::sqrt(sum_sq / static_cast<double>(fd.both_fixed));
    }
    return fd;
}

// --- Milestone 2b IMU plumbing (harness side) ---
//
// Loads imu.csv (already-FLU raw axes, gyro converted to rad/s by loadImuCsv),
// runs the SAME Stage-1 ESKF alignment used by `gnss fuse` to get the initial
// body->ENU attitude + gyro/accel bias, latches heading off the first GNSS
// motion, and packages everything into FGOProblem::ImuInput. Keeping this in
// the harness (rather than the backend) means the GTSAM backend depends only
// on the packaged ImuInput, not on the fusion/ alignment module.
struct RefRow {
    libgnss::GNSSTime time;
    double roll_deg = 0.0, pitch_deg = 0.0, heading_deg = 0.0;
    double ve = 0.0, vn = 0.0, vu = 0.0;
    libgnss::Vector3d ecef = libgnss::Vector3d::Zero();
};

std::vector<RefRow> loadReference(const std::string& path) {
    std::vector<RefRow> rows;
    std::ifstream in(path);
    if (!in) return rows;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> v;
        while (std::getline(ss, cell, ',')) {
            try { v.push_back(std::stod(cell)); } catch (...) { v.push_back(0.0); }
        }
        if (v.size() < 14) continue;
        RefRow r;
        r.time = libgnss::GNSSTime(static_cast<int>(v[1]), v[0]);
        r.ecef = libgnss::Vector3d(v[5], v[6], v[7]);
        r.roll_deg = v[8]; r.pitch_deg = v[9]; r.heading_deg = v[10];
        r.ve = v[11]; r.vn = v[12]; r.vu = v[13];
        rows.push_back(r);
    }
    return rows;
}

// Horizontal (E/N) position error of a result vs reference.csv, split by
// FLOAT vs FIXED solution status. Matches epochs to reference rows by time.
struct HorizError {
    std::size_t n_float = 0, n_fixed = 0;
    double float_rms = 0.0, float_max = 0.0;
    double fixed_rms = 0.0, fixed_max = 0.0;
    double frac_all_under_50cm = 0.0;  ///< fraction of all (float+fixed) epochs with horiz<0.5 m
    // Diagnostics only (populated regardless; cheap): FIXED-epoch horizontal
    // error histogram + the worst offenders, so we can tell whether FIXED RMS
    // mass comes from a broad inflation or a handful of gross outliers.
    std::array<std::size_t, 6> fixed_hist{};  // bins: <0.5, 0.5-1, 1-2, 2-5, 5-10, >=10 m
    std::array<double, 6> fixed_hist_sumsq{};  // sum(horiz^2) per bin -- where the RMS mass sits
    std::vector<std::pair<double, double>> fixed_worst;  // (time.tow-ish sod, horiz), top 15
};

HorizError horizontalErrorVsRef(const libgnss::FGOProcessor::FGOResult& r,
                                const std::vector<RefRow>& ref) {
    HorizError he;
    if (ref.empty()) return he;
    double lat0 = 0.0, lon0 = 0.0, h0 = 0.0;
    libgnss::ecef2geodetic(ref.front().ecef, lat0, lon0, h0);
    double fsum = 0.0, xsum = 0.0;
    std::size_t ri = 0;  // reference cursor (both streams time-sorted)
    std::size_t n_all = 0, n_all_under50 = 0;
    for (const auto& s : r.solution.solutions) {
        if (s.status == libgnss::SolutionStatus::NONE || !s.position_ecef.allFinite()) continue;
        // Advance reference cursor to the nearest row in time.
        while (ri + 1 < ref.size() &&
               std::abs(ref[ri + 1].time - s.time) < std::abs(ref[ri].time - s.time)) {
            ++ri;
        }
        if (std::abs(ref[ri].time - s.time) > 0.11) continue;
        const libgnss::Vector3d enu =
            libgnss::ecef2enu(s.position_ecef - ref[ri].ecef, lat0, lon0);
        const double horiz = std::hypot(enu.x(), enu.y());
        ++n_all;
        if (horiz < 0.5) ++n_all_under50;
        if (s.status == libgnss::SolutionStatus::FIXED) {
            xsum += horiz * horiz;
            he.fixed_max = std::max(he.fixed_max, horiz);
            ++he.n_fixed;
            std::size_t bin = 0;
            if (horiz < 0.5) bin = 0;
            else if (horiz < 1.0) bin = 1;
            else if (horiz < 2.0) bin = 2;
            else if (horiz < 5.0) bin = 3;
            else if (horiz < 10.0) bin = 4;
            else bin = 5;
            ++he.fixed_hist[bin];
            he.fixed_hist_sumsq[bin] += horiz * horiz;
            he.fixed_worst.emplace_back(s.time.tow, horiz);
        } else {
            fsum += horiz * horiz;
            he.float_max = std::max(he.float_max, horiz);
            ++he.n_float;
        }
    }
    if (he.n_float > 0) he.float_rms = std::sqrt(fsum / double(he.n_float));
    if (he.n_fixed > 0) he.fixed_rms = std::sqrt(xsum / double(he.n_fixed));
    if (n_all > 0) he.frac_all_under_50cm = double(n_all_under50) / double(n_all);
    std::sort(he.fixed_worst.begin(), he.fixed_worst.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });
    if (he.fixed_worst.size() > 15) he.fixed_worst.resize(15);
    return he;
}

// Debug/plotting aid: one row per solved epoch (status != NONE), reusing the
// exact same nearest-in-time reference cursor as horizontalErrorVsRef() so the
// per-epoch numbers are consistent with the printed headline metrics. Columns:
//   tow,status,e_err_m,n_err_m,u_err_m,horiz_err_m,e_pos_m,n_pos_m,u_pos_m,
//   x_ecef_m,y_ecef_m,z_ecef_m,position_covariance_trace_m2,
//   ref_e_pos_m,ref_n_pos_m,ref_u_pos_m
// where *_err_m is the ENU delta vs the time-matched reference.csv row (the
// same quantity the fix-rate/RMS metrics are computed from) and *_pos_m is the
// solution/reference position in local ENU relative to the trajectory start
// (ref.front()), for ground-track plots. status is FIXED/FLOAT after all
// demotions -- i.e. exactly the label horizontalErrorVsRef() buckets on.
void dumpEpochCsv(const libgnss::FGOProcessor::FGOResult& r,
                  const std::vector<RefRow>& ref,
                  const std::string& path) {
    if (ref.empty()) {
        std::cerr << "Warning: --dump-csv requires --ref reference.csv; skipping dump.\n";
        return;
    }
    std::ofstream out(path);
    if (!out) {
        std::cerr << "Error: cannot open --dump-csv output file " << path << "\n";
        return;
    }
    double lat0 = 0.0, lon0 = 0.0, h0 = 0.0;
    libgnss::ecef2geodetic(ref.front().ecef, lat0, lon0, h0);
    // Fixed-point with millisecond/millimeter resolution: default ostream
    // precision (6 significant digits) truncates sub-second tow at these
    // magnitudes (~1.8e5), collapsing 0.2s epochs onto the same displayed tow.
    out << std::fixed;
    out.precision(3);
    out << "tow,status,e_err_m,n_err_m,u_err_m,horiz_err_m,e_pos_m,n_pos_m,u_pos_m,"
           "x_ecef_m,y_ecef_m,z_ecef_m,position_covariance_trace_m2,"
           "ref_e_pos_m,ref_n_pos_m,ref_u_pos_m,"
           "vel_e_mps,vel_n_mps,vel_u_mps,"
           "ratio,ratio_threshold,nfixed,ar_outcome,ddpr_rms_m,sd_doppler_rms_mps,gdop,nsat,sd_doppler_n,"
           "amb_candidates,lambda_attempts,lambda_stage,amb_var_median,amb_var_max,"
           "imu_pose_correction_m,ddpr_anchor_eval,ddpr_anchor_n,ddpr_anchor_res_m,"
           "ddpr_anchor_x_ecef_m,ddpr_anchor_y_ecef_m,ddpr_anchor_z_ecef_m,"
           "ddpr_anchor_h_err_m,ddpr_anchor_u_err_m,ddpr_anchor_prior,"
           "fixed_float_sep_m,fixed_imu_pred_sep_m,fixed_postfit_ddcp_rms_m,fixed_postfit_ddcp_max_norm,fixed_postfit_ddcp_chi2_dof,fixed_postfit_ddcp_n,external_dr_sep_m,external_dr_mahal2,external_dr_age,external_doppler_valid,external_doppler_vel_e_mps,external_doppler_vel_n_mps,external_doppler_vel_u_mps,external_dr_eval,external_dr_accept,external_dr_reject,cp_hold,"
           "imu_aperture_accept,imu_aperture_reject,cp_available,cp_added,"
           "cp_suppressed_hold,amb_after_hold,amb_final,amb_excl_build,"
           "amb_excl_hold,amb_excl_one_band,amb_excl_constellation,"
           "amb_excl_prev_residual,amb_excl_fde,amb_excl_stale,"
           "gen_bump_hold,gen_bump_fde,gen_bump_reset,"
           "gen_bump_warm_reset,gen_bump_stale_pin,"
           "surplus_eval,surplus_pass,surplus_level,surplus_used,surplus_rescue,surplus_veto,"
           "low_count_attempted,low_count_used,"
           "integer_reopt_eval,integer_reopt_pass,integer_reopt_base_cost_before,"
           "integer_reopt_base_cost_after,integer_reopt_base_cost_delta,"
           "dr_bypass_eval,dr_bypass_horiz_err_m,dr_bypass_applied\n";
    std::size_t ri = 0;
    for (std::size_t si = 0; si < r.solution.solutions.size(); ++si) {
        const auto& s = r.solution.solutions[si];
        if (s.status == libgnss::SolutionStatus::NONE || !s.position_ecef.allFinite()) continue;
        while (ri + 1 < ref.size() &&
               std::abs(ref[ri + 1].time - s.time) < std::abs(ref[ri].time - s.time)) {
            ++ri;
        }
        if (std::abs(ref[ri].time - s.time) > 0.11) continue;
        const libgnss::Vector3d err_enu =
            libgnss::ecef2enu(s.position_ecef - ref[ri].ecef, lat0, lon0);
        const libgnss::Vector3d pos_enu =
            libgnss::ecef2enu(s.position_ecef - ref.front().ecef, lat0, lon0);
        const libgnss::Vector3d ref_pos_enu =
            libgnss::ecef2enu(ref[ri].ecef - ref.front().ecef, lat0, lon0);
        const double horiz = std::hypot(err_enu.x(), err_enu.y());
        const char* status =
            s.status == libgnss::SolutionStatus::FIXED ? "FIXED" : "FLOAT";
        const libgnss::Vector3d velocity_nav =
            si < r.epoch_velocity_nav_mps.size()
                ? r.epoch_velocity_nav_mps[si]
                : libgnss::Vector3d::Zero();
        // Fix 3 (agent/realtime-fix-integrity follow-up): a non-positive (or
        // exactly all-zero) covariance is never a genuine position
        // uncertainty for any real KF/FGO estimator -- it means this
        // solution path never populated position_covariance (PositionSolution
        // leaves it default-constructed/uninitialized). Emitting 0.000 in
        // that case is indistinguishable downstream from a real sub-
        // millimetre estimate; ShadowEstimateHealthGate on the consuming
        // side then (correctly) either trusts a fake perfect covariance or,
        // once fixed, treats a bare 0.0 as unpopulated -- so the CSV must
        // carry "missing", not "zero", here. See
        // output/ppc_realtime_fix_integrity_matrix.md's "After fix" section
        // (Gap 2) for the regression this closes.
        const double covariance_trace = s.position_covariance.trace();
        const bool covariance_populated = std::isfinite(covariance_trace) &&
            covariance_trace > 0.0 &&
            !(s.position_covariance.array() == 0.0).all();
        out << s.time.tow << ',' << status << ','
            << err_enu.x() << ',' << err_enu.y() << ',' << err_enu.z() << ','
            << horiz << ',' << pos_enu.x() << ',' << pos_enu.y() << ',' << pos_enu.z() << ','
            << s.position_ecef.x() << ',' << s.position_ecef.y() << ',' << s.position_ecef.z() << ',';
        if (covariance_populated) {
            out << covariance_trace;
        }
        out << ','
            << ref_pos_enu.x() << ',' << ref_pos_enu.y() << ',' << ref_pos_enu.z() << ','
            << velocity_nav.x() << ',' << velocity_nav.y() << ',' << velocity_nav.z() << ','
            << s.ratio;
        if (si < r.epoch_diagnostics.size()) {
            const auto& d = r.epoch_diagnostics[si];
            out << ',' << d.effective_ratio_threshold
                << ',' << s.num_fixed_ambiguities
                << ',' << static_cast<int>(d.ar_outcome)
                << ',' << d.ddpr_rms_m << ',' << d.sd_doppler_rms_mps
                << ',' << d.gdop << ',' << d.num_satellites
                << ',' << d.sd_doppler_factors
                << ',' << d.ambiguity_candidates << ',' << d.lambda_attempts
                << ',' << d.lambda_selected_stage
                << ',' << d.ambiguity_variance_median_cycles2
                << ',' << d.ambiguity_variance_max_cycles2
                << ',' << d.imu_pose_correction_m;
            double anchor_horiz = -1.0;
            double anchor_up = -1.0;
            if (d.ddpr_anchor_evaluated && d.ddpr_anchor_position_ecef.norm() > 1e6) {
                const libgnss::Vector3d anchor_err_enu = libgnss::ecef2enu(
                    d.ddpr_anchor_position_ecef - ref[ri].ecef, lat0, lon0);
                anchor_horiz = std::hypot(anchor_err_enu.x(), anchor_err_enu.y());
                anchor_up = anchor_err_enu.z();
            }
            const libgnss::Vector3d external_doppler_velocity_nav =
                d.external_doppler_velocity_valid
                    ? libgnss::ecef2enu(
                          d.external_doppler_velocity_ecef_mps, lat0, lon0)
                    : libgnss::Vector3d::Zero();
            out << ',' << (d.ddpr_anchor_evaluated ? 1 : 0)
                << ',' << d.ddpr_anchor_active_factors
                << ',' << d.ddpr_anchor_residual_rms_m
                << ',' << d.ddpr_anchor_position_ecef.x()
                << ',' << d.ddpr_anchor_position_ecef.y()
                << ',' << d.ddpr_anchor_position_ecef.z()
                << ',' << anchor_horiz
                << ',' << anchor_up
                << ',' << (d.ddpr_anchor_bootstrap_prior_applied ? 1 : 0)
                << ',' << d.fixed_float_separation_m
                << ',' << d.fixed_imu_prediction_separation_m
                << ',' << d.fixed_postfit_ddcp_rms_m
                << ',' << d.fixed_postfit_ddcp_max_normalized
                << ',' << d.fixed_postfit_ddcp_chi2_per_dof
                << ',' << d.fixed_postfit_ddcp_factors
                << ',' << d.external_dr_separation_m
                << ',' << d.external_dr_mahalanobis2
                << ',' << d.external_dr_age_epochs
                << ',' << (d.external_doppler_velocity_valid ? 1 : 0)
                << ',' << external_doppler_velocity_nav.x()
                << ',' << external_doppler_velocity_nav.y()
                << ',' << external_doppler_velocity_nav.z()
                << ',' << (d.external_dr_evaluated ? 1 : 0)
                << ',' << (d.external_dr_accepted ? 1 : 0)
                << ',' << (d.external_dr_rejected ? 1 : 0)
                << ',' << (d.carrier_hold_active ? 1 : 0)
                << ',' << (d.imu_aperture_accepted ? 1 : 0)
                << ',' << (d.imu_aperture_rejected ? 1 : 0)
                << ',' << d.carrier_factors_available
                << ',' << d.carrier_factors_added
                << ',' << d.carrier_factors_suppressed_hold
                << ',' << d.ambiguity_candidates_after_hold
                << ',' << d.ambiguity_candidates_final
                << ',' << d.ambiguity_candidates_excluded_build_time
                << ',' << d.ambiguity_candidates_excluded_hold
                << ',' << d.ambiguity_candidates_excluded_one_band
                << ',' << d.ambiguity_candidates_excluded_constellation
                << ',' << d.ambiguity_candidates_excluded_previous_residual
                << ',' << d.ambiguity_candidates_excluded_fde
                << ',' << d.ambiguity_candidates_excluded_stale
                << ',' << d.ambiguity_generation_bumps_hold
                << ',' << d.ambiguity_generation_bumps_fde
                << ',' << d.ambiguity_generation_bumps_reset
                << ',' << d.ambiguity_generation_bumps_warm_reset
                << ',' << d.ambiguity_generation_bumps_stale_pin
                << ',' << (d.surplus_validation_evaluated ? 1 : 0)
                << ',' << (d.surplus_validation_pass ? 1 : 0)
                << ',' << d.surplus_validation_fallback_level
                << ',' << d.surplus_validation_surplus_used
                << ',' << (d.surplus_validation_used_for_rescue ? 1 : 0)
                << ',' << (d.surplus_validation_used_for_veto ? 1 : 0)
                << ',' << (d.low_count_ar_attempted ? 1 : 0)
                << ',' << (d.low_count_ar_used ? 1 : 0)
                << ',' << (d.integer_constrained_reoptimization_evaluated ? 1 : 0)
                << ',' << (d.integer_constrained_reoptimization_pass ? 1 : 0)
                << ',' << d.integer_constrained_base_cost_before
                << ',' << d.integer_constrained_base_cost_after
                << ',' << (d.integer_constrained_base_cost_after -
                             d.integer_constrained_base_cost_before);
            double dr_bypass_horiz = -1.0;
            if (d.dr_bypass_candidate_evaluated) {
                const libgnss::Vector3d dr_bypass_err_enu = libgnss::ecef2enu(
                    d.dr_bypass_candidate_position_ecef - ref[ri].ecef, lat0, lon0);
                dr_bypass_horiz =
                    std::hypot(dr_bypass_err_enu.x(), dr_bypass_err_enu.y());
            }
            out << ',' << (d.dr_bypass_candidate_evaluated ? 1 : 0)
                << ',' << dr_bypass_horiz
                << ',' << (d.dr_bypass_applied ? 1 : 0);
        }
        out << '\n';
    }

    // Normalized companion table: the epoch CSV above answers "how many",
    // while this row-per-satellite/signal trace answers exactly which carrier
    // row left the LAMBDA funnel and at which existing gate.  It deliberately
    // shares --dump-csv rather than adding another command-line switch.
    const std::string trace_path = path + ".ar_candidates.csv";
    std::ofstream trace_out(trace_path);
    if (!trace_out) {
        std::cerr << "Warning: cannot open AR candidate trace output file "
                  << trace_path << "\n";
        return;
    }
    trace_out << "tow,satellite,reference_satellite,system,prn,"
                 "reference_system,reference_prn,signal,ambiguity_index,disposition\n";
    trace_out << std::fixed;
    trace_out.precision(3);
    for (const auto& epoch : r.epoch_diagnostics) {
        for (const auto& candidate : epoch.ambiguity_candidate_trace) {
            trace_out << epoch.time.tow << ','
                      << candidate.satellite.toString() << ','
                      << candidate.reference_satellite.toString() << ','
                      << static_cast<int>(candidate.satellite.system) << ','
                      << static_cast<int>(candidate.satellite.prn) << ','
                      << static_cast<int>(candidate.reference_satellite.system) << ','
                      << static_cast<int>(candidate.reference_satellite.prn) << ','
                      << static_cast<int>(candidate.signal) << ',';
            if (candidate.ambiguity_index !=
                std::numeric_limits<std::size_t>::max()) {
                trace_out << candidate.ambiguity_index;
            }
            trace_out << ',' << static_cast<int>(candidate.disposition) << '\n';
        }
    }

    // One row per concrete LAMBDA subset attempt.  This is a shadow ledger:
    // FFRT pass flags are counterfactual only and never feed the solver's
    // existing static ratio/integrity decision.
    const std::string attempts_path = path + ".ar_attempts.csv";
    std::ofstream attempts_out(attempts_path);
    if (!attempts_out) {
        std::cerr << "Warning: cannot open AR attempt trace output file "
                  << attempts_path << "\n";
        return;
    }
    attempts_out
        << "tow,attempt_index,subset_size,lambda_stage,lambda_search_solved,"
           "ratio,candidate_position_valid,candidate_x_ecef_m,"
           "candidate_y_ecef_m,candidate_z_ecef_m,fixed_float_sep_m,"
           "fixed_imu_pred_sep_m,configured_ratio_pass,effective_ratio_pass";
    for (const int scale : {1, 2, 4, 8, 16}) {
        attempts_out << ",bsr_q" << scale
                     << ",ffrt_min_ratio_q" << scale
                     << ",ffrt_supported_q" << scale
                     << ",ffrt_accepts_any_candidate_q" << scale
                     << ",ffrt_pass_q" << scale;
    }
    attempts_out
        << ",surplus_eval,surplus_pass,surplus_level,surplus_used,"
           "postfit_ddcp_n,postfit_ddcp_rms_m,postfit_ddcp_max_norm,"
           "postfit_ddcp_chi2_dof,static_fixed_decision_pass,"
           "accepted_by_existing_pipeline\n";
    attempts_out.precision(17);
    for (const auto& epoch : r.epoch_diagnostics) {
        for (const auto& attempt :
             epoch.ambiguity_resolution_attempt_trace) {
            attempts_out
                << epoch.time.tow << ',' << attempt.attempt_index << ','
                << attempt.subset_size << ',' << attempt.lambda_stage << ','
                << (attempt.lambda_search_solved ? 1 : 0) << ','
                << attempt.ratio << ','
                << (attempt.candidate_position_valid ? 1 : 0) << ',';
            if (attempt.candidate_position_valid) {
                attempts_out
                    << attempt.candidate_position_ecef.x() << ','
                    << attempt.candidate_position_ecef.y() << ','
                    << attempt.candidate_position_ecef.z();
            } else {
                attempts_out << ",,";
            }
            attempts_out
                << ',' << attempt.fixed_float_separation_m
                << ',' << attempt.fixed_imu_prediction_separation_m
                << ',' << (attempt.configured_ratio_pass ? 1 : 0)
                << ',' << (attempt.effective_ratio_pass ? 1 : 0);
            for (std::size_t scale_index = 0;
                 scale_index <
                 libgnss::FGOProcessor::
                     AmbiguityResolutionAttemptTrace::
                         covariance_scales.size();
                 ++scale_index) {
                attempts_out
                    << ','
                    << attempt.bootstrapped_success_rate[scale_index]
                    << ','
                    << attempt.ffrt_minimum_ratio[scale_index]
                    << ','
                    << (attempt.ffrt_supported[scale_index] ? 1 : 0)
                    << ','
                    << (attempt.ffrt_accepts_any_candidate[scale_index]
                            ? 1
                            : 0)
                    << ','
                    << (attempt.ffrt_pass[scale_index] ? 1 : 0);
            }
            attempts_out
                << ',' << (attempt.surplus_evaluated ? 1 : 0)
                << ',' << (attempt.surplus_pass ? 1 : 0)
                << ',' << attempt.surplus_fallback_level
                << ',' << attempt.surplus_used
                << ',' << attempt.postfit_ddcp_factors
                << ',' << attempt.postfit_ddcp_rms_m
                << ',' << attempt.postfit_ddcp_max_normalized
                << ',' << attempt.postfit_ddcp_chi2_per_dof
                << ','
                << (attempt.static_fixed_decision_pass ? 1 : 0)
                << ','
                << (attempt.accepted_by_existing_pipeline ? 1 : 0)
                << '\n';
        }
    }
}

// Populate problem.imu from an imu.csv + the already-built FGOProblem epochs.
// Returns false (and leaves problem.imu.valid=false) if anything is missing.
bool buildImuInput(const std::string& imu_path,
                   libgnss::FGOProcessor::FGOProblem& problem) {
    if (problem.epochs.size() < 2) return false;
    libgnss::ImuSeries imu_series;
    const auto load = libgnss::loadImuCsv(imu_path, imu_series);
    if (!load.ok || imu_series.isEmpty()) {
        std::cerr << "Error: cannot load IMU csv " << imu_path
                  << " (" << load.error << ")\n";
        return false;
    }
    // tokyo imu.csv is already body FLU (identity axis convention, same default
    // as gnss_fuse.cpp) with gyro converted to rad/s at load time.
    const libgnss::ImuAxisConvention axis;  // identity
    for (auto& s : imu_series.samples) {
        s.accel_raw = axis.apply(s.accel_raw);
        s.gyro_raw_radps = axis.apply(s.gyro_raw_radps);
    }

    auto& imu = problem.imu;
    imu.samples_body_flu = imu_series.samples;

    // Nav (ENU) origin: first epoch antenna ECEF -> geodetic lat/lon.
    const libgnss::Vector3d origin_ecef = problem.epochs.front().position_ecef;
    double lat = 0.0, lon = 0.0, h = 0.0;
    libgnss::ecef2geodetic(origin_ecef, lat, lon, h);
    imu.nav_origin_ecef = origin_ecef;
    imu.nav_origin_lat_rad = lat;
    imu.nav_origin_lon_rad = lon;

    // Stage-1 static leveling, reusing fusion_initialization::alignStatic.
    // Always use the original run's leading static IMU window.  In a
    // --start-epoch diagnostic replay, the first retained GNSS epoch can be
    // mid-motion and must never be mistaken for a leveling interval.
    const libgnss::GNSSTime t_first = problem.epochs.front().time;
    const bool midrun_start =
        !imu.samples_body_flu.empty() && (t_first - imu.samples_body_flu.front().time) > 5.0;
    std::vector<libgnss::ImuSample> stationary;
    for (const auto& s : imu.samples_body_flu) {
        if (stationary.size() >= 250) break;  // ~2.5 s at 100 Hz
        stationary.push_back(s);
    }
    const libgnss::NominalState aligned = libgnss::fusion_initialization::alignStatic(
        stationary, libgnss::Vector3d::Zero(), imu.noise.gravity_mps2);

    // Heading latch from a sustained, windowed GNSS course.  A consecutive
    // 0.2-s position difference is far too sensitive to urban code noise: on
    // Nagoya run1 it falsely latched a stationary outlier as
    // [10,18,-40] m/s.  Use a 5-s displacement and require three consistent
    // horizontal course estimates before accepting yaw.
    libgnss::FusionState fstate;
    fstate.nominal = aligned;
    // Both PPC runs provide a static leveling window at the start.  Heading
    // may be learned from later motion, but that later velocity must never be
    // copied back into the initial state.
    libgnss::Vector3d init_vel_enu = libgnss::Vector3d::Zero();
    bool heading_latched = false;
    constexpr std::size_t kCourseWindowEpochs = 25;  // 5 s at 5 Hz
    libgnss::Vector3d previous_course_velocity = libgnss::Vector3d::Zero();
    libgnss::Vector3d course_velocity_sum = libgnss::Vector3d::Zero();
    int consistent_course_count = 0;
    for (std::size_t i = 0; i + kCourseWindowEpochs < problem.epochs.size(); ++i) {
        const std::size_t j = i + kCourseWindowEpochs;
        const double dt = problem.epochs[j].time - problem.epochs[i].time;
        if (dt <= 1e-3) continue;
        const libgnss::Vector3d enu0 = libgnss::ecef2enu(
            problem.epochs[i].position_ecef - origin_ecef, lat, lon);
        const libgnss::Vector3d enu1 = libgnss::ecef2enu(
            problem.epochs[j].position_ecef - origin_ecef, lat, lon);
        const libgnss::Vector3d vel = (enu1 - enu0) / dt;
        const double horizontal_speed = std::hypot(vel.x(), vel.y());
        const bool plausible = horizontal_speed >= 2.0 && horizontal_speed <= 50.0 &&
                               std::abs(vel.z()) <= 3.0;
        if (!plausible) {
            consistent_course_count = 0;
            course_velocity_sum.setZero();
            continue;
        }
        bool consistent = true;
        if (consistent_course_count > 0) {
            const double prev_speed = std::hypot(previous_course_velocity.x(),
                                                 previous_course_velocity.y());
            const double cosine = (vel.x() * previous_course_velocity.x() +
                                   vel.y() * previous_course_velocity.y()) /
                                  std::max(1e-9, horizontal_speed * prev_speed);
            consistent = cosine >= std::cos(20.0 * 3.14159265358979323846 / 180.0);
        }
        if (!consistent) {
            consistent_course_count = 0;
            course_velocity_sum.setZero();
        }
        previous_course_velocity = vel;
        course_velocity_sum += vel;
        ++consistent_course_count;
        if (consistent_course_count >= 3) {
            const libgnss::Vector3d course_velocity =
                course_velocity_sum / static_cast<double>(consistent_course_count);
            if (libgnss::fusion_initialization::tryAlignHeading(
                    fstate, course_velocity, 1.0, 5.0)) {
                if (midrun_start) {
                    // A segment replay starts in motion, unlike a full run's
                    // static epoch zero.  Seed its velocity from the same
                    // sustained course used for heading; full runs retain the
                    // intentional zero-velocity initialization.
                    init_vel_enu = course_velocity;
                }
                heading_latched = true;
                break;
            }
        }
    }

    imu.init_attitude_body_to_nav = fstate.nominal.attitude_body_to_enu.toRotationMatrix();
    imu.init_velocity_nav = init_vel_enu;
    imu.init_accel_bias = aligned.accel_bias;
    imu.init_gyro_bias = aligned.gyro_bias;
    // Low-cost MEMS-ish noise (order of the tokyo low-cost preset); 2b is a
    // wiring/sanity gate, tuning is 2c/2e.
    // Tighter than the initial 2b sanity setting so the IMU actually bridges
    // weak-GNSS urban stretches (still conservative MEMS-grade; deep tuning is
    // 2e). Loose IMU let the float wander to metres in canyon; tightening pulls
    // it back toward the IMU-predicted trajectory between good fixes.
    imu.noise.accel_noise_sigma = 2.0e-2;
    imu.noise.gyro_noise_sigma = 2.0e-3;
    imu.noise.accel_bias_rw_sigma = 3.0e-3;
    imu.noise.gyro_bias_rw_sigma = 3.0e-4;
    imu.noise.integration_sigma = 1.0e-3;
    imu.init_attitude_sigma_roll_pitch_rad = 0.05;
    imu.init_attitude_sigma_yaw_rad = heading_latched ? 0.15 : 3.0;
    imu.init_velocity_sigma_mps = 0.5;
    imu.init_accel_bias_sigma = 0.1;
    imu.init_gyro_bias_sigma = 0.01;
    imu.valid = true;

    {
        // Epoch/IMU cadence sanity (helps diagnose preintegration dt issues).
        const double dt_epoch = problem.epochs.size() >= 2
                                    ? (problem.epochs[1].time - problem.epochs[0].time)
                                    : 0.0;
        double dt_imu = 0.0;
        for (std::size_t k = 1; k < imu.samples_body_flu.size() && k < 5; ++k) {
            dt_imu = imu.samples_body_flu[k].time - imu.samples_body_flu[k - 1].time;
        }
        std::cout << "IMU cadence: epoch dt=" << dt_epoch << " s, imu dt~=" << dt_imu
                  << " s; epoch0.tow=" << problem.epochs[0].time.tow
                  << " imu0.tow=" << imu.samples_body_flu[0].time.tow
                  << " (weeks e" << problem.epochs[0].time.week << "/i"
                  << imu.samples_body_flu[0].time.week << ")\n";
    }
    std::cout << "IMU: loaded " << imu.samples_body_flu.size() << " samples, stationary_window="
              << stationary.size() << ", heading_latched=" << (heading_latched ? "yes" : "no")
              << "\n  init attitude(body->ENU) roll/pitch from leveling; init_vel_enu=["
              << init_vel_enu.transpose() << "] m/s\n"
              << "  init_gyro_bias=[" << aligned.gyro_bias.transpose() << "] rad/s"
              << " init_accel_bias=[" << aligned.accel_bias.transpose() << "] m/s^2\n";
    return true;
}

// H1: apply CLI IMU noise overrides on top of buildImuInput()'s defaults.
// --imu-preset-tactical sets all four to the reference's IMU_PRESETS['tactical']
// (gnss_fgo/config.py ~line 32); individual --imu-accel-noise etc. win over the
// preset when both are given, so a preset + single-field sweep composes cleanly.
void applyImuNoiseOverrides(const Args& args, libgnss::FGOProcessor::FGOProblem& problem) {
    if (args.imu_preset_tactical) {
        problem.imu.noise.accel_noise_sigma = 2.84e-4;
        problem.imu.noise.gyro_noise_sigma = 4.01e-5;
        problem.imu.noise.accel_bias_rw_sigma = 3.14e-4;
        problem.imu.noise.gyro_bias_rw_sigma = 9.70e-6;
    }
    if (args.imu_accel_noise >= 0.0) problem.imu.noise.accel_noise_sigma = args.imu_accel_noise;
    if (args.imu_gyro_noise >= 0.0) problem.imu.noise.gyro_noise_sigma = args.imu_gyro_noise;
    if (args.imu_accel_bias >= 0.0) problem.imu.noise.accel_bias_rw_sigma = args.imu_accel_bias;
    if (args.imu_gyro_bias >= 0.0) problem.imu.noise.gyro_bias_rw_sigma = args.imu_gyro_bias;
    // Gravity measured variant (see Args::gravity_mps2): the reference's
    // utils/imu.py:39 uses gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);
    // buildImuInput() here defaults to 9.80665. Not applied silently -- only
    // when --gravity is passed.
    if (args.gravity_mps2 >= 0.0) problem.imu.noise.gravity_mps2 = args.gravity_mps2;
}

// ---------------------------------------------------------------------------
// --problem-cache: opt-in FGOProblem cache.
//
// Skips the RINEX parse (rover+base+nav) and buildDoubleDifferenceProblem()
// call -- the dominant cost of a validation run before the fixed-lag solve
// itself -- when an on-disk cache already holds the identical problem for
// identical inputs+config. Every type reachable from FGOProblem (down through
// PseudorangeFactor, DoubleDifferenceCarrierFactor, AmbiguityState, EpochSeed,
// ObservationModelDebug, SatelliteId, GNSSTime, Eigen::Vector3d/Matrix3d, ...)
// is a flat POD struct: no pointers, no std::string, no std::map. A generic
// length-prefixed raw-bytes writer/reader therefore suffices for every vector
// member; std::vector<bool> (FGOProblem::clock_jumps) is the one exception
// (bit-packed, not a raw T[]) and gets its own byte-per-flag helpers.
//
// Scope: only the pre-IMU FGOProblem (RINEX parse + DD factor construction)
// is cached. buildImuInput()/applyImuNoiseOverrides() (imu.csv parse + static
// leveling/heading-latch) still run unconditionally on every invocation --
// they are a small linear pass next to the RINEX/DD cost above, so caching
// them isn't worth the extra serialization surface.
//
// Fingerprint: format version + size/mtime of the 4 input files + only the
// config fields that affect buildPseudorangeProblem/buildDoubleDifferenceProblem,
// stored in a zero-filled byte snapshot, plus the Args fields that affect
// the SAME build phase's epoch selection (--max-epochs/--start-epoch). Solver-
// only knobs (FDE, AR validation, held-status reporting, etc.) deliberately do
// not invalidate the cached input problem, which makes controlled A/B sweeps
// both faster and guarantees that they optimize byte-identical factors. Any
// mismatch (missing/changed file, different knob, different format version)
// -> loud stderr warning + full rebuild; the cache is never silently stale.
namespace problem_cache {

// NOTE on std::is_trivially_copyable: every FGOProblem-reachable type here is
// a flat struct of scalars / SatelliteId / GNSSTime / fixed-size Eigen
// vectors-matrices -- safe to round-trip via raw bytes. They are NOT,
// however, all std::is_trivially_copyable: Eigen's fixed-size types (and
// GNSSTime, which wraps a chrono conversion helper) declare their own
// (semantically trivial) copy constructor/assignment for expression-template
// support, which disqualifies them from that strict trait even though a
// memcpy is exactly what those constructors do. So this intentionally does
// NOT static_assert on is_trivially_copyable; standard_layout is checked
// instead as a much weaker, but still useful, guard against accidentally
// pointing this at a type with a std::string/std::vector/pointer member.
template <typename T>
void writePod(std::ostream& os, const T& v) {
    static_assert(std::is_standard_layout_v<T>, "writePod requires a standard-layout (flat POD-like) type");
    os.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

template <typename T>
bool readPod(std::istream& is, T& v) {
    static_assert(std::is_standard_layout_v<T>, "readPod requires a standard-layout (flat POD-like) type");
    is.read(reinterpret_cast<char*>(&v), sizeof(T));
    return static_cast<bool>(is);
}

template <typename T>
void writeVec(std::ostream& os, const std::vector<T>& v) {
    static_assert(std::is_standard_layout_v<T>, "writeVec requires a standard-layout (flat POD-like) element type");
    const uint64_t n = v.size();
    writePod(os, n);
    if (n != 0) {
        os.write(reinterpret_cast<const char*>(v.data()), static_cast<std::streamsize>(sizeof(T) * n));
    }
}

template <typename T>
bool readVec(std::istream& is, std::vector<T>& v) {
    static_assert(std::is_standard_layout_v<T>, "readVec requires a standard-layout (flat POD-like) element type");
    uint64_t n = 0;
    if (!readPod(is, n)) return false;
    if (n > (1ull << 34)) return false;  // sanity cap against a corrupt/foreign file
    v.assign(static_cast<std::size_t>(n), T{});
    if (n != 0) {
        is.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(sizeof(T) * n));
    }
    return static_cast<bool>(is);
}

// vector<bool> is bit-packed, not a raw array the tricks above can alias --
// give it a dedicated byte-per-flag round trip.
void writeBoolVec(std::ostream& os, const std::vector<bool>& v) {
    const uint64_t n = v.size();
    writePod(os, n);
    for (bool b : v) {
        const uint8_t byte = b ? 1 : 0;
        writePod(os, byte);
    }
}

bool readBoolVec(std::istream& is, std::vector<bool>& v) {
    uint64_t n = 0;
    if (!readPod(is, n)) return false;
    if (n > (1ull << 34)) return false;
    v.assign(static_cast<std::size_t>(n), false);
    for (uint64_t i = 0; i < n; ++i) {
        uint8_t byte = 0;
        if (!readPod(is, byte)) return false;
        v[static_cast<std::size_t>(i)] = (byte != 0);
    }
    return true;
}

void writeProblem(std::ostream& os, const libgnss::FGOProcessor::FGOProblem& p) {
    writeVec(os, p.epochs);
    writeBoolVec(os, p.clock_jumps);
    writeVec(os, p.pseudorange_factors);
    writeVec(os, p.tdcp_factors);
    writeVec(os, p.single_difference_doppler_factors);
    writeVec(os, p.single_difference_tdcp_factors);
    writeVec(os, p.ambiguity_states);
    writeVec(os, p.carrier_observations);
    writeVec(os, p.double_difference_pseudorange_observations);
    writeVec(os, p.double_difference_reference_observations);
    writeVec(os, p.carrier_phase_factors);
    writeVec(os, p.double_difference_pseudorange_factors);
    writeVec(os, p.double_difference_carrier_factors);
    writeVec(os, p.excluded_double_difference_carrier_factors);
    writeVec(os, p.ambiguity_between_factors);
    writePod(os, p.diagnostics);
}

bool readProblem(std::istream& is, libgnss::FGOProcessor::FGOProblem& p) {
    return readVec(is, p.epochs) &&
           readBoolVec(is, p.clock_jumps) &&
           readVec(is, p.pseudorange_factors) &&
           readVec(is, p.tdcp_factors) &&
           readVec(is, p.single_difference_doppler_factors) &&
           readVec(is, p.single_difference_tdcp_factors) &&
           readVec(is, p.ambiguity_states) &&
           readVec(is, p.carrier_observations) &&
           readVec(is, p.double_difference_pseudorange_observations) &&
           readVec(is, p.double_difference_reference_observations) &&
           readVec(is, p.carrier_phase_factors) &&
           readVec(is, p.double_difference_pseudorange_factors) &&
           readVec(is, p.double_difference_carrier_factors) &&
           readVec(is, p.excluded_double_difference_carrier_factors) &&
           readVec(is, p.ambiguity_between_factors) &&
           readPod(is, p.diagnostics);
}

struct FileId {
    int64_t size_bytes = -1;
    int64_t mtime_ns = -1;
};

FileId statFile(const std::string& path) {
    FileId id;
    if (path.empty()) return id;
    std::error_code ec;
    const auto sz = std::filesystem::file_size(path, ec);
    if (!ec) id.size_bytes = static_cast<int64_t>(sz);
    ec.clear();
    const auto mt = std::filesystem::last_write_time(path, ec);
    if (!ec) {
        id.mtime_ns = static_cast<int64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(mt.time_since_epoch()).count());
    }
    return id;
}

// Bump whenever writeProblem/readProblem's on-disk layout changes (new
// vector, new factor field, ...) -- an old cache then fails the magic/version
// check in load() below and is rebuilt instead of misread.
constexpr uint32_t kMagic = 0x50434647u;  // "PCFG" (problem-cache fgo)
constexpr uint32_t kVersion = 6u;
constexpr std::size_t kBuilderFingerprintBytes = 512u;

struct Fingerprint {
    uint32_t magic;
    uint32_t version;
    FileId rover;
    FileId base;
    FileId nav;
    FileId imu;
    int32_t max_epochs;
    int32_t start_epoch;
    // Stable packed snapshot of builder-visible fields only. Its size and
    // layout deliberately do not depend on sizeof(FGOConfig), so adding a
    // solver-only option cannot invalidate a parsed-problem cache.
    std::array<unsigned char, kBuilderFingerprintBytes> config_bytes;
};

Fingerprint computeFingerprint(const Args& args, const libgnss::FGOProcessor::FGOConfig& config) {
    Fingerprint fp;
    std::memset(&fp, 0, sizeof(fp));  // deterministic padding bytes for the memcmp below
    fp.magic = kMagic;
    fp.version = kVersion;
    fp.rover = statFile(args.rover_path);
    fp.base = statFile(args.base_path);
    fp.nav = statFile(args.nav_path);
    fp.imu = statFile(args.imu_path);
    fp.max_epochs = args.max_epochs;
    fp.start_epoch = args.start_epoch;
    // fp was zero-filled above. Pack fields consecutively instead of at their
    // FGOConfig offsets; only this explicit list defines cache identity. Keep
    // it in sync with config_ accesses in the problem builders.
    std::size_t config_cursor = 0;
#define COPY_BUILD_FIELD(name)                                                        \
    do {                                                                              \
        if (config_cursor + sizeof(config.name) > fp.config_bytes.size()) {           \
            throw std::runtime_error("problem-cache builder fingerprint overflow");  \
        }                                                                             \
        std::memcpy(fp.config_bytes.data() + config_cursor, &config.name,             \
                    sizeof(config.name));                                             \
        config_cursor += sizeof(config.name);                                         \
    } while (false)
    COPY_BUILD_FIELD(ambiguity_between_sigma_cycles);
    COPY_BUILD_FIELD(base_epoch_match_tolerance_s);
    COPY_BUILD_FIELD(base_interpolation_max_gap_s);
    COPY_BUILD_FIELD(carrier_phase_sigma_m);
    COPY_BUILD_FIELD(cmc_aware_reference_selection);
    COPY_BUILD_FIELD(code_minus_carrier_baseline_alpha);
    COPY_BUILD_FIELD(code_minus_carrier_jump_threshold_m);
    COPY_BUILD_FIELD(code_minus_carrier_level_pseudorange_only);
    COPY_BUILD_FIELD(code_minus_carrier_level_threshold_m);
    COPY_BUILD_FIELD(code_minus_carrier_warmup_epochs);
    COPY_BUILD_FIELD(double_difference_base_min_snr_dbhz);
    COPY_BUILD_FIELD(double_difference_carrier_sigma_m);
    COPY_BUILD_FIELD(double_difference_pseudorange_sigma_m);
    COPY_BUILD_FIELD(double_difference_reference_min_snr_dbhz);
    COPY_BUILD_FIELD(double_difference_secondary_carrier_sigma_scale);
    COPY_BUILD_FIELD(double_difference_secondary_pseudorange_sigma_scale);
    COPY_BUILD_FIELD(elevation_sigma_clock_stability);
    COPY_BUILD_FIELD(elevation_sigma_err_a_m);
    COPY_BUILD_FIELD(elevation_sigma_err_b_m);
    COPY_BUILD_FIELD(elevation_sigma_pseudorange_ratio);
    COPY_BUILD_FIELD(max_tdcp_gap_s);
    COPY_BUILD_FIELD(min_elevation_deg);
    COPY_BUILD_FIELD(min_satellites_per_epoch);
    COPY_BUILD_FIELD(min_snr_dbhz);
    COPY_BUILD_FIELD(pseudorange_elevation_sigma_power);
    COPY_BUILD_FIELD(pseudorange_sigma_m);
    COPY_BUILD_FIELD(reject_rover_carrier_loss_of_lock);
    COPY_BUILD_FIELD(reject_tdcp_code_phase_jump);
    COPY_BUILD_FIELD(reject_tdcp_loss_of_lock);
    COPY_BUILD_FIELD(reset_double_difference_ambiguities_each_epoch);
    COPY_BUILD_FIELD(single_difference_doppler_sigma_mps);
    COPY_BUILD_FIELD(single_difference_tdcp_sigma_m);
    COPY_BUILD_FIELD(tdcp_code_phase_jump_threshold_m);
    COPY_BUILD_FIELD(tdcp_sigma_m);
    COPY_BUILD_FIELD(use_ambiguity_between_factors);
    COPY_BUILD_FIELD(use_carrier_phase_factors);
    COPY_BUILD_FIELD(use_code_minus_carrier_screening);
    COPY_BUILD_FIELD(use_double_difference_factors);
    COPY_BUILD_FIELD(use_double_difference_secondary_code_alignment);
    COPY_BUILD_FIELD(use_elevation_dependent_sigma);
    COPY_BUILD_FIELD(use_external_doppler_dr_validation);
    COPY_BUILD_FIELD(use_ionosphere_model);
    COPY_BUILD_FIELD(use_multi_constellation);
    COPY_BUILD_FIELD(use_multi_frequency_double_difference);
    COPY_BUILD_FIELD(use_pseudorange_factors);
    COPY_BUILD_FIELD(use_single_difference_doppler_factors);
    COPY_BUILD_FIELD(use_single_difference_tdcp_factors);
    COPY_BUILD_FIELD(use_spp_seed);
    COPY_BUILD_FIELD(use_tdcp_factors);
    COPY_BUILD_FIELD(use_troposphere_model);
#undef COPY_BUILD_FIELD
    return fp;
}

struct CacheMeta {
    uint64_t rover_epoch_count = 0;
    uint64_t base_epoch_count = 0;
    libgnss::Vector3d base_position = libgnss::Vector3d::Zero();
};

// Returns true and populates {problem, meta} on a validated hit. On ANY
// mismatch (missing file, truncated/corrupt file, format version bump, or a
// fingerprint field that no longer matches the current run) prints a loud
// warning and returns false -- callers must then rebuild normally. Never
// silently serves a stale problem.
bool load(const std::string& path, const Fingerprint& want, libgnss::FGOProcessor::FGOProblem& problem,
          CacheMeta& meta) {
    std::ifstream is(path, std::ios::binary);
    if (!is) {
        std::cout << "problem-cache: no cache file at " << path << " -- building normally\n";
        return false;
    }
    Fingerprint have;
    std::memset(&have, 0, sizeof(have));
    if (!readPod(is, have)) {
        std::cerr << "problem-cache: WARNING -- " << path
                  << " is truncated/unreadable; ignoring and rebuilding.\n";
        return false;
    }
    if (have.magic != kMagic || have.version != kVersion) {
        std::cerr << "problem-cache: WARNING -- " << path
                  << " has an incompatible format (magic/version mismatch); ignoring and rebuilding.\n";
        return false;
    }
    if (std::memcmp(&have, &want, sizeof(Fingerprint)) != 0) {
        const auto* have_bytes = reinterpret_cast<const unsigned char*>(&have);
        const auto* want_bytes = reinterpret_cast<const unsigned char*>(&want);
        std::size_t first_diff = 0;
        while (first_diff < sizeof(Fingerprint) &&
               have_bytes[first_diff] == want_bytes[first_diff]) {
            ++first_diff;
        }
        std::cerr << "problem-cache: WARNING -- " << path
                  << " fingerprint mismatch at byte " << first_diff
                  << " (cached=" << static_cast<unsigned>(have_bytes[first_diff])
                  << ", requested=" << static_cast<unsigned>(want_bytes[first_diff]) << ')'
                  << " (inputs or builder config changed since it was written);"
                     " ignoring and rebuilding.\n";
        return false;
    }
    if (!readPod(is, meta) || !readProblem(is, problem)) {
        std::cerr << "problem-cache: WARNING -- " << path
                  << " is truncated/corrupt after the fingerprint; ignoring and rebuilding.\n";
        return false;
    }
    return true;
}

void save(const std::string& path, const Fingerprint& fp, const libgnss::FGOProcessor::FGOProblem& problem,
          const CacheMeta& meta) {
    const std::string tmp_path = path + ".tmp";
    std::ofstream os(tmp_path, std::ios::binary | std::ios::trunc);
    if (!os) {
        std::cerr << "problem-cache: WARNING -- cannot open " << tmp_path
                  << " for writing; skipping cache write.\n";
        return;
    }
    writePod(os, fp);
    writePod(os, meta);
    writeProblem(os, problem);
    os.close();
    if (!os) {
        std::cerr << "problem-cache: WARNING -- write to " << tmp_path << " failed; skipping cache write.\n";
        return;
    }
    std::error_code ec;
    std::filesystem::rename(tmp_path, path, ec);  // atomic-ish swap; avoids a half-written cache on crash
    if (ec) {
        std::error_code ec2;
        std::filesystem::remove(path, ec2);
        std::filesystem::rename(tmp_path, path, ec);
    }
}

}  // namespace problem_cache

}  // namespace

int main(int argc, char** argv) {
    const Args args = parseArgs(argc, argv);

    // config depends only on args (see buildFgoConfig) -- build it FIRST so a
    // --problem-cache fingerprint (which snapshots these bytes) can gate the
    // expensive RINEX parse + buildDoubleDifferenceProblem() call below.
    libgnss::FGOProcessor::FGOConfig config = buildFgoConfig(args);

    libgnss::FGOProcessor::FGOProblem problem;
    uint64_t rover_epoch_count = 0;
    uint64_t base_epoch_count = 0;
    libgnss::Vector3d base_position = libgnss::Vector3d::Zero();
    bool problem_from_cache = false;
    const bool use_problem_cache = !args.problem_cache_path.empty();
    problem_cache::Fingerprint problem_fp;
    if (use_problem_cache) {
        problem_fp = problem_cache::computeFingerprint(args, config);
        problem_cache::CacheMeta meta;
        if (problem_cache::load(args.problem_cache_path, problem_fp, problem, meta)) {
            problem_from_cache = true;
            rover_epoch_count = meta.rover_epoch_count;
            base_epoch_count = meta.base_epoch_count;
            base_position = meta.base_position;
            std::cout << "problem-cache: HIT (" << args.problem_cache_path
                      << ") -- skipped RINEX parse + problem build\n";
        }
    }

    if (!problem_from_cache) {
        // Navigation.
        libgnss::io::RINEXReader nav_reader;
        if (!nav_reader.open(args.nav_path)) {
            std::cerr << "Error: cannot open nav " << args.nav_path << "\n";
            return 1;
        }
        libgnss::NavigationData nav;
        if (!nav_reader.readNavigationData(nav)) {
            std::cerr << "Error: cannot read nav " << args.nav_path << "\n";
            return 1;
        }

        // Base station position from its RINEX header (mirrors gnss_fgo.cpp).
        libgnss::io::RINEXReader base_header_reader;
        if (!base_header_reader.open(args.base_path)) {
            std::cerr << "Error: cannot open base " << args.base_path << "\n";
            return 1;
        }
        libgnss::io::RINEXReader::RINEXHeader base_header;
        if (!base_header_reader.readHeader(base_header)) {
            std::cerr << "Error: cannot read base header " << args.base_path << "\n";
            return 1;
        }
        if (base_header.approximate_position.norm() <= 1e6) {
            std::cerr << "Error: base approximate position unavailable in " << args.base_path << "\n";
            return 1;
        }
        base_position = base_header.approximate_position;

        const std::vector<libgnss::ObservationData> rover_epochs =
            loadEpochs(args.rover_path, args.max_epochs, libgnss::Vector3d::Zero(), false,
                       args.start_epoch);
        // Base is read fully (its epochs are matched/interpolated to rover time
        // inside buildDoubleDifferenceProblem); no rover-side epoch cap on base.
        const std::vector<libgnss::ObservationData> base_epochs =
            loadEpochs(args.base_path, 0, base_position, true);
        rover_epoch_count = rover_epochs.size();
        base_epoch_count = base_epochs.size();

        if (use_problem_cache) {
            std::cout << "problem-cache: MISS (" << args.problem_cache_path
                      << ") -- parsing RINEX + building problem normally\n";
        }

        const libgnss::FGOProcessor builder(config);
        problem = builder.buildDoubleDifferenceProblem(rover_epochs, base_epochs, nav, base_position);

        if (use_problem_cache) {
            problem_cache::CacheMeta meta;
            meta.rover_epoch_count = rover_epoch_count;
            meta.base_epoch_count = base_epoch_count;
            meta.base_position = base_position;
            problem_cache::save(args.problem_cache_path, problem_fp, problem, meta);
        }
    }

    std::cout << "Dataset:\n"
              << "  rover=" << args.rover_path << " (" << rover_epoch_count << " epochs"
              << (args.max_epochs > 0 ? " capped" : "")
              << (args.start_epoch > 0
                      ? ", starting at input epoch " + std::to_string(args.start_epoch)
                      : std::string())
              << ")\n"
              << "  base=" << args.base_path << " (" << base_epoch_count << " epochs)\n"
              << "  nav=" << args.nav_path << "\n"
              << "  base_pos_ecef=[" << base_position.transpose() << "]\n";

    std::cout << "FGOProblem: epochs=" << problem.epochs.size()
              << " dd_pr_factors=" << problem.double_difference_pseudorange_factors.size()
              << " dd_cp_factors=" << problem.double_difference_carrier_factors.size()
              << " sd_doppler_factors=" << problem.single_difference_doppler_factors.size()
              << " ambiguity_states=" << problem.ambiguity_states.size()
              << " pr_factors=" << problem.pseudorange_factors.size() << "\n";
    std::cout << "  CMC screening: " << (args.cmc ? "on" : "off")
              << " (jump=" << config.code_minus_carrier_jump_threshold_m
              << " m, level=" << config.code_minus_carrier_level_threshold_m
              << " m (" << (args.cmc_level_pr_only ? "PR-only" : "PR+CP") << ')'
              << ", warmup=" << config.code_minus_carrier_warmup_epochs
              << ", alpha=" << config.code_minus_carrier_baseline_alpha << ")"
              << " -- jump_resets=" << problem.diagnostics.code_minus_carrier_jump_resets
              << ", level_exclusions=" << problem.diagnostics.code_minus_carrier_level_exclusions
              << ", cmc_ref=" << (args.cmc_ref ? "on" : "off")
              << ", ref_avoided=" << problem.diagnostics.cmc_ref_avoided_count
              << "\n";

    // --- MF hygiene diagnostics: per-signal DD residuals at the reference
    // trajectory (no solver). PR residual mean exposes per-band code/DCB
    // bias; per-arc carrier residual fractional-cycle offset exposes phase
    // misalignment / broken integer-ness. ---
    if (args.dd_resid) {
        const std::vector<RefRow> ref_rows = loadReference(args.ref_path);
        if (ref_rows.empty()) {
            std::cerr << "Error: --dd-resid requires --ref reference.csv\n";
            return 1;
        }
        // Nearest-truth lookup per problem epoch (both time-sorted).
        std::vector<libgnss::Vector3d> truth(problem.epochs.size(),
                                             libgnss::Vector3d::Zero());
        std::vector<bool> has_truth(problem.epochs.size(), false);
        {
            std::size_t ri = 0;
            for (std::size_t i = 0; i < problem.epochs.size(); ++i) {
                const auto& t = problem.epochs[i].time;
                while (ri + 1 < ref_rows.size() &&
                       std::abs(ref_rows[ri + 1].time - t) <
                           std::abs(ref_rows[ri].time - t)) {
                    ++ri;
                }
                if (std::abs(ref_rows[ri].time - t) <= 0.11) {
                    truth[i] = ref_rows[ri].ecef;
                    has_truth[i] = true;
                }
            }
        }
        auto dd_geom = [](const auto& f, const libgnss::Vector3d& x) {
            return ((f.rover_satellite_position_ecef - x).norm() -
                    (f.base_satellite_position_ecef - f.base_position_ecef).norm()) -
                   ((f.rover_reference_position_ecef - x).norm() -
                    (f.base_reference_position_ecef - f.base_position_ecef).norm());
        };
        struct PrStats {
            std::size_t n = 0;
            double sum = 0.0, sq = 0.0;
        };
        std::map<int, PrStats> pr_stats;
        for (const auto& f : problem.double_difference_pseudorange_factors) {
            if (f.epoch_index >= truth.size() || !has_truth[f.epoch_index]) continue;
            const double r = f.observed_dd_pseudorange_m - dd_geom(f, truth[f.epoch_index]);
            auto& s = pr_stats[static_cast<int>(f.signal)];
            ++s.n;
            s.sum += r;
            s.sq += r * r;
        }
        // Carrier: per-arc mean residual -> fractional cycles from nearest int.
        struct ArcAcc {
            std::size_t n = 0;
            double sum = 0.0;
            int signal = 0;
            double wavelength = 0.0;
        };
        std::map<std::size_t, ArcAcc> arcs;
        for (const auto& f : problem.double_difference_carrier_factors) {
            if (f.epoch_index >= truth.size() || !has_truth[f.epoch_index]) continue;
            if (f.ambiguity_index >= problem.ambiguity_states.size()) continue;
            auto& a = arcs[f.ambiguity_index];
            ++a.n;
            a.sum += f.observed_dd_carrier_m - dd_geom(f, truth[f.epoch_index]);
            a.signal = static_cast<int>(f.signal);
            a.wavelength = problem.ambiguity_states[f.ambiguity_index].wavelength_m;
        }
        struct CarrierStats {
            std::size_t arcs = 0, tight = 0;
            std::vector<double> fracs;
        };
        std::map<int, CarrierStats> cp_stats;
        for (const auto& [idx, a] : arcs) {
            if (a.n < 10 || a.wavelength <= 0.0) continue;
            const double cycles = (a.sum / double(a.n)) / a.wavelength;
            const double frac = std::abs(cycles - std::round(cycles));
            auto& s = cp_stats[a.signal];
            ++s.arcs;
            if (frac < 0.15) ++s.tight;
            s.fracs.push_back(frac);
        }
        std::cout << "\n=== --dd-resid: per-signal DD residuals at the reference trajectory ===\n"
                  << "  DD pseudorange (bias -> code/DCB mismatch):\n";
        for (auto& [sig, s] : pr_stats) {
            const double mean = s.sum / double(s.n);
            const double rms = std::sqrt(s.sq / double(s.n));
            std::cout << "    signal_enum=" << sig << "  n=" << s.n
                      << "  mean=" << mean << " m  rms=" << rms << " m\n";
        }
        std::cout << "  DD carrier per-arc mean, fractional cycles from nearest integer\n"
                  << "  (arcs with >=10 epochs; frac<0.15 = integer-consistent):\n";
        for (auto& [sig, s] : cp_stats) {
            std::sort(s.fracs.begin(), s.fracs.end());
            const double med = s.fracs.empty() ? 0.0 : s.fracs[s.fracs.size() / 2];
            std::cout << "    signal_enum=" << sig << "  arcs=" << s.arcs
                      << "  median_frac=" << med << "  frac<0.15: "
                      << (s.arcs > 0 ? 100.0 * double(s.tight) / double(s.arcs) : 0.0)
                      << "%\n";
        }
        std::cout.flush();
        return 0;
    }

    if (problem.epochs.empty() ||
        (problem.double_difference_carrier_factors.empty() &&
         problem.double_difference_pseudorange_factors.empty())) {
        std::cerr << "Error: no DD factors built; cannot compare backends.\n";
        return 1;
    }

    // --- (a4) MILESTONE 2c: IncrementalFixedLagSmoother at scale. Runs FIRST
    // and returns, so the full run does NOT pay for the heavy batch (a)/(a2)/(b)
    // DD comparisons (whose full-batch gtsam::Marginals is exactly what 2c
    // avoids). Requires --imu + --fixed-lag S. ---
    if (args.fixed_lag_s > 0.0 && !args.imu_path.empty()) {
        libgnss::FGOProcessor::FGOProblem problem_imu = problem;
        if (!buildImuInput(args.imu_path, problem_imu)) {
            std::cerr << "Error: could not build IMU input for fixed-lag run.\n";
            return 1;
        }
        applyImuNoiseOverrides(args, problem_imu);
        std::vector<RefRow> ref_rows;
        if (!args.ref_path.empty()) ref_rows = loadReference(args.ref_path);

        double t_fl = 0.0;
        const auto fl = run(problem_imu, config, libgnss::FGOBackend::GTSAM, true, t_fl,
                             /*use_pose3=*/true, /*use_imu=*/true, args.fixed_lag_s,
                             args.use_nhc, args.use_zupt, args.use_hold, args.fixed_lag_qr);
        std::size_t nonfinite = 0, none_epochs = 0;
        for (const auto& s : fl.solution.solutions) {
            if (s.status == libgnss::SolutionStatus::NONE) ++none_epochs;
            else if (!s.position_ecef.allFinite()) ++nonfinite;
        }
        const std::size_t fl_fixed = countFixedEpochs(fl);
        const std::size_t ne = fl.solution.solutions.size();
        const HorizError he = horizontalErrorVsRef(fl, ref_rows);
        if (!args.dump_csv_path.empty()) {
            dumpEpochCsv(fl, ref_rows, args.dump_csv_path);
        }

        std::cout << "\n=== (a4) MILESTONE 2c: IncrementalFixedLagSmoother (full-scale) ===\n"
                  << "  lag=" << args.fixed_lag_s << " s, epochs=" << ne
                  << ", smoother_updates=" << fl.diagnostics.smoother_updates
                  << ", peak_window_vars=" << fl.diagnostics.smoother_max_window_vars << "\n"
                  << "  wall_clock=" << t_fl << " s ("
                  << (ne > 0 ? t_fl / double(ne) * 1e3 : 0.0)
                  << " ms/epoch), nonfinite=" << nonfinite << ", NONE_epochs=" << none_epochs << "\n"
                  << "  per-epoch LAMBDA: attempts=" << fl.diagnostics.lambda_ambiguity_attempts
                  << ", fixed_epochs=" << fl_fixed << "/" << ne << " ("
                  << (ne > 0 ? 100.0 * double(fl_fixed) / double(ne) : 0.0)
                  << "% fix-rate), best_ratio=" << fl.diagnostics.lambda_ambiguity_ratio
                  << ", stale_candidates_filtered="
                  << fl.diagnostics.lambda_stale_candidates_filtered
                  << ", joint_marginal_failures="
                  << fl.diagnostics.lambda_joint_marginal_failures << "\n"
                  << "  partial AR: " << (config.use_fixed_lag_partial_lambda ? "on" : "off")
                  << " (min_fraction=" << config.fixed_lag_partial_lambda_min_fraction
                  << ", min_fixed=" << config.min_fixed_ambiguities
                  << ", max_std_cycles=" << config.partial_ar_max_std_cycles << ")\n"
                  << "  IMU ratio aperture: " << (args.imu_ratio_aperture ? "on" : "off")
                  << " (accepted=" << fl.diagnostics.imu_aided_ratio_accepts
                  << ", rejected=" << fl.diagnostics.imu_aided_ratio_rejects
                  << ", relaxed_ratio=" << config.imu_aided_relaxed_ratio_threshold
                  << ", float_sep_m=" << config.imu_aided_max_float_separation_m
                  << ", pred_sep_m=" << config.imu_aided_max_prediction_separation_m << ")\n"
                  << "  fixed-history DR validation: "
                  << (args.fixed_history_dr ? "on" : "off")
                  << " (accepted=" << fl.diagnostics.fixed_history_dr_accepts
                  << ", rejected=" << fl.diagnostics.fixed_history_dr_rejects
                  << ", window=" << config.fixed_history_dr_window_epochs
                  << ", sep_m=" << config.fixed_history_dr_max_separation_m
                  << ", surplus_overrides=" << (args.surplus_overrides_dr ? "on" : "off")
                  << " (n=" << fl.diagnostics.fixed_history_dr_surplus_overrides
                  << ", min_used=" << config.surplus_validation_overrides_history_dr_min_surplus_used
                  << ", max_consec=" << config.surplus_validation_overrides_history_dr_max_consecutive
                  << ", capped=" << fl.diagnostics.fixed_history_dr_surplus_override_capped
                  << ")"
                  << ")\n"
                  << "  DDPR anchor validation: "
                  << (args.anchor_aided_validation ? "on" : "off")
                  << " (accepted=" << fl.diagnostics.ddpr_anchor_validation_accepts
                  << ", rejected=" << fl.diagnostics.ddpr_anchor_validation_rejects
                  << ", sep_m=" << config.ddpr_anchor_validation_max_separation_m << ")\n"
                  << "  fixed-hypothesis postfit: "
                  << (config.use_fixed_hypothesis_postfit_validation ? "on" : "off")
                  << " (accepted=" << fl.diagnostics.fixed_postfit_validation_accepts
                  << ", rejected=" << fl.diagnostics.fixed_postfit_validation_rejects
                  << ", min_n=" << config.fixed_postfit_min_factors
                  << ", rms_m=" << config.fixed_postfit_max_rms_m
                  << ", max_norm=" << config.fixed_postfit_max_normalized_residual
                  << ", chi2_dof=" << config.fixed_postfit_max_chi2_per_dof << ")\n"
                  << "  satellite-count adaptive ratio: "
                  << (config.use_satellite_count_adaptive_ratio ? "on" : "off")
                  << " (>=20:" << config.adaptive_ratio_nsat20
                  << ", >=15:" << config.adaptive_ratio_nsat15
                  << ", >=10:" << config.adaptive_ratio_nsat10
                  << ", low:" << config.adaptive_ratio_nsat_low << ")\n"
                  << "  external Doppler-DR SSE: "
                  << (config.use_external_doppler_dr_validation ? "on" : "off")
                  << " (accepted=" << fl.diagnostics.external_doppler_dr_accepts
                  << ", rejected=" << fl.diagnostics.external_doppler_dr_rejects
                  << ", unavailable=" << fl.diagnostics.external_doppler_dr_unavailable
                  << ", max_age=" << config.external_doppler_dr_max_age_epochs
                  << ", chi2=" << config.external_doppler_dr_chi2_threshold
                  << ", reset_ratio=" << config.external_doppler_dr_reset_min_ratio << ")\n"
                  << "  NHC/ZUPT: nhc=" << (args.use_nhc ? "on" : "off")
                  << " (applied " << fl.diagnostics.nhc_epochs << " epochs), zupt="
                  << (args.use_zupt ? "on" : "off") << " (applied " << fl.diagnostics.zupt_epochs
                  << " epochs)\n"
                  << "  quality-gates: " << (args.gates ? "on" : "off")
                  << " (fixing suppressed on " << fl.diagnostics.quality_gated_epochs
                  << " epochs)\n"
                  << "  fix-and-hold: " << (args.use_hold ? "on" : "off")
                  << " (pinned " << fl.diagnostics.ambiguity_hold_arcs << " arcs, "
                  << fl.diagnostics.ambiguity_hold_epochs << " epochs FIXED via held integers, "
                  << "held_only_label=" << (args.no_held_fix_label ? "off" : "on") << ")\n"
                  << "  continuous-unfix reset: "
                  << (config.use_continuous_unfix_ambiguity_reset ? "on" : "off")
                  << " (resets=" << fl.diagnostics.ambiguity_continuous_unfix_resets
                  << ", epochs=" << config.continuous_unfix_reset_epochs
                  << ", min_sat=" << config.continuous_unfix_min_satellites
                  << ", max_gdop=" << config.continuous_unfix_max_gdop
                  << ", max_fde_fraction="
                  << config.continuous_unfix_max_fde_reject_fraction << ")\n"
                  << "  CMC screening: " << (args.cmc ? "on" : "off")
                  << " (jump_resets=" << fl.diagnostics.code_minus_carrier_jump_resets
                  << ", level_exclusions=" << fl.diagnostics.code_minus_carrier_level_exclusions
                  << ", cmc_ref=" << (args.cmc_ref ? "on" : "off")
                  << ", ref_avoided=" << fl.diagnostics.cmc_ref_avoided_count
                  << ")\n"
                  << "  CP-hold/sanity FSM: " << (args.cp_hold ? "on" : "off")
                  << " (triggers=" << fl.diagnostics.cp_hold_triggers
                  << ", epochs_held=" << fl.diagnostics.cp_hold_epochs_held
                  << ", anchor_releases=" << fl.diagnostics.cp_hold_anchor_releases
                  << ", selectively_downweighted="
                  << fl.diagnostics.selective_cp_hold_downweighted_factors
                  << ", mass_resets=" << fl.diagnostics.sanity_mass_resets
                  << ", fast_resets=" << fl.diagnostics.sanity_fast_resets
                  << ", pose_replacements=" << fl.diagnostics.sanity_pose_replacements
                  << ", multipath_skips=" << fl.diagnostics.sanity_multipath_skips
                  << ", gdop_skips=" << fl.diagnostics.sanity_gdop_skips
                  << ", generation_bumps=" << fl.diagnostics.ambiguity_generation_bumps
                  << " [hold=" << fl.diagnostics.ambiguity_generation_bumps_hold
                  << ", fde=" << fl.diagnostics.ambiguity_generation_bumps_fde
                  << ", reset=" << fl.diagnostics.ambiguity_generation_bumps_reset
                  << ", warm_reset=" << fl.diagnostics.ambiguity_generation_bumps_warm_reset
                  << ", stale_pin=" << fl.diagnostics.ambiguity_generation_bumps_stale_pin << ']'
                  << ")\n"
                  << "  stale-pin invalidation: " << (args.stale_pin ? "on" : "off")
                  << " (invalidations=" << fl.diagnostics.stale_pin_invalidations
                  << ", per_sat_res_m=" << config.stale_pin_per_sat_residual_m
                  << ", min_hold_age=" << config.stale_pin_min_hold_age_epochs
                  << ")\n"
                  << "  fix plausibility demotion: " << (args.fix_demote ? "on" : "off")
                  << " (demotions=" << fl.diagnostics.fix_plausibility_demotions
                  << ", anchor_demotions=" << fl.diagnostics.fix_plausibility_anchor_demotions
                  << ", hold_skips=" << fl.diagnostics.fix_plausibility_hold_skips
                  << ", surplus_crosscheck=" << (args.fix_demote_surplus_crosscheck ? "on" : "off")
                  << ", surplus_reprieves=" << fl.diagnostics.fix_plausibility_surplus_reprieves
                  << ", distance_m=" << config.fix_demote_distance_m
                  << ", anchor=" << (args.fix_demote_anchor ? "on" : "off")
                  << ", anchor_distance_m=" << config.fix_demote_anchor_distance_m
                  << ", anchor_trust_res_m=" << config.fix_demote_anchor_trust_res_m
                  << ", res_m=" << config.fix_demote_res_m
                  << ", posthold=" << config.fix_demote_posthold_epochs
                  << ", res_rel=" << config.fix_demote_res_rel
                  << ", anchor_gross=" << (args.fix_demote_anchor_gross ? "on" : "off")
                  << ", anchor_gross_ratio=" << config.fix_demote_anchor_gross_ratio
                  << ", anchor_gross_abs_m=" << config.fix_demote_anchor_gross_abs_m
                  << ", anchor_gross_gated=" << fl.diagnostics.fix_plausibility_anchor_gross_gated
                  << ")\n"
                  << "  surplus-satellite validation: " << (args.surplus_validation ? "on" : "off")
                  << " (monitor_only=" << (args.surplus_validation_monitor ? "on" : "off")
                  << ", veto=" << (args.surplus_validation_veto ? "on" : "off")
                  << ", attempts=" << fl.diagnostics.surplus_validation_attempts
                  << ", passes=" << fl.diagnostics.surplus_validation_passes
                  << ", fails=" << fl.diagnostics.surplus_validation_fails
                  << ", insufficient_surplus=" << fl.diagnostics.surplus_validation_insufficient_surplus
                  << ", rescued_epochs=" << fl.diagnostics.surplus_validation_rescued_epochs
                  << ", separation_rejects="
                  << fl.diagnostics.surplus_validation_separation_rejects
                  << ", quality_rejects="
                  << fl.diagnostics.surplus_validation_quality_rejects
                  << ", vetoed_epochs=" << fl.diagnostics.surplus_validation_vetoed_epochs
                  << ", min_n=" << config.surplus_validation_min_surplus_satellites
                  << ", aperture_cycles(lt1/1to2/gt2)=" << config.surplus_validation_aperture_pdop_lt1_cycles
                  << "/" << config.surplus_validation_aperture_pdop_1to2_cycles
                  << "/" << config.surplus_validation_aperture_pdop_gt2_cycles
                  << ", require_all=" << (config.surplus_validation_require_all ? "on" : "off")
                  << ", fallback_level_hist(GQEBR,GQEB,GQER,GQE,GQB,GQ)=["
                  << fl.diagnostics.surplus_validation_fallback_level_histogram[0] << ","
                  << fl.diagnostics.surplus_validation_fallback_level_histogram[1] << ","
                  << fl.diagnostics.surplus_validation_fallback_level_histogram[2] << ","
                  << fl.diagnostics.surplus_validation_fallback_level_histogram[3] << ","
                  << fl.diagnostics.surplus_validation_fallback_level_histogram[4] << ","
                  << fl.diagnostics.surplus_validation_fallback_level_histogram[5] << "]"
                  << ")\n"
                  << "  low-count AR rescue: " << (args.low_count_ar ? "on" : "off")
                  << " (min_candidates=" << config.low_count_min_candidates
                  << ", min_ratio=" << config.low_count_min_ratio
                  << ", attempts=" << fl.diagnostics.low_count_ambiguity_attempts
                  << ", accepted=" << fl.diagnostics.low_count_ambiguity_fix_accepted
                  << ")\n"
                  << "  integer-constrained reoptimization: "
                  << (args.integer_constrained_reoptimization ? "on" : "off")
                  << " (attempts=" << fl.diagnostics.integer_constrained_reoptimization_attempts
                  << ", accepted=" << fl.diagnostics.integer_constrained_reoptimization_accepts
                  << ", rejected=" << fl.diagnostics.integer_constrained_reoptimization_rejects
                  << ", prior_sigma_cycles="
                  << config.integer_constrained_prior_sigma_cycles
                  << ", cost_tolerance="
                  << config.integer_constrained_cost_abs_tolerance
                  << ", max_iterations="
                  << config.integer_constrained_max_iterations << ")\n"
                  << "  leaky persist (C1): " << (args.leaky_persist ? "on" : "off")
                  << " (decay=" << config.cp_hold_persist_decay
                  << ", persist_epochs=" << config.cp_hold_persist_epochs
                  << ")\n"
                  << "  exception recovery: " << (args.exc_recovery ? "on" : "off")
                  << " (smoother_recovery_epochs=" << fl.diagnostics.smoother_recovery_epochs
                  << ", loose_prior_recoveries=" << fl.diagnostics.solve_exception_recoveries
                  << ", full_warm_resets=" << fl.diagnostics.solve_exception_warm_resets
                  << ")\n"
                  << "  DDPR-LS anchor: " << (args.ddpr_anchor ? "on" : "off")
                  << " (solves=" << fl.diagnostics.ddpr_anchor_solves
                  << ", successes=" << fl.diagnostics.ddpr_anchor_successes
                  << ", gated_resets_skipped=" << fl.diagnostics.ddpr_anchor_gated_resets_skipped
                  << ", gated_resets_allowed=" << fl.diagnostics.ddpr_anchor_gated_resets_allowed
                  << ", anchored_warm_resets=" << fl.diagnostics.ddpr_anchored_warm_resets
                  << ", bootstrap_prior_epochs=" << fl.diagnostics.ddpr_anchor_bootstrap_prior_epochs
                  << ")\n"
                  << "  FDE: " << (args.fde ? "on" : "off")
                  << " (mode=" << (args.fde_pr_only ? "pseudorange-only" :
                                      (args.fde_cp_quarantine ? "carrier-AR-quarantine" :
                                                               "pseudorange+carrier"))
                  << ", pr_rejections=" << fl.diagnostics.fde_pseudorange_rejections
                  << ", cp_rejections=" << fl.diagnostics.fde_carrier_rejections
                  << ", cp_quarantines=" << fl.diagnostics.fde_carrier_quarantines
                  << ", safeguard_skips=" << fl.diagnostics.fde_safeguard_skips
                  << ", fde_epochs=" << fl.diagnostics.fde_epochs
                  << ")\n"
                  << "  sat-badness down-weighting: " << (args.sat_badness ? "on" : "off")
                  << " (downweighted_factors=" << fl.diagnostics.sat_badness_downweighted_factors
                  << ", max_score_seen=" << fl.diagnostics.sat_badness_max_score_seen
                  << ", clamp_m=" << config.sat_badness_residual_clamp_m
                  << ", score_cap=" << config.sat_badness_score_cap
                  << ", cppr_decay=" << config.sat_badness_cppr_decay
                  << ")\n"
                  << "  varerr elevation-dependent DD sigma: " << (args.varerr ? "on" : "off")
                  << " (err_a=" << config.elevation_sigma_err_a_m
                  << " m, err_b=" << config.elevation_sigma_err_b_m
                  << " m, eratio_pr=" << config.elevation_sigma_pseudorange_ratio
                  << ", sclkstab=" << config.elevation_sigma_clock_stability
                  << " s/s)\n"
                  << "  IMU integration covariance: value=" << config.imu_integration_covariance
                  << " m^2/s, inflation=" << (args.integ_cov_inflate ? "on" : "off")
                  << " (cap=" << config.imu_integration_covariance_max
                  << ", stale_epochs=" << config.imu_integration_covariance_stale_epochs
                  << "), gravity=" << problem_imu.imu.noise.gravity_mps2 << " m/s^2\n";
        if (!ref_rows.empty()) {
            std::cout << "  horizontal error vs reference.csv:\n"
                      << "    FLOAT: n=" << he.n_float << " rms=" << he.float_rms
                      << " m max=" << he.float_max << " m\n"
                      << "    FIXED: n=" << he.n_fixed << " rms=" << he.fixed_rms
                      << " m max=" << he.fixed_max << " m\n"
                      << "    ALL (float+fixed) <50cm rate=" << (100.0 * he.frac_all_under_50cm)
                      << "%\n"
                      << "    (inuex35 truth target run1: FixRMS 0.815 m / fix 49.5% / <50cm 56.7%)\n";
            if (args.fixed_err_hist) {
                std::cout << "    FIXED error histogram: <0.5m=" << he.fixed_hist[0]
                          << " 0.5-1m=" << he.fixed_hist[1] << " 1-2m=" << he.fixed_hist[2]
                          << " 2-5m=" << he.fixed_hist[3] << " 5-10m=" << he.fixed_hist[4]
                          << " >=10m=" << he.fixed_hist[5] << "\n";
                const double tot_sumsq = he.fixed_rms * he.fixed_rms * double(he.n_fixed);
                std::cout << "    FIXED sum(err^2) by bin [pct of total " << tot_sumsq << " m^2]: "
                          << "<0.5m=" << he.fixed_hist_sumsq[0] << " (" << (tot_sumsq > 0 ? 100.0*he.fixed_hist_sumsq[0]/tot_sumsq : 0.0) << "%) "
                          << "0.5-1m=" << he.fixed_hist_sumsq[1] << " (" << (tot_sumsq > 0 ? 100.0*he.fixed_hist_sumsq[1]/tot_sumsq : 0.0) << "%) "
                          << "1-2m=" << he.fixed_hist_sumsq[2] << " (" << (tot_sumsq > 0 ? 100.0*he.fixed_hist_sumsq[2]/tot_sumsq : 0.0) << "%) "
                          << "2-5m=" << he.fixed_hist_sumsq[3] << " (" << (tot_sumsq > 0 ? 100.0*he.fixed_hist_sumsq[3]/tot_sumsq : 0.0) << "%) "
                          << "5-10m=" << he.fixed_hist_sumsq[4] << " (" << (tot_sumsq > 0 ? 100.0*he.fixed_hist_sumsq[4]/tot_sumsq : 0.0) << "%) "
                          << ">=10m=" << he.fixed_hist_sumsq[5] << " (" << (tot_sumsq > 0 ? 100.0*he.fixed_hist_sumsq[5]/tot_sumsq : 0.0) << "%)\n";
                double tail_sumsq = 0.0;
                for (const auto& [tow, err] : he.fixed_worst) tail_sumsq += err * err;
                std::cout << "    FIXED worst-15 sum(err^2)=" << tail_sumsq
                          << " m^2 of total=" << (he.fixed_rms * he.fixed_rms * double(he.n_fixed))
                          << " m^2 (" << (he.n_fixed > 0 ? 100.0 * tail_sumsq /
                                (he.fixed_rms * he.fixed_rms * double(he.n_fixed)) : 0.0)
                          << "%)\n    FIXED worst offenders (tow, horiz_m):";
                for (const auto& [tow, err] : he.fixed_worst) {
                    std::cout << " (" << tow << "," << err << ")";
                }
                std::cout << "\n";
            }
            // Stationary-epoch velocity: should collapse to ~0 with ZUPT.
            if (!fl.epoch_velocity_nav_mps.empty()) {
                double vsum = 0.0, vmax = 0.0;
                std::size_t vn = 0, ri = 0;
                for (std::size_t i = 0; i < ne && i < fl.epoch_velocity_nav_mps.size(); ++i) {
                    const auto& sol = fl.solution.solutions[i];
                    while (ri + 1 < ref_rows.size() &&
                           std::abs(ref_rows[ri + 1].time - sol.time) <
                               std::abs(ref_rows[ri].time - sol.time)) {
                        ++ri;
                    }
                    if (std::abs(ref_rows[ri].time - sol.time) > 0.11) continue;
                    const double ref_speed = std::hypot(ref_rows[ri].ve, ref_rows[ri].vn);
                    if (ref_speed > 0.1) continue;  // reference-stationary epochs only
                    const auto& v = fl.epoch_velocity_nav_mps[i];
                    const double sp = v.norm();
                    vsum += sp * sp;
                    vmax = std::max(vmax, sp);
                    ++vn;
                }
                if (vn > 0) {
                    std::cout << "    stationary-epoch |velocity| (ref speed<0.1): rms="
                              << std::sqrt(vsum / double(vn)) << " m/s max=" << vmax
                              << " m/s (n=" << vn << ")\n";
                }
            }
        }
        // Consistency vs 2b batch on the overlap (only when small enough that
        // the batch path is safe). Apples-to-apples: both float-only.
        if (ne <= 600) {
            double t_b = 0.0, t_flf = 0.0;
            const auto batch = run(problem_imu, config, libgnss::FGOBackend::GTSAM, false, t_b,
                                   /*use_pose3=*/true, /*use_imu=*/true);
            const auto fl_float = run(problem_imu, config, libgnss::FGOBackend::GTSAM, false, t_flf,
                                      /*use_pose3=*/true, /*use_imu=*/true, args.fixed_lag_s);
            const FloatParity cons = compareFloat(batch, fl_float);
            const HorizError he_batch = horizontalErrorVsRef(batch, ref_rows);
            const HorizError he_flf = horizontalErrorVsRef(fl_float, ref_rows);
            std::cout << "  consistency vs 2b batch (float-vs-float, over " << cons.compared_epochs
                      << " epochs): max=" << cons.max_delta_m << " m rms=" << cons.rms_delta_m << " m\n"
                      << "    batch-float horiz err vs ref:    rms=" << he_batch.float_rms << " m\n"
                      << "    smoother-float horiz err vs ref: rms=" << he_flf.float_rms << " m\n";
        }
        // --- 2e gap-attribution diagnostics (--diag) ---
        if (args.diag) {
            // (i) Frequency / signal content of the DD carrier factors: is this
            // single-frequency (L1/E1/B1 only) or multi-frequency? inuex35 runs
            // NF=3. This is an upstream (front-end / observation-model) property.
            std::map<int, std::size_t> signal_counts;
            auto isSecondFreq = [](libgnss::SignalType s) {
                switch (s) {
                    case libgnss::SignalType::GPS_L2P:
                    case libgnss::SignalType::GPS_L2C:
                    case libgnss::SignalType::GPS_L5:
                    case libgnss::SignalType::GLO_L2CA:
                    case libgnss::SignalType::GLO_L2P:
                    case libgnss::SignalType::GAL_E5A:
                    case libgnss::SignalType::GAL_E5B:
                    case libgnss::SignalType::BDS_B2I:
                    case libgnss::SignalType::BDS_B2A:
                    case libgnss::SignalType::QZS_L2C:
                    case libgnss::SignalType::QZS_L5:
                        return true;
                    default:
                        return false;
                }
            };
            std::size_t second_freq = 0;
            for (const auto& f : problem_imu.double_difference_carrier_factors) {
                ++signal_counts[static_cast<int>(f.signal)];
                if (isSecondFreq(f.signal)) ++second_freq;
            }
            std::cout << "\n  (i0) per-signal DD-carrier factor histogram:\n";
            for (const auto& [sig, cnt] : signal_counts) {
                std::cout << "       signal_enum=" << sig << "  count=" << cnt << "\n";
            }
            std::cout << "\n=== (2e diag) gap attribution on the SAME full-run1 FGOProblem ===\n"
                      << "  (i) DD-carrier signal content: " << signal_counts.size()
                      << " distinct signal(s); L2/L5/E5/B2 (2nd-freq) DD factors = " << second_freq
                      << " / " << problem_imu.double_difference_carrier_factors.size() << "  => "
                      << (second_freq > 0 ? "MULTI-frequency" : "SINGLE-frequency (L1/E1/B1 only)")
                      << "\n";

            // (ii) Apples-to-apples per-epoch ratio-gated fix-rate WITHOUT
            // fix-and-hold: Eigen native backend vs GTSAM fixed-lag. Same
            // problem, same ratio gate (config.lambda_ratio_threshold).
            double t_e = 0.0, t_g = 0.0;
            const auto eigen_fix = run(problem_imu, config, libgnss::FGOBackend::Eigen, true, t_e);
            const std::size_t eigen_fixed = countFixedEpochs(eigen_fix);
            const auto gtsam_nohold =
                run(problem_imu, config, libgnss::FGOBackend::GTSAM, true, t_g,
                    /*use_pose3=*/true, /*use_imu=*/true, args.fixed_lag_s, args.use_nhc,
                    args.use_zupt, /*use_hold=*/false);
            const std::size_t gtsam_nohold_fixed = countFixedEpochs(gtsam_nohold);
            const HorizError he_eigen = horizontalErrorVsRef(eigen_fix, ref_rows);
            std::cout << "  (ii) per-epoch ratio-gated fix-rate (NO fix-and-hold), same problem:\n"
                      << "       Eigen native backend: " << eigen_fixed << "/" << ne << " ("
                      << (ne > 0 ? 100.0 * double(eigen_fixed) / double(ne) : 0.0)
                      << "%), FIXED horiz rms=" << he_eigen.fixed_rms << " m (" << t_e << " s)\n"
                      << "       GTSAM fixed-lag:      " << gtsam_nohold_fixed << "/" << ne << " ("
                      << (ne > 0 ? 100.0 * double(gtsam_nohold_fixed) / double(ne) : 0.0)
                      << "%) (" << t_g << " s)\n"
                      << "       => port-faithfulness: GTSAM-no-hold vs Eigen within "
                      << std::abs(double(gtsam_nohold_fixed) - double(eigen_fixed)) << " epochs\n";

            // (iii) fix-and-hold before -> after (this run has hold on).
            std::cout << "  (iii) fix-and-hold effect: " << gtsam_nohold_fixed << "/" << ne << " ("
                      << (ne > 0 ? 100.0 * double(gtsam_nohold_fixed) / double(ne) : 0.0)
                      << "%) WITHOUT hold  ->  " << fl_fixed << "/" << ne << " ("
                      << (ne > 0 ? 100.0 * double(fl_fixed) / double(ne) : 0.0)
                      << "%) WITH hold\n";
            std::cout.flush();
        }

        const bool go_2c = fl.diagnostics.smoother_updates >= ne && nonfinite == 0 &&
                           none_epochs == 0 && fl.diagnostics.lambda_ambiguity_attempts > 0;
        std::cout << "\nRESULT (milestone 2c): fixed-lag full-scale run "
                  << (go_2c ? "GO" : "NO-GO") << " (epochs=" << ne
                  << ", peak_window_vars=" << fl.diagnostics.smoother_max_window_vars
                  << ", fix-rate=" << (ne > 0 ? 100.0 * double(fl_fixed) / double(ne) : 0.0)
                  << "%, wall=" << t_fl << " s)\n";
        std::cout.flush();
        return 0;
    }

    // --- (a) Float parity: LAMBDA off on both backends ---
    double t_ef = 0.0, t_gf = 0.0;
    const auto eigen_float = run(problem, config, libgnss::FGOBackend::Eigen, false, t_ef);
    const auto gtsam_float = run(problem, config, libgnss::FGOBackend::GTSAM, false, t_gf);
    const FloatParity fp = compareFloat(eigen_float, gtsam_float);

    std::cout << "\n=== (a) FLOAT parity (LAMBDA off) ===\n"
              << "  eigen: " << t_ef << " s, final_cost=" << eigen_float.diagnostics.final_cost
              << ", dd_cp_rms=" << eigen_float.diagnostics.double_difference_carrier_residual_rms_m
              << " m\n"
              << "  gtsam: " << t_gf << " s, iters=" << gtsam_float.diagnostics.iterations
              << ", final_cost=" << gtsam_float.diagnostics.final_cost
              << ", dd_cp_rms=" << gtsam_float.diagnostics.double_difference_carrier_residual_rms_m
              << " m, graph_values=" << gtsam_float.diagnostics.graph_values
              << " graph_factors=" << gtsam_float.diagnostics.graph_factors << "\n"
              << "  compared_epochs=" << fp.compared_epochs << "\n"
              << "  max ECEF delta = " << fp.max_delta_m << " m\n"
              << "  rms ECEF delta = " << fp.rms_delta_m << " m\n"
              << "  status_disagreements=" << fp.status_disagreements
              << " nonfinite(eigen=" << fp.nonfinite_eigen << ", gtsam=" << fp.nonfinite_gtsam
              << ")\n";
    std::cout.flush();

    // --- (a2) Milestone 2a: Pose3-GTSAM (lever-arm '...FactorArm' DD factors)
    // vs Point3-GTSAM antenna position parity, both with LAMBDA off. Attitude
    // is unobservable without IMU (2b), so only the recovered ANTENNA
    // position is validated here; the 2a success gate is Pose3 vs Point3
    // sub-cm agreement (docs/gtsam_backend_design.md, milestone 2a). ---
    double t_gp3 = 0.0;
    const auto gtsam_pose3_float =
        run(problem, config, libgnss::FGOBackend::GTSAM, false, t_gp3, /*use_pose3=*/true);
    const FloatParity fp_pose3_vs_point3 = compareFloat(gtsam_float, gtsam_pose3_float);
    const FloatParity fp_pose3_vs_eigen = compareFloat(eigen_float, gtsam_pose3_float);

    std::cout << "\n=== (a2) MILESTONE 2a: Pose3-GTSAM vs Point3-GTSAM antenna position parity ===\n"
              << "  lever_arm_body_flu=[" << kTokyoLeverArmX << ", " << kTokyoLeverArmY << ", "
              << kTokyoLeverArmZ << "] m\n"
              << "  pose3-gtsam: " << t_gp3 << " s, iters=" << gtsam_pose3_float.diagnostics.iterations
              << ", final_cost=" << gtsam_pose3_float.diagnostics.final_cost
              << ", dd_cp_rms=" << gtsam_pose3_float.diagnostics.double_difference_carrier_residual_rms_m
              << " m\n"
              << "  compared_epochs=" << fp_pose3_vs_point3.compared_epochs << "\n"
              << "  Pose3 vs Point3 (both GTSAM) max ECEF delta = " << fp_pose3_vs_point3.max_delta_m
              << " m\n"
              << "  Pose3 vs Point3 (both GTSAM) rms ECEF delta = " << fp_pose3_vs_point3.rms_delta_m
              << " m\n"
              << "  Pose3 (GTSAM) vs Eigen      max ECEF delta = " << fp_pose3_vs_eigen.max_delta_m
              << " m\n"
              << "  Pose3 (GTSAM) vs Eigen      rms ECEF delta = " << fp_pose3_vs_eigen.rms_delta_m
              << " m\n"
              << "  status_disagreements=" << fp_pose3_vs_point3.status_disagreements
              << " nonfinite(point3=" << fp_pose3_vs_point3.nonfinite_eigen
              << ", pose3=" << fp_pose3_vs_point3.nonfinite_gtsam << ")\n";
    std::cout.flush();

    const bool go_2a = fp_pose3_vs_point3.compared_epochs > 0 && fp_pose3_vs_point3.max_delta_m < 0.01;
    std::cout << "\nRESULT (milestone 2a): Pose3-vs-Point3 antenna-position parity "
              << (go_2a ? "GO" : "NO-GO") << " (max=" << fp_pose3_vs_point3.max_delta_m
              << " m rms=" << fp_pose3_vs_point3.rms_delta_m << " m over "
              << fp_pose3_vs_point3.compared_epochs << " epochs)\n";
    std::cout.flush();

    // --- IMU-coupled paths (2b batch or 2c fixed-lag). Attach IMU once. ---
    if (!args.imu_path.empty()) {
        libgnss::FGOProcessor::FGOProblem problem_imu = problem;
        if (buildImuInput(args.imu_path, problem_imu)) {
            applyImuNoiseOverrides(args, problem_imu);
            std::vector<RefRow> ref_rows;
            if (!args.ref_path.empty()) ref_rows = loadReference(args.ref_path);

            // ============================================================
            // (a3) MILESTONE 2b: IMU-coupled batch Pose3 (CombinedImuFactor).
            // (Fixed-lag 2c is handled earlier, before section (a).)
            // ============================================================
            double t_imu = 0.0;
            const auto gtsam_imu =
                run(problem_imu, config, libgnss::FGOBackend::GTSAM, false, t_imu,
                    /*use_pose3=*/true, /*use_imu=*/true);
            const FloatParity fp_imu_vs_pose3 = compareFloat(gtsam_pose3_float, gtsam_imu);
            const FloatParity fp_imu_vs_eigen = compareFloat(eigen_float, gtsam_imu);

            // Divergence / NaN check.
            std::size_t nonfinite = 0;
            for (const auto& s : gtsam_imu.solution.solutions) {
                if (!s.position_ecef.allFinite()) ++nonfinite;
            }

            std::cout << "\n=== (a3) MILESTONE 2b: IMU-coupled Pose3 (CombinedImuFactor) ===\n"
                      << "  imu: " << t_imu << " s, iters=" << gtsam_imu.diagnostics.iterations
                      << ", imu_intervals=" << gtsam_imu.diagnostics.imu_intervals
                      << ", initial_cost=" << gtsam_imu.diagnostics.initial_cost
                      << ", final_cost=" << gtsam_imu.diagnostics.final_cost
                      << ", graph_values=" << gtsam_imu.diagnostics.graph_values
                      << ", graph_factors=" << gtsam_imu.diagnostics.graph_factors << "\n"
                      << "  solve_ok=" << (gtsam_imu.diagnostics.converged ? "yes" : "no")
                      << ", nonfinite_epochs=" << nonfinite << "\n"
                      << "  IMU-Pose3 vs 2a-Pose3 (no IMU) antenna delta: max="
                      << fp_imu_vs_pose3.max_delta_m << " m rms=" << fp_imu_vs_pose3.rms_delta_m
                      << " m (over " << fp_imu_vs_pose3.compared_epochs << " epochs)\n"
                      << "  IMU-Pose3 vs Eigen antenna delta:            max="
                      << fp_imu_vs_eigen.max_delta_m << " m rms=" << fp_imu_vs_eigen.rms_delta_m
                      << " m\n";

            // Attitude observability + sanity (vs reference.csv if provided).
            const std::vector<RefRow>& ref = ref_rows;
            auto refAt = [&](const libgnss::GNSSTime& t) -> const RefRow* {
                const RefRow* best = nullptr;
                double best_dt = 0.25;
                for (const auto& r : ref) {
                    const double d = std::abs(r.time - t);
                    if (d < best_dt) { best_dt = d; best = &r; }
                }
                return best;
            };
            if (!gtsam_imu.epoch_attitude_rpy_deg.empty()) {
                const std::size_t ne = gtsam_imu.epoch_attitude_rpy_deg.size();
                const std::size_t samples[3] = {std::size_t(0), ne / 2, ne - 1};
                std::cout << "  estimated attitude [roll pitch heading] deg / velocity ENU (m/s):\n";
                for (std::size_t k = 0; k < 3; ++k) {
                    const std::size_t i = samples[k];
                    const auto& a = gtsam_imu.epoch_attitude_rpy_deg[i];
                    const auto& v = gtsam_imu.epoch_velocity_nav_mps[i];
                    std::cout << "    epoch " << i << ": [" << a.transpose() << "]  v=["
                              << v.transpose() << "]";
                    const RefRow* r = refAt(gtsam_imu.solution.solutions[i].time);
                    if (r) {
                        std::cout << "  ref[roll pitch heading]=[" << r->roll_deg << " "
                                  << r->pitch_deg << " " << r->heading_deg << "] ref_v=["
                                  << r->ve << " " << r->vn << " " << r->vu << "]";
                    }
                    std::cout << "\n";
                }
                // Heading RMS error vs reference over moving epochs (speed>1 m/s).
                if (!ref.empty()) {
                    double sum_sq = 0.0; std::size_t cnt = 0;
                    for (std::size_t i = 0; i < ne; ++i) {
                        const auto& v = gtsam_imu.epoch_velocity_nav_mps[i];
                        if (std::hypot(v.x(), v.y()) < 1.0) continue;
                        const RefRow* r = refAt(gtsam_imu.solution.solutions[i].time);
                        if (!r) continue;
                        double dh = gtsam_imu.epoch_attitude_rpy_deg[i].z() - r->heading_deg;
                        while (dh > 180.0) dh -= 360.0;
                        while (dh < -180.0) dh += 360.0;
                        sum_sq += dh * dh; ++cnt;
                    }
                    if (cnt > 0) {
                        std::cout << "  heading RMS error vs reference (moving epochs, n=" << cnt
                                  << ") = " << std::sqrt(sum_sq / double(cnt)) << " deg\n";
                    }
                }
            }
            // Confirm per-epoch LAMBDA (gtsam::Marginals over the now
            // IMU-augmented graph, with the 2a rotation pin removed) still
            // factorizes -- the 2b requirement that Marginals succeeds.
            double t_imu_fix = 0.0;
            const auto gtsam_imu_fix =
                run(problem_imu, config, libgnss::FGOBackend::GTSAM, true, t_imu_fix,
                    /*use_pose3=*/true, /*use_imu=*/true);
            const std::size_t imu_fixed_epochs = countFixedEpochs(gtsam_imu_fix);
            const bool marginals_ok = gtsam_imu_fix.diagnostics.lambda_ambiguity_attempts > 0;
            std::cout << "  IMU + per-epoch LAMBDA: marginals_ok=" << (marginals_ok ? "yes" : "no")
                      << " attempts=" << gtsam_imu_fix.diagnostics.lambda_ambiguity_attempts
                      << " fixed_epochs=" << imu_fixed_epochs << "/"
                      << gtsam_imu_fix.solution.solutions.size()
                      << " lambda_ratio=" << gtsam_imu_fix.diagnostics.lambda_ambiguity_ratio
                      << " (" << t_imu_fix << " s)\n";

            const bool go_2b = gtsam_imu.diagnostics.converged && nonfinite == 0 &&
                               gtsam_imu.diagnostics.imu_intervals > 0 &&
                               fp_imu_vs_pose3.compared_epochs > 0 && marginals_ok;
            std::cout << "\nRESULT (milestone 2b): IMU-coupled solve "
                      << (go_2b ? "GO" : "NO-GO") << " (imu_intervals="
                      << gtsam_imu.diagnostics.imu_intervals << ", nonfinite=" << nonfinite
                      << ", IMU-vs-2a max delta=" << fp_imu_vs_pose3.max_delta_m << " m)\n";
            std::cout.flush();
        }
    }

    if (args.float_only) {
        const bool go = fp.compared_epochs > 0 && fp.max_delta_m < 0.05;
        std::cout << "\nRESULT (float-only): float-parity " << (go ? "GO" : "NO-GO")
                  << " (max=" << fp.max_delta_m << " m rms=" << fp.rms_delta_m << " m over "
                  << fp.compared_epochs << " epochs)\n";
        std::cout.flush();
        return 0;
    }

    // --- (b) Fix agreement: LAMBDA on both backends ---
    double t_ex = 0.0, t_gx = 0.0;
    const auto eigen_fix = run(problem, config, libgnss::FGOBackend::Eigen, true, t_ex);
    const auto gtsam_fix = run(problem, config, libgnss::FGOBackend::GTSAM, true, t_gx);

    const std::size_t e_fixed_epochs = countFixedEpochs(eigen_fix);
    const std::size_t g_fixed_epochs = countFixedEpochs(gtsam_fix);
    // Per-epoch fix-status disagreement (one FIXED, the other not).
    std::size_t fix_disagreements = 0;
    const std::size_t nfx =
        std::min(eigen_fix.solution.solutions.size(), gtsam_fix.solution.solutions.size());
    for (std::size_t i = 0; i < nfx; ++i) {
        const bool ef = eigen_fix.solution.solutions[i].status == libgnss::SolutionStatus::FIXED;
        const bool gf = gtsam_fix.solution.solutions[i].status == libgnss::SolutionStatus::FIXED;
        if (ef != gf) {
            ++fix_disagreements;
        }
    }

    const FixedDelta fd = compareFixedPositions(eigen_fix, gtsam_fix);

    std::cout << "\n=== (b) FIXED / LAMBDA agreement (per-epoch LAMBDA on both) ===\n"
              << "  eigen: fixed_solution=" << (eigen_fix.diagnostics.fixed_solution ? "1" : "0")
              << " fixed_ambiguities=" << eigen_fix.diagnostics.fixed_ambiguities
              << " lambda_ratio=" << eigen_fix.diagnostics.lambda_ambiguity_ratio
              << " fixed_epochs=" << e_fixed_epochs << "/" << eigen_fix.solution.solutions.size()
              << " (" << t_ex << " s)\n"
              << "  gtsam: fixed_solution=" << (gtsam_fix.diagnostics.fixed_solution ? "1" : "0")
              << " fixed_ambiguities=" << gtsam_fix.diagnostics.fixed_ambiguities
              << " lambda_ratio=" << gtsam_fix.diagnostics.lambda_ambiguity_ratio
              << " lambda_candidates=" << gtsam_fix.diagnostics.lambda_ambiguity_candidates
              << " fixed_epochs=" << g_fixed_epochs << "/" << gtsam_fix.solution.solutions.size()
              << " (" << t_gx << " s)\n"
              << "  per-epoch fix-status disagreements=" << fix_disagreements << "\n"
              << "  both-fixed epochs=" << fd.both_fixed
              << ", fixed-position delta: max=" << fd.max_delta_m << " m rms=" << fd.rms_delta_m
              << " m\n";

    // --- (b2) Milestone 2a: does per-epoch LAMBDA still work on the Pose3
    // path? gtsam::Marginals needs the graph Hessian to be non-singular;
    // the Pose3RotationPrior gauge pin (fgo_gtsam_backend.cpp) exists
    // specifically so Cholesky succeeds here. ---
    double t_gp3x = 0.0;
    const auto gtsam_pose3_fix =
        run(problem, config, libgnss::FGOBackend::GTSAM, true, t_gp3x, /*use_pose3=*/true);
    const std::size_t gp3_fixed_epochs = countFixedEpochs(gtsam_pose3_fix);
    const FixedDelta fd_pose3 = compareFixedPositions(gtsam_fix, gtsam_pose3_fix);
    std::size_t fix_disagreements_pose3 = 0;
    const std::size_t nfx3 =
        std::min(gtsam_fix.solution.solutions.size(), gtsam_pose3_fix.solution.solutions.size());
    for (std::size_t i = 0; i < nfx3; ++i) {
        const bool gf = gtsam_fix.solution.solutions[i].status == libgnss::SolutionStatus::FIXED;
        const bool pf = gtsam_pose3_fix.solution.solutions[i].status == libgnss::SolutionStatus::FIXED;
        if (gf != pf) {
            ++fix_disagreements_pose3;
        }
    }
    std::cout << "\n=== (b2) MILESTONE 2a: Pose3-GTSAM per-epoch LAMBDA (Marginals/Cholesky) ===\n"
              << "  pose3-gtsam: fixed_solution=" << (gtsam_pose3_fix.diagnostics.fixed_solution ? "1" : "0")
              << " fixed_ambiguities=" << gtsam_pose3_fix.diagnostics.fixed_ambiguities
              << " lambda_ratio=" << gtsam_pose3_fix.diagnostics.lambda_ambiguity_ratio
              << " fixed_epochs=" << gp3_fixed_epochs << "/" << gtsam_pose3_fix.solution.solutions.size()
              << " (" << t_gp3x << " s)\n"
              << "  per-epoch fix-status disagreements vs Point3-GTSAM=" << fix_disagreements_pose3 << "\n"
              << "  both-fixed epochs=" << fd_pose3.both_fixed
              << ", fixed-position delta vs Point3-GTSAM: max=" << fd_pose3.max_delta_m
              << " m rms=" << fd_pose3.rms_delta_m << " m\n";

    // Go/No-go on the float parity (the Phase-1 gate).
    const bool go = fp.compared_epochs > 0 && fp.max_delta_m < 0.05 /* 5 cm */;
    std::cout << "\nRESULT: float-parity " << (go ? "GO" : "NO-GO")
              << " (max=" << fp.max_delta_m << " m over " << fp.compared_epochs << " epochs)\n";
    return 0;
}
