#pragma once

#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/core/types.hpp>
#include <libgnss++/io/imu.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace libgnss {

/**
 * @brief Selects which numerical backend FGOProcessor::optimizeProblem uses.
 *
 * `Eigen` (default) is the native dense/sparse Gauss-Newton-ish solver
 * implemented directly in fgo.cpp. `GTSAM` delegates to a GTSAM
 * NonlinearFactorGraph + LevenbergMarquardtOptimizer built in
 * fgo_gtsam_backend.cpp (only available when the library is built with
 * GTSAM found, i.e. GNSSPP_HAS_GTSAM is defined). Selecting GTSAM when the
 * library was built without it is a no-op: optimizeProblem falls back to the
 * Eigen backend.
 */
enum class FGOBackend {
    Eigen,
    GTSAM,
};

/**
 * @brief Batch pseudorange factor-graph optimizer.
 *
 * This is the native Eigen backend for the GNSS FGO pipeline. It keeps the
 * factor/problem representation explicit so a GTSAM backend can be added later
 * without changing callers that prepare GNSS factors from RINEX/navigation data.
 */
class FGOProcessor {
public:
    struct FGOConfig {
        FGOBackend backend = FGOBackend::Eigen;
        int max_iterations = 8;
        double convergence_threshold_m = 1e-4;
        double relative_cost_convergence_threshold = 0.0;
        double absolute_cost_convergence_threshold = 0.0;
        double pseudorange_sigma_m = 3.0;
        double pseudorange_elevation_sigma_power = 1.0;
        double min_elevation_deg = 10.0;
        double min_snr_dbhz = 0.0;
        double double_difference_reference_min_snr_dbhz = -1.0;
        double double_difference_base_min_snr_dbhz = -1.0;
        int min_satellites_per_epoch = 4;
        int min_output_double_difference_carrier_factors_per_epoch = 0;
        double max_float_seed_position_divergence_m = 0.0;
        double max_float_position_jump_m = 0.0;

        bool use_spp_seed = true;
        bool use_pseudorange_factors = true;
        bool use_motion_factors = true;
        bool use_position_motion_factors = true;
        bool use_clock_motion_factors = true;
        bool use_tdcp_factors = true;
        bool use_carrier_phase_factors = false;
        bool use_double_difference_factors = false;
        // When true (default) AND use_double_difference_factors, the DD
        // problem builder emits DD pseudorange + carrier factors for
        // secondary-band signals too (GPS L2C/L5, Galileo E5A/E5B/E6, BeiDou
        // B2I/B3I/B2A, QZSS L2C/L5), not just the primary band (L1CA/E1/B1I).
        // Each (satellite, signal, arc) still gets its own AmbiguityState /
        // ambiguity index with its own wavelength (see signals.hpp), and the
        // reference satellite is selected independently per (system, signal)
        // group so a band with fewer tracked satellites doesn't starve. This
        // is the front-end lever for closing the fix-rate gap vs multi-
        // frequency references (inuex35 runs NF=3): feeding the raw per-
        // frequency DD carrier factors to the joint LAMBDA search already
        // exploits the inter-frequency information without needing explicit
        // widelane/narrowlane combinations. GLONASS is unaffected: its DD
        // factors are already skipped upstream (FDMA, no shared wavelength).
        // Ignored when use_multi_constellation is false (single GPS-L1 mode).
        //
        // Default OFF: on the tokyo1 low-cost PPC data, adding raw L2/L5 DD
        // measurements REGRESSES the DD-RTK solution vs single-frequency
        // (fix-rate 52.5% -> 47%, FLOAT 1.6 m -> 7.8 m, <50cm 58% -> 37% on the
        // first-2500 good-geometry subset), and neither per-band weighting nor
        // the LAMBDA-independence lever below recovers it (best multi-freq
        // fix-rate 43.9% still < 52.5%). The secondary-band DD carries a
        // systematic bias that degrades the float even when correctly
        // down-weighted -- most consistent with a rover/base observation-CODE
        // mismatch per band (RINEX C2W/C2L at the rover vs C2W/C2X at the base
        // both collapse to GPS_L2C, leaving an uncancelled inter-code/DCB bias
        // in the DD). The capability is fully wired; enable it only after the
        // secondary-band measurement hygiene (identical code selection at
        // rover+base, or DCB correction) lands. A wide-lane combination would
        // INHERIT the same bias, so it is not the right next step.
        bool use_multi_frequency_double_difference = false;

        // --- MF hygiene: secondary-band observation-code alignment ---
        // Port of the cssrlib/inuex35 (RTKLIB-style) signal-selection policy:
        // each receiver uses exactly ONE RINEX observation code per (system,
        // band) for the whole file (chosen from its own data by an RTKLIB
        // code-priority table), so all satellites contribute the same code per
        // band and the between-satellite difference cancels the receiver-side
        // inter-code bias. Cures the mixed-code DD bias (rover GPS L2 mixing
        // C2W/C2L across satellites; base BDS B2I C7D vs rover C7I) diagnosed
        // as the multi-frequency FLOAT blowup. Secondary bands only; the
        // primary-band single-frequency path is untouched. No-op when
        // use_multi_frequency_double_difference is false.
        bool use_double_difference_secondary_code_alignment = true;

        // --- MF-AR step 1: multi-frequency ambiguity-resolution robustness ---
        // Per-band DD measurement weighting. Secondary-band (L2C/L5/E5a/E5b/
        // B2I/B2a/...) DD carrier & pseudorange factors are noisier and more
        // slip-prone on low-cost receivers; adding them at L1-identical sigma
        // over-weights them and drags the float (observed: naive multi-freq
        // blew the FLOAT RMS 1.6 m -> 7.8 m). The per-factor sigma is
        // multiplied by these scales for secondary-band signals only, on top
        // of the existing elevation weighting. 1.0 = no de-weighting (the old
        // behaviour). No-op for single-frequency DD (no secondary factors).
        double double_difference_secondary_carrier_sigma_scale = 3.0;
        double double_difference_secondary_pseudorange_sigma_scale = 2.0;
        // LAMBDA candidate independence. When true, the per-epoch LAMBDA
        // ratio-test candidate set keeps at most one ambiguity per satellite
        // (primary band preferred, else the longest-wavelength secondary), so
        // the integer set spans INDEPENDENT satellites instead of multiple
        // highly-correlated bands of the same satellite. Multiple bands of one
        // satellite share the same line-of-sight, so they add little to the DD
        // geometry but do inflate the all-or-nothing ratio test and lower the
        // ratio (observed: naive multi-freq dropped fix-rate 52.5% -> 47.0%).
        // The dropped bands' DD factors still constrain the float; they are
        // just not forced into the integer ratio test. No-op for single-
        // frequency DD (one band per satellite already).
        //
        // Default OFF: measured net-harmful on tokyo1. Because this pipeline's
        // fix-rate is dominated by fix-and-hold (pinning many arcs early), a
        // SMALLER integer candidate set pins fewer arcs and lowers the
        // sustained fix-rate (observed: enabling it dropped multi-freq fix-rate
        // from 47% to 29%). Kept as an opt-in knob for a future partial-AR
        // scheme that fixes independent satellites first, then adds bands.
        bool double_difference_lambda_one_band_per_satellite = false;
        bool use_single_difference_doppler_factors = false;
        bool use_single_difference_tdcp_factors = false;
        bool use_velocity_states = false;
        bool use_velocity_motion_factors = false;
        bool use_ambiguity_between_factors = false;
        bool linearize_double_difference_factors_at_seed = false;
        bool reset_double_difference_ambiguities_each_epoch = false;
        bool use_inter_system_biases = true;
        bool use_ambiguity_priors = true;
        bool fix_ambiguities = false;
        bool prefer_double_difference_ambiguity_fixing = true;
        bool use_lambda_ambiguity_fix = true;
        bool use_epoch_lambda_fixed_output = false;
        bool use_partial_lambda_ambiguity_fix = true;
        // Independently re-optimize the active fixed-lag graph with the
        // candidate integers imposed as tight priors. The candidate is
        // accepted only when the original (prior-free) active graph cost
        // does not worsen beyond the configured tolerance. This also checks
        // that constrained batch refinement remains consistent with the
        // active incremental solution. Default OFF.
        bool use_integer_constrained_reoptimization = false;
        double integer_constrained_prior_sigma_cycles = 1e-3;
        double integer_constrained_cost_abs_tolerance = 1e-6;
        int integer_constrained_max_iterations = 1;
        bool use_robust_loss = true;
        double motion_sigma_m = 50.0;
        double clock_motion_sigma_m = 300.0;
        double velocity_prior_sigma_mps = 100.0;
        double velocity_motion_sigma_m = 0.01;
        double ambiguity_between_sigma_cycles = 0.001;
        double position_prior_sigma_m = 0.0;
        double clock_prior_sigma_m = 0.0;
        double tdcp_sigma_m = 0.03;
        double carrier_phase_sigma_m = 0.01;
        double single_difference_doppler_sigma_mps = 0.2;
        double single_difference_tdcp_sigma_m = 0.003;
        double double_difference_pseudorange_sigma_m = 1.0;
        double double_difference_carrier_sigma_m = 0.02;

        // --- Elevation-dependent DD sigma (RTKLIB-demo5 "varerr", rtkpos.c:402)
        // ---
        // Port of the inuex35 reference's preprocess/prefit.py varerr_dd_sigma
        // + buildfactor/factors.py pair_sigma. This was a previously-UNPORTED
        // piece: it ran DEFAULT-ON in every reference benchmark (its enable
        // flag, config.py's varerr_enable, is a dataclass default of 1, never
        // toggled by the benchmark profile dicts we had scraped), so earlier
        // parity work missed it entirely.
        //
        // When enabled, the DD pseudorange/carrier base sigma (fed to BOTH
        // double_difference_pseudorange_factor.sigma_m and
        // double_difference_carrier_factor.sigma_m, before the secondary-band
        // scale and sat-badness inflation multiply on top -- see
        // buildDoubleDifferenceProblem in fgo.cpp) is, per (reference,target)
        // pair:
        //
        //   el_pair  = max(min(el_ref_rad, el_target_rad), el_min_rad)
        //   el_min_rad = radians(max(1.0, min_elevation_deg))
        //   fact     = elevation_sigma_pseudorange_ratio (pseudorange) | 1.0 (carrier)
        //   a        = fact * elevation_sigma_err_a_m
        //   b        = fact * elevation_sigma_err_b_m
        //   d        = SPEED_OF_LIGHT * elevation_sigma_clock_stability * dt_s
        //   sinel    = max(sin(el_pair), 0.05)   // cap below ~3 deg
        //   var_sd   = 2*(a*a + b*b/(sinel*sinel)) + d*d   // single-difference variance
        //   sigma_dd = sqrt(2 * var_sd)                    // DD = difference of 2 SDs
        //
        // dt_s mapping decision: the reference's tc._epoch_dt is "actual
        // seconds since the runner's last process() call" -- i.e. the ROVER's
        // own epoch-to-epoch interval (defaults to 0.2 s / 5 Hz until the
        // first real update; see runner.py's _update_epoch_dt), NOT the
        // rover-base differential/interpolation age. Our port mirrors this
        // exactly: dt_s is the elapsed time between the current and the
        // immediately-preceding ROVER epoch in the batch (problem.epochs[i].time
        // - problem.epochs[i-1].time), persisted across epochs and only
        // updated when positive (same "keep last known dt" behaviour as the
        // reference), defaulting to 0.2 s before the first update -- which
        // also happens to match the tokyo PPC dataset's actual 0.200 s rover
        // observation interval (data/PPC-Dataset/tokyo/run*/rover.obs
        // INTERVAL header). The clock term's contribution is small next to
        // the a/b terms at these defaults, so this default's exact value is
        // not performance-critical.
        //
        // Undifferenced-PR decision: the reference's varerr_dd_sigma /
        // pair_sigma are only ever called from factors.py's DD pseudorange
        // and DD carrier factor builders -- nothing in the reference feeds
        // varerr into the undifferenced (single-receiver SPP-style)
        // pseudorange factor. This port matches that: use_elevation_dependent_sigma
        // affects ONLY the DD pseudorange/carrier sigma_m computed in
        // buildDoubleDifferenceProblem; FGOConfig::pseudorange_sigma_m and the
        // undifferenced PseudorangeFactor path (buildPseudorangeProblem) are
        // untouched.
        //
        // Default OFF (use_elevation_dependent_sigma = false): bit-identical
        // to the pre-existing flat/elevation-power sigma
        // (band_scale * double_difference_{pseudorange,carrier}_sigma_m /
        // sqrt(sin(el))). Turning it on switches the DD base-sigma SOURCE only;
        // composition order with the other sigma multipliers is unchanged:
        // varerr (or the flat fallback) -> secondary-band scale
        // (double_difference_secondary_{carrier,pseudorange}_sigma_scale,
        // still multiplies on top) -> sat-badness EWMA inflation (still
        // multiplies factor.sigma_m in the GTSAM backend, unaffected by this
        // knob) -> robust/Huber wrapping (unaffected).
        bool use_elevation_dependent_sigma = false;
        double elevation_sigma_err_a_m = 0.001;             ///< reference err_a (RTKLIB err[1]=0.003)
        double elevation_sigma_err_b_m = 0.001;              ///< reference err_b (RTKLIB err[2]=0.003)
        double elevation_sigma_pseudorange_ratio = 100.0;    ///< reference err_eratio_pr (RTKLIB eratio[0]=300)
        double elevation_sigma_clock_stability = 5e-12;      ///< reference err_sclkstab [s/s]

        double pseudorange_huber_threshold_sigma = 4.0;
        double carrier_phase_huber_threshold_sigma = 4.0;
        double tdcp_huber_threshold_sigma = 4.0;
        double ambiguity_prior_sigma_m = 1000.0;
        double fixed_ambiguity_sigma_m = 0.003;
        double ambiguity_fix_max_fractional_cycles = 0.15;
        double lambda_ratio_threshold = 3.0;
        // PPC satellite-count adaptive ratio schedule:
        // >=20:1.5, 15..19:2.0, 10..14:2.5, <10:3.0.
        bool use_satellite_count_adaptive_ratio = false;
        double adaptive_ratio_nsat20 = 1.5;
        double adaptive_ratio_nsat15 = 2.0;
        double adaptive_ratio_nsat10 = 2.5;
        double adaptive_ratio_nsat_low = 3.0;
        // Validate a candidate integer subset back in the DD carrier
        // observation domain before reporting FIX.  Thresholds <= 0 disable
        // that individual test; the whole gate is opt-in so existing runs
        // remain behavior-identical while diagnostics are collected.
        bool use_fixed_hypothesis_postfit_validation = false;
        bool fixed_postfit_validate_normal_ratio = false;
        // When > 0, apply the normal-ratio postfit gate only to candidates at
        // or below this ratio. This targets near-threshold ambiguity fixes
        // without discarding high-aperture fixes solely for noisy DD code.
        double fixed_postfit_normal_ratio_ceiling = 0.0;
        int fixed_postfit_min_factors = 4;
        double fixed_postfit_max_rms_m = 0.0;
        double fixed_postfit_max_normalized_residual = 0.0;
        double fixed_postfit_max_chi2_per_dof = 0.0;
        // Permit a lower LAMBDA ratio only when the implied fixed position
        // agrees with both the float solution and the IMU-predicted pose.
        // This is a solution-separation aperture test; the normal ratio gate
        // remains authoritative when this opt-in path is disabled.
        bool use_imu_aided_ratio_aperture = false;
        double imu_aided_relaxed_ratio_threshold = 1.0;
        double imu_aided_max_float_separation_m = 0.5;
        double imu_aided_max_prediction_separation_m = 2.0;
        // RTK-EVC-style validation: propagate several recent accepted FIX
        // positions with the optimized velocity history, then require the new
        // fixed candidate to agree with their weighted DR prediction.
        bool use_fixed_history_dr_validation = false;
        int fixed_history_dr_window_epochs = 50;
        double fixed_history_dr_max_separation_m = 0.5;
        // "c2" lever: a candidate that FAILS the fixed-history-DR
        // consistency check above is accepted anyway when the independent
        // surplus-satellite validation (use_surplus_satellite_validation,
        // which must ALSO be enabled) already evaluated and PASSED for this
        // same attempt. Surplus validation re-differences DD carriers that
        // did NOT produce the candidate against an alternate reference
        // satellite -- an independent geometric cross-check, unlike the DR
        // gate which only re-uses this arc's own recent fixed history. Two
        // independent tests agreeing is treated as sufficient integrity
        // evidence to override the (single) DR gate's rejection. Default
        // OFF; when off (or when use_surplus_satellite_validation is off)
        // this override never fires and behavior is byte-identical to
        // before this knob existed. See fixed_history_dr_surplus_overrides
        // in FGODiagnostics and dr_bypass_* in FGOEpochDiagnostics for the
        // monitor-style counterfactual (candidate ECEF position, logged
        // whenever the raw DR verdict alone would have blocked a
        // surplus-passed candidate) used to characterize this lever before
        // trusting it.
        bool surplus_validation_overrides_history_dr = false;
        // Conditional variant: require the surplus pool that decided THIS
        // attempt's pass to contain at least this many surplus satellites
        // (epoch_diagnostics[i].surplus_validation_surplus_used) before the
        // override may fire, on top of the mandatory surplus_evaluated+pass
        // above. <=0 = no extra floor (any surplus-passed candidate that
        // clears surplus_validation_min_surplus_satellites qualifies).
        // Measured on tokyo1 full-run1: the worst bypassed candidates
        // (>1 m true error) cluster heavily at surplus_used==min_n (the
        // weakest possible independent check); requiring >=4 cuts run1's
        // bypassed-population RMS roughly in half while keeping most of the
        // fix-rate gain.
        int surplus_validation_overrides_history_dr_min_surplus_used = 0;
        // Consecutive-run cap on the override above. A SUSTAINED run of
        // consecutive overrides is a fundamentally different risk than a
        // scattered single-epoch one: the wrong position basin persists
        // across the whole run, so the DR gate keeps (correctly) rejecting
        // while the override keeps (incorrectly) accepting. Measured on
        // tokyo run2 full (min_surplus_used=4): 11 of 328 overrides landed
        // >5 m true error, and ALL 11 sit inside one continuous ~4 s /
        // 15-epoch override run where ratio (up to 4.27) and the surplus
        // check were correlated-fooled together; every OTHER override run
        // in the same log (lengths up to 50) is clean (sub-30 cm).
        // <=0 = no cap (unlimited consecutive overrides; byte-identical to
        // before this knob existed). >0 = an override may only fire while
        // FEWER than this many immediately-preceding FIXED epochs were
        // themselves overrides; once the cap is hit, the DR gate's
        // rejection stands (this epoch falls back to FLOAT / whatever the
        // non-override paths decide) until a normally-validated FIXED
        // epoch (normal ratio path or an existing ambiguity hold; NOT a
        // FLOAT epoch) resets the streak. FLOAT epochs in between
        // deliberately do NOT reset the streak and do NOT count toward
        // it -- offline replay of the tokyo run2 monitor CSV shows the bad
        // run's intervening epochs are themselves all override-or-FLOAT,
        // so a FLOAT-resets rule would never let the cap engage at all.
        // Offline replay of that same CSV also shows this is a blunt
        // instrument: because the FIRST override of any run is always
        // admitted (the streak starts at 0), no K can drive the bad run's
        // >5 m count to zero without K=0 (i.e. turning the override off
        // outright); and because several of the CLEAN override runs are
        // themselves far longer (up to 50 epochs) than the bad run
        // (15 epochs), a K small enough to meaningfully shorten the bad
        // run also truncates most of the clean runs. See
        // fixed_history_dr_surplus_override_capped in FGODiagnostics for
        // the count this knob suppressed.
        int surplus_validation_overrides_history_dr_max_consecutive = 0;
        // Independent solution-separation track: propagate the last
        // high-confidence FIX using an SD-Doppler-only velocity LS solution,
        // then test the next integer candidate with a 3-D Mahalanobis chi2.
        bool use_external_doppler_dr_validation = false;
        bool external_doppler_dr_require_for_relaxed_fix = true;
        // A statistically validated relaxed-ratio solution may label the
        // current epoch FIXED, but must not become a persistent graph prior
        // until it also passes the configured (non-relaxed) ratio test.
        // This prevents one marginal early decision from steering all later
        // ambiguity estimates through fix-and-hold.
        bool allow_relaxed_ratio_fix_and_hold = false;
        int external_doppler_dr_max_age_epochs = 30;
        int external_doppler_dr_min_factors = 4;
        double external_doppler_dr_chi2_threshold = 11.345;  ///< 99% / 3 dof
        double external_doppler_dr_process_noise_mps = 0.5;
        double external_doppler_dr_reset_min_ratio = 3.0;
        // Independent current-epoch DDPR-LS/FDE anchor may rescue a candidate
        // rejected by the IMU/temporal aperture when both solutions agree.
        bool use_ddpr_anchor_aided_validation = false;
        double ddpr_anchor_validation_max_separation_m = 1.0;
        // Try the PPC constellation partial-AR cascade as separate LAMBDA
        // pools: GQEBR -> GQEB -> GQER -> GQB -> GQR -> GQ.
        bool use_constellation_ranked_partial_ar = false;
        // Remove ambiguity candidates implicated by a gross current-epoch
        // DD pseudorange residual before the PPC constellation cascade.
        bool use_residual_screened_partial_ar = false;
        // Diagnostic-only RTKLIB-demo5-style satellite exclusion audit.
        // On a ratio-rejected epoch, retry LAMBDA after removing each target
        // satellite in turn and record the best candidate. Never changes the
        // selected subset, FIX/FLOAT status, graph, or hold state.
        bool monitor_ratio_impact_partial_ar = false;
        // Diagnostic-only ambiguity solution-separation audit. Eligible DD
        // ambiguities are grouped by constellation, whole constellation
        // groups are balanced into two satellite-disjoint partitions, and
        // each partition runs an independent reduced LAMBDA search. The
        // shared float graph still correlates both partitions, so this is
        // evidence telemetry rather than an acceptance authority.
        bool monitor_disjoint_constellation_ar = false;
        int disjoint_constellation_ar_min_ambiguities = 4;
        // Diagnostic-only two-stage multi-frequency ambiguity resolution.
        // Resolve one primary band per satellite, then condition the remaining
        // ambiguity distribution on those integers. Never changes estimator
        // state or the exported FIX/FLOAT decision.
        bool monitor_conditional_multiband_ar = false;
        // Diagnostic-only multi-epoch AR shadow. Ambiguity integers must be
        // identical on the same uninterrupted DD arc for the configured
        // number of consecutive epochs before they enter a fresh, reduced
        // LAMBDA search. Never changes graph state or FIX/FLOAT output.
        bool monitor_multiepoch_ar = false;
        int multiepoch_ar_min_consensus_epochs = 3;
        int multiepoch_ar_min_ambiguities = 4;
        // Diagnostic-only geometry-free slip/arc-continuity shadow.
        bool monitor_geometry_free_cycle_slip = false;
        // Diagnostic-only, receiver-clock-free temporal carrier shadow.
        // It forms satellite single differences against a causal reference,
        // resets continuity whenever that reference changes, and time-
        // differences only consecutive observations.  Unlike the legacy
        // Taroz-parity SD-TDCP path, it uses the actual previous/current LOS
        // vectors.  The shadow is never inserted into either optimizer and
        // cannot change the exported solution or FIX/FLOAT decision.
        bool monitor_clock_resilient_temporal_carrier = false;
        // Diagnostic-only temporal DD pseudorange quality monitor. It
        // compares measured DDPR changes with DD Doppler and the causal
        // pre-solve IMU pose prediction; no result is consumed by the graph.
        bool monitor_predicted_ddpr_quality = false;
        // Diagnostic-only causal random-walk model of persistent DD
        // pseudorange bias. It consumes the monitor above internally but is
        // independent of its output switch. No result is consumed by the
        // graph, ambiguity resolver, or exported solution.
        bool monitor_predicted_ddpr_bias_state = false;
        double predicted_ddpr_bias_process_noise_m_sqrt_s = 0.25;
        double predicted_ddpr_bias_initial_sigma_m = 5.0;
        double predicted_ddpr_bias_min_measurement_sigma_m = 0.5;
        double predicted_ddpr_bias_robust_update_sigma = 3.0;
        int predicted_ddpr_bias_min_prior_updates = 2;
        // When a rover/base single-difference geometry-free combination jumps
        // while the underlying rover arcs remain continuous, break both bands'
        // DD arcs after the confirmation interval below. Resetting both bands
        // is conservative because the geometry-free combination cannot identify
        // which integer changed. This is opt-in because it changes the graph;
        // the monitor above remains diagnostic-only.
        bool use_geometry_free_cycle_slip_reset = false;
        double geometry_free_cycle_slip_threshold_m = 0.05;
        // Debounce the inferred slip so an imminent receiver-side LLI, gap, or
        // dropout arc break can supersede it. The reset fires only if the same
        // receiver arc is still alive after this interval.
        double geometry_free_cycle_slip_confirmation_s = 1.0;
        // GICI-style PAR: prefer ambiguities with the smallest marginal
        // variance and do not attempt a subset whose least precise member
        // exceeds this standard deviation in cycles (<=0 disables the gate).
        bool use_variance_ranked_partial_ar = false;
        double partial_ar_max_std_cycles = 0.25;
        // GICI-style ambiguity reacquisition: when geometry is good and FDE
        // rejects less than the configured fraction, a long run of fresh-AR
        // failures means the ambiguity state itself is stale. Bump every live
        // arc generation after the streak so the next epoch starts from new
        // float ambiguities, without breaking the IMU chain or suppressing
        // carrier factors. Default off.
        bool use_continuous_unfix_ambiguity_reset = false;
        int continuous_unfix_reset_epochs = 10;
        int continuous_unfix_min_satellites = 10;
        double continuous_unfix_max_gdop = 2.0;
        double continuous_unfix_max_fde_reject_fraction = 0.1;
        // Fail-closed variant: reset live ambiguities only when a trusted
        // current-epoch DDPR anchor also disagrees with the optimized
        // antenna position.
        bool continuous_unfix_require_ddpr_anchor_disagreement = false;
        double continuous_unfix_anchor_min_gap_m = 1.0;
        int min_fixed_ambiguities = 4;
        int max_lambda_ambiguities = 12;
        // --- Surplus-satellite independent integrity validation ---
        // Rescues LAMBDA candidates whose ratio falls in
        // (imu_aided_relaxed_ratio_threshold, lambda_ratio_threshold] using
        // an INDEPENDENT re-check: DD carrier observations EXCLUDED from the
        // fixed subset this epoch (FDE quarantine, CMC level-exclusion,
        // partial-AR drops -- i.e. present in cp_by_epoch[i] but whose
        // ambiguity_index is not part of the accepted LAMBDA subset) are
        // re-differenced against an ALTERNATE reference satellite (a
        // fixed-set satellite, in the same (system,signal) group, other than
        // the group's own DD reference) and their distance from the nearest
        // integer cycle is tested against a PDOP-scaled aperture. Mirrors
        // the PPC paper (Okada/Sasaki/Ando 2024) reliability check and the
        // "Modified RTK-GNSS" MDPI paper (sensors 24-9-2712): validate a
        // fixed candidate using observations that did NOT produce it, not
        // observations correlated with it (unlike the IMU/temporal/DDPR-
        // anchor apertures above, which all re-use graph state derived from
        // the SAME fix). Default OFF; when off this whole block is skipped
        // and behavior is byte-identical to before this feature existed.
        bool use_surplus_satellite_validation = false;
        // Monitor mode: evaluate + record diagnostics (per-epoch pass/fail,
        // fallback level, surplus count) for EVERY LAMBDA attempt (both the
        // established ratio>=lambda_ratio_threshold zone and the relaxed
        // rescue zone) WITHOUT changing any FIX/FLOAT decision. Use this to
        // validate the test itself (confusion matrix vs reference.csv
        // horizontal error) before trusting it to rescue or veto.
        bool surplus_validation_monitor_only = false;
        // Stage B: also demote (veto) an already-accepted
        // ratio>=lambda_ratio_threshold fix that FAILS this test. Separate
        // knob, default off; evaluate in Stage A before enabling.
        bool surplus_validation_veto_high_ratio_fails = false;
        // Optional selective-veto aperture. Positive values restrict the
        // established-ratio veto to weak-ratio candidates and/or epochs with
        // poor DD-code consistency. Zero preserves the original all-candidate
        // veto semantics.
        double surplus_validation_veto_ratio_ceiling = 0.0;
        double surplus_validation_veto_min_ddpr_rms_m = 0.0;
        // Minimum number of surplus satellites required (at whichever
        // constellation fallback level is being tried) before the test can
        // render a verdict; below this the epoch is "insufficient surplus"
        // (does not rescue, does not veto, regardless of monitor mode).
        int surplus_validation_min_surplus_satellites = 2;
        // Nearest-integer aperture (cycles), selected by the epoch's PDOP
        // (computed from the FIXED-set geometry at the candidate fixed
        // antenna position -- see computeFixedSetPdop in
        // fgo_gtsam_backend.cpp).
        double surplus_validation_aperture_pdop_lt1_cycles = 0.1;
        double surplus_validation_aperture_pdop_1to2_cycles = 0.2;
        double surplus_validation_aperture_pdop_gt2_cycles = 0.3;
        // Aggregation over the (possibly constellation-fallback-reduced)
        // surplus pool: true (default) = ALL surplus satellites in the pool
        // must be within the aperture; false = a majority (>=
        // surplus_validation_majority_fraction) must be within it.
        bool surplus_validation_require_all = true;
        double surplus_validation_majority_fraction = 0.5;
        // --- Below-floor ("low-count") ambiguity resolution rescue ---
        // The per-epoch LAMBDA entry gate (min_candidates in
        // fgo_gtsam_backend.cpp, = max(6, min_fixed_ambiguities+1)) exists
        // because LAMBDA's ratio test alone is not a sufficient integrity
        // check below six candidates in this multipath environment --
        // unconditionally lowering that floor was tried and is net-negative
        // (more wrong-but-plausible-ratio fixes flood in than are rescued).
        // This knob instead lets LAMBDA additionally attempt epochs whose
        // in-window ambiguity count is below the floor (but still >=
        // low_count_min_candidates), and accepts the result ONLY when the
        // independent surplus-satellite validation (use_surplus_satellite_
        // validation, which must ALSO be enabled) evaluates AND passes for
        // that candidate -- the independent geometric check compensates for
        // the weak ratio statistics at low count. A below-floor fix is never
        // eligible to seed fix-and-hold (never pinned, never contributes to
        // the fixed-history-DR prediction window used to validate OTHER
        // epochs' relaxed-ratio candidates), so a wrong low-count fix cannot
        // poison a later epoch's arc; it remains subject to the normal
        // demotion pass (fix_demote_*) like every other FIXED epoch. Default
        // OFF; when off (or when use_surplus_satellite_validation is off)
        // this entire path is skipped and behavior is byte-identical to
        // before this feature existed.
        bool use_low_count_ambiguity_resolution = false;
        int low_count_min_candidates = 4;
        // Modest ratio floor for the low-count path, independent of (and in
        // addition to) the mandatory surplus-pass requirement above. <=0
        // disables this extra floor (a surplus pass alone then suffices).
        double low_count_min_ratio = 1.5;
        double max_tdcp_gap_s = 2.0;
        double base_epoch_match_tolerance_s = 0.02;
        double base_interpolation_max_gap_s = 1.2;
        bool reject_tdcp_loss_of_lock = true;
        bool reject_rover_carrier_loss_of_lock = false;
        bool reject_tdcp_code_phase_jump = true;
        double tdcp_code_phase_jump_threshold_m = 10.0;

        bool use_ionosphere_model = true;
        bool use_troposphere_model = true;
        bool use_multi_constellation = true;
        bool collect_lambda_debug = false;

        // --- Phase 2 milestone 2a (docs/gtsam_backend_design.md) ---
        // When true AND backend == FGOBackend::GTSAM, the GTSAM backend keys
        // the rover state as a body gtsam::Pose3 (translation + attitude) per
        // epoch instead of a bare gtsam::Point3, and builds the DD factors
        // with the lever-arm '...FactorArm' variants (antenna_ecef =
        // pose.translation() + pose.rotation() * pose3_lever_arm_body_m).
        // Attitude is unobservable from GNSS DD alone (no IMU until
        // milestone 2b) and is pinned near its identity seed by a dedicated
        // rotation-only prior; only the recovered ANTENNA position is a
        // validated output of this mode. Ignored by the native Eigen backend
        // and by the GTSAM backend when false (existing Point3 path,
        // unchanged). Default OFF.
        bool use_pose3_state = false;
        // Body-frame (FLU) translation from the body/IMU origin to the GNSS
        // antenna, used only when use_pose3_state is true. Zero means the
        // Pose3 state's translation IS the antenna (degenerate lever arm).
        Vector3d pose3_lever_arm_body_m = Vector3d::Zero();

        // --- Phase 2 milestone 2b: IMU tight coupling ---
        // When true AND use_pose3_state AND the GTSAM backend AND the problem
        // carries a valid ImuInput, the GTSAM backend augments each epoch with
        // a Vector3 velocity node V(i) and an imuBias::ConstantBias node B(i),
        // links consecutive epochs with a gtsam::CombinedImuFactor built from
        // PreintegratedCombinedMeasurements, and interprets the per-epoch Pose3
        // as body-in-nav (local ENU) with the DD '...FactorArm' consuming it
        // through ecef_T_nav (the same Pose3 is shared with the IMU factor).
        // Attitude/velocity become observable, so the per-epoch 2a rotation pin
        // is dropped in favour of first-state priors (attitude/velocity/bias)
        // for gauge. Ignored unless use_pose3_state is also set. Default OFF.
        bool use_imu = false;

        // --- Reference parity: IMU preintegration covariance value semantics
        // + residual-driven per-epoch inflation (port of the inuex35
        // reference's buildfactor/imu_preintegration.py's
        // _apply_mres_integ_cov_override + config.py's imu_integ_cov /
        // imu_integ_cov_max). Fixed-lag GTSAM path only
        // (optimizeProblemFixedLag in fgo_gtsam_backend.cpp); the batch path
        // (optimizeProblemWithGtsam) is untouched. ---
        //
        // The reference passes imu_integ_cov (dataclass default 1e-3)
        // DIRECTLY to gtsam's setIntegrationCovariance (utils/imu.py:44,
        // `p.setIntegrationCovariance(integ_cov * np.eye(3))`) -- i.e. it IS
        // the covariance, not a sigma to be squared. This backend's
        // ImuNoiseParams::integration_sigma is instead squared before that
        // same GTSAM call (`sq(problem.imu.noise.integration_sigma)`); this
        // field replaces that computation for the fixed-lag path so the
        // value semantics match exactly (no squaring). Default 1e-6
        // reproduces the harness's current hardcoded covariance
        // (integration_sigma=1e-3, squared) bit-for-bit, so leaving this
        // field untouched changes nothing.
        double imu_integration_covariance = 1e-6;

        // Per-epoch inflation (reference _apply_mres_integ_cov_override,
        // imu_preintegration.py:53-72). Before building EACH epoch's PIM the
        // reference recomputes:
        //   is_stale = stale_max > 0 && (epoch - last_mres_epoch) > stale_max
        //   integ_eff = is_stale ? imu_integ_cov
        //                        : clamp(max(imu_integ_cov, last_mres^2 / dt),
        //                                upper = imu_integ_cov_max)
        // and mutates the ONE shared PreintegrationCombinedParams instance in
        // place (tc.imu_params) before gtsam::PreintegratedCombinedMeasurements
        // is constructed for the interval -- this backend mirrors that
        // exactly: imu_params (this function's local shared_ptr, built once)
        // is mutated in place immediately before each epoch's PIM object is
        // constructed, so it affects only the PIM about to be integrated,
        // never factors already linearized into the graph.
        //
        //   last_mres  = the PREVIOUS epoch's post-fit main DDPR RMS residual
        //                [m] -- reference tc._last_main_ddpr_res /
        //                _mres_signals.last_res, written once per epoch in
        //                optimize/stage.py's _compute_postfit_diagnostics
        //                immediately after main_ddpr_residuals() runs (i.e.
        //                AFTER that epoch's ISAM2 update, using the smoothed
        //                estimate, pre-FDE). This backend's equivalent is
        //                the existing `last_ddpr_rms` local (already
        //                computed under use_cp_hold_recovery /
        //                use_epoch_quality_gates / use_sat_badness_downweight
        //                for the CP-hold FSM) -- no new residual computation
        //                needed, just a new consumer of the same value.
        //   dt         = real elapsed seconds since the previous epoch
        //                (reference tc._epoch_dt) -- this backend's
        //                (epoch[i].time - epoch[i-1].time), floored the same
        //                way (max(dt, 1e-3)).
        //   stale_max  = imu_integration_covariance_stale_epochs (reference
        //                reuses its unrelated-by-name
        //                ddcp_res_weight_stale_max_epochs config field for
        //                this same staleness test; ported here as its own
        //                dedicated field since the two features are
        //                independent in this backend). A long IMU-only
        //                outage (no recent DDPR solve) must not carry
        //                forward a possibly ancient inflation value, so the
        //                override falls back to the static floor instead.
        //
        // Requires imu_integration_covariance to be set as the static floor.
        // Master switch, default OFF (bit-identical baseline without it).
        bool use_imu_integration_covariance_inflation = false;
        double imu_integration_covariance_max = 0.5;       ///< reference imu_integ_cov_max
        int imu_integration_covariance_stale_epochs = 2;   ///< reference ddcp_res_weight_stale_max_epochs

        // --- Phase 2 milestone 2c: incremental fixed-lag smoother ---
        // When true AND use_imu (implies use_pose3_state, GTSAM backend), the
        // GTSAM backend streams the graph through a
        // gtsam::IncrementalFixedLagSmoother instead of a single batch LM
        // solve: each epoch's Pose3/Vel/Bias + factors are added incrementally
        // with key timestamps, and states older than fixed_lag_smoother_lag_s
        // (relative to the newest epoch) are marginalized. This bounds memory
        // and makes per-epoch LAMBDA feasible at full-dataset scale (the batch
        // gtsam::Marginals hit bad_alloc at ~25k vars). Ignored unless use_imu.
        // Default OFF (batch LM path unchanged).
        bool use_fixed_lag_smoother = false;
        // Smoother lag in seconds. States (and ambiguity/clock nodes) whose
        // last timestamp is older than this window are marginalized out.
        double fixed_lag_smoother_lag_s = 5.0;
        /// QR tolerates rank-deficient urban marginalization better than
        /// Cholesky at higher computational cost.
        bool fixed_lag_use_qr_factorization = false;

        // --- Phase 2 milestone 2d: NHC + ZUPT pseudo-measurements ---
        // Applied per-epoch in the IMU-coupled fixed-lag path (gated), mirror
        // inuex35 buildfactor/nhc.py + zupt.py. Default OFF.
        //
        // NHC: a ground vehicle's body-frame lateral (left) and vertical (up)
        // velocity is ~0. Binary factor on (Pose3(i), Vel(i)) with residual
        // [v_body.y, v_body.z] = [ (R^T v_nav).y, (R^T v_nav).z ]. Gated to
        // moving, non-turning epochs so it never fights legitimate lateral
        // motion.
        bool use_nhc = false;
        double nhc_min_speed_mps = 2.0;          ///< only apply NHC above this speed
        double nhc_max_yaw_rate_radps = 0.20;    ///< skip NHC when |yaw rate| exceeds this (turn)
        double nhc_sigma_lateral_mps = 0.3;      ///< body-lateral velocity sigma
        double nhc_sigma_vertical_mps = 0.2;     ///< body-vertical velocity sigma
        // ZUPT: when the epoch's IMU window is stationary, pin Vel(i) ~ 0 with a
        // PriorFactor. Stationary detection mirrors the Stage-1 ESKF detector
        // (accel_std / gyro_std / gyro_median of bias-referenced IMU samples).
        bool use_zupt = false;
        double zupt_sigma_mps = 0.05;            ///< zero-velocity prior sigma
        // Velocity gate (mirrors inuex35): only fire ZUPT when the current
        // (IMU-predicted) speed is already below this. Without it, a quiet
        // accelerometer during CONSTANT-VELOCITY cruising is misread as
        // stationary and ZUPT freezes the vehicle (observed: FLOAT max 94 m).
        // 0 disables the gate.
        double zupt_max_speed_mps = 0.5;
        double zupt_max_accel_std = 0.55;        ///< stationary gate: accel deviation std [m/s^2]
        double zupt_max_gyro_std = 0.030;        ///< stationary gate: gyro deviation std [rad/s]
        double zupt_max_gyro_median = 0.020;     ///< stationary gate: gyro deviation median [rad/s]
        int zupt_min_samples = 5;                ///< minimum IMU samples in window to test

        // --- Phase 2 milestone 2e: fix-and-hold ambiguity resolution ---
        // In the fixed-lag path, once an arc's DD ambiguity is validated-fixed
        // (LAMBDA ratio above ambiguity_hold_ratio_threshold), pin it in the
        // graph with a tight prior at the integer for the remainder of the arc.
        // Held ambiguities have ~zero variance, so subsequent per-epoch LAMBDA
        // over the remaining floats passes far more often (fix-rate lever) and
        // the held integers stabilize the float. Reset is automatic: a cycle
        // slip / gap makes the problem builder assign a NEW ambiguity index
        // (fgo.cpp segments arcs on loss_of_lock), which is not in the held set.
        // Gated; default OFF.
        bool use_ambiguity_hold = false;
        // When false, hold priors still stabilize the graph but an epoch is
        // never labelled FIXED solely because enough arcs were pinned by an
        // earlier epoch; a fresh ambiguity-resolution pass is required.
        // Default true preserves the historical fix-and-hold output policy.
        bool report_held_ambiguities_as_fixed = true;
        double ambiguity_hold_ratio_threshold = 3.0;  ///< min ratio to hold (stricter than fix)
        double ambiguity_hold_sigma_cycles = 1e-3;    ///< tight prior sigma at the held integer
        int ambiguity_hold_min_fixed = 4;             ///< min ambiguities in a passing epoch to hold

        // --- MF-AR step 2: partial AR in the fixed-lag per-epoch LAMBDA ---
        // Port of the native Eigen path's use_partial_lambda_ambiguity_fix
        // semantics into the GTSAM fixed-lag smoother's per-epoch LAMBDA:
        // rank the epoch's ambiguity candidates best-first by (fractional
        // cycles from the nearest integer, then float variance), cap the set
        // at max_lambda_ambiguities, and when the ratio test fails retry on
        // shrinking best-ranked prefixes down to min_fixed_ambiguities. The
        // validated subset is marked FIXED (and pinned by fix-and-hold as
        // usual); dropped candidates simply stay float. This is the
        // all-or-nothing -> partial upgrade: with multipath-corrupted arcs in
        // the window (urban), P(the ENTIRE candidate set validates) collapses
        // as the set grows, which is what capped multi-frequency fix-rates.
        // Separate knob from use_partial_lambda_ambiguity_fix (which is
        // default-ON and consumed by the Eigen batch path) so the existing
        // fixed-lag behaviour stays bit-identical unless opted in.
        bool use_fixed_lag_partial_lambda = false;
        // Partial-AR subset floor as a fraction of the (capped) candidate
        // set. Classic partial AR drops a FEW outliers, never the majority:
        // in deep urban the ratio test over an arbitrarily shrunken subset of
        // multipath-corrupted candidates passes by chance and labels wrong
        // integers FIXED (measured on tokyo1 full-run: FIXED rms 5-11 m when
        // subsets may shrink to min_fixed_ambiguities). The subset never
        // shrinks below ceil(min_fraction * candidates) (and never below
        // min_fixed_ambiguities). 1.0 degenerates to all-or-nothing.
        double fixed_lag_partial_lambda_min_fraction = 0.7;
        // Exclude Galileo arcs from the LAMBDA candidate set / fix-and-hold
        // (their DD factors still constrain the float). On tokyo1 the truth-
        // trajectory DD residuals show ~50% of Galileo arcs (E1 AND E5a) are
        // multipath-corrupted (vs ~75-90% integer-consistent for GPS/BDS/QZS),
        // so Galileo candidates mostly poison the all-or-nothing ratio test.
        bool exclude_galileo_ambiguity_fixing = false;

        // --- Code-Minus-Carrier (CMC) multipath screening ---
        // Port of the inuex35 reference's preprocess/slip_detect.py CMC logic
        // (single-difference CMC = (PR_rover - PR_base) - (CP_rover -
        // CP_base) * wavelength, per (satellite, signal), computed only when
        // the DD builder already has both rover and base pseudorange+carrier
        // for that pair this epoch). Two independent checks:
        //  1. Jump check (this port's primary target): |cmc - previous_epoch
        //     cmc| > code_minus_carrier_jump_threshold_m forces the SAME arc
        //     break the existing loss-of-lock / gap logic already performs
        //     (new ambiguity_index at the next DD carrier factor for that
        //     (satellite, signal)) -- a multipath jump breaks the integer N
        //     just like a cycle slip does.
        //  2. Sustained-level check (reference default OFF via level_thresh
        //     == 0.0, ported for parity but not expected to be the lever):
        //     maintains a per-(satellite, signal) baseline (seeded by the
        //     first observation, running-averaged for
        //     code_minus_carrier_warmup_epochs, then EWMA-updated with
        //     code_minus_carrier_baseline_alpha in steady state); a
        //     steady-state baseline deviation beyond
        //     code_minus_carrier_level_threshold_m excludes THAT (satellite,
        //     signal) pair's DD pseudorange AND carrier factors for the
        //     epoch (both, matching the reference's single `continue` before
        //     either is built) without updating the baseline.
        //
        // Deliberate deviation from the reference: whenever the arc for a
        // (satellite, signal) resets for ANY reason (CMC jump, cycle slip /
        // loss-of-lock, or outage -- i.e. the rover-side single-receiver
        // carrier arc restarts and a fresh ambiguity_index is assigned), this
        // port ALSO resets that (satellite, signal)'s CMC baseline/warmup
        // count/previous-epoch value. CMC embeds a -wavelength*N term, so a
        // new ambiguity invalidates any baseline computed under the old one;
        // the reference does not reset it (state.cmc / cmc_baseline survive
        // amb_key resets in slip_detect.py), which looks like an oversight --
        // it is harmless there only because the reference ships the level
        // check OFF by default.
        //
        // Default OFF: bit-identical to the pre-port baseline when false.
        bool use_code_minus_carrier_screening = false;
        double code_minus_carrier_jump_threshold_m = 3.0;
        double code_minus_carrier_level_threshold_m = 0.0;  ///< 0 = level check off (reference default)
        /// On sustained CMC deviation, suppress the contaminated DD code
        /// factor but retain DD carrier continuity for robust-loss/FDE AR
        /// quarantine.  Default false preserves reference parity.
        bool code_minus_carrier_level_pseudorange_only = false;
        int code_minus_carrier_warmup_epochs = 5;
        double code_minus_carrier_baseline_alpha = 0.05;
        // DD reference-satellite selection normally picks the highest-
        // elevation satellite in each (system, signal) group with no regard
        // for the CMC sustained-level exclusion computed just above
        // (cmc_level_exclude_this_epoch). Because every DD pair in a group
        // is formed against that ONE reference, a high-elevation but
        // multipath/NLOS-biased satellite becoming the reference poisons
        // every DD residual in the group simultaneously (diagnosed on tokyo
        // run1: ddpr_rms toggling 2m<->40-58m across all satellites while
        // geometry stayed fine, 42% of the run's float squared-error from
        // one such episode). When true (and use_code_minus_carrier_screening
        // is also on), the highest-elevation NON-CMC-excluded candidate is
        // preferred as reference; if every candidate in the group is
        // CMC-excluded this epoch, falls back to the original max-elevation
        // choice so the group is never dropped. Default OFF: bit-identical
        // to the pre-change baseline when false.
        bool cmc_aware_reference_selection = false;

        // --- Per-epoch quality gates (port of the inuex35 reference's
        // preprocess/gate.py + validation/postfit.py policy) ---
        // The reference never lets the integer search run on a corrupt epoch:
        // a GDOP/nsat gate skips weak-geometry epochs entirely, per-satellite
        // post-fit DD-pseudorange residuals drop multipath satellites from
        // the NEXT epoch's LAMBDA tree, and a main-DDPR-RMS sanity threshold
        // suppresses carrier fixing while the solution is inconsistent.
        // This port applies the same three screens to the fixed-lag path as
        // FIXING gates: DD factors still enter the graph (robust loss handles
        // them), but a gated epoch attempts no LAMBDA, adds no holds, and is
        // never labelled FIXED via held integers -- killing the meaningless
        // deep-urban FIXED labels measured on tokyo1 full-run (FIXED rms 6-11
        // m). Thresholds default to the reference's config.py values.
        // Master switch, default OFF (bit-identical baseline without it).
        bool use_epoch_quality_gates = false;
        double gate_gdop_max = 10.0;          ///< reference gdop_max
        int gate_min_satellites = 6;          ///< reference nsat_min
        double gate_ddpr_res_max_m = 3.0;     ///< reference main_ddpr_res_thresh
        double gate_per_sat_res_max_m = 3.0;  ///< reference per_sat_res_thresh

        // --- CP-hold / sanity FSM (port of the inuex35 reference's
        // validation/postfit.py + validation/recovery.py + state.py +
        // preprocess/gate.py "wrong integer basin" recovery policy) ---
        //
        // The quality gates above (use_epoch_quality_gates) stop the pipeline
        // from FIXING on a corrupt epoch, but do nothing when the graph has
        // already locked onto a WRONG integer basin (a bad LAMBDA fix earlier
        // got pinned by fix-and-hold and now poisons every subsequent epoch's
        // float, since the held priors keep dragging the solution back to the
        // wrong integers). This FSM detects that condition from post-fit DD
        // pseudorange residuals (which see the wrong-basin float pose, not the
        // held carrier terms) and recovers by mass-invalidating every current
        // ambiguity: bump each arc's "generation" so the next observation gets
        // a FRESH graph symbol (new prior, no held integer -- see the
        // per-ambiguity generation-overlay comment beside ambSymbolId() in
        // fgo_gtsam_backend.cpp), and suspend carrier fixing (CP-hold) for a
        // configured number of epochs so the float can re-converge on
        // pseudorange alone before AR is attempted again.
        //
        // NOTE: this port originally skipped the DDPR-LS anchor stages
        // (_ddpr_sanity_fetch_anchor / _anchor_vs_imu in postfit.py). They are
        // now ported below (FGOConfig::use_ddpr_anchor) -- see that switch's
        // comment for why the anchor turns out to be diagnostic-only in
        // THIS particular slot (the reference's own control flow converges
        // on the same reset regardless of anchor trust) and for where the
        // anchor position actually changes behaviour (exception recovery +
        // bootstrap re-seed).
        //
        // Master switch, default OFF (bit-identical baseline without it).
        bool use_cp_hold_recovery = false;
        // FGO-specific recovery: while integer fixing is held, keep carrier
        // factors alive at reduced weight so fresh float ambiguities can
        // converge.  The legacy mode removes carrier and bumps its generation
        // every held epoch, which can deadlock reacquisition indefinitely.
        bool use_cp_hold_float_recovery = false;
        double cp_hold_float_recovery_sigma_scale = 10.0;
        // Release a carrier hold early only when an independent DDPR-LS/FDE
        // position is trusted, and anchor the reacquisition epoch to it.
        bool use_cp_hold_anchor_release = false;
        double cp_hold_main_residual_threshold_m = 3.0;    ///< reference main_ddpr_res_thresh
        double cp_hold_catastrophic_threshold_m = 15.0;    ///< reference main_ddpr_res_catastrophic
        double cp_hold_fast_worst_satellite_min_m = 10.0;  ///< reference ddpr_fast_worst_sat_min (tokyo profile)
        int cp_hold_persist_epochs = 3;                    ///< reference ddpr_sanity_persist
        // --- Leaky persist accumulator (DEVIATION from the reference; this
        // port's own addition -- see the .cpp change-site beside
        // ddpr_bad_count for the exact mechanics). ---
        //
        // The reference hard-resets _ddpr_bad_count to 0 on the FIRST clean
        // epoch (ddpr_rms <= cp_hold_main_residual_threshold_m); this port
        // matches that faithfully whenever use_cp_hold_leaky_persist is
        // false (bit-identical baseline). Measured failure mode on tokyo
        // run3: an INTERMITTENTLY-bad wrong-integer-basin stretch (one bad
        // epoch, one borderline-clean epoch, repeat) never accumulates
        // cp_hold_persist_epochs CONSECUTIVE bad epochs -- every other
        // epoch resets the counter to 0 -- so the mass reset that would
        // flush the bad basin never fires, even though the run's own
        // trigger rate at that stretch is high. When true, a clean epoch
        // instead LEAKS the counter down by cp_hold_persist_decay
        // (ddpr_bad_count = max(0, ddpr_bad_count - cp_hold_persist_decay))
        // rather than zeroing it, so bad epochs interleaved with
        // borderline-clean ones can still accumulate net "credit" toward
        // the persist threshold. cp_hold_persist_decay >=
        // cp_hold_persist_epochs reproduces the hard reset in a single
        // clean epoch (matching the reference exactly at that setting);
        // smaller decay (0.25 / 0.5 / 1.0 swept) lets the counter survive
        // multiple clean epochs. The mass/fast reset firing itself still
        // zeroes ddpr_bad_count outright (resetAmbiguitiesWithCpHold();
        // unrelated to this clean-epoch decay and unaffected by this
        // knob) -- only the "epoch judged clean, no reset fired" path
        // changes. ddpr_bad_count is a double specifically to support
        // fractional decay values without precision loss for the
        // integer-valued default path. Requires use_cp_hold_recovery
        // (there is no persist counter otherwise). Default OFF.
        bool use_cp_hold_leaky_persist = false;
        double cp_hold_persist_decay = 1.0;
        int cp_hold_epochs = 5;                            ///< reference recov_cp_hold
        // Keep clean carrier arcs available during an urban multipath hold;
        // only pairs implicated by the previous epoch's per-satellite DD
        // pseudorange residuals are suppressed.
        bool use_selective_cp_hold = false;
        double selective_cp_hold_sigma_scale = 100.0;
        double cp_hold_release_threshold_m = 2.0;          ///< reference recov_cp_release_thresh (tokyo profile)
        int cp_hold_release_count = 5;                     ///< reference recov_cp_release_count (tokyo profile)
        double cp_hold_pose_replace_threshold_m = 5.0;     ///< reference sanity_pose_replace_thresh
        double cp_hold_multipath_median_ratio = 5.0;       ///< reference sanity_max_median_ratio
        int cp_hold_multipath_min_satellites = 6;          ///< reference sanity_max_median_min_sats
        double cp_hold_max_gdop = 5.0;                     ///< reference sanity_max_gdop (tokyo profile)
        // Break the IMU preintegration chain at the epoch after a mass/fast
        // reset (reference sanity_break_pim, default on): the next epoch adds
        // a loose PriorPose3/PriorVector seeded at the IMU prediction instead
        // of a CombinedImuFactor linking back to the pre-reset state, so a
        // wrong pre-reset pose cannot drag the post-reset solution back.
        bool cp_hold_break_imu_chain = true;
        // Translation sigma [m] of that loose post-reset pose prior (reference
        // pim_break_trans_sigma; the tokyo profile widens this from the 1.0 m
        // dataclass default to 100.0 m -- i.e. barely constrain translation at
        // all after a wrong-basin reset).
        double cp_hold_imu_break_translation_sigma_m = 100.0;

        // --- Stale-pin invalidation (per-arc hold release at the FSM trigger)
        // ---
        // The FSM above only mass-resets after cp_hold_persist_epochs
        // CONSECUTIVE bad epochs (or on the catastrophic fast path). Measured
        // failure mode on tokyo run2: a wrong / still-converging integer got
        // pinned by fix-and-hold and produced a ~3 m offset for ~270 s -- the
        // post-fit DDPR RMS hovers around the trigger threshold, the persist
        // counter keeps resetting, and the stale pin rides for minutes with
        // no escalation. This mechanism acts at the trigger itself: on every
        // trigger epoch (post-fit DDPR RMS above
        // cp_hold_main_residual_threshold_m) that did NOT already mass-reset
        // -- INCLUDING multipath-dominated epochs, where the FSM skips the
        // mass reset precisely because one bad satellite does not justify
        // resetting everything, but releasing THAT satellite's pin per-arc
        // is the right-sized response --
        // any arc currently PINNED by fix-and-hold whose own satellite's
        // per-satellite post-fit DD pseudorange residual exceeds
        // stale_pin_per_sat_residual_m has its live factors (hold prior
        // included) removed via the same factor-removal path the mass reset
        // uses -- but per-arc -- and its generation bumped so the next
        // observation re-enters as a fresh float arc. No CP-hold engages
        // beyond what the trigger already did, no IMU-chain break, and every
        // other arc's pin survives: the consistent pins keep carrying the
        // solution while only the offenders re-float. Requires
        // use_cp_hold_recovery (trigger + generation overlay) and
        // use_ambiguity_hold (there are no pins otherwise).
        // Default OFF (bit-identical baseline when false).
        bool use_stale_pin_invalidation = false;
        // Per-satellite post-fit DDPR residual [m] above which that
        // satellite's pinned arc is released at a trigger epoch.
        double stale_pin_per_sat_residual_m = 2.0;
        // Only release pins at least this many epochs old (0 = any age).
        // Guards against churning a pin created this very epoch before it
        // ever had a chance to prove itself.
        int stale_pin_min_hold_age_epochs = 0;

        // --- Fix plausibility demotion (label-level IMU gap check) ---
        // Measured failure mode on tokyo run1: sparse, isolated single-epoch
        // FIXED outliers (worst: 45.8 m) dominate the FIXED-only RMS, but the
        // legitimate fixes routinely show 5-20 m post-fit DDPR residuals so
        // no residual threshold separates them. The IMU prediction does:
        // after the epoch's AR / fix-and-hold labeling, if the epoch is
        // labelled FIXED but its fixed antenna position is farther than
        // fix_demote_distance_m from the IMU-predicted position for this
        // epoch (pose_seed -- the same prediction the FSM's pose-replacement
        // stage compares against), demote the LABEL to FLOAT. Mirrors the
        // pose-replacement logic (cp_hold_pose_replace_threshold_m, 5 m) but
        // acts on the label/hold only: the graph, the reported position and
        // pre-existing pins are untouched. Additionally, an epoch failing
        // the same check never PINS its freshly-validated integers
        // (fix-and-hold skip), so an implausible fix cannot poison the rest
        // of the arc. Independent of use_cp_hold_recovery (the IMU seed
        // always exists on the fixed-lag IMU path).
        // Default OFF (bit-identical baseline when false).
        bool use_fix_plausibility_demotion = false;
        // Max distance [m] between the fixed position and the IMU-predicted
        // position for the FIXED label to stand.
        double fix_demote_distance_m = 5.0;
        // Anchor-referenced variant of the same demotion (requires
        // use_ddpr_anchor for the anchor plumbing). The IMU prediction is
        // dead-reckoned from the PREVIOUS solved epoch, so once the window
        // has already converged into a wrong integer basin the prediction
        // rides along with it and the IMU-gap check above goes blind
        // (measured on tokyo run2: a ~190-epoch wrong-basin FIXED stretch at
        // ~3.4 m with IMU gap ~0 -- pin release and IMU-gap demotion both
        // left it intact). The per-epoch DDPR-LS anchor is the one reference
        // that does NOT ride the basin: it is re-solved from THIS epoch's DD
        // pseudoranges alone, which stay unbiased when the carrier-driven
        // pose is wrong. When a TRUSTED anchor (>= ddpr_anchor_min_factors
        // active rows, res_rms <= ddpr_anchor_max_residual_m -- the same
        // trust test the FSM's anchor stage uses) disagrees with a FIXED
        // epoch's position by more than fix_demote_anchor_distance_m, the
        // FIXED label is demoted exactly like the IMU-gap case. On epochs
        // where the fix is legitimate but pseudoranges are multipath-heavy
        // (tokyo run1's 5-20 m post-fit DDPR residuals on true fixes), the
        // anchor fails its own residual trust gate and never demotes.
        // Default OFF.
        bool fix_demote_use_ddpr_anchor = false;
        double fix_demote_anchor_distance_m = 3.0;
        // Anchor trust ceiling for the DEMOTION check only [m]. 0 = reuse
        // ddpr_anchor_max_residual_m (2.0). The FSM's 2.0 m trust gate is
        // tuned for authorizing mass resets; for a label-only veto a looser
        // ceiling is acceptable (worst case: a FIXED label is demoted to
        // FLOAT -- no graph change), and on tokyo run2 part of the
        // wrong-basin stretch only shows anchors in the 2-4 m residual band.
        double fix_demote_anchor_trust_res_m = 0.0;
        // --- Gross-offender gate on the anchor-gap variant (DEVIATION; this
        // port's own addition -- see the .cpp change-site for exact
        // mechanics). ---
        //
        // Evaluated unconditionally, the anchor-gap variant above measured
        // catastrophic on tokyo run1 (1662 false demotions of legitimate
        // fixes, fix-rate 50.0% -> 35.4%): run1's failure mode is DIFFUSE
        // multipath (several satellites moderately bad at once), which also
        // drags the anchor's own (non-robust) LS solve by ~3 m, so even a
        // "trusted" (>= min_factors rows, res_rms under the trust ceiling)
        // anchor disagrees with a perfectly good fix by more than
        // fix_demote_anchor_distance_m. Tokyo run3's actual wrong-basin
        // bands (tow ~180804, ~181065) instead show a GROSS single-
        // satellite offender signature per epoch. When true, the anchor-gap
        // check (anchor solve, trust test, gap compare) is only EVALUATED on
        // epochs whose pre-FDE per-satellite post-fit DD residual signature
        // is gross: max(per_sat_res) > fix_demote_anchor_gross_abs_m AND
        // max(per_sat_res) > fix_demote_anchor_gross_ratio *
        // median(per_sat_res). Epochs without that signature never reach the
        // anchor solve for this variant at all (fail-safe: no solve => no
        // demotion via this path; the IMU-gap/absolute/relative-residual
        // variants above are untouched by this gate). No-op unless
        // fix_demote_use_ddpr_anchor is also true. Default OFF (bit-
        // identical to the ungated anchor-gap variant when false).
        bool fix_demote_anchor_gross = false;
        double fix_demote_anchor_gross_ratio = 10.0;
        double fix_demote_anchor_gross_abs_m = 20.0;
        // Extreme-residual demotion [m]: demote the FIXED label when THIS
        // epoch's post-fit DD pseudorange RMS exceeds this. 0 = off. Unlike
        // use_epoch_quality_gates' gate_ddpr_res_max_m (which suppresses the
        // whole fixing pipeline -- LAMBDA, holds -- and thereby starves the
        // fix rate on runs whose legitimate fixes ride 5-20 m multipath
        // residuals), this only strips the LABEL of the extreme epoch
        // itself. Measured separation on the PPC tokyo set: legitimate
        // deep-urban fixes show post-fit DDPR RMS up to ~20 m (run1), while
        // run2's wrong-basin FIXED stretch rides at 32-60 m -- a threshold
        // in the 25-30 m band separates them cleanly where no absolute
        // FIXING gate could. Needs use_cp_hold_recovery or
        // use_epoch_quality_gates (or any feature that computes the shared
        // post-fit residual pass); without those the residual is 0 and this
        // check never fires.
        double fix_demote_res_m = 0.0;
        // Post-CP-hold cooldown [epochs]: demote any FIXED label produced
        // within this many epochs of the last carrier-suppressed (CP-hold)
        // epoch. 0 = off; needs use_cp_hold_recovery (no holds otherwise).
        // Rationale (measured, tokyo run1 tow 188395.4 -- the single epoch
        // carrying ~40% of run1's FIXED sum-of-squares at 45.8 m error): a
        // fix validated moments after a hold releases is built on arcs the
        // hold's generation-regeneration just re-created, i.e. sub-second
        // float history in exactly the conditions (recovering from a
        // wrong-basin / multipath episode) where LAMBDA's ratio test is
        // least trustworthy. That epoch passed EVERY local plausibility
        // witness (post-fit DDPR RMS 5.7 m, IMU gap 0.73 m, trusted DDPR-LS
        // anchor gap 0.78 m) because the whole local window had excursed
        // together -- only its position in time relative to the hold gives
        // it away. Label-only, like the other demotion criteria.
        int fix_demote_posthold_epochs = 0;
        // RELATIVE residual demotion: demote the FIXED label when THIS
        // epoch's post-fit DDPR RMS exceeds fix_demote_res_rel times the
        // rolling MEDIAN of the last fix_demote_res_rel_window epochs' RMS
        // (and also exceeds cp_hold_main_residual_threshold_m as an absolute
        // floor, so a quiet run's noise never triggers it). 0 = off. This is
        // the run-adaptive complement to the absolute fix_demote_res_m:
        // tokyo run1's legitimate fixes ride 5-20 m residuals CHRONICALLY
        // (median high -> relative test stays quiet), while run3's bad fixed
        // stretches sit at 4.7-13.3 m over a 0.7-1.8 m ambient (a 4-19x
        // excursion) and run2's wrong-basin band at 32-60 m over a ~1-3 m
        // ambient -- exactly the "suddenly much worse than this run's own
        // recent normal" signature. Median (not mean) so the excursion
        // epochs themselves do not drag the baseline up. Needs the shared
        // post-fit residual pass (see fix_demote_res_m).
        double fix_demote_res_rel = 0.0;
        int fix_demote_res_rel_window = 100;  ///< rolling window [epochs]; >=20 history required
        // Surplus-satellite cross-check on demotion (requires
        // use_surplus_satellite_validation). Measured on tokyo run1 (2026-07):
        // of the epochs the res/dist/posthold/res_rel criteria above would
        // demote, the great majority (367/406, <0.5 m; mean 0.176 m) are
        // FALSE ALARMS -- legitimate fixes caught by a residual threshold
        // tuned to also catch run2's genuine wrong-basin cluster. The
        // surplus-satellite test (independent re-differencing against
        // excluded/surplus satellites, see use_surplus_satellite_validation)
        // already renders a verdict on 353/406 of these same epochs from
        // this epoch's own fresh LAMBDA attempt, and it discriminates
        // cleanly within that population: 299 PASS (mean error 0.117 m) vs
        // 54 FAIL (mean error 0.438 m). When true, a demotion is REPRIEVED
        // (the FIXED label stands) if and only if this epoch's surplus
        // verdict was evaluated AND passed; any other case (no verdict,
        // failed verdict) demotes exactly as before -- fail-safe toward
        // demoting when the independent check cannot vouch for the fix.
        // Default OFF (bit-identical baseline without it; no effect unless
        // both use_fix_plausibility_demotion and
        // use_surplus_satellite_validation are also on).
        bool fix_demote_surplus_crosscheck = false;
        // Fail-closed reprieve preset: a surplus PASS may cancel a demotion
        // only when an independently solved DDPR anchor also agrees with the
        // fixed candidate and the frozen observation-quality floors pass.
        bool fix_demote_surplus_anchor_reprieve = false;
        int fix_demote_surplus_anchor_min_satellites = 12;
        double fix_demote_surplus_anchor_max_float_separation_m = 1.0;
        double fix_demote_surplus_anchor_max_postfit_ddcp_rms_m = 0.1;
        double fix_demote_surplus_anchor_max_gap_m = 8.0;
        // Residual-only demotion reprieve backed by two witnesses that do
        // not use the DD pseudorange residual which triggered the demotion:
        // a strong LAMBDA/IMU model and this epoch's fresh standalone SPP
        // solution. The reprieve is fail-closed and label-only. It never
        // applies when distance, post-hold, relative-residual, or DDPR-anchor
        // demotion also fires, when the LAMBDA candidate is stale/missing,
        // or when the builder had to coast an older SPP seed. Defaults are
        // frozen from Tokyo run1, validated unchanged on run2, then audited
        // once on held-out run3 (521 correct / 0 wrong rescued epochs).
        bool fix_demote_spp_model_reprieve = false;
        int fix_demote_spp_model_min_fixed_ambiguities = 10;
        double fix_demote_spp_model_max_imu_separation_m = 0.02;
        double fix_demote_spp_model_max_agreement_m = 8.0;

        // --- Exception recovery (port of recovery.py's handle_solve_exception)
        // ---
        // Independent switch: when the smoother throws (numerical failure,
        // e.g. IndeterminantLinearSystemException from a rank-deficient
        // epoch) and use_ddpr_anchor is OFF, retry the epoch with ONLY loose
        // re-seed priors (pose sigma 1.0 m, vel sigma 1.0 m/s, bias sigma 0.1)
        // at the IMU-predicted state, discarding that epoch's GNSS/IMU
        // factors. When use_ddpr_anchor is ON, the reference's actual first
        // choice (recovery.handle_solve_exception: try_ddpr_reset before the
        // loose-prior fallback) is tried FIRST instead -- see
        // use_ddpr_anchor's comment. If THAT retry also
        // throws, perform a full warm reset: destroy and recreate the
        // IncrementalFixedLagSmoother from scratch (reference
        // warm_reset_phase2) with fresh priors at the last good (or
        // IMU-predicted) pose -- rotation sigma from
        // cp_hold_warm_reset_rotation_sigma_deg, position sigma [2,2,3] m
        // (ENU), velocity isotropic 3.0 m/s (seeded to zero), bias isotropic
        // 0.01 -- bump every tracked ambiguity's generation, and engage
        // CP-hold. This targets the known IndeterminantLinearSystemException
        // tail failure on the tokyo run2 dataset (~epoch 6390): the retry lets
        // a single bad epoch pass without giving up on the whole run, and the
        // warm reset recovers from a genuinely poisoned linearization point.
        // Default OFF (bit-identical baseline without it).
        bool use_solve_exception_recovery = false;
        double solve_exception_pose_sigma_m = 1.0;
        double solve_exception_velocity_sigma_mps = 1.0;
        double solve_exception_bias_sigma = 0.1;
        double cp_hold_warm_reset_rotation_sigma_deg = 1.0;  ///< reference body_rot_std

        // --- DDPR-LS anchor (port of the inuex35 reference's
        // utils/ls_solvers.py ddpr_only_position + the anchor stages of
        // validation/postfit.py/recovery.py/optimize/stage.py that the
        // CP-hold FSM port above deliberately skipped). ---
        //
        // The anchor is a standalone DDPR-only least-squares position solve
        // (single Pose3 key, THIS epoch's DD pseudorange factors only, no
        // carrier/IMU/ambiguity coupling) built by reusing the same
        // DoubleDifferencePseudorangeFactorArm construction the main graph
        // uses (see solveDdprAnchor() in fgo_gtsam_backend.cpp): a tight-
        // rotation/loose-translation PriorPose3 around the IMU-predicted
        // pose, plain (non-robust -- matching the reference's huber_pr=0
        // default) noise, LevenbergMarquardt (max 10 iterations), then up to
        // 3 rounds of single-pass FDE (drop factors with raw residual >
        // ddpr_anchor_fde_threshold_m while >= ddpr_anchor_min_factors
        // remain, re-solve).
        //
        // It is wired into THREE places:
        //  1. Diagnostics inside the CP-hold FSM's persist (mass-reset) path
        //     (postfit.py's _ddpr_sanity_fetch_anchor / _anchor_vs_imu).
        //     IMPORTANT / faithfully-ported reference quirk: in the
        //     reference, EVERY branch of run_ddpr_sanity past the persist
        //     gate -- anchor untrusted, anchor disagrees with the IMU
        //     prediction, or anchor agrees -- converges on the exact same
        //     _apply_sanity_reset() call with the exact same arguments (the
        //     anchor ECEF itself is never read by _apply_sanity_reset). So
        //     this stage does NOT gate whether the mass reset fires here --
        //     it only records whether the anchor WOULD have been trusted /
        //     agreed with the IMU (diagnostics: ddpr_anchor_gated_resets_
        //     skipped/allowed), exactly mirroring the reference's info-dict-
        //     only behaviour. Requires use_cp_hold_recovery.
        //  2. Exception recovery (recovery.py's handle_solve_exception /
        //     try_ddpr_reset): here the anchor's POSITION does matter. When
        //     the smoother throws, this is now tried FIRST -- warm-reset the
        //     smoother seeded at (anchor translation, IMU-predicted
        //     rotation, zero velocity) -- and only when the anchor is
        //     untrusted (solve failed, too few factors, or res_rms too high)
        //     does control fall through to the existing loose-prior retry /
        //     IMU-seeded full warm reset. Requires use_solve_exception_recovery.
        //  3. Bootstrap re-seed (optimize/stage.py's BOOT_DDPR_EPOCHS /
        //     tightly_coupled.py's Phase-2-init arming): our primary lever
        //     against the CP-hold FSM's known FLOAT-degradation cost. It is
        //     armed at Phase-2 initialization and after recovery resets; every
        //     epoch for ddpr_anchor_bootstrap_epochs epochs gets a
        //     translation-only PriorPose3 at that epoch's anchor position
        //     (rotation sigma ~unconstrained, translation sigma
        //     ddpr_anchor_bootstrap_sigma_m) added alongside the normal
        //     graph -- a pull-back channel to truth that does not depend on
        //     carrier/AR recovering first. The countdown decrements every
        //     armed epoch regardless of whether that epoch's anchor solve
        //     succeeds (matches the reference: stage.py decrements outside
        //     the try/except). While armed, effective CP-hold length is
        //     forced to 0 (bootstrap and CP-hold are mutually exclusive in
        //     the reference's state.effective_cp_hold_epochs -- bootstrap
        //     wins) via effectiveCpHoldEpochs() in the .cpp.
        //
        // The reference arms the bootstrap countdown at Phase-2
        // initialization. This port does the same at epoch 0, re-arms it
        // after EVERY full warm reset (both the anchor-seeded and IMU-seeded
        // paths, since both destroy and recreate the smoother from scratch),
        // and,
        // opt-in via cp_hold_bootstrap_after_mass_reset, after the CP-hold
        // FSM's mass/fast ambiguity resets too (those do NOT recreate the
        // smoother, but DO invalidate every held integer, which is the same
        // "float needs a pull-back channel" situation the bootstrap targets).
        // Shipped default for cp_hold_bootstrap_after_mass_reset: FALSE.
        // Measured on the tokyo full runs (FSM+anchor, boot-after-mass on,
        // reference-default sigma 0.5 m / 20 epochs): mass/fast resets fire
        // hundreds of times per run, concentrated in exactly the deepest-
        // multipath sections, so arming there injects sub-metre-sigma
        // anchor priors at the LEAST trustworthy DDPR-LS positions AND (per
        // the reference's bootstrap-suppresses-CP-hold rule) disables the
        // FSM's protective carrier suppression in those same sections --
        // run1 FLOAT RMS 26.9 -> 95.9 m and FIXED RMS 0.80 -> 17.7 m; run2
        // FLOAT 6.9 -> 26.4 m, FIXED 0.50 -> 4.05 m. The reference's arming
        // moment (Phase-2 init, right after a window of verified-good fixes
        // in workable sky) has no analogue at a mid-canyon mass reset. With
        // this false the bootstrap still arms after full warm resets (rare:
        // solver-exception recovery), where the smoother has genuinely lost
        // its history and the anchor is the only absolute-position channel.
        //
        // Master switch, default OFF (bit-identical baseline without it).
        bool use_ddpr_anchor = false;
        double ddpr_anchor_max_residual_m = 2.0;       ///< reference ddpr_max_res
        double ddpr_anchor_fde_threshold_m = 4.0;      ///< reference fde_pr
        int ddpr_anchor_min_factors = 4;               ///< reference ddpr_only_position's hard floor
        double ddpr_anchor_imu_max_gap_m = 20.0;       ///< reference anchor_imu_max_gap
        double ddpr_anchor_imu_hard_max_m = 200.0;     ///< reference anchor_imu_hard_max
        double ddpr_anchor_clean_residual_m = 1.0;     ///< reference anchor_imu_clean_res / ddpr_clean_res
        double ddpr_anchor_clean_main_residual_m = 15.0;  ///< reference anchor_imu_clean_main_res
        int ddpr_anchor_persist_override = 6;          ///< reference ddpr_bad_persist_override
        int ddpr_anchor_bootstrap_epochs = 20;         ///< reference BOOT_DDPR_EPOCHS
        double ddpr_anchor_bootstrap_sigma_m = 0.5;    ///< reference BOOT_DDPR_SIGMA
        bool cp_hold_bootstrap_after_mass_reset = false;  ///< see deviation + validation note above

        // --- FDE: GICI-style Fault Detection and Exclusion (port of the
        // inuex35 reference's validation/postfit.py apply_fde + its call
        // site in optimize/stage.py's _compute_postfit_diagnostics). Runs
        // AFTER this epoch's main iSAM2 solve but BEFORE the per-epoch
        // LAMBDA / fix-and-hold block (reference: apply_fde is called
        // right after the shared main_ddpr_residuals diagnostics pass --
        // which is what feeds BOTH the quality gates and the CP-hold/
        // sanity FSM trigger -- and strictly before _run_lambda_ar), so
        // ambiguity resolution sees a float already cleaned of gross
        // outliers while the sanity FSM's residual INPUT stays pre-FDE
        // (see fgo_gtsam_backend.cpp's insertion point for the exact
        // ordering rationale).
        //
        // Evaluates each of THIS epoch's just-added DD pseudorange/
        // carrier factors at the current (post-solve, pre-FDE) estimate via
        // evaluateError() -- NOT factor->error() -- to get the raw
        // (unwhitened) residual in meters directly. This is a deliberate
        // improvement over a literal port of the reference's res_m =
        // sqrt(2*err)*cfg.sigma_pr*sqrt(2) (which reconstructs the raw
        // residual from factor->error()'s chi-squared value and the
        // factor's own sigma): factor->error() runs the noise model's
        // loss(), which for a Robust/Huber-wrapped factor returns the
        // DOWN-WEIGHTED loss rather than the raw chi-squared distance,
        // silently weakening outlier detection for exactly the large
        // residuals FDE exists to catch. The reference sidesteps this only
        // in main_ddpr_residuals (its _ddpr_factor_error has a
        // rebuild_for_robust branch that bypasses error() the same way)
        // but NOT in apply_fde's own _fde_collect_residuals -- harmless
        // there only because the reference's DD factors default to
        // non-robust (huber_pr=0.0). Our use_robust_loss defaults to TRUE,
        // so evaluateError() is used unconditionally here to stay correct
        // under both settings.
        //
        // SINGLE-PASS (fde_max_iterations<=1, the reference default):
        // scans ONLY this epoch's own DD PR/CP factors (their live graph
        // indices, resolved precisely via ISAM2Result::newFactorsIndices
        // -- strictly more precise than the reference's "last g3.size()
        // slots of the live factor array" heuristic, which can
        // mis-attribute a factor when findUnusedFactorSlots recycles an
        // older, now-unused slot). Rejects every PR entry with res_m >
        // fde_pseudorange_threshold_m and every CP entry with res_m >
        // fde_carrier_threshold_m (optional per-group median subtraction,
        // default off, mirroring the reference's FDE_MEDIAN_SUB env
        // knob). If the rejected count exceeds
        // fde_max_rejected_fraction * nv (nv = this epoch's own PR+CP
        // factor count), FDE is abandoned entirely for the epoch and
        // (mirroring the reference's trigger_cp_hold(...,
        // skip_if_active=True)) CP-hold is engaged at full strength
        // UNLESS it is already active. Deliberate deviation: when
        // use_cp_hold_recovery is off there is no hold to engage, so the
        // safeguard then simply skips FDE for the epoch with no other
        // effect.
        //
        // ITERATIVE (fde_max_iterations>1): scans the WHOLE live graph by
        // factor type (refreshed every iteration, so a removal never
        // leaves a stale index dangling) and removes only the single
        // worst |res-median| exceeder per round, re-estimating between
        // rounds; no safeguard (matching the reference, which only
        // guards the single-pass branch).
        //
        // Every rejected CP factor is treated as a cycle slip (reference
        // _fde_reset_rejected_amb): release any fix-and-hold pin for that
        // arc and bump its ambiguity generation so the next observation
        // gets a fresh graph symbol (see the ambSymbolId overlay). PR
        // rejects get no ambiguity-side action, matching the reference.
        //
        // Master switch, default OFF (bit-identical baseline without it).
        bool use_fde = false;
        /// PPC-style residual exclusion: screen code only and never sever a
        /// carrier ambiguity arc.  Carrier residuals remain robust-loss
        /// downweighted and are still visible in diagnostics.
        bool fde_pseudorange_only = false;
        /// Keep a gross-residual carrier factor under its robust loss, but
        /// quarantine its ambiguity from this epoch's LAMBDA/PAR set.  This
        /// avoids tearing a live fixed-lag graph while still protecting AR.
        bool fde_carrier_quarantine = false;
        double fde_pseudorange_threshold_m = 4.0;   ///< reference fde_pr
        double fde_carrier_threshold_m = 0.5;       ///< reference fde_cp
        double fde_max_rejected_fraction = 0.5;     ///< reference fde_max_frac
        int fde_max_iterations = 1;                 ///< reference fde_max_iter (1 = single-pass, the reference default)
        bool fde_median_subtraction = false;        ///< reference FDE_MEDIAN_SUB env knob (default off)

        // --- Sat-badness EWMA down-weighting (port of the inuex35 reference's
        // preprocess/sat_quality.py SatQualityState.sat_badness() score and its
        // consumption in buildfactor/factors.py's DD sigma inflation
        // (_add_ddpr_factor / _compute_cp_sigma)). ---
        //
        // A continuous per-(reference, target, signal) "badness" score, built
        // from persistent per-satellite quality memory that the GTSAM backend
        // now tracks alongside the existing CP-hold/sanity FSM and FDE state
        // (see fgo_gtsam_backend.cpp's optimizeProblemFixedLag): an EWMA and a
        // consecutive-bad-epoch streak of last-epoch's post-fit per-sat DDPR
        // residual, a decayed "was this the worst satellite" indicator, a
        // decayed "bad while serving as DD reference" indicator, an optional
        // decayed directional-pair indicator, and (when the data reaches the
        // backend -- see below) an elevation and SNR penalty. At each DD
        // factor's build time, bad_pair = max(badness(ref_sat), badness(
        // target_sat, paired with ref_sat)) inflates that factor's sigma
        // BEFORE robust-loss wrapping: sigma *= (1 + scale * bad_pair). This
        // mirrors the reference's timing exactly: the state feeding a given
        // epoch's bad_pair is always the PREVIOUS epoch's post-fit residuals
        // (postfit-then-next-epoch-factor-build), never the current epoch's
        // own not-yet-computed residual.
        //
        // Master switch, default OFF (bit-identical baseline without it: the
        // score is forced to 0.0 and every sigma computation is a no-op
        // multiply-by-one that is skipped entirely).
        //
        // Deliberate deviations from the reference (see the .cpp change-site
        // comments for the exact mechanics):
        //  1. cppr / recent_cppr: the reference's direct-and-recent "CP-vs-PR
        //     innovation consistency" reject terms are fed by a gate
        //     (factors.py's rejc_cp_pr) this codebase never ported. Both
        //     terms are instead fed by a persistent per-(satellite, signal)
        //     count of THIS backend's own FDE carrier rejections
        //     (use_fde) -- the closest existing analogue (both flag a
        //     specific arc's carrier observation as inconsistent with the
        //     rest of the solution). Unlike the reference's gate, this count
        //     is never reset (no analogue of the reference's unported
        //     cp_pr_rejc_max-triggered wipe), so it behaves like the
        //     reference run with that reset gate disabled. Structurally 0
        //     whenever use_fde is off, matching "reference with the gate
        //     disabled" behaviour.
        //  2. Satellite/pair identity: keyed by this backend's SatelliteId
        //     and SignalType (the AmbiguityState / DoubleDifference*Factor
        //     structs already carry per-factor satellite/reference_satellite/
        //     signal identity) instead of the reference's raw integer sat id
        //     -- a representational substitution only, not a behavioural one.
        //  3. Elevation is wired: DoubleDifferencePseudorangeFactor already
        //     carries the target satellite's elevation_rad, and the
        //     reference satellite's elevation is available via
        //     rover_reference_model.elevation_rad -- both populated upstream
        //     in fgo.cpp, so the el penalty term needed no new plumbing.
        //  4. SNR required a small addition: ObservationModelDebug gained a
        //     snr_dbhz field (populated from Observation::snr at both
        //     model_debug construction sites in fgo.cpp) so the backend can
        //     read rover_satellite_model.snr_dbhz / rover_reference_model.
        //     snr_dbhz per DD factor. Deemed small enough to do rather than
        //     skip (per the port's own scoping note); the snr penalty term
        //     defaults nonzero like the reference profile.
        //  5. update_pair_quality's per-(ref, target, signal) memory is only
        //     updated when sat_badness_alpha_recent_pair > 0 (the reference
        //     profile ships it at 0.0, i.e. contributing nothing); gating the
        //     UPDATE itself (not just its consumption) on the alpha avoids
        //     bookkeeping a term that is provably inert at the shipped
        //     defaults. No behavioural difference at any alpha value.
        // Sats not observed in a given epoch hard-reset (not decay) their
        // EWMA/streak entries to 0/0, exactly matching the reference.
        bool use_sat_badness_downweight = false;
        double sat_badness_ddpr_threshold_m = 1.0;        ///< reference sat_badness_ddpr_thresh
        int sat_badness_cppr_threshold = 1;               ///< reference sat_badness_cppr_thresh
        double sat_badness_alpha_ddpr = 1.0;
        double sat_badness_alpha_cppr = 1.0;
        double sat_badness_alpha_recent_cppr = 0.25;
        double sat_badness_alpha_recent_worst = 0.75;
        double sat_badness_alpha_recent_ref = 0.5;
        double sat_badness_alpha_recent_pair = 0.0;       ///< reference profile default: inert (see deviation 5)
        double sat_badness_alpha_el = 0.05;
        double sat_badness_alpha_snr = 0.05;
        double sat_badness_carrier_sigma_scale = 1.5;     ///< reference sigma_scale_cp
        double sat_badness_pseudorange_sigma_scale = 0.0; ///< reference sigma_scale_pr
        double sat_badness_el_ref_deg = 30.0;
        double sat_badness_snr_ref_dbhz = 40.0;
        double sat_badness_snr_span_db = 10.0;
        // --- obsq_* (reference SatQualityState/TcConfig dataclass defaults) ---
        double sat_badness_obsq_res_threshold_m = 2.0;         ///< reference obsq_res_thresh
        int sat_badness_obsq_bad_streak_cap = 8;               ///< reference obsq_bad_streak_cap
        double sat_badness_alpha_obsq_ewma = 1.0;
        double sat_badness_alpha_obsq_streak = 0.5;
        double sat_badness_obsq_ewma_alpha = 0.2;               ///< reference obsq_ewma_alpha (getattr default)
        double sat_badness_obsq_bad_streak_threshold_m = 2.0;   ///< reference obsq_bad_streak_thresh
        double sat_badness_recent_worst_decay = 0.8;            ///< reference obsq_recent_worst_decay
        double sat_badness_recent_cppr_decay = 0.8;             ///< reference obsq_recent_cppr_decay
        double sat_badness_recent_ref_decay = 0.85;             ///< reference obsq_recent_ref_decay
        double sat_badness_recent_pair_decay = 0.85;            ///< reference obsq_recent_pair_decay

        // --- CLAMPED variant (DEVIATION from the reference; first
        // non-port invention in this series -- see fgo_gtsam_backend.cpp's
        // satBadness()/runFde() change-sites for the exact mechanics). ---
        //
        // Measured failure of the faithful port above: sat_badness() is
        // uncapped and directly proportional to the raw post-fit DDPR
        // residual. In the reference's own operating regime (FLOAT error
        // stays ~1-2 m) that self-limits to O(1) scores -> a gentle 2-5x
        // carrier-sigma inflation. This codebase's deep-urban FLOAT
        // excursions instead reach 100s of meters, driving scores into the
        // hundreds to low thousands -> carrier sigma inflated ~1000x
        // graph-wide -> AR starves -> FLOAT never recovers (positive
        // feedback with no exit). Full-run verdict when shipped faithfully
        // (use_sat_badness_downweight on, these three knobs at their
        // faithful/no-op values below): fix-rate 39/63/66% -> 6/8/6%.
        //
        // These three knobs bound that feedback loop by construction while
        // leaving the reference's own gentle regime numerically untouched
        // (a run whose residuals/rejects never approach the clamp/cap is
        // bit-identical to the faithful port). Each defaults to a bound
        // (clamp=15 m, cap=3.0, cppr_decay=0.8); setting clamp=0, cap=0,
        // cppr_decay=1.0 together is the exact faithful-port escape hatch.
        // All three are no-ops whenever use_sat_badness_downweight is off.
        //
        // 1. Residual input clamp: caps the per-satellite post-fit DDPR
        //    residual BEFORE it feeds any badness term, at the single point
        //    (per_sat_res) all of them read it: the obsq EWMA input, the
        //    obsq bad-streak comparison, the next-epoch res_s term (via the
        //    sb_last_ddpr_per_sat snapshot), and the recent_ref_bad/
        //    recent_pair_bad increment inputs (both already separately
        //    min(2,.)-limited by the reference -- that existing limit is
        //    left in place; this clamp is strictly upstream of it). Chosen
        //    at 15 m = this codebase's own CP-hold/sanity FSM catastrophic
        //    residual threshold, i.e. "residual big enough that OUR OWN
        //    other recovery machinery already treats this epoch as
        //    catastrophic" -- beyond that point, charging MORE badness to
        //    the reference satellite for a residual that is almost
        //    certainly a FLOAT-wide problem (not that satellite's fault)
        //    is exactly the poisoning mechanism above. Does NOT touch the
        //    shared per_sat_res map itself (that map is also read raw by
        //    use_epoch_quality_gates/use_cp_hold_recovery, which must stay
        //    numerically untouched) -- only badness's own reads of it.
        //    0 = no clamp = faithful.
        double sat_badness_residual_clamp_m = 15.0;
        // 2. Final score cap: applied to sat_badness()'s returned score,
        //    after every additive term (including the now-clamped residual
        //    term above) but before the caller multiplies it into a sigma
        //    scale. At the defaults, max carrier-sigma inflation becomes
        //    1 + 1.5*3.0 = 5.5x -- the reference's own gentle regime,
        //    versus the uncapped port's observed 400-1550 (~1000x). Bounds
        //    the OTHER additive terms too (recent_worst/recent_ref/etc.),
        //    not just the residual one, since those can also accumulate.
        //    0 = no cap = faithful.
        double sat_badness_score_cap = 3.0;
        // 3. FDE-carrier-reject counter decay: the faithful port's cppr
        //    substitute (sb_fde_cp_reject_count; see use_sat_badness_
        //    downweight's deviation 1 above) is a persistent per-(satellite,
        //    signal) integer count that NEVER resets -- a second, independent
        //    source of unbounded growth alongside the residual-proportional
        //    term. This turns it into a decayed float, updated once per
        //    epoch as cppr[s,sig] = decay*prev + this_epoch_rejects (the
        //    number of that satellite/signal's FDE carrier rejections
        //    DURING that epoch's runFde() call, not cumulative). Feeds both
        //    sat_badness()'s direct cppr term and (via cppr_this_epoch)
        //    recent_cppr's own separate decay (sat_badness_recent_cppr_decay,
        //    unchanged) -- previously that decay chased an ever-growing
        //    target and could never plateau either. 1.0 = never decays =
        //    exactly the old (faithful) ever-growing-counter behaviour.
        double sat_badness_cppr_decay = 0.8;
    };

    struct EpochSeed {
        GNSSTime time;
        Vector3d position_ecef = Vector3d::Zero();
        double receiver_clock_bias_m = 0.0;
        // True only when position_ecef came from a valid SPP solve at this
        // epoch; false for last-valid/header fallbacks.
        bool fresh_spp_solution = false;
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
        GNSSSystem clock_group = GNSSSystem::GPS;
        Vector3d satellite_position_ecef = Vector3d::Zero();
        double corrected_pseudorange_m = 0.0;
        double sigma_m = 1.0;
        double elevation_rad = 0.0;
    };

    struct TimeDifferencedCarrierFactor {
        std::size_t previous_epoch_index = 0;
        std::size_t current_epoch_index = 0;
        SatelliteId satellite;
        SignalType signal = SignalType::GPS_L1CA;
        Vector3d previous_satellite_position_ecef = Vector3d::Zero();
        Vector3d current_satellite_position_ecef = Vector3d::Zero();
        double delta_carrier_m = 0.0;
        double sigma_m = 0.03;
        double dt_s = 0.0;
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

    struct FGOProblemDiagnostics {
        std::size_t input_epochs = 0;
        std::size_t seeded_epochs = 0;
        std::size_t skipped_epochs_without_seed = 0;
        std::size_t double_difference_matched_base_epochs = 0;
        std::size_t double_difference_interpolated_base_epochs = 0;
        std::size_t double_difference_candidate_pairs = 0;
        std::size_t double_difference_rejected_no_base_epoch = 0;
        std::size_t double_difference_rejected_no_reference = 0;
        std::size_t tdcp_candidate_pairs = 0;
        std::size_t tdcp_rejected_gap = 0;
        std::size_t tdcp_rejected_missing_previous = 0;
        std::size_t tdcp_rejected_loss_of_lock = 0;
        std::size_t tdcp_rejected_code_phase_jump = 0;
        std::size_t code_minus_carrier_jump_resets = 0;       ///< CMC screening: arc breaks forced
        std::size_t geometry_free_cycle_slip_resets = 0;      ///< confirmed geometry-free band resets
        std::size_t code_minus_carrier_level_exclusions = 0;  ///< CMC screening: (sat,signal) epochs excluded
        std::size_t cmc_ref_avoided_count = 0;  ///< cmc_aware_reference_selection: references changed away from a CMC-excluded candidate
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
        Vector3d init_velocity_nav = Vector3d::Zero();
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

    struct FGOProblem {
        std::vector<EpochSeed> epochs;
        ImuInput imu;  ///< Milestone 2b IMU inputs (valid only when populated).
        std::vector<bool> clock_jumps;
        std::vector<PseudorangeFactor> pseudorange_factors;
        std::vector<TimeDifferencedCarrierFactor> tdcp_factors;
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

    struct FGODiagnostics {
        int iterations = 0;
        bool converged = false;
        std::size_t epochs = 0;
        std::size_t pseudorange_factors = 0;
        /// TDCP measurements present in the backend-independent problem.
        std::size_t tdcp_factors = 0;
        /// TDCP residual rows/factors actually inserted by the selected backend.
        std::size_t tdcp_factors_inserted = 0;
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
        std::size_t ambiguity_between_factors = 0;
        std::size_t robust_pseudorange_factors = 0;
        std::size_t robust_carrier_phase_factors = 0;
        std::size_t robust_double_difference_pseudorange_factors = 0;
        std::size_t robust_double_difference_carrier_factors = 0;
        std::size_t robust_tdcp_factors = 0;
        std::size_t graph_factors = 0;
        std::size_t graph_values = 0;
        std::size_t imu_intervals = 0;  ///< 2b: CombinedImuFactors added between epochs
        std::size_t smoother_max_window_vars = 0;  ///< 2c: peak in-window variable count
        std::size_t smoother_updates = 0;          ///< 2c: number of smoother.update() calls
        std::size_t smoother_recovery_epochs = 0;  ///< 2e: epochs re-anchored after an indeterminate update
        std::size_t nhc_epochs = 0;   ///< 2d: epochs an NHC factor was applied
        std::size_t zupt_epochs = 0;  ///< 2d: epochs a ZUPT prior was applied
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

    /// Per-epoch fixed-lag integrity state.  These values expose why an epoch
    /// did or did not fix, rather than only reporting the final FIX/FLOAT label.
    struct FGOEpochDiagnostics {
        GNSSTime time;
        AmbiguityResolutionOutcome ar_outcome =
            AmbiguityResolutionOutcome::NotAttempted;
        double ddpr_rms_m = 0.0;
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
    };

    struct FGOResult {
        Solution solution;
        FGODiagnostics diagnostics;
        std::vector<AmbiguityEstimate> ambiguity_estimates;
        std::vector<std::set<SatelliteId>> ambiguity_candidate_satellites_by_epoch;
        std::vector<std::set<SatelliteId>> ambiguity_reference_satellites_by_epoch;
        std::vector<std::map<SatelliteId, double>> ambiguity_estimate_cycles_by_epoch;
        std::vector<Vector3d> epoch_velocities_ecef_mps;
        std::vector<LambdaDebugEntry> lambda_debug_entries;
        std::vector<CostTraceEntry> cost_trace_entries;
        std::vector<FGOEpochDiagnostics> epoch_diagnostics;
        std::vector<TemporalCarrierShadowFactorDiagnostics>
            temporal_carrier_shadow_factors;
        std::vector<PredictedDdprQualityFactorDiagnostics>
            predicted_ddpr_quality_factors;
        std::vector<PredictedDdprBiasStateDiagnostics>
            predicted_ddpr_bias_state_factors;
        // Milestone 2b (populated only by the GTSAM IMU-coupled path):
        // per-epoch estimated attitude as [roll, pitch, heading] in degrees
        // (body FLU -> nav ENU; heading is clockwise from North) and estimated
        // velocity in the ENU nav frame [m/s].
        std::vector<Vector3d> epoch_attitude_rpy_deg;
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
