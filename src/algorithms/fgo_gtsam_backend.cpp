#include <libgnss++/fusion/relative_height_pairs.hpp>

// GTSAM backend for FGOProcessor::optimizeProblem (Phase 1, RTK/DD-focused).
//
// Consumes the exact same FGOProcessor::FGOProblem the native Eigen backend
// (fgo.cpp) consumes and produces an equivalent FGOProcessor::FGOResult.
// Everything in this file is compiled only when GNSSPP_HAS_GTSAM is defined
// (see CMakeLists.txt: only added to gnss_lib_noopt when find_package(GTSAM)
// succeeds), so no other translation unit needs to guard against a missing
// GTSAM install.
//
// --- Observation-convention note (see docs/gtsam_backend_design.md risk #2) ---
// gtsam::gnss::DoubleDifferenceData::observed() computes
//     (rovRef - baseRef) - (rovTarget - baseTarget)
// while libgnss's FGOProcessor::DoubleDifferencePseudorangeFactor stores
//     observed_dd_pseudorange_m = (satellite_* - base_satellite_*)
//                                - (reference_* - base_reference_*)
// i.e. (target - reference), the mirror image of GTSAM's (ref - target).
// Feeding libgnss's "satellite" (target) into GTSAM's "Ref" slot and
// libgnss's "reference_satellite" into GTSAM's "Target" slot makes the two
// conventions coincide exactly (verified by the runtime assertion below,
// and by the DD carrier equivalent). The same swap applies to the ambiguity
// keys of DoubleDifferenceCarrierPhaseFactor: GTSAM forms
// lam*(ambRef - ambTarget); libgnss tracks a single lumped DD ambiguity
// equal to lam*(N_target - N_reference). So ambRef <- the real ambiguity
// node (seeded with libgnss's lumped value, in cycles) and ambTarget <- a
// single shared dummy node pinned to 0 with a tight prior.

#include <libgnss++/algorithms/disjoint_constellation_partition.hpp>
#include <libgnss++/algorithms/residual_ionosphere_contract.hpp>
#include <libgnss++/algorithms/joint_ionosphere_graph_plan.hpp>
#include <libgnss++/algorithms/tdcp_frequency_pairing.hpp>
#include <libgnss++/algorithms/pseudorange_position_information.hpp>
#include <libgnss++/algorithms/tdcp_residual_state_factor.hpp>
#include <libgnss++/algorithms/source_clock_c0d_initializer.hpp>
#include <libgnss++/algorithms/native_lm_lambda_floor.hpp>
#include <libgnss++/algorithms/native_nhc_gate.hpp>
#include <libgnss++/algorithms/native_imu_bias_density.hpp>
#include <libgnss++/algorithms/native_raw_p_ecef_doppler_staging.hpp>
#include <libgnss++/algorithms/doppler_rotation_rate.hpp>
#include "fgo_gtsam_internal.hpp"
#include "fgo_pseudorange_cauchy.hpp"
#include <gtsam/navigation/ImuFactor.h>

namespace libgnss {

using namespace fgo_gtsam_internal;

FGOProcessor::FGOResult optimizeProblemWithGtsam(
    const FGOProcessor::FGOProblem& problem,
    const FGOProcessor::FGOConfig& config,
    FGOProcessor::FGOResult result) {
    if (config.use_official_tdcp_huber_k &&
        config.use_official_tdcp_snr_type_sigma) {
        throw std::invalid_argument(
            "Phase118 official TDCP Huber-k mapping cannot be combined with "
            "the Phase117 dynamic TDCP sigma candidate");
    }
    if (config.use_official_tdcp_resl_atmosphere_cancellation &&
        config.use_official_tdcp_snr_type_sigma) {
        throw std::invalid_argument(
            "Phase120 official resL TDCP normalization cannot be combined "
            "with the Phase117 dynamic TDCP sigma candidate");
    }
    double ordinary_tdcp_huber_threshold_sigma =
        config.tdcp_huber_threshold_sigma;
    if (!fgo::resolveOrdinaryTdcpHuberThresholdSigma(
            config, ordinary_tdcp_huber_threshold_sigma)) {
        throw std::invalid_argument(
            "Phase118 official TDCP Huber-k mapping requires a recognised "
            "source setting.Type");
    }
    const std::size_t num_epochs = problem.epochs.size();
    const bool use_pose3 = config.use_pose3_state;
    const bool use_imu =
        use_pose3 && config.use_imu && problem.imu.valid && num_epochs >= 2;
    const bool use_gnss_velocity_states =
        !use_imu && !use_pose3 && config.use_velocity_states &&
        (!problem.undifferenced_doppler_factors.empty() ||
         config.use_native_raw_p_no_doppler_graph ||
         config.use_native_raw_p_ecef_doppler_gnss_first);
    const bool use_native_raw_p_no_doppler_graph =
        config.use_native_raw_p_no_doppler_graph;
    const bool use_native_raw_p_ecef_doppler_graph =
        config.use_native_raw_p_ecef_doppler_gnss_first;
    const bool use_native_raw_p_seed_graph =
        use_native_raw_p_no_doppler_graph ||
        use_native_raw_p_ecef_doppler_graph;
    const bool use_native_source_clock_c0d_gnss_first =
        config.use_native_source_clock_c0d_gnss_first_meter_state_handoff &&
        use_gnss_velocity_states;
    const bool use_native_direct_wls_ephemeral_main_seed =
        config.use_native_direct_wls_ephemeral_c7d_main_seed;
    const bool phase135_requested =
        config.use_native_phase135_official_affine_measurement_family;
    const bool phase138_requested =
        config.use_native_phase138_affine_tdcp_anchor_range_constant;
    const bool phase171_requested =
        config.use_native_phase171_raw_p_no_doppler_imu_main;
    const bool phase201_requested =
        config.use_native_phase201_source_inclusive_forward_imu_schedule;
    result.diagnostics.native_phase171_no_doppler_imu_main_enabled =
        phase171_requested;
    if (phase171_requested) {
        result.diagnostics.native_phase171_unobserved_clock_gauge_sigma_m =
            config.native_raw_p_no_doppler_unobserved_clock_gauge_sigma_m;
    }
    if (phase138_requested && !phase135_requested) {
        throw std::invalid_argument(
            "Phase138 affine TDCP anchor-range correction requires the "
            "Phase135 official affine measurement family");
    }
    if (phase171_requested &&
        (config.backend != FGOBackend::GTSAM || !use_imu ||
         !config.use_native_source_clock_c0d_factor ||
         !config.use_native_source_clock_c0d_meter_state_parity ||
         !config.use_native_source_clock_c0d_epoch_vector_parity ||
         !config.use_native_source_clock_c0d_gnss_first_meter_state_handoff ||
         !config.use_upstream_observable_quality ||
         config.use_native_source_clock_c0d_raw_drift_d_initializer ||
         !config.use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver ||
         config.use_undifferenced_doppler_factors ||
         !config.use_motion_factors || !config.use_clock_motion_factors ||
         config.use_native_direct_wls_ephemeral_c7d_main_seed ||
         config.use_native_pdc_state_bridge ||
         config.use_inter_system_biases ||
         config.use_receiver_signal_bias_states ||
         config.use_residual_ionosphere_states ||
         config.use_native_phase135_official_affine_measurement_family ||
         !std::isfinite(
             config.native_raw_p_no_doppler_unobserved_clock_gauge_sigma_m) ||
         config.native_raw_p_no_doppler_unobserved_clock_gauge_sigma_m <= 0.0 ||
         !problem.undifferenced_doppler_factors.empty() ||
         !problem.single_difference_doppler_factors.empty() ||
         !problem.double_difference_pseudorange_factors.empty() ||
         !problem.double_difference_carrier_factors.empty() ||
         config.max_iterations != 12)) {
        throw std::invalid_argument(
            "Phase171 requires the dedicated Pose3/IMU source C0/D graph "
            "with an empty generic Doppler family and configured max_iterations=12");
    }
    if (phase201_requested &&
        (config.backend != FGOBackend::GTSAM || !phase171_requested ||
         !use_imu || config.native_source_clock_c0d_phone != "pixel5")) {
        throw std::invalid_argument(
            "Phase201 source-inclusive-forward IMU schedule requires the "
            "dedicated Pixel5 Phase171 Pose3/IMU main graph");
    }
    const bool phase205_requested = config.use_native_phase205_source_count_bias_density;
    if (phase205_requested &&
        (!phase171_requested || !use_imu || phase201_requested ||
         config.native_source_clock_c0d_phone != "pixel5")) {
        throw std::invalid_argument(
            "Phase205 requires Pixel5 Phase171 IMU main with legacy integration");
    }
    result.diagnostics.native_phase205_bias_density_enabled = phase205_requested;
    const bool phase209_requested = config.use_native_phase209_source_separate_imu_factors;
    if (phase209_requested &&
        (!phase171_requested || !use_imu || phase201_requested || phase205_requested ||
         config.native_source_clock_c0d_phone != "pixel5")) {
        throw std::invalid_argument(
            "Phase209 requires Pixel5 Phase171 main, legacy integration, and Phase205 off");
    }
    result.diagnostics.native_phase209_separate_imu_enabled = phase209_requested;
    const bool phase213_requested = config.use_native_phase213_main_doppler;
    if (config.use_native_doppler_rotation_rate &&
        !phase213_requested && !use_native_raw_p_ecef_doppler_graph)
        throw std::invalid_argument("Rotation-rate Doppler requires dedicated raw ECEF-D or Phase213 main graph");
    std::vector<FGOProcessor::UndifferencedDopplerFactor> phase213_rows;
    if (!phase213_requested && !problem.native_phase213_main_doppler_rows.empty()) {
        throw std::invalid_argument("Phase213 rows require explicit selector");
    }
    if (phase213_requested) {
        if (!phase171_requested || !use_imu || phase201_requested ||
            phase205_requested || phase209_requested ||
            config.native_source_clock_c0d_phone != "pixel5" ||
            problem.native_phase213_main_doppler_rows.empty() ||
            !std::isfinite(problem.imu.nav_origin_lat_rad) ||
            !std::isfinite(problem.imu.nav_origin_lon_rad)) {
            throw std::invalid_argument("Phase213 requires Pixel5 Phase171 main and corrected raw D rows with legacy IMU");
        }
        std::string failure;
        if (!raw_p_ecef_doppler::remapDopplerFactors(
                problem.epochs, problem.epochs, problem.native_phase213_main_doppler_rows,
                phase213_rows, failure)) {
            throw std::invalid_argument("Phase213 invalid main Doppler transfer: " + failure);
        }
    }
    result.diagnostics.native_phase213_main_doppler_enabled = phase213_requested;
    const bool phase217_requested = config.use_native_phase217_main_pose3_motion;
    if (phase217_requested && (!phase171_requested || !use_imu || phase201_requested ||
        phase205_requested || phase209_requested ||
        config.native_source_clock_c0d_phone != "pixel5" ||
        config.use_position_motion_factors || !config.pose3_lever_arm_body_m.allFinite() ||
        config.pose3_lever_arm_body_m.norm() > 1e-12)) {
        throw std::invalid_argument("Phase217 requires Pixel5 Phase171 legacy IMU, zero lever arm and no duplicate position motion");
    }
    result.diagnostics.native_phase217_main_motion_enabled = phase217_requested;
    if (phase201_requested) {
        result.diagnostics
            .native_phase201_source_inclusive_forward_imu_schedule_enabled =
            true;
        result.diagnostics
            .native_phase201_source_inclusive_forward_imu_schedule_attempted =
            true;
    }
    if (config.allow_native_raw_p_sparse_epochs &&
        (!use_native_raw_p_ecef_doppler_graph || !config.use_motion_factors ||
         !config.use_clock_motion_factors)) {
        throw std::invalid_argument("Sparse raw P requires ECEF Doppler staging with motion and clock edges");
    }
    if (use_native_raw_p_seed_graph) {
        result.diagnostics.native_raw_p_no_doppler_graph_enabled =
            use_native_raw_p_no_doppler_graph;
        result.diagnostics.native_raw_p_ecef_doppler_gnss_first_enabled =
            use_native_raw_p_ecef_doppler_graph;
        result.diagnostics
            .native_raw_p_no_doppler_unobserved_clock_gauge_sigma_m =
            config.native_raw_p_no_doppler_unobserved_clock_gauge_sigma_m;
        // Phase164/171 staging are complete, dedicated Point3/V/C7/D
        // recipes.  Do not let a caller accidentally combine them with
        // Pose3/IMU, legacy global ISBs, or a partial source-clock selector.
        if (config.backend != FGOBackend::GTSAM || use_pose3 || use_imu ||
            !config.use_velocity_states || !config.use_motion_factors ||
            !config.use_velocity_motion_factors ||
            !config.use_clock_motion_factors ||
            !config.use_native_source_clock_c0d_factor ||
            !config.use_native_source_clock_c0d_meter_state_parity ||
            !config.use_native_source_clock_c0d_epoch_vector_parity ||
            !std::isfinite(
                config.native_raw_p_no_doppler_unobserved_clock_gauge_sigma_m) ||
            config.native_raw_p_no_doppler_unobserved_clock_gauge_sigma_m <= 0.0 ||
            config.use_inter_system_biases ||
            config.use_receiver_signal_bias_states ||
            config.use_residual_ionosphere_states ||
            config.use_native_pdc_state_bridge || config.use_fixed_lag_smoother ||
            config.native_source_clock_c0d_phone.empty() ||
            nativeSourceClockC0DPhoneExcluded(
                config.native_source_clock_c0d_phone) ||
            (use_native_raw_p_no_doppler_graph &&
             !problem.undifferenced_doppler_factors.empty()) ||
            (use_native_raw_p_ecef_doppler_graph &&
             problem.undifferenced_doppler_factors.empty()) ||
            (use_native_raw_p_ecef_doppler_graph &&
             (!config.use_undifferenced_doppler_factors ||
              !config.use_corrected_undifferenced_doppler_factors ||
              !config.use_upstream_observable_quality)) ||
            (use_native_raw_p_no_doppler_graph &&
             use_native_raw_p_ecef_doppler_graph) ||
            !problem.carrier_phase_factors.empty() ||
            !problem.double_difference_pseudorange_factors.empty() ||
            !problem.double_difference_carrier_factors.empty() ||
            num_epochs < 2 || problem.native_raw_p_no_doppler_seeds.size() !=
                                   num_epochs) {
            throw std::invalid_argument(
                use_native_raw_p_ecef_doppler_graph
                    ? "Phase171 ECEF raw-P+D staging requires the complete "
                      "Point3/V/C7/D recipe and corrected direct-quality rows"
                    : "Phase164 requires the complete Point3/V/C7/D raw-P "
                      "graph recipe and an empty generic Doppler family");
        }
        std::vector<std::size_t> pseudorange_rows_per_epoch(num_epochs, 0U);
        for (const auto& factor : problem.pseudorange_factors) {
            if (factor.epoch_index >= num_epochs ||
                !factor.satellite_position_ecef.allFinite() ||
                !std::isfinite(factor.corrected_pseudorange_m) ||
                !std::isfinite(factor.sigma_m) || factor.sigma_m <= 0.0 ||
                sourceClockComponentFor(factor.satellite.system,
                                        factor.signal) < 0) {
                std::fprintf(stderr, "[native-raw-staging] invalid pseudorange row\n");
                result.diagnostics.converged = false;
                return result;
            }
            ++pseudorange_rows_per_epoch[factor.epoch_index];
        }
        if (use_native_raw_p_ecef_doppler_graph) {
            for (const auto& factor : problem.undifferenced_doppler_factors) {
                if (factor.epoch_index >= num_epochs ||
                    !factor.los.allFinite() || factor.los.norm() <= 0.0 ||
                    std::abs(factor.los.norm() - 1.0) > 1e-6 ||
                    !std::isfinite(factor.residual_mps) ||
                    !std::isfinite(factor.sigma_mps) ||
                    factor.sigma_mps <= 0.0 ||
                    !factor.source_satellite_state_available ||
                    !factor.source_satellite_position_ecef.allFinite() ||
                    !factor.source_satellite_velocity_ecef.allFinite() ||
                    !factor.satellite_position_ecef.allFinite() ||
                    !factor.satellite_velocity_ecef.allFinite() ||
                    !std::isfinite(factor.measured_range_rate_mps) ||
                    !std::isfinite(factor.satellite_clock_drift_mps) ||
                    !factor.includes_receiver_clock_drift ||
                    !factor.uses_rotated_satellite_state) {
                    std::fprintf(stderr, "[native-raw-staging] invalid Doppler row\n");
                    result.diagnostics.converged = false;
                    return result;
                }
            }
        }
        if (problem.pseudorange_factors.empty()) {
            std::fprintf(stderr, "[native-raw-staging] empty pseudorange family\n");
            result.diagnostics.converged = false;
            return result;
        }
        if (config.allow_native_raw_p_sparse_epochs &&
            std::none_of(pseudorange_rows_per_epoch.begin(), pseudorange_rows_per_epoch.end(),
                         [](std::size_t count) { return count >= 4U; })) {
            std::fprintf(stderr, "[native-raw-staging] no supported P epoch\n");
            result.diagnostics.converged = false;
            return result;
        }
        for (std::size_t i = 0; i < num_epochs; ++i) {
            const auto& seed = problem.native_raw_p_no_doppler_seeds[i];
            const auto& epoch = problem.epochs[i];
            if (seed.epoch_index != i || seed.status !=
                                             raw_p_seed::SeedAdapterStatus::Accepted ||
                !seed.has_position || !seed.position_ecef.allFinite() ||
                !seed.has_velocity || !seed.velocity_ecef_mps.allFinite() ||
                !seed.has_clock || !std::isfinite(seed.clock_bias_m) ||
                !seed.has_clock_rate || !std::isfinite(seed.clock_rate_mps) ||
                seed.reference_clock_group != GNSSSystem::GPS ||
                !seed.c7_clock_mapping_supported ||
                !seed.clock_bias_component_available[0] ||
                !std::isfinite(seed.clock_bias_components_m[0]) ||
                !(seed.time == epoch.time) ||
                seed.raw_source_index != epoch.raw_source_index ||
                seed.raw_utc_time_millis != epoch.raw_utc_time_millis ||
                (!config.allow_native_raw_p_sparse_epochs && pseudorange_rows_per_epoch[i] < 4U)) {
                std::fprintf(stderr,
                    "[native-raw-staging] invalid seed or insufficient P support; fewer_than_four_P=%d\n",
                    pseudorange_rows_per_epoch[i] < 4U ? 1 : 0);
                result.diagnostics.converged = false;
                return result;
            }
            if (i > 0U) {
                const double dt_s = problem.epochs[i].time -
                                    problem.epochs[i - 1U].time;
                const bool clock_jump =
                    i < problem.clock_jumps.size() && problem.clock_jumps[i];
                if (!nativeSourceClockC0DEdgeDecision(
                         dt_s, clock_jump,
                         config.native_source_clock_c0d_phone)
                         .eligible) {
                    std::fprintf(stderr,
                        "[native-raw-staging] ineligible clock edge; clock_jump=%d; dt_s=%.9g\n",
                        clock_jump ? 1 : 0, dt_s);
                    result.diagnostics.converged = false;
                    return result;
                }
            }
        }
    }
    const bool source_clock_c0d_problem_path =
        use_imu || use_native_source_clock_c0d_gnss_first ||
        use_native_raw_p_seed_graph;
    const bool phase99_qr_requested =
        config.use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver;
    const bool phase99_main_meter_state_graph =
        use_imu && config.use_native_source_clock_c0d_factor &&
        config.use_native_source_clock_c0d_meter_state_parity &&
        (config.use_native_source_clock_c0d_gnss_first_meter_state_handoff ||
         use_native_direct_wls_ephemeral_main_seed) &&
        !config.use_native_source_clock_c0d_raw_drift_d_initializer;
    const bool phase99_qr_selected =
        phase99_qr_requested && phase99_main_meter_state_graph;
    const bool phase167_requested =
        config.use_native_phase167_raw_p_no_doppler_lm_termination_budget;
    const bool phase143_requested =
        config.use_native_phase143_official_main_lm_termination_budget ||
        phase167_requested;
    const bool phase143_main_scope =
        phase143_requested && phase99_main_meter_state_graph;
    const bool phase143_gnss_first_scope =
        phase143_requested && !use_imu && !use_pose3 &&
        use_gnss_velocity_states && config.max_iterations == 1000;
    if (phase143_requested && !phase143_main_scope &&
        !phase143_gnss_first_scope) {
        throw std::invalid_argument(
            "Phase143 official main LM budget requires the Phase142/99 "
            "meter-state main graph or the unchanged 1000-iteration "
            "GNSS-first staging graph");
    }
    if (phase143_main_scope && !phase99_qr_selected) {
        throw std::invalid_argument(
            "Phase143 official main LM budget requires the Phase99 "
            "MULTIFRONTAL_QR main graph");
    }
    if (phase143_main_scope && config.max_iterations != 12 &&
        config.max_iterations != 1000) {
        throw std::invalid_argument(
            "Phase143 main LM budget requires the frozen configured "
            "12/1000 iteration boundary");
    }
    // The selector is intentionally a Phase93 main-graph opt-in.  A
    // GNSS-first Point3/velocity call remains Cholesky even if a caller
    // accidentally carries the selector into that staging configuration;
    // an unrelated Pose3 path fails closed instead of silently changing
    // solver policy.
    if (phase99_qr_requested && use_imu && !phase99_main_meter_state_graph) {
        throw std::invalid_argument(
            "Phase99 main multifrontal QR solver requires the Phase93/114 "
            "metre-valued C0/D Pose3+IMU main graph");
    }
    if (phase99_qr_selected && config.use_fixed_lag_smoother) {
        throw std::invalid_argument(
            "Phase99 main multifrontal QR solver requires the batch LM path");
    }
    // GTSAM is the historical backend used by this translation unit.  Make
    // the selected branch explicit even on fail-closed pre-solve returns and
    // on the legacy/GNSS-first Cholesky paths.
    result.diagnostics.selected_linear_solver_type =
        phase99_qr_selected ? "MULTIFRONTAL_QR" : "MULTIFRONTAL_CHOLESKY";
    result.diagnostics.selected_solver_branch = "multifrontal";
    result.diagnostics.selected_elimination_function =
        phase99_qr_selected ? "EliminateQR" : "EliminatePreferCholesky";
    result.diagnostics.phase135_official_affine_measurement_family_enabled =
        phase135_requested;
    result.diagnostics.phase135_fixed_initial_geometry = phase135_requested;
    result.diagnostics.phase135_single_sagnac_representation = phase135_requested;
    if (phase135_requested) {
        result.diagnostics.phase135_geometry_representation =
            "RTKLIB-geodist-single-Sagnac-fixed-initial-LOS";
    }
    result.diagnostics.phase138_affine_tdcp_anchor_range_constant_enabled =
        phase138_requested;
    if (phase138_requested) {
        result.diagnostics.phase138_measurement_equation =
            "tdcp_native-(rho_current_initial-rho_previous_initial)";
        result.diagnostics.phase138_geometry_representation =
            "RTKLIB-geodist-single-Sagnac-fixed-initial-endpoints";
        result.diagnostics.phase138_same_endpoint_epoch_and_satellite_state =
            false;
        result.diagnostics.phase138_same_satellite_state = false;
        result.diagnostics.phase138_finite_adjusted_measurements = false;
        result.diagnostics.phase138_no_raw_or_zero_fallback = false;
        result.diagnostics.phase138_transactional = false;
    }
    result.diagnostics
        .native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected =
        phase99_qr_selected;
    if (config.use_native_source_clock_c0d_active_solve_diagnostic &&
        !config.use_native_source_clock_c0d_factor) {
        throw std::invalid_argument(
            "native source ClockFactor_CCDD active-solve diagnostics require "
            "the source C0/D factor");
    }
    if (config.use_native_source_clock_c0d_phase96_main_diagnostics &&
        (!config.use_native_source_clock_c0d_factor ||
         !config.use_native_source_clock_c0d_meter_state_parity)) {
        throw std::invalid_argument(
            "Phase96 main diagnostics require the metre-valued source C0/D graph");
    }
    if (config.use_native_source_clock_c0d_phase96_main_diagnostics &&
        !use_imu) {
        throw std::invalid_argument(
            "Phase96 main diagnostics require the Pose3+IMU main graph");
    }
    if (config.use_native_source_clock_c0d_phase97_singular_system_diagnostics &&
        (!config.use_native_source_clock_c0d_factor ||
         !config.use_native_source_clock_c0d_meter_state_parity)) {
        throw std::invalid_argument(
            "Phase97 singular-system diagnostics require the metre-valued source C0/D graph");
    }
    // The source-exact row is intentionally scoped to either the frozen
    // direct observable-quality Pose3+IMU batch path or the Phase93
    // GNSS-first Point3+velocity staging path.  Reject an invalid library
    // configuration rather than silently falling back to the legacy scalar
    // clock row while the candidate flag is set.
    if (config.use_native_source_clock_c0d_factor &&
        (!nativeSourceClockC0DBackendConfigurationAllowed(config) ||
         !source_clock_c0d_problem_path || num_epochs < 2)) {
        throw std::invalid_argument(
            "native source ClockFactor_CCDD C0/D requires direct observable "
            "quality, no PDC bridge, and an approved GTSAM batch path");
    }
    if (config.use_native_source_clock_c0d_raw_drift_d_initializer &&
        (!config.use_native_source_clock_c0d_factor ||
         !nativeSourceClockC0DBackendConfigurationAllowed(config) ||
         !source_clock_c0d_problem_path || num_epochs < 2)) {
        throw std::invalid_argument(
            "native source raw-drift D initializer requires the direct "
            "source C0/D batch path");
    }
    if (config.use_native_source_clock_c0d_meter_state_parity &&
        (!config.use_native_source_clock_c0d_factor ||
         !nativeSourceClockC0DBackendConfigurationAllowed(config) ||
         !source_clock_c0d_problem_path || num_epochs < 2)) {
        throw std::invalid_argument(
            "native source meter clock-state parity requires the direct "
            "source C0/D batch path");
    }
    if (config.use_native_source_clock_c0d_gnss_first_meter_state_handoff &&
        (!config.use_native_source_clock_c0d_factor ||
         !config.use_native_source_clock_c0d_meter_state_parity ||
         (!use_imu && !config.use_native_source_clock_c0d_raw_drift_d_initializer) ||
         (use_imu && config.use_native_source_clock_c0d_raw_drift_d_initializer))) {
        throw std::invalid_argument(
            "native source GNSS-first meter-state handoff requires either "
            "the staged Point3/velocity raw-D initializer or the main "
            "Pose3+IMU optimized-D handoff");
    }
    if (config.use_native_source_clock_c0d_gnss_first_meter_state_handoff &&
        use_imu) {
        // The main graph is intentionally forbidden from falling back to its
        // raw EpochSeed D, zero, WLS, or any inferred value.  The app places
        // the same-run GNSS-first result here after exact retained-key checks.
        if (problem.native_source_clock_c0d_gnss_first_d_handoff_mps.size() !=
            num_epochs ||
            !std::all_of(
                problem.native_source_clock_c0d_gnss_first_d_handoff_mps.begin(),
                problem.native_source_clock_c0d_gnss_first_d_handoff_mps.end(),
                [](double value) { return std::isfinite(value); })) {
            result.diagnostics.converged = false;
            if (phase135_requested) {
                result.diagnostics.phase135_configuration_valid = false;
                result.diagnostics.phase135_configuration_failure =
                    "missing or nonfinite GNSS-first D handoff";
                if (phase138_requested) {
                    result.diagnostics.phase138_configuration_valid = false;
                    result.diagnostics.phase138_configuration_failure =
                        result.diagnostics.phase135_configuration_failure;
                    result.diagnostics.phase138_factor_count_unchanged = false;
                }
            }
            return result;
        }
        if (config.use_native_source_clock_c0d_epoch_vector_parity &&
            (problem.native_source_clock_c0d_gnss_first_c_handoff_m.size() !=
                 num_epochs ||
             !std::all_of(
                 problem.native_source_clock_c0d_gnss_first_c_handoff_m.begin(),
                 problem.native_source_clock_c0d_gnss_first_c_handoff_m.end(),
                 [](const FGOProcessor::EpochClockBiasComponentsM& value) {
                     return std::all_of(value.begin(), value.end(),
                                        [](double component) {
                                            return std::isfinite(component);
                                        });
                 }))) {
            // Main-stage C must come only from the same-run optimized
            // GNSS-first export; no scalar/raw/zero fallback is permitted.
            result.diagnostics.converged = false;
            if (phase135_requested) {
                result.diagnostics.phase135_configuration_valid = false;
                result.diagnostics.phase135_configuration_failure =
                    "missing or nonfinite GNSS-first C handoff";
                if (phase138_requested) {
                    result.diagnostics.phase138_configuration_valid = false;
                    result.diagnostics.phase138_configuration_failure =
                        result.diagnostics.phase135_configuration_failure;
                    result.diagnostics.phase138_factor_count_unchanged = false;
                }
            }
            return result;
        }
    }
    if (phase135_requested) {
        // Phase135 is one all-or-nothing source factor family.  The
        // candidate cannot silently fall back to the nonlinear/native rows,
        // the legacy global ISB namespace, or an alternate measurement
        // preparation path.
        if (!config.use_native_source_clock_c0d_epoch_vector_parity ||
            config.use_fixed_lag_smoother ||
            (use_pose3 && !use_imu) ||
            (use_imu && config.pose3_lever_arm_body_m.norm() > 1e-12) ||
            (use_imu && !phase99_qr_requested) ||
            (!use_imu && !problem.undifferenced_doppler_factors.empty() &&
             !use_gnss_velocity_states) ||
            config.use_native_direct_wls_ephemeral_c7d_main_seed &&
                config.use_native_source_clock_c0d_raw_drift_d_initializer) {
            result.diagnostics.phase135_configuration_valid = false;
            result.diagnostics.phase135_configuration_failure =
                "unsupported Phase135 graph/configuration combination";
            if (phase138_requested) {
                result.diagnostics.phase138_configuration_valid = false;
                result.diagnostics.phase138_configuration_failure =
                    result.diagnostics.phase135_configuration_failure;
                result.diagnostics.phase138_factor_count_unchanged = false;
            }
            throw std::invalid_argument(
                "Phase135 official affine family requires the approved batch "
                "C7 graph, QR main path, and complete GNSS-first state binding");
        }
        if (!problem.carrier_phase_factors.empty() ||
            !problem.double_difference_pseudorange_factors.empty() ||
            !problem.double_difference_carrier_factors.empty() ||
            !problem.single_difference_doppler_factors.empty() ||
            !problem.single_difference_tdcp_factors.empty()) {
            result.diagnostics.phase135_configuration_valid = false;
            result.diagnostics.phase135_configuration_failure =
                "unsupported carrier or difference factor family present";
            if (phase138_requested) {
                result.diagnostics.phase138_configuration_valid = false;
                result.diagnostics.phase138_configuration_failure =
                    result.diagnostics.phase135_configuration_failure;
                result.diagnostics.phase138_factor_count_unchanged = false;
            }
            return result;
        }
    }
    if (use_native_direct_wls_ephemeral_main_seed) {
        const bool direct_recipe =
            use_imu && config.use_native_source_clock_c0d_factor &&
            config.use_native_source_clock_c0d_meter_state_parity &&
            config.use_native_source_clock_c0d_epoch_vector_parity &&
            config.use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver &&
            config.use_upstream_observable_quality &&
            config.use_undifferenced_doppler_factors &&
            config.use_doppler_velocity_wls_initialization &&
            !config.use_native_source_clock_c0d_gnss_first_meter_state_handoff &&
            !config.use_native_source_clock_c0d_raw_drift_d_initializer &&
            !config.use_native_pdc_state_bridge &&
            !config.use_inter_system_biases && !config.use_fixed_lag_smoother;
        if (!direct_recipe) {
            throw std::invalid_argument(
                "Phase114 direct-WLS ephemeral C7/D seed requires the "
                "raw-only Phase99 QR+C0/D Pose3+IMU recipe");
        }
        if (problem.native_direct_wls_ephemeral_velocity_ecef_mps.size() !=
                num_epochs ||
            problem.native_direct_wls_ephemeral_d_handoff_mps.size() !=
                num_epochs ||
            problem.native_direct_wls_ephemeral_c_handoff_m.size() !=
                num_epochs ||
            !std::all_of(
                problem.native_direct_wls_ephemeral_velocity_ecef_mps.begin(),
                problem.native_direct_wls_ephemeral_velocity_ecef_mps.end(),
                [](const Vector3d& value) { return value.allFinite(); }) ||
            !std::all_of(
                problem.native_direct_wls_ephemeral_d_handoff_mps.begin(),
                problem.native_direct_wls_ephemeral_d_handoff_mps.end(),
                [](double value) { return std::isfinite(value); }) ||
            !std::all_of(
                problem.native_direct_wls_ephemeral_c_handoff_m.begin(),
                problem.native_direct_wls_ephemeral_c_handoff_m.end(),
                [](const FGOProcessor::EpochClockBiasComponentsM& value) {
                    return std::all_of(value.begin(), value.end(),
                                       [](double component) {
                                           return std::isfinite(component);
                                       });
                })) {
            // The app-side adapter has the source-key and raw lineage checks;
            // this backend check prevents a direct library caller from
            // silently falling back to raw/zero/global state initialization.
            result.diagnostics.converged = false;
            return result;
        }
    }
    // Milestone 2c: dispatch to the incremental fixed-lag smoother when
    // requested and the IMU-coupled Pose3 path is fully specified. Otherwise
    // fall through to the batch LM path below (Phase-1 / 2a / 2b), unchanged.
    if (config.use_fixed_lag_smoother && config.use_pose3_state && config.use_imu &&
        problem.imu.valid && problem.epochs.size() >= 2) {
        return optimizeProblemFixedLag(problem, config, std::move(result));
    }
    const auto start_time = std::chrono::high_resolution_clock::now();

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // --- Milestone 2a/2b: Pose3 + lever-arm state (docs/gtsam_backend_design.md) ---
    // use_pose3 selects whether positionKey(epoch) holds a gtsam::Pose3 (body
    // pose, antenna offset by pose3_lever_arm_body_m) or the Phase-1
    // gtsam::Point3 (bare antenna position).
    //
    // use_imu (2b) additionally: interprets the Pose3 as body-in-nav (local
    // ENU) via a real ecef_T_nav, adds Vector3 velocity + imuBias states per
    // epoch, and links consecutive epochs with CombinedImuFactors. In 2a
    // (use_pose3 without IMU) the pose is expressed directly in ECEF (no
    // ecef_T_nav) and only the rotation gauge-pin constrains attitude.
    std::vector<double> raw_drift_d_initializer;
    if (config.use_native_source_clock_c0d_raw_drift_d_initializer) {
        result.diagnostics
            .native_source_clock_c0d_raw_drift_d_initializer_attempted = true;
        std::vector<double> raw_drift_mps;
        raw_drift_mps.reserve(problem.epochs.size());
        for (const auto& epoch : problem.epochs) {
            raw_drift_mps.push_back(epoch.receiver_clock_drift_mps);
        }
        source_clock_c0d::RawDriftDInitializationReport drift_report;
        if (!source_clock_c0d::validateAndCopyRawDriftD(
                raw_drift_mps, raw_drift_d_initializer, drift_report)) {
            result.diagnostics
                .native_source_clock_c0d_raw_drift_d_initializer_epoch_count =
                drift_report.epoch_count;
            result.diagnostics
                .native_source_clock_c0d_raw_drift_d_initializer_finite_count =
                drift_report.finite_count;
            result.diagnostics
                .native_source_clock_c0d_raw_drift_d_initializer_nonfinite_count =
                drift_report.nonfinite_count;
            result.diagnostics
                .native_source_clock_c0d_raw_drift_d_initializer_min_mps =
                drift_report.min_mps;
            result.diagnostics
                .native_source_clock_c0d_raw_drift_d_initializer_max_mps =
                drift_report.max_mps;
            result.diagnostics
                .native_source_clock_c0d_raw_drift_d_initializer_failure =
                drift_report.failure;
            result.diagnostics.converged = false;
            return result;
        }
        result.diagnostics
            .native_source_clock_c0d_raw_drift_d_initializer_coverage_valid =
            true;
        result.diagnostics
            .native_source_clock_c0d_raw_drift_d_initializer_epoch_count =
            drift_report.epoch_count;
        result.diagnostics
            .native_source_clock_c0d_raw_drift_d_initializer_finite_count =
            drift_report.finite_count;
        result.diagnostics
            .native_source_clock_c0d_raw_drift_d_initializer_nonfinite_count =
            drift_report.nonfinite_count;
        result.diagnostics
            .native_source_clock_c0d_raw_drift_d_initializer_min_mps =
            drift_report.min_mps;
        result.diagnostics
            .native_source_clock_c0d_raw_drift_d_initializer_max_mps =
            drift_report.max_mps;
    }
    const bool use_upstream_stop_constraints =
        use_imu && config.use_upstream_stop_constraints;
    upstream_stop::Detection upstream_stop_detection;
    if (use_upstream_stop_constraints) {
        std::vector<GNSSTime> epoch_times;
        epoch_times.reserve(problem.epochs.size());
        for (const auto& epoch : problem.epochs) epoch_times.push_back(epoch.time);
        upstream_stop::Config stop_config;
        stop_config.window_samples = static_cast<std::size_t>(
            std::max(2, config.upstream_stop_window_samples));
        stop_config.acceleration_std_offset_mps2 =
            config.upstream_stop_acceleration_std_offset_mps2;
        stop_config.gyro_std_offset_radps =
            config.upstream_stop_gyro_std_offset_radps;
        stop_config.gyro_norm_max_radps = config.upstream_stop_gyro_norm_max_radps;
        upstream_stop_detection = upstream_stop::detect(
            problem.imu.samples_body_flu, epoch_times, stop_config);
        if (!upstream_stop_detection.ok ||
            upstream_stop_detection.epoch_stop.size() != num_epochs) {
            // A malformed or incomplete raw stop stream must not turn into a
            // silently weakened graph.  Returning an unsolved result lets the
            // raw-only caller fail closed rather than selecting a fallback lane.
            result.diagnostics.upstream_stop_imu_samples =
                upstream_stop_detection.finite_samples;
            result.diagnostics.converged = false;
            return result;
        }
        result.diagnostics.upstream_stop_epochs =
            upstream_stop_detection.stop_epochs;
        result.diagnostics.upstream_stop_imu_samples =
            upstream_stop_detection.finite_samples;
        result.diagnostics.upstream_stop_acceleration_std_threshold_mps2 =
            upstream_stop_detection.acceleration_std_threshold_mps2;
        result.diagnostics.upstream_stop_gyro_std_threshold_radps =
            upstream_stop_detection.gyro_std_threshold_radps;
    }
    // The upstream GNSS-first pass uses explicit ENU velocity and receiver
    // clock-range-rate states.  Keep this path separate from the Pose3 IMU
    // velocity states so the established IMU graph and all production
    // defaults remain unchanged.
    const bool use_residual_ionosphere =
        config.use_residual_ionosphere_states && !problem.epochs.empty();
    double gnss_velocity_origin_lat_rad = 0.0;
    double gnss_velocity_origin_lon_rad = 0.0;
    // The opt-in raw-observable quality candidate uses the same receiver-only
    // Doppler rows in the IMU graph.  Its velocity state is already in the
    // IMU's ENU frame, so use that exact nav origin for the ECEF->ENU LOS
    // conversion.  Legacy IMU graphs keep this path disabled.
    const bool use_imu_doppler_factors =
        use_imu && config.use_upstream_observable_quality &&
        config.use_undifferenced_doppler_factors &&
        !problem.undifferenced_doppler_factors.empty();
    const bool use_native_source_clock_c0d_factor =
        config.use_native_source_clock_c0d_factor &&
        (use_imu || use_native_source_clock_c0d_gnss_first ||
         use_native_raw_p_seed_graph);
    const bool use_native_source_clock_c0d_meter_state =
        config.use_native_source_clock_c0d_meter_state_parity &&
        use_native_source_clock_c0d_factor;
    const bool use_native_source_clock_epoch_vector =
        config.use_native_source_clock_c0d_epoch_vector_parity &&
        use_native_source_clock_c0d_meter_state;
    result.diagnostics.native_source_clock_c0d_meter_state_parity_enabled =
        use_native_source_clock_c0d_meter_state;
    result.diagnostics.native_source_clock_c0d_epoch_vector_parity_enabled =
        use_native_source_clock_epoch_vector;
    result.diagnostics.native_source_clock_c0d_epoch_vector_dimension =
        use_native_source_clock_epoch_vector
            ? kNativeSourceClockVectorDimension
            : 0U;
    result.diagnostics.native_source_clock_c0d_factor_enabled =
        use_native_source_clock_c0d_factor;
    const bool has_epoch_local_c_handoff_source =
        config.use_native_source_clock_c0d_gnss_first_meter_state_handoff ||
        use_native_direct_wls_ephemeral_main_seed ||
        use_native_raw_p_seed_graph;
    if (config.use_native_source_clock_c0d_epoch_vector_parity &&
        (!use_native_source_clock_c0d_factor ||
         !use_native_source_clock_c0d_meter_state ||
         !has_epoch_local_c_handoff_source ||
         config.use_inter_system_biases ||
         config.use_receiver_signal_bias_states ||
         config.use_residual_ionosphere_states)) {
        throw std::invalid_argument(
            "Phase101/114 epoch-local C-vector parity requires the metre "
            "C0/D handoff graph with global ISB, signal-bias, and "
            "residual-ionosphere states disabled");
    }
    if (use_native_source_clock_epoch_vector) {
        // The source helper returns sentinel 7 for unsupported systems and
        // frequencies.  Reject the whole candidate before graph insertion so
        // one unsupported row cannot silently fall back to the legacy global
        // ISB/scalar-C factor family.
        for (const auto& factor : problem.pseudorange_factors) {
            if (sourceClockComponentFor(factor.satellite.system,
                                        factor.signal) < 0) {
                result.diagnostics.converged = false;
                return result;
            }
        }
        // Phase101 freezes the source XC/CCDD/TDCP topology only.  A caller
        // presenting undifferenced carrier or DD rows would require an
        // additional source vector factor family; reject it rather than bind
        // those rows to the scalar clock key.
        if (!problem.carrier_phase_factors.empty() ||
            !problem.double_difference_pseudorange_factors.empty() ||
            !problem.double_difference_carrier_factors.empty()) {
            result.diagnostics.converged = false;
            return result;
        }
    }
    if (use_gnss_velocity_states) {
        double origin_height = 0.0;
        const Vector3d origin_seed =
            use_native_raw_p_seed_graph &&
                    !problem.native_raw_p_no_doppler_seeds.empty()
                ? problem.native_raw_p_no_doppler_seeds.front().position_ecef
                : problem.epochs.front().position_ecef;
        ecef2geodetic(origin_seed,
                      gnss_velocity_origin_lat_rad,
                      gnss_velocity_origin_lon_rad,
                      origin_height);
        if (!std::isfinite(gnss_velocity_origin_lat_rad) ||
            !std::isfinite(gnss_velocity_origin_lon_rad)) {
            return result;
        }
    } else if (use_imu_doppler_factors) {
        gnss_velocity_origin_lat_rad = problem.imu.nav_origin_lat_rad;
        gnss_velocity_origin_lon_rad = problem.imu.nav_origin_lon_rad;
        if (!std::isfinite(gnss_velocity_origin_lat_rad) ||
            !std::isfinite(gnss_velocity_origin_lon_rad)) {
            return result;
        }
    }
    const Point3 lever_arm_body(config.pose3_lever_arm_body_m.x(),
                                config.pose3_lever_arm_body_m.y(),
                                config.pose3_lever_arm_body_m.z());

    // ecef_T_nav: identity/unused in 2a (pose in ECEF); the ENU-from-ECEF
    // transform at the nav origin in 2b (pose in nav). The DD '...FactorArm'
    // factors and the IMU factor share this same Pose3 through it.
    gtsam::gnss::LeverArm gnss_lever_arm(lever_arm_body);
    Pose3 ecef_T_nav;  // identity unless use_imu
    if (use_imu) {
        const Rot3 R_ecef_nav = ecefFromEnuRotation(problem.imu.nav_origin_lat_rad,
                                                    problem.imu.nav_origin_lon_rad);
        ecef_T_nav = Pose3(R_ecef_nav, Point3(problem.imu.nav_origin_ecef));
        gnss_lever_arm = gtsam::gnss::LeverArm(lever_arm_body, ecef_T_nav);
    }

    // The native PDC bridge is solved in this process from the same raw P/D
    // rows that are added below.  Use its finite state only as an initializer;
    // never add a second PDC prior/factor for those already-owned rows.
    auto nativePdcSeedFor = [&](std::size_t epoch)
        -> const FGOProcessor::NativePdcStateSeed* {
        if (!config.use_native_pdc_state_bridge) return nullptr;
        for (const auto& seed : problem.native_pdc_state_seeds) {
            if (seed.epoch_index == epoch) return &seed;
        }
        return nullptr;
    };
    auto nativeRawPNoDopplerSeedFor = [&](std::size_t epoch)
        -> const raw_p_seed::RawPNoDopplerSeed* {
        if (!use_native_raw_p_seed_graph ||
            epoch >= problem.native_raw_p_no_doppler_seeds.size()) {
            return nullptr;
        }
        const auto& seed = problem.native_raw_p_no_doppler_seeds[epoch];
        return seed.epoch_index == epoch ? &seed : nullptr;
    };
    auto positionSeedEcef = [&](std::size_t epoch) -> Vector3d {
        const auto* raw_seed = nativeRawPNoDopplerSeedFor(epoch);
        if (raw_seed != nullptr && raw_seed->has_position &&
            raw_seed->position_ecef.allFinite()) {
            return raw_seed->position_ecef;
        }
        const auto* seed = nativePdcSeedFor(epoch);
        if (seed != nullptr && seed->has_position && seed->position_ecef.allFinite()) {
            return seed->position_ecef;
        }
        return problem.epochs[epoch].position_ecef;
    };

    // Validate failures are returned through the same transaction boundary
    // used by Phase135.  Phase138 adds no fallback: a bad range endpoint or
    // adjusted measurement invalidates the complete affine family.
    auto failPhase135 = [&](const std::string& reason) {
        result.diagnostics.phase135_configuration_valid = false;
        result.diagnostics.phase135_configuration_failure = reason;
        if (phase138_requested) {
            result.diagnostics.phase138_configuration_valid = false;
            result.diagnostics.phase138_configuration_failure = reason;
            result.diagnostics.phase138_factor_count_unchanged = false;
        }
        result.diagnostics.converged = false;
        return result;
    };

    if (phase135_requested) {
        // Validate every source geometry and retained row before constructing
        // any candidate factor.  This is the transaction boundary: a missing
        // source transmit state, unsupported C7 component, nonfinite value,
        // or malformed endpoint returns an unsolved result and never permits
        // a mixed native/affine graph to reach LM.
        if (problem.pseudorange_factors.empty() ||
            problem.undifferenced_doppler_factors.empty() ||
            problem.tdcp_factors.empty()) {
            return failPhase135(
                "Phase135 requires nonempty pseudorange, Doppler, and TDCP families");
        }
        for (const auto& factor : problem.pseudorange_factors) {
            if (factor.epoch_index >= num_epochs ||
                !factor.source_satellite_position_available ||
                !factor.source_satellite_position_ecef.allFinite() ||
                !std::isfinite(factor.corrected_pseudorange_m) ||
                !std::isfinite(factor.sigma_m) || factor.sigma_m <= 0.0 ||
                sourceClockComponentFor(factor.satellite.system,
                                        factor.signal) < 0) {
                return failPhase135("pseudorange source row validation failed");
            }
            Phase135SourceGeometry geometry;
            if (!phase135SourceGeodist(
                    Point3(factor.source_satellite_position_ecef),
                    Point3(positionSeedEcef(factor.epoch_index)), geometry)) {
                return failPhase135("pseudorange source Sagnac geometry failed");
            }
            ++result.diagnostics.phase135_geometry_rows_validated;
        }
        for (const auto& factor : problem.undifferenced_doppler_factors) {
            if (factor.epoch_index >= num_epochs ||
                !factor.source_satellite_state_available ||
                !factor.source_satellite_position_ecef.allFinite() ||
                !factor.source_satellite_velocity_ecef.allFinite() ||
                !std::isfinite(factor.residual_mps) ||
                !std::isfinite(factor.measured_range_rate_mps) ||
                !std::isfinite(factor.satellite_clock_drift_mps) ||
                !std::isfinite(factor.sigma_mps) || factor.sigma_mps <= 0.0) {
                return failPhase135("Doppler source row validation failed");
            }
            Phase135SourceGeometry geometry;
            if (!phase135SourceGeodist(
                    Point3(factor.source_satellite_position_ecef),
                    Point3(positionSeedEcef(factor.epoch_index)), geometry)) {
                return failPhase135("Doppler source Sagnac geometry failed");
            }
            ++result.diagnostics.phase135_geometry_rows_validated;
        }
        for (const auto& factor : problem.tdcp_factors) {
            if (factor.previous_epoch_index >= num_epochs ||
                factor.current_epoch_index >= num_epochs ||
                factor.current_epoch_index <= factor.previous_epoch_index ||
                !factor.source_satellite_positions_available ||
                !factor.previous_source_satellite_position_ecef.allFinite() ||
                !factor.current_source_satellite_position_ecef.allFinite() ||
                !std::isfinite(factor.delta_carrier_m) ||
                !std::isfinite(factor.sigma_m) || factor.sigma_m <= 0.0) {
                return failPhase135("TDCP source row validation failed");
            }
            Phase135SourceGeometry geometry;
            if (!phase135SourceGeodist(
                    Point3(factor.previous_source_satellite_position_ecef),
                    Point3(positionSeedEcef(factor.previous_epoch_index)), geometry)) {
                return failPhase135("TDCP source Sagnac geometry failed");
            }
            // TDCPFactor_XXCC linearizes with the previous endpoint LOS, but
            // the pair still requires a finite, source-certified current
            // endpoint.  Validate it before any candidate factor is exposed
            // so a malformed endpoint cannot become a partial affine row.
            Phase135SourceGeometry current_geometry;
            if (!phase135SourceGeodist(
                    Point3(factor.current_source_satellite_position_ecef),
                    Point3(positionSeedEcef(factor.current_epoch_index)),
                    current_geometry)) {
                return failPhase135("TDCP current source Sagnac geometry failed");
            }
            ++result.diagnostics.phase135_geometry_rows_validated;
        }
        result.diagnostics.phase135_configuration_valid = true;
    }

    // Initial body->nav (ENU) attitude from Stage-1 alignment; used as the
    // dead-reckoning start point for the 2b pose attitude seeds below.
    const Rot3 init_attitude_nav(problem.imu.init_attitude_body_to_nav);

    // Antenna ECEF position for the optimized state at `epoch`, uniform across
    // all three representations (Point3, 2a Pose3-in-ECEF, 2b Pose3-in-nav):
    // gnss::LeverArm::antennaPosition applies ecef_T_nav internally when set.
    // Used everywhere downstream that needs "the position": LAMBDA covariance
    // propagation, residual RMS diagnostics, final per-epoch solution mapping.
    auto antennaPositionOf = [&](const gtsam::Values& values,
                                 std::size_t epoch) -> Point3 {
        if (phase135_requested && use_imu) {
            const gtsam::Vector x =
                values.at<gtsam::Vector>(affinePositionKey(epoch));
            if (x.size() != 3 || !x.allFinite()) {
                throw std::runtime_error(
                    "Phase135 auxiliary X state is missing or nonfinite");
            }
            const Vector3d x_nav(x(0), x(1), x(2));
            return Point3(problem.imu.nav_origin_ecef + enu2ecef(
                x_nav, problem.imu.nav_origin_lat_rad,
                problem.imu.nav_origin_lon_rad));
        }
        if (use_pose3) {
            return gnss_lever_arm.antennaPosition(values.at<Pose3>(positionKey(epoch)));
        }
        return values.at<Point3>(positionKey(epoch));
    };

    // Pose seeds (Point3 and 2a Pose3-in-ECEF; the 2b IMU poses are seeded
    // below, after preintegration, so their attitudes can be dead-reckoned).
    for (std::size_t i = 0; i < num_epochs; ++i) {
        if (use_imu) {
            continue;
        }
        if (use_pose3) {
            // 2a: pose in ECEF, identity attitude (unobservable without IMU);
            // translation = antenna seed minus lever arm.
            initial.insert(positionKey(i),
                           Pose3(Rot3(), Point3(positionSeedEcef(i)) - lever_arm_body));
        } else {
            initial.insert(positionKey(i), Point3(positionSeedEcef(i)));
        }
    }

    if (use_gnss_velocity_states) {
        // Materialize the Eigen expression before putting it in Values.  A
        // direct Vector3::Zero() insert is an expression-template type and
        // is not a concrete GTSAM value on all supported Eigen versions.
        const gtsam::Vector3 zero_velocity = gtsam::Vector3::Zero();
        auto insertClockDrift = [&](std::size_t epoch, double drift) {
            if (use_native_source_clock_epoch_vector) {
                const gtsam::Vector drift_state =
                    gtsam::Vector::Constant(1, drift);
                initial.insert(dopplerClockDriftKey(epoch), drift_state);
            } else {
                initial.insert(dopplerClockDriftKey(epoch), drift);
            }
        };
        for (std::size_t i = 0; i < num_epochs; ++i) {
            const auto* raw_seed = nativeRawPNoDopplerSeedFor(i);
            if (raw_seed != nullptr && raw_seed->has_velocity &&
                raw_seed->velocity_ecef_mps.allFinite() &&
                raw_seed->has_clock_rate &&
                std::isfinite(raw_seed->clock_rate_mps)) {
                const Vector3d velocity_nav = ecef2enu(
                    raw_seed->velocity_ecef_mps,
                    gnss_velocity_origin_lat_rad,
                    gnss_velocity_origin_lon_rad);
                // Legacy Doppler factors use an ENU velocity state.  The
                // dedicated XXVV factor couples ECEF Point3 positions, so
                // its same-run raw-P velocity state must stay in ECEF.  Keep
                // the conversion above as a finite-frame sanity check but do
                // not insert the ENU representation into this graph.
                if (!velocity_nav.allFinite() ||
                    !raw_seed->velocity_ecef_mps.allFinite()) {
                    result.diagnostics.converged = false;
                    return result;
                }
                const gtsam::Vector3 velocity_ecef(
                    raw_seed->velocity_ecef_mps);
                initial.insert(velocityKey(i), velocity_ecef);
                insertClockDrift(i, raw_seed->clock_rate_mps);
                continue;
            }
            if (use_native_source_clock_c0d_gnss_first &&
                config.use_native_source_clock_c0d_raw_drift_d_initializer) {
                // Phase93 owns the GNSS-first D_i initializer.  Insert it in
                // this Point3/velocity loop so the later IMU-only D loop does
                // not attempt to insert the same GTSAM key twice.
                initial.insert(velocityKey(i), zero_velocity);
                insertClockDrift(i, raw_drift_d_initializer[i]);
                continue;
            }
            const auto* pdc_seed = nativePdcSeedFor(i);
            if (pdc_seed != nullptr && pdc_seed->has_velocity &&
                pdc_seed->velocity_ecef_mps.allFinite() &&
                pdc_seed->has_clock_rate &&
                std::isfinite(pdc_seed->clock_rate_mps)) {
                const Vector3d velocity_nav = ecef2enu(
                    pdc_seed->velocity_ecef_mps,
                    gnss_velocity_origin_lat_rad,
                    gnss_velocity_origin_lon_rad);
                if (velocity_nav.allFinite()) {
                    initial.insert(velocityKey(i), gtsam::Vector3(velocity_nav));
                    insertClockDrift(i, pdc_seed->clock_rate_mps);
                    continue;
                }
            }
            if (config.use_doppler_velocity_wls_initialization &&
                i < problem.doppler_velocity_wls_estimates.size() &&
                problem.doppler_velocity_wls_estimates[i].valid) {
                const auto& estimate = problem.doppler_velocity_wls_estimates[i];
                const Vector3d velocity_nav = ecef2enu(
                    estimate.velocity_ecef_mps,
                    gnss_velocity_origin_lat_rad,
                    gnss_velocity_origin_lon_rad);
                if (velocity_nav.allFinite() &&
                    std::isfinite(estimate.clock_rate_mps)) {
                    initial.insert(velocityKey(i), gtsam::Vector3(velocity_nav));
                    insertClockDrift(i, estimate.clock_rate_mps);
                    continue;
                }
            }
            initial.insert(velocityKey(i), zero_velocity);
            insertClockDrift(i, 0.0);
        }
    }

    if (use_imu_doppler_factors ||
        (use_native_source_clock_c0d_factor && !use_gnss_velocity_states)) {
        // IMU velocity states are initialized above from the IMU/GNSS-first
        // handoff.  Receiver clock drift is a separate metre-per-second state;
        // a broad prior below prevents a sparse Doppler set from becoming an
        // unconstrained gauge while preserving the raw D measurement.
        for (std::size_t i = 0; i < num_epochs; ++i) {
            double clock_rate = 0.0;
            if (use_native_direct_wls_ephemeral_main_seed) {
                // Phase114 direct main seed is the only source for D_i in
                // this graph.  The app-side adapter has already proven exact
                // raw-key equality and full finite coverage; do not use raw,
                // zero, WLS, or interpolated fallback here.
                clock_rate =
                    problem.native_direct_wls_ephemeral_d_handoff_mps[i];
            } else if (config.use_native_source_clock_c0d_gnss_first_meter_state_handoff &&
                use_imu) {
                // Phase93 main-graph initialization is deliberately sourced
                // only from the optimized same-run GNSS-first D export.  The
                // exact size/finiteness contract was checked before graph
                // construction above, so no raw/zero/WLS fallback can occur.
                clock_rate =
                    problem.native_source_clock_c0d_gnss_first_d_handoff_mps[i];
            } else if (config.use_native_source_clock_c0d_raw_drift_d_initializer) {
                // The complete finite sequence was validated above. This is
                // the only Phase91 state change: main-graph D_i receives the
                // retained raw Android drift in m/s, without a PDC bridge.
                clock_rate = raw_drift_d_initializer[i];
            } else {
                const auto* pdc_seed = nativePdcSeedFor(i);
                clock_rate =
                    pdc_seed != nullptr && pdc_seed->has_clock_rate &&
                            std::isfinite(pdc_seed->clock_rate_mps)
                        ? pdc_seed->clock_rate_mps
                        : 0.0;
            }
            if (use_native_source_clock_epoch_vector) {
                const gtsam::Vector drift_state =
                    gtsam::Vector::Constant(1, clock_rate);
                initial.insert(dopplerClockDriftKey(i), drift_state);
            } else {
                initial.insert(dopplerClockDriftKey(i), clock_rate);
            }
        }
    }

    if (use_imu) {
        const gtsam::imuBias::ConstantBias init_bias(problem.imu.init_accel_bias,
                                                     problem.imu.init_gyro_bias);

        // --- IMU preintegration params (ENU / Z-up; gravity along -Z) ---
        auto imu_params = gtsam::PreintegrationCombinedParams::MakeSharedU(
            problem.imu.noise.gravity_mps2);
        const auto sq = [](double s) { return s * s; };
        imu_params->setAccelerometerCovariance(gtsam::I_3x3 * sq(problem.imu.noise.accel_noise_sigma));
        imu_params->setGyroscopeCovariance(gtsam::I_3x3 * sq(problem.imu.noise.gyro_noise_sigma));
        imu_params->setIntegrationCovariance(gtsam::I_3x3 * sq(problem.imu.noise.integration_sigma));
        imu_params->setBiasAccCovariance(gtsam::I_3x3 * sq(problem.imu.noise.accel_bias_rw_sigma));
        imu_params->setBiasOmegaCovariance(gtsam::I_3x3 * sq(problem.imu.noise.gyro_bias_rw_sigma));

        // --- Preintegrate each [epoch i, epoch i+1) interval up front ---
        // We need the preintegrated rotations to dead-reckon attitude seeds
        // BEFORE inserting the pose values: seeding every pose at the same
        // initial attitude makes each IMU factor's rotation residual the full
        // accumulated turn between epochs, which gives a huge initial cost and
        // a divergent first LM step (observed: 1e19 cost, no convergence). By
        // forward-propagating attitude through deltaRij() the rotation residual
        // starts ~0 and the DD-accurate translation keeps position anchored, so
        // the graph is well conditioned from iteration 0.
        const auto& imu_samples = problem.imu.samples_body_flu;
        if (phase205_requested || phase209_requested) {
            for (std::size_t k = 0; k < imu_samples.size(); ++k) {
                if (!std::isfinite(imu_samples[k].time.tow) ||
                    !imu_samples[k].accel_raw.allFinite() ||
                    !imu_samples[k].gyro_raw_radps.allFinite() ||
                    (k > 0 && !(imu_samples[k - 1].time < imu_samples[k].time))) {
                    throw std::invalid_argument("Phase205/209 requires finite ordered IMU samples");
                }
            }
        }
        std::vector<gtsam::PreintegratedCombinedMeasurements> pims;
        // Standard preintegration has its own 9x9 covariance propagation.
        // Do not approximate it by slicing the Combined covariance. Existing
        // Combined preintegrations remain only to preserve attitude seeding.
        std::shared_ptr<gtsam::PreintegrationParams> phase209_params;
        std::vector<gtsam::PreintegratedImuMeasurements> phase209_pims;
        std::vector<std::size_t> phase209_counts;
        if (phase209_requested) {
            phase209_params = gtsam::PreintegrationParams::MakeSharedU(problem.imu.noise.gravity_mps2);
            phase209_params->setAccelerometerCovariance(imu_params->getAccelerometerCovariance());
            phase209_params->setGyroscopeCovariance(imu_params->getGyroscopeCovariance());
            phase209_params->setIntegrationCovariance(imu_params->getIntegrationCovariance());
            for (const double sigma : {problem.imu.noise.accel_bias_rw_sigma,
                                       problem.imu.noise.gyro_bias_rw_sigma}) {
                if (!std::isfinite(sigma) || sigma <= 0.0) {
                    throw std::invalid_argument("Phase209 requires positive finite bias sigmas");
                }
            }
        }
        pims.reserve(num_epochs > 0 ? num_epochs - 1 : 0);
        std::vector<bool> pim_valid(num_epochs > 0 ? num_epochs - 1 : 0, false);
        std::size_t sample_cursor = 0;
        std::size_t phase201_sample_cursor = 0;
        bool phase201_stream_validated = false;
        std::size_t imu_intervals = 0;
        for (std::size_t i = 0; i + 1 < num_epochs; ++i) {
            const GNSSTime t0 = problem.epochs[i].time;
            const GNSSTime t1 = problem.epochs[i + 1].time;
            if (phase201_requested) {
                const auto schedule =
                    makeSourceInclusiveForwardImuSchedule(
                        imu_samples, t0, t1, phase201_sample_cursor,
                        !phase201_stream_validated);
                auto& phase201 = result.diagnostics;
                ++phase201.native_phase201_intervals;
                phase201.native_phase201_invalid_sample_count +=
                    schedule.invalid_sample_count;
                phase201.native_phase201_nonfinite_dt_count +=
                    schedule.nonfinite_dt_count;
                phase201.native_phase201_nonpositive_dt_count +=
                    schedule.nonpositive_dt_count;
                if (schedule.empty_interval) {
                    ++phase201.native_phase201_empty_interval_count;
                }
                if (!schedule.valid) {
                    phase201
                        .native_phase201_source_inclusive_forward_imu_schedule_configuration_valid =
                        false;
                    phase201.native_phase201_configuration_failure =
                        "interval " + std::to_string(i) + ": " + schedule.failure;
                    phase201.converged = false;
                    return result;
                }
                phase201_stream_validated = true;
                phase201_sample_cursor = schedule.next_sample_index;

                const double gnss_interval_s = t1 - t0;
                const double integrated_duration_s =
                    schedule.integrated_duration_s;
                const double duration_error_s =
                    integrated_duration_s - gnss_interval_s;
                ++phase201.native_phase201_intervals_with_samples;
                phase201.native_phase201_inserted_samples +=
                    schedule.segments.size();
                if (phase201.native_phase201_intervals_with_samples == 1U) {
                    phase201.native_phase201_integrated_duration_min_s =
                        integrated_duration_s;
                    phase201.native_phase201_integrated_duration_max_s =
                        integrated_duration_s;
                    phase201.native_phase201_gnss_interval_duration_min_s =
                        gnss_interval_s;
                    phase201.native_phase201_gnss_interval_duration_max_s =
                        gnss_interval_s;
                    phase201.native_phase201_duration_error_min_s =
                        duration_error_s;
                    phase201.native_phase201_duration_error_max_s =
                        duration_error_s;
                    phase201.native_phase201_duration_error_max_abs_s =
                        std::abs(duration_error_s);
                } else {
                    phase201.native_phase201_integrated_duration_min_s =
                        std::min(phase201.native_phase201_integrated_duration_min_s,
                                 integrated_duration_s);
                    phase201.native_phase201_integrated_duration_max_s =
                        std::max(phase201.native_phase201_integrated_duration_max_s,
                                 integrated_duration_s);
                    phase201.native_phase201_gnss_interval_duration_min_s =
                        std::min(phase201.native_phase201_gnss_interval_duration_min_s,
                                 gnss_interval_s);
                    phase201.native_phase201_gnss_interval_duration_max_s =
                        std::max(phase201.native_phase201_gnss_interval_duration_max_s,
                                 gnss_interval_s);
                    phase201.native_phase201_duration_error_min_s =
                        std::min(phase201.native_phase201_duration_error_min_s,
                                 duration_error_s);
                    phase201.native_phase201_duration_error_max_s =
                        std::max(phase201.native_phase201_duration_error_max_s,
                                 duration_error_s);
                    phase201.native_phase201_duration_error_max_abs_s =
                        std::max(phase201.native_phase201_duration_error_max_abs_s,
                                 std::abs(duration_error_s));
                }

                gtsam::PreintegratedCombinedMeasurements pim(imu_params, init_bias);
                for (const auto& segment : schedule.segments) {
                    const auto& sample = imu_samples[segment.sample_index];
                    pim.integrateMeasurement(
                        gtsam::Vector3(sample.accel_raw),
                        gtsam::Vector3(sample.gyro_raw_radps), segment.dt_s);
                }
                pims.push_back(pim);
                pim_valid[i] = true;
                ++imu_intervals;
                continue;
            }
            while (sample_cursor < imu_samples.size() && imu_samples[sample_cursor].time < t0) {
                ++sample_cursor;
            }
            auto interval_params = imu_params;
            if (phase209_requested) {
                const double duration = t1 - t0;
                if (!std::isfinite(duration) || duration <= 0.0 || duration >= 1.5) {
                    throw std::invalid_argument("Phase209 requires intervals in (0, 1.5) seconds");
                }
                std::size_t count = 0;
                for (std::size_t k = sample_cursor; k < imu_samples.size() &&
                     !(t1 < imu_samples[k].time); ++k) ++count;
                if (count == 0) throw std::invalid_argument("Phase209 empty IMU interval");
                phase209_counts.push_back(count);
                phase209_pims.emplace_back(phase209_params, init_bias);
                result.diagnostics.native_phase209_inclusive_samples += count;
            }
            double phase205_duration = 0.0;
            if (phase205_requested) {
                // Count exact inclusive samples as in the source, but retain
                // native preceding-delta/tail measurement selection below.
                std::size_t count = 0;
                for (std::size_t k = sample_cursor; k < imu_samples.size() &&
                     !(t1 < imu_samples[k].time); ++k) ++count;
                GNSSTime previous = t0;
                std::size_t selected = 0;
                for (std::size_t k = sample_cursor; k < imu_samples.size() &&
                     imu_samples[k].time < t1; ++k) {
                    const double dt = imu_samples[k].time - previous;
                    if (dt > 1e-9) { phase205_duration += dt; ++selected; }
                    previous = imu_samples[k].time;
                }
                const double tail = t1 - previous;
                if (selected > 0 && tail > 1e-9) phase205_duration += tail;
                const double scale = native_imu_bias_density::sourceCountScale(
                    count, phase205_duration);
                interval_params = std::make_shared<gtsam::PreintegrationCombinedParams>(*imu_params);
                interval_params->setBiasAccCovariance(imu_params->getBiasAccCovariance() * scale);
                interval_params->setBiasOmegaCovariance(imu_params->getBiasOmegaCovariance() * scale);
                auto& d = result.diagnostics;
                if (d.native_phase205_bias_density_intervals++ == 0) {
                    d.native_phase205_bias_density_scale_min = scale;
                    d.native_phase205_bias_density_scale_max = scale;
                } else {
                    d.native_phase205_bias_density_scale_min = std::min(d.native_phase205_bias_density_scale_min, scale);
                    d.native_phase205_bias_density_scale_max = std::max(d.native_phase205_bias_density_scale_max, scale);
                }
                d.native_phase205_bias_density_samples += count;
            }
            gtsam::PreintegratedCombinedMeasurements pim(interval_params, init_bias);
            std::size_t j = sample_cursor;
            GNSSTime prev_time = t0;
            std::size_t integrated = 0;
            while (j < imu_samples.size() && imu_samples[j].time < t1) {
                const double dt = imu_samples[j].time - prev_time;
                if (dt > 1e-9) {
                    pim.integrateMeasurement(gtsam::Vector3(imu_samples[j].accel_raw),
                                             gtsam::Vector3(imu_samples[j].gyro_raw_radps), dt);
                    if (phase209_requested) {
                        phase209_pims.back().integrateMeasurement(
                            gtsam::Vector3(imu_samples[j].accel_raw),
                            gtsam::Vector3(imu_samples[j].gyro_raw_radps), dt);
                    }
                    ++integrated;
                }
                prev_time = imu_samples[j].time;
                ++j;
            }
            const double dt_tail = t1 - prev_time;
            if (integrated > 0 && dt_tail > 1e-9 && j > 0) {
                pim.integrateMeasurement(gtsam::Vector3(imu_samples[j - 1].accel_raw),
                                         gtsam::Vector3(imu_samples[j - 1].gyro_raw_radps), dt_tail);
                if (phase209_requested) {
                    phase209_pims.back().integrateMeasurement(
                        gtsam::Vector3(imu_samples[j - 1].accel_raw),
                        gtsam::Vector3(imu_samples[j - 1].gyro_raw_radps), dt_tail);
                }
            }
            pims.push_back(pim);
            pim_valid[i] = integrated > 0;
            if (phase209_requested && (integrated == 0 ||
                std::abs(phase209_pims.back().deltaTij() - pim.deltaTij()) > 1e-12 ||
                !phase209_pims.back().deltaRij().equals(pim.deltaRij(), 1e-12))) {
                throw std::runtime_error("Phase209 invalid integration or seed mismatch");
            }
            if (phase205_requested && std::abs(pim.deltaTij() - phase205_duration) > 1e-12) {
                throw std::runtime_error("Phase205 integration duration mismatch");
            }
            if (integrated > 0) ++imu_intervals;
        }

        // --- Dead-reckon per-epoch attitude seeds from the aligned initial
        // attitude (rotation residual ~0 at the seed). ---
        std::vector<Rot3> attitude_seed(num_epochs, init_attitude_nav);
        for (std::size_t i = 0; i + 1 < num_epochs; ++i) {
            attitude_seed[i + 1] =
                pim_valid[i] ? attitude_seed[i] * pims[i].deltaRij() : attitude_seed[i];
        }
        if (config.use_native_epoch_heading_attitude_seeds) {
            for (std::size_t i = 0; i < num_epochs; ++i) {
                attitude_seed[i] = Rot3(problem.imu.epoch_heading_attitudes_body_to_nav.at(i));
            }
        }

        // GNSS antenna positions in nav, for translation + velocity seeds.
        std::vector<Point3> antenna_nav(num_epochs);
        for (std::size_t i = 0; i < num_epochs; ++i) {
            antenna_nav[i] = Point3(ecef2enu(
                positionSeedEcef(i) - problem.imu.nav_origin_ecef,
                problem.imu.nav_origin_lat_rad, problem.imu.nav_origin_lon_rad));
        }

        // --- Insert pose / velocity / bias seeds ---
        for (std::size_t i = 0; i < num_epochs; ++i) {
            // body-in-nav translation: antenna at the DD position, offset back
            // through the (dead-reckoned) attitude's lever arm.
            const Point3 body_nav = antenna_nav[i] - attitude_seed[i] * lever_arm_body;
            initial.insert(positionKey(i), Pose3(attitude_seed[i], body_nav));
            if (config.use_native_epoch_heading_attitude_seeds) {
                ++result.diagnostics.epoch_heading_attitude_seeds_inserted;
            }
            if (phase135_requested) {
                // Official fgo_gnss_imu uses an independent ENU X vector
                // tied to the Pose3 translation by an exact PX factor.  The
                // frozen Pixel5 recipe has a zero lever arm; a non-zero arm
                // is rejected above rather than silently changing PX's
                // source equation.
                gtsam::Vector x_initial(3);
                x_initial << antenna_nav[i].x(), antenna_nav[i].y(),
                    antenna_nav[i].z();
                if (!x_initial.allFinite()) {
                    return failPhase135("nonfinite auxiliary X initial value");
                }
                initial.insert(affinePositionKey(i), x_initial);
                graph.emplace_shared<Phase135Pose3Point3FactorPX>(
                    positionKey(i), affinePositionKey(i),
                    gtsam::noiseModel::Constrained::All(3));
                ++result.diagnostics.phase135_pose3_x_bridge_factors;
            }

            Vector3d vel = problem.imu.init_velocity_nav;
            const auto* pdc_seed = nativePdcSeedFor(i);
            bool have_velocity_seed = false;
            if (use_native_direct_wls_ephemeral_main_seed) {
                const Vector3d& direct_velocity =
                    problem.native_direct_wls_ephemeral_velocity_ecef_mps[i];
                if (direct_velocity.allFinite()) {
                    const Vector3d direct_velocity_nav = ecef2enu(
                        direct_velocity, problem.imu.nav_origin_lat_rad,
                        problem.imu.nav_origin_lon_rad);
                    if (direct_velocity_nav.allFinite()) {
                        vel = direct_velocity_nav;
                        have_velocity_seed = true;
                    }
                }
                if (!have_velocity_seed) {
                    // A direct candidate never synthesizes velocity from
                    // position differences or zeros, even if a later graph
                    // seed branch would otherwise be available.
                    result.diagnostics.converged = false;
                    return result;
                }
            }
            if (!have_velocity_seed && pdc_seed != nullptr && pdc_seed->has_velocity &&
                pdc_seed->velocity_ecef_mps.allFinite()) {
                const Vector3d seed_velocity_nav = ecef2enu(
                    pdc_seed->velocity_ecef_mps,
                    problem.imu.nav_origin_lat_rad,
                    problem.imu.nav_origin_lon_rad);
                if (seed_velocity_nav.allFinite()) {
                    vel = seed_velocity_nav;
                    have_velocity_seed = true;
                }
            }
            if (!have_velocity_seed && !config.use_native_pdc_state_bridge &&
                config.use_doppler_velocity_wls_initialization &&
                i < problem.doppler_velocity_wls_estimates.size()) {
                const auto& estimate =
                    problem.doppler_velocity_wls_estimates[i];
                if (estimate.valid && estimate.velocity_ecef_mps.allFinite()) {
                    const Vector3d seed_velocity_nav = ecef2enu(
                        estimate.velocity_ecef_mps,
                        problem.imu.nav_origin_lat_rad,
                        problem.imu.nav_origin_lon_rad);
                    if (seed_velocity_nav.allFinite()) {
                        // Phase40 direct handoff: use the bounded raw WLS
                        // estimate for every IMU velocity state, preserving
                        // velocity-only semantics (clock/position states are
                        // still initialized by their ordinary path).
                        vel = seed_velocity_nav;
                        have_velocity_seed = true;
                    }
                }
            }
            if (num_epochs >= 2) {
                const std::size_t a = (i + 1 < num_epochs) ? i : i - 1;
                const double dt = problem.epochs[a + 1].time - problem.epochs[a].time;
                if (!have_velocity_seed) {
                  if (dt > 1e-3) {
                    vel = (antenna_nav[a + 1] - antenna_nav[a]) / dt;
                  }
                }
            }
            initial.insert(velocityKey(i), gtsam::Vector3(vel));
            initial.insert(biasKey(i), init_bias);
        }

        // --- First-state priors (replace the 2a per-epoch rotation pin): they
        // anchor the IMU integration chain and resolve the gauge. Attitude is
        // now observable (gravity fixes roll/pitch; motion fixes yaw), so no
        // per-pose rotation pin is needed. Position stays fully DD-driven. ---
        gtsam::Vector6 pose_prior_sigmas;
        pose_prior_sigmas << problem.imu.init_attitude_sigma_roll_pitch_rad,
            problem.imu.init_attitude_sigma_roll_pitch_rad,
            problem.imu.init_attitude_sigma_yaw_rad,
            1e6, 1e6, 1e6;  // translation left free (DD constrains it)
        graph.addPrior(positionKey(0), initial.at<Pose3>(positionKey(0)),
                       gtsam::noiseModel::Diagonal::Sigmas(pose_prior_sigmas));
        if (config.omit_native_first_imu_velocity_prior) {
            ++result.diagnostics.first_imu_velocity_priors_omitted;
        } else {
            graph.addPrior(velocityKey(0), gtsam::Vector3(problem.imu.init_velocity_nav),
                           gtsam::noiseModel::Isotropic::Sigma(3, problem.imu.init_velocity_sigma_mps));
            ++result.diagnostics.first_imu_velocity_priors_inserted;
        }
        gtsam::Vector6 bias_prior_sigmas;
        bias_prior_sigmas << problem.imu.init_accel_bias_sigma, problem.imu.init_accel_bias_sigma,
            problem.imu.init_accel_bias_sigma, problem.imu.init_gyro_bias_sigma,
            problem.imu.init_gyro_bias_sigma, problem.imu.init_gyro_bias_sigma;
        if (config.omit_native_first_imu_bias_prior) {
            ++result.diagnostics.first_imu_bias_priors_omitted;
        } else {
            graph.addPrior(biasKey(0), init_bias,
                           gtsam::noiseModel::Diagonal::Sigmas(bias_prior_sigmas));
            ++result.diagnostics.first_imu_bias_priors_inserted;
        }

        // --- Add the CombinedImuFactors (or a loose velocity/bias continuity
        // across an IMU dropout) ---
        for (std::size_t i = 0; i + 1 < num_epochs; ++i) {
            if (pim_valid[i]) {
                if (phase209_requested) {
                    graph.emplace_shared<gtsam::ImuFactor>(
                        positionKey(i), velocityKey(i), positionKey(i + 1), velocityKey(i + 1),
                        biasKey(i + 1), phase209_pims.at(i));
                    gtsam::Vector6 sigmas;
                    sigmas << problem.imu.noise.accel_bias_rw_sigma,
                        problem.imu.noise.accel_bias_rw_sigma, problem.imu.noise.accel_bias_rw_sigma,
                        problem.imu.noise.gyro_bias_rw_sigma, problem.imu.noise.gyro_bias_rw_sigma,
                        problem.imu.noise.gyro_bias_rw_sigma;
                    sigmas *= std::sqrt(static_cast<double>(phase209_counts.at(i)));
                    graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
                        biasKey(i), biasKey(i + 1), gtsam::imuBias::ConstantBias(),
                        gtsam::noiseModel::Diagonal::Sigmas(sigmas));
                    ++result.diagnostics.native_phase209_motion_factors;
                    ++result.diagnostics.native_phase209_bias_factors;
                } else {
                  graph.emplace_shared<gtsam::CombinedImuFactor>(
                    positionKey(i), velocityKey(i), positionKey(i + 1), velocityKey(i + 1),
                    biasKey(i), biasKey(i + 1), pims[i]);
                }
            } else {
                graph.emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
                    velocityKey(i), velocityKey(i + 1), gtsam::Vector3::Zero(),
                    gtsam::noiseModel::Isotropic::Sigma(3, 10.0));
                graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
                    biasKey(i), biasKey(i + 1), gtsam::imuBias::ConstantBias(),
                    gtsam::noiseModel::Isotropic::Sigma(6, 1.0));
            }
        }
        result.diagnostics.imu_intervals = imu_intervals;
    } else if (use_pose3) {
        // 2a rotation-only gauge pin (no IMU): resolves the exact rotational
        // null space left by lever-arm-only position observations, without
        // perturbing the estimated antenna position beyond kRotationPinSigmaRad
        // * |lever_arm| (~0.06 mm at 1e-4 rad and the tokyo ~0.62 m lever arm).
        constexpr double kRotationPinSigmaRad = 1e-4;
        const auto rotation_noise = gtsam::noiseModel::Isotropic::Sigma(3, kRotationPinSigmaRad);
        for (std::size_t i = 0; i < num_epochs; ++i) {
            graph.emplace_shared<Pose3RotationPrior>(positionKey(i), Rot3(), rotation_noise);
        }
    }

    // Undifferenced factors need a per-epoch base receiver clock [s] plus, for
    // every non-GPS constellation, ONE global (time-constant) inter-system bias
    // node shared across all epochs -- matching the native backend's per-epoch
    // clock + per-constellation bias columns.
    const bool need_clock_states =
        !problem.pseudorange_factors.empty() ||
        !problem.carrier_phase_factors.empty() ||
        (phase135_requested && !problem.tdcp_factors.empty());
    std::set<gtsam::Key> inserted_clock_keys;
    std::set<int> inserted_isb_ordinals;
    std::set<int> inserted_signal_bias_ordinals;
    auto ensureBaseClock = [&](std::size_t epoch) -> gtsam::Key {
        const gtsam::Key key = clockKey(epoch);
        if (inserted_clock_keys.insert(key).second) {
            const auto* pdc_seed = nativePdcSeedFor(epoch);
            const auto* raw_seed = nativeRawPNoDopplerSeedFor(epoch);
            const double clock_bias_m =
                raw_seed != nullptr && raw_seed->has_clock &&
                        std::isfinite(raw_seed->clock_bias_m)
                    ? raw_seed->clock_bias_m
                    : pdc_seed != nullptr && pdc_seed->has_clock &&
                        std::isfinite(pdc_seed->clock_bias_m[0])
                    ? pdc_seed->clock_bias_m[0]
                    : (epoch < num_epochs ? problem.epochs[epoch].receiver_clock_bias_m
                                           : 0.0);
            if (use_native_source_clock_epoch_vector) {
                // Main-stage C handoff is the only initializer allowed for
                // the same-run Pose3 graph.  GNSS-first staging has no
                // handoff vector yet and therefore uses its retained raw SPP
                // clock seed for component zero.  Components 1..6 start at
                // the official zero values.
                gtsam::Vector clock = gtsam::Vector::Zero(
                    static_cast<Eigen::Index>(kNativeSourceClockVectorDimension));
                if (use_imu &&
                    ((use_native_direct_wls_ephemeral_main_seed &&
                      problem.native_direct_wls_ephemeral_c_handoff_m.size() ==
                          num_epochs) ||
                     (!use_native_direct_wls_ephemeral_main_seed &&
                      problem.native_source_clock_c0d_gnss_first_c_handoff_m.size() ==
                          num_epochs))) {
                    const auto& handoff =
                        use_native_direct_wls_ephemeral_main_seed
                            ? problem.native_direct_wls_ephemeral_c_handoff_m[epoch]
                            : problem.native_source_clock_c0d_gnss_first_c_handoff_m[epoch];
                    for (std::size_t component = 0;
                         component < kNativeSourceClockVectorDimension;
                         ++component) {
                        clock(static_cast<Eigen::Index>(component)) = handoff[component];
                    }
                } else {
                    clock(0) = clock_bias_m;
                }
                initial.insert(key, clock);
            } else {
                initial.insert(
                    key, epoch < num_epochs
                             ? (use_native_source_clock_c0d_meter_state
                                    ? clock_bias_m
                                    : clock_bias_m / constants::SPEED_OF_LIGHT)
                             : 0.0);
            }
        }
        return key;
    };
    // Returns the ISB ordinal for a system: 0 means GPS/base (no ISB node),
    // >0 means a global ISB node was ensured for that constellation.
    auto ensureIsb = [&](GNSSSystem system) -> int {
        if (use_native_source_clock_epoch_vector) {
            // Every source component is epoch-local in C[0..6].  Creating a
            // legacy global `i` state here would double-count ISB and violate
            // the frozen topology, even when the caller left the legacy
            // inter-system setting enabled by mistake (the validation above
            // rejects that configuration before this point).
            (void)system;
            return 0;
        }
        const int ordinal =
            clockGroupOrdinal(clockBiasGroup(system), config.use_inter_system_biases);
        if (ordinal != 0 && inserted_isb_ordinals.insert(ordinal).second) {
            initial.insert(isbKey(ordinal), 0.0);
        }
        return ordinal;
    };
    auto ensureSignalBias = [&](GNSSSystem system, SignalType signal) -> int {
        if (!config.use_receiver_signal_bias_states ||
            !signal_bias::isEligible(system, signal)) {
            return -1;
        }
        const int ordinal = signal_bias::ordinal(system, signal);
        if (ordinal > 0 && inserted_signal_bias_ordinals.insert(ordinal).second) {
            initial.insert(signalBiasKey(ordinal), 0.0);
        }
        return ordinal;
    };

    // One state per problem epoch keeps the residual ionosphere contract
    // explicit even through a sparse GNSS epoch.  States are tied only within
    // a continuous time/clock segment below; a detected clock jump or an
    // invalid/long interval receives a fresh physical prior instead.
    if (use_residual_ionosphere) {
        result.diagnostics.residual_ionosphere_states = num_epochs;
        result.diagnostics.residual_ionosphere_invalid_coefficients =
            problem.diagnostics.residual_ionosphere_invalid_coefficients;
        result.diagnostics.residual_ionosphere_min_coefficient =
            std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < num_epochs; ++i) {
            initial.insert(residualIonosphereKey(i), 0.0);
        }
    }

    // Ambiguity nodes: one per AmbiguityState, in the units that state's
    // consuming factor expects (cycles for DD, meters for undifferenced).
    for (std::size_t i = 0; i < problem.ambiguity_states.size(); ++i) {
        const auto& ambiguity = problem.ambiguity_states[i];
        const double value =
            ambiguity.is_double_difference && ambiguity.wavelength_m > 0.0
                ? ambiguity.initial_ambiguity_m / ambiguity.wavelength_m
                : ambiguity.initial_ambiguity_m;
        initial.insert(ambiguityKey(i), value);
    }
    const bool need_dummy_ambiguity =
        std::any_of(problem.ambiguity_states.begin(), problem.ambiguity_states.end(),
                    [](const FGOProcessor::AmbiguityState& a) { return a.is_double_difference; });
    if (need_dummy_ambiguity) {
        initial.insert(dummyAmbiguityKey(), 0.0);
        // Pin the shared dummy ambiguity at 0 so the lumped libgnss DD ambiguity
        // lives entirely in ambRef (the estimate is then simply ambRef). The
        // dummy is shared across every DD carrier factor, which introduces a
        // common-mode null space (shift dummy + every ambRef by the same delta
        // leaves all carrier residuals unchanged); only this prior constrains
        // it. A near-zero sigma (e.g. 1e-6) pins it but makes the prior's
        // Hessian block ~1e12 against position blocks ~1e2, i.e. condition
        // number ~1e10 -> Cholesky returns a garbage step, every LM trial
        // diverges to inf error, and LM gives up at iteration 0. 1e-3 cycles
        // (~0.2 mm of range) is still "fixed" physically but keeps the system
        // well conditioned (~1e4).
        constexpr double kDummyAmbiguityPinSigmaCycles = 1e-3;
        graph.addPrior(dummyAmbiguityKey(), 0.0,
                       gtsam::noiseModel::Isotropic::Sigma(1, kDummyAmbiguityPinSigmaCycles));
    }

    auto phase135InitialPosition = [&](std::size_t epoch) -> gtsam::Vector3 {
        const Vector3d seed = positionSeedEcef(epoch);
        if (!use_imu) return gtsam::Vector3(seed);
        return gtsam::Vector3(ecef2enu(
            seed - problem.imu.nav_origin_ecef,
            problem.imu.nav_origin_lat_rad,
            problem.imu.nav_origin_lon_rad));
    };
    auto phase135GeometryAt = [&](const Vector3d& source_satellite,
                                  std::size_t epoch,
                                  gtsam::Vector3& los) -> std::optional<double> {
        Phase135SourceGeometry source_geometry;
        if (!phase135SourceGeodist(
                Point3(source_satellite), Point3(positionSeedEcef(epoch)),
                source_geometry)) {
            return std::nullopt;
        }
        los = gtsam::Vector3(source_geometry.los);
        if (use_imu) {
            los = gtsam::Vector3(ecef2enu(
                Vector3d(los(0), los(1), los(2)),
                problem.imu.nav_origin_lat_rad,
                problem.imu.nav_origin_lon_rad));
        }
        if (!los.allFinite() || los.norm() <= 0.0 ||
            std::abs(los.norm() - 1.0) > 1e-8) {
            return std::nullopt;
        }
        return source_geometry.range_m;
    };
    auto phase135InitialVelocityEcef =
        [&](std::size_t epoch,
            const gtsam::Vector3& velocity_state) -> std::optional<Vector3d> {
        if (epoch >= num_epochs || !velocity_state.allFinite()) {
            return std::nullopt;
        }
        // The affine factor's V key is ENU in both the official IMU and the
        // native velocity-state graph.  Gsat's known range-rate expression
        // is ECEF, so convert the same-run initial state exactly once before
        // evaluating its receiver-velocity and Sagnac terms.
        const Vector3d velocity_nav(velocity_state(0), velocity_state(1),
                                    velocity_state(2));
        const Vector3d velocity_ecef = enu2ecef(
            velocity_nav,
            use_imu ? problem.imu.nav_origin_lat_rad
                    : gnss_velocity_origin_lat_rad,
            use_imu ? problem.imu.nav_origin_lon_rad
                    : gnss_velocity_origin_lon_rad);
        if (!velocity_ecef.allFinite()) return std::nullopt;
        return velocity_ecef;
    };

    struct CodeIonosphereDiagnosticRow {
        Eigen::RowVectorXd nuisance;
        double coefficient, sigma, weight, residual;
    };
    std::vector<std::vector<CodeIonosphereDiagnosticRow>> code_ionosphere_rows(num_epochs);
    std::size_t code_ionosphere_invalid = 0;
    std::vector<code_ionosphere::CodeBinding> joint_codes;
    std::vector<code_ionosphere::TdcpBinding> joint_carriers;
    // --- Undifferenced pseudorange / carrier-phase factors ---
    for (const auto& factor : problem.pseudorange_factors) {
        const gtsam::Key base_clock = ensureBaseClock(factor.epoch_index);
        const int ordinal = ensureIsb(factor.satellite.system);
        const int signal_bias_ordinal =
            ensureSignalBias(factor.satellite.system, factor.signal);
        const auto noise = config.use_main_pseudorange_cauchy_loss
            ? makePseudorangeCauchyNoise(factor.sigma_m,4.0)
            : makeNoise(factor.sigma_m, config.use_robust_loss,
                        config.pseudorange_huber_threshold_sigma);
        if (phase135_requested) {
            const int component = sourceClockComponentFor(
                factor.satellite.system, factor.signal);
            gtsam::Vector3 los;
            const auto source_range = phase135GeometryAt(
                factor.source_satellite_position_ecef,
                factor.epoch_index, los);
            if (component < 0 || !source_range.has_value() ||
                !std::isfinite(*source_range)) {
                return failPhase135(
                    "pseudorange affine geometry insertion failed");
            }
            const double residual_m = factor.corrected_pseudorange_m -
                                      *source_range;
            const gtsam::Vector3 initial_position =
                phase135InitialPosition(factor.epoch_index);
            if (!initial_position.allFinite() || !std::isfinite(residual_m)) {
                return failPhase135(
                    "nonfinite pseudorange affine initial value");
            }
            if (use_imu) {
                graph.emplace_shared<Phase135PseudorangeAffineVectorFactor>(
                    affinePositionKey(factor.epoch_index), base_clock, los,
                    residual_m, component, initial_position, noise);
            } else {
                graph.emplace_shared<Phase135PseudorangeAffinePointFactor>(
                    positionKey(factor.epoch_index), base_clock, los,
                    residual_m, component, initial_position, noise);
            }
            ++result.diagnostics.phase135_pseudorange_factors_inserted;
            continue;
        }
        if (use_native_source_clock_epoch_vector) {
            const int component = sourceClockComponentFor(
                factor.satellite.system, factor.signal);
            if (component < 0) {
                result.diagnostics.converged = false;
                return result;
            }
            if (use_pose3) {
                graph.emplace_shared<PseudorangeFactorSourceClockArm>(
                    positionKey(factor.epoch_index), base_clock,
                    factor.corrected_pseudorange_m,
                    Point3(factor.satellite_position_ecef), gnss_lever_arm,
                    component, noise);
            } else {
                graph.emplace_shared<PseudorangeFactorSourceClock>(
                    positionKey(factor.epoch_index), base_clock,
                    factor.corrected_pseudorange_m,
                    Point3(factor.satellite_position_ecef), component, noise);
            }
            if (config.use_native_joint_ionosphere)
                joint_codes.push_back({graph.size()-1,factor.epoch_index,factor.residual_ionosphere_coefficient});
            if (config.use_native_phase171_raw_p_no_doppler_imu_main && use_pose3) {
                const auto actual = std::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph.back());
                std::vector<gtsam::Matrix> jacobians(2);
                const auto residual = actual->unwhitenedError(initial, jacobians);
                const double weight = actual->weight(initial);
                const double coefficient = factor.residual_ionosphere_coefficient;
                if (residual.size() != 1 || !residual.allFinite() ||
                    jacobians[0].rows() != 1 || jacobians[0].cols() != 6 ||
                    jacobians[1].rows() != 1 || jacobians[1].cols() != 7 ||
                    !jacobians[0].allFinite() || !jacobians[1].allFinite() ||
                    !std::isfinite(weight) || weight <= 0 || weight > 1. ||
                    !std::isfinite(coefficient) || coefficient <= 0) {
                    ++code_ionosphere_invalid;
                } else {
                    Eigen::RowVectorXd nuisance(13);
                    nuisance << jacobians[0], jacobians[1];
                    code_ionosphere_rows[factor.epoch_index].push_back({
                        nuisance, coefficient, std::max(1e-9, factor.sigma_m), weight, residual[0]});
                }
            }
            continue;
        }
        const bool valid_ionosphere_coefficient =
            residual_ionosphere::finiteCoefficient(
                factor.residual_ionosphere_coefficient);
        if (use_residual_ionosphere && !valid_ionosphere_coefficient) {
            ++result.diagnostics.residual_ionosphere_invalid_coefficients;
            continue;
        }
        if (use_residual_ionosphere) {
            const double coefficient = factor.residual_ionosphere_coefficient;
            ++result.diagnostics.residual_ionosphere_factors;
            result.diagnostics.residual_ionosphere_min_coefficient = std::min(
                result.diagnostics.residual_ionosphere_min_coefficient,
                coefficient);
            result.diagnostics.residual_ionosphere_max_coefficient = std::max(
                result.diagnostics.residual_ionosphere_max_coefficient,
                coefficient);
            const gtsam::Key ionosphere =
                residualIonosphereKey(factor.epoch_index);
            if (signal_bias_ordinal > 0) {
                const gtsam::Key signal_bias = signalBiasKey(signal_bias_ordinal);
                if (use_pose3) {
                    if (ordinal == 0) {
                        graph.emplace_shared<
                            PseudorangeFactorPlainSignalBiasResidualIonosphereArm>(
                            positionKey(factor.epoch_index), base_clock,
                            signal_bias, ionosphere,
                            factor.corrected_pseudorange_m,
                            Point3(factor.satellite_position_ecef),
                            gnss_lever_arm, coefficient, noise,
                            use_native_source_clock_c0d_meter_state);
                    } else {
                        graph.emplace_shared<
                            PseudorangeFactorISBSignalBiasResidualIonosphereArm>(
                            positionKey(factor.epoch_index), base_clock,
                            isbKey(ordinal), signal_bias, ionosphere,
                            factor.corrected_pseudorange_m,
                            Point3(factor.satellite_position_ecef),
                            gnss_lever_arm, coefficient, noise,
                            use_native_source_clock_c0d_meter_state);
                    }
                } else if (ordinal == 0) {
                    graph.emplace_shared<
                        PseudorangeFactorPlainSignalBiasResidualIonosphere>(
                        positionKey(factor.epoch_index), base_clock, signal_bias,
                        ionosphere, factor.corrected_pseudorange_m,
                        Point3(factor.satellite_position_ecef), coefficient, noise,
                        use_native_source_clock_c0d_meter_state);
                } else {
                    graph.emplace_shared<
                        PseudorangeFactorISBSignalBiasResidualIonosphere>(
                        positionKey(factor.epoch_index), base_clock,
                        isbKey(ordinal), signal_bias, ionosphere,
                        factor.corrected_pseudorange_m,
                        Point3(factor.satellite_position_ecef), coefficient, noise,
                        use_native_source_clock_c0d_meter_state);
                }
            } else if (use_pose3) {
                if (ordinal == 0) {
                    graph.emplace_shared<PseudorangeFactorPlainResidualIonosphereArm>(
                        positionKey(factor.epoch_index), base_clock, ionosphere,
                        factor.corrected_pseudorange_m,
                        Point3(factor.satellite_position_ecef), gnss_lever_arm,
                        coefficient, noise,
                        use_native_source_clock_c0d_meter_state);
                } else {
                    graph.emplace_shared<PseudorangeFactorISBResidualIonosphereArm>(
                        positionKey(factor.epoch_index), base_clock, isbKey(ordinal),
                        ionosphere, factor.corrected_pseudorange_m,
                        Point3(factor.satellite_position_ecef), gnss_lever_arm,
                        coefficient, noise,
                        use_native_source_clock_c0d_meter_state);
                }
            } else if (ordinal == 0) {
                graph.emplace_shared<PseudorangeFactorPlainResidualIonosphere>(
                    positionKey(factor.epoch_index), base_clock, ionosphere,
                    factor.corrected_pseudorange_m,
                    Point3(factor.satellite_position_ecef), coefficient, noise,
                    use_native_source_clock_c0d_meter_state);
            } else {
                graph.emplace_shared<PseudorangeFactorISBResidualIonosphere>(
                    positionKey(factor.epoch_index), base_clock, isbKey(ordinal),
                    ionosphere, factor.corrected_pseudorange_m,
                    Point3(factor.satellite_position_ecef), coefficient, noise,
                    use_native_source_clock_c0d_meter_state);
            }
        } else if (signal_bias_ordinal > 0) {
            const gtsam::Key signal_bias = signalBiasKey(signal_bias_ordinal);
            if (use_pose3) {
                if (ordinal == 0) {
                    graph.emplace_shared<PseudorangeFactorPlainSignalBiasArm>(
                        positionKey(factor.epoch_index), base_clock, signal_bias,
                        factor.corrected_pseudorange_m,
                        Point3(factor.satellite_position_ecef), gnss_lever_arm,
                        noise, use_native_source_clock_c0d_meter_state);
                } else {
                    graph.emplace_shared<PseudorangeFactorISBSignalBiasArm>(
                        positionKey(factor.epoch_index), base_clock,
                        isbKey(ordinal), signal_bias,
                        factor.corrected_pseudorange_m,
                        Point3(factor.satellite_position_ecef), gnss_lever_arm,
                        noise, use_native_source_clock_c0d_meter_state);
                }
            } else if (ordinal == 0) {
                graph.emplace_shared<PseudorangeFactorPlainSignalBias>(
                    positionKey(factor.epoch_index), base_clock, signal_bias,
                    factor.corrected_pseudorange_m,
                    Point3(factor.satellite_position_ecef), noise,
                    use_native_source_clock_c0d_meter_state);
            } else {
                graph.emplace_shared<PseudorangeFactorISBSignalBias>(
                    positionKey(factor.epoch_index), base_clock, isbKey(ordinal),
                    signal_bias, factor.corrected_pseudorange_m,
                    Point3(factor.satellite_position_ecef), noise,
                    use_native_source_clock_c0d_meter_state);
            }
        } else if (use_pose3) {
            if (ordinal == 0) {
                graph.emplace_shared<PseudorangeFactorPlainArm>(
                    positionKey(factor.epoch_index), base_clock,
                    factor.corrected_pseudorange_m, Point3(factor.satellite_position_ecef),
                    gnss_lever_arm, noise,
                    use_native_source_clock_c0d_meter_state);
            } else {
                graph.emplace_shared<PseudorangeFactorISBArm>(
                    positionKey(factor.epoch_index), base_clock, isbKey(ordinal),
                    factor.corrected_pseudorange_m, Point3(factor.satellite_position_ecef),
                    gnss_lever_arm, noise,
                    use_native_source_clock_c0d_meter_state);
            }
        } else if (ordinal == 0) {
            graph.emplace_shared<PseudorangeFactorPlain>(
                positionKey(factor.epoch_index), base_clock,
                factor.corrected_pseudorange_m, Point3(factor.satellite_position_ecef),
                noise, use_native_source_clock_c0d_meter_state);
        } else {
            graph.emplace_shared<PseudorangeFactorISB>(
                positionKey(factor.epoch_index), base_clock, isbKey(ordinal),
                factor.corrected_pseudorange_m, Point3(factor.satellite_position_ecef),
                noise, use_native_source_clock_c0d_meter_state);
        }
    }

    // --- Receiver-only P+D velocity/clock-drift factors ---
    // Dedicated main-graph rows leave generic Phase171 no-D admission intact.
    // Corrected range-rate and source clock drift are both in metres/second.
    const auto rotationRateRow = [&](const FGOProcessor::UndifferencedDopplerFactor& row) {
        if (!raw_p_ecef_doppler::validCorrectedEcefDopplerRow(row))
            throw std::invalid_argument("Rotation-rate Doppler requires valid corrected raw row");
        const auto geometry = doppler_rotation_rate::fromLos(-row.los,
            row.satellite_position_ecef, row.satellite_velocity_ecef);
        if (!geometry) throw std::invalid_argument("Invalid rotation-rate Doppler geometry");
        const double measured = row.measured_range_rate_mps - geometry->satellite_mps +
            row.satellite_clock_drift_mps;
        if (!std::isfinite(measured)) throw std::invalid_argument("Invalid rotation-rate Doppler residual");
        return std::make_pair(geometry->receiver_velocity_jacobian, measured);
    };
    for (const auto& factor : phase213_rows) {
        gtsam::Vector3 los_nav(ecef2enu(factor.los,
            problem.imu.nav_origin_lat_rad, problem.imu.nav_origin_lon_rad));
        if (!los_nav.allFinite() || std::abs(los_nav.norm() - 1.0) > 1e-6 ||
            !initial.exists(velocityKey(factor.epoch_index)) ||
            !initial.exists(dopplerClockDriftKey(factor.epoch_index))) {
            throw std::invalid_argument("Phase213 invalid ENU geometry or missing state");
        }
        const auto noise = makeNoise(factor.sigma_mps, config.use_robust_loss,
            config.undifferenced_doppler_huber_threshold_sigma);
        double measured = factor.residual_mps;
        if (config.use_native_doppler_rotation_rate) {
            const auto corrected = rotationRateRow(factor);
            // This is a velocity coefficient, not a unit LOS. Rotate only.
            los_nav = ecef2enu(corrected.first, problem.imu.nav_origin_lat_rad,
                              problem.imu.nav_origin_lon_rad);
            measured = corrected.second;
        }
        graph.emplace_shared<UndifferencedDopplerVelocityFactorSourceClock>(
            velocityKey(factor.epoch_index), dopplerClockDriftKey(factor.epoch_index),
            los_nav, measured, noise);
        ++result.diagnostics.native_phase213_main_doppler_factors;
    }
    // In the historical GNSS-only path these are the explicit velocity-state
    // graph.  The raw upstream-quality candidate also inserts the same rows
    // into the IMU Pose3 graph, where velocityKey is the already-existing ENU
    // kinematic state and dKey is a separate receiver clock-rate state.
    std::size_t undifferenced_doppler_inserted = 0;
    if (use_gnss_velocity_states || use_imu_doppler_factors ||
        use_native_source_clock_c0d_factor) {
        for (const auto& factor : problem.undifferenced_doppler_factors) {
            if (factor.epoch_index >= num_epochs || !factor.los.allFinite() ||
                !std::isfinite(factor.residual_mps) ||
                !std::isfinite(factor.sigma_mps)) {
                continue;
            }
            const auto noise = makeNoise(
                factor.sigma_mps, config.use_robust_loss,
                config.undifferenced_doppler_huber_threshold_sigma);
            if (use_native_raw_p_ecef_doppler_graph) {
                // Phase171 staging keeps the Point3/V velocity state in ECEF.
                // Do not rotate this source LOS; the existing Pose3/IMU path
                // below remains the only ECEF->ENU conversion boundary.
                gtsam::Vector3 los_ecef(factor.los);
                if (!los_ecef.allFinite() ||
                    std::abs(los_ecef.norm() - 1.0) > 1e-6) {
                    result.diagnostics.converged = false;
                    return result;
                }
                double measured = factor.residual_mps;
                if (config.use_native_doppler_rotation_rate) {
                    const auto corrected = rotationRateRow(factor);
                    los_ecef = corrected.first;
                    measured = corrected.second;
                }
                graph.emplace_shared<
                    UndifferencedDopplerVelocityFactorSourceClockEcef>(
                    velocityKey(factor.epoch_index),
                    dopplerClockDriftKey(factor.epoch_index), los_ecef,
                    measured, noise);
                ++undifferenced_doppler_inserted;
                continue;
            }
            const gtsam::Vector3 los_nav(ecef2enu(
                factor.los, gnss_velocity_origin_lat_rad,
                gnss_velocity_origin_lon_rad));
            if (!los_nav.allFinite() || los_nav.norm() <= 0.0) continue;
            if (phase135_requested) {
                gtsam::Vector3 los_affine;
                const auto source_range = phase135GeometryAt(
                    factor.source_satellite_position_ecef,
                    factor.epoch_index, los_affine);
                if (!source_range.has_value() ||
                    !initial.exists(velocityKey(factor.epoch_index)) ||
                    !std::isfinite(*source_range)) {
                    return failPhase135(
                        "Doppler affine geometry/state insertion failed");
                }
                const gtsam::Vector3 initial_velocity =
                    initial.at<gtsam::Vector3>(velocityKey(factor.epoch_index));
                if (!initial_velocity.allFinite()) {
                    return failPhase135(
                        "nonfinite Doppler affine initial velocity");
                }
                const auto initial_velocity_ecef =
                    phase135InitialVelocityEcef(factor.epoch_index,
                                                initial_velocity);
                if (!initial_velocity_ecef.has_value()) {
                    return failPhase135(
                        "nonfinite Doppler affine ECEF initial velocity");
                }
                double modeled_range_rate_mps = 0.0;
                double residual_mps = 0.0;
                if (!phase135OfficialDopplerResidual(
                        Point3(factor.source_satellite_position_ecef),
                        gtsam::Vector3(factor.source_satellite_velocity_ecef),
                        Point3(positionSeedEcef(factor.epoch_index)),
                        gtsam::Vector3(*initial_velocity_ecef),
                        factor.measured_range_rate_mps,
                        factor.satellite_clock_drift_mps,
                        modeled_range_rate_mps, residual_mps)) {
                    return failPhase135(
                        "official Doppler range-rate/resD adapter failed");
                }
                graph.emplace_shared<Phase135DopplerAffineFactor>(
                    velocityKey(factor.epoch_index),
                    dopplerClockDriftKey(factor.epoch_index), los_affine,
                    residual_mps, initial_velocity, noise);
                ++undifferenced_doppler_inserted;
                ++result.diagnostics.phase135_doppler_factors_inserted;
                continue;
            }
            if (use_native_source_clock_epoch_vector) {
                graph.emplace_shared<UndifferencedDopplerVelocityFactorSourceClock>(
                    velocityKey(factor.epoch_index),
                    dopplerClockDriftKey(factor.epoch_index),
                    los_nav, factor.residual_mps, noise);
            } else {
                graph.emplace_shared<UndifferencedDopplerVelocityFactor>(
                    velocityKey(factor.epoch_index),
                    dopplerClockDriftKey(factor.epoch_index),
                    los_nav, factor.residual_mps, noise);
            }
            ++undifferenced_doppler_inserted;
        }
    }
    result.diagnostics.undifferenced_doppler_factors_inserted =
        undifferenced_doppler_inserted;
    // Undifferenced carrier phase uses the same base-clock unit boundary as
    // pseudorange.  Preserve the historical behavior exactly when the
    // Phase92 meter-state selector is absent: stock GTSAM handles the
    // Point3/seconds path and the pre-existing Pose3 path skips these factors.
    // The local equivalents are inserted only for the opt-in Pose3 meter
    // graph, because stock CarrierPhaseFactor hard-codes a seconds-valued
    // clock key.  ISB remains a separate DD concern and is intentionally not
    // added to this undifferenced carrier contract.
    for (const auto& factor : problem.carrier_phase_factors) {
        if (factor.ambiguity_index >= problem.ambiguity_states.size()) {
            continue;
        }
        if (use_pose3 && !use_native_source_clock_c0d_meter_state) {
            std::fprintf(stderr,
                         "[fgo_gtsam_backend] WARNING: undifferenced carrier_phase_factors are "
                         "not yet supported with use_pose3_state; skipping %zu factor(s).\n",
                         problem.carrier_phase_factors.size());
            break;
        }
        const gtsam::Key base_clock = ensureBaseClock(factor.epoch_index);
        const auto noise = makeNoise(factor.sigma_m, config.use_robust_loss,
                                     config.carrier_phase_huber_threshold_sigma);
        if (use_pose3) {
            graph.emplace_shared<UndifferencedCarrierPhaseFactorArm>(
                positionKey(factor.epoch_index), base_clock,
                ambiguityKey(factor.ambiguity_index), factor.corrected_carrier_m,
                Point3(factor.satellite_position_ecef), gnss_lever_arm, noise,
                true);
        } else if (use_native_source_clock_c0d_meter_state) {
            graph.emplace_shared<UndifferencedCarrierPhaseFactorPlain>(
                positionKey(factor.epoch_index), base_clock,
                ambiguityKey(factor.ambiguity_index), factor.corrected_carrier_m,
                Point3(factor.satellite_position_ecef), noise, true);
        } else {
            graph.emplace_shared<gtsam::CarrierPhaseFactor>(
                positionKey(factor.epoch_index), base_clock,
                ambiguityKey(factor.ambiguity_index), factor.corrected_carrier_m,
                Point3(factor.satellite_position_ecef), 0.0, noise);
        }
    }

    // --- Ordinary (single-receiver) TDCP factors ---
    // Keep the exact v1 temporal carrier constraint in the IMU-coupled Pose3
    // graph.  This is deliberately separate from the base-dependent
    // single/double-difference paths: a no-base smartphone run has one
    // receiver clock per epoch and one undifferenced carrier delta per
    // satellite.  The factor uses the same sigma/Huber contract as the Eigen
    // backend; invalid epoch indices are skipped and counted as not inserted.
    std::size_t ordinary_tdcp_inserted = 0;
    const bool frequency_residual_states = config.use_native_tdcp_frequency_residual_states;
    const auto no_frequency_pair = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> frequency_pair_by_factor;
    std::vector<double> frequency_alpha_by_factor;
    if (frequency_residual_states) {
        if (!phase171_requested || !use_imu || !use_native_source_clock_epoch_vector ||
            !config.use_tdcp_factors || use_native_raw_p_seed_graph || phase135_requested ||
            config.use_native_tdcp_only_affine_geometry ||
            config.use_source_tdcp_resl_observable ||
            config.use_official_tdcp_resl_atmosphere_cancellation ||
            !std::isfinite(config.native_tdcp_frequency_residual_prior_sigma_m) ||
            config.native_tdcp_frequency_residual_prior_sigma_m <= 0.0 ||
            problem.clock_jumps.size() != num_epochs) {
            throw std::invalid_argument("TDCP frequency states require corrected-carrier native IMU C7 main");
        }
        const auto pairs = tdcp_frequency::pairAdmittedFactorsAtEpochs(
            problem.tdcp_factors, problem.clock_jumps, problem.epochs);
        frequency_pair_by_factor.assign(problem.tdcp_factors.size(), no_frequency_pair);
        frequency_alpha_by_factor.assign(problem.tdcp_factors.size(), 0.0);
        for (std::size_t pair_index=0; pair_index<pairs.size(); ++pair_index) {
            const auto key = Symbol('u', pair_index);
            if (initial.exists(key)) throw std::invalid_argument("TDCP frequency state key collision");
            initial.insert(key, 0.0);
            graph.emplace_shared<gtsam::PriorFactor<double>>(key, 0.0,
                gtsam::noiseModel::Isotropic::Sigma(1, config.native_tdcp_frequency_residual_prior_sigma_m));
            ++result.diagnostics.tdcp_frequency_residual_priors;
            const auto& pair = pairs[pair_index];
            frequency_pair_by_factor[pair.l1_index] = pair_index;
            frequency_pair_by_factor[pair.l5_index] = pair_index;
            frequency_alpha_by_factor[pair.l1_index] = 1.0;
            frequency_alpha_by_factor[pair.l5_index] = std::pow(1575.42/1176.45, 2);
        }
        result.diagnostics.tdcp_frequency_residual_states = pairs.size();
    }
    if (config.use_native_tdcp_only_affine_geometry &&
        (!use_native_source_clock_epoch_vector || phase135_requested ||
         !(use_native_raw_p_seed_graph || (phase171_requested && use_imu)))) {
        throw std::invalid_argument("TDCP-only affine geometry requires native source-clock raw stage or Phase171 IMU main");
    }
    if ((use_pose3 || phase135_requested || use_native_raw_p_seed_graph) &&
        config.use_tdcp_factors) {
        for (const auto& factor : problem.tdcp_factors) {
            if (factor.previous_epoch_index >= num_epochs ||
                factor.current_epoch_index >= num_epochs ||
                factor.current_epoch_index <= factor.previous_epoch_index) {
                continue;
            }
            if (!std::isfinite(factor.delta_carrier_m) ||
                !factor.previous_satellite_position_ecef.allFinite() ||
                !factor.current_satellite_position_ecef.allFinite()) {
                continue;
            }
            const gtsam::Key previous_clock =
                ensureBaseClock(factor.previous_epoch_index);
            const gtsam::Key current_clock =
                ensureBaseClock(factor.current_epoch_index);
            const auto noise = makeNoise(
                factor.sigma_m, config.use_robust_loss,
                ordinary_tdcp_huber_threshold_sigma);
            if (config.use_native_tdcp_only_affine_geometry) {
                const auto k1 = positionKey(factor.previous_epoch_index);
                const auto k2 = positionKey(factor.current_epoch_index);
                const Point3 a1 = use_native_raw_p_seed_graph
                    ? initial.at<Point3>(k1)
                    : gnss_lever_arm.antennaPosition(initial.at<Pose3>(k1));
                const Point3 a2 = use_native_raw_p_seed_graph
                    ? initial.at<Point3>(k2)
                    : gnss_lever_arm.antennaPosition(initial.at<Pose3>(k2));
                gtsam::Matrix13 h;
                const double r1 = plainRange(Point3(factor.previous_satellite_position_ecef), a1, &h);
                const double r2 = plainRange(
                    Point3(factor.current_satellite_position_ecef), a2, nullptr);
                const double measurement = factor.delta_carrier_m - (r2 - r1);
                if (!a1.allFinite() || !a2.allFinite() || !h.allFinite() ||
                    !std::isfinite(measurement) || r1 <= 0.0 || r2 <= 0.0) {
                    throw std::invalid_argument("Invalid TDCP-only affine anchor geometry");
                }
                if (use_native_raw_p_seed_graph) {
                    graph.emplace_shared<Phase135TdcpAffinePointFactor>(
                        k1, k2, previous_clock, current_clock, h.transpose(),
                        measurement, a1, a2, noise);
                } else {
                    graph.emplace_shared<SourceAffineTdcpPoseFactor>(
                        k1, previous_clock, k2, current_clock, h.transpose(),
                        a1, a2, measurement, gnss_lever_arm, noise);
                }
                ++result.diagnostics.tdcp_only_affine_factors_inserted;
                ++ordinary_tdcp_inserted;
                continue;
            }
            if (phase135_requested) {
                gtsam::Vector3 los_affine;
                const auto source_range = phase135GeometryAt(
                    factor.previous_source_satellite_position_ecef,
                    factor.previous_epoch_index, los_affine);
                if (!source_range.has_value() || !std::isfinite(*source_range)) {
                    return failPhase135("TDCP affine geometry insertion failed");
                }
                double tdcp_measurement_m = factor.delta_carrier_m;
                if (phase138_requested) {
                    // The affine TDCP Jacobian is anchored at the previous
                    // endpoint LOS, but its measurement constant must carry
                    // the source-consistent range change for both endpoints.
                    // Each endpoint is evaluated by the same geodist helper,
                    // which applies the frozen single-Sagnac convention once.
                    gtsam::Vector3 current_los_unused;
                    const auto current_range = phase135GeometryAt(
                        factor.current_source_satellite_position_ecef,
                        factor.current_epoch_index, current_los_unused);
                    if (!current_range.has_value() ||
                        !std::isfinite(*current_range)) {
                        return failPhase135(
                            "Phase138 current initial range geometry failed");
                    }
                    tdcp_contract::Phase138AffineTdcpMeasurement adjusted;
                    if (!tdcp_contract::applyPhase138AffineTdcpAnchorRangeConstant(
                            factor.delta_carrier_m, *source_range, *current_range,
                            adjusted)) {
                        return failPhase135(
                            "Phase138 TDCP anchor-range measurement adjustment failed");
                    }
                    tdcp_measurement_m = adjusted.tdcp_m;
                    ++result.diagnostics.phase138_tdcp_range_constants_validated;
                }
                const gtsam::Vector3 initial_previous =
                    phase135InitialPosition(factor.previous_epoch_index);
                const gtsam::Vector3 initial_current =
                    phase135InitialPosition(factor.current_epoch_index);
                if (!initial_previous.allFinite() ||
                    !initial_current.allFinite()) {
                    return failPhase135("nonfinite TDCP affine initial value");
                }
                if (use_imu) {
                    graph.emplace_shared<Phase135TdcpAffineVectorFactor>(
                        affinePositionKey(factor.previous_epoch_index),
                        affinePositionKey(factor.current_epoch_index),
                        previous_clock, current_clock, los_affine,
                        tdcp_measurement_m, initial_previous,
                        initial_current, noise);
                } else {
                    graph.emplace_shared<Phase135TdcpAffinePointFactor>(
                        positionKey(factor.previous_epoch_index),
                        positionKey(factor.current_epoch_index),
                        previous_clock, current_clock, los_affine,
                        tdcp_measurement_m, initial_previous,
                        initial_current, noise);
                }
                ++ordinary_tdcp_inserted;
                ++result.diagnostics.phase135_tdcp_factors_inserted;
                if (phase138_requested) {
                    ++result.diagnostics.phase138_tdcp_measurements_adjusted;
                }
                continue;
            }
            if (use_native_source_clock_epoch_vector) {
                if (use_native_raw_p_seed_graph) {
                    graph.emplace_shared<TimeDifferencedCarrierFactorSourceClockPoint>(
                        positionKey(factor.previous_epoch_index), previous_clock,
                        positionKey(factor.current_epoch_index), current_clock,
                        Point3(factor.previous_satellite_position_ecef),
                        Point3(factor.current_satellite_position_ecef),
                        factor.delta_carrier_m, noise);
                } else {
                    graph.emplace_shared<TimeDifferencedCarrierFactorSourceClockArm>(
                        positionKey(factor.previous_epoch_index), previous_clock,
                        positionKey(factor.current_epoch_index), current_clock,
                        Point3(factor.previous_satellite_position_ecef),
                        Point3(factor.current_satellite_position_ecef),
                        factor.delta_carrier_m, gnss_lever_arm, noise);
                }
            } else {
                graph.emplace_shared<TimeDifferencedCarrierFactorArm>(
                    positionKey(factor.previous_epoch_index), previous_clock,
                    positionKey(factor.current_epoch_index), current_clock,
                    Point3(factor.previous_satellite_position_ecef),
                    Point3(factor.current_satellite_position_ecef),
                    factor.delta_carrier_m, gnss_lever_arm, noise,
                    use_native_source_clock_c0d_meter_state);
            }
            if (config.use_native_joint_ionosphere)
                joint_carriers.push_back({graph.size()-1,factor.previous_epoch_index,factor.current_epoch_index,
                    factor.previous_residual_ionosphere_coefficient,factor.current_residual_ionosphere_coefficient});
            if (frequency_residual_states) {
                const auto index = static_cast<std::size_t>(&factor-problem.tdcp_factors.data());
                if (frequency_pair_by_factor[index] != no_frequency_pair) {
                    const auto original = std::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph.back());
                    // FactorGraph::back() returns a shared_ptr BY VALUE.
                    // Assign through the mutable index to replace the graph slot.
                    graph[graph.size()-1] = std::make_shared<tdcp_frequency::ResidualStateFactor>(
                        original, Symbol('u', frequency_pair_by_factor[index]), frequency_alpha_by_factor[index]);
                    ++result.diagnostics.tdcp_frequency_residual_factors;
                }
            }
            ++ordinary_tdcp_inserted;
        }
    }
    result.diagnostics.tdcp_factors_inserted = ordinary_tdcp_inserted;
    if (frequency_residual_states && result.diagnostics.tdcp_frequency_residual_factors !=
        2*result.diagnostics.tdcp_frequency_residual_states) {
        throw std::invalid_argument("TDCP frequency pair was not inserted exactly twice");
    }
    if (phase138_requested) {
        result.diagnostics.phase138_factor_count_unchanged =
            ordinary_tdcp_inserted == problem.tdcp_factors.size();
        const bool exact_adjustment =
            result.diagnostics.phase138_factor_count_unchanged &&
            result.diagnostics.phase138_tdcp_range_constants_validated ==
                problem.tdcp_factors.size() &&
            result.diagnostics.phase138_tdcp_measurements_adjusted ==
                problem.tdcp_factors.size();
        result.diagnostics.phase138_adjustment_application_passes =
            exact_adjustment ? 1U : 0U;
        result.diagnostics.phase138_adjusted_exactly_once = exact_adjustment;
        result.diagnostics.phase138_same_endpoint_epoch_and_satellite_state =
            exact_adjustment;
        result.diagnostics.phase138_same_satellite_state = exact_adjustment;
        result.diagnostics.phase138_finite_adjusted_measurements =
            exact_adjustment;
        result.diagnostics.phase138_no_raw_or_zero_fallback = exact_adjustment;
        result.diagnostics.phase138_transactional = exact_adjustment;
        result.diagnostics.phase138_legacy_tdcp_factor_count = 0U;
        if (!exact_adjustment) {
            return failPhase135(
                "Phase138 TDCP adjustment or factor-count conservation failed");
        }
    }
    if (phase135_requested &&
        (result.diagnostics.phase135_pseudorange_factors_inserted !=
             problem.pseudorange_factors.size() ||
         result.diagnostics.phase135_doppler_factors_inserted !=
             problem.undifferenced_doppler_factors.size() ||
         result.diagnostics.phase135_tdcp_factors_inserted !=
             problem.tdcp_factors.size() ||
         result.diagnostics.phase135_pose3_x_bridge_factors !=
             (use_imu ? num_epochs : 0U))) {
        return failPhase135(
            "Phase135 affine family row or Pose3-X bridge conservation failed");
    }
    if (phase135_requested) {
        // This is the commit point of the all-or-nothing affine family.  The
        // booleans below are report fields only: all checks above have already
        // completed and no factor/ordering/solver input is modified here.
        result.diagnostics.phase135_transactional = true;
        result.diagnostics.phase135_finite_jacobians = true;
        result.diagnostics.phase135_pseudorange_key_order_exact = true;
        result.diagnostics.phase135_pseudorange_finite_values = true;
        result.diagnostics.phase135_pseudorange_source_geometry_same_path = true;
        result.diagnostics.phase135_doppler_key_order_exact = true;
        result.diagnostics.phase135_doppler_finite_values = true;
        result.diagnostics.phase135_doppler_source_geometry_same_path = true;
        result.diagnostics.phase135_tdcp_key_order_exact = true;
        result.diagnostics.phase135_tdcp_finite_values = true;
        result.diagnostics.phase135_tdcp_source_geometry_same_path = true;
        result.diagnostics.phase135_legacy_pseudorange_factor_count = 0U;
        result.diagnostics.phase135_legacy_doppler_factor_count = 0U;
        result.diagnostics.phase135_legacy_tdcp_factor_count = 0U;
        result.diagnostics.phase135_pose3_x_bridge_keys_exact =
            result.diagnostics.phase135_pose3_x_bridge_factors ==
            (use_imu ? num_epochs : 0U);
    }

    // --- Double-difference pseudorange factors ---
    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        const gtsam::gnss::DoubleDifferenceData dd{
            factor.rover_satellite_model.corrected_pseudorange_m,
            factor.base_satellite_model.corrected_pseudorange_m,
            factor.rover_reference_model.corrected_pseudorange_m,
            factor.base_reference_model.corrected_pseudorange_m,
            Point3(factor.rover_satellite_position_ecef),
            Point3(factor.rover_reference_position_ecef),
            Point3(factor.base_satellite_position_ecef),
            Point3(factor.base_reference_position_ecef),
            Point3(factor.base_position_ecef),
        };
        checkObservedDdMatches(dd.observed(), factor.observed_dd_pseudorange_m,
                              "DD pseudorange");

        const auto noise = makeNoise(factor.sigma_m, config.use_robust_loss,
                                     config.pseudorange_huber_threshold_sigma);
        if (use_imu) {
            // 2b: pose is body-in-nav (ENU) -> feed ecef_T_nav so the factor
            // reconstructs the antenna ECEF from the nav-frame pose.
            graph.emplace_shared<gtsam::DoubleDifferencePseudorangeFactorArm>(
                positionKey(factor.epoch_index),
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, lever_arm_body, ecef_T_nav, noise);
        } else if (use_pose3) {
            // 2a: pose expressed directly in ECEF (no ecef_T_nav).
            graph.emplace_shared<gtsam::DoubleDifferencePseudorangeFactorArm>(
                positionKey(factor.epoch_index),
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, lever_arm_body, noise);
        } else {
            graph.emplace_shared<gtsam::DoubleDifferencePseudorangeFactor>(
                positionKey(factor.epoch_index),
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, noise);
        }
    }

    // --- Double-difference carrier-phase factors ---
    for (const auto& factor : problem.double_difference_carrier_factors) {
        if (factor.ambiguity_index >= problem.ambiguity_states.size()) {
            continue;
        }
        const auto& ambiguity = problem.ambiguity_states[factor.ambiguity_index];
        const gtsam::gnss::DoubleDifferenceData dd{
            factor.rover_satellite_model.corrected_carrier_m,
            factor.base_satellite_model.corrected_carrier_m,
            factor.rover_reference_model.corrected_carrier_m,
            factor.base_reference_model.corrected_carrier_m,
            Point3(factor.rover_satellite_position_ecef),
            Point3(factor.rover_reference_position_ecef),
            Point3(factor.base_satellite_position_ecef),
            Point3(factor.base_reference_position_ecef),
            Point3(factor.base_position_ecef),
        };
        checkObservedDdMatches(dd.observed(), factor.observed_dd_carrier_m, "DD carrier");

        const auto noise = makeNoise(factor.sigma_m, config.use_robust_loss,
                                     config.carrier_phase_huber_threshold_sigma);
        if (use_imu) {
            graph.emplace_shared<gtsam::DoubleDifferenceCarrierPhaseFactorArm>(
                positionKey(factor.epoch_index),
                ambiguityKey(factor.ambiguity_index),  // ambRef <- real DD ambiguity node
                dummyAmbiguityKey(),                   // ambTarget <- pinned at 0
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, ambiguity.wavelength_m, lever_arm_body, ecef_T_nav, noise);
        } else if (use_pose3) {
            graph.emplace_shared<gtsam::DoubleDifferenceCarrierPhaseFactorArm>(
                positionKey(factor.epoch_index),
                ambiguityKey(factor.ambiguity_index),  // ambRef <- real DD ambiguity node
                dummyAmbiguityKey(),                   // ambTarget <- pinned at 0
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, ambiguity.wavelength_m, lever_arm_body, noise);
        } else {
            graph.emplace_shared<gtsam::DoubleDifferenceCarrierPhaseFactor>(
                positionKey(factor.epoch_index),
                ambiguityKey(factor.ambiguity_index),  // ambRef <- real DD ambiguity node
                dummyAmbiguityKey(),                   // ambTarget <- pinned at 0
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, ambiguity.wavelength_m, noise);
        }
    }

    // --- Ambiguity priors ---
    if (config.use_ambiguity_priors && config.ambiguity_prior_sigma_m > 0.0) {
        for (std::size_t i = 0; i < problem.ambiguity_states.size(); ++i) {
            const auto& ambiguity = problem.ambiguity_states[i];
            if (ambiguity.is_double_difference && ambiguity.wavelength_m > 0.0) {
                const double mean = ambiguity.initial_ambiguity_m / ambiguity.wavelength_m;
                const double sigma = config.ambiguity_prior_sigma_m / ambiguity.wavelength_m;
                graph.addPrior(ambiguityKey(i), mean, gtsam::noiseModel::Isotropic::Sigma(1, sigma));
            } else {
                graph.addPrior(ambiguityKey(i), ambiguity.initial_ambiguity_m,
                               gtsam::noiseModel::Isotropic::Sigma(1, config.ambiguity_prior_sigma_m));
            }
        }
    }

    // --- Ambiguity continuity (BetweenFactor<double>) ---
    for (const auto& factor : problem.ambiguity_between_factors) {
        if (factor.previous_ambiguity_index >= problem.ambiguity_states.size() ||
            factor.current_ambiguity_index >= problem.ambiguity_states.size()) {
            continue;
        }
        const auto& ambiguity = problem.ambiguity_states[factor.current_ambiguity_index];
        const double wavelength = ambiguity.wavelength_m;
        const double sigma_cycles = wavelength > 0.0 ? factor.sigma_m / wavelength : factor.sigma_m;
        graph.emplace_shared<gtsam::BetweenFactor<double>>(
            ambiguityKey(factor.previous_ambiguity_index),
            ambiguityKey(factor.current_ambiguity_index), 0.0,
            gtsam::noiseModel::Isotropic::Sigma(1, std::max(1e-9, sigma_cycles)));
    }

    // --- Position motion (random-walk) factors between consecutive epochs ---
    if (phase217_requested) {
        // Source Street sigma and time-gap threshold, not tuned to H truth.
        const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
        for (std::size_t i = 1; i < num_epochs; ++i) {
            const double dt = problem.epochs[i].time - problem.epochs[i - 1].time;
            if (!std::isfinite(dt) || dt <= 0.0) {
                throw std::invalid_argument("Phase217 invalid epoch duration");
            }
            if (dt >= 1.5) {
                ++result.diagnostics.native_phase217_main_motion_gap_skips;
                continue;
            }
            graph.emplace_shared<MotionFactorPose3XXVV>(
                positionKey(i - 1), positionKey(i), velocityKey(i - 1), velocityKey(i), dt, noise);
            ++result.diagnostics.native_phase217_main_motion_factors;
        }
    }
    if (use_native_raw_p_seed_graph) {
        // The dedicated no-D graph uses the source XXVV kinematic equation;
        // never substitute the legacy zero-motion Point3 random walk here.
        if (!std::isfinite(config.velocity_motion_sigma_m) ||
            config.velocity_motion_sigma_m <= 0.0) {
            result.diagnostics.converged = false;
            return result;
        }
        const auto noise = gtsam::noiseModel::Isotropic::Sigma(
            3, config.velocity_motion_sigma_m);
        for (std::size_t i = 1; i < num_epochs; ++i) {
            const double dt_s = problem.epochs[i].time -
                                problem.epochs[i - 1U].time;
            graph.emplace_shared<MotionFactorXXVV>(
                positionKey(i - 1U), positionKey(i), velocityKey(i - 1U),
                velocityKey(i), dt_s, noise);
        }
    } else if (config.use_motion_factors && config.use_position_motion_factors &&
               config.motion_sigma_m > 0.0) {
        if (use_pose3) {
            // Pose3 analog of the Point3 zero-motion BetweenFactor below: same
            // loose translation sigma (motion_sigma_m, e.g. 100 m -- a
            // near-no-op regularizer at DD noise levels), and an even looser
            // rotation sigma so it never competes with the dedicated
            // Pose3RotationPrior gauge pin above.
            constexpr double kLooseRotationSigmaRad = 10.0;
            gtsam::Vector6 sigmas;
            sigmas << kLooseRotationSigmaRad, kLooseRotationSigmaRad, kLooseRotationSigmaRad,
                config.motion_sigma_m, config.motion_sigma_m, config.motion_sigma_m;
            const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
            for (std::size_t i = 1; i < num_epochs; ++i) {
                graph.emplace_shared<gtsam::BetweenFactor<Pose3>>(
                    positionKey(i - 1), positionKey(i), Pose3(Rot3(), Point3(0.0, 0.0, 0.0)),
                    noise);
            }
        } else {
            const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, config.motion_sigma_m);
            for (std::size_t i = 1; i < num_epochs; ++i) {
                graph.emplace_shared<gtsam::BetweenFactor<Point3>>(
                    positionKey(i - 1), positionKey(i), Point3(0.0, 0.0, 0.0), noise);
            }
        }
    }

    // The native v1 motion graph also contains an independent receiver-clock
    // random-walk row for each adjacent epoch.  Pose3's BetweenFactor above
    // only constrains pose translation/rotation, so retain the clock part
    // explicitly.  The candidate meter graph uses a metre-valued clock key and
    // metre sigma directly; the legacy graph keeps its historical seconds key
    // and the exact C_LIGHT conversion.  A detected common GPS code jump
    // receives the same loose 1e6 m gate used by the Eigen backend.
    if (need_clock_states && config.use_motion_factors &&
        config.use_clock_motion_factors && num_epochs >= 2 &&
        (config.clock_motion_sigma_m > 0.0 ||
         use_native_source_clock_c0d_factor)) {
        bool have_positive_dt = false;
        for (std::size_t i = 1; i < num_epochs; ++i) {
            const gtsam::Key previous_clock = ensureBaseClock(i - 1);
            const gtsam::Key current_clock = ensureBaseClock(i);
            const bool clock_jump =
                i < problem.clock_jumps.size() && problem.clock_jumps[i];
            if (use_native_source_clock_c0d_factor) {
                const double dt_s =
                    problem.epochs[i].time - problem.epochs[i - 1].time;
                if (std::isfinite(dt_s) && dt_s > 0.0) {
                    if (!have_positive_dt) {
                        result.diagnostics.native_source_clock_c0d_dt_min_s = dt_s;
                        result.diagnostics.native_source_clock_c0d_dt_max_s = dt_s;
                        have_positive_dt = true;
                    } else {
                        result.diagnostics.native_source_clock_c0d_dt_min_s =
                            std::min(result.diagnostics.native_source_clock_c0d_dt_min_s,
                                      dt_s);
                        result.diagnostics.native_source_clock_c0d_dt_max_s =
                            std::max(result.diagnostics.native_source_clock_c0d_dt_max_s,
                                      dt_s);
                    }
                }
                const auto decision = nativeSourceClockC0DEdgeDecision(
                    dt_s, clock_jump, config.native_source_clock_c0d_phone);
                switch (decision.reason) {
                    case NativeSourceClockC0DSkipReason::InvalidDt:
                        ++result.diagnostics.native_source_clock_c0d_invalid_dt_skips;
                        break;
                    case NativeSourceClockC0DSkipReason::Gap:
                        ++result.diagnostics.native_source_clock_c0d_gap_skips;
                        break;
                    case NativeSourceClockC0DSkipReason::PhoneExcluded:
                        ++result.diagnostics.native_source_clock_c0d_phone_exclusion_skips;
                        break;
                    case NativeSourceClockC0DSkipReason::ClockJump:
                        ++result.diagnostics.native_source_clock_c0d_clock_jump_skips;
                        break;
                    case NativeSourceClockC0DSkipReason::Eligible:
                        if (use_native_source_clock_epoch_vector) {
                            // Official CCDD uses a seven-dimensional
                            // residual: row zero has finite clock/drift noise,
                            // while zero sigmas constrain the other six C
                            // differences to zero. They are NOT uninformative
                            // or unconstrained components; this is not a scalar
                            // BetweenFactor plus global ISB keys.
                            gtsam::Vector sigmas = gtsam::Vector::Zero(
                                static_cast<Eigen::Index>(
                                    kNativeSourceClockVectorDimension));
                            sigmas(0) = kNativeSourceClockC0DSigmaM;
                            graph.emplace_shared<SourceClockVectorC0DFactor>(
                                previous_clock, current_clock,
                                dopplerClockDriftKey(i - 1),
                                dopplerClockDriftKey(i), dt_s,
                                gtsam::noiseModel::Diagonal::Sigmas(sigmas));
                        } else {
                            graph.emplace_shared<SourceClockC0DFactor>(
                                previous_clock, current_clock,
                                dopplerClockDriftKey(i - 1),
                                dopplerClockDriftKey(i), dt_s,
                                gtsam::noiseModel::Isotropic::Sigma(
                                    1, use_native_source_clock_c0d_meter_state
                                           ? kNativeSourceClockC0DSigmaM
                                           : kNativeSourceClockC0DSigmaM /
                                                 gtsam::gnss::C_LIGHT),
                                use_native_source_clock_c0d_meter_state);
                        }
                        if (config.use_native_source_clock_c0d_active_solve_diagnostic) {
                            const double state_scale =
                                clockStateScale(use_native_source_clock_c0d_meter_state);
                            const double sigma_state =
                                use_native_source_clock_c0d_meter_state
                                    ? kNativeSourceClockC0DSigmaM
                                    : kNativeSourceClockC0DSigmaM /
                                          gtsam::gnss::C_LIGHT;
                            const double clock_column_norm = 1.0 / sigma_state;
                            const double drift_column_norm =
                                (dt_s / (2.0 * state_scale)) / sigma_state;
                            result.diagnostics
                                .native_source_clock_c0d_max_whitened_clock_column_norm =
                                std::max(
                                    result.diagnostics
                                        .native_source_clock_c0d_max_whitened_clock_column_norm,
                                    clock_column_norm);
                            result.diagnostics
                                .native_source_clock_c0d_max_whitened_drift_column_norm =
                                std::max(
                                    result.diagnostics
                                        .native_source_clock_c0d_max_whitened_drift_column_norm,
                                    drift_column_norm);
                        }
                        ++result.diagnostics.native_source_clock_c0d_factor_count;
                        break;
                }
                continue;
            }
            const double sigma_m = clock_jump ? 1.0e6 : config.clock_motion_sigma_m;
            graph.emplace_shared<gtsam::BetweenFactor<double>>(
                previous_clock, current_clock, 0.0,
                gtsam::noiseModel::Isotropic::Sigma(
                    1, use_native_source_clock_c0d_meter_state
                           ? sigma_m
                           : sigma_m / gtsam::gnss::C_LIGHT));
            ++result.diagnostics.native_source_clock_c0d_legacy_between_factor_count;
        }
    }

    result.diagnostics.motion_factors =
        (config.use_motion_factors && num_epochs >= 2)
            ? num_epochs - 1
            : 0;

    // The main-stage handoff is generated in memory by this invocation,
    // never loaded positions. Monitoring alone does not alter the graph.
    if ((config.monitor_motion_constraints || config.use_native_batch_nhc) &&
        config.use_native_phase171_raw_p_no_doppler_imu_main) {
        if (problem.imu.stop_velocity_seeds_nav.size() != num_epochs)
            throw std::invalid_argument("Native NHC monitor requires complete same-run velocity seeds");
        const auto& samples = problem.imu.samples_body_flu;
        for (std::size_t k = 1; k < samples.size(); ++k) {
            if (!(samples[k - 1].time < samples[k].time))
                throw std::invalid_argument("Native NHC monitor requires ordered IMU");
        }
        std::size_t cursor = 0, supported = 0, admitted = 0;
        for (std::size_t i = 1; i < num_epochs; ++i) {
            const auto& t0 = problem.epochs[i - 1].time;
            const auto& t1 = problem.epochs[i].time;
            while (cursor < samples.size() && samples[cursor].time < t0) ++cursor;
            const std::size_t begin = cursor;
            while (cursor < samples.size() && samples[cursor].time < t1) ++cursor;
            const auto& velocity = problem.imu.stop_velocity_seeds_nav[i];
            const auto gate = nativeNhcGate(samples, begin, cursor, t0, t1,
                problem.imu.init_gyro_bias, std::hypot(velocity.x(), velocity.y()),
                2.0, 0.2, 0.05);
            supported += gate.supported;
            admitted += gate.admitted;
            if (config.use_native_batch_nhc && gate.admitted) {
                // Pose rotation is body-to-local-ENU; velocity is also ENU.
                // Zero vehicle lever arm is an explicit approximation.
                const auto noise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.3, 0.2)));
                graph.emplace_shared<NonHolonomicFactor>(
                    positionKey(i), velocityKey(i), noise);
                ++result.diagnostics.nhc_epochs;
            }
        }
        std::fprintf(stderr,
            "[native-nhc-monitor] intervals=%zu supported=%zu admitted=%zu "
            "min_speed_mps=2 max_angular_speed_radps=0.2 max_gap_s=0.05 factors_added=%zu\n",
            num_epochs > 0 ? num_epochs - 1 : 0, supported, admitted,
            config.use_native_batch_nhc ? result.diagnostics.nhc_epochs : 0);
    }

    // Pair selection is frozen from the same-run GNSS-first handoff before
    // the main solve. Do not substitute graph velocity fallback or main
    // optimized coordinates, and do not apply the stop-velocity gate here.
    if (config.use_native_relative_height_pairs) {
        if (problem.imu.stop_velocity_seeds_nav.size() != num_epochs ||
            !upstream_stop_detection.ok ||
            upstream_stop_detection.epoch_stop.size() != num_epochs) {
            throw std::invalid_argument("Relative height requires complete native seed and stop coverage");
        }
        std::vector<fusion_initialization::RelativeHeightSeed> seeds;
        seeds.reserve(num_epochs);
        for (std::size_t i = 0; i < num_epochs; ++i) {
            seeds.push_back({problem.epochs[i].time, problem.epochs[i].position_ecef,
                             problem.imu.stop_velocity_seeds_nav[i],
                             upstream_stop_detection.epoch_stop[i]});
        }
        const auto pairs =
            fusion_initialization::selectSourceSampleSumRelativeHeightPairs(seeds);
        result.diagnostics.relative_height_pairs_selected = pairs.size();
        const auto noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(0.5),
            gtsam::noiseModel::Isotropic::Sigma(1, 0.1));
        const gtsam::Vector3 up =
            ecef_T_nav.rotation().rotate(gtsam::Vector3::UnitZ());
        for (const auto& pair : pairs) {
            graph.emplace_shared<RelativeHeightPoseFactor>(
                positionKey(pair.first), positionKey(pair.second), up, gnss_lever_arm, noise);
            ++result.diagnostics.relative_height_factors_inserted;
        }
    }

    // --- Upstream stationary-stop constraints -----------------------------
    // fgo_gnss_imu.m adds a robust zero-velocity prior when the raw IMU stop
    // flag and the preceding GNSS velocity gate both pass, and an identity
    // Pose3 between-factor across two consecutive stops.  The batch path did
    // not previously carry these constraints (the fixed-lag ZUPT knob is a
    // separate contract), so keep this explicitly opt-in and raw-only.
    if (use_upstream_stop_constraints) {
        // Evaluate the existing gate once per detected epoch and retain only
        // scalar provenance.  The factor loops below consume this exact
        // decision, preserving their ordering and numerical recipe while
        // distinguishing a failed candidate from a missing seed.
        std::vector<bool> upstream_stop_velocity_gate_passed(
            num_epochs, false);
        const bool full_stop_seed_vector =
            problem.imu.stop_velocity_seeds_nav.size() == num_epochs;
        for (std::size_t i = 0; i < num_epochs; ++i) {
            if (!upstream_stop_detection.epoch_stop[i]) continue;
            if (!initial.exists(velocityKey(i))) {
                ++result.diagnostics
                      .upstream_stop_velocity_key_missing_epochs;
                continue;
            }
            const gtsam::Vector3 graph_velocity =
                initial.at<gtsam::Vector3>(velocityKey(i));
            const bool have_upstream_seed =
                full_stop_seed_vector &&
                problem.imu.stop_velocity_seeds_nav[i].allFinite();
            if (!full_stop_seed_vector) {
                ++result.diagnostics.upstream_stop_seed_unavailable_epochs;
            } else if (!have_upstream_seed) {
                ++result.diagnostics.upstream_stop_seed_nonfinite_epochs;
            }
            if (!have_upstream_seed) {
                ++result.diagnostics
                      .upstream_stop_graph_velocity_fallback_epochs;
            }
            const double stop_speed = have_upstream_seed
                ? problem.imu.stop_velocity_seeds_nav[i].norm()
                : graph_velocity.norm();
            if (!graph_velocity.allFinite()) {
                ++result.diagnostics
                      .upstream_stop_graph_velocity_nonfinite_epochs;
                continue;
            }
            if (!std::isfinite(stop_speed)) {
                ++result.diagnostics.upstream_stop_speed_nonfinite_epochs;
                continue;
            }
            ++result.diagnostics.upstream_stop_speed_evaluated_epochs;
            if (result.diagnostics.upstream_stop_speed_evaluated_epochs == 1U) {
                result.diagnostics.upstream_stop_speed_min_mps = stop_speed;
                result.diagnostics.upstream_stop_speed_max_mps = stop_speed;
            } else {
                result.diagnostics.upstream_stop_speed_min_mps = std::min(
                    result.diagnostics.upstream_stop_speed_min_mps, stop_speed);
                result.diagnostics.upstream_stop_speed_max_mps = std::max(
                    result.diagnostics.upstream_stop_speed_max_mps, stop_speed);
            }
            if (stop_speed >= config.upstream_stop_velocity_threshold_mps) {
                ++result.diagnostics
                      .upstream_stop_speed_gate_rejected_epochs;
                continue;
            }
            ++result.diagnostics.upstream_stop_speed_gate_accepted_epochs;
            upstream_stop_velocity_gate_passed[i] = true;
        }
        const auto velocity_noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(
                config.upstream_stop_velocity_huber_k_sigma),
            gtsam::noiseModel::Isotropic::Sigma(
                3, config.upstream_stop_velocity_sigma_mps));
        for (std::size_t i = 0; i < num_epochs; ++i) {
            if (!upstream_stop_velocity_gate_passed[i]) {
                continue;
            }
            graph.addPrior<gtsam::Vector3>(
                velocityKey(i), gtsam::Vector3::Zero(), velocity_noise);
            ++result.diagnostics.upstream_stop_velocity_factors;
        }
        gtsam::Vector6 pose_sigmas;
        pose_sigmas << config.upstream_stop_pose_rotation_sigma_rad,
            config.upstream_stop_pose_rotation_sigma_rad,
            config.upstream_stop_pose_rotation_sigma_rad,
            config.upstream_stop_pose_translation_sigma_m,
            config.upstream_stop_pose_translation_sigma_m,
            config.upstream_stop_pose_translation_sigma_m;
        const auto pose_noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(
                config.upstream_stop_pose_huber_k_sigma),
            gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas));
        for (std::size_t i = 0; i + 1U < num_epochs; ++i) {
            if (!upstream_stop_detection.epoch_stop[i] ||
                !upstream_stop_detection.epoch_stop[i + 1U] ||
                !upstream_stop_velocity_gate_passed[i]) {
                continue;
            }
            graph.emplace_shared<gtsam::BetweenFactor<Pose3>>(
                positionKey(i), positionKey(i + 1U), Pose3(), pose_noise);
            ++result.diagnostics.upstream_stop_pose_factors;
        }
    }

    // --- Optional absolute priors (disabled by default: sigma <= 0) ---
    if (config.position_prior_sigma_m > 0.0) {
        if (use_pose3) {
            constexpr double kLooseRotationSigmaRad = 1e6;
            gtsam::Vector6 sigmas;
            sigmas << kLooseRotationSigmaRad, kLooseRotationSigmaRad, kLooseRotationSigmaRad,
                config.position_prior_sigma_m, config.position_prior_sigma_m,
                config.position_prior_sigma_m;
            const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
            for (std::size_t i = 0; i < num_epochs; ++i) {
                graph.addPrior(positionKey(i),
                               Pose3(Rot3(), Point3(problem.epochs[i].position_ecef) - lever_arm_body),
                               noise);
            }
        } else {
            const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, config.position_prior_sigma_m);
            for (std::size_t i = 0; i < num_epochs; ++i) {
                graph.addPrior(positionKey(i), Point3(problem.epochs[i].position_ecef), noise);
            }
        }
    }
    if (need_clock_states && config.clock_prior_sigma_m > 0.0) {
        if (use_native_source_clock_epoch_vector) {
            gtsam::Vector sigmas = gtsam::Vector::Constant(
                static_cast<Eigen::Index>(kNativeSourceClockVectorDimension),
                std::numeric_limits<double>::infinity());
            sigmas(0) = config.clock_prior_sigma_m;
            const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
            for (const gtsam::Key key : inserted_clock_keys) {
                graph.addPrior(key, initial.at<gtsam::Vector>(key), noise);
            }
        } else {
            const auto noise = gtsam::noiseModel::Isotropic::Sigma(
                1, use_native_source_clock_c0d_meter_state
                       ? config.clock_prior_sigma_m
                       : config.clock_prior_sigma_m / constants::SPEED_OF_LIGHT);
            for (const gtsam::Key key : inserted_clock_keys) {
                graph.addPrior(key, initial.at<double>(key), noise);
            }
        }
    }
    if ((use_native_raw_p_seed_graph || phase171_requested) &&
        use_native_source_clock_epoch_vector) {
        // The raw-P adapter certifies C[0] only.  C[1..6] are still real
        // source state slots, but a slot with no retained P row has no
        // observation in this graph.  Give such nuisance slots an explicit,
        // very weak numerical gauge at their zero initial guess.  This is
        // not a measured ISB, and it prevents an inactive C component from
        // becoming an accidental unconstrained variable.  Observed slots
        // remain fully estimated by the source P factor family.  Phase171
        // uses this same source-backed numerical gauge in the main graph;
        // it is not an observability claim or a fabricated clock measurement.
        std::array<bool, kNativeSourceClockVectorDimension>
            observed_clock_components{};
        observed_clock_components.fill(false);
        for (const auto& factor : problem.pseudorange_factors) {
            const int component = sourceClockComponentFor(
                factor.satellite.system, factor.signal);
            if (component >= 0 &&
                static_cast<std::size_t>(component) <
                    kNativeSourceClockVectorDimension) {
                observed_clock_components[static_cast<std::size_t>(component)] =
                    true;
            }
        }
        gtsam::Vector sigmas = gtsam::Vector::Constant(
            static_cast<Eigen::Index>(kNativeSourceClockVectorDimension),
            std::numeric_limits<double>::infinity());
        for (std::size_t component = 1;
             component < kNativeSourceClockVectorDimension; ++component) {
            if (!observed_clock_components[component]) {
                sigmas(static_cast<Eigen::Index>(component)) =
                    config.native_raw_p_no_doppler_unobserved_clock_gauge_sigma_m;
                if (use_native_raw_p_no_doppler_graph) {
                    ++result.diagnostics
                              .native_raw_p_no_doppler_unobserved_clock_gauge_components;
                } else if (use_native_raw_p_ecef_doppler_graph) {
                    // The ECEF raw-P+D staging lane uses the same explicit
                    // numerical C7 gauge, but is reported separately from
                    // the no-D graph provenance.
                    ++result.diagnostics
                              .native_raw_p_ecef_doppler_unobserved_clock_gauge_components;
                } else {
                    ++result.diagnostics
                              .native_phase171_unobserved_clock_gauge_components;
                }
            }
        }
        const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
        for (const gtsam::Key key : inserted_clock_keys) {
            graph.addPrior(key, initial.at<gtsam::Vector>(key), noise);
        }
    }
    if (config.use_receiver_signal_bias_states &&
        config.receiver_signal_bias_prior_sigma_m > 0.0) {
        const auto noise = gtsam::noiseModel::Isotropic::Sigma(
            1, config.receiver_signal_bias_prior_sigma_m);
        for (const int ordinal : inserted_signal_bias_ordinals) {
            graph.addPrior(signalBiasKey(ordinal), 0.0, noise);
        }
    }
    if (use_residual_ionosphere &&
        std::isfinite(config.residual_ionosphere_prior_sigma_m) &&
        config.residual_ionosphere_prior_sigma_m > 0.0) {
        const auto prior_noise = gtsam::noiseModel::Isotropic::Sigma(
            1, config.residual_ionosphere_prior_sigma_m);
        // The first state is a gauge anchor.  Additional segment starts get
        // the same weak physical prior after a clock jump or an unusable
        // interval so no state can drift through a discontinuity.
        graph.addPrior(residualIonosphereKey(0), 0.0, prior_noise);
        for (std::size_t i = 1; i < num_epochs; ++i) {
            const double dt = problem.epochs[i].time - problem.epochs[i - 1].time;
            const bool clock_jump =
                i < problem.clock_jumps.size() && problem.clock_jumps[i];
            const double rw_sigma = residual_ionosphere::randomWalkSigma(
                dt, config.residual_ionosphere_random_walk_sigma_m_per_sqrt_s);
            const bool continuous =
                !clock_jump && std::isfinite(rw_sigma) &&
                (config.residual_ionosphere_max_gap_s <= 0.0 ||
                 (std::isfinite(dt) &&
                  dt <= config.residual_ionosphere_max_gap_s));
            if (continuous) {
                graph.emplace_shared<gtsam::BetweenFactor<double>>(
                    residualIonosphereKey(i - 1), residualIonosphereKey(i), 0.0,
                    gtsam::noiseModel::Isotropic::Sigma(1, rw_sigma));
            } else {
                graph.addPrior(residualIonosphereKey(i), 0.0, prior_noise);
                ++result.diagnostics.residual_ionosphere_resets;
            }
        }
    }
    if (use_gnss_velocity_states || use_imu_doppler_factors) {
        // The Doppler rows normally provide full rank per epoch; this very
        // broad prior is only a numerical gauge guard for a sparse/degenerate
        // epoch and is not a motion or truth-derived constraint.
        constexpr double kBroadVelocityPriorSigmaMps = 1.0e6;
        constexpr double kBroadClockRatePriorSigmaMps = 1.0e6;
        const gtsam::Vector3 zero_velocity = gtsam::Vector3::Zero();
        for (std::size_t i = 0; i < num_epochs; ++i) {
            graph.addPrior(velocityKey(i), zero_velocity,
                           gtsam::noiseModel::Isotropic::Sigma(
                               3, kBroadVelocityPriorSigmaMps));
            if (use_native_source_clock_epoch_vector) {
                const gtsam::Vector zero_clock_rate = gtsam::Vector::Zero(1);
                graph.addPrior(
                    dopplerClockDriftKey(i), zero_clock_rate,
                    gtsam::noiseModel::Isotropic::Sigma(
                        1, kBroadClockRatePriorSigmaMps));
            } else {
                graph.addPrior(dopplerClockDriftKey(i), 0.0,
                               gtsam::noiseModel::Isotropic::Sigma(
                                   1, kBroadClockRatePriorSigmaMps));
            }
        }
    }

    if (config.use_native_joint_ionosphere) {
        if (joint_codes.size()!=problem.pseudorange_factors.size() ||
            joint_carriers.size()!=problem.tdcp_factors.size() || joint_codes.empty() || joint_carriers.empty())
            throw std::invalid_argument("Incomplete joint ionosphere observation coverage");
        std::vector<GNSSTime> times;
        std::vector<gtsam::Key> keys;
        for (std::size_t i=0;i<num_epochs;++i) {
            times.push_back(problem.epochs[i].time); keys.push_back(Symbol('j',i));
        }
        // Physical residual state is not reset merely by receiver clock jumps.
        // Gaps split the process prior; existing accepted TDCP edges stay intact.
        const auto plan=code_ionosphere::planGraph(graph,initial,times,
            std::vector<bool>(num_epochs,false),keys,joint_codes,joint_carriers,
            config.native_joint_ionosphere_anchor_sigma_m,
            config.native_joint_ionosphere_density_m_sqrt_s,
            config.native_joint_ionosphere_max_gap_s);
        for (const auto& replacement:plan.replacements) graph[replacement.first]=replacement.second;
        graph.push_back(plan.priors); initial.insert(plan.states);
        std::fprintf(stderr,"[native-joint-ionosphere] states=%zu priors=%zu code=%zu tdcp=%zu anchor_sigma=%.12g density=%.12g max_gap=%.12g\n",
            keys.size(),plan.priors.size(),joint_codes.size(),joint_carriers.size(),
            config.native_joint_ionosphere_anchor_sigma_m,config.native_joint_ionosphere_density_m_sqrt_s,
            config.native_joint_ionosphere_max_gap_s);
    }
    result.diagnostics.graph_factors = graph.size();
    result.diagnostics.graph_values = initial.size();
    result.diagnostics.native_source_clock_c0d_epoch_vector_state_count =
        use_native_source_clock_epoch_vector ? inserted_clock_keys.size() : 0U;
    result.diagnostics.native_source_clock_c0d_global_isb_state_count =
        inserted_isb_ordinals.size();

    const bool phase96_main_diagnostic =
        config.use_native_source_clock_c0d_phase96_main_diagnostics;
    const bool phase97_singular_system_diagnostic =
        config.use_native_source_clock_c0d_phase97_singular_system_diagnostics;
    const bool phase98_solver_rank_diagnostic =
        config.use_native_source_clock_c0d_phase98_solver_rank_diagnostic;
    if (phase96_main_diagnostic) {
        auto& phase96 =
            result.diagnostics.native_source_clock_c0d_phase96_main;
        phase96.enabled = true;
        // This is a sidecar observation of the exact graph/initial Values
        // handed to the existing optimizer.  It has no path back into the
        // graph or the optimizer parameters.
        collectPhase96InitialGraphDiagnostics(graph, initial, phase96);
    }
    if (phase97_singular_system_diagnostic) {
        auto& phase97 =
            result.diagnostics.native_source_clock_c0d_phase97_singular_system;
        phase97.enabled = true;
        // This is a read-only structural/linearization observation of the
        // exact graph and Values handed to the unchanged optimizer.
        collectPhase97InitialGraphDiagnostics(graph, initial, phase97);
    }

    if (config.use_native_phase171_raw_p_no_doppler_imu_main && use_pose3) {
        std::vector<double> nominal, irls;
        std::vector<double> alignment;
        std::size_t unsupported = 0, adjacent_pairs = 0, same_sign_pairs = 0;
        std::optional<std::pair<std::size_t, double>> previous_alignment;
        std::size_t downweighted = 0, total_rows = 0;
        for (std::size_t epoch = 0; epoch < code_ionosphere_rows.size(); ++epoch) {
            const auto& rows = code_ionosphere_rows[epoch];
            if (rows.empty()) { previous_alignment.reset(); continue; }
            Eigen::MatrixXd nuisance(rows.size(), 13);
            Eigen::VectorXd coefficient(rows.size()), sigma(rows.size()), effective(rows.size()), residual(rows.size());
            for (std::size_t i = 0; i < rows.size(); ++i) {
                nuisance.row(i) = rows[i].nuisance;
                coefficient[i] = rows[i].coefficient;
                sigma[i] = rows[i].sigma;
                effective[i] = rows[i].sigma / std::sqrt(rows[i].weight);
                residual[i] = rows[i].residual;
                downweighted += rows[i].weight < 1.;
                ++total_rows;
            }
            nominal.push_back(pseudorange_information::scalarProjectedInformation(coefficient, nuisance, sigma));
            irls.push_back(pseudorange_information::scalarProjectedInformation(coefficient, nuisance, effective));
            const auto moments = pseudorange_information::scalarResidualProjection(
                coefficient, nuisance, residual, effective);
            const double full_information = (coefficient.array()/effective.array()).matrix().squaredNorm();
            const double full_energy = (residual.array()/effective.array()).matrix().squaredNorm();
            // Numerical degeneracy guard only, not a physical admission gate.
            if (moments.information <= 1e-12*full_information ||
                moments.residual_energy <= 1e-12*full_energy ||
                moments.information <= 0 || moments.residual_energy <= 0) {
                ++unsupported; previous_alignment.reset();
            } else {
                const double correlation = moments.coefficient_residual_dot /
                    std::sqrt(moments.information) / std::sqrt(moments.residual_energy);
                alignment.push_back(correlation);
                if (previous_alignment && previous_alignment->first + 1 == epoch) {
                    const double dt = problem.epochs[epoch].time - problem.epochs[epoch-1].time;
                    if (dt > 0 && dt <= 1.5) {
                        ++adjacent_pairs;
                        same_sign_pairs += correlation * previous_alignment->second > 0;
                    }
                }
                previous_alignment = std::make_pair(epoch, correlation);
            }
        }
        const auto median = [](std::vector<double> x) {
            if (x.empty()) return 0.;
            std::sort(x.begin(), x.end());
            return x.size()%2 ? x[x.size()/2] : .5*(x[x.size()/2-1]+x[x.size()/2]);
        };
        std::fprintf(stderr, "[native-code-ionosphere-irls] epochs=%zu rows=%zu invalid=%zu downweighted=%zu nominal_median=%.12g irls_median=%.12g factors_changed=0\n",
                     nominal.size(), total_rows, code_ionosphere_invalid, downweighted, median(nominal), median(irls));
        std::fprintf(stderr, "[native-code-ionosphere-residual] supported=%zu unsupported=%zu median_alignment=%.12g adjacent_pairs=%zu same_sign_pairs=%zu corrections_applied=0\n",
                     alignment.size(), unsupported, median(alignment), adjacent_pairs, same_sign_pairs);
    }
    // --- Solve ---
    gtsam::LevenbergMarquardtParams params;
    if (std::getenv("GNSSPP_GTSAM_LM_VERBOSE") != nullptr) {
        params.setVerbosityLM("SUMMARY");
    }
    const int configured_max_iterations = std::max(1, config.max_iterations);
    const int effective_max_iterations = phase143EffectiveMaxIterations(
        configured_max_iterations, phase143_requested, phase143_main_scope);
    params.setMaxIterations(effective_max_iterations);
    params.setAbsoluteErrorTol(config.absolute_cost_convergence_threshold > 0.0
                                   ? config.absolute_cost_convergence_threshold
                                   : 1e-10);
    params.setRelativeErrorTol(config.relative_cost_convergence_threshold > 0.0
                                   ? config.relative_cost_convergence_threshold
                                   : 1e-8);
    // This is the sole Phase99 algorithmic selector.  All factors, Values,
    // ordering, LM lambda schedule, tolerances, and iteration bounds above
    // remain exactly as supplied by the existing path.
    selectPhase99MainSolver(params, phase99_qr_selected);
    applyNativeLmLambdaFloor(params, config.use_native_lm_lambda_floor,
        use_native_source_clock_epoch_vector &&
        (config.use_native_raw_p_ecef_doppler_gnss_first ||
         (phase99_qr_selected && config.use_native_phase171_raw_p_no_doppler_imu_main)));
    result.diagnostics.selected_linear_solver_type =
        params.getLinearSolverType();
    result.diagnostics.selected_solver_branch = phase98SolverBranch(params);
    result.diagnostics.selected_elimination_function =
        phase98EliminationFunction(params);

    const bool active_solve_diagnostic =
        config.use_native_source_clock_c0d_active_solve_diagnostic;
    const bool trial_diagnostic = active_solve_diagnostic ||
                                  phase96_main_diagnostic ||
                                  phase97_singular_system_diagnostic ||
                                  phase98_solver_rank_diagnostic;
    const bool trace_diagnostic = trial_diagnostic || phase143_requested;
    const double initial_cost = graph.error(initial);
    GtsamLmActiveSolveOptimizer optimizer(
        graph, initial, params, trace_diagnostic,
        trial_diagnostic, phase98_solver_rank_diagnostic, phase143_requested);
    if (phase96_main_diagnostic) {
        result.diagnostics.native_source_clock_c0d_phase96_main.attempted =
            true;
    }
    gtsam::Values optimized;
    bool solved = true;
    try {
        optimized = optimizer.optimize();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[fgo_gtsam_backend] LM optimize() threw: %s\n", e.what());
        solved = false;
        optimized = initial;
    }
    if (phase98_solver_rank_diagnostic) {
        // Copy only the compact solver-boundary sidecar.  The optimizer,
        // graph, Values, ordering, damping, and failure behavior are already
        // fixed by the call above; this assignment publishes no solution
        // state.
        result.diagnostics.native_source_clock_c0d_phase98_solver_rank =
            optimizer.telemetry().phase98;
    }
    if (config.use_native_joint_ionosphere) {
        if (!solved) throw std::runtime_error("Joint ionosphere solve failed; no initial-state fallback");
        double maximum=0., rms=0., code_maximum=0., tdcp_maximum=0.;
        for (std::size_t i=0;i<num_epochs;++i) {
            const double state=optimized.at<double>(Symbol('j',i));
            if (!std::isfinite(state)) throw std::runtime_error("Nonfinite joint ionosphere state");
            maximum=std::max(maximum,std::abs(state));
            rms=std::hypot(rms,state/std::sqrt(static_cast<double>(num_epochs)));
        }
        for (const auto& b:joint_codes) {
            const double correction=b.coefficient*optimized.at<double>(Symbol('j',b.epoch));
            if (!std::isfinite(correction)) throw std::runtime_error("Nonfinite joint code correction");
            code_maximum=std::max(code_maximum,std::abs(correction));
        }
        for (const auto& b:joint_carriers) {
            const double correction=b.a_previous*optimized.at<double>(Symbol('j',b.previous))-
                                    b.a_current*optimized.at<double>(Symbol('j',b.current));
            if (!std::isfinite(correction)) throw std::runtime_error("Nonfinite joint TDCP correction");
            tdcp_maximum=std::max(tdcp_maximum,std::abs(correction));
        }
        std::fprintf(stderr,"[native-joint-ionosphere-solved] states=%zu max_abs_m=%.12g rms_m=%.12g max_code_m=%.12g max_tdcp_m=%.12g clipped=0\n",
            num_epochs,maximum,rms,code_maximum,tdcp_maximum);
    }
    if (frequency_residual_states && solved) {
        result.tdcp_frequency_corrections.reserve(problem.tdcp_factors.size());
        for (std::size_t index=0; index<problem.tdcp_factors.size(); ++index) {
            const auto& factor=problem.tdcp_factors[index];
            double correction=0.0;
            if (frequency_pair_by_factor[index]!=no_frequency_pair) {
                correction=frequency_alpha_by_factor[index]*optimized.at<double>(
                    Symbol('u',frequency_pair_by_factor[index]));
            }
            if (!std::isfinite(correction)) throw std::invalid_argument("Nonfinite TDCP frequency correction export");
            result.tdcp_frequency_corrections.push_back({factor.previous_epoch_index,
                factor.current_epoch_index,factor.satellite,factor.signal,correction});
        }
    }
    const double final_cost = graph.error(optimized);
    // Nominal single-epoch P geometry, not posterior/full-graph information.
    // Limit to the audited non-affine C7 IMU main factor branch.
    if (solved && phase171_requested && use_imu &&
        use_native_source_clock_epoch_vector && !phase135_requested) {
        std::vector<std::vector<const FGOProcessor::PseudorangeFactor*>> rows(num_epochs);
        for (const auto& factor : problem.pseudorange_factors) {
            rows.at(factor.epoch_index).push_back(&factor);
        }
        double minimum = std::numeric_limits<double>::infinity();
        for (std::size_t epoch = 0; epoch < num_epochs; ++epoch) {
            Eigen::MatrixXd a(rows[epoch].size(), 3), b(rows[epoch].size(), 7);
            Eigen::VectorXd sigma(rows[epoch].size());
            const Point3 antenna = antennaPositionOf(optimized, epoch);
            for (std::size_t j = 0; j < rows[epoch].size(); ++j) {
                const auto& factor = *rows[epoch][j];
                const Vector3d delta = antenna - factor.satellite_position_ecef;
                const double range = delta.norm();
                const int component = sourceClockComponentFor(factor.satellite.system, factor.signal);
                if (!(range > 0) || !std::isfinite(range) || component < 0) {
                    throw std::invalid_argument("Invalid native P information geometry");
                }
                a.row(j) = delta.transpose() / range;
                b.row(j) = sourceClockComponentJacobian(component).transpose();
                sigma(j) = factor.sigma_m;
            }
            const Eigen::Matrix3d info = pseudorange_information::clockProjectedInformation(a,b,sigma);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(info);
            if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite()) {
                throw std::invalid_argument("Invalid native P information eigensystem");
            }
            const double lo = std::max(0.0, eigen.eigenvalues()(0));
            const double hi = std::max(0.0, eigen.eigenvalues()(2));
            ++result.diagnostics.nominal_p_information_epochs;
            if (lo <= std::max(1e-12, hi*1e-10)) {
                ++result.diagnostics.nominal_p_information_rank_deficient_epochs;
                // Observation topology only: no position, bias or satellite
                // state series. Counts do not prove full temporal graph rank.
                std::size_t previous_tdcp = 0, next_tdcp = 0;
                for (const auto& tdcp : problem.tdcp_factors) {
                    if (tdcp.current_epoch_index == epoch) ++previous_tdcp;
                    if (tdcp.previous_epoch_index == epoch) ++next_tdcp;
                }
                const auto clock_rank = b.rows() == 0 ? 0 :
                    Eigen::JacobiSVD<Eigen::MatrixXd>(b).rank();
                std::fprintf(stderr,
                    "[nominal-p-information] epoch=%zu p_rows=%zu clock_rank=%ld "
                    "incoming_tdcp=%zu outgoing_tdcp=%zu min_eigenvalue=%.17g\n",
                    epoch, rows[epoch].size(), static_cast<long>(clock_rank),
                    previous_tdcp, next_tdcp, lo);
            }
            minimum = std::min(minimum,lo);
        }
        if (num_epochs > 0) result.diagnostics.nominal_p_information_min_eigenvalue_per_m2 = minimum;
    }
    // Aggregate-only post-solve diagnostics; no exported bias series or feedback.
    if (use_imu && solved) {
        for (std::size_t i = 0; i < num_epochs; ++i) {
            const auto& bias = optimized.at<gtsam::imuBias::ConstantBias>(biasKey(i));
            const double accel_norm = bias.accelerometer().norm();
            const double gyro_norm = bias.gyroscope().norm();
            if (!std::isfinite(accel_norm) || !std::isfinite(gyro_norm)) {
                throw std::invalid_argument("Nonfinite optimized IMU bias diagnostic");
            }
            ++result.diagnostics.optimized_imu_bias_count;
            result.diagnostics.optimized_accel_bias_max_norm_mps2 = std::max(
                result.diagnostics.optimized_accel_bias_max_norm_mps2, accel_norm);
            result.diagnostics.optimized_gyro_bias_max_norm_radps = std::max(
                result.diagnostics.optimized_gyro_bias_max_norm_radps, gyro_norm);
        }
    }

    if (phase143_requested) {
        const auto& telemetry = optimizer.telemetry();
        const std::string stage = phase143_main_scope ? "main" : "gnss-first";
        result.diagnostics.native_phase143_termination =
            makePhase143TerminationDiagnostics(
                params, telemetry, stage,
                static_cast<std::size_t>(configured_max_iterations),
                true, solved);
        if (!result.diagnostics.native_phase143_termination.configuration_valid) {
            // Phase167 is a diagnostic-only boundary.  Preserve the native
            // scalar sidecar and optimizer counters for the caller, but do
            // not copy Values into any solution/export vector and do not
            // relabel an incomplete trace as a successful solve.  Historical
            // Phase143 behavior remains throwing/fail-closed below.
            if (phase167_requested) {
                result.diagnostics.iterations =
                    static_cast<int>(optimizer.iterations());
                result.diagnostics.converged = false;
                result.diagnostics.initial_cost = initial_cost;
                result.diagnostics.final_cost = final_cost;
                const auto end_time = std::chrono::high_resolution_clock::now();
                result.diagnostics.processing_time_ms =
                    std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                        end_time - start_time)
                        .count();
                result.diagnostics.total_processing_time_ms =
                    result.diagnostics.processing_time_ms;
                return result;
            }
            throw std::invalid_argument(
                "Phase143 termination telemetry failed closed: " +
                result.diagnostics.native_phase143_termination.configuration_failure);
        }
    }

    result.diagnostics.iterations = static_cast<int>(optimizer.iterations());
    result.diagnostics.converged = solved;
    result.diagnostics.initial_cost = initial_cost;
    result.diagnostics.final_cost = final_cost;
    if (active_solve_diagnostic) {
        const auto& telemetry = optimizer.telemetry();
        result.diagnostics
            .native_source_clock_c0d_active_solve_diagnostic_enabled = true;
        result.diagnostics.native_source_clock_c0d_active_solve_attempted =
            telemetry.active_solve_attempted;
        result.diagnostics.native_source_clock_c0d_accepted_outer_iterations =
            telemetry.accepted_outer_iterations;
        result.diagnostics.native_source_clock_c0d_total_inner_lambda_attempts =
            telemetry.total_inner_lambda_attempts;
        result.diagnostics
            .native_source_clock_c0d_indeterminate_linear_solve_count =
            telemetry.indeterminate_linear_solve_count;
        result.diagnostics
            .native_source_clock_c0d_unsuccessful_model_step_count =
            telemetry.unsuccessful_model_step_count;
        result.diagnostics
            .native_source_clock_c0d_small_cost_change_stop_count =
            telemetry.small_cost_change_stop_count;
        result.diagnostics
            .native_source_clock_c0d_maximum_lambda_stop_count =
            telemetry.maximum_lambda_stop_count;
        result.diagnostics.native_source_clock_c0d_initial_lambda =
            telemetry.initial_lambda;
        result.diagnostics.native_source_clock_c0d_maximum_lambda =
            telemetry.maximum_lambda;
        result.diagnostics.native_source_clock_c0d_final_lambda =
            telemetry.final_lambda;
        result.diagnostics.native_source_clock_c0d_active_solve_finite_costs =
            std::isfinite(initial_cost) && std::isfinite(final_cost);
        result.diagnostics
            .native_source_clock_c0d_termination_trace_complete =
            telemetry.termination_trace_complete;
        result.diagnostics.native_source_clock_c0d_termination_branch_reason =
            telemetry.termination_branch_reason;
        if (result.diagnostics
                .native_source_clock_c0d_max_whitened_drift_column_norm > 0.0) {
            result.diagnostics.native_source_clock_c0d_conditioning_proxy =
                result.diagnostics
                    .native_source_clock_c0d_max_whitened_clock_column_norm /
                result.diagnostics
                    .native_source_clock_c0d_max_whitened_drift_column_norm;
        }
    }
    if (phase96_main_diagnostic) {
        const auto& telemetry = optimizer.telemetry();
        auto& phase96 =
            result.diagnostics.native_source_clock_c0d_phase96_main;
        phase96.lm_trials = telemetry.phase96_lm_trials;
        phase96.trial_trace_complete = telemetry.phase96_trace_complete;
        phase96.initial_cost = initial_cost;
        phase96.final_cost = final_cost;
        phase96.accepted_outer_iterations =
            telemetry.accepted_outer_iterations;
        phase96.terminal_branch = telemetry.termination_branch_reason;
        phase96.exceptions.insert(phase96.exceptions.end(),
                                  telemetry.phase96_exceptions.begin(),
                                  telemetry.phase96_exceptions.end());
    }
    if (phase97_singular_system_diagnostic) {
        const auto& telemetry = optimizer.telemetry();
        auto& phase97 =
            result.diagnostics.native_source_clock_c0d_phase97_singular_system;
        phase97.attempted = true;
        phase97.initial_cost = initial_cost;
        phase97.final_cost = final_cost;
        phase97.accepted_outer_iterations =
            telemetry.accepted_outer_iterations;
        phase97.trial_trace_complete = telemetry.phase96_trace_complete;
        phase97.terminal_branch = telemetry.termination_branch_reason;
        phase97.lm_trials.reserve(telemetry.phase96_lm_trials.size());
        for (const auto& source_trial : telemetry.phase96_lm_trials) {
            FGOProcessor::FGOPhase97LmTrialDiagnostics trial;
            trial.trial_index = source_trial.trial_index;
            trial.outer_iteration = source_trial.outer_iteration;
            trial.lambda = source_trial.lambda;
            trial.old_linearized_cost = source_trial.old_linearized_cost;
            trial.new_linearized_cost = source_trial.new_linearized_cost;
            trial.predicted_reduction = source_trial.predicted_reduction;
            trial.candidate_nonlinear_cost = source_trial.candidate_nonlinear_cost;
            trial.actual_reduction = source_trial.actual_reduction;
            trial.model_fidelity = source_trial.model_fidelity;
            trial.candidate_finite = source_trial.candidate_finite;
            trial.linear_system_solved = source_trial.linear_system_solved;
            trial.linear_system_status = source_trial.linear_system_status;
            trial.rejection_reason = source_trial.rejection_reason;
            // The pinned LM implementation catches this exception inside
            // tryLambda without exposing it through its public telemetry.  A
            // fabricated key would be misleading, so every trial records the
            // exact availability boundary explicitly.
            trial.nearby_variable_available = false;
            trial.nearby_variable_status =
                source_trial.linear_system_solved
                    ? "not-an-indeterminate-trial"
                    : "nearby_variable_unavailable";
            phase97.lm_trials.push_back(std::move(trial));
        }
        phase97.exceptions.insert(phase97.exceptions.end(),
                                 telemetry.phase96_exceptions.begin(),
                                 telemetry.phase96_exceptions.end());
        phase97.nearby_variable_capture_status =
            phase97.nearby_variables.empty()
                ? "nearby_variable_unavailable"
                : "captured_from_indeterminant_exception";
    }

    if (use_native_source_clock_epoch_vector) {
        // Export every optimized source-parity C_i component before any
        // output mapping.  The caller uses this exact retained order for the
        // same-run main handoff; partial/nonfinite state export is a hard
        // failure and never falls back to scalar C, raw C, or global ISB.
        result.epoch_clock_bias_components_m.resize(num_epochs);
        for (std::size_t i = 0; i < num_epochs; ++i) {
            const gtsam::Key key = clockKey(i);
            if (!optimized.exists(key)) {
                result.diagnostics.converged = false;
                result.epoch_clock_bias_components_m.clear();
                return result;
            }
            const gtsam::Vector clock = optimized.at<gtsam::Vector>(key);
            if (!finiteSourceClockVector(clock)) {
                result.diagnostics.converged = false;
                result.epoch_clock_bias_components_m.clear();
                return result;
            }
            for (std::size_t component = 0;
                 component < kNativeSourceClockVectorDimension;
                 ++component) {
                result.epoch_clock_bias_components_m[i][component] =
                    clock(static_cast<Eigen::Index>(component));
            }
        }
        result.diagnostics.native_source_clock_c0d_epoch_vector_handoff_count =
            result.epoch_clock_bias_components_m.size();
    }
    if (use_native_source_clock_c0d_factor) {
        // Export the solved D_i states before any per-epoch output mapping.
        // The Phase93 caller uses this vector as the only main-graph D
        // initializer, so a missing/nonfinite key is a hard failure rather
        // than an opportunity to substitute raw, zero, held, interpolated,
        // WLS, or inferred values.
        result.epoch_clock_drift_mps.assign(
            num_epochs, std::numeric_limits<double>::quiet_NaN());
        for (std::size_t i = 0; i < num_epochs; ++i) {
            const gtsam::Key key = dopplerClockDriftKey(i);
            if (!optimized.exists(key)) {
                result.diagnostics.converged = false;
                result.epoch_clock_drift_mps.clear();
                return result;
            }
            const double drift =
                use_native_source_clock_epoch_vector
                    ? optimized.at<gtsam::Vector>(key)(0)
                    : optimized.at<double>(key);
            if (!std::isfinite(drift)) {
                result.diagnostics.converged = false;
                result.epoch_clock_drift_mps.clear();
                return result;
            }
            result.epoch_clock_drift_mps[i] = drift;
        }
    }

    // --- Per-epoch / partial LAMBDA integer fixing ---
    // Mirrors the native backend's per-epoch LAMBDA path (fgo.cpp
    // process_epoch_lambda_work): group each epoch's DD ambiguities, take their
    // joint marginal covariance (here via gtsam::Marginals over the whole
    // graph), run the SAME libgnss::lambdaSearch with the same ratio test, and
    // -- when use_epoch_lambda_fixed_output is set -- snap that epoch's position
    // to the fixed solution via the position/ambiguity cross-covariance. GTSAM
    // ambiguity nodes are already in cycles, so no wavelength scaling is needed.
    std::map<std::size_t, int> fixed_cycles_by_index;      // amb index -> integer cycles
    std::map<std::size_t, double> fixed_residual_by_index;  // amb index -> float-int residual
    std::vector<bool> epoch_fixed(num_epochs, false);
    std::vector<int> epoch_fixed_count(num_epochs, 0);
    std::vector<double> epoch_ratio(num_epochs, 0.0);
    std::vector<Point3> epoch_output_position(num_epochs, Point3(0, 0, 0));
    for (std::size_t i = 0; i < num_epochs; ++i) {
        epoch_output_position[i] = antennaPositionOf(optimized, i);
    }
    std::size_t total_fixed_ambiguities = 0;
    std::size_t covariance_failures = 0;
    double best_ratio = 0.0;

    const bool do_fix =
        solved && config.use_lambda_ambiguity_fix &&
        !problem.double_difference_carrier_factors.empty();
    if (do_fix) {
        // DD ambiguity indices present at each epoch.
        std::map<std::size_t, std::set<std::size_t>> ambiguity_indices_by_epoch;
        for (const auto& factor : problem.double_difference_carrier_factors) {
            if (factor.epoch_index < num_epochs &&
                factor.ambiguity_index < problem.ambiguity_states.size()) {
                ambiguity_indices_by_epoch[factor.epoch_index].insert(factor.ambiguity_index);
            }
        }
        // Same minimum-candidate gate as the native per-epoch path.
        const int min_candidates =
            std::max(6, std::max(1, config.min_fixed_ambiguities + 1));

        gtsam::Marginals marginals;
        bool have_marginals = false;
        try {
            marginals = gtsam::Marginals(graph, optimized, gtsam::Marginals::CHOLESKY);
            have_marginals = true;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[fgo_gtsam_backend] Marginals failed, no fixing: %s\n",
                         e.what());
        }

        for (auto it = ambiguity_indices_by_epoch.begin();
             have_marginals && it != ambiguity_indices_by_epoch.end(); ++it) {
            const std::size_t epoch_index = it->first;
            if (static_cast<int>(it->second.size()) < min_candidates) {
                continue;
            }
            // Deterministic candidate order (satellite/reference), matching the
            // native path's sort.
            std::vector<std::size_t> candidates(it->second.begin(), it->second.end());
            // MF-AR step 1: one band per satellite for the integer ratio test.
            if (config.double_difference_lambda_one_band_per_satellite &&
                candidates.size() > 1) {
                candidates = selectOneBandPerSatellite(candidates, problem);
            }
            std::sort(candidates.begin(), candidates.end(),
                      [&](std::size_t a, std::size_t b) {
                          const auto& sa = problem.ambiguity_states[a];
                          const auto& sb = problem.ambiguity_states[b];
                          return std::tie(sa.satellite, sa.reference_satellite, sa.signal, a) <
                                 std::tie(sb.satellite, sb.reference_satellite, sb.signal, b);
                      });
            const int n = static_cast<int>(candidates.size());

            gtsam::KeyVector keys;
            keys.reserve(candidates.size() + 1);
            keys.push_back(positionKey(epoch_index));
            for (std::size_t idx : candidates) {
                keys.push_back(ambiguityKey(idx));
            }

            Eigen::VectorXd float_amb(n);
            Eigen::MatrixXd q_amb(n, n);
            Eigen::MatrixXd pos_amb(3, n);  // Cov(antenna_position, ambiguity_row)
            bool ok = true;
            try {
                const gtsam::JointMarginal joint = marginals.jointMarginalCovariance(keys);
                const gtsam::Key pos_key = positionKey(epoch_index);
                // Pose3 mode: the joint marginal gives Cov(pose_tangent, amb),
                // a 6x1 block; propagate it through the antenna Jacobian
                // (3x6, constant for this epoch/optimized-value) to get
                // Cov(antenna_position, amb), matching the Point3 case's
                // native 3x1 block below.
                gtsam::Matrix36 H_antenna_pose;
                if (use_pose3) {
                    gtsam::gnss::LeverArm::PoseFrame frame;
                    gnss_lever_arm.antennaPosition(optimized.at<Pose3>(pos_key), &frame);
                    for (int r3 = 0; r3 < 3; ++r3) {
                        gtsam::Matrix13 unit = gtsam::Matrix13::Zero();
                        unit(0, r3) = 1.0;
                        H_antenna_pose.row(r3) = gnss_lever_arm.antennaPoseJacobian(unit, frame);
                    }
                }
                for (int r = 0; r < n && ok; ++r) {
                    const gtsam::Key rk = ambiguityKey(candidates[r]);
                    float_amb(r) = optimized.at<double>(rk);
                    const gtsam::Matrix pr = joint(pos_key, rk);  // 6x1 (Pose3) or 3x1 (Point3)
                    const Eigen::Vector3d pr3 = use_pose3 ? Eigen::Vector3d(H_antenna_pose * pr)
                                                          : Eigen::Vector3d(pr.col(0));
                    pos_amb(0, r) = pr3(0);
                    pos_amb(1, r) = pr3(1);
                    pos_amb(2, r) = pr3(2);
                    for (int c = 0; c < n; ++c) {
                        q_amb(r, c) = joint(rk, ambiguityKey(candidates[c]))(0, 0);
                    }
                }
            } catch (const std::exception&) {
                ok = false;
            }
            if (!ok || !float_amb.allFinite() || !q_amb.allFinite()) {
                ++covariance_failures;
                continue;
            }

            q_amb = 0.5 * (q_amb + q_amb.transpose());
            for (int i = 0; i < n; ++i) {
                q_amb(i, i) += std::max(1e-12, std::abs(q_amb(i, i)) * 1e-9);
            }

            Eigen::VectorXd fixed_amb;
            double ratio = 0.0;
            ++result.diagnostics.lambda_ambiguity_attempts;
            result.diagnostics.lambda_ambiguity_candidates += static_cast<std::size_t>(n);
            const bool solved_lambda = lambdaSearch(float_amb, q_amb, fixed_amb, ratio);
            if (!solved_lambda) {
                continue;
            }
            result.diagnostics.lambda_ambiguity_fix_solved = true;
            best_ratio = std::max(best_ratio, ratio);
            epoch_ratio[epoch_index] = ratio;
            const bool fixed_epoch =
                std::isfinite(ratio) &&
                (config.lambda_ratio_threshold <= 0.0 || ratio > config.lambda_ratio_threshold) &&
                fixed_amb.size() == n;
            if (!fixed_epoch) {
                continue;
            }

            epoch_fixed[epoch_index] = true;
            epoch_fixed_count[epoch_index] = n;
            total_fixed_ambiguities += static_cast<std::size_t>(n);
            result.diagnostics.lambda_ambiguity_fix_used = true;
            result.diagnostics.lambda_ambiguity_used_candidates += static_cast<std::size_t>(n);
            for (int r = 0; r < n; ++r) {
                const int fixed_int = static_cast<int>(std::lround(fixed_amb(r)));
                fixed_cycles_by_index[candidates[r]] = fixed_int;
                fixed_residual_by_index[candidates[r]] = float_amb(r) - static_cast<double>(fixed_int);
            }

            // Position snap to the fixed ambiguities (only when requested):
            //   x_fixed = x_float - Cov(x,N) Cov(N,N)^-1 (N_float - N_fixed)
            if (config.use_epoch_lambda_fixed_output) {
                const Eigen::VectorXd delta = float_amb - fixed_amb;
                const Eigen::LDLT<Eigen::MatrixXd> ldlt(q_amb);
                if (ldlt.info() == Eigen::Success) {
                    const Eigen::VectorXd correction = ldlt.solve(delta);
                    if (correction.allFinite()) {
                        const Eigen::Vector3d pos_delta = pos_amb * correction;
                        if (pos_delta.allFinite()) {
                            epoch_output_position[epoch_index] =
                                antennaPositionOf(optimized, epoch_index) - Point3(pos_delta);
                        }
                    }
                }
            }
        }
    }

    const bool any_fixed = total_fixed_ambiguities > 0;
    result.diagnostics.lambda_ambiguity_ratio = best_ratio;
    result.diagnostics.fixed_solution = any_fixed;
    result.diagnostics.fixed_ambiguities = total_fixed_ambiguities;
    if (covariance_failures > 0) {
        std::fprintf(stderr,
                     "[fgo_gtsam_backend] %zu epoch(s) skipped: non-PSD / marginal failure\n",
                     covariance_failures);
    }

    // --- Map ambiguity estimates ---
    result.ambiguity_estimates.reserve(problem.ambiguity_states.size());
    for (std::size_t i = 0; i < problem.ambiguity_states.size(); ++i) {
        const auto& ambiguity = problem.ambiguity_states[i];
        FGOProcessor::AmbiguityEstimate estimate;
        estimate.satellite = ambiguity.satellite;
        estimate.signal = ambiguity.signal;
        estimate.segment_index = ambiguity.segment_index;
        estimate.wavelength_m = ambiguity.wavelength_m;
        const double value = optimized.at<double>(ambiguityKey(i));
        if (ambiguity.is_double_difference && ambiguity.wavelength_m > 0.0) {
            estimate.ambiguity_cycles = value;
            estimate.ambiguity_m = value * ambiguity.wavelength_m;
        } else {
            estimate.ambiguity_m = value;
            estimate.ambiguity_cycles =
                ambiguity.wavelength_m > 0.0 ? value / ambiguity.wavelength_m : 0.0;
        }
        const auto fixed_it = fixed_cycles_by_index.find(i);
        if (fixed_it != fixed_cycles_by_index.end()) {
            estimate.is_fixed = true;
            estimate.fixed_by_lambda = true;
            estimate.fixed_cycles = fixed_it->second;
            estimate.fixed_ambiguity_m = fixed_it->second * ambiguity.wavelength_m;
            estimate.fix_residual_cycles = fixed_residual_by_index.at(i);
        }
        result.ambiguity_estimates.push_back(estimate);
    }

    // Read-only clock accessors used by public output and residual
    // diagnostics.  A source-parity factor selects the same C component as
    // its graph row; legacy factors retain the scalar/global-ISB convention.
    auto clockBaseStateAt = [&](const gtsam::Values& values,
                                std::size_t epoch) -> double {
        if (use_native_source_clock_epoch_vector) {
            return values.at<gtsam::Vector>(clockKey(epoch))(0);
        }
        return values.at<double>(clockKey(epoch));
    };
    auto clockContributionAt = [&](const gtsam::Values& values,
                                   std::size_t epoch, GNSSSystem system,
                                   SignalType signal) -> double {
        if (use_native_source_clock_epoch_vector) {
            const gtsam::Vector clock =
                values.at<gtsam::Vector>(clockKey(epoch));
            const int component = sourceClockComponentFor(system, signal);
            if (!finiteSourceClockVector(clock) || component < 0) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            return sourceClockComponentJacobian(component).dot(clock);
        }
        double clock_state = values.at<double>(clockKey(epoch));
        const int ordinal = clockGroupOrdinal(
            clockBiasGroup(system), config.use_inter_system_biases);
        if (ordinal != 0 && values.exists(isbKey(ordinal))) {
            clock_state += values.at<double>(isbKey(ordinal));
        }
        return clockStateTerm(clock_state,
                              use_native_source_clock_c0d_meter_state);
    };
    auto clockBaseContributionAt = [&](const gtsam::Values& values,
                                       std::size_t epoch) -> double {
        if (use_native_source_clock_epoch_vector) {
            return values.at<gtsam::Vector>(clockKey(epoch))(0);
        }
        return clockStateTerm(values.at<double>(clockKey(epoch)),
                              use_native_source_clock_c0d_meter_state);
    };

    // --- Map per-epoch solutions ---
    if (config.use_receiver_signal_bias_states) {
        for (const int ordinal : inserted_signal_bias_ordinals) {
            const gtsam::Key key = signalBiasKey(ordinal);
            if (!optimized.exists(key)) continue;
            for (const auto& factor : problem.pseudorange_factors) {
                if (!signal_bias::isEligible(factor.satellite.system, factor.signal) ||
                    signal_bias::ordinal(factor.satellite.system, factor.signal) != ordinal) {
                    continue;
                }
                result.receiver_signal_bias_estimates_m.emplace(
                    std::make_pair(factor.satellite.system, factor.signal),
                    optimized.at<double>(key));
                break;
            }
        }
    }

    if (use_residual_ionosphere) {
        result.residual_ionosphere_estimates_m.assign(
            num_epochs, std::numeric_limits<double>::quiet_NaN());
        double sum_squared = 0.0;
        std::size_t finite_count = 0;
        for (std::size_t i = 0; i < num_epochs; ++i) {
            const gtsam::Key key = residualIonosphereKey(i);
            if (!optimized.exists(key)) continue;
            const double estimate = optimized.at<double>(key);
            result.residual_ionosphere_estimates_m[i] = estimate;
            if (!std::isfinite(estimate)) {
                ++result.diagnostics.residual_ionosphere_invalid_coefficients;
                continue;
            }
            result.diagnostics.residual_ionosphere_max_abs_m = std::max(
                result.diagnostics.residual_ionosphere_max_abs_m,
                std::abs(estimate));
            sum_squared += estimate * estimate;
            ++finite_count;
        }
        result.diagnostics.residual_ionosphere_rms_m =
            finite_count > 0
                ? std::sqrt(sum_squared / static_cast<double>(finite_count))
                : std::numeric_limits<double>::infinity();
        if (!std::isfinite(result.diagnostics.residual_ionosphere_min_coefficient)) {
            result.diagnostics.residual_ionosphere_min_coefficient = 0.0;
        }
    }

    // --- Map per-epoch solutions ---
    const bool have_ambiguities = !problem.ambiguity_states.empty();
    for (std::size_t i = 0; i < num_epochs; ++i) {
        PositionSolution solution;
        solution.time = problem.epochs[i].time;
        if (epoch_fixed[i]) {
            solution.status = SolutionStatus::FIXED;
        } else if (have_ambiguities) {
            solution.status = SolutionStatus::FLOAT;
        } else {
            solution.status = SolutionStatus::SPP;
        }
        // Fixed position snap only takes effect when use_epoch_lambda_fixed_output
        // is set; otherwise epoch_output_position holds the float position.
        solution.position_ecef = epoch_output_position[i];
        if (need_clock_states && optimized.exists(clockKey(i))) {
            const double clock_state = clockBaseStateAt(optimized, i);
            // PositionSolution's public contract is seconds.  The Phase92
            // graph stores C_i in metres only while it is inside GTSAM, so
            // cross this boundary exactly once at export.  Legacy seconds
            // graphs pass through unchanged.
            solution.receiver_clock_bias = publicClockSeconds(
                clock_state, use_native_source_clock_c0d_meter_state);
        }
        solution.num_frequencies = 1;
        solution.ratio = epoch_ratio[i];
        solution.num_fixed_ambiguities = epoch_fixed_count[i];
        solution.iterations = result.diagnostics.iterations;

        double lat = 0.0;
        double lon = 0.0;
        double height = 0.0;
        ecef2geodetic(solution.position_ecef, lat, lon, height);
        solution.position_geodetic = GeodeticCoord(lat, lon, height);

        result.solution.addSolution(solution);
    }

    // --- Milestone 2b: export estimated attitude (roll/pitch/heading, deg)
    // and ENU velocity so callers can confirm attitude is now observable. ---
    if (use_imu) {
        result.epoch_attitude_rpy_deg.resize(num_epochs);
        result.epoch_attitude_rpy_rad.resize(num_epochs);
        result.epoch_velocity_nav_mps.resize(num_epochs);
        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
        for (std::size_t i = 0; i < num_epochs; ++i) {
            const Rot3 R_body_to_nav = optimized.at<Pose3>(positionKey(i)).rotation();
            const gtsam::Vector3 rpy = R_body_to_nav.rpy();
            const gtsam::Matrix3 R = R_body_to_nav.matrix();
            const Eigen::Vector3d fwd = R.col(0);   // body forward in ENU
            const Eigen::Vector3d left = R.col(1);  // body left in ENU
            // heading: clockwise from North (atan2(E, N)); pitch: nose-up from
            // the forward axis Up component; roll: from the left axis Up
            // component. Matches the reference.csv Roll/Pitch/Heading convention.
            double heading = std::atan2(fwd.x(), fwd.y()) * kRadToDeg;
            if (heading < 0.0) heading += 360.0;
            const double pitch =
                std::asin(std::max(-1.0, std::min(1.0, fwd.z()))) * kRadToDeg;
            const double roll = std::atan2(left.z(), R.col(2).z()) * kRadToDeg;
            result.epoch_attitude_rpy_deg[i] = Vector3d(roll, pitch, heading);
            result.epoch_attitude_rpy_rad[i] =
                Vector3d(rpy.x(), rpy.y(), rpy.z());
            result.epoch_velocity_nav_mps[i] =
                Vector3d(optimized.at<gtsam::Vector3>(velocityKey(i)));
        }
    } else if (use_gnss_velocity_states) {
        result.epoch_velocities_ecef_mps.resize(num_epochs);
        for (std::size_t i = 0; i < num_epochs; ++i) {
            const Vector3d velocity_nav(
                optimized.at<gtsam::Vector3>(velocityKey(i)));
            if (use_native_raw_p_seed_graph) {
                // Phase164 XXVV is defined directly against ECEF Point3
                // states; its Vector3 state is therefore ECEF as well.
                result.epoch_velocities_ecef_mps[i] = velocity_nav;
            } else {
                // Existing Doppler/gnss-first graphs retain their ENU state
                // convention and one-way export conversion.
                result.epoch_velocities_ecef_mps[i] = enu2ecef(
                    velocity_nav, gnss_velocity_origin_lat_rad,
                    gnss_velocity_origin_lon_rad);
            }
        }
    }

    // --- Residual RMS diagnostics (recomputed from the optimized state so
    // they are directly comparable with the native backend's fields) ---
    auto accumulate_rms = [](double sum, std::size_t count) {
        return count > 0 ? std::sqrt(sum / static_cast<double>(count)) : 0.0;
    };

    {
        double sum = 0.0;
        std::size_t count = 0;
        for (const auto& factor : problem.double_difference_pseudorange_factors) {
            const Point3 position = antennaPositionOf(optimized, factor.epoch_index);
            const gtsam::gnss::DoubleDifferenceData dd{
                factor.rover_satellite_model.corrected_pseudorange_m,
                factor.base_satellite_model.corrected_pseudorange_m,
                factor.rover_reference_model.corrected_pseudorange_m,
                factor.base_reference_model.corrected_pseudorange_m,
                Point3(factor.rover_satellite_position_ecef),
                Point3(factor.rover_reference_position_ecef),
                Point3(factor.base_satellite_position_ecef),
                Point3(factor.base_reference_position_ecef),
                Point3(factor.base_position_ecef),
            };
            const double residual = dd.observed() - dd.model(position);
            sum += residual * residual;
            ++count;
        }
        result.diagnostics.double_difference_pseudorange_residual_rms_m =
            accumulate_rms(sum, count);
    }
    {
        double sum = 0.0;
        std::size_t count = 0;
        for (const auto& factor : problem.double_difference_carrier_factors) {
            if (factor.ambiguity_index >= problem.ambiguity_states.size()) {
                continue;
            }
            const auto& ambiguity = problem.ambiguity_states[factor.ambiguity_index];
            const Point3 position = antennaPositionOf(optimized, factor.epoch_index);
            const double amb_cycles = optimized.at<double>(ambiguityKey(factor.ambiguity_index));
            const gtsam::gnss::DoubleDifferenceData dd{
                factor.rover_satellite_model.corrected_carrier_m,
                factor.base_satellite_model.corrected_carrier_m,
                factor.rover_reference_model.corrected_carrier_m,
                factor.base_reference_model.corrected_carrier_m,
                Point3(factor.rover_satellite_position_ecef),
                Point3(factor.rover_reference_position_ecef),
                Point3(factor.base_satellite_position_ecef),
                Point3(factor.base_reference_position_ecef),
                Point3(factor.base_position_ecef),
            };
            const double residual =
                dd.observed() - (dd.model(position) + ambiguity.wavelength_m * amb_cycles);
            sum += residual * residual;
            ++count;
        }
        result.diagnostics.double_difference_carrier_residual_rms_m = accumulate_rms(sum, count);
    }
    {
        double sum = 0.0;
        std::size_t count = 0;
        double normalized_sum = 0.0;
        std::size_t normalized_count = 0;
        for (const auto& factor : problem.pseudorange_factors) {
            const Point3 position = antennaPositionOf(optimized, factor.epoch_index);
            const double clock_term = clockContributionAt(
                optimized, factor.epoch_index, factor.satellite.system,
                factor.signal);
            double signal_bias_m = 0.0;
            if (config.use_receiver_signal_bias_states &&
                signal_bias::isEligible(factor.satellite.system, factor.signal)) {
                const int signal_bias_ordinal =
                    signal_bias::ordinal(factor.satellite.system, factor.signal);
                if (signal_bias_ordinal > 0 &&
                    optimized.exists(signalBiasKey(signal_bias_ordinal))) {
                    signal_bias_m = optimized.at<double>(
                        signalBiasKey(signal_bias_ordinal));
                }
            }
            double residual_ionosphere_m = 0.0;
            if (use_residual_ionosphere &&
                optimized.exists(residualIonosphereKey(factor.epoch_index))) {
                residual_ionosphere_m =
                    factor.residual_ionosphere_coefficient *
                    optimized.at<double>(
                        residualIonosphereKey(factor.epoch_index));
            }
            // Plain Euclidean range to match the native backend and the factor
            // model above (satellite positions are already earth-rotation
            // corrected; no additional Sagnac term).
            const double range =
                (Point3(factor.satellite_position_ecef) - position).norm();
            const double predicted = range +
                                     clock_term +
                                     signal_bias_m + residual_ionosphere_m;
            const double residual = factor.corrected_pseudorange_m - predicted;
            sum += residual * residual;
            ++count;
            if (config.use_upstream_observable_quality &&
                std::isfinite(factor.sigma_m) && factor.sigma_m > 0.0) {
                const double normalized = residual / factor.sigma_m;
                if (std::isfinite(normalized)) {
                    normalized_sum += normalized * normalized;
                    ++normalized_count;
                }
            }
        }
        result.diagnostics.residual_rms_m = accumulate_rms(sum, count);
        if (config.use_upstream_observable_quality && normalized_count > 0) {
            result.diagnostics.upstream_pseudorange_normalized_rms = std::sqrt(
                normalized_sum / static_cast<double>(normalized_count));
        }
    }

    // Undifferenced carrier and TDCP diagnostics use the same explicit clock
    // state boundary as their graph factors.  In particular, a metre-valued
    // C_i is added directly to a metre range; it is never multiplied by C
    // here and divided again at output.
    {
        double sum = 0.0;
        std::size_t count = 0;
        for (const auto& factor : problem.carrier_phase_factors) {
            if (factor.epoch_index >= num_epochs ||
                factor.ambiguity_index >= problem.ambiguity_states.size() ||
                !optimized.exists(clockKey(factor.epoch_index)) ||
                !optimized.exists(ambiguityKey(factor.ambiguity_index))) {
                continue;
            }
            const Point3 position = antennaPositionOf(optimized, factor.epoch_index);
            const double ambiguity_m = optimized.at<double>(
                ambiguityKey(factor.ambiguity_index));
            const double range =
                (Point3(factor.satellite_position_ecef) - position).norm();
            const double predicted =
                range + clockBaseContributionAt(optimized, factor.epoch_index) +
                ambiguity_m;
            const double residual = factor.corrected_carrier_m - predicted;
            if (!std::isfinite(residual)) continue;
            sum += residual * residual;
            ++count;
        }
        result.diagnostics.carrier_phase_residual_rms_m =
            accumulate_rms(sum, count);
    }
    {
        double sum = 0.0;
        std::size_t count = 0;
        for (const auto& factor : problem.tdcp_factors) {
            if (factor.previous_epoch_index >= num_epochs ||
                factor.current_epoch_index >= num_epochs ||
                !optimized.exists(clockKey(factor.previous_epoch_index)) ||
                !optimized.exists(clockKey(factor.current_epoch_index))) {
                continue;
            }
            const Point3 previous_position = antennaPositionOf(
                optimized, factor.previous_epoch_index);
            const Point3 current_position = antennaPositionOf(
                optimized, factor.current_epoch_index);
            const double previous_range =
                (Point3(factor.previous_satellite_position_ecef) -
                 previous_position).norm();
            const double current_range =
                (Point3(factor.current_satellite_position_ecef) -
                 current_position).norm();
            const double predicted =
                current_range +
                    clockBaseContributionAt(optimized, factor.current_epoch_index) -
                previous_range -
                    clockBaseContributionAt(optimized, factor.previous_epoch_index);
            double residual = factor.delta_carrier_m - predicted;
            if (frequency_residual_states) {
                const auto index = static_cast<std::size_t>(&factor-problem.tdcp_factors.data());
                if (frequency_pair_by_factor[index] != no_frequency_pair) {
                    residual += frequency_alpha_by_factor[index] * optimized.at<double>(
                        Symbol('u', frequency_pair_by_factor[index]));
                }
            }
            if (!std::isfinite(residual)) continue;
            sum += residual * residual;
            ++count;
        }
        result.diagnostics.tdcp_residual_rms_m = accumulate_rms(sum, count);
    }

    // Receiver-only Doppler post-fit diagnostics use the same native sign
    // contract as UndifferencedDopplerVelocityFactor.  This is evaluated for
    // both the explicit GNSS velocity-state graph and the opt-in IMU graph;
    // the latter's velocity is already in the IMU ENU frame.
    if (use_gnss_velocity_states || use_imu_doppler_factors) {
        double sum = 0.0;
        std::size_t count = 0;
        double normalized_sum = 0.0;
        std::size_t normalized_count = 0;
        for (const auto& factor : problem.undifferenced_doppler_factors) {
            if (factor.epoch_index >= num_epochs ||
                !optimized.exists(velocityKey(factor.epoch_index)) ||
                !optimized.exists(dopplerClockDriftKey(factor.epoch_index)) ||
                !factor.los.allFinite() || !std::isfinite(factor.residual_mps) ||
                !std::isfinite(factor.sigma_mps) || factor.sigma_mps <= 0.0) {
                continue;
            }
            const gtsam::Vector3 los_nav(ecef2enu(
                factor.los, gnss_velocity_origin_lat_rad,
                gnss_velocity_origin_lon_rad));
            const gtsam::Vector3 velocity_nav =
                optimized.at<gtsam::Vector3>(velocityKey(factor.epoch_index));
            const double clock_drift =
                use_native_source_clock_epoch_vector
                    ? optimized.at<gtsam::Vector>(
                          dopplerClockDriftKey(factor.epoch_index))(0)
                    : optimized.at<double>(
                          dopplerClockDriftKey(factor.epoch_index));
            const double residual =
                los_nav.dot(velocity_nav) + clock_drift - factor.residual_mps;
            if (!std::isfinite(residual)) continue;
            sum += residual * residual;
            ++count;
            if (config.use_upstream_observable_quality) {
                const double normalized = residual / factor.sigma_mps;
                if (std::isfinite(normalized)) {
                    normalized_sum += normalized * normalized;
                    ++normalized_count;
                }
            }
        }
        result.diagnostics.undifferenced_doppler_residual_rms_mps =
            accumulate_rms(sum, count);
        if (config.use_upstream_observable_quality && normalized_count > 0) {
            result.diagnostics.upstream_doppler_normalized_rms = std::sqrt(
                normalized_sum / static_cast<double>(normalized_count));
        }
    }

    const auto end_time = std::chrono::high_resolution_clock::now();
    result.diagnostics.processing_time_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end_time -
                                                                               start_time)
            .count();
    result.diagnostics.total_processing_time_ms = result.diagnostics.processing_time_ms;

    return result;
}

}  // namespace libgnss
