#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/rtk_ar_evaluation.hpp>
#include <libgnss++/algorithms/rtk_ar_selection.hpp>
#include <libgnss++/algorithms/disjoint_satellite_fix_evidence.hpp>
#include <libgnss++/algorithms/fix_failure_budget.hpp>
#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/rtk_cp_pr_gate.hpp>
#include <libgnss++/algorithms/rtk_ddpr_anchor.hpp>
#include <libgnss++/algorithms/rtk_measurement.hpp>
#include <libgnss++/algorithms/rtk_selection.hpp>
#include <libgnss++/algorithms/rtk_ins_time_update.hpp>
#include <libgnss++/algorithms/rtk_tdcp_diagnostics.hpp>
#include <libgnss++/algorithms/rtk_update.hpp>
#include <libgnss++/algorithms/spp_velocity.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>

#include "rtk_internal.hpp"

namespace libgnss {

using namespace rtk_internal;

void RTKProcessor::applyLibraryFixedQualityGate(
    PositionSolution& solution) {
    const auto& config = rtk_config_.library_fixed_quality_gate;
    debug_telemetry_.library_fixed_quality_gate_enabled = config.enabled;
    if (!config.enabled) {
        return;
    }
    const PositionSolution original_solution = solution;
    debug_telemetry_.library_fixed_quality_gate_original_status =
        static_cast<int>(original_solution.status);
    debug_telemetry_.library_fixed_quality_gate_original_ecef_x =
        original_solution.position_ecef.x();
    debug_telemetry_.library_fixed_quality_gate_original_ecef_y =
        original_solution.position_ecef.y();
    debug_telemetry_.library_fixed_quality_gate_original_ecef_z =
        original_solution.position_ecef.z();
    debug_telemetry_.library_fixed_quality_gate_original_ratio =
        original_solution.ratio;
    bool promoted_current_epoch_candidate = false;
    const Vector3d primary_candidate_position(
        debug_telemetry_.lambda_shadow_best_ecef_x,
        debug_telemetry_.lambda_shadow_best_ecef_y,
        debug_telemetry_.lambda_shadow_best_ecef_z);
    const auto disjoint_consensus =
        disjoint_satellite_fix_evidence::closestPairConsensus(
            primary_candidate_position,
            external_disjoint_satellite_fix_evidence_
                .partition_a_candidate_ecef,
            external_disjoint_satellite_fix_evidence_
                .partition_b_candidate_ecef);
    double selected_pair_min_ratio =
        std::numeric_limits<double>::quiet_NaN();
    const auto finite_min_ratio = [](double first, double second) {
        return std::isfinite(first) && std::isfinite(second)
                   ? std::min(first, second)
                   : std::numeric_limits<double>::quiet_NaN();
    };
    switch (disjoint_consensus.selected_pair) {
        case disjoint_satellite_fix_evidence::SelectedPair::PRIMARY_A:
            selected_pair_min_ratio = finite_min_ratio(
                debug_telemetry_.full_ratio,
                external_disjoint_satellite_fix_evidence_
                    .partition_a_ratio);
            break;
        case disjoint_satellite_fix_evidence::SelectedPair::PRIMARY_B:
            selected_pair_min_ratio = finite_min_ratio(
                debug_telemetry_.full_ratio,
                external_disjoint_satellite_fix_evidence_
                    .partition_b_ratio);
            break;
        case disjoint_satellite_fix_evidence::SelectedPair::A_B:
            selected_pair_min_ratio = finite_min_ratio(
                external_disjoint_satellite_fix_evidence_
                    .partition_a_ratio,
                external_disjoint_satellite_fix_evidence_
                    .partition_b_ratio);
            break;
        case disjoint_satellite_fix_evidence::SelectedPair::NONE:
            break;
    }
    debug_telemetry_.disjoint_consensus_selected_pair =
        static_cast<int>(disjoint_consensus.selected_pair);
    debug_telemetry_.disjoint_consensus_selected_pair_min_ratio =
        selected_pair_min_ratio;
    safe_fix::Candidate disjoint_state_candidate;
    disjoint_state_candidate.time_s =
        static_cast<double>(solution.time.week) * 604800.0 +
        solution.time.tow;
    if (solution.isValid() && disjoint_consensus.valid) {
        disjoint_state_candidate.correction_m =
            disjoint_consensus.position_ecef -
            primary_candidate_position;
    }
    disjoint_state_candidate.nis_per_observation =
        debug_telemetry_.float_update_nis_per_observation;
    disjoint_state_candidate.prefit_residual_rms_m =
        debug_telemetry_.float_update_prefit_residual_rms_m;
    disjoint_state_candidate.pair_count =
        debug_telemetry_.pair_count;
    disjoint_state_candidate.ambiguity_ratio =
        rtk_config_.disjoint_consensus_use_selected_pair_ratio
            ? selected_pair_min_ratio
            : debug_telemetry_.full_ratio;
    disjoint_state_candidate.independent_consensus_delta_m =
        debug_telemetry_
            .disjoint_satellite_fix_partition_separation_m;
    disjoint_state_candidate.independent_failure_budget_passed =
        debug_telemetry_.safe_fix_shadow_failure_budget_passed;
    disjoint_state_candidate.acquisition_eligible =
        solution.isValid() &&
        disjoint_consensus.valid &&
        debug_telemetry_.disjoint_satellite_fix_evidence_passed &&
        debug_telemetry_.lambda_shadow_ffrt_passed &&
        debug_telemetry_.pair_count >=
            rtk_config_.safe_fix_shadow_minimum_pairs &&
        std::isfinite(
            disjoint_state_candidate.nis_per_observation) &&
        disjoint_state_candidate.nis_per_observation <=
            rtk_config_
                .safe_fix_shadow_maximum_nis_per_observation &&
        std::isfinite(
            disjoint_state_candidate.prefit_residual_rms_m) &&
        disjoint_state_candidate.prefit_residual_rms_m <=
            rtk_config_
                .safe_fix_shadow_maximum_prefit_residual_rms_m;
    const auto disjoint_state_decision =
        disjoint_consensus_state_machine_.update(
            rtk_config_.disjoint_consensus_state_machine,
            disjoint_state_candidate);
    debug_telemetry_.disjoint_consensus_declared_fixed =
        disjoint_state_decision.declared_fixed;
    debug_telemetry_.disjoint_consensus_state =
        static_cast<int>(disjoint_state_decision.state);
    debug_telemetry_.disjoint_consensus_acquisition_streak =
        disjoint_state_decision.acquisition_streak;
    const Vector3d causal_arc_candidate_position(
        debug_telemetry_.lambda_causal_arc_subset_best_ecef_x,
        debug_telemetry_.lambda_causal_arc_subset_best_ecef_y,
        debug_telemetry_.lambda_causal_arc_subset_best_ecef_z);
    disjoint_satellite_fix_evidence::Config
        causal_arc_disjoint_config;
    causal_arc_disjoint_config.maximum_partition_separation_m =
        rtk_config_
            .disjoint_satellite_fix_max_partition_separation_m;
    causal_arc_disjoint_config.maximum_primary_separation_m =
        rtk_config_
            .disjoint_satellite_fix_max_primary_separation_m;
    causal_arc_disjoint_config.covariance_scale =
        rtk_config_.disjoint_satellite_fix_covariance_scale;
    causal_arc_disjoint_config.maximum_nis_per_dimension =
        rtk_config_
            .disjoint_satellite_fix_max_nis_per_dimension;
    causal_arc_disjoint_config.maximum_statistical_separation_m =
        rtk_config_
            .disjoint_satellite_fix_max_statistical_separation_m;
    disjoint_satellite_fix_evidence::Evidence
        causal_arc_disjoint_evidence;
    causal_arc_disjoint_evidence.available =
        external_disjoint_satellite_fix_evidence_.available;
    causal_arc_disjoint_evidence.inputs_verified_disjoint =
        external_disjoint_satellite_fix_evidence_
            .inputs_verified_disjoint;
    causal_arc_disjoint_evidence.primary_ffrt_passed =
        debug_telemetry_.lambda_causal_arc_subset_ffrt_passed &&
        std::isfinite(
            debug_telemetry_.lambda_causal_arc_subset_ratio) &&
        debug_telemetry_.lambda_causal_arc_subset_ratio >=
            rtk_config_.causal_arc_consensus_state_machine
                .minimum_absolute_ratio;
    causal_arc_disjoint_evidence.partition_a_ffrt_passed =
        external_disjoint_satellite_fix_evidence_
            .partition_a_ffrt_passed;
    causal_arc_disjoint_evidence.partition_b_ffrt_passed =
        external_disjoint_satellite_fix_evidence_
            .partition_b_ffrt_passed;
    causal_arc_disjoint_evidence.primary_candidate_ecef =
        causal_arc_candidate_position;
    causal_arc_disjoint_evidence.partition_a_candidate_ecef =
        external_disjoint_satellite_fix_evidence_
            .partition_a_candidate_ecef;
    causal_arc_disjoint_evidence.partition_b_candidate_ecef =
        external_disjoint_satellite_fix_evidence_
            .partition_b_candidate_ecef;
    causal_arc_disjoint_evidence.partition_a_covariance_ecef =
        external_disjoint_satellite_fix_evidence_
            .partition_a_covariance_ecef;
    causal_arc_disjoint_evidence.partition_b_covariance_ecef =
        external_disjoint_satellite_fix_evidence_
            .partition_b_covariance_ecef;
    if (std::isfinite(
            debug_telemetry_.float_position_covariance_trace_m2) &&
        debug_telemetry_.float_position_covariance_trace_m2 > 0.0) {
        causal_arc_disjoint_evidence.primary_covariance_ecef =
            Matrix3d::Identity() *
            (debug_telemetry_.float_position_covariance_trace_m2 /
             3.0);
    }
    const auto causal_arc_disjoint_decision =
        disjoint_satellite_fix_evidence::evaluate(
            causal_arc_disjoint_config,
            causal_arc_disjoint_evidence);
    debug_telemetry_.causal_arc_disjoint_evidence_passed =
        causal_arc_disjoint_decision.passed;
    safe_fix::Candidate causal_arc_state_candidate;
    causal_arc_state_candidate.time_s =
        disjoint_state_candidate.time_s;
    if (solution.isValid() &&
        causal_arc_candidate_position.allFinite()) {
        causal_arc_state_candidate.correction_m =
            causal_arc_candidate_position -
            solution.position_ecef;
    }
    causal_arc_state_candidate.nis_per_observation =
        debug_telemetry_.float_update_nis_per_observation;
    causal_arc_state_candidate.prefit_residual_rms_m =
        debug_telemetry_.float_update_prefit_residual_rms_m;
    causal_arc_state_candidate.pair_count =
        debug_telemetry_.lambda_causal_arc_subset_pair_count;
    causal_arc_state_candidate.ambiguity_ratio =
        debug_telemetry_.lambda_causal_arc_subset_ratio;
    causal_arc_state_candidate.independent_consensus_delta_m =
        std::max(
            causal_arc_disjoint_decision
                .partition_a_primary_separation_m,
            causal_arc_disjoint_decision
                .partition_b_primary_separation_m);
    causal_arc_state_candidate.independent_failure_budget_passed =
        causal_arc_disjoint_decision.passed;
    causal_arc_state_candidate.acquisition_eligible =
        solution.isValid() &&
        causal_arc_disjoint_decision.passed &&
        debug_telemetry_.lambda_causal_arc_subset_pair_count >=
            rtk_config_.safe_fix_shadow_minimum_pairs &&
        std::isfinite(
            causal_arc_state_candidate.nis_per_observation) &&
        causal_arc_state_candidate.nis_per_observation <=
            rtk_config_
                .safe_fix_shadow_maximum_nis_per_observation &&
        std::isfinite(
            causal_arc_state_candidate.prefit_residual_rms_m) &&
        causal_arc_state_candidate.prefit_residual_rms_m <=
            rtk_config_
                .safe_fix_shadow_maximum_prefit_residual_rms_m;
    const auto causal_arc_state_decision =
        causal_arc_consensus_state_machine_.update(
            rtk_config_.causal_arc_consensus_state_machine,
            causal_arc_state_candidate);
    debug_telemetry_.causal_arc_consensus_declared_fixed =
        causal_arc_state_decision.declared_fixed;
    debug_telemetry_.causal_arc_consensus_state =
        static_cast<int>(causal_arc_state_decision.state);
    debug_telemetry_.causal_arc_consensus_acquisition_streak =
        causal_arc_state_decision.acquisition_streak;
    const Vector3d satellite_par_candidate_position(
        debug_telemetry_.lambda_satellite_par_shadow_best_ecef_x,
        debug_telemetry_.lambda_satellite_par_shadow_best_ecef_y,
        debug_telemetry_.lambda_satellite_par_shadow_best_ecef_z);
    auto satellite_par_disjoint_evidence =
        causal_arc_disjoint_evidence;
    satellite_par_disjoint_evidence.primary_ffrt_passed =
        debug_telemetry_.lambda_satellite_par_shadow_ffrt_passed;
    satellite_par_disjoint_evidence.primary_candidate_ecef =
        satellite_par_candidate_position;
    const auto satellite_par_disjoint_decision =
        disjoint_satellite_fix_evidence::evaluate(
            causal_arc_disjoint_config,
            satellite_par_disjoint_evidence);
    debug_telemetry_.satellite_par_disjoint_evidence_passed =
        satellite_par_disjoint_decision.passed;
    safe_fix::Candidate satellite_par_state_candidate;
    satellite_par_state_candidate.time_s =
        disjoint_state_candidate.time_s;
    if (solution.isValid() &&
        satellite_par_candidate_position.allFinite()) {
        satellite_par_state_candidate.correction_m =
            satellite_par_candidate_position -
            solution.position_ecef;
    }
    satellite_par_state_candidate.nis_per_observation =
        debug_telemetry_.float_update_nis_per_observation;
    satellite_par_state_candidate.prefit_residual_rms_m =
        debug_telemetry_.float_update_prefit_residual_rms_m;
    satellite_par_state_candidate.pair_count =
        debug_telemetry_.lambda_satellite_par_shadow_subset_size;
    satellite_par_state_candidate.ambiguity_ratio =
        debug_telemetry_.lambda_satellite_par_shadow_ratio;
    satellite_par_state_candidate.independent_consensus_delta_m =
        std::max(
            satellite_par_disjoint_decision
                .partition_a_primary_separation_m,
            satellite_par_disjoint_decision
                .partition_b_primary_separation_m);
    satellite_par_state_candidate
        .independent_failure_budget_passed =
        satellite_par_disjoint_decision.passed;
    satellite_par_state_candidate.acquisition_eligible =
        solution.isValid() &&
        satellite_par_disjoint_decision.passed &&
        debug_telemetry_.lambda_satellite_par_shadow_subset_size >=
            rtk_config_.safe_fix_shadow_minimum_pairs &&
        std::isfinite(
            satellite_par_state_candidate.nis_per_observation) &&
        satellite_par_state_candidate.nis_per_observation <=
            rtk_config_
                .safe_fix_shadow_maximum_nis_per_observation &&
        std::isfinite(
            satellite_par_state_candidate
                .prefit_residual_rms_m) &&
        satellite_par_state_candidate.prefit_residual_rms_m <=
            rtk_config_
                .safe_fix_shadow_maximum_prefit_residual_rms_m;
    const auto satellite_par_state_decision =
        satellite_par_consensus_state_machine_.update(
            rtk_config_.satellite_par_consensus_state_machine,
            satellite_par_state_candidate);
    debug_telemetry_.satellite_par_consensus_declared_fixed =
        satellite_par_state_decision.declared_fixed;
    debug_telemetry_.satellite_par_consensus_state =
        static_cast<int>(satellite_par_state_decision.state);
    debug_telemetry_.satellite_par_consensus_acquisition_streak =
        satellite_par_state_decision.acquisition_streak;
    const Vector3d src_par_candidate_position(
        debug_telemetry_.lambda_src_par_shadow_best_ecef_x,
        debug_telemetry_.lambda_src_par_shadow_best_ecef_y,
        debug_telemetry_.lambda_src_par_shadow_best_ecef_z);
    auto src_par_disjoint_evidence =
        causal_arc_disjoint_evidence;
    src_par_disjoint_evidence.primary_ffrt_passed =
        debug_telemetry_.lambda_src_par_shadow_ffrt_passed;
    src_par_disjoint_evidence.primary_candidate_ecef =
        src_par_candidate_position;
    const auto src_par_disjoint_decision =
        disjoint_satellite_fix_evidence::evaluate(
            causal_arc_disjoint_config,
            src_par_disjoint_evidence);
    debug_telemetry_.src_par_disjoint_evidence_passed =
        src_par_disjoint_decision.passed;
    debug_telemetry_.src_par_partition_a_separation_m =
        src_par_disjoint_decision.partition_a_primary_separation_m;
    debug_telemetry_.src_par_partition_b_separation_m =
        src_par_disjoint_decision.partition_b_primary_separation_m;
    safe_fix::Candidate src_par_state_candidate;
    src_par_state_candidate.time_s =
        disjoint_state_candidate.time_s;
    if (solution.isValid() && src_par_candidate_position.allFinite()) {
        src_par_state_candidate.correction_m =
            src_par_candidate_position - solution.position_ecef;
    }
    src_par_state_candidate.nis_per_observation =
        debug_telemetry_.float_update_nis_per_observation;
    src_par_state_candidate.prefit_residual_rms_m =
        debug_telemetry_.float_update_prefit_residual_rms_m;
    src_par_state_candidate.pair_count =
        debug_telemetry_.lambda_src_par_shadow_subset_size;
    src_par_state_candidate.ambiguity_ratio =
        debug_telemetry_.lambda_src_par_shadow_ratio;
    src_par_state_candidate.independent_consensus_delta_m =
        std::max(
            src_par_disjoint_decision
                .partition_a_primary_separation_m,
            src_par_disjoint_decision
                .partition_b_primary_separation_m);
    src_par_state_candidate.independent_failure_budget_passed =
        src_par_disjoint_decision.passed;
    const bool src_par_hard_primary_separation_passed =
        std::isfinite(
            src_par_disjoint_decision
                .partition_a_primary_separation_m) &&
        std::isfinite(
            src_par_disjoint_decision
                .partition_b_primary_separation_m) &&
        src_par_disjoint_decision
                .partition_a_primary_separation_m <=
            rtk_config_
                .disjoint_satellite_fix_max_primary_separation_m &&
        src_par_disjoint_decision
                .partition_b_primary_separation_m <=
            rtk_config_
                .disjoint_satellite_fix_max_primary_separation_m;
    const bool src_par_current_epoch_eligible =
        solution.isValid() &&
        src_par_disjoint_decision.passed &&
        src_par_hard_primary_separation_passed &&
        debug_telemetry_.lambda_src_par_shadow_ffrt_passed &&
        std::isfinite(
            debug_telemetry_
                .lambda_src_par_shadow_second_position_delta_m) &&
        debug_telemetry_
                .lambda_src_par_shadow_second_position_delta_m <=
            rtk_config_
                .safe_fix_shadow_maximum_second_position_delta_m &&
        debug_telemetry_.lambda_src_par_shadow_subset_size >=
            rtk_config_.safe_fix_shadow_minimum_pairs &&
        std::isfinite(src_par_state_candidate.nis_per_observation) &&
        src_par_state_candidate.nis_per_observation <=
            rtk_config_
                .safe_fix_shadow_maximum_nis_per_observation &&
        std::isfinite(
            src_par_state_candidate.prefit_residual_rms_m) &&
        src_par_state_candidate.prefit_residual_rms_m <=
            rtk_config_
                .safe_fix_shadow_maximum_prefit_residual_rms_m;
    src_par_state_candidate.acquisition_eligible =
        src_par_current_epoch_eligible;
    const auto src_par_state_decision =
        src_par_consensus_state_machine_.update(
            rtk_config_.src_par_consensus_state_machine,
            src_par_state_candidate);
    debug_telemetry_.src_par_consensus_declared_fixed =
        src_par_state_decision.declared_fixed;
    debug_telemetry_.src_par_consensus_state =
        static_cast<int>(src_par_state_decision.state);
    debug_telemetry_.src_par_consensus_acquisition_streak =
        src_par_state_decision.acquisition_streak;
    safe_fix::Candidate inertial_referenced_state_candidate;
    inertial_referenced_state_candidate.time_s =
        disjoint_state_candidate.time_s;
    if (primary_candidate_position.allFinite() &&
        external_inertial_fix_evidence_.position_ecef.allFinite()) {
        inertial_referenced_state_candidate.correction_m =
            primary_candidate_position -
            external_inertial_fix_evidence_.position_ecef;
        debug_telemetry_.inertial_referenced_correction_x =
            inertial_referenced_state_candidate.correction_m.x();
        debug_telemetry_.inertial_referenced_correction_y =
            inertial_referenced_state_candidate.correction_m.y();
        debug_telemetry_.inertial_referenced_correction_z =
            inertial_referenced_state_candidate.correction_m.z();
    }
    inertial_referenced_state_candidate.nis_per_observation =
        debug_telemetry_.float_update_nis_per_observation;
    inertial_referenced_state_candidate.prefit_residual_rms_m =
        debug_telemetry_.float_update_prefit_residual_rms_m;
    inertial_referenced_state_candidate.pair_count =
        debug_telemetry_.pair_count;
    inertial_referenced_state_candidate.ambiguity_ratio =
        debug_telemetry_.full_ratio;
    inertial_referenced_state_candidate
        .independent_consensus_delta_m =
        debug_telemetry_.inertial_fix_evidence_position_delta_m;
    inertial_referenced_state_candidate
        .independent_failure_budget_passed =
        debug_telemetry_.inertial_fix_evidence_passed &&
        debug_telemetry_.safe_fix_shadow_failure_budget_passed;
    inertial_referenced_state_candidate.acquisition_eligible =
        solution.isValid() &&
        debug_telemetry_.lambda_shadow_ffrt_passed &&
        debug_telemetry_.inertial_fix_evidence_passed &&
        debug_telemetry_.pair_count >=
            rtk_config_.safe_fix_shadow_minimum_pairs &&
        std::isfinite(
            inertial_referenced_state_candidate
                .nis_per_observation) &&
        inertial_referenced_state_candidate.nis_per_observation <=
            rtk_config_
                .safe_fix_shadow_maximum_nis_per_observation &&
        std::isfinite(
            inertial_referenced_state_candidate
                .prefit_residual_rms_m) &&
        inertial_referenced_state_candidate.prefit_residual_rms_m <=
            rtk_config_
                .safe_fix_shadow_maximum_prefit_residual_rms_m;
    const auto inertial_referenced_state_decision =
        inertial_referenced_consensus_state_machine_.update(
            rtk_config_.inertial_referenced_consensus_state_machine,
            inertial_referenced_state_candidate);
    debug_telemetry_
        .inertial_referenced_consensus_declared_fixed =
        inertial_referenced_state_decision.declared_fixed;
    debug_telemetry_.inertial_referenced_consensus_state =
        static_cast<int>(inertial_referenced_state_decision.state);
    debug_telemetry_
        .inertial_referenced_consensus_acquisition_streak =
        inertial_referenced_state_decision.acquisition_streak;
    const Vector3d multifrequency_candidate_position(
        debug_telemetry_.lambda_l1_l5_wlnl_shadow_best_ecef_x,
        debug_telemetry_.lambda_l1_l5_wlnl_shadow_best_ecef_y,
        debug_telemetry_.lambda_l1_l5_wlnl_shadow_best_ecef_z);
    auto multifrequency_disjoint_evidence =
        causal_arc_disjoint_evidence;
    multifrequency_disjoint_evidence.primary_ffrt_passed =
        debug_telemetry_.lambda_l1_l5_wlnl_shadow_wl_ffrt_passed &&
        debug_telemetry_.lambda_l1_l5_wlnl_shadow_nl_ffrt_passed;
    multifrequency_disjoint_evidence.primary_candidate_ecef =
        multifrequency_candidate_position;
    const auto multifrequency_disjoint_decision =
        disjoint_satellite_fix_evidence::evaluate(
            causal_arc_disjoint_config,
            multifrequency_disjoint_evidence);
    debug_telemetry_.multifrequency_disjoint_evidence_passed =
        multifrequency_disjoint_decision.passed;
    safe_fix::Candidate multifrequency_state_candidate;
    multifrequency_state_candidate.time_s =
        disjoint_state_candidate.time_s;
    if (solution.isValid() &&
        multifrequency_candidate_position.allFinite()) {
        multifrequency_state_candidate.correction_m =
            multifrequency_candidate_position -
            solution.position_ecef;
    }
    multifrequency_state_candidate.nis_per_observation =
        debug_telemetry_.float_update_nis_per_observation;
    multifrequency_state_candidate.prefit_residual_rms_m =
        debug_telemetry_.float_update_prefit_residual_rms_m;
    multifrequency_state_candidate.pair_count =
        debug_telemetry_
            .lambda_l1_l5_wlnl_shadow_candidate_pair_count;
    multifrequency_state_candidate.ambiguity_ratio =
        debug_telemetry_.lambda_l1_l5_wlnl_shadow_nl_ratio;
    multifrequency_state_candidate.independent_consensus_delta_m =
        std::max(
            multifrequency_disjoint_decision
                .partition_a_primary_separation_m,
            multifrequency_disjoint_decision
                .partition_b_primary_separation_m);
    multifrequency_state_candidate
        .independent_failure_budget_passed =
        multifrequency_disjoint_decision.passed;
    multifrequency_state_candidate.acquisition_eligible =
        solution.isValid() &&
        multifrequency_disjoint_decision.passed &&
        multifrequency_state_candidate.pair_count >=
            rtk_config_.safe_fix_shadow_minimum_pairs &&
        std::isfinite(
            multifrequency_state_candidate.nis_per_observation) &&
        multifrequency_state_candidate.nis_per_observation <=
            rtk_config_
                .safe_fix_shadow_maximum_nis_per_observation &&
        std::isfinite(
            multifrequency_state_candidate.prefit_residual_rms_m) &&
        multifrequency_state_candidate.prefit_residual_rms_m <=
            rtk_config_
                .safe_fix_shadow_maximum_prefit_residual_rms_m;
    const auto multifrequency_state_decision =
        multifrequency_consensus_state_machine_.update(
            rtk_config_.multifrequency_consensus_state_machine,
            multifrequency_state_candidate);
    debug_telemetry_.multifrequency_consensus_declared_fixed =
        multifrequency_state_decision.declared_fixed;
    debug_telemetry_.multifrequency_consensus_state =
        static_cast<int>(multifrequency_state_decision.state);
    debug_telemetry_.multifrequency_consensus_acquisition_streak =
        multifrequency_state_decision.acquisition_streak;
    const Vector3d l1_l2_multifrequency_candidate_position(
        debug_telemetry_.lambda_l1_l2_wlnl_shadow_best_ecef_x,
        debug_telemetry_.lambda_l1_l2_wlnl_shadow_best_ecef_y,
        debug_telemetry_.lambda_l1_l2_wlnl_shadow_best_ecef_z);
    auto l1_l2_multifrequency_disjoint_evidence =
        causal_arc_disjoint_evidence;
    l1_l2_multifrequency_disjoint_evidence.primary_ffrt_passed =
        debug_telemetry_.lambda_l1_l2_wlnl_shadow_wl_ffrt_passed &&
        debug_telemetry_.lambda_l1_l2_wlnl_shadow_nl_ffrt_passed;
    l1_l2_multifrequency_disjoint_evidence.primary_candidate_ecef =
        l1_l2_multifrequency_candidate_position;
    const auto l1_l2_multifrequency_disjoint_decision =
        disjoint_satellite_fix_evidence::evaluate(
            causal_arc_disjoint_config,
            l1_l2_multifrequency_disjoint_evidence);
    debug_telemetry_.l1_l2_multifrequency_disjoint_evidence_passed =
        l1_l2_multifrequency_disjoint_decision.passed;
    safe_fix::Candidate l1_l2_multifrequency_state_candidate;
    l1_l2_multifrequency_state_candidate.time_s =
        disjoint_state_candidate.time_s;
    if (solution.isValid() &&
        l1_l2_multifrequency_candidate_position.allFinite()) {
        l1_l2_multifrequency_state_candidate.correction_m =
            l1_l2_multifrequency_candidate_position -
            solution.position_ecef;
    }
    l1_l2_multifrequency_state_candidate.nis_per_observation =
        debug_telemetry_.float_update_nis_per_observation;
    l1_l2_multifrequency_state_candidate.prefit_residual_rms_m =
        debug_telemetry_.float_update_prefit_residual_rms_m;
    l1_l2_multifrequency_state_candidate.pair_count =
        debug_telemetry_
            .lambda_l1_l2_wlnl_shadow_candidate_pair_count;
    l1_l2_multifrequency_state_candidate.ambiguity_ratio =
        debug_telemetry_.lambda_l1_l2_wlnl_shadow_nl_ratio;
    l1_l2_multifrequency_state_candidate
        .independent_consensus_delta_m =
        std::max(
            l1_l2_multifrequency_disjoint_decision
                .partition_a_primary_separation_m,
            l1_l2_multifrequency_disjoint_decision
                .partition_b_primary_separation_m);
    l1_l2_multifrequency_state_candidate
        .independent_failure_budget_passed =
        l1_l2_multifrequency_disjoint_decision.passed;
    l1_l2_multifrequency_state_candidate.acquisition_eligible =
        solution.isValid() &&
        l1_l2_multifrequency_disjoint_decision.passed &&
        l1_l2_multifrequency_state_candidate.pair_count >=
            rtk_config_.safe_fix_shadow_minimum_pairs &&
        std::isfinite(
            l1_l2_multifrequency_state_candidate
                .nis_per_observation) &&
        l1_l2_multifrequency_state_candidate
                .nis_per_observation <=
            rtk_config_
                .safe_fix_shadow_maximum_nis_per_observation &&
        std::isfinite(
            l1_l2_multifrequency_state_candidate
                .prefit_residual_rms_m) &&
        l1_l2_multifrequency_state_candidate
                .prefit_residual_rms_m <=
            rtk_config_
                .safe_fix_shadow_maximum_prefit_residual_rms_m;
    const auto l1_l2_multifrequency_state_decision =
        l1_l2_multifrequency_consensus_state_machine_.update(
            rtk_config_.multifrequency_consensus_state_machine,
            l1_l2_multifrequency_state_candidate);
    debug_telemetry_
        .l1_l2_multifrequency_consensus_declared_fixed =
        l1_l2_multifrequency_state_decision.declared_fixed;
    debug_telemetry_.l1_l2_multifrequency_consensus_state =
        static_cast<int>(l1_l2_multifrequency_state_decision.state);
    debug_telemetry_
        .l1_l2_multifrequency_consensus_acquisition_streak =
        l1_l2_multifrequency_state_decision.acquisition_streak;
    const bool safe_shadow_candidate =
        config.allow_safe_shadow_promotion &&
        debug_telemetry_.safe_fix_shadow_declared_fixed;
    const bool disjoint_state_safe_candidate =
        config.allow_safe_shadow_promotion &&
        disjoint_state_decision.declared_fixed &&
        disjoint_consensus.valid;
    fixed_quality_gate::Evidence original_quality_evidence;
    original_quality_evidence.safe_fix_shadow_declared_fixed =
        debug_telemetry_.safe_fix_shadow_declared_fixed ||
        disjoint_state_decision.declared_fixed;
    original_quality_evidence.independent_failure_budget_passed =
        debug_telemetry_.safe_fix_shadow_failure_budget_passed;
    original_quality_evidence.float_position_covariance_trace_m2 =
        debug_telemetry_.float_position_covariance_trace_m2;
    original_quality_evidence.update_observations =
        debug_telemetry_.float_update_observation_count;
    original_quality_evidence.suppressed_outliers =
        debug_telemetry_.float_update_suppressed_outliers;
    original_quality_evidence.update_nis_per_observation =
        debug_telemetry_.float_update_nis_per_observation;
    const bool replace_rejected_original_fixed_candidate =
        solution.isFixed() &&
        !fixed_quality_gate::evaluate(
             config, original_quality_evidence).passed;
    const bool causal_arc_state_safe_candidate =
        rtk_config_.causal_arc_consensus_promotion &&
        causal_arc_state_decision.declared_fixed &&
        causal_arc_disjoint_decision.passed &&
        causal_arc_candidate_position.allFinite();
    const bool causal_arc_promotion_candidate =
        causal_arc_state_safe_candidate &&
        (!solution.isFixed() ||
         replace_rejected_original_fixed_candidate);
    const bool satellite_par_state_safe_candidate =
        rtk_config_.satellite_par_consensus_promotion &&
        satellite_par_state_decision.declared_fixed &&
        satellite_par_disjoint_decision.passed &&
        satellite_par_candidate_position.allFinite();
    const bool satellite_par_promotion_candidate =
        satellite_par_state_safe_candidate &&
        (!solution.isFixed() ||
         replace_rejected_original_fixed_candidate) &&
        !causal_arc_promotion_candidate;
    const bool src_par_state_safe_candidate =
        rtk_config_.src_par_consensus_promotion &&
        src_par_state_decision.declared_fixed &&
        src_par_current_epoch_eligible &&
        src_par_candidate_position.allFinite();
    const bool src_par_promotion_candidate =
        src_par_state_safe_candidate &&
        (!solution.isFixed() ||
         replace_rejected_original_fixed_candidate) &&
        !causal_arc_promotion_candidate &&
        !satellite_par_promotion_candidate;
    const bool inertial_referenced_state_safe_candidate =
        rtk_config_.inertial_referenced_consensus_promotion &&
        inertial_referenced_state_decision.declared_fixed &&
        debug_telemetry_.inertial_fix_evidence_passed &&
        primary_candidate_position.allFinite();
    const bool inertial_referenced_promotion_candidate =
        inertial_referenced_state_safe_candidate &&
        (!solution.isFixed() ||
         replace_rejected_original_fixed_candidate) &&
        !causal_arc_promotion_candidate &&
        !satellite_par_promotion_candidate &&
        !src_par_promotion_candidate;
    const bool multifrequency_state_safe_candidate =
        rtk_config_.multifrequency_consensus_promotion &&
        multifrequency_state_decision.declared_fixed &&
        multifrequency_disjoint_decision.passed &&
        multifrequency_candidate_position.allFinite();
    const bool multifrequency_promotion_candidate =
        multifrequency_state_safe_candidate &&
        (!solution.isFixed() ||
         replace_rejected_original_fixed_candidate) &&
        !causal_arc_promotion_candidate &&
        !satellite_par_promotion_candidate &&
        !src_par_promotion_candidate &&
        !inertial_referenced_promotion_candidate;
    const bool l1_l2_multifrequency_state_safe_candidate =
        rtk_config_.multifrequency_consensus_promotion &&
        l1_l2_multifrequency_state_decision.declared_fixed &&
        l1_l2_multifrequency_disjoint_decision.passed &&
        l1_l2_multifrequency_candidate_position.allFinite();
    const bool l1_l2_multifrequency_promotion_candidate =
        l1_l2_multifrequency_state_safe_candidate &&
        (!solution.isFixed() ||
         replace_rejected_original_fixed_candidate) &&
        !causal_arc_promotion_candidate &&
        !satellite_par_promotion_candidate &&
        !src_par_promotion_candidate &&
        !inertial_referenced_promotion_candidate &&
        !multifrequency_promotion_candidate;
    const bool disjoint_consensus_candidate =
        config.allow_failure_budget_candidate_promotion &&
        debug_telemetry_.disjoint_satellite_fix_evidence_passed;
    const bool independent_replacement_candidate =
        causal_arc_promotion_candidate ||
        satellite_par_promotion_candidate ||
        src_par_promotion_candidate ||
        inertial_referenced_promotion_candidate ||
        multifrequency_promotion_candidate ||
        l1_l2_multifrequency_promotion_candidate;
    if ((!solution.isFixed() ||
         independent_replacement_candidate ||
         disjoint_state_safe_candidate) &&
        (safe_shadow_candidate ||
         disjoint_state_safe_candidate ||
         causal_arc_promotion_candidate ||
         satellite_par_promotion_candidate ||
         src_par_promotion_candidate ||
         inertial_referenced_promotion_candidate ||
         multifrequency_promotion_candidate ||
         l1_l2_multifrequency_promotion_candidate ||
         disjoint_consensus_candidate) &&
        solution.isValid() &&
        ((causal_arc_promotion_candidate &&
          causal_arc_disjoint_decision.passed) ||
         (satellite_par_promotion_candidate &&
          satellite_par_disjoint_decision.passed) ||
         (src_par_promotion_candidate &&
          src_par_disjoint_decision.passed) ||
         (inertial_referenced_promotion_candidate &&
          debug_telemetry_.inertial_fix_evidence_passed) ||
         (multifrequency_promotion_candidate &&
          multifrequency_disjoint_decision.passed) ||
         (l1_l2_multifrequency_promotion_candidate &&
          l1_l2_multifrequency_disjoint_decision.passed) ||
         (debug_telemetry_.safe_fix_shadow_failure_budget_passed &&
          debug_telemetry_.lambda_shadow_ffrt_passed)) &&
        (safe_shadow_candidate ||
         disjoint_state_safe_candidate ||
         causal_arc_promotion_candidate ||
         satellite_par_promotion_candidate ||
         src_par_promotion_candidate ||
         inertial_referenced_promotion_candidate ||
         multifrequency_promotion_candidate ||
         l1_l2_multifrequency_promotion_candidate ||
         disjoint_consensus_candidate)) {
        Vector3d promoted_position = primary_candidate_position;
        if (causal_arc_promotion_candidate) {
            promoted_position = causal_arc_candidate_position;
            debug_telemetry_
                .library_fixed_quality_gate_causal_arc_promoted = true;
            debug_telemetry_
                .safe_fix_shadow_failure_budget_passed = true;
            debug_telemetry_
                .safe_fix_shadow_independent_source_families = 2;
            debug_telemetry_
                .safe_fix_shadow_joint_failure_probability = 1e-6;
        } else if (satellite_par_promotion_candidate) {
            promoted_position = satellite_par_candidate_position;
            debug_telemetry_
                .library_fixed_quality_gate_satellite_par_promoted =
                true;
            debug_telemetry_
                .safe_fix_shadow_failure_budget_passed = true;
            debug_telemetry_
                .safe_fix_shadow_independent_source_families = 2;
            debug_telemetry_
                .safe_fix_shadow_joint_failure_probability = 1e-6;
        } else if (src_par_promotion_candidate) {
            promoted_position = src_par_candidate_position;
            debug_telemetry_
                .library_fixed_quality_gate_src_par_promoted = true;
            debug_telemetry_
                .safe_fix_shadow_failure_budget_passed = true;
            debug_telemetry_
                .safe_fix_shadow_independent_source_families = 2;
            debug_telemetry_
                .safe_fix_shadow_joint_failure_probability = 1e-6;
        } else if (inertial_referenced_promotion_candidate) {
            promoted_position = primary_candidate_position;
            debug_telemetry_
                .library_fixed_quality_gate_inertial_referenced_promoted =
                true;
            debug_telemetry_
                .safe_fix_shadow_failure_budget_passed = true;
            debug_telemetry_
                .safe_fix_shadow_independent_source_families = 2;
            debug_telemetry_
                .safe_fix_shadow_joint_failure_probability = 1e-6;
        } else if (multifrequency_promotion_candidate) {
            promoted_position = multifrequency_candidate_position;
            debug_telemetry_
                .library_fixed_quality_gate_multifrequency_promoted =
                true;
            debug_telemetry_
                .safe_fix_shadow_failure_budget_passed = true;
            debug_telemetry_
                .safe_fix_shadow_independent_source_families = 2;
            debug_telemetry_
                .safe_fix_shadow_joint_failure_probability = 1e-6;
        } else if (l1_l2_multifrequency_promotion_candidate) {
            promoted_position =
                l1_l2_multifrequency_candidate_position;
            debug_telemetry_
                .library_fixed_quality_gate_multifrequency_promoted =
                true;
            debug_telemetry_
                .safe_fix_shadow_failure_budget_passed = true;
            debug_telemetry_
                .safe_fix_shadow_independent_source_families = 2;
            debug_telemetry_
                .safe_fix_shadow_joint_failure_probability = 1e-6;
        } else if (disjoint_state_safe_candidate ||
            (!safe_shadow_candidate &&
             disjoint_consensus_candidate)) {
            promoted_position = disjoint_consensus.position_ecef;
            debug_telemetry_
                .library_fixed_quality_gate_disjoint_consensus_promoted =
                disjoint_consensus.valid;
        }
        if (promoted_position.allFinite()) {
            solution.position_ecef = promoted_position;
            solution.position_geodetic =
                spp_utils::ecefToGeodetic(promoted_position);
            solution.status = SolutionStatus::FIXED;
            solution.ratio =
                causal_arc_promotion_candidate
                    ? debug_telemetry_
                          .lambda_causal_arc_subset_ratio
                    : satellite_par_promotion_candidate
                        ? debug_telemetry_
                              .lambda_satellite_par_shadow_ratio
                    : src_par_promotion_candidate
                        ? debug_telemetry_
                              .lambda_src_par_shadow_ratio
                    : inertial_referenced_promotion_candidate
                        ? debug_telemetry_.full_ratio
                    : multifrequency_promotion_candidate
                        ? debug_telemetry_
                              .lambda_l1_l5_wlnl_shadow_nl_ratio
                    : l1_l2_multifrequency_promotion_candidate
                        ? debug_telemetry_
                              .lambda_l1_l2_wlnl_shadow_nl_ratio
                    : debug_telemetry_.full_ratio;
            solution.num_fixed_ambiguities =
                causal_arc_promotion_candidate
                    ? debug_telemetry_
                          .lambda_causal_arc_ready_pairs
                    : satellite_par_promotion_candidate
                        ? debug_telemetry_
                              .lambda_satellite_par_shadow_subset_size
                    : src_par_promotion_candidate
                        ? debug_telemetry_
                              .lambda_src_par_shadow_subset_size
                    : inertial_referenced_promotion_candidate
                        ? debug_telemetry_.pair_count
                    : multifrequency_promotion_candidate
                        ? debug_telemetry_
                              .lambda_l1_l5_wlnl_shadow_candidate_pair_count
                    : l1_l2_multifrequency_promotion_candidate
                        ? debug_telemetry_
                              .lambda_l1_l2_wlnl_shadow_candidate_pair_count
                    : debug_telemetry_.pair_count;
            debug_telemetry_
                .library_fixed_quality_gate_promoted = true;
            promoted_current_epoch_candidate = true;
        }
    }
    if (!solution.isFixed()) {
        return;
    }
    fixed_quality_gate::Evidence evidence;
    evidence.safe_fix_shadow_declared_fixed =
        debug_telemetry_.safe_fix_shadow_declared_fixed ||
        disjoint_state_decision.declared_fixed ||
        debug_telemetry_
            .library_fixed_quality_gate_causal_arc_promoted ||
        debug_telemetry_
            .library_fixed_quality_gate_satellite_par_promoted;
    evidence.safe_fix_shadow_declared_fixed =
        evidence.safe_fix_shadow_declared_fixed ||
        debug_telemetry_
            .library_fixed_quality_gate_src_par_promoted ||
        debug_telemetry_
            .library_fixed_quality_gate_inertial_referenced_promoted ||
        debug_telemetry_
            .library_fixed_quality_gate_multifrequency_promoted;
    evidence.independent_failure_budget_passed =
        debug_telemetry_.safe_fix_shadow_failure_budget_passed;
    evidence.float_position_covariance_trace_m2 =
        debug_telemetry_.float_position_covariance_trace_m2;
    evidence.update_observations =
        debug_telemetry_.float_update_observation_count;
    evidence.suppressed_outliers =
        debug_telemetry_.float_update_suppressed_outliers;
    evidence.update_nis_per_observation =
        debug_telemetry_.float_update_nis_per_observation;
    const auto decision = fixed_quality_gate::evaluate(config, evidence);
    const bool unresolved_raw_partition_conflict =
        original_solution.isFixed() &&
        !causal_arc_promotion_candidate &&
        !satellite_par_promotion_candidate &&
        !src_par_promotion_candidate &&
        !inertial_referenced_promotion_candidate &&
        !multifrequency_promotion_candidate &&
        !l1_l2_multifrequency_promotion_candidate &&
        !debug_telemetry_.safe_fix_shadow_declared_fixed &&
        !debug_telemetry_.inertial_fix_evidence_healthy_anchor &&
        disjoint_state_decision.state == safe_fix::State::IDLE &&
        debug_telemetry_.disjoint_satellite_fix_evidence_passed &&
        !debug_telemetry_
             .disjoint_satellite_fix_hard_separation_passed &&
        std::isfinite(
            debug_telemetry_
                .disjoint_satellite_fix_partition_separation_m) &&
        debug_telemetry_
                .disjoint_satellite_fix_partition_separation_m <
            debug_telemetry_
                .disjoint_satellite_fix_partition_a_primary_separation_m &&
        debug_telemetry_
                .disjoint_satellite_fix_partition_separation_m <
            debug_telemetry_
                .disjoint_satellite_fix_partition_b_primary_separation_m;
    debug_telemetry_.library_fixed_quality_gate_raw_partition_conflict =
        unresolved_raw_partition_conflict;
    const bool quality_passed =
        decision.passed && !unresolved_raw_partition_conflict;
    debug_telemetry_.library_fixed_quality_gate_passed =
        quality_passed;
    debug_telemetry_.library_fixed_quality_gate_safe_shadow_branch =
        decision.safe_shadow_branch;
    debug_telemetry_.library_fixed_quality_gate_covariance_branch =
        decision.covariance_branch;
    debug_telemetry_
        .library_fixed_quality_gate_strong_innovation_branch =
        decision.strong_innovation_branch;
    if (!quality_passed) {
        if (promoted_current_epoch_candidate) {
            // A provisional current-epoch integer candidate must not leak into
            // the exported FLOAT solution or become an inertial/filter seed
            // after the ordinary quality gate rejects it.
            solution = original_solution;
        }
        solution.status = SolutionStatus::FLOAT;
        debug_telemetry_.library_fixed_quality_gate_demoted = true;
    }
}

void RTKProcessor::updateIndependentFailureBudgetTelemetry() {
    double bsr = std::numeric_limits<double>::quiet_NaN();
    switch (rtk_config_.safe_fix_shadow_covariance_scale) {
        case 1: bsr = debug_telemetry_.lambda_shadow_bsr; break;
        case 2: bsr = debug_telemetry_.lambda_shadow_bsr_qscale2; break;
        case 4: bsr = debug_telemetry_.lambda_shadow_bsr_qscale4; break;
        case 8: bsr = debug_telemetry_.lambda_shadow_bsr_qscale8; break;
        case 16: bsr = debug_telemetry_.lambda_shadow_bsr_qscale16; break;
        default: break;
    }
    FixedFailureRateRatioThreshold threshold;
    const bool ffrt_passed =
        debug_telemetry_.lambda_shadow_solved &&
        fixedFailureRateRatioThreshold(
            debug_telemetry_.pair_count, bsr, 0.001, threshold) &&
        threshold.accepts_any_candidate &&
        std::isfinite(debug_telemetry_.full_ratio) &&
        debug_telemetry_.full_ratio >=
            threshold.minimum_second_to_best_ratio;

    const auto& inertial = external_inertial_fix_evidence_;
    debug_telemetry_.inertial_fix_evidence_available =
        inertial.available;
    debug_telemetry_.inertial_fix_evidence_healthy_anchor =
        inertial.healthy_independent_anchor;
    const Vector3d primary_candidate_ecef(
        debug_telemetry_.lambda_shadow_best_ecef_x,
        debug_telemetry_.lambda_shadow_best_ecef_y,
        debug_telemetry_.lambda_shadow_best_ecef_z);
    bool inertial_passed = false;
    if (inertial.available &&
        inertial.healthy_independent_anchor &&
        inertial.position_ecef.allFinite() &&
        inertial.position_covariance_ecef.allFinite() &&
        primary_candidate_ecef.allFinite()) {
        const double time_error_s =
            std::abs(inertial.time - current_epoch_time_);
        debug_telemetry_.inertial_fix_evidence_time_error_s =
            time_error_s;
        inertial_fix_evidence::Config monitor_config;
        monitor_config.maximum_time_error_s =
            rtk_config_.inertial_fix_evidence_max_time_error_s;
        monitor_config.covariance_scale =
            rtk_config_.inertial_fix_evidence_covariance_scale;
        monitor_config.maximum_nis_per_dimension =
            rtk_config_
                .inertial_fix_evidence_max_nis_per_dimension;
        monitor_config.maximum_position_delta_m =
            rtk_config_
                .inertial_fix_evidence_max_position_delta_m;
        inertial_fix_evidence::Evidence monitor_evidence;
        monitor_evidence.available = inertial.available;
        monitor_evidence.healthy_independent_anchor =
            inertial.healthy_independent_anchor;
        monitor_evidence.time_error_s = time_error_s;
        monitor_evidence.predicted_position_ecef =
            inertial.position_ecef;
        monitor_evidence.predicted_position_covariance_ecef =
            inertial.position_covariance_ecef;
        monitor_evidence.primary_candidate_ecef =
            primary_candidate_ecef;
        const auto monitor_decision =
            inertial_fix_evidence::evaluate(
                monitor_config, monitor_evidence);
        debug_telemetry_
            .inertial_fix_evidence_position_delta_m =
            monitor_decision.position_delta_m;
        debug_telemetry_
            .inertial_fix_evidence_nis_per_dimension =
            monitor_decision.nis_per_dimension;
        inertial_passed = monitor_decision.passed;
    }
    debug_telemetry_.inertial_fix_evidence_passed =
        inertial_passed;

    const auto& disjoint =
        external_disjoint_satellite_fix_evidence_;
    debug_telemetry_.disjoint_satellite_fix_evidence_available =
        disjoint.available;
    debug_telemetry_.disjoint_satellite_fix_inputs_verified =
        disjoint.inputs_verified_disjoint;
    debug_telemetry_
        .disjoint_satellite_fix_partition_a_ffrt_passed =
        disjoint.partition_a_ffrt_passed;
    debug_telemetry_
        .disjoint_satellite_fix_partition_b_ffrt_passed =
        disjoint.partition_b_ffrt_passed;
    disjoint_satellite_fix_evidence::Config disjoint_config;
    disjoint_config.maximum_partition_separation_m =
        rtk_config_
            .disjoint_satellite_fix_max_partition_separation_m;
    disjoint_config.maximum_primary_separation_m =
        rtk_config_
            .disjoint_satellite_fix_max_primary_separation_m;
    disjoint_config.covariance_scale =
        rtk_config_.disjoint_satellite_fix_covariance_scale;
    disjoint_config.maximum_nis_per_dimension =
        rtk_config_
            .disjoint_satellite_fix_max_nis_per_dimension;
    disjoint_config.maximum_statistical_separation_m =
        rtk_config_
            .disjoint_satellite_fix_max_statistical_separation_m;
    disjoint_satellite_fix_evidence::Evidence disjoint_evidence;
    disjoint_evidence.available = disjoint.available;
    disjoint_evidence.inputs_verified_disjoint =
        disjoint.inputs_verified_disjoint;
    disjoint_evidence.primary_ffrt_passed = ffrt_passed;
    disjoint_evidence.partition_a_ffrt_passed =
        disjoint.partition_a_ffrt_passed;
    disjoint_evidence.partition_b_ffrt_passed =
        disjoint.partition_b_ffrt_passed;
    disjoint_evidence.partition_a_candidate_ecef =
        disjoint.partition_a_candidate_ecef;
    disjoint_evidence.partition_b_candidate_ecef =
        disjoint.partition_b_candidate_ecef;
    disjoint_evidence.primary_candidate_ecef =
        primary_candidate_ecef;
    disjoint_evidence.partition_a_covariance_ecef =
        disjoint.partition_a_covariance_ecef;
    disjoint_evidence.partition_b_covariance_ecef =
        disjoint.partition_b_covariance_ecef;
    if (std::isfinite(
            debug_telemetry_
                .float_position_covariance_trace_m2) &&
        debug_telemetry_
                .float_position_covariance_trace_m2 > 0.0) {
        disjoint_evidence.primary_covariance_ecef =
            Matrix3d::Identity() *
            (debug_telemetry_
                 .float_position_covariance_trace_m2 /
             3.0);
    }
    const auto disjoint_decision =
        disjoint_satellite_fix_evidence::evaluate(
            disjoint_config, disjoint_evidence);
    debug_telemetry_.disjoint_satellite_fix_evidence_passed =
        disjoint_decision.passed;
    debug_telemetry_
        .disjoint_satellite_fix_partition_separation_m =
        disjoint_decision.partition_separation_m;
    debug_telemetry_
        .disjoint_satellite_fix_partition_a_primary_separation_m =
        disjoint_decision.partition_a_primary_separation_m;
    debug_telemetry_
        .disjoint_satellite_fix_partition_b_primary_separation_m =
        disjoint_decision.partition_b_primary_separation_m;
    debug_telemetry_
        .disjoint_satellite_fix_hard_separation_passed =
        disjoint_decision.hard_separation_passed;
    debug_telemetry_
        .disjoint_satellite_fix_statistical_separation_passed =
        disjoint_decision.statistical_separation_passed;
    debug_telemetry_
        .disjoint_satellite_fix_partition_nis_per_dimension =
        disjoint_decision.partition_nis_per_dimension;
    debug_telemetry_
        .disjoint_satellite_fix_partition_a_primary_nis_per_dimension =
        disjoint_decision
            .partition_a_primary_nis_per_dimension;
    debug_telemetry_
        .disjoint_satellite_fix_partition_b_primary_nis_per_dimension =
        disjoint_decision
            .partition_b_primary_nis_per_dimension;

    // When two genuinely disjoint partitions both pass, they provide the
    // two independent fault domains themselves. Do not also count the
    // overlapping all-satellite primary solution as an independent family.
    const bool use_disjoint_partitions =
        disjoint_decision.passed;
    const std::array<fix_failure_budget::Evidence, 6>
        failure_evidence{{
            {
                fix_failure_budget::SourceFamily::
                    PRIMARY_CARRIER_AR,
                ffrt_passed && !use_disjoint_partitions,
                0.001,
            },
            {
                // Full and satellite-PAR share carrier observations and
                // ambiguity states, so they deliberately occupy the same
                // fault domain.
                fix_failure_budget::SourceFamily::
                    PRIMARY_CARRIER_AR,
                debug_telemetry_
                        .lambda_satellite_par_shadow_ffrt_passed &&
                    debug_telemetry_
                        .lambda_satellite_par_shadow_solved &&
                    !use_disjoint_partitions,
                0.001,
            },
            {
                fix_failure_budget::SourceFamily::
                    MULTIFREQUENCY_CASCADE,
                (debug_telemetry_
                         .lambda_l1_l5_wlnl_shadow_wl_ffrt_passed &&
                 debug_telemetry_
                         .lambda_l1_l5_wlnl_shadow_nl_ffrt_passed &&
                 std::isfinite(
                     debug_telemetry_
                         .lambda_l1_l5_wlnl_shadow_best_ecef_x) &&
                 std::isfinite(
                     debug_telemetry_
                         .lambda_l1_l5_wlnl_shadow_best_ecef_y) &&
                 std::isfinite(
                     debug_telemetry_
                         .lambda_l1_l5_wlnl_shadow_best_ecef_z)) ||
                    (debug_telemetry_
                         .lambda_l1_l2_wlnl_shadow_wl_ffrt_passed &&
                     debug_telemetry_
                         .lambda_l1_l2_wlnl_shadow_nl_ffrt_passed &&
                     std::isfinite(
                         debug_telemetry_
                             .lambda_l1_l2_wlnl_shadow_best_ecef_x) &&
                     std::isfinite(
                         debug_telemetry_
                             .lambda_l1_l2_wlnl_shadow_best_ecef_y) &&
                     std::isfinite(
                         debug_telemetry_
                             .lambda_l1_l2_wlnl_shadow_best_ecef_z)) ||
                    (debug_telemetry_
                         .lambda_l2_l5_wlnl_shadow_wl_ffrt_passed &&
                     debug_telemetry_
                         .lambda_l2_l5_wlnl_shadow_nl_ffrt_passed &&
                     std::isfinite(
                         debug_telemetry_
                             .lambda_l2_l5_wlnl_shadow_best_ecef_x) &&
                     std::isfinite(
                         debug_telemetry_
                             .lambda_l2_l5_wlnl_shadow_best_ecef_y) &&
                     std::isfinite(
                         debug_telemetry_
                             .lambda_l2_l5_wlnl_shadow_best_ecef_z)),
                0.002,
            },
            {
                fix_failure_budget::SourceFamily::
                    INERTIAL_SOLUTION_SEPARATION,
                inertial_passed,
                rtk_config_
                    .inertial_fix_evidence_failure_probability,
            },
            {
                fix_failure_budget::SourceFamily::
                    DISJOINT_SATELLITE_PARTITION_A,
                use_disjoint_partitions,
                rtk_config_
                    .disjoint_satellite_fix_failure_probability,
            },
            {
                fix_failure_budget::SourceFamily::
                    DISJOINT_SATELLITE_PARTITION_B,
                use_disjoint_partitions,
                rtk_config_
                    .disjoint_satellite_fix_failure_probability,
            },
        }};
    const auto failure_budget =
        fix_failure_budget::evaluate(
            fix_failure_budget::Config{}, failure_evidence);
    debug_telemetry_
        .safe_fix_shadow_independent_source_families =
        failure_budget.independent_families;
    debug_telemetry_
        .safe_fix_shadow_joint_failure_probability =
        failure_budget.joint_failure_probability;
    debug_telemetry_
        .safe_fix_shadow_failure_budget_passed =
        failure_budget.passed;
    independent_failure_budget_evaluated_this_epoch_ = true;
}

void RTKProcessor::updateSafeFixShadowStateMachine(const GNSSTime& time) {
    const auto& config = rtk_config_.safe_fix_shadow_state_machine;
    debug_telemetry_.safe_fix_shadow_enabled = config.enabled;

    safe_fix::Candidate candidate;
    candidate.time_s =
        static_cast<double>(time.week) * 604800.0 + time.tow;
    candidate.correction_m = Vector3d(
        debug_telemetry_.lambda_shadow_best_correction_x,
        debug_telemetry_.lambda_shadow_best_correction_y,
        debug_telemetry_.lambda_shadow_best_correction_z);
    candidate.nis_per_observation =
        debug_telemetry_.float_update_nis_per_observation;
    candidate.prefit_residual_rms_m =
        debug_telemetry_.float_update_prefit_residual_rms_m;
    candidate.pair_count = debug_telemetry_.pair_count;
    candidate.ambiguity_ratio = debug_telemetry_.full_ratio;
    const Vector3d satellite_candidate_ecef(
        debug_telemetry_.lambda_satellite_par_shadow_best_ecef_x,
        debug_telemetry_.lambda_satellite_par_shadow_best_ecef_y,
        debug_telemetry_.lambda_satellite_par_shadow_best_ecef_z);
    const Vector3d full_candidate_ecef(
        debug_telemetry_.lambda_shadow_best_ecef_x,
        debug_telemetry_.lambda_shadow_best_ecef_y,
        debug_telemetry_.lambda_shadow_best_ecef_z);
    if (satellite_candidate_ecef.allFinite() &&
        full_candidate_ecef.allFinite()) {
        candidate.independent_consensus_delta_m =
            (satellite_candidate_ecef - full_candidate_ecef).norm();
    }
    debug_telemetry_
        .safe_fix_shadow_independent_consensus_delta_m =
        candidate.independent_consensus_delta_m;

    double bsr = std::numeric_limits<double>::quiet_NaN();
    switch (rtk_config_.safe_fix_shadow_covariance_scale) {
        case 1: bsr = debug_telemetry_.lambda_shadow_bsr; break;
        case 2: bsr = debug_telemetry_.lambda_shadow_bsr_qscale2; break;
        case 4: bsr = debug_telemetry_.lambda_shadow_bsr_qscale4; break;
        case 8: bsr = debug_telemetry_.lambda_shadow_bsr_qscale8; break;
        case 16: bsr = debug_telemetry_.lambda_shadow_bsr_qscale16; break;
        default: break;
    }
    FixedFailureRateRatioThreshold threshold;
    const bool ffrt_passed =
        debug_telemetry_.lambda_shadow_solved &&
        fixedFailureRateRatioThreshold(
            candidate.pair_count, bsr, 0.001, threshold) &&
        threshold.accepts_any_candidate &&
        std::isfinite(debug_telemetry_.full_ratio) &&
        debug_telemetry_.full_ratio >=
            threshold.minimum_second_to_best_ratio;
    if (!independent_failure_budget_evaluated_this_epoch_) {
        updateIndependentFailureBudgetTelemetry();
    }
    candidate.independent_failure_budget_passed =
        debug_telemetry_.safe_fix_shadow_failure_budget_passed;
    candidate.acquisition_eligible =
        ffrt_passed &&
        candidate.pair_count >= rtk_config_.safe_fix_shadow_minimum_pairs &&
        std::isfinite(
            debug_telemetry_.lambda_shadow_second_position_delta_m) &&
        debug_telemetry_.lambda_shadow_second_position_delta_m <=
            rtk_config_
                .safe_fix_shadow_maximum_second_position_delta_m &&
        std::isfinite(candidate.nis_per_observation) &&
        candidate.nis_per_observation <=
            rtk_config_.safe_fix_shadow_maximum_nis_per_observation &&
        std::isfinite(candidate.prefit_residual_rms_m) &&
        candidate.prefit_residual_rms_m <=
            rtk_config_
                .safe_fix_shadow_maximum_prefit_residual_rms_m;
    candidate.strong_acquisition_eligible =
        candidate.acquisition_eligible &&
        std::isfinite(candidate.ambiguity_ratio) &&
        candidate.ambiguity_ratio >= 10.0 &&
        candidate.pair_count >= 20 &&
        std::isfinite(
            debug_telemetry_.lambda_shadow_second_position_delta_m) &&
        debug_telemetry_.lambda_shadow_second_position_delta_m <= 0.05 &&
        std::isfinite(candidate.independent_consensus_delta_m) &&
        candidate.independent_consensus_delta_m <= 0.01 &&
        candidate.correction_m.norm() <= 0.05;
    candidate.change_point_acquisition_eligible =
        ffrt_passed &&
        std::isfinite(candidate.ambiguity_ratio) &&
        candidate.ambiguity_ratio >= 1.10 &&
        candidate.pair_count >= 20 &&
        std::isfinite(
            debug_telemetry_.lambda_shadow_second_position_delta_m) &&
        debug_telemetry_.lambda_shadow_second_position_delta_m <= 0.07 &&
        std::isfinite(candidate.nis_per_observation) &&
        candidate.nis_per_observation <= 3.0 &&
        std::isfinite(candidate.prefit_residual_rms_m) &&
        candidate.prefit_residual_rms_m <= 50.0 &&
        std::isfinite(candidate.independent_consensus_delta_m) &&
        candidate.independent_consensus_delta_m <= 0.05;

    const auto decision =
        safe_fix_shadow_state_machine_.update(config, candidate);
    debug_telemetry_.safe_fix_shadow_state =
        static_cast<int>(decision.state);
    debug_telemetry_.safe_fix_shadow_declared_fixed =
        decision.declared_fixed;
    debug_telemetry_.safe_fix_shadow_candidate_accepted =
        decision.candidate_accepted;
    debug_telemetry_.safe_fix_shadow_held = decision.held;
    debug_telemetry_.safe_fix_shadow_revoked = decision.revoked;
    debug_telemetry_.safe_fix_shadow_strong_acquisition =
        decision.strong_acquisition;
    debug_telemetry_.safe_fix_shadow_change_point_acquisition =
        decision.change_point_acquisition;
    debug_telemetry_.safe_fix_shadow_acquisition_streak =
        decision.acquisition_streak;
    debug_telemetry_.safe_fix_shadow_hold_epochs =
        decision.hold_epochs;
}

}  // namespace libgnss
