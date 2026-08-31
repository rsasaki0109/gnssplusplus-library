#include <libgnss++/algorithms/fgo.hpp>

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/signal_bias_contract.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#ifdef GNSSPP_HAS_CHOLMOD
#include <Eigen/CholmodSupport>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <tuple>


#include "fgo_internal.hpp"

namespace libgnss {

using namespace fgo_internal;

FGOProcessor::FGOResult FGOProcessor::optimize(
    const std::vector<ObservationData>& epochs,
    const NavigationData& nav) const {
    return optimizeProblem(buildPseudorangeProblem(epochs, nav));
}


FGOProcessor::FGOResult FGOProcessor::optimize(
    const std::vector<ObservationData>& rover_epochs,
    const std::vector<ObservationData>& base_epochs,
    const NavigationData& nav,
    const Vector3d& base_position_ecef) const {
    return optimizeProblem(
        buildDoubleDifferenceProblem(rover_epochs, base_epochs, nav, base_position_ecef));
}

FGOProcessor::FGOResult FGOProcessor::optimizeProblem(const FGOProblem& problem) const {
    const auto optimize_problem_start =
        std::chrono::high_resolution_clock::now();
    FGOResult result;
    result.diagnostics.epochs = problem.epochs.size();
    result.diagnostics.pseudorange_factors = problem.pseudorange_factors.size();
    if (config_.use_receiver_signal_bias_states) {
        std::set<std::pair<GNSSSystem, SignalType>> signal_bias_groups;
        for (const auto& factor : problem.pseudorange_factors) {
            if (signal_bias::isEligible(factor.satellite.system, factor.signal)) {
                ++result.diagnostics.receiver_signal_bias_factors;
                signal_bias_groups.emplace(factor.satellite.system, factor.signal);
            }
        }
        result.diagnostics.receiver_signal_bias_states = signal_bias_groups.size();
    }
    result.diagnostics.tdcp_factors = problem.tdcp_factors.size();
    result.diagnostics.undifferenced_doppler_factors =
        problem.undifferenced_doppler_factors.size();
    // The Eigen backend consumes the backend-independent rows directly.  The
    // GTSAM backend overwrites this with its actual insertion count after
    // validity checks, so diagnostics distinguish candidate rows from graph
    // factors without changing the legacy default recipe.
    result.diagnostics.undifferenced_doppler_factors_inserted =
        problem.undifferenced_doppler_factors.size();
    if (config_.use_doppler_velocity_wls_initialization) {
        for (const auto& estimate : problem.doppler_velocity_wls_estimates) {
            if (estimate.valid) {
                ++result.diagnostics.doppler_velocity_wls_valid_epochs;
                if (estimate.propagated) {
                    ++result.diagnostics.doppler_velocity_wls_propagated_epochs;
                }
                result.diagnostics.doppler_velocity_wls_max_condition_number =
                    std::max(result.diagnostics.doppler_velocity_wls_max_condition_number,
                             estimate.condition_number);
                result.diagnostics.doppler_velocity_wls_max_normalized_rms =
                    std::max(result.diagnostics.doppler_velocity_wls_max_normalized_rms,
                             estimate.normalized_rms);
                result.diagnostics.doppler_velocity_wls_max_velocity_norm_mps =
                    std::max(result.diagnostics.doppler_velocity_wls_max_velocity_norm_mps,
                             estimate.velocity_ecef_mps.norm());
                result.diagnostics.doppler_velocity_wls_max_clock_rate_abs_mps =
                    std::max(result.diagnostics.doppler_velocity_wls_max_clock_rate_abs_mps,
                             std::abs(estimate.clock_rate_mps));
            } else {
                ++result.diagnostics.doppler_velocity_wls_rejected_epochs;
            }
        }
    }
    result.diagnostics.single_difference_doppler_factors =
        problem.single_difference_doppler_factors.size();
    result.diagnostics.sparse_epochs_retained =
        problem.diagnostics.sparse_epochs_retained;
    result.diagnostics.sparse_empty_epochs_retained =
        problem.diagnostics.sparse_empty_epochs_retained;
    if (config_.use_native_pdc_state_bridge) {
        for (const auto& seed : problem.native_pdc_state_seeds) {
            if (seed.has_position) {
                ++result.diagnostics.native_pdc_position_seeds;
            }
            if (seed.has_velocity) {
                ++result.diagnostics.native_pdc_velocity_seeds;
            }
            if (seed.has_clock) {
                ++result.diagnostics.native_pdc_clock_seeds;
            }
            if (seed.has_clock_rate) {
                ++result.diagnostics.native_pdc_clock_rate_seeds;
            }
        }
    }
    result.diagnostics.single_difference_tdcp_factors =
        problem.single_difference_tdcp_factors.size();
    result.diagnostics.carrier_phase_factors = problem.carrier_phase_factors.size();
    result.diagnostics.double_difference_pseudorange_factors =
        problem.double_difference_pseudorange_factors.size();
    result.diagnostics.double_difference_carrier_factors =
        problem.double_difference_carrier_factors.size();
    result.diagnostics.ambiguity_between_factors =
        problem.ambiguity_between_factors.size();
    result.diagnostics.ambiguity_states = problem.ambiguity_states.size();
    result.diagnostics.tdcp_candidate_pairs = problem.diagnostics.tdcp_candidate_pairs;
    result.diagnostics.tdcp_rejected_gap = problem.diagnostics.tdcp_rejected_gap;
    result.diagnostics.tdcp_rejected_missing_previous =
        problem.diagnostics.tdcp_rejected_missing_previous;
    result.diagnostics.tdcp_rejected_loss_of_lock =
        problem.diagnostics.tdcp_rejected_loss_of_lock;
    result.diagnostics.tdcp_rejected_code_phase_jump =
        problem.diagnostics.tdcp_rejected_code_phase_jump;
    result.diagnostics.double_difference_matched_base_epochs =
        problem.diagnostics.double_difference_matched_base_epochs;
    result.diagnostics.double_difference_interpolated_base_epochs =
        problem.diagnostics.double_difference_interpolated_base_epochs;
    result.diagnostics.double_difference_candidate_pairs =
        problem.diagnostics.double_difference_candidate_pairs;
    result.diagnostics.double_difference_rejected_no_base_epoch =
        problem.diagnostics.double_difference_rejected_no_base_epoch;
    result.diagnostics.double_difference_rejected_no_reference =
        problem.diagnostics.double_difference_rejected_no_reference;
    result.diagnostics.code_minus_carrier_jump_resets =
        problem.diagnostics.code_minus_carrier_jump_resets;
    result.diagnostics.geometry_free_cycle_slip_resets =
        problem.diagnostics.geometry_free_cycle_slip_resets;
    result.diagnostics.code_minus_carrier_level_exclusions =
        problem.diagnostics.code_minus_carrier_level_exclusions;
    result.diagnostics.cmc_ref_avoided_count =
        problem.diagnostics.cmc_ref_avoided_count;

    if (problem.epochs.empty() ||
        (problem.pseudorange_factors.empty() &&
         problem.carrier_phase_factors.empty() &&
         problem.double_difference_pseudorange_factors.empty() &&
         problem.double_difference_carrier_factors.empty() &&
         problem.tdcp_factors.empty() &&
         problem.undifferenced_doppler_factors.empty() &&
         problem.single_difference_doppler_factors.empty() &&
         problem.single_difference_tdcp_factors.empty())) {
        return result;
    }

#ifdef GNSSPP_HAS_GTSAM
    if (config_.backend == FGOBackend::GTSAM) {
        return optimizeProblemWithGtsam(problem, config_, std::move(result));
    }
#endif

    // The native optimizer consumes every TDCP entry in the problem as one
    // residual row. Keep this separate from tdcp_factors (measurements built)
    // so a backend can never silently report generated factors as inserted.
    result.diagnostics.tdcp_factors_inserted = problem.tdcp_factors.size();
    result.diagnostics.single_difference_tdcp_factors_inserted =
        problem.single_difference_tdcp_factors.size();

    const int num_epochs = static_cast<int>(problem.epochs.size());
    std::vector<GNSSSystem> bias_groups;
    std::map<GNSSSystem, int> bias_group_columns;
    if (config_.use_inter_system_biases) {
        std::map<GNSSSystem, bool> present_bias_groups;
        for (const auto& factor : problem.pseudorange_factors) {
            if (usesSeparateClockBias(factor.clock_group)) {
                present_bias_groups[factor.clock_group] = true;
            }
        }
        for (const auto& factor : problem.carrier_phase_factors) {
            if (usesSeparateClockBias(factor.clock_group)) {
                present_bias_groups[factor.clock_group] = true;
            }
        }
        for (const auto& [group, present] : present_bias_groups) {
            if (!present) {
                continue;
            }
            bias_group_columns[group] = static_cast<int>(bias_groups.size());
            bias_groups.push_back(group);
        }
    }
    const int epoch_state_size = 4;
    const int position_clock_state_size = epoch_state_size * num_epochs;
    const bool use_velocity_states = config_.use_velocity_states;
    const int velocity_state_size = use_velocity_states ? 3 * num_epochs : 0;
    const int velocity_state_offset = position_clock_state_size;
    const int bias_state_offset = velocity_state_offset + velocity_state_size;
    const int base_state_size =
        bias_state_offset + static_cast<int>(bias_groups.size());
    const int ambiguity_count = static_cast<int>(problem.ambiguity_states.size());
    const int state_size = base_state_size + ambiguity_count;
    constexpr int kSparseNormalStateThreshold = 300;
    const bool use_sparse_normal = state_size > kSparseNormalStateThreshold;
    Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(state_size);
    const bool use_doppler_velocity_wls =
        config_.use_doppler_velocity_wls_initialization;
    if (use_doppler_velocity_wls &&
        problem.doppler_velocity_wls_estimates.size() !=
            static_cast<std::size_t>(num_epochs)) {
        // A requested initializer must never silently fall back to zero
        // velocity.  The caller can inspect the rejected-epoch diagnostics.
        return result;
    }
    if (use_doppler_velocity_wls &&
        std::any_of(problem.doppler_velocity_wls_estimates.begin(),
                    problem.doppler_velocity_wls_estimates.end(),
                    [](const auto& estimate) { return !estimate.valid; })) {
        return result;
    }
    for (int i = 0; i < num_epochs; ++i) {
        const int epoch_col = epoch_state_size * i;
        initial_state.segment<3>(epoch_col) = problem.epochs[i].position_ecef;
        initial_state(epoch_col + 3) = problem.epochs[i].receiver_clock_bias_m;
    }
    if (use_velocity_states) {
        for (int i = 0; i < num_epochs; ++i) {
            if (use_doppler_velocity_wls) {
                initial_state.segment<3>(velocity_state_offset + 3 * i) =
                    problem.doppler_velocity_wls_estimates[
                        static_cast<std::size_t>(i)]
                        .velocity_ecef_mps;
            } else {
                initial_state.segment<3>(velocity_state_offset + 3 * i).setZero();
            }
        }
    }
    for (int i = 0; i < ambiguity_count; ++i) {
        initial_state(base_state_size + i) =
            problem.ambiguity_states[i].initial_ambiguity_m;
    }

    auto epoch_state_col = [&](std::size_t epoch_index) -> int {
        return epoch_state_size * static_cast<int>(epoch_index);
    };

    auto velocity_state_col = [&](std::size_t epoch_index) -> int {
        if (!use_velocity_states) {
            return -1;
        }
        return velocity_state_offset + 3 * static_cast<int>(epoch_index);
    };

    auto system_bias_col = [&](int /*epoch_col*/, GNSSSystem group) -> int {
        const auto bias_it = bias_group_columns.find(group);
        if (bias_it == bias_group_columns.end()) {
            return -1;
        }
        return bias_state_offset + bias_it->second;
    };

    auto motion_rows_per_pair = [&]() -> std::size_t {
        const bool use_velocity_motion =
            config_.use_velocity_motion_factors && use_velocity_states;
        if (!config_.use_motion_factors && !use_velocity_motion) {
            return 0;
        }
        std::size_t rows_per_pair = 0;
        if (config_.use_motion_factors &&
            config_.use_position_motion_factors) {
            rows_per_pair += 3;
        }
        if (config_.use_motion_factors && config_.use_clock_motion_factors) {
            rows_per_pair += 1;
        }
        if (use_velocity_motion) {
            rows_per_pair += 3;
        }
        return rows_per_pair;
    };

    auto motion_factor_count = [&]() -> std::size_t {
        const bool use_velocity_motion =
            config_.use_velocity_motion_factors && use_velocity_states;
        if ((!config_.use_motion_factors && !use_velocity_motion) ||
            num_epochs < 2) {
            return 0;
        }
        return static_cast<std::size_t>(num_epochs - 1) * motion_rows_per_pair();
    };

    auto ambiguity_prior_count = [&]() -> std::size_t {
        if (!config_.use_ambiguity_priors || config_.ambiguity_prior_sigma_m <= 0.0) {
            return 0;
        }
        return problem.ambiguity_states.size();
    };

    auto velocity_prior_count = [&]() -> std::size_t {
        if (!use_velocity_states || config_.velocity_prior_sigma_mps <= 0.0) {
            return 0;
        }
        if (use_doppler_velocity_wls) {
            return std::count_if(
                       problem.doppler_velocity_wls_estimates.begin(),
                       problem.doppler_velocity_wls_estimates.end(),
                       [](const auto& estimate) { return estimate.valid; }) *
                   3U;
        }
        return static_cast<std::size_t>(num_epochs) * 3U;
    };

    auto doppler_wls_clock_prior_count = [&]() -> std::size_t {
        if (!use_doppler_velocity_wls || num_epochs < 2) {
            return 0;
        }
        std::size_t count = 0;
        for (int i = 1; i < num_epochs; ++i) {
            const auto& previous =
                problem.doppler_velocity_wls_estimates[
                    static_cast<std::size_t>(i - 1)];
            const auto& current = problem.doppler_velocity_wls_estimates[
                static_cast<std::size_t>(i)];
            const double dt = problem.epochs[static_cast<std::size_t>(i)].time -
                              problem.epochs[static_cast<std::size_t>(i - 1)].time;
            const bool clock_jump =
                static_cast<std::size_t>(i) < problem.clock_jumps.size() &&
                problem.clock_jumps[static_cast<std::size_t>(i)];
            if (previous.valid && current.valid && dt > 0.0 &&
                (config_.max_tdcp_gap_s <= 0.0 ||
                 dt <= config_.max_tdcp_gap_s) &&
                !clock_jump) {
                ++count;
            }
        }
        return count;
    };

    struct OptimizationOutput {
        Eigen::VectorXd state;
        Eigen::MatrixXd normal_matrix;
        Eigen::SparseMatrix<double> sparse_normal_matrix;
        int iterations = 0;
        bool converged = false;
        double final_cost = 0.0;
        double processing_ms = 0.0;
        double last_dx_norm = std::numeric_limits<double>::infinity();
        std::size_t robust_pseudorange_factors = 0;
        std::size_t robust_carrier_phase_factors = 0;
        std::size_t robust_double_difference_pseudorange_factors = 0;
        std::size_t robust_double_difference_carrier_factors = 0;
        std::size_t robust_tdcp_factors = 0;
        std::vector<FGOProcessor::CostTraceEntry> cost_trace_entries;
    };

    auto robust_scale = [&](double normalized_residual,
                            double threshold_sigma) -> double {
        if (!config_.use_robust_loss || threshold_sigma <= 0.0) {
            return 1.0;
        }
        const double abs_residual = std::abs(normalized_residual);
        if (!std::isfinite(abs_residual) || abs_residual <= threshold_sigma) {
            return 1.0;
        }
        return std::sqrt(threshold_sigma / abs_residual);
    };

    auto run_optimizer =
        [&](const Eigen::VectorXd& start_state,
            const std::vector<FixedAmbiguityConstraint>& fixed_constraints,
            const std::string& phase,
            int global_iteration_offset)
            -> OptimizationOutput {
        OptimizationOutput output;
        output.state = start_state;

        const auto start_time = std::chrono::high_resolution_clock::now();
        double previous_cost = std::numeric_limits<double>::infinity();
        for (int iter = 0; iter < config_.max_iterations; ++iter) {
            const int pr_rows = static_cast<int>(problem.pseudorange_factors.size());
            const int carrier_phase_rows =
                static_cast<int>(problem.carrier_phase_factors.size());
            const int double_difference_pseudorange_rows =
                static_cast<int>(problem.double_difference_pseudorange_factors.size());
            const int double_difference_carrier_rows =
                static_cast<int>(problem.double_difference_carrier_factors.size());
            const int ambiguity_between_rows =
                static_cast<int>(problem.ambiguity_between_factors.size());
            const int tdcp_rows = static_cast<int>(problem.tdcp_factors.size());
            const int undifferenced_doppler_rows = static_cast<int>(
                problem.undifferenced_doppler_factors.size());
            const int single_difference_doppler_rows =
                static_cast<int>(problem.single_difference_doppler_factors.size());
            const int single_difference_tdcp_rows =
                static_cast<int>(problem.single_difference_tdcp_factors.size());
            const int motion_rows = static_cast<int>(motion_factor_count());
            const int ambiguity_prior_rows = static_cast<int>(ambiguity_prior_count());
            const int velocity_prior_rows = static_cast<int>(velocity_prior_count());
            const int fixed_ambiguity_rows = static_cast<int>(fixed_constraints.size());
            const int estimated_rows =
                pr_rows + carrier_phase_rows + double_difference_pseudorange_rows +
                double_difference_carrier_rows + ambiguity_between_rows + tdcp_rows +
                undifferenced_doppler_rows +
                single_difference_doppler_rows + single_difference_tdcp_rows +
                motion_rows + ambiguity_prior_rows + velocity_prior_rows +
                static_cast<int>(doppler_wls_clock_prior_count()) +
                fixed_ambiguity_rows;
            (void)estimated_rows;
            Eigen::MatrixXd normal_matrix;
            std::vector<Eigen::Triplet<double>> normal_triplets;
            if (use_sparse_normal) {
                normal_triplets.reserve(
                    static_cast<std::size_t>(std::max(1, estimated_rows)) * 25U);
            } else {
                normal_matrix = Eigen::MatrixXd::Zero(state_size, state_size);
            }
            Eigen::VectorXd normal_rhs = Eigen::VectorXd::Zero(state_size);
            double weighted_residual_square_sum = 0.0;
            std::size_t robust_pseudorange_count = 0;
            std::size_t robust_carrier_phase_count = 0;
            std::size_t robust_double_difference_pseudorange_count = 0;
            std::size_t robust_double_difference_carrier_count = 0;
            std::size_t robust_tdcp_count = 0;

            int row = 0;
            auto add_weighted_row =
                [&](const std::vector<std::pair<int, double>>& jacobian,
                    double weighted_residual) {
                    if (jacobian.empty()) {
                        return;
                    }
                    weighted_residual_square_sum +=
                        weighted_residual * weighted_residual;
                    for (const auto& lhs : jacobian) {
                        if (lhs.first < 0 || lhs.first >= state_size) {
                            continue;
                        }
                        normal_rhs(lhs.first) += lhs.second * weighted_residual;
                        for (const auto& rhs : jacobian) {
                            if (rhs.first < 0 || rhs.first >= state_size) {
                                continue;
                            }
                            const double normal_entry = lhs.second * rhs.second;
                            if (use_sparse_normal) {
                                normal_triplets.emplace_back(
                                    lhs.first, rhs.first, normal_entry);
                            } else {
                                normal_matrix(lhs.first, rhs.first) +=
                                    normal_entry;
                            }
                        }
                    }
                    ++row;
                };
            if (config_.position_prior_sigma_m > 0.0) {
                const double position_prior_weight =
                    1.0 / std::max(1e-3, config_.position_prior_sigma_m);
                for (int i = 0; i < num_epochs; ++i) {
                    const int epoch_col = epoch_state_col(static_cast<std::size_t>(i));
                    for (int axis = 0; axis < 3; ++axis) {
                        const double weighted_residual =
                            (problem.epochs[i].position_ecef(axis) -
                             output.state(epoch_col + axis)) *
                            position_prior_weight;
                        add_weighted_row(
                            {{epoch_col + axis, position_prior_weight}},
                            weighted_residual);
                    }
                }
            }

            if (config_.clock_prior_sigma_m > 0.0) {
                const double clock_prior_weight =
                    1.0 / std::max(1e-3, config_.clock_prior_sigma_m);
                for (int i = 0; i < num_epochs; ++i) {
                    const int epoch_col = epoch_state_col(static_cast<std::size_t>(i));
                    const double weighted_residual =
                        (problem.epochs[i].receiver_clock_bias_m -
                         output.state(epoch_col + 3)) *
                        clock_prior_weight;
                    add_weighted_row(
                        {{epoch_col + 3, clock_prior_weight}},
                        weighted_residual);
                }
                for (const auto& [group, offset] : bias_group_columns) {
                    (void)group;
                    const int bias_col = bias_state_offset + offset;
                    const double weighted_residual =
                        -output.state(bias_col) * clock_prior_weight;
                    add_weighted_row({{bias_col, clock_prior_weight}},
                                     weighted_residual);
                }
            }

            if (use_velocity_states && config_.velocity_prior_sigma_mps > 0.0) {
                for (int i = 0; i < num_epochs; ++i) {
                    const int velocity_col =
                        velocity_state_col(static_cast<std::size_t>(i));
                    const auto* wls_estimate =
                        use_doppler_velocity_wls
                            ? &problem.doppler_velocity_wls_estimates[
                                  static_cast<std::size_t>(i)]
                            : nullptr;
                    if (wls_estimate != nullptr && !wls_estimate->valid) {
                        // Never add a zero-centered prior for a rejected WLS
                        // epoch.  The requested opt-in is fail-closed before
                        // reaching this point, but retaining this guard keeps
                        // manually constructed FGOProblem values safe.
                        continue;
                    }
                    for (int axis = 0; axis < 3; ++axis) {
                        double sigma = config_.velocity_prior_sigma_mps;
                        double target = 0.0;
                        if (wls_estimate != nullptr) {
                            target = wls_estimate->velocity_ecef_mps(axis);
                            const double variance =
                                wls_estimate->covariance(axis, axis);
                            if (std::isfinite(variance) && variance > 0.0) {
                                sigma = std::sqrt(variance);
                            }
                            sigma = std::max(
                                0.2, std::min(sigma,
                                              config_.velocity_prior_sigma_mps));
                        }
                        const double velocity_prior_weight =
                            1.0 / std::max(1e-6, sigma);
                        const double weighted_residual =
                            (target - output.state(velocity_col + axis)) *
                            velocity_prior_weight;
                        add_weighted_row(
                            {{velocity_col + axis, velocity_prior_weight}},
                            weighted_residual);
                    }
                }
            }

            if (use_doppler_velocity_wls && num_epochs >= 2) {
                for (int i = 1; i < num_epochs; ++i) {
                    const auto& previous =
                        problem.doppler_velocity_wls_estimates[
                            static_cast<std::size_t>(i - 1)];
                    const auto& current = problem.doppler_velocity_wls_estimates[
                        static_cast<std::size_t>(i)];
                    const double dt =
                        problem.epochs[static_cast<std::size_t>(i)].time -
                        problem.epochs[static_cast<std::size_t>(i - 1)].time;
                    const bool clock_jump =
                        static_cast<std::size_t>(i) < problem.clock_jumps.size() &&
                        problem.clock_jumps[static_cast<std::size_t>(i)];
                    if (!previous.valid || !current.valid || !(dt > 0.0) ||
                        (config_.max_tdcp_gap_s > 0.0 &&
                         dt > config_.max_tdcp_gap_s) ||
                        clock_jump) {
                        continue;
                    }
                    double sigma = 1.0;
                    const double variance = current.covariance(3, 3);
                    if (std::isfinite(variance) && variance > 0.0) {
                        sigma = std::sqrt(variance);
                    }
                    sigma = std::max(0.5, std::min(sigma, 1000.0));
                    const double weight = 1.0 / sigma;
                    const int previous_col =
                        epoch_state_col(static_cast<std::size_t>(i - 1));
                    const int current_col =
                        epoch_state_col(static_cast<std::size_t>(i));
                    const double desired_change = current.clock_rate_mps * dt;
                    const double actual_change =
                        output.state(current_col + 3) -
                        output.state(previous_col + 3);
                    const double weighted_residual =
                        (desired_change - actual_change) * weight;
                    add_weighted_row(
                        {{previous_col + 3, -weight},
                         {current_col + 3, weight}},
                        weighted_residual);
                }
            }

            for (const auto& factor : problem.pseudorange_factors) {
                if (factor.epoch_index >= problem.epochs.size()) {
                    continue;
                }

                const int epoch_col = epoch_state_col(factor.epoch_index);
                const Vector3d position = output.state.segment<3>(epoch_col);
                const double clock_bias_m = output.state(epoch_col + 3);
                const int bias_col = system_bias_col(epoch_col, factor.clock_group);
                const Vector3d delta = factor.satellite_position_ecef - position;
                const double range = delta.norm();
                if (range <= 0.0) {
                    continue;
                }

                const Vector3d los = delta / range;
                double predicted = range + clock_bias_m;
                if (bias_col >= 0) {
                    predicted += output.state(bias_col);
                }
                const double sigma = std::max(1e-3, factor.sigma_m);
                const double raw_residual =
                    factor.corrected_pseudorange_m - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.pseudorange_huber_threshold_sigma);
                if (scale < 1.0) {
                    ++robust_pseudorange_count;
                }
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                jacobian.reserve(5);
                jacobian.emplace_back(epoch_col + 0, -los(0) * weight);
                jacobian.emplace_back(epoch_col + 1, -los(1) * weight);
                jacobian.emplace_back(epoch_col + 2, -los(2) * weight);
                jacobian.emplace_back(epoch_col + 3, 1.0 * weight);
                if (bias_col >= 0) {
                    jacobian.emplace_back(bias_col, 1.0 * weight);
                }
                add_weighted_row(jacobian, raw_residual * weight);
            }

            for (const auto& factor : problem.carrier_phase_factors) {
                if (factor.epoch_index >= problem.epochs.size() ||
                    factor.ambiguity_index >= problem.ambiguity_states.size()) {
                    continue;
                }

                const int epoch_col = epoch_state_col(factor.epoch_index);
                const int ambiguity_col =
                    base_state_size + static_cast<int>(factor.ambiguity_index);
                const Vector3d position = output.state.segment<3>(epoch_col);
                const double clock_bias_m = output.state(epoch_col + 3);
                const int bias_col = system_bias_col(epoch_col, factor.clock_group);
                const double ambiguity_m = output.state(ambiguity_col);
                const Vector3d delta = factor.satellite_position_ecef - position;
                const double range = delta.norm();
                if (range <= 0.0) {
                    continue;
                }

                const Vector3d los = delta / range;
                double predicted = range + clock_bias_m + ambiguity_m;
                if (bias_col >= 0) {
                    predicted += output.state(bias_col);
                }
                const double sigma = std::max(1e-4, factor.sigma_m);
                const double raw_residual =
                    factor.corrected_carrier_m - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.carrier_phase_huber_threshold_sigma);
                if (scale < 1.0) {
                    ++robust_carrier_phase_count;
                }
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                jacobian.reserve(6);
                jacobian.emplace_back(epoch_col + 0, -los(0) * weight);
                jacobian.emplace_back(epoch_col + 1, -los(1) * weight);
                jacobian.emplace_back(epoch_col + 2, -los(2) * weight);
                jacobian.emplace_back(epoch_col + 3, 1.0 * weight);
                if (bias_col >= 0) {
                    jacobian.emplace_back(bias_col, 1.0 * weight);
                }
                jacobian.emplace_back(ambiguity_col, 1.0 * weight);
                add_weighted_row(jacobian, raw_residual * weight);
            }

            for (const auto& factor : problem.double_difference_pseudorange_factors) {
                if (factor.epoch_index >= problem.epochs.size()) {
                    continue;
                }

                const int epoch_col = epoch_state_col(factor.epoch_index);
                const Vector3d position = output.state.segment<3>(epoch_col);
                const Vector3d seed_position =
                    problem.epochs[factor.epoch_index].position_ecef;
                const Vector3d linearization_position =
                    config_.linearize_double_difference_factors_at_seed
                        ? seed_position
                        : position;
                const DoubleDifferencePrediction dd_prediction =
                    doubleDifferencePredictionAt(
                        linearization_position,
                        factor.base_position_ecef,
                        factor.rover_satellite_position_ecef,
                        factor.rover_reference_position_ecef,
                        factor.base_satellite_position_ecef,
                        factor.base_reference_position_ecef);
                if (!dd_prediction.valid) {
                    continue;
                }

                double predicted = dd_prediction.geometry_m;
                if (config_.linearize_double_difference_factors_at_seed) {
                    predicted +=
                        dd_prediction.position_jacobian.dot(position - seed_position);
                }
                const double sigma = std::max(1e-3, factor.sigma_m);
                const double raw_residual =
                    factor.observed_dd_pseudorange_m - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.pseudorange_huber_threshold_sigma);
                if (scale < 1.0) {
                    ++robust_double_difference_pseudorange_count;
                }
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                jacobian.reserve(3);
                jacobian.emplace_back(epoch_col + 0,
                                      dd_prediction.position_jacobian(0) * weight);
                jacobian.emplace_back(epoch_col + 1,
                                      dd_prediction.position_jacobian(1) * weight);
                jacobian.emplace_back(epoch_col + 2,
                                      dd_prediction.position_jacobian(2) * weight);
                add_weighted_row(jacobian, raw_residual * weight);
            }

            for (const auto& factor : problem.double_difference_carrier_factors) {
                if (factor.epoch_index >= problem.epochs.size() ||
                    factor.ambiguity_index >= problem.ambiguity_states.size() ||
                    (factor.use_ambiguity_difference &&
                     factor.reference_ambiguity_index >=
                         problem.ambiguity_states.size())) {
                    continue;
                }

                const int epoch_col = epoch_state_col(factor.epoch_index);
                const int ambiguity_col =
                    base_state_size + static_cast<int>(factor.ambiguity_index);
                const int reference_ambiguity_col =
                    factor.use_ambiguity_difference
                        ? base_state_size +
                              static_cast<int>(factor.reference_ambiguity_index)
                        : -1;
                const Vector3d position = output.state.segment<3>(epoch_col);
                const Vector3d seed_position =
                    problem.epochs[factor.epoch_index].position_ecef;
                const Vector3d linearization_position =
                    config_.linearize_double_difference_factors_at_seed
                        ? seed_position
                        : position;
                const DoubleDifferencePrediction dd_prediction =
                    doubleDifferencePredictionAt(
                        linearization_position,
                        factor.base_position_ecef,
                        factor.rover_satellite_position_ecef,
                        factor.rover_reference_position_ecef,
                        factor.base_satellite_position_ecef,
                        factor.base_reference_position_ecef);
                if (!dd_prediction.valid) {
                    continue;
                }

                const double ambiguity_m = output.state(ambiguity_col);
                double predicted = dd_prediction.geometry_m + ambiguity_m;
                if (config_.linearize_double_difference_factors_at_seed) {
                    predicted +=
                        dd_prediction.position_jacobian.dot(position - seed_position);
                }
                if (factor.use_ambiguity_difference) {
                    predicted -= output.state(reference_ambiguity_col);
                }
                const double sigma = std::max(1e-4, factor.sigma_m);
                const double raw_residual =
                    factor.observed_dd_carrier_m - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.carrier_phase_huber_threshold_sigma);
                if (scale < 1.0) {
                    ++robust_double_difference_carrier_count;
                }
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                jacobian.reserve(5);
                jacobian.emplace_back(epoch_col + 0,
                                      dd_prediction.position_jacobian(0) * weight);
                jacobian.emplace_back(epoch_col + 1,
                                      dd_prediction.position_jacobian(1) * weight);
                jacobian.emplace_back(epoch_col + 2,
                                      dd_prediction.position_jacobian(2) * weight);
                jacobian.emplace_back(ambiguity_col, 1.0 * weight);
                if (factor.use_ambiguity_difference) {
                    jacobian.emplace_back(reference_ambiguity_col, -1.0 * weight);
                }
                add_weighted_row(jacobian, raw_residual * weight);
            }

            for (const auto& factor : problem.ambiguity_between_factors) {
                if (factor.previous_ambiguity_index >=
                        problem.ambiguity_states.size() ||
                    factor.current_ambiguity_index >=
                        problem.ambiguity_states.size()) {
                    continue;
                }

                const int previous_col =
                    base_state_size +
                    static_cast<int>(factor.previous_ambiguity_index);
                const int current_col =
                    base_state_size +
                    static_cast<int>(factor.current_ambiguity_index);
                const double sigma = std::max(1e-9, factor.sigma_m);
                const double weight = 1.0 / sigma;
                const double predicted =
                    output.state(current_col) - output.state(previous_col);
                add_weighted_row({{previous_col, -weight},
                                  {current_col, weight}},
                                 -predicted * weight);
            }

            for (const auto& factor : problem.tdcp_factors) {
                if (factor.previous_epoch_index >= problem.epochs.size() ||
                    factor.current_epoch_index >= problem.epochs.size()) {
                    continue;
                }

                const int previous_col =
                    epoch_state_col(factor.previous_epoch_index);
                const int current_col = epoch_state_col(factor.current_epoch_index);
                const int previous_bias_col =
                    system_bias_col(previous_col,
                                    clockBiasGroup(factor.satellite.system));
                const int current_bias_col =
                    system_bias_col(current_col,
                                    clockBiasGroup(factor.satellite.system));
                const Vector3d previous_position = output.state.segment<3>(previous_col);
                const Vector3d current_position = output.state.segment<3>(current_col);
                const Vector3d previous_delta =
                    factor.previous_satellite_position_ecef - previous_position;
                const Vector3d current_delta =
                    factor.current_satellite_position_ecef - current_position;
                const double previous_range = previous_delta.norm();
                const double current_range = current_delta.norm();
                if (previous_range <= 0.0 || current_range <= 0.0) {
                    continue;
                }

                const Vector3d previous_los = previous_delta / previous_range;
                const Vector3d current_los = current_delta / current_range;
                double predicted =
                    current_range + output.state(current_col + 3) -
                    previous_range - output.state(previous_col + 3);
                if (previous_bias_col >= 0 && current_bias_col >= 0 &&
                    previous_bias_col != current_bias_col) {
                    predicted += output.state(current_bias_col) -
                                 output.state(previous_bias_col);
                }
                const double sigma = std::max(1e-4, factor.sigma_m);
                const double raw_residual = factor.delta_carrier_m - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.tdcp_huber_threshold_sigma);
                if (scale < 1.0) {
                    ++robust_tdcp_count;
                }
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                jacobian.reserve(10);
                jacobian.emplace_back(previous_col + 0, previous_los(0) * weight);
                jacobian.emplace_back(previous_col + 1, previous_los(1) * weight);
                jacobian.emplace_back(previous_col + 2, previous_los(2) * weight);
                jacobian.emplace_back(previous_col + 3, -1.0 * weight);
                if (previous_bias_col >= 0 &&
                    previous_bias_col != current_bias_col) {
                    jacobian.emplace_back(previous_bias_col, -1.0 * weight);
                }
                jacobian.emplace_back(current_col + 0, -current_los(0) * weight);
                jacobian.emplace_back(current_col + 1, -current_los(1) * weight);
                jacobian.emplace_back(current_col + 2, -current_los(2) * weight);
                jacobian.emplace_back(current_col + 3, 1.0 * weight);
                if (current_bias_col >= 0 &&
                    previous_bias_col != current_bias_col) {
                    jacobian.emplace_back(current_bias_col, 1.0 * weight);
                }
                add_weighted_row(jacobian, raw_residual * weight);
            }

            // Receiver-only Doppler rows are deliberately kept separate from
            // the base-dependent single-difference path below.  The legacy
            // preparation leaves residual_mps - LOS*v_receiver.  The opt-in
            // Android-corrected contract additionally models the uncorrected
            // receiver clock frequency error as the finite difference of the
            // existing per-epoch clock-bias states.  With velocity states this
            // is a direct three-column row; the finite-difference fallback
            // mirrors the existing SD contract.
            for (const auto& factor : problem.undifferenced_doppler_factors) {
                if (factor.epoch_index >= problem.epochs.size()) {
                    continue;
                }

                std::size_t clock_previous_epoch =
                    std::numeric_limits<std::size_t>::max();
                double receiver_clock_drift_mps = 0.0;
                if (factor.includes_receiver_clock_drift) {
                    clock_previous_epoch = factor.previous_epoch_index;
                    if (clock_previous_epoch >= problem.epochs.size() ||
                        clock_previous_epoch >= factor.epoch_index) {
                        continue;
                    }
                    const double clock_dt = factor.dt_s > 0.0
                                                ? factor.dt_s
                                                : problem.epochs[factor.epoch_index].time -
                                                      problem.epochs[clock_previous_epoch].time;
                    if (!(clock_dt > 0.0) ||
                        (config_.max_tdcp_gap_s > 0.0 &&
                         clock_dt > config_.max_tdcp_gap_s)) {
                        continue;
                    }
                    receiver_clock_drift_mps =
                        (output.state(epoch_state_col(factor.epoch_index) + 3) -
                         output.state(epoch_state_col(clock_previous_epoch) + 3)) /
                        clock_dt;
                    if (!std::isfinite(receiver_clock_drift_mps)) {
                        continue;
                    }
                }

                int velocity_col = velocity_state_col(factor.epoch_index);
                Vector3d velocity = Vector3d::Zero();
                if (velocity_col >= 0) {
                    velocity = output.state.segment<3>(velocity_col);
                } else {
                    if (factor.epoch_index == 0) {
                        continue;
                    }
                    const std::size_t previous_epoch_index =
                        factor.epoch_index - 1;
                    const double dt =
                        problem.epochs[factor.epoch_index].time -
                        problem.epochs[previous_epoch_index].time;
                    if (dt <= 0.0 ||
                        (config_.max_tdcp_gap_s > 0.0 &&
                         dt > config_.max_tdcp_gap_s)) {
                        continue;
                    }
                    const int previous_col =
                        epoch_state_col(previous_epoch_index);
                    const int current_col = epoch_state_col(factor.epoch_index);
                    velocity =
                        (output.state.segment<3>(current_col) -
                         output.state.segment<3>(previous_col)) /
                        dt;
                    velocity_col = current_col;
                }

                const double predicted = doppler_velocity_wls::predict(
                    factor.los, velocity, receiver_clock_drift_mps);
                const double sigma = std::max(1e-4, factor.sigma_mps);
                const double raw_residual = factor.residual_mps - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.tdcp_huber_threshold_sigma);
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                if (use_velocity_states) {
                    jacobian.reserve(factor.includes_receiver_clock_drift ? 5 : 3);
                    for (int axis = 0; axis < 3; ++axis) {
                        jacobian.emplace_back(velocity_col + axis,
                                              factor.los(axis) * weight);
                    }
                } else {
                    if (factor.epoch_index == 0) {
                        continue;
                    }
                    const std::size_t previous_epoch_index =
                        factor.epoch_index - 1;
                    const double dt =
                        problem.epochs[factor.epoch_index].time -
                        problem.epochs[previous_epoch_index].time;
                    if (dt <= 0.0 ||
                        (config_.max_tdcp_gap_s > 0.0 &&
                         dt > config_.max_tdcp_gap_s)) {
                        continue;
                    }
                    const int previous_col =
                        epoch_state_col(previous_epoch_index);
                    const int current_col = epoch_state_col(factor.epoch_index);
                    jacobian.reserve(6);
                    for (int axis = 0; axis < 3; ++axis) {
                        jacobian.emplace_back(previous_col + axis,
                                              -factor.los(axis) / dt * weight);
                        jacobian.emplace_back(current_col + axis,
                                              factor.los(axis) / dt * weight);
                    }
                }
                if (factor.includes_receiver_clock_drift) {
                    const double clock_dt = factor.dt_s > 0.0
                                                ? factor.dt_s
                                                : problem.epochs[factor.epoch_index].time -
                                                      problem.epochs[clock_previous_epoch].time;
                    const int previous_clock_col =
                        epoch_state_col(clock_previous_epoch) + 3;
                    const int current_clock_col =
                        epoch_state_col(factor.epoch_index) + 3;
                    jacobian.emplace_back(previous_clock_col,
                                          -1.0 / clock_dt * weight);
                    jacobian.emplace_back(current_clock_col,
                                          1.0 / clock_dt * weight);
                }
                add_weighted_row(jacobian, raw_residual * weight);
            }

            for (const auto& factor : problem.single_difference_doppler_factors) {
                if (factor.epoch_index >= problem.epochs.size()) {
                    continue;
                }

                int velocity_col = velocity_state_col(factor.epoch_index);
                Vector3d velocity = Vector3d::Zero();
                if (velocity_col >= 0) {
                    velocity = output.state.segment<3>(velocity_col);
                } else {
                    if (factor.epoch_index == 0) {
                        continue;
                    }
                    const std::size_t previous_epoch_index =
                        factor.epoch_index - 1;
                    const double dt =
                        problem.epochs[factor.epoch_index].time -
                        problem.epochs[previous_epoch_index].time;
                    if (dt <= 0.0 ||
                        (config_.max_tdcp_gap_s > 0.0 &&
                         dt > config_.max_tdcp_gap_s)) {
                        continue;
                    }

                    const int previous_col = epoch_state_col(previous_epoch_index);
                    const int current_col = epoch_state_col(factor.epoch_index);
                    velocity =
                        (output.state.segment<3>(current_col) -
                         output.state.segment<3>(previous_col)) /
                        dt;
                    velocity_col = current_col;
                }
                const double predicted = factor.los.dot(velocity);
                const double sigma = std::max(1e-4, factor.sigma_mps);
                const double raw_residual = factor.residual_mps - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.tdcp_huber_threshold_sigma);
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                if (use_velocity_states) {
                    jacobian.reserve(3);
                    for (int axis = 0; axis < 3; ++axis) {
                        jacobian.emplace_back(velocity_col + axis,
                                              factor.los(axis) * weight);
                    }
                } else {
                    if (factor.epoch_index == 0) {
                        continue;
                    }
                    const std::size_t previous_epoch_index =
                        factor.epoch_index - 1;
                    const double dt =
                        problem.epochs[factor.epoch_index].time -
                        problem.epochs[previous_epoch_index].time;
                    const int previous_col = epoch_state_col(previous_epoch_index);
                    const int current_col = epoch_state_col(factor.epoch_index);
                    jacobian.reserve(6);
                    for (int axis = 0; axis < 3; ++axis) {
                        jacobian.emplace_back(previous_col + axis,
                                              -factor.los(axis) / dt * weight);
                        jacobian.emplace_back(current_col + axis,
                                              factor.los(axis) / dt * weight);
                    }
                }
                add_weighted_row(jacobian, raw_residual * weight);
            }

            for (const auto& factor : problem.single_difference_tdcp_factors) {
                if (factor.previous_epoch_index >= problem.epochs.size() ||
                    factor.current_epoch_index >= problem.epochs.size()) {
                    continue;
                }

                const int previous_col =
                    epoch_state_col(factor.previous_epoch_index);
                const int current_col = epoch_state_col(factor.current_epoch_index);
                const Vector3d previous_delta =
                    output.state.segment<3>(previous_col) -
                    problem.epochs[factor.previous_epoch_index].position_ecef;
                const Vector3d current_delta =
                    output.state.segment<3>(current_col) -
                    problem.epochs[factor.current_epoch_index].position_ecef;
                const double predicted =
                    factor.los.dot(current_delta) -
                    factor.previous_los.dot(previous_delta);
                const double sigma = std::max(1e-4, factor.sigma_m);
                const double raw_residual = factor.delta_carrier_m - predicted;
                const double scale =
                    robust_scale(raw_residual / sigma,
                                 config_.tdcp_huber_threshold_sigma);
                const double weight = scale / sigma;

                std::vector<std::pair<int, double>> jacobian;
                jacobian.reserve(6);
                for (int axis = 0; axis < 3; ++axis) {
                    jacobian.emplace_back(previous_col + axis,
                                          -factor.previous_los(axis) * weight);
                    jacobian.emplace_back(current_col + axis,
                                          factor.los(axis) * weight);
                }
                add_weighted_row(jacobian, raw_residual * weight);
            }

            const bool use_velocity_motion =
                config_.use_velocity_motion_factors && use_velocity_states;
            if ((config_.use_motion_factors || use_velocity_motion) &&
                num_epochs >= 2) {
                const double motion_sigma = std::max(1e-3, config_.motion_sigma_m);
                const double clock_motion_sigma =
                    std::max(1e-3, config_.clock_motion_sigma_m);
                const double velocity_motion_sigma =
                    std::max(1e-6, config_.velocity_motion_sigma_m);
                for (int i = 1; i < num_epochs; ++i) {
                    const int prev_col =
                        epoch_state_col(static_cast<std::size_t>(i - 1));
                    const int curr_col =
                        epoch_state_col(static_cast<std::size_t>(i));
                    const double dt = std::max(
                        1e-3,
                        std::abs(problem.epochs[i].time - problem.epochs[i - 1].time));
                    const double pos_weight = 1.0 / (motion_sigma * dt);
                    const bool clock_jump =
                        static_cast<std::size_t>(i) < problem.clock_jumps.size() &&
                        problem.clock_jumps[static_cast<std::size_t>(i)];
                    const double epoch_clock_motion_sigma =
                        clock_jump ? 1e6 : clock_motion_sigma;
                    const double clock_weight =
                        1.0 / (epoch_clock_motion_sigma * dt);

                    if (config_.use_motion_factors &&
                        config_.use_position_motion_factors) {
                        for (int axis = 0; axis < 3; ++axis) {
                            const double weighted_residual =
                                -(output.state(curr_col + axis) -
                                  output.state(prev_col + axis)) *
                                pos_weight;
                            add_weighted_row(
                                {{prev_col + axis, -pos_weight},
                                 {curr_col + axis, pos_weight}},
                                weighted_residual);
                        }
                    }

                    if (config_.use_motion_factors &&
                        config_.use_clock_motion_factors) {
                        const double weighted_clock_residual =
                            -(output.state(curr_col + 3) -
                              output.state(prev_col + 3)) *
                            clock_weight;
                        add_weighted_row(
                            {{prev_col + 3, -clock_weight},
                             {curr_col + 3, clock_weight}},
                            weighted_clock_residual);

                        for (int bias_offset = 4; bias_offset < epoch_state_size;
                             ++bias_offset) {
                            const double weighted_bias_residual =
                                -(output.state(curr_col + bias_offset) -
                                  output.state(prev_col + bias_offset)) *
                                clock_weight;
                            add_weighted_row(
                                {{prev_col + bias_offset, -clock_weight},
                                 {curr_col + bias_offset, clock_weight}},
                                weighted_bias_residual);
                        }
                    }

                    if (use_velocity_motion) {
                        const int prev_velocity_col =
                            velocity_state_col(static_cast<std::size_t>(i - 1));
                        const int curr_velocity_col =
                            velocity_state_col(static_cast<std::size_t>(i));
                        const double weight = 1.0 / velocity_motion_sigma;
                        for (int axis = 0; axis < 3; ++axis) {
                            const double predicted =
                                output.state(curr_col + axis) -
                                output.state(prev_col + axis) -
                                0.5 * dt *
                                    (output.state(prev_velocity_col + axis) +
                                     output.state(curr_velocity_col + axis));
                            add_weighted_row(
                                {{prev_col + axis, -weight},
                                 {curr_col + axis, weight},
                                 {prev_velocity_col + axis, -0.5 * dt * weight},
                                 {curr_velocity_col + axis, -0.5 * dt * weight}},
                                -predicted * weight);
                        }
                    }
                }
            }

            if (config_.use_ambiguity_priors && config_.ambiguity_prior_sigma_m > 0.0) {
                const double ambiguity_prior_sigma =
                    std::max(1e-3, config_.ambiguity_prior_sigma_m);
                const double ambiguity_prior_weight = 1.0 / ambiguity_prior_sigma;
                for (int i = 0; i < ambiguity_count; ++i) {
                    const int ambiguity_col = base_state_size + i;
                    const double weighted_residual =
                        (problem.ambiguity_states[i].initial_ambiguity_m -
                         output.state(ambiguity_col)) *
                        ambiguity_prior_weight;
                    add_weighted_row(
                        {{ambiguity_col, ambiguity_prior_weight}},
                        weighted_residual);
                }
            }

            if (!fixed_constraints.empty()) {
                const double fixed_sigma =
                    std::max(1e-5, config_.fixed_ambiguity_sigma_m);
                const double fixed_weight = 1.0 / fixed_sigma;
                for (const auto& fixed : fixed_constraints) {
                    if (fixed.ambiguity_index >= problem.ambiguity_states.size()) {
                        continue;
                    }
                    const int ambiguity_col =
                        base_state_size + static_cast<int>(fixed.ambiguity_index);
                    const double weighted_residual =
                        (fixed.fixed_ambiguity_m - output.state(ambiguity_col)) *
                        fixed_weight;
                    add_weighted_row({{ambiguity_col, fixed_weight}},
                                     weighted_residual);
                }
            }

            if (row == 0) {
                return output;
            }

            auto cost_delta = [&]() -> std::pair<double, double> {
                if (!std::isfinite(previous_cost) ||
                    !std::isfinite(weighted_residual_square_sum)) {
                    return {
                        std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN(),
                    };
                }
                const double absolute_decrease =
                    previous_cost - weighted_residual_square_sum;
                const double relative_decrease =
                    previous_cost > 0.0
                        ? absolute_decrease / previous_cost
                        : absolute_decrease;
                return {absolute_decrease, relative_decrease};
            };
            auto record_cost_trace = [&](double update_norm, bool converged) {
                const auto [absolute_decrease, relative_decrease] = cost_delta();
                FGOProcessor::CostTraceEntry entry;
                entry.phase = phase;
                entry.local_iteration = iter;
                entry.global_iteration = global_iteration_offset + iter;
                entry.cost = weighted_residual_square_sum;
                entry.absolute_decrease = absolute_decrease;
                entry.relative_decrease = relative_decrease;
                entry.update_norm = update_norm;
                entry.converged = converged;
                output.cost_trace_entries.push_back(std::move(entry));
            };

            Eigen::VectorXd dx;
            bool solved = false;
            Eigen::SparseMatrix<double> current_sparse_normal;
            auto store_current_linearization = [&]() {
                output.final_cost = weighted_residual_square_sum;
                if (!use_sparse_normal) {
                    output.normal_matrix = normal_matrix;
                }
                output.robust_pseudorange_factors = robust_pseudorange_count;
                output.robust_carrier_phase_factors = robust_carrier_phase_count;
                output.robust_double_difference_pseudorange_factors =
                    robust_double_difference_pseudorange_count;
                output.robust_double_difference_carrier_factors =
                    robust_double_difference_carrier_count;
                output.robust_tdcp_factors = robust_tdcp_count;
            };
            auto cost_converged = [&]() {
                if (iter == 0 || !std::isfinite(previous_cost) ||
                    !std::isfinite(weighted_residual_square_sum)) {
                    return false;
                }
                const double absolute_decrease =
                    previous_cost - weighted_residual_square_sum;
                if (absolute_decrease < 0.0) {
                    return false;
                }
                const double relative_decrease =
                    previous_cost > 0.0
                        ? absolute_decrease / previous_cost
                        : absolute_decrease;
                return (config_.absolute_cost_convergence_threshold > 0.0 &&
                        absolute_decrease <
                            config_.absolute_cost_convergence_threshold) ||
                       (config_.relative_cost_convergence_threshold > 0.0 &&
                        relative_decrease <
                            config_.relative_cost_convergence_threshold);
            };
            if (use_sparse_normal) {
                Eigen::SparseMatrix<double> sparse_normal(state_size, state_size);
                sparse_normal.setFromTriplets(
                    normal_triplets.begin(), normal_triplets.end());
                sparse_normal.makeCompressed();
                if (cost_converged()) {
                    store_current_linearization();
                    if (config_.collect_lambda_debug ||
                        config_.use_epoch_lambda_fixed_output) {
                        output.sparse_normal_matrix = std::move(sparse_normal);
                    }
                    output.iterations = iter;
                    output.converged = true;
                    record_cost_trace(
                        std::numeric_limits<double>::quiet_NaN(), true);
                    break;
                }

                double max_diagonal = 0.0;
                for (int i = 0; i < state_size; ++i) {
                    max_diagonal =
                        std::max(max_diagonal, std::abs(sparse_normal.coeff(i, i)));
                }
                double damping = std::max(1e-12, max_diagonal * 1e-12);
                for (int attempt = 0; attempt < 6; ++attempt) {
                    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt;
                    ldlt.setShift(damping);
                    ldlt.compute(sparse_normal);
                    if (ldlt.info() == Eigen::Success) {
                        dx = ldlt.solve(normal_rhs);
                        if (ldlt.info() == Eigen::Success && dx.allFinite()) {
                            solved = true;
                            break;
                        }
                    }
                    damping *= 10.0;
                }
                if (config_.collect_lambda_debug ||
                    config_.use_epoch_lambda_fixed_output) {
                    current_sparse_normal = std::move(sparse_normal);
                }
            } else {
                if (cost_converged()) {
                    store_current_linearization();
                    output.iterations = iter;
                    output.converged = true;
                    record_cost_trace(
                        std::numeric_limits<double>::quiet_NaN(), true);
                    break;
                }
                const double max_diagonal =
                    normal_matrix.diagonal().cwiseAbs().maxCoeff();
                double damping = std::max(1e-12, max_diagonal * 1e-12);
                for (int attempt = 0; attempt < 6; ++attempt) {
                    Eigen::MatrixXd damped_normal = normal_matrix;
                    damped_normal.diagonal().array() += damping;
                    Eigen::LDLT<Eigen::MatrixXd> ldlt(damped_normal);
                    if (ldlt.info() == Eigen::Success) {
                        dx = ldlt.solve(normal_rhs);
                        if (dx.allFinite()) {
                            solved = true;
                            break;
                        }
                    }
                    damping *= 10.0;
                }
            }

            if (!solved) {
                break;
            }

            output.state += dx;
            output.last_dx_norm = dx.norm();
            output.iterations = iter + 1;
            store_current_linearization();
            if (use_sparse_normal &&
                (config_.collect_lambda_debug ||
                 config_.use_epoch_lambda_fixed_output)) {
                output.sparse_normal_matrix = std::move(current_sparse_normal);
            }
            const bool update_converged =
                output.last_dx_norm < config_.convergence_threshold_m;
            record_cost_trace(output.last_dx_norm, update_converged);
            previous_cost = weighted_residual_square_sum;

            if (update_converged) {
                output.converged = true;
                break;
            }
        }

        const auto end_time = std::chrono::high_resolution_clock::now();
        output.processing_ms =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                end_time - start_time)
                .count();
        return output;
    };

    OptimizationOutput optimization = run_optimizer(initial_state, {}, "float", 0);
    result.cost_trace_entries = optimization.cost_trace_entries;
    int total_iterations = optimization.iterations;
    double total_processing_ms = optimization.processing_ms;
    std::vector<FixedAmbiguityConstraint> fixed_constraints;
    const bool has_ambiguity_measurements =
        !problem.carrier_phase_factors.empty() ||
        !problem.double_difference_carrier_factors.empty();
    if (config_.fix_ambiguities &&
        ambiguity_count > 0 &&
        has_ambiguity_measurements) {
        const double max_fractional =
            std::max(0.0, config_.ambiguity_fix_max_fractional_cycles);
        double fixed_residual_square_sum = 0.0;
        const bool has_double_difference_ambiguities =
            std::any_of(problem.ambiguity_states.begin(),
                        problem.ambiguity_states.end(),
                        [](const AmbiguityState& ambiguity) {
                            return ambiguity.is_double_difference;
                        });
        auto is_fix_candidate = [&](const AmbiguityState& ambiguity) -> bool {
            if (config_.prefer_double_difference_ambiguity_fixing &&
                has_double_difference_ambiguities) {
                return ambiguity.is_double_difference;
            }
            return true;
        };

        auto build_nearest_integer_constraints = [&]() {
            fixed_constraints.clear();
            fixed_residual_square_sum = 0.0;
            for (int i = 0; i < ambiguity_count; ++i) {
                const auto& ambiguity = problem.ambiguity_states[i];
                if (ambiguity.wavelength_m <= 0.0 || !is_fix_candidate(ambiguity)) {
                    continue;
                }

                const double ambiguity_m = optimization.state(base_state_size + i);
                const double ambiguity_cycles = ambiguity_m / ambiguity.wavelength_m;
                if (!std::isfinite(ambiguity_cycles)) {
                    continue;
                }

                ++result.diagnostics.ambiguity_fix_candidates;
                const double fixed_cycles_d = std::round(ambiguity_cycles);
                const double residual_cycles = ambiguity_cycles - fixed_cycles_d;
                if (std::abs(residual_cycles) > max_fractional) {
                    continue;
                }

                FixedAmbiguityConstraint fixed;
                fixed.ambiguity_index = static_cast<std::size_t>(i);
                fixed.fixed_cycles = static_cast<int>(fixed_cycles_d);
                fixed.fixed_ambiguity_m = fixed_cycles_d * ambiguity.wavelength_m;
                fixed.residual_cycles = residual_cycles;
                fixed_constraints.push_back(fixed);
                fixed_residual_square_sum += residual_cycles * residual_cycles;
            }
        };

        auto build_lambda_constraints = [&]() -> bool {
            if (!config_.use_lambda_ambiguity_fix ||
                optimization.normal_matrix.rows() != state_size) {
                return false;
            }

            const Eigen::MatrixXd float_covariance =
                pseudoInverse(optimization.normal_matrix);
            if (float_covariance.rows() != state_size) {
                return false;
            }

            struct LambdaCandidate {
                int ambiguity_index = 0;
                double variance_cycles = 0.0;
                double fractional_cycles = 0.0;
            };

            std::vector<LambdaCandidate> candidates;
            candidates.reserve(problem.ambiguity_states.size());
            for (int i = 0; i < ambiguity_count; ++i) {
                const auto& ambiguity = problem.ambiguity_states[i];
                if (ambiguity.wavelength_m <= 0.0 || !is_fix_candidate(ambiguity)) {
                    continue;
                }

                const int col = base_state_size + i;
                const double variance_m2 = float_covariance(col, col);
                const double variance_cycles =
                    variance_m2 / (ambiguity.wavelength_m * ambiguity.wavelength_m);
                const double ambiguity_cycles =
                    optimization.state(col) / ambiguity.wavelength_m;
                if (!std::isfinite(variance_cycles) || variance_cycles <= 0.0 ||
                    !std::isfinite(ambiguity_cycles)) {
                    continue;
                }

                const double nearest_cycles = std::round(ambiguity_cycles);
                LambdaCandidate candidate;
                candidate.ambiguity_index = i;
                candidate.variance_cycles = variance_cycles;
                candidate.fractional_cycles = std::abs(ambiguity_cycles - nearest_cycles);
                candidates.push_back(candidate);
            }

            std::stable_sort(candidates.begin(),
                             candidates.end(),
                             [](const LambdaCandidate& lhs,
                                const LambdaCandidate& rhs) {
                                 if (lhs.fractional_cycles == rhs.fractional_cycles) {
                                     return lhs.variance_cycles < rhs.variance_cycles;
                                 }
                                 return lhs.fractional_cycles < rhs.fractional_cycles;
                             });

            const int max_lambda_ambiguities =
                std::max(0, config_.max_lambda_ambiguities);
            if (max_lambda_ambiguities > 0 &&
                static_cast<int>(candidates.size()) > max_lambda_ambiguities) {
                candidates.resize(static_cast<std::size_t>(max_lambda_ambiguities));
            }

            result.diagnostics.lambda_ambiguity_candidates = candidates.size();
            result.diagnostics.ambiguity_fix_candidates = candidates.size();
            const int min_fixed_ambiguities = std::max(1, config_.min_fixed_ambiguities);
            if (static_cast<int>(candidates.size()) < min_fixed_ambiguities) {
                return false;
            }

            auto solve_candidate_subset = [&](std::size_t subset_size) -> bool {
                const int n = static_cast<int>(subset_size);
                Eigen::VectorXd float_ambiguities = Eigen::VectorXd::Zero(n);
                Eigen::MatrixXd ambiguity_covariance = Eigen::MatrixXd::Zero(n, n);
                for (int row = 0; row < n; ++row) {
                    const int ambiguity_row = candidates[row].ambiguity_index;
                    const auto& row_state = problem.ambiguity_states[ambiguity_row];
                    const int row_col = base_state_size + ambiguity_row;
                    float_ambiguities(row) =
                        optimization.state(row_col) / row_state.wavelength_m;
                    for (int col = 0; col < n; ++col) {
                        const int ambiguity_col = candidates[col].ambiguity_index;
                        const auto& col_state = problem.ambiguity_states[ambiguity_col];
                        const int state_col = base_state_size + ambiguity_col;
                        ambiguity_covariance(row, col) =
                            float_covariance(row_col, state_col) /
                            (row_state.wavelength_m * col_state.wavelength_m);
                    }
                }

                ambiguity_covariance =
                    0.5 * (ambiguity_covariance + ambiguity_covariance.transpose());
                for (int i = 0; i < n; ++i) {
                    const double diagonal = std::abs(ambiguity_covariance(i, i));
                    ambiguity_covariance(i, i) += std::max(1e-12, diagonal * 1e-9);
                }

                Eigen::VectorXd fixed_ambiguities;
                double lambda_ratio = 0.0;
                ++result.diagnostics.lambda_ambiguity_attempts;
                const bool lambda_solved =
                    lambdaSearch(float_ambiguities,
                                 ambiguity_covariance,
                                 fixed_ambiguities,
                                 lambda_ratio);
                if (lambda_solved) {
                    result.diagnostics.lambda_ambiguity_fix_solved = true;
                }
                if (lambda_solved && std::isfinite(lambda_ratio)) {
                    result.diagnostics.lambda_ambiguity_ratio =
                        std::max(result.diagnostics.lambda_ambiguity_ratio,
                                 lambda_ratio);
                }
                if (!lambda_solved || !std::isfinite(lambda_ratio) ||
                    (config_.lambda_ratio_threshold > 0.0 &&
                     lambda_ratio < config_.lambda_ratio_threshold)) {
                    return false;
                }

                std::vector<FixedAmbiguityConstraint> lambda_constraints;
                lambda_constraints.reserve(subset_size);
                double lambda_residual_square_sum = 0.0;
                for (int i = 0; i < n; ++i) {
                    const int ambiguity_index = candidates[i].ambiguity_index;
                    const auto& ambiguity = problem.ambiguity_states[ambiguity_index];
                    const double fixed_cycles_d = std::round(fixed_ambiguities(i));
                    const double residual_cycles = float_ambiguities(i) - fixed_cycles_d;

                    FixedAmbiguityConstraint fixed;
                    fixed.ambiguity_index = static_cast<std::size_t>(ambiguity_index);
                    fixed.fixed_cycles = static_cast<int>(fixed_cycles_d);
                    fixed.fixed_ambiguity_m = fixed_cycles_d * ambiguity.wavelength_m;
                    fixed.residual_cycles = residual_cycles;
                    fixed.fixed_by_lambda = true;
                    lambda_constraints.push_back(fixed);
                    lambda_residual_square_sum += residual_cycles * residual_cycles;
                }

                if (static_cast<int>(lambda_constraints.size()) <
                    min_fixed_ambiguities) {
                    return false;
                }

                fixed_constraints = std::move(lambda_constraints);
                fixed_residual_square_sum = lambda_residual_square_sum;
                result.diagnostics.lambda_ambiguity_fix_used = true;
                result.diagnostics.partial_lambda_ambiguity_fix_used =
                    subset_size < candidates.size();
                result.diagnostics.lambda_ambiguity_used_candidates = subset_size;
                result.diagnostics.lambda_ambiguity_ratio = lambda_ratio;
                return true;
            };

            const std::size_t min_subset_size =
                static_cast<std::size_t>(min_fixed_ambiguities);
            for (std::size_t subset_size = candidates.size();
                 subset_size >= min_subset_size;
                 --subset_size) {
                if (solve_candidate_subset(subset_size)) {
                    return true;
                }
                if (!config_.use_partial_lambda_ambiguity_fix ||
                    subset_size == min_subset_size) {
                    break;
                }
            }

            return false;
        };

        const bool can_attempt_lambda =
            config_.use_lambda_ambiguity_fix &&
            optimization.normal_matrix.rows() == state_size;
        const bool lambda_constraints_built =
            can_attempt_lambda && build_lambda_constraints();
        if (!lambda_constraints_built &&
            (!config_.use_lambda_ambiguity_fix || !can_attempt_lambda)) {
            build_nearest_integer_constraints();
        }

        if (static_cast<int>(fixed_constraints.size()) >=
            std::max(1, config_.min_fixed_ambiguities)) {
            const int fixed_global_iteration_offset =
                result.cost_trace_entries.empty()
                    ? total_iterations
                    : result.cost_trace_entries.back().global_iteration + 1;
            OptimizationOutput fixed_optimization =
                run_optimizer(optimization.state,
                              fixed_constraints,
                              "fixed",
                              fixed_global_iteration_offset);
            total_iterations += fixed_optimization.iterations;
            total_processing_ms += fixed_optimization.processing_ms;
            result.cost_trace_entries.insert(
                result.cost_trace_entries.end(),
                fixed_optimization.cost_trace_entries.begin(),
                fixed_optimization.cost_trace_entries.end());
            optimization = std::move(fixed_optimization);
            result.diagnostics.fixed_solution = true;
            result.diagnostics.fixed_ambiguities = fixed_constraints.size();
            result.diagnostics.fixed_ambiguity_residual_rms_cycles =
                std::sqrt(fixed_residual_square_sum /
                          static_cast<double>(fixed_constraints.size()));
        } else {
            fixed_constraints.clear();
        }
    }

    const Eigen::VectorXd& state = optimization.state;
    result.diagnostics.iterations = total_iterations;
    if (!result.cost_trace_entries.empty()) {
        result.diagnostics.initial_cost = result.cost_trace_entries.front().cost;
    }
    result.diagnostics.final_cost = optimization.final_cost;
    result.diagnostics.converged = optimization.converged;
    if (!result.diagnostics.converged &&
        std::isfinite(optimization.last_dx_norm) &&
        optimization.last_dx_norm < config_.convergence_threshold_m) {
        result.diagnostics.converged = true;
    }
    const double processing_ms = total_processing_ms;
    result.diagnostics.processing_time_ms = processing_ms;
    result.diagnostics.last_update_norm_m =
        std::isfinite(optimization.last_dx_norm) ? optimization.last_dx_norm : 0.0;

    Eigen::MatrixXd covariance;
    if (optimization.normal_matrix.rows() == state_size) {
        covariance = pseudoInverse(optimization.normal_matrix);
    }

    std::vector<Vector3d> epoch_output_positions(
        static_cast<std::size_t>(num_epochs),
        Vector3d::Zero());
    std::vector<bool> epoch_lambda_fixed_output(
        static_cast<std::size_t>(num_epochs),
        false);
    std::vector<double> epoch_lambda_ratios(
        static_cast<std::size_t>(num_epochs),
        0.0);
    std::vector<int> epoch_fixed_ambiguity_counts(
        static_cast<std::size_t>(num_epochs),
        0);
    for (int i = 0; i < num_epochs; ++i) {
        epoch_output_positions[static_cast<std::size_t>(i)] =
            state.segment<3>(epoch_state_col(static_cast<std::size_t>(i)));
    }
    std::size_t epoch_lambda_fixed_solution_count = 0;
    std::size_t epoch_lambda_fixed_ambiguity_total = 0;

    const bool compute_epoch_lambda =
        (config_.collect_lambda_debug ||
         config_.use_epoch_lambda_fixed_output) &&
        ambiguity_count > 0;
    const auto epoch_lambda_start =
        std::chrono::high_resolution_clock::now();
    if (compute_epoch_lambda) {
        const auto elapsed_ms = [](const auto& start, const auto& end) {
            return std::chrono::duration_cast<
                       std::chrono::duration<double, std::milli>>(end - start)
                .count();
        };
        const auto setup_start = std::chrono::high_resolution_clock::now();
        std::map<std::size_t, std::set<std::size_t>> ambiguity_indices_by_epoch;
        for (const auto& factor : problem.double_difference_carrier_factors) {
            if (factor.epoch_index < problem.epochs.size() &&
                factor.ambiguity_index < problem.ambiguity_states.size()) {
                ambiguity_indices_by_epoch[factor.epoch_index].insert(
                    factor.ambiguity_index);
            }
        }

        const int min_lambda_debug_candidates =
            std::max(6, std::max(1, config_.min_fixed_ambiguities + 1));
        struct EpochLambdaWork {
            std::size_t epoch_index = 0;
            std::vector<std::size_t> candidate_indices;
            std::vector<int> state_columns;
            int epoch_col = 0;
            int candidate_count = 0;
        };
        std::vector<EpochLambdaWork> epoch_lambda_work;
        epoch_lambda_work.reserve(ambiguity_indices_by_epoch.size());
        for (const auto& [epoch_index, ambiguity_set] :
             ambiguity_indices_by_epoch) {
            if (static_cast<int>(ambiguity_set.size()) <
                min_lambda_debug_candidates) {
                continue;
            }

            EpochLambdaWork work;
            work.epoch_index = epoch_index;
            work.candidate_indices.assign(ambiguity_set.begin(),
                                          ambiguity_set.end());
            std::sort(
                work.candidate_indices.begin(),
                work.candidate_indices.end(),
                [&](std::size_t lhs, std::size_t rhs) {
                    const auto& lhs_ambiguity = problem.ambiguity_states[lhs];
                    const auto& rhs_ambiguity = problem.ambiguity_states[rhs];
                    return std::tie(lhs_ambiguity.satellite,
                                    lhs_ambiguity.reference_satellite,
                                    lhs_ambiguity.signal,
                                    lhs) <
                           std::tie(rhs_ambiguity.satellite,
                                    rhs_ambiguity.reference_satellite,
                                    rhs_ambiguity.signal,
                                    rhs);
                });

            work.candidate_count =
                static_cast<int>(work.candidate_indices.size());
            work.state_columns.reserve(
                static_cast<std::size_t>(work.candidate_count));
            for (const std::size_t ambiguity_index : work.candidate_indices) {
                work.state_columns.push_back(
                    base_state_size + static_cast<int>(ambiguity_index));
            }
            work.epoch_col = epoch_state_col(epoch_index);
            epoch_lambda_work.push_back(std::move(work));
        }

        if (config_.collect_lambda_debug) {
            std::size_t lambda_debug_capacity = 0;
            for (const auto& work : epoch_lambda_work) {
                lambda_debug_capacity +=
                    static_cast<std::size_t>(work.candidate_count) *
                    static_cast<std::size_t>(work.candidate_count);
            }
            result.lambda_debug_entries.reserve(lambda_debug_capacity);
        }
        result.diagnostics.epoch_lambda_setup_time_ms += elapsed_ms(
            setup_start, std::chrono::high_resolution_clock::now());

#ifdef GNSSPP_HAS_CHOLMOD
        Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>>
            sparse_covariance_ldlt;
#else
        Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> sparse_covariance_ldlt;
#endif
        bool has_sparse_covariance_solver = false;
        if (!epoch_lambda_work.empty() &&
            covariance.rows() != state_size &&
            optimization.sparse_normal_matrix.rows() == state_size) {
            const auto factorization_start =
                std::chrono::high_resolution_clock::now();
            double max_diagonal = 0.0;
            for (int i = 0; i < state_size; ++i) {
                max_diagonal =
                    std::max(max_diagonal,
                             std::abs(optimization.sparse_normal_matrix.coeff(i, i)));
            }
            const double damping = std::max(1e-12, max_diagonal * 1e-12);
            sparse_covariance_ldlt.setShift(damping);
            sparse_covariance_ldlt.compute(optimization.sparse_normal_matrix);
            has_sparse_covariance_solver =
                sparse_covariance_ldlt.info() == Eigen::Success;
            result.diagnostics.epoch_lambda_factorization_time_ms +=
                elapsed_ms(factorization_start,
                           std::chrono::high_resolution_clock::now());
        }

        auto process_epoch_lambda_work =
            [&](const EpochLambdaWork& work, const auto& covarianceValue) {
                const std::size_t epoch_index = work.epoch_index;
                const auto& candidate_indices = work.candidate_indices;
                const int candidate_count = work.candidate_count;
                const int epoch_col = work.epoch_col;
                Eigen::VectorXd ambiguity_float =
                    Eigen::VectorXd::Zero(candidate_count);
                Eigen::MatrixXd ambiguity_covariance =
                    Eigen::MatrixXd::Zero(candidate_count, candidate_count);
                bool valid_covariance = true;
                for (int row = 0; row < candidate_count; ++row) {
                    const auto& row_ambiguity =
                        problem.ambiguity_states[candidate_indices[row]];
                    if (row_ambiguity.wavelength_m <= 0.0) {
                        valid_covariance = false;
                        break;
                    }
                    const int row_col =
                        work.state_columns[static_cast<std::size_t>(row)];
                    ambiguity_float(row) =
                        state(row_col) / row_ambiguity.wavelength_m;
                    for (int col = 0; col < candidate_count; ++col) {
                        const auto& col_ambiguity =
                            problem.ambiguity_states[candidate_indices[col]];
                        if (col_ambiguity.wavelength_m <= 0.0) {
                            valid_covariance = false;
                            break;
                        }
                        ambiguity_covariance(row, col) =
                            covarianceValue(row_col, col) /
                            (row_ambiguity.wavelength_m *
                             col_ambiguity.wavelength_m);
                    }
                    if (!valid_covariance) {
                        break;
                    }
                }
                if (!valid_covariance || !ambiguity_float.allFinite() ||
                    !ambiguity_covariance.allFinite()) {
                    return;
                }

                ambiguity_covariance =
                    0.5 * (ambiguity_covariance +
                           ambiguity_covariance.transpose());
                for (int i = 0; i < candidate_count; ++i) {
                    const double diagonal = std::abs(ambiguity_covariance(i, i));
                    ambiguity_covariance(i, i) +=
                        std::max(1e-12, diagonal * 1e-9);
                }

                Eigen::VectorXd fixed_ambiguities;
                double lambda_ratio = 0.0;
                const auto lambda_search_start =
                    std::chrono::high_resolution_clock::now();
                const bool lambda_solved =
                    lambdaSearch(ambiguity_float,
                                 ambiguity_covariance,
                                 fixed_ambiguities,
                                 lambda_ratio);
                result.diagnostics.epoch_lambda_search_time_ms += elapsed_ms(
                    lambda_search_start,
                    std::chrono::high_resolution_clock::now());
                const bool fixed_epoch =
                    lambda_solved && std::isfinite(lambda_ratio) &&
                    (config_.lambda_ratio_threshold <= 0.0 ||
                     lambda_ratio > config_.lambda_ratio_threshold);

                result.diagnostics.lambda_ambiguity_candidates +=
                    static_cast<std::size_t>(candidate_count);
                ++result.diagnostics.lambda_ambiguity_attempts;

                if (lambda_solved) {
                    result.diagnostics.lambda_ambiguity_fix_solved = true;
                    result.diagnostics.lambda_ambiguity_ratio =
                        std::max(result.diagnostics.lambda_ambiguity_ratio,
                                 lambda_ratio);
                    epoch_lambda_ratios[epoch_index] = lambda_ratio;
                }

                if (config_.use_epoch_lambda_fixed_output && fixed_epoch &&
                    fixed_ambiguities.size() == candidate_count) {
                    const auto fixed_output_start =
                        std::chrono::high_resolution_clock::now();
                    Eigen::MatrixXd position_ambiguity_covariance =
                        Eigen::MatrixXd::Zero(3, candidate_count);
                    for (int row = 0; row < candidate_count; ++row) {
                        const auto& row_ambiguity =
                            problem.ambiguity_states[candidate_indices[row]];
                        if (row_ambiguity.wavelength_m <= 0.0) {
                            valid_covariance = false;
                            break;
                        }
                        position_ambiguity_covariance(0, row) =
                            covarianceValue(epoch_col, row) /
                            row_ambiguity.wavelength_m;
                        position_ambiguity_covariance(1, row) =
                            covarianceValue(epoch_col + 1, row) /
                            row_ambiguity.wavelength_m;
                        position_ambiguity_covariance(2, row) =
                            covarianceValue(epoch_col + 2, row) /
                            row_ambiguity.wavelength_m;
                    }
                    if (valid_covariance &&
                        position_ambiguity_covariance.allFinite()) {
                        Eigen::LDLT<Eigen::MatrixXd> ambiguity_ldlt(
                            ambiguity_covariance);
                        if (ambiguity_ldlt.info() == Eigen::Success) {
                            const Eigen::VectorXd ambiguity_delta =
                                ambiguity_float - fixed_ambiguities;
                            const Eigen::VectorXd correction =
                                ambiguity_ldlt.solve(ambiguity_delta);
                            if (ambiguity_ldlt.info() == Eigen::Success &&
                                correction.allFinite()) {
                                const Vector3d position_delta =
                                    position_ambiguity_covariance * correction;
                                if (position_delta.allFinite()) {
                                    epoch_output_positions[epoch_index] =
                                        state.segment<3>(epoch_col) -
                                        position_delta;
                                    epoch_lambda_fixed_output[epoch_index] = true;
                                    epoch_fixed_ambiguity_counts[epoch_index] =
                                        candidate_count;
                                    ++epoch_lambda_fixed_solution_count;
                                    epoch_lambda_fixed_ambiguity_total +=
                                        static_cast<std::size_t>(candidate_count);
                                    result.diagnostics
                                        .lambda_ambiguity_used_candidates +=
                                        static_cast<std::size_t>(candidate_count);
                                    result.diagnostics
                                        .lambda_ambiguity_fix_used = true;
                                }
                            }
                        }
                    }
                    result.diagnostics.epoch_lambda_fixed_output_time_ms +=
                        elapsed_ms(fixed_output_start,
                                   std::chrono::high_resolution_clock::now());
                }

                if (config_.collect_lambda_debug) {
                    const auto debug_record_start =
                        std::chrono::high_resolution_clock::now();
                    for (int row = 0; row < candidate_count; ++row) {
                        const std::size_t row_ambiguity_index =
                            candidate_indices[row];
                        const auto& row_ambiguity =
                            problem.ambiguity_states[row_ambiguity_index];
                        for (int col = 0; col < candidate_count; ++col) {
                            const auto& col_ambiguity =
                                problem.ambiguity_states[candidate_indices[col]];
                            LambdaDebugEntry entry;
                            entry.epoch_index = epoch_index;
                            entry.time = problem.epochs[epoch_index].time;
                            entry.solved = lambda_solved;
                            entry.fixed_epoch = fixed_epoch;
                            entry.ratio = lambda_solved ? lambda_ratio : 0.0;
                            entry.candidate_count = candidate_count;
                            entry.row = row;
                            entry.col = col;
                            entry.local_index = row;
                            entry.other_local_index = col;
                            entry.satellite = row_ambiguity.satellite;
                            entry.other_satellite = col_ambiguity.satellite;
                            entry.ambiguity_float = ambiguity_float(row);
                            entry.fixed_ambiguity =
                                lambda_solved && fixed_ambiguities.size() > row
                                    ? fixed_ambiguities(row)
                                    : std::numeric_limits<double>::quiet_NaN();
                            entry.covariance = ambiguity_covariance(row, col);
                            entry.position_covariance_x =
                                covarianceValue(epoch_col, row) /
                                row_ambiguity.wavelength_m;
                            entry.position_covariance_y =
                                covarianceValue(epoch_col + 1, row) /
                                row_ambiguity.wavelength_m;
                            entry.position_covariance_z =
                                covarianceValue(epoch_col + 2, row) /
                                row_ambiguity.wavelength_m;
                            result.lambda_debug_entries.push_back(entry);
                        }
                    }
                    result.diagnostics.epoch_lambda_debug_record_time_ms +=
                        elapsed_ms(debug_record_start,
                                   std::chrono::high_resolution_clock::now());
                }
            };

        if (covariance.rows() == state_size) {
            for (const auto& work : epoch_lambda_work) {
                process_epoch_lambda_work(
                    work,
                    [&](int row, int column_index) -> double {
                        return covariance(
                            row,
                            work.state_columns[static_cast<std::size_t>(
                                column_index)]);
                    });
            }
        } else if (has_sparse_covariance_solver) {
            constexpr int kMaxSparseCovarianceBatchColumns = 64;
            for (std::size_t begin = 0; begin < epoch_lambda_work.size();) {
                std::size_t end = begin;
                int batch_column_count = 0;
                while (end < epoch_lambda_work.size()) {
                    const int next_count =
                        epoch_lambda_work[end].candidate_count;
                    if (batch_column_count > 0 &&
                        batch_column_count + next_count >
                            kMaxSparseCovarianceBatchColumns) {
                        break;
                    }
                    batch_column_count += next_count;
                    ++end;
                }

                Eigen::MatrixXd rhs =
                    Eigen::MatrixXd::Zero(state_size, batch_column_count);
                int column_offset = 0;
                for (std::size_t work_index = begin; work_index < end;
                     ++work_index) {
                    const auto& work = epoch_lambda_work[work_index];
                    for (int col = 0; col < work.candidate_count; ++col) {
                        rhs(work.state_columns[static_cast<std::size_t>(col)],
                            column_offset + col) = 1.0;
                    }
                    column_offset += work.candidate_count;
                }

                const auto covariance_solve_start =
                    std::chrono::high_resolution_clock::now();
                const Eigen::MatrixXd batch_covariance_columns =
                    sparse_covariance_ldlt.solve(rhs);
                result.diagnostics.epoch_lambda_covariance_solve_time_ms +=
                    elapsed_ms(covariance_solve_start,
                               std::chrono::high_resolution_clock::now());
                if (sparse_covariance_ldlt.info() != Eigen::Success ||
                    batch_covariance_columns.cols() != batch_column_count) {
                    begin = end;
                    continue;
                }

                column_offset = 0;
                for (std::size_t work_index = begin; work_index < end;
                     ++work_index) {
                    const auto& work = epoch_lambda_work[work_index];
                    const int work_column_offset = column_offset;
                    process_epoch_lambda_work(
                        work,
                        [&](int row, int column_index) -> double {
                            return batch_covariance_columns(
                                row, work_column_offset + column_index);
                        });
                    column_offset += work.candidate_count;
                }

                begin = end;
            }
        }
    }
    if (compute_epoch_lambda) {
        const auto epoch_lambda_end =
            std::chrono::high_resolution_clock::now();
        result.diagnostics.epoch_lambda_processing_time_ms =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                epoch_lambda_end - epoch_lambda_start)
                .count();
    }

    const bool has_epoch_lambda_fixed_outputs =
        epoch_lambda_fixed_solution_count > 0;
    if (has_epoch_lambda_fixed_outputs) {
        result.diagnostics.fixed_solution = true;
        result.diagnostics.fixed_ambiguities =
            epoch_lambda_fixed_ambiguity_total;
    }

    const std::size_t rows_per_motion_pair = motion_rows_per_pair();
    result.diagnostics.motion_factors =
        rows_per_motion_pair > 0
            ? motion_factor_count() / rows_per_motion_pair
            : 0;
    const std::size_t position_prior_factors =
        config_.position_prior_sigma_m > 0.0
            ? static_cast<std::size_t>(num_epochs)
            : 0;
    const std::size_t clock_prior_factors =
        config_.clock_prior_sigma_m > 0.0
            ? static_cast<std::size_t>(num_epochs)
            : 0;
    const std::size_t velocity_prior_factors =
        use_velocity_states && config_.velocity_prior_sigma_mps > 0.0
            ? static_cast<std::size_t>(num_epochs)
            : 0;
    const std::size_t ambiguity_prior_factors =
        config_.use_ambiguity_priors && config_.ambiguity_prior_sigma_m > 0.0
            ? problem.ambiguity_states.size()
            : 0;
    result.diagnostics.graph_factors =
        result.diagnostics.pseudorange_factors +
        result.diagnostics.tdcp_factors +
        result.diagnostics.undifferenced_doppler_factors +
        result.diagnostics.single_difference_doppler_factors +
        result.diagnostics.single_difference_tdcp_factors +
        result.diagnostics.carrier_phase_factors +
        result.diagnostics.double_difference_pseudorange_factors +
        result.diagnostics.double_difference_carrier_factors +
        result.diagnostics.ambiguity_between_factors +
        result.diagnostics.motion_factors +
        position_prior_factors +
        clock_prior_factors +
        velocity_prior_factors +
        ambiguity_prior_factors +
        fixed_constraints.size();
    result.diagnostics.graph_values =
        static_cast<std::size_t>(state_size);
    result.diagnostics.robust_pseudorange_factors =
        optimization.robust_pseudorange_factors;
    result.diagnostics.robust_carrier_phase_factors =
        optimization.robust_carrier_phase_factors;
    result.diagnostics.robust_double_difference_pseudorange_factors =
        optimization.robust_double_difference_pseudorange_factors;
    result.diagnostics.robust_double_difference_carrier_factors =
        optimization.robust_double_difference_carrier_factors;
    result.diagnostics.robust_tdcp_factors = optimization.robust_tdcp_factors;
    result.ambiguity_estimates.reserve(problem.ambiguity_states.size());
    for (std::size_t i = 0; i < problem.ambiguity_states.size(); ++i) {
        const auto& ambiguity = problem.ambiguity_states[i];
        AmbiguityEstimate estimate;
        estimate.satellite = ambiguity.satellite;
        estimate.signal = ambiguity.signal;
        estimate.segment_index = ambiguity.segment_index;
        estimate.wavelength_m = ambiguity.wavelength_m;
        estimate.ambiguity_m = state(base_state_size + static_cast<int>(i));
        if (ambiguity.wavelength_m > 0.0) {
            estimate.ambiguity_cycles = estimate.ambiguity_m / ambiguity.wavelength_m;
        }
        const auto fixed_it =
            std::find_if(fixed_constraints.begin(),
                         fixed_constraints.end(),
                         [i](const FixedAmbiguityConstraint& fixed) {
                             return fixed.ambiguity_index == i;
                         });
        if (fixed_it != fixed_constraints.end()) {
            estimate.is_fixed = true;
            estimate.fixed_cycles = fixed_it->fixed_cycles;
            estimate.fixed_ambiguity_m = fixed_it->fixed_ambiguity_m;
            estimate.fix_residual_cycles = fixed_it->residual_cycles;
            estimate.fixed_by_lambda = fixed_it->fixed_by_lambda;
        }
        result.ambiguity_estimates.push_back(estimate);
    }

    std::map<std::size_t, std::vector<const PseudorangeFactor*>> factors_by_epoch;
    for (const auto& factor : problem.pseudorange_factors) {
        factors_by_epoch[factor.epoch_index].push_back(&factor);
    }
    std::map<std::size_t, std::set<SatelliteId>> dd_satellites_by_epoch;
    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        dd_satellites_by_epoch[factor.epoch_index].insert(factor.satellite);
        dd_satellites_by_epoch[factor.epoch_index].insert(
            factor.reference_satellite);
    }
    for (const auto& factor : problem.double_difference_carrier_factors) {
        dd_satellites_by_epoch[factor.epoch_index].insert(factor.satellite);
        dd_satellites_by_epoch[factor.epoch_index].insert(
            factor.reference_satellite);
    }
    std::map<std::size_t, std::size_t> dd_carrier_factors_by_epoch;
    for (const auto& factor : problem.double_difference_carrier_factors) {
        ++dd_carrier_factors_by_epoch[factor.epoch_index];
    }

    double residual_square_sum = 0.0;
    std::size_t residual_count = 0;
    double carrier_phase_residual_square_sum = 0.0;
    std::size_t carrier_phase_residual_count = 0;
    double double_difference_pseudorange_residual_square_sum = 0.0;
    std::size_t double_difference_pseudorange_residual_count = 0;
    double double_difference_carrier_residual_square_sum = 0.0;
    std::size_t double_difference_carrier_residual_count = 0;
    double tdcp_residual_square_sum = 0.0;
    std::size_t tdcp_residual_count = 0;
    double undifferenced_doppler_residual_square_sum = 0.0;
    std::size_t undifferenced_doppler_residual_count = 0;
    double single_difference_doppler_residual_square_sum = 0.0;
    std::size_t single_difference_doppler_residual_count = 0;
    double single_difference_tdcp_residual_square_sum = 0.0;
    std::size_t single_difference_tdcp_residual_count = 0;
    const bool has_float_ambiguity_solution =
        ambiguity_count > 0 && has_ambiguity_measurements;
    const SolutionStatus fgo_solution_status =
        result.diagnostics.fixed_solution && !has_epoch_lambda_fixed_outputs
            ? SolutionStatus::FIXED
            : (has_float_ambiguity_solution ? SolutionStatus::FLOAT
                                            : SolutionStatus::SPP);

    if (use_velocity_states) {
        result.epoch_velocities_ecef_mps.resize(static_cast<std::size_t>(num_epochs));
    }

    Vector3d previous_output_position = Vector3d::Zero();
    bool has_previous_output_position = false;
    bool block_float_until_fixed = false;
    for (int i = 0; i < num_epochs; ++i) {
        const int epoch_col = epoch_state_col(static_cast<std::size_t>(i));
        PositionSolution solution;
        solution.time = problem.epochs[i].time;
        solution.status = fgo_solution_status;
        const std::size_t epoch_index = static_cast<std::size_t>(i);
        if (has_epoch_lambda_fixed_outputs && has_float_ambiguity_solution) {
            solution.status = epoch_lambda_fixed_output[epoch_index]
                                  ? SolutionStatus::FIXED
                                  : SolutionStatus::FLOAT;
        }
        solution.position_ecef = epoch_output_positions[epoch_index];
        solution.receiver_clock_bias =
            state(epoch_col + 3) / constants::SPEED_OF_LIGHT;
        solution.num_frequencies = 1;
        if (has_epoch_lambda_fixed_outputs && has_float_ambiguity_solution) {
            solution.ratio = epoch_lambda_ratios[epoch_index] > 0.0
                                 ? epoch_lambda_ratios[epoch_index]
                                 : 0.0;
        } else {
            solution.ratio =
                epoch_lambda_ratios[epoch_index] > 0.0
                    ? epoch_lambda_ratios[epoch_index]
                    : result.diagnostics.lambda_ambiguity_ratio;
        }
        solution.num_fixed_ambiguities =
            has_epoch_lambda_fixed_outputs
                ? (epoch_lambda_fixed_output[epoch_index]
                       ? epoch_fixed_ambiguity_counts[epoch_index]
                       : 0)
                : static_cast<int>(result.diagnostics.fixed_ambiguities);
        solution.iterations = result.diagnostics.iterations;
        solution.processing_time_ms = processing_ms / static_cast<double>(num_epochs);
        solution.position_covariance = Matrix3d::Identity() * 9999.0;

        double lat = 0.0;
        double lon = 0.0;
        double height = 0.0;
        ecef2geodetic(solution.position_ecef, lat, lon, height);
        solution.position_geodetic = GeodeticCoord(lat, lon, height);

        const auto epoch_factor_it = factors_by_epoch.find(static_cast<std::size_t>(i));
        bool has_epoch_pseudorange_solution = false;
        if (epoch_factor_it != factors_by_epoch.end()) {
            has_epoch_pseudorange_solution = true;
            const auto& epoch_factors = epoch_factor_it->second;
            solution.num_satellites = static_cast<int>(epoch_factors.size());
            solution.satellites_used.reserve(epoch_factors.size());
            solution.satellite_elevations.reserve(epoch_factors.size());
            solution.satellite_residuals.reserve(epoch_factors.size());

            Eigen::MatrixXd geometry = Eigen::MatrixXd::Zero(epoch_factors.size(), 4);
            for (int j = 0; j < static_cast<int>(epoch_factors.size()); ++j) {
                const auto& factor = *epoch_factors[j];
                const Vector3d delta = factor.satellite_position_ecef - solution.position_ecef;
                const double range = delta.norm();
                const Vector3d los = delta / range;
                const int bias_col = system_bias_col(epoch_col, factor.clock_group);
                double predicted_pseudorange = range + state(epoch_col + 3);
                if (bias_col >= 0) {
                    predicted_pseudorange += state(bias_col);
                }
                const double pseudorange_residual =
                    factor.corrected_pseudorange_m - predicted_pseudorange;

                solution.satellites_used.push_back(factor.satellite);
                solution.satellite_elevations.push_back(factor.elevation_rad);
                solution.satellite_residuals.push_back(pseudorange_residual);
                residual_square_sum += pseudorange_residual * pseudorange_residual;
                ++residual_count;

                geometry(j, 0) = -los(0);
                geometry(j, 1) = -los(1);
                geometry(j, 2) = -los(2);
                geometry(j, 3) = 1.0;
            }

            solution.residual_rms = std::sqrt(std::accumulate(
                solution.satellite_residuals.begin(),
                solution.satellite_residuals.end(),
                0.0,
                [](double acc, double residual) {
                    return acc + residual * residual;
                }) / static_cast<double>(solution.satellite_residuals.size()));

            if (geometry.rows() >= 4) {
                const Eigen::MatrixXd q = pseudoInverse(geometry.transpose() * geometry);
                solution.gdop = std::sqrt(std::max(0.0, q.trace()));
                solution.pdop = std::sqrt(std::max(0.0, q(0, 0) + q(1, 1) + q(2, 2)));
                solution.hdop = std::sqrt(std::max(0.0, q(0, 0) + q(1, 1)));
                solution.vdop = std::sqrt(std::max(0.0, q(2, 2)));
            }
        } else {
            const auto dd_satellite_it =
                dd_satellites_by_epoch.find(static_cast<std::size_t>(i));
            if (dd_satellite_it != dd_satellites_by_epoch.end()) {
                solution.num_satellites =
                    static_cast<int>(dd_satellite_it->second.size());
                solution.satellites_used.assign(dd_satellite_it->second.begin(),
                                                dd_satellite_it->second.end());
            }
        }

        if (!has_epoch_pseudorange_solution &&
            config_.min_output_double_difference_carrier_factors_per_epoch > 0) {
            const auto dd_carrier_count_it =
                dd_carrier_factors_by_epoch.find(static_cast<std::size_t>(i));
            const std::size_t dd_carrier_count =
                dd_carrier_count_it == dd_carrier_factors_by_epoch.end()
                    ? 0
                    : dd_carrier_count_it->second;
            if (dd_carrier_count <
                static_cast<std::size_t>(
                    config_
                        .min_output_double_difference_carrier_factors_per_epoch)) {
                solution.status = SolutionStatus::NONE;
            }
        }

        applyFloatPositionJumpGate(
            solution,
            previous_output_position,
            has_previous_output_position,
            config_.max_float_position_jump_m,
            block_float_until_fixed,
            result.diagnostics.float_rejected_position_jump);
        applyFloatSeedPositionDivergenceGate(
            solution,
            problem,
            static_cast<std::size_t>(i),
            config_.max_float_seed_position_divergence_m,
            result.diagnostics.float_rejected_seed_position_divergence);

        if (covariance.rows() == state_size) {
            solution.position_covariance =
                covariance.block<3, 3>(epoch_col, epoch_col) *
                (config_.pseudorange_sigma_m * config_.pseudorange_sigma_m);
        }

        result.solution.addSolution(solution);
        previous_output_position = solution.position_ecef;
        has_previous_output_position = true;

        if (use_velocity_states) {
            const int velocity_col = velocity_state_col(static_cast<std::size_t>(i));
            result.epoch_velocities_ecef_mps[static_cast<std::size_t>(i)] =
                state.segment<3>(velocity_col);
        }
    }

    for (const auto& factor : problem.carrier_phase_factors) {
        if (factor.epoch_index >= problem.epochs.size() ||
            factor.ambiguity_index >= problem.ambiguity_states.size()) {
            continue;
        }

        const int epoch_col = epoch_state_col(factor.epoch_index);
        const int ambiguity_col = base_state_size + static_cast<int>(factor.ambiguity_index);
        const Vector3d position = state.segment<3>(epoch_col);
        const double range = (factor.satellite_position_ecef - position).norm();
        const int bias_col = system_bias_col(epoch_col, factor.clock_group);
        double predicted = range + state(epoch_col + 3) + state(ambiguity_col);
        if (bias_col >= 0) {
            predicted += state(bias_col);
        }
        const double carrier_phase_residual = factor.corrected_carrier_m - predicted;
        carrier_phase_residual_square_sum +=
            carrier_phase_residual * carrier_phase_residual;
        ++carrier_phase_residual_count;
    }

    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        if (factor.epoch_index >= problem.epochs.size()) {
            continue;
        }

        const int epoch_col = epoch_state_col(factor.epoch_index);
        const Vector3d position = state.segment<3>(epoch_col);
        const Vector3d seed_position =
            problem.epochs[factor.epoch_index].position_ecef;
        const Vector3d linearization_position =
            config_.linearize_double_difference_factors_at_seed
                ? seed_position
                : position;
        const DoubleDifferencePrediction dd_prediction =
            doubleDifferencePredictionAt(linearization_position,
                                         factor.base_position_ecef,
                                         factor.rover_satellite_position_ecef,
                                         factor.rover_reference_position_ecef,
                                         factor.base_satellite_position_ecef,
                                         factor.base_reference_position_ecef);
        if (!dd_prediction.valid) {
            continue;
        }
        double predicted = dd_prediction.geometry_m;
        if (config_.linearize_double_difference_factors_at_seed) {
            predicted +=
                dd_prediction.position_jacobian.dot(position - seed_position);
        }
        const double residual =
            factor.observed_dd_pseudorange_m - predicted;
        double_difference_pseudorange_residual_square_sum += residual * residual;
        ++double_difference_pseudorange_residual_count;
    }

    for (const auto& factor : problem.double_difference_carrier_factors) {
        if (factor.epoch_index >= problem.epochs.size() ||
            factor.ambiguity_index >= problem.ambiguity_states.size() ||
            (factor.use_ambiguity_difference &&
             factor.reference_ambiguity_index >= problem.ambiguity_states.size())) {
            continue;
        }

        const int epoch_col = epoch_state_col(factor.epoch_index);
        const int ambiguity_col = base_state_size + static_cast<int>(factor.ambiguity_index);
        const int reference_ambiguity_col =
            factor.use_ambiguity_difference
                ? base_state_size +
                      static_cast<int>(factor.reference_ambiguity_index)
                : -1;
        const Vector3d position = state.segment<3>(epoch_col);
        const Vector3d seed_position =
            problem.epochs[factor.epoch_index].position_ecef;
        const Vector3d linearization_position =
            config_.linearize_double_difference_factors_at_seed
                ? seed_position
                : position;
        const DoubleDifferencePrediction dd_prediction =
            doubleDifferencePredictionAt(linearization_position,
                                         factor.base_position_ecef,
                                         factor.rover_satellite_position_ecef,
                                         factor.rover_reference_position_ecef,
                                         factor.base_satellite_position_ecef,
                                         factor.base_reference_position_ecef);
        if (!dd_prediction.valid) {
            continue;
        }
        double predicted = dd_prediction.geometry_m + state(ambiguity_col);
        if (config_.linearize_double_difference_factors_at_seed) {
            predicted +=
                dd_prediction.position_jacobian.dot(position - seed_position);
        }
        if (factor.use_ambiguity_difference) {
            predicted -= state(reference_ambiguity_col);
        }
        const double residual =
            factor.observed_dd_carrier_m - predicted;
        double_difference_carrier_residual_square_sum += residual * residual;
        ++double_difference_carrier_residual_count;
    }

    for (const auto& factor : problem.tdcp_factors) {
        if (factor.previous_epoch_index >= problem.epochs.size() ||
            factor.current_epoch_index >= problem.epochs.size()) {
            continue;
        }

        const int previous_col = epoch_state_col(factor.previous_epoch_index);
        const int current_col = epoch_state_col(factor.current_epoch_index);
        const int previous_bias_col =
            system_bias_col(previous_col, clockBiasGroup(factor.satellite.system));
        const int current_bias_col =
            system_bias_col(current_col, clockBiasGroup(factor.satellite.system));
        const Vector3d previous_position = state.segment<3>(previous_col);
        const Vector3d current_position = state.segment<3>(current_col);
        const double previous_range =
            (factor.previous_satellite_position_ecef - previous_position).norm();
        const double current_range =
            (factor.current_satellite_position_ecef - current_position).norm();
        double predicted =
            current_range + state(current_col + 3) -
            previous_range - state(previous_col + 3);
        if (previous_bias_col >= 0 && current_bias_col >= 0 &&
            previous_bias_col != current_bias_col) {
            predicted += state(current_bias_col) - state(previous_bias_col);
        }
        const double tdcp_residual = factor.delta_carrier_m - predicted;
        tdcp_residual_square_sum += tdcp_residual * tdcp_residual;
        ++tdcp_residual_count;
    }

    for (const auto& factor : problem.single_difference_doppler_factors) {
        if (factor.epoch_index >= problem.epochs.size()) {
            continue;
        }

        Vector3d velocity = Vector3d::Zero();
        const int velocity_col = velocity_state_col(factor.epoch_index);
        if (velocity_col >= 0) {
            velocity = state.segment<3>(velocity_col);
        } else {
            if (factor.epoch_index == 0) {
                continue;
            }
            const std::size_t previous_epoch_index = factor.epoch_index - 1;
            const double dt =
                problem.epochs[factor.epoch_index].time -
                problem.epochs[previous_epoch_index].time;
            if (dt <= 0.0 ||
                (config_.max_tdcp_gap_s > 0.0 && dt > config_.max_tdcp_gap_s)) {
                continue;
            }

            const int previous_col = epoch_state_col(previous_epoch_index);
            const int current_col = epoch_state_col(factor.epoch_index);
            velocity =
                (state.segment<3>(current_col) -
                 state.segment<3>(previous_col)) /
                dt;
        }
        const double residual = factor.residual_mps - factor.los.dot(velocity);
        single_difference_doppler_residual_square_sum += residual * residual;
        ++single_difference_doppler_residual_count;
    }

    for (const auto& factor : problem.undifferenced_doppler_factors) {
        if (factor.epoch_index >= problem.epochs.size()) {
            continue;
        }

        std::size_t clock_previous_epoch =
            std::numeric_limits<std::size_t>::max();
        double receiver_clock_drift_mps = 0.0;
        if (factor.includes_receiver_clock_drift) {
            clock_previous_epoch = factor.previous_epoch_index;
            if (clock_previous_epoch >= problem.epochs.size() ||
                clock_previous_epoch >= factor.epoch_index) {
                continue;
            }
            const double clock_dt = factor.dt_s > 0.0
                                        ? factor.dt_s
                                        : problem.epochs[factor.epoch_index].time -
                                              problem.epochs[clock_previous_epoch].time;
            if (!(clock_dt > 0.0) ||
                (config_.max_tdcp_gap_s > 0.0 &&
                 clock_dt > config_.max_tdcp_gap_s)) {
                continue;
            }
            receiver_clock_drift_mps =
                (state(epoch_state_col(factor.epoch_index) + 3) -
                 state(epoch_state_col(clock_previous_epoch) + 3)) /
                clock_dt;
            if (!std::isfinite(receiver_clock_drift_mps)) {
                continue;
            }
        }

        Vector3d velocity = Vector3d::Zero();
        const int velocity_col = velocity_state_col(factor.epoch_index);
        if (velocity_col >= 0) {
            velocity = state.segment<3>(velocity_col);
        } else {
            if (factor.epoch_index == 0) {
                continue;
            }
            const std::size_t previous_epoch_index = factor.epoch_index - 1;
            const double dt =
                problem.epochs[factor.epoch_index].time -
                problem.epochs[previous_epoch_index].time;
            if (dt <= 0.0 ||
                (config_.max_tdcp_gap_s > 0.0 && dt > config_.max_tdcp_gap_s)) {
                continue;
            }
            const int previous_col = epoch_state_col(previous_epoch_index);
            const int current_col = epoch_state_col(factor.epoch_index);
            velocity =
                (state.segment<3>(current_col) -
                 state.segment<3>(previous_col)) /
                dt;
        }
        const double residual =
            factor.residual_mps - doppler_velocity_wls::predict(
                                      factor.los, velocity,
                                      receiver_clock_drift_mps);
        undifferenced_doppler_residual_square_sum += residual * residual;
        ++undifferenced_doppler_residual_count;
    }

    for (const auto& factor : problem.single_difference_tdcp_factors) {
        if (factor.previous_epoch_index >= problem.epochs.size() ||
            factor.current_epoch_index >= problem.epochs.size()) {
            continue;
        }

        const int previous_col = epoch_state_col(factor.previous_epoch_index);
        const int current_col = epoch_state_col(factor.current_epoch_index);
        const Vector3d previous_delta =
            state.segment<3>(previous_col) -
            problem.epochs[factor.previous_epoch_index].position_ecef;
        const Vector3d current_delta =
            state.segment<3>(current_col) -
            problem.epochs[factor.current_epoch_index].position_ecef;
        const double predicted =
            factor.los.dot(current_delta) -
            factor.previous_los.dot(previous_delta);
        const double residual = factor.delta_carrier_m - predicted;
        single_difference_tdcp_residual_square_sum += residual * residual;
        ++single_difference_tdcp_residual_count;
    }

    if (residual_count > 0) {
        result.diagnostics.residual_rms_m =
            std::sqrt(residual_square_sum / static_cast<double>(residual_count));
    }
    if (carrier_phase_residual_count > 0) {
        result.diagnostics.carrier_phase_residual_rms_m =
            std::sqrt(carrier_phase_residual_square_sum /
                      static_cast<double>(carrier_phase_residual_count));
    }
    if (double_difference_pseudorange_residual_count > 0) {
        result.diagnostics.double_difference_pseudorange_residual_rms_m =
            std::sqrt(double_difference_pseudorange_residual_square_sum /
                      static_cast<double>(
                          double_difference_pseudorange_residual_count));
    }
    if (double_difference_carrier_residual_count > 0) {
        result.diagnostics.double_difference_carrier_residual_rms_m =
            std::sqrt(double_difference_carrier_residual_square_sum /
                      static_cast<double>(double_difference_carrier_residual_count));
    }
    if (tdcp_residual_count > 0) {
        result.diagnostics.tdcp_residual_rms_m =
            std::sqrt(tdcp_residual_square_sum / static_cast<double>(tdcp_residual_count));
    }
    if (undifferenced_doppler_residual_count > 0) {
        result.diagnostics.undifferenced_doppler_residual_rms_mps =
            std::sqrt(undifferenced_doppler_residual_square_sum /
                      static_cast<double>(undifferenced_doppler_residual_count));
    }
    if (single_difference_doppler_residual_count > 0) {
        result.diagnostics.single_difference_doppler_residual_rms_mps =
            std::sqrt(single_difference_doppler_residual_square_sum /
                      static_cast<double>(
                          single_difference_doppler_residual_count));
    }
    if (single_difference_tdcp_residual_count > 0) {
        result.diagnostics.single_difference_tdcp_residual_rms_m =
            std::sqrt(single_difference_tdcp_residual_square_sum /
                      static_cast<double>(
                          single_difference_tdcp_residual_count));
    }

    const auto optimize_problem_end =
        std::chrono::high_resolution_clock::now();
    result.diagnostics.total_processing_time_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            optimize_problem_end - optimize_problem_start)
            .count();
    result.diagnostics.postprocessing_time_ms =
        std::max(0.0,
                 result.diagnostics.total_processing_time_ms -
                     result.diagnostics.processing_time_ms);

    return result;
}

}  // namespace libgnss
