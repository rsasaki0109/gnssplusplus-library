#pragma once

// Shared helpers for the FGO GTSAM backend translation units
// (fgo_gtsam_backend.cpp, fgo_gtsam_fixed_lag.cpp). Extracted from the
// former single-TU anonymous namespace; every function is inline.
// Include only when GNSSPP_HAS_GTSAM is defined.

#include <libgnss++/algorithms/disjoint_constellation_partition.hpp>
#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/fgo_ddpr_gnc.hpp>
#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/algorithms/signal_bias_contract.hpp>
#include <libgnss++/algorithms/tdcp_contract.hpp>
#include <libgnss++/algorithms/upstream_stop_constraints.hpp>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/types.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/navigation/CarrierPhaseFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GnssCommon.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/PreintegrationCombinedParams.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PseudorangeFactor.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <iostream>
#include <iomanip>
#include <typeinfo>

#include <Eigen/Eigenvalues>
#include <Eigen/Dense>


namespace libgnss {

// IncrementalFixedLagSmoother path, implemented in fgo_gtsam_fixed_lag.cpp.
FGOProcessor::FGOResult optimizeProblemFixedLag(
    const FGOProcessor::FGOProblem& problem,
    const FGOProcessor::FGOConfig& config,
    FGOProcessor::FGOResult result);

namespace fgo_gtsam_internal {


using gtsam::Point3;
using gtsam::Pose3;

constexpr int kGfGuardMaxFixedAmbiguities = 6;
constexpr double kGfGuardMaxRatio = 10.0;
constexpr double kGfGuardMinDdprRmsM = 10.0;
constexpr double kGfGuardMaxSppSeparationM = 25.0;
using gtsam::Rot3;
using gtsam::Symbol;
using SharedNoise = gtsam::SharedNoiseModel;

// Phase201 deliberately has its own schedule helper instead of refactoring
// the legacy backend loop.  The cached source selects every sample in the
// inclusive interval and integrates each selected sample forward to the next
// native timestamp; when the final stream sample is selected it repeats the
// stream's final sample-to-sample delta.  There is no clipping to the GNSS boundary and no
// synthetic tail.  The helper consumes the already mapped native timestamps
// (typically GPST after the UTC/GPS affine map); any tiny UTC-vs-GPS slope
// distinction therefore remains in the caller's mapped times rather than
// being silently replaced with a fixed offset.
struct SourceInclusiveForwardImuSegment {
    std::size_t sample_index = 0;
    double dt_s = 0.0;
};

struct SourceInclusiveForwardImuSchedule {
    bool valid = false;
    std::vector<SourceInclusiveForwardImuSegment> segments;
    // First candidate for the next adjacent GNSS interval.  Keeping one
    // sample of overlap preserves an exactly-on-boundary sample without
    // rescanning the whole native stream for every epoch.
    std::size_t next_sample_index = 0;
    std::size_t invalid_sample_count = 0;
    std::size_t nonfinite_dt_count = 0;
    std::size_t nonpositive_dt_count = 0;
    bool empty_interval = false;
    double integrated_duration_s =
        std::numeric_limits<double>::quiet_NaN();
    std::string failure;
};

inline SourceInclusiveForwardImuSchedule makeSourceInclusiveForwardImuSchedule(
    const std::vector<ImuSample>& samples, const GNSSTime& t0,
    const GNSSTime& t1, std::size_t sample_start_index = 0,
    bool validate_all_samples = true) {
    SourceInclusiveForwardImuSchedule schedule;
    const double interval_s = t1 - t0;
    if (!std::isfinite(t0.tow) || !std::isfinite(t1.tow) ||
        !std::isfinite(interval_s)) {
        ++schedule.nonfinite_dt_count;
        schedule.failure = "non-finite GNSS interval boundary";
        return schedule;
    }
    if (!(interval_s > 0.0)) {
        ++schedule.nonpositive_dt_count;
        schedule.failure = "non-positive GNSS interval";
        return schedule;
    }

    if (sample_start_index > samples.size()) {
        schedule.failure = "IMU schedule cursor is outside the sample stream";
        return schedule;
    }
    // A malformed timestamp anywhere in the native stream is not allowed to
    // influence a selected sample's forward delta.  Validate the complete
    // stream once; adjacent production intervals then reuse the returned
    // overlap cursor instead of rescanning all native samples.
    if (validate_all_samples) {
        for (std::size_t i = 0; i < samples.size(); ++i) {
            if (!std::isfinite(samples[i].time.tow)) {
                ++schedule.invalid_sample_count;
                schedule.failure = "non-finite IMU sample timestamp";
                return schedule;
            }
            if (i > 0 && !(samples[i].time - samples[i - 1].time > 0.0)) {
                ++schedule.nonpositive_dt_count;
                schedule.failure = "non-positive IMU stream timestamp delta";
                return schedule;
            }
        }
    }

    double total_duration_s = 0.0;
    std::size_t i = sample_start_index;
    for (; i < samples.size(); ++i) {
        if (!std::isfinite(samples[i].time.tow)) {
            ++schedule.invalid_sample_count;
            schedule.failure = "non-finite IMU sample timestamp";
            return schedule;
        }
        // GNSSTime::operator<= and operator> intentionally use a 1 us
        // equality tolerance.  Phase201 uses a direct finite offset so a
        // sample just outside [t0,t1] cannot be admitted by that fuzzy
        // comparison.
        const double sample_offset_s = samples[i].time - t0;
        if (!std::isfinite(sample_offset_s)) {
            ++schedule.invalid_sample_count;
            schedule.failure = "non-finite IMU sample offset";
            return schedule;
        }
        if (sample_offset_s < 0.0 || sample_offset_s > interval_s) {
            if (sample_offset_s > interval_s) break;
            continue;
        }
        if (!samples[i].accel_raw.allFinite() ||
            !samples[i].gyro_raw_radps.allFinite()) {
            ++schedule.invalid_sample_count;
            schedule.failure = "non-finite selected IMU sample";
            return schedule;
        }

        double dt_s = std::numeric_limits<double>::quiet_NaN();
        if (i + 1 < samples.size()) {
            dt_s = samples[i + 1].time - samples[i].time;
        } else if (i > 0) {
            // The source repeats the last stream delta for the final sample.
            dt_s = samples[i].time - samples[i - 1].time;
        }
        if (!std::isfinite(dt_s)) {
            ++schedule.nonfinite_dt_count;
            schedule.failure = "non-finite selected IMU forward delta";
            return schedule;
        }
        if (!(dt_s > 0.0)) {
            ++schedule.nonpositive_dt_count;
            schedule.failure = "non-positive selected IMU forward delta";
            return schedule;
        }
        schedule.segments.push_back({i, dt_s});
        total_duration_s += dt_s;
        if (!std::isfinite(total_duration_s)) {
            ++schedule.nonfinite_dt_count;
            schedule.failure = "non-finite integrated IMU duration";
            schedule.segments.clear();
            return schedule;
        }
    }

    schedule.next_sample_index = i > 0 ? i - 1 : i;

    if (schedule.segments.empty()) {
        schedule.empty_interval = true;
        schedule.failure = "no IMU samples in inclusive GNSS interval";
        return schedule;
    }
    if (!(total_duration_s > 0.0)) {
        ++schedule.nonpositive_dt_count;
        schedule.failure = "non-positive integrated IMU duration";
        schedule.segments.clear();
        return schedule;
    }
    schedule.integrated_duration_s = total_duration_s;
    schedule.valid = true;
    return schedule;
}

inline std::string phase98SolverBranch(
    const gtsam::NonlinearOptimizerParams& params) {
    if (params.isMultifrontal()) return "multifrontal";
    if (params.isSequential()) return "sequential";
    if (params.isIterative()) return "iterative";
    if (params.isCholmod()) return "cholmod";
    return "invalid";
}

inline std::string phase98EliminationFunction(
    const gtsam::NonlinearOptimizerParams& params) {
    switch (params.linearSolverType) {
        case gtsam::NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY:
        case gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY:
            return "EliminatePreferCholesky";
        case gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR:
        case gtsam::NonlinearOptimizerParams::SEQUENTIAL_QR:
            return "EliminateQR";
        default:
            return "not-applicable";
    }
}

// Phase99 has one deliberately narrow mutation point: the existing
// Levenberg-Marquardt parameter object receives the already-supported
// multifrontal QR enum.  No ordering, damping, iteration, factor, or Values
// setting is touched here.  Keeping this helper tiny also gives the focused
// synthetic test an exact selector boundary to exercise.
inline void selectPhase99MainSolver(
    gtsam::LevenbergMarquardtParams& params, bool enabled) {
    if (enabled) {
        params.linearSolverType =
            gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR;
    }
}

// Hash only the effective ordering sequence.  The exact key list is
// deliberately not published: Phase98 requires a compact ordering witness,
// and this FNV-1a digest does not carry a state value or coordinate.
inline std::string phase98OrderingDigest(const gtsam::Ordering& ordering) {
    std::uint64_t digest = 1469598103934665603ULL;
    for (const gtsam::Key key : ordering) {
        const std::uint64_t value = static_cast<std::uint64_t>(key);
        for (unsigned int shift = 0; shift < 64U; shift += 8U) {
            digest ^= (value >> shift) & 0xffU;
            digest *= 1099511628211ULL;
        }
    }
    std::ostringstream output;
    output << "fnv1a64:" << std::hex << std::setw(16) << std::setfill('0')
           << digest;
    return output.str();
}

inline FGOProcessor::FGOPhase97KeyReference phase98KeyReference(
    gtsam::Key key) {
    const gtsam::Symbol symbol(key);
    FGOProcessor::FGOPhase97KeyReference reference;
    reference.numeric_key = static_cast<std::uint64_t>(key);
    reference.symbol_character = static_cast<char>(symbol.chr());
    reference.symbol_index = static_cast<std::uint64_t>(symbol.index());
    return reference;
}

inline void initializePhase98SolverDiagnostics(
    FGOProcessor::FGOPhase98SolverDiagnostics& diagnostics,
    const gtsam::LevenbergMarquardtParams& requested,
    const gtsam::LevenbergMarquardtParams& effective) {
    diagnostics.enabled = true;
    diagnostics.solver_type = effective.getLinearSolverType();
    diagnostics.solver_branch = phase98SolverBranch(effective);
    diagnostics.elimination_function = phase98EliminationFunction(effective);
    diagnostics.ordering_type = effective.getOrderingType();
    diagnostics.explicit_ordering_present = requested.ordering.has_value();
    if (effective.ordering) {
        diagnostics.ordering_size = effective.ordering->size();
        diagnostics.ordering_digest =
            phase98OrderingDigest(*effective.ordering);
    }
    diagnostics.diagonal_damping = effective.getDiagonalDamping();
}

inline FGOProcessor::FGOPhase98IndeterminateExceptionDiagnostics
phase98MakeIndeterminateExceptionDiagnostics(
    const gtsam::IndeterminantLinearSystemException& exception,
    const gtsam::LevenbergMarquardtParams& effective,
    bool explicit_ordering_present, double lambda,
    const std::string& stage) {
    FGOProcessor::FGOPhase98IndeterminateExceptionDiagnostics diagnostics;
    diagnostics.stage = stage;
    diagnostics.lambda = lambda;
    diagnostics.solver_type = effective.getLinearSolverType();
    diagnostics.solver_branch = phase98SolverBranch(effective);
    diagnostics.elimination_function = phase98EliminationFunction(effective);
    diagnostics.ordering_type = effective.getOrderingType();
    diagnostics.explicit_ordering_present = explicit_ordering_present;
    if (effective.ordering) {
        diagnostics.ordering_size = effective.ordering->size();
        diagnostics.ordering_digest =
            phase98OrderingDigest(*effective.ordering);
    }
    diagnostics.diagonal_damping = effective.getDiagonalDamping();
    diagnostics.nearby_variable_available = true;
    diagnostics.nearby_variable =
        phase98KeyReference(exception.nearbyVariable());
    diagnostics.nearby_variable_status = "captured_from_exception";
    diagnostics.exception_type =
        "gtsam::IndeterminantLinearSystemException";
    diagnostics.exception_message = exception.what();
    return diagnostics;
}

// Opt-in telemetry wrapper for the unchanged pinned GTSAM LM optimizer. The
// The optimizer's public lambda/outer-iteration accessors provide state
// counters, while SUMMARY output exposes every per-lambda trial and its
// linear-system success bit. The trace is needed because the pinned optimizer
// does not increment its public inner counter for the final small-cost-change
// trial. No optimizer step, damping policy, factor, or tolerance is changed;
// SUMMARY is enabled only while this diagnostic is requested.
struct GtsamLmActiveSolveTelemetry {
    bool active_solve_attempted = false;
    double initial_cost = 0.0;
    double final_cost = 0.0;
    // These counts describe outer LM attempts, not lambda trials.  An
    // accepted GTSAM iterate is one accepted outer attempt; a terminal
    // small-cost/max-lambda/exception branch contributes one rejected outer
    // attempt when the unchanged optimizer exposes it.
    std::size_t attempted_outer_iterations = 0;
    std::size_t accepted_outer_iterations = 0;
    std::size_t rejected_outer_iterations = 0;
    std::size_t total_inner_lambda_attempts = 0;
    std::size_t parsed_trial_count = 0;
    std::size_t native_inner_iterations = 0;
    std::size_t expected_trial_count = 0;
    std::size_t indeterminate_linear_solve_count = 0;
    std::size_t unsuccessful_model_step_count = 0;
    std::size_t small_cost_change_stop_count = 0;
    std::size_t maximum_lambda_stop_count = 0;
    double initial_lambda = 0.0;
    double maximum_lambda = 0.0;
    double final_lambda = 0.0;
    bool finite_costs = false;
    bool termination_trace_complete = false;
    std::string termination_branch_reason;
    std::vector<FGOProcessor::FGOPhase96LmTrialDiagnostics>
        phase96_lm_trials;
    std::vector<FGOProcessor::FGOPhase96ExceptionDiagnostics>
        phase96_exceptions;
    bool phase96_trace_complete = false;
    FGOProcessor::FGOPhase98SolverDiagnostics phase98;
};

// Phase143 has one algorithmic mutation point: the main meter-state call's
// maxIterations field.  Keeping this helper independent of graph construction
// makes the selector boundary directly testable without raw data or a solve.
inline int phase143EffectiveMaxIterations(int configured_max_iterations,
                                           bool selector_enabled,
                                           bool main_meter_state_scope) {
    if (selector_enabled && main_meter_state_scope) return 1000;
    return std::max(1, configured_max_iterations);
}

inline bool phase143TerminationBranchAllowed(const std::string& branch) {
    return branch == "maximum_outer_iterations" ||
           branch == "outer_convergence_tolerance" ||
           branch == "small_cost_change" || branch == "maximum_lambda" ||
           branch == "no_inner_iteration" || branch == "exception" ||
           branch == "no_progress_unclassified";
}

// Validate the complete scalar contract emitted by the native LM boundary.
// This helper intentionally rejects missing, renamed, inferred, conflicting,
// or non-finite fields when the selector is active.  It never looks at a
// Values object or reconstructs a counter from FGOResult::diagnostics.iterations.
inline bool validatePhase143TerminationDiagnostics(
    const FGOProcessor::FGOPhase143TerminationDiagnostics& diagnostics,
    std::string* failure = nullptr) {
    const auto fail = [&](const std::string& message) {
        if (failure) *failure = message;
        return false;
    };
    if (!diagnostics.selector_enabled) return true;
    if (diagnostics.stage != "main" && diagnostics.stage != "gnss-first") {
        return fail("stage must be main or gnss-first");
    }
    if (diagnostics.configured_max_iterations == 0U ||
        diagnostics.effective_max_iterations == 0U) {
        return fail("configured/effective maxIterations is missing");
    }
    if (diagnostics.effective_max_iterations != 1000U) {
        return fail("Phase143 effective maxIterations is not exactly 1000");
    }
    if (diagnostics.stage == "main" &&
        diagnostics.configured_max_iterations != 12U &&
        diagnostics.configured_max_iterations != 1000U) {
        return fail("main configured maxIterations is outside the frozen 12/1000 boundary");
    }
    if (diagnostics.stage == "gnss-first" &&
        diagnostics.configured_max_iterations != 1000U) {
        return fail("GNSS-first maxIterations changed from the frozen 1000");
    }
    if (!diagnostics.attempted) return fail("optimizer attempt marker is missing");
    if (diagnostics.accepted_outer_iterations >
        diagnostics.attempted_outer_iterations) {
        return fail("accepted outer iterations exceed attempted outer iterations");
    }
    if (diagnostics.rejected_outer_iterations !=
        diagnostics.attempted_outer_iterations -
            diagnostics.accepted_outer_iterations) {
        return fail("attempted/accepted/rejected outer iteration counts disagree");
    }
    if (diagnostics.accepted_outer_iterations >
        diagnostics.effective_max_iterations) {
        return fail("accepted outer iterations exceed effective maxIterations");
    }
    if (diagnostics.total_inner_lambda_attempts <
        diagnostics.accepted_outer_iterations) {
        return fail("lambda trial count is below accepted outer iterations");
    }
    if (!std::isfinite(diagnostics.initial_cost) ||
        !std::isfinite(diagnostics.final_cost) ||
        !std::isfinite(diagnostics.relative_error_tolerance) ||
        !std::isfinite(diagnostics.absolute_error_tolerance) ||
        !std::isfinite(diagnostics.error_tolerance) ||
        !std::isfinite(diagnostics.initial_lambda) ||
        !std::isfinite(diagnostics.final_lambda) ||
        !std::isfinite(diagnostics.maximum_lambda) ||
        !std::isfinite(diagnostics.lambda_factor) ||
        !std::isfinite(diagnostics.lambda_lower_bound) ||
        !std::isfinite(diagnostics.lambda_upper_bound) ||
        !std::isfinite(diagnostics.min_model_fidelity)) {
        return fail("termination telemetry contains a non-finite scalar");
    }
    if (!diagnostics.costs_finite ||
        diagnostics.strict_cost_decrease !=
            (diagnostics.final_cost < diagnostics.initial_cost)) {
        return fail("cost finiteness/strict-progress marker disagrees with native costs");
    }
    if (!phase143TerminationBranchAllowed(diagnostics.termination_branch)) {
        return fail("termination branch is missing or outside the frozen enum");
    }
    if (diagnostics.termination_branch == "maximum_outer_iterations" &&
        diagnostics.accepted_outer_iterations !=
            diagnostics.effective_max_iterations) {
        return fail("maximum-iteration branch does not reach the effective cap");
    }
    if (diagnostics.termination_branch == "no_inner_iteration" &&
        (diagnostics.accepted_outer_iterations != 0U ||
         diagnostics.attempted_outer_iterations != 0U ||
         diagnostics.rejected_outer_iterations != 0U)) {
        return fail("no-inner-iteration branch has nonzero outer counts");
    }
    if (diagnostics.linear_solver.empty() || diagnostics.elimination.empty() ||
        diagnostics.ordering_type.empty()) {
        return fail("solver/elimination/ordering telemetry is missing");
    }
    if (!diagnostics.no_fallback) return fail("Phase143 solve used fallback");
    if (!diagnostics.termination_trace_complete) {
        return fail("native termination trace is incomplete");
    }
    return true;
}

inline FGOProcessor::FGOPhase143TerminationDiagnostics
makePhase143TerminationDiagnostics(
    const gtsam::LevenbergMarquardtParams& params,
    const GtsamLmActiveSolveTelemetry& telemetry,
    const std::string& stage, std::size_t configured_max_iterations,
    bool selector_enabled, bool no_fallback) {
    FGOProcessor::FGOPhase143TerminationDiagnostics diagnostics;
    diagnostics.selector_enabled = selector_enabled;
    diagnostics.stage = stage;
    diagnostics.configured_max_iterations = configured_max_iterations;
    diagnostics.effective_max_iterations = params.getMaxIterations();
    diagnostics.attempted = telemetry.active_solve_attempted;
    diagnostics.attempted_outer_iterations =
        telemetry.attempted_outer_iterations;
    diagnostics.accepted_outer_iterations =
        telemetry.accepted_outer_iterations;
    diagnostics.rejected_outer_iterations = telemetry.rejected_outer_iterations;
    diagnostics.total_inner_lambda_attempts =
        telemetry.total_inner_lambda_attempts;
    diagnostics.parsed_trial_count = telemetry.parsed_trial_count;
    diagnostics.native_inner_iterations = telemetry.native_inner_iterations;
    diagnostics.expected_trial_count = telemetry.expected_trial_count;
    diagnostics.initial_cost = telemetry.initial_cost;
    diagnostics.final_cost = telemetry.final_cost;
    diagnostics.costs_finite = telemetry.finite_costs;
    diagnostics.strict_cost_decrease =
        std::isfinite(diagnostics.initial_cost) &&
        std::isfinite(diagnostics.final_cost) &&
        diagnostics.final_cost < diagnostics.initial_cost;
    diagnostics.termination_branch = telemetry.termination_branch_reason;
    diagnostics.relative_error_tolerance = params.getRelativeErrorTol();
    diagnostics.absolute_error_tolerance = params.getAbsoluteErrorTol();
    diagnostics.error_tolerance = params.getErrorTol();
    diagnostics.initial_lambda = telemetry.initial_lambda;
    diagnostics.final_lambda = telemetry.final_lambda;
    diagnostics.maximum_lambda = telemetry.maximum_lambda;
    diagnostics.lambda_factor = params.getlambdaFactor();
    diagnostics.lambda_lower_bound = params.getlambdaLowerBound();
    diagnostics.lambda_upper_bound = params.getlambdaUpperBound();
    diagnostics.min_model_fidelity = params.minModelFidelity;
    diagnostics.diagonal_damping = params.getDiagonalDamping();
    diagnostics.use_fixed_lambda_factor = params.useFixedLambdaFactor;
    diagnostics.linear_solver = params.getLinearSolverType();
    diagnostics.elimination = phase98EliminationFunction(params);
    diagnostics.ordering_type = params.getOrderingType();
    diagnostics.explicit_ordering_present = params.ordering.has_value();
    diagnostics.no_fallback = no_fallback;
    diagnostics.termination_trace_complete =
        telemetry.termination_trace_complete;
    std::string failure;
    diagnostics.configuration_valid =
        validatePhase143TerminationDiagnostics(diagnostics, &failure);
    diagnostics.configuration_failure = failure;
    return diagnostics;
}

class GtsamLmActiveSolveOptimizer {
    struct Attempt {
        std::size_t outer_iteration = 0;
        double new_error = 0.0;
        double cost_change = 0.0;
        double lambda = 0.0;
        bool system_solved_successfully = false;
    };

    struct PendingPhase96Trial {
        FGOProcessor::FGOPhase96LmTrialDiagnostics trial;
        double old_nonlinear_cost = std::numeric_limits<double>::quiet_NaN();
        bool saw_delta = false;
        bool saw_linearized = false;
        bool saw_candidate = false;
        bool saw_increasing_lambda = false;
        bool saw_small_cost_stop = false;
        bool saw_maximum_lambda_stop = false;
    };

    static bool parseValueAfter(const std::string& line,
                                const std::string& marker,
                                double& value) {
        const std::size_t offset = line.find(marker);
        if (offset == std::string::npos) return false;
        const std::size_t begin = offset + marker.size();
        try {
            std::size_t consumed = 0;
            value = std::stod(line.substr(begin), &consumed);
            return consumed > 0;
        } catch (...) {
            value = std::numeric_limits<double>::quiet_NaN();
            return false;
        }
    }

    static void closePhase96Trial(
        PendingPhase96Trial& pending,
        std::vector<FGOProcessor::FGOPhase96LmTrialDiagnostics>& trials,
        std::size_t& outer_iteration) {
        auto& trial = pending.trial;
        trial.linear_system_solved = pending.saw_delta;
        trial.linear_system_status = pending.saw_delta ? "solved"
                                                       : "indeterminate_linear_system";
        if (pending.saw_linearized && std::isfinite(trial.new_linearized_cost) &&
            std::isfinite(trial.predicted_reduction)) {
            trial.old_linearized_cost =
                trial.new_linearized_cost + trial.predicted_reduction;
        }
        trial.candidate_finite =
            pending.saw_candidate && std::isfinite(trial.candidate_nonlinear_cost);
        if (std::isfinite(pending.old_nonlinear_cost) &&
            std::isfinite(trial.candidate_nonlinear_cost)) {
            trial.actual_reduction = pending.old_nonlinear_cost -
                                     trial.candidate_nonlinear_cost;
        }
        if (!pending.saw_delta) {
            trial.rejection_reason = "indeterminate_linear_system";
        } else if (!pending.saw_linearized ||
                   !std::isfinite(trial.new_linearized_cost) ||
                   !std::isfinite(trial.predicted_reduction)) {
            trial.rejection_reason = "nonfinite_linearized_error";
        } else if (trial.predicted_reduction < 0.0) {
            trial.rejection_reason = "negative_predicted_reduction";
        } else if (!pending.saw_candidate || !trial.candidate_finite) {
            trial.rejection_reason = "nonfinite_candidate_cost";
        } else if (pending.saw_maximum_lambda_stop) {
            trial.rejection_reason = "maximum_lambda_stop";
        } else if (pending.saw_increasing_lambda) {
            const double actual = trial.actual_reduction;
            if (!std::isfinite(actual) || actual <= 0.0) {
                trial.rejection_reason = "non_decreasing_nonlinear_cost";
            } else if (std::isfinite(trial.model_fidelity)) {
                trial.rejection_reason = "model_fidelity_below_threshold";
            } else {
                trial.rejection_reason = "lambda_retry";
            }
        } else if (pending.saw_small_cost_stop) {
            trial.rejection_reason = "small_cost_change_stop";
        } else {
            // tryLambda returns immediately after an accepted step.  There is
            // no separate success line in the pinned source, so the absence
            // of its retry/stop marker is the exact observable success path.
            trial.rejection_reason = "accepted_outer_step";
            ++outer_iteration;
        }
        trials.push_back(trial);
    }

    static void appendPhase96Exception(
        std::vector<FGOProcessor::FGOPhase96ExceptionDiagnostics>& exceptions,
        const std::string& stage, const std::string& classification,
        const std::string& type, const std::string& message) {
        for (auto& existing : exceptions) {
            if (existing.stage == stage &&
                existing.classification == classification &&
                existing.type == type && existing.message == message) {
                ++existing.count;
                return;
            }
        }
        exceptions.push_back({stage, classification, type, message, 1});
    }

    static std::vector<FGOProcessor::FGOPhase96LmTrialDiagnostics>
    parsePhase96Trace(const std::string& text,
                      bool limit_to_first_ten = true) {
        std::vector<FGOProcessor::FGOPhase96LmTrialDiagnostics> trials;
        std::istringstream lines(text);
        std::string line;
        PendingPhase96Trial pending;
        bool have_pending = false;
        std::size_t outer_iteration = 0;
        while (std::getline(lines, line)) {
            const std::size_t first = line.find_first_not_of(" \t");
            const std::string trimmed =
                first == std::string::npos ? std::string() : line.substr(first);
            if (trimmed.rfind("trying lambda = ", 0) == 0) {
                if (have_pending) {
                    closePhase96Trial(pending, trials, outer_iteration);
                }
                pending = PendingPhase96Trial();
                have_pending = true;
                pending.trial.trial_index = trials.size();
                pending.trial.outer_iteration = outer_iteration;
                parseValueAfter(trimmed, "trying lambda = ",
                                pending.trial.lambda);
                continue;
            }
            if (!have_pending) continue;
            if (trimmed.rfind("linear delta norm = ", 0) == 0) {
                pending.saw_delta = true;
                pending.trial.linear_system_status = "solved";
                continue;
            }
            // GTSAM 4.3 spells the field with a capital L; retain the
            // lowercase spelling accepted by the historical synthetic
            // parser/tests for compatibility with older pinned traces.
            const std::string linearized_error_marker =
                trimmed.rfind("newLinearizedError = ", 0) == 0
                    ? "newLinearizedError = "
                    : "newlinearizedError = ";
            if (trimmed.rfind(linearized_error_marker, 0) == 0) {
                pending.saw_linearized = true;
                parseValueAfter(trimmed, linearized_error_marker,
                                pending.trial.new_linearized_cost);
                parseValueAfter(trimmed, "linearizedCostChange = ",
                                pending.trial.predicted_reduction);
                continue;
            }
            if (trimmed.rfind("old error (", 0) == 0) {
                const std::string marker = "old error (";
                const std::size_t begin = trimmed.find(marker) + marker.size();
                const std::size_t end = trimmed.find(")", begin);
                const std::string new_marker = ") new (tentative) error (";
                const std::size_t new_begin = trimmed.find(new_marker, end);
                if (end != std::string::npos && new_begin != std::string::npos) {
                    try {
                        pending.old_nonlinear_cost = std::stod(
                            trimmed.substr(begin, end - begin));
                        const std::size_t value_begin =
                            new_begin + new_marker.size();
                        const std::size_t value_end =
                            trimmed.find(")", value_begin);
                        pending.trial.candidate_nonlinear_cost = std::stod(
                            trimmed.substr(value_begin, value_end - value_begin));
                        pending.saw_candidate = true;
                    } catch (...) {
                        pending.trial.candidate_nonlinear_cost =
                            std::numeric_limits<double>::quiet_NaN();
                    }
                }
                continue;
            }
            if (trimmed.rfind("modelFidelity: ", 0) == 0) {
                parseValueAfter(trimmed, "modelFidelity: ",
                                pending.trial.model_fidelity);
                continue;
            }
            if (trimmed == "increasing lambda") {
                pending.saw_increasing_lambda = true;
                continue;
            }
            if (trimmed.find("stopping as relative cost reduction is small") !=
                std::string::npos) {
                pending.saw_small_cost_stop = true;
                continue;
            }
            if (trimmed.find("giving up because cannot decrease error with maximum lambda") !=
                std::string::npos) {
                pending.saw_maximum_lambda_stop = true;
                continue;
            }
        }
        if (have_pending) closePhase96Trial(pending, trials, outer_iteration);
        // The Phase96 freeze exposes only the first ten inner trials.  Keep
        // the parser bounded even if a future route drives the unchanged
        // optimizer through a longer lambda search.
        if (limit_to_first_ten && trials.size() > 10U) trials.resize(10U);
        return trials;
    }

    static gtsam::LevenbergMarquardtParams diagnosticParams(
        const gtsam::LevenbergMarquardtParams& params, bool enabled,
        bool phase96_enabled) {
        gtsam::LevenbergMarquardtParams result = params;
        if (enabled) {
            // The trace is the pinned optimizer's existing diagnostic surface.
            // This changes verbosity only, never solve parameters.
            result.verbosityLM =
                phase96_enabled ? gtsam::LevenbergMarquardtParams::TRYLAMBDA
                                 : gtsam::LevenbergMarquardtParams::SUMMARY;
        }
        return result;
    }

    // std::istream::operator>>(double) rejects the lowercase `inf` token
    // emitted by the pinned GTSAM SUMMARY surface on an indeterminate trial.
    // Parse the numeric token explicitly so that a nonfinite trial remains a
    // counted diagnostic row and cannot be silently dropped or treated as a
    // successful row; the existing aggregate/termination contract remains
    // authoritative and is not weakened here.
    static bool parseDiagnosticDouble(std::istringstream& row,
                                      double& value) {
        std::string token;
        if (!(row >> token)) return false;
        char* end = nullptr;
        value = std::strtod(token.c_str(), &end);
        return end != token.c_str() && end != nullptr && *end == '\0';
    }

    static std::vector<Attempt> parseAttempts(const std::string& text) {
        std::vector<Attempt> attempts;
        std::istringstream lines(text);
        std::string line;
        while (std::getline(lines, line)) {
            std::istringstream row(line);
            Attempt attempt;
            int solved = 0;
            double elapsed = 0.0;
            if (row >> attempt.outer_iteration &&
                parseDiagnosticDouble(row, attempt.new_error) &&
                parseDiagnosticDouble(row, attempt.cost_change) &&
                parseDiagnosticDouble(row, attempt.lambda) && row >> solved &&
                (solved == 0 || solved == 1) &&
                parseDiagnosticDouble(row, elapsed)) {
                attempt.system_solved_successfully = solved != 0;
                attempts.push_back(attempt);
            }
        }
        return attempts;
    }

    void finalize(const std::string& trace) {
        telemetry_.accepted_outer_iterations = optimizer_.iterations();
        const std::size_t optimizer_inner_iterations = static_cast<std::size_t>(
            std::max(0, optimizer_.getInnerIterations()));
        telemetry_.native_inner_iterations = optimizer_inner_iterations;
        telemetry_.final_lambda = optimizer_.lambda();
        telemetry_.maximum_lambda = std::max(
            telemetry_.initial_lambda, telemetry_.final_lambda);

        const std::vector<Attempt> attempts = parseAttempts(trace);
        std::vector<FGOProcessor::FGOPhase96LmTrialDiagnostics>
            all_phase96_lm_trials;
        std::size_t phase96_attempts = 0;
        if (phase96_enabled_) {
            all_phase96_lm_trials = parsePhase96Trace(trace, false);
            phase96_attempts = all_phase96_lm_trials.size();
            telemetry_.phase96_lm_trials = all_phase96_lm_trials;
            telemetry_.phase96_trace_complete =
                !all_phase96_lm_trials.empty();
            if (all_phase96_lm_trials.empty() && optimizer_inner_iterations > 0U) {
                appendPhase96Exception(
                    telemetry_.phase96_exceptions, "lm_trace",
                    "trace_unavailable", "TRYLAMBDA output",
                    "no pinned LM trial records were captured");
            }
            if (all_phase96_lm_trials.size() > 10U) {
                telemetry_.phase96_lm_trials.resize(10U);
            }
            if (all_phase96_lm_trials.size() <= 10U &&
                !telemetry_.phase96_lm_trials.empty() &&
                optimizer_.lambda() >= optimizer_.params().lambdaUpperBound &&
                telemetry_.phase96_lm_trials.back().rejection_reason !=
                    "accepted_outer_step") {
                telemetry_.phase96_lm_trials.back().rejection_reason =
                    "maximum_lambda_stop";
            }
        }
        // Count actual lambda trials from the trace. GTSAM's public inner
        // counter omits a terminal tryLambda() that stops on tiny cost change.
        telemetry_.total_inner_lambda_attempts =
            phase96_attempts > 0
                ? phase96_attempts
                : (attempts.empty() ? optimizer_inner_iterations : attempts.size());
        telemetry_.termination_trace_complete = phase96_enabled_
                                                    ? telemetry_.phase96_trace_complete
                                                    : (!attempts.empty() ||
                                                       optimizer_inner_iterations == 0);
        double failed_lambda_min = std::numeric_limits<double>::infinity();
        double failed_lambda_max = 0.0;
        std::size_t failed_lambda_count = 0;
        for (const Attempt& attempt : attempts) {
            telemetry_.maximum_lambda =
                std::max(telemetry_.maximum_lambda, attempt.lambda);
            if (!attempt.system_solved_successfully) {
                ++telemetry_.indeterminate_linear_solve_count;
                if (!phase96_enabled_ && std::isfinite(attempt.lambda) && attempt.lambda >= 0.0) {
                    failed_lambda_min = std::min(failed_lambda_min, attempt.lambda);
                    failed_lambda_max = std::max(failed_lambda_max, attempt.lambda);
                    ++failed_lambda_count;
                }
            }
        }
        if (phase96_enabled_) {
            telemetry_.indeterminate_linear_solve_count = 0;
            for (const auto& trial : all_phase96_lm_trials) {
                if (!trial.linear_system_solved) {
                    ++telemetry_.indeterminate_linear_solve_count;
                    if (std::isfinite(trial.lambda) && trial.lambda >= 0.0) {
                        failed_lambda_min = std::min(failed_lambda_min, trial.lambda);
                        failed_lambda_max = std::max(failed_lambda_max, trial.lambda);
                        ++failed_lambda_count;
                    }
                    appendPhase96Exception(
                        telemetry_.phase96_exceptions, "lm_trial",
                        "indeterminate_linear_system", "IndeterminantLinearSystemException",
                        "pinned LM trial did not solve the damped linear system");
                }
                if (trial.rejection_reason == "nonfinite_linearized_error" ||
                    trial.rejection_reason == "nonfinite_candidate_cost") {
                    appendPhase96Exception(
                        telemetry_.phase96_exceptions, "lm_trial", "nonfinite",
                        "LM trial error", "candidate or linearized error is nonfinite");
                }
                if (trial.lambda > telemetry_.maximum_lambda) {
                    telemetry_.maximum_lambda = trial.lambda;
                }
            }
        }
        if (failed_lambda_count > 0) {
            std::fprintf(stderr,
                "[native-lm-failed-lambda] count=%zu min=%.17g max=%.17g nearby_key=unavailable\n",
                failed_lambda_count, failed_lambda_min, failed_lambda_max);
        }
        const std::size_t accounted_attempts =
            phase96_enabled_
                ? phase96_attempts
                : (telemetry_.termination_trace_complete
                       ? telemetry_.total_inner_lambda_attempts
                       : std::min(telemetry_.total_inner_lambda_attempts,
                                  attempts.size()));
        const std::size_t accepted = telemetry_.accepted_outer_iterations;
        const std::size_t indeterminate =
            telemetry_.indeterminate_linear_solve_count;
        if (accounted_attempts >= accepted + indeterminate) {
            telemetry_.unsuccessful_model_step_count =
                accounted_attempts - accepted - indeterminate;
        }

        const bool has_terminal_retry = phase96_enabled_
            ? (!all_phase96_lm_trials.empty() &&
               all_phase96_lm_trials.back().outer_iteration >= accepted)
            : (!attempts.empty() && attempts.back().outer_iteration >= accepted);
        const double lambda_upper = optimizer_.params().lambdaUpperBound;
        if (has_terminal_retry && telemetry_.final_lambda >= lambda_upper) {
            telemetry_.maximum_lambda_stop_count = 1;
            telemetry_.termination_branch_reason = "maximum_lambda";
        } else if (has_terminal_retry) {
            telemetry_.small_cost_change_stop_count = 1;
            telemetry_.termination_branch_reason = "small_cost_change";
        } else if (telemetry_.total_inner_lambda_attempts == 0) {
            telemetry_.termination_branch_reason = "no_inner_iteration";
        } else if (accepted >= optimizer_.params().maxIterations) {
            telemetry_.termination_branch_reason = "maximum_outer_iterations";
        } else if (accepted > 0) {
            telemetry_.termination_branch_reason =
                "outer_convergence_tolerance";
        } else {
            telemetry_.termination_branch_reason = "no_progress_unclassified";
        }
        // The pinned GTSAM trace labels each lambda trial with the current
        // outer-iteration index.  Count distinct indices instead of deriving
        // attempted/rejected counts from the generic result iteration field or
        // from a return code.  Retries therefore remain one outer attempt,
        // while a terminal rejected lambda gets its own trace index.
        std::set<std::size_t> outer_attempts;
        if (phase96_enabled_) {
            for (const auto& trial : all_phase96_lm_trials) {
                outer_attempts.insert(trial.outer_iteration);
            }
        } else {
            for (const Attempt& attempt : attempts) {
                outer_attempts.insert(attempt.outer_iteration);
            }
        }
        telemetry_.attempted_outer_iterations = outer_attempts.size();
        telemetry_.rejected_outer_iterations =
            telemetry_.attempted_outer_iterations >= accepted
                ? telemetry_.attempted_outer_iterations - accepted
                : 0U;
        if (phase143_enabled_) {
            // Every lambda trial must be represented by the selected native
            // verbosity surface.  GTSAM increments its public inner counter
            // for every trial except the terminal small-cost trial; this is a
            // strict completeness check, not a reconstructed solve count.
            const std::size_t trace_trial_count =
                phase96_enabled_ ? phase96_attempts : attempts.size();
            const std::size_t expected_trial_count =
                optimizer_inner_iterations +
                (telemetry_.termination_branch_reason == "small_cost_change"
                     ? 1U
                     : 0U);
            telemetry_.parsed_trial_count = trace_trial_count;
            telemetry_.expected_trial_count = expected_trial_count;
            telemetry_.termination_trace_complete =
                (trace_trial_count == expected_trial_count) &&
                (trace_trial_count == 0U || !outer_attempts.empty());
        }
        telemetry_.finite_costs = std::isfinite(telemetry_.initial_cost) &&
                                  std::isfinite(telemetry_.final_cost);
    }

    gtsam::LevenbergMarquardtOptimizer optimizer_;
    bool enabled_ = false;
    bool phase96_enabled_ = false;
    bool phase98_enabled_ = false;
    bool phase143_enabled_ = false;
    bool phase98_explicit_ordering_present_ = false;
    GtsamLmActiveSolveTelemetry telemetry_;

 public:
    GtsamLmActiveSolveOptimizer(
        const gtsam::NonlinearFactorGraph& graph,
        const gtsam::Values& initial,
        const gtsam::LevenbergMarquardtParams& params,
        bool enabled, bool phase96_enabled = false,
        bool phase98_enabled = false, bool phase143_enabled = false)
        : optimizer_(graph, initial,
                     diagnosticParams(params,
                                      enabled || phase96_enabled || phase98_enabled ||
                                          phase143_enabled,
                                      phase96_enabled || phase98_enabled)),
          enabled_(enabled || phase96_enabled || phase98_enabled ||
                   phase143_enabled),
          phase96_enabled_(phase96_enabled || phase98_enabled),
          phase98_enabled_(phase98_enabled),
          phase143_enabled_(phase143_enabled),
          phase98_explicit_ordering_present_(params.ordering.has_value()) {
        telemetry_.initial_cost = optimizer_.error();
        telemetry_.initial_lambda = optimizer_.lambda();
        telemetry_.maximum_lambda = telemetry_.initial_lambda;
        if (phase98_enabled_) {
            initializePhase98SolverDiagnostics(
                telemetry_.phase98, params, optimizer_.params());
        }
    }

    gtsam::Values optimize() {
        telemetry_.active_solve_attempted = true;
        if (phase98_enabled_) telemetry_.phase98.attempted = true;
        std::ostringstream trace;
        std::streambuf* previous = nullptr;
        if (enabled_) previous = std::cout.rdbuf(trace.rdbuf());
        try {
            gtsam::Values optimized = optimizer_.optimize();
            if (enabled_) std::cout.rdbuf(previous);
            telemetry_.final_cost = optimizer_.error();
            finalize(trace.str());
            return optimized;
        } catch (const gtsam::IndeterminantLinearSystemException& error) {
            if (enabled_) std::cout.rdbuf(previous);
            telemetry_.final_cost = optimizer_.error();
            finalize(trace.str());
            if (phase98_enabled_) {
                telemetry_.phase98.exception_captured = true;
                telemetry_.phase98.indeterminate_exceptions.push_back(
                    phase98MakeIndeterminateExceptionDiagnostics(
                        error, optimizer_.params(),
                        phase98_explicit_ordering_present_, optimizer_.lambda(),
                        "lm_optimize"));
            }
            telemetry_.termination_branch_reason = "exception";
            telemetry_.attempted_outer_iterations =
                telemetry_.accepted_outer_iterations + 1U;
            telemetry_.rejected_outer_iterations = 1U;
            throw;
        } catch (const std::exception& error) {
            if (enabled_) std::cout.rdbuf(previous);
            telemetry_.final_cost = optimizer_.error();
            finalize(trace.str());
            if (phase96_enabled_) {
                appendPhase96Exception(telemetry_.phase96_exceptions,
                                       "lm_optimize", "exception",
                                       typeid(error).name(), error.what());
            }
            telemetry_.termination_branch_reason = "exception";
            telemetry_.attempted_outer_iterations =
                telemetry_.accepted_outer_iterations + 1U;
            telemetry_.rejected_outer_iterations = 1U;
            throw;
        } catch (...) {
            if (enabled_) std::cout.rdbuf(previous);
            telemetry_.final_cost = optimizer_.error();
            finalize(trace.str());
            if (phase96_enabled_) {
                appendPhase96Exception(telemetry_.phase96_exceptions,
                                       "lm_optimize", "exception", "unknown",
                                       "unknown exception");
            }
            telemetry_.termination_branch_reason = "exception";
            telemetry_.attempted_outer_iterations =
                telemetry_.accepted_outer_iterations + 1U;
            telemetry_.rejected_outer_iterations = 1U;
            throw;
        }
    }

    const GtsamLmActiveSolveTelemetry& telemetry() const { return telemetry_; }
    std::size_t iterations() const { return optimizer_.iterations(); }

    static std::vector<FGOProcessor::FGOPhase96LmTrialDiagnostics>
    parsePhase96TraceForTesting(const std::string& text) {
        return parsePhase96Trace(text);
    }

    static std::size_t parseAttemptsCountForTesting(const std::string& text) {
        return parseAttempts(text).size();
    }

    static std::size_t parseLinearFailuresForTesting(const std::string& text) {
        const auto attempts = parseAttempts(text);
        return std::count_if(attempts.begin(), attempts.end(),
            [](const Attempt& attempt) { return !attempt.system_solved_successfully; });
    }
};

// MF-AR step 1: restrict a per-epoch LAMBDA candidate set to independent
// satellites -- keep at most one ambiguity per satellite (primary band
// preferred, else the longest-wavelength secondary; ties broken by index for
// determinism). Multiple frequency bands of one satellite share the same
// line-of-sight, so they contribute little to the DD geometry but do inflate
// the all-or-nothing integer ratio test and lower the ratio. The dropped
// bands' DD factors remain in the graph and still constrain the float; they
// are simply not forced into the integer search. No-op for single-frequency
// DD (each satellite already has a single band). Input indices are assumed
// unique (one DD carrier factor per (sat,signal) per epoch).
inline std::vector<std::size_t> selectOneBandPerSatellite(
    const std::vector<std::size_t>& indices,
    const FGOProcessor::FGOProblem& problem) {
    std::map<SatelliteId, std::size_t> best_by_satellite;
    for (std::size_t idx : indices) {
        if (idx >= problem.ambiguity_states.size()) {
            continue;
        }
        const auto& amb = problem.ambiguity_states[idx];
        const auto it = best_by_satellite.find(amb.satellite);
        if (it == best_by_satellite.end()) {
            best_by_satellite.emplace(amb.satellite, idx);
            continue;
        }
        const auto& cur = problem.ambiguity_states[it->second];
        const bool amb_primary =
            signal_policy::isPrimarySignal(amb.satellite.system, amb.signal);
        const bool cur_primary =
            signal_policy::isPrimarySignal(cur.satellite.system, cur.signal);
        bool replace = false;
        if (amb_primary != cur_primary) {
            replace = amb_primary;  // prefer the primary band
        } else if (amb.wavelength_m != cur.wavelength_m) {
            replace = amb.wavelength_m > cur.wavelength_m;  // longer wavelength: easier to fix
        } else {
            replace = idx < it->second;  // deterministic
        }
        if (replace) {
            it->second = idx;
        }
    }
    std::vector<std::size_t> selected;
    selected.reserve(best_by_satellite.size());
    for (const auto& [sat, idx] : best_by_satellite) {
        (void)sat;
        selected.push_back(idx);
    }
    return selected;
}

// --- Surplus-satellite independent integrity validation (see FGOConfig::
// use_surplus_satellite_validation) ---
//
// Constellation fallback ladder for the surplus pool: NOT the same ladder as
// use_constellation_ranked_partial_ar above (which is GQEBR/GQEB/GQER/GQB/
// GQR/GQ) -- this one is specified directly by the surplus-validation design
// (docs: PPC paper / MDPI sensors 24-9-2712 reliability-check port):
//   0: GQEBR (all)  1: GQEB  2: GQER  3: GQE  4: GQB  5: GQ
// G=GPS, Q=QZSS, E=Galileo, B=BeiDou, R=GLONASS. GPS/QZSS are always kept
// (never the least-reliable constellation being dropped).
inline bool surplusSystemAllowedAtLevel(GNSSSystem system, int level) {
    switch (system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            return true;
        case GNSSSystem::Galileo:
            return level >= 0 && level <= 3;  // GQEBR, GQEB, GQER, GQE
        case GNSSSystem::BeiDou:
            return level == 0 || level == 1 || level == 4;  // GQEBR, GQEB, GQB
        case GNSSSystem::GLONASS:
            return level == 0 || level == 2;  // GQEBR, GQER
        default:
            return level == 0;
    }
}

// PDOP (position-only, no clock term -- DD observations are already
// clock-differenced) computed from the CURRENT FIXED-set geometry at the
// candidate fixed antenna position. Used to select the surplus-validation
// nearest-integer aperture. Mirrors the epoch's shared GDOP computation
// above (nsat/gdop locals) but restricted to the fixed subset and without
// the clock column, per FGOConfig::surplus_validation_aperture_* docs.
inline double computeFixedSetPdop(
    const std::vector<const FGOProcessor::DoubleDifferenceCarrierFactor*>& epoch_cp_factors,
    const std::map<std::size_t, int>& fixed_cycles_by_index,
    const Eigen::Vector3d& candidate_ant) {
    std::map<SatelliteId, Eigen::Vector3d> sat_pos;
    for (const auto* fp : epoch_cp_factors) {
        if (fixed_cycles_by_index.count(fp->ambiguity_index) == 0) continue;
        sat_pos.emplace(fp->satellite, Eigen::Vector3d(fp->rover_satellite_position_ecef));
        sat_pos.emplace(fp->reference_satellite,
                        Eigen::Vector3d(fp->rover_reference_position_ecef));
    }
    if (sat_pos.size() < 4) return std::numeric_limits<double>::infinity();
    Eigen::MatrixXd H(static_cast<int>(sat_pos.size()), 3);
    int row = 0;
    for (const auto& [sid, sp] : sat_pos) {
        (void)sid;
        const Eigen::Vector3d d = sp - candidate_ant;
        const double rng = d.norm();
        H.row(row) = (rng > 0.0) ? Eigen::RowVector3d(-d / rng) : Eigen::RowVector3d::Zero();
        ++row;
    }
    const Eigen::Matrix3d Ninv = (H.transpose() * H).inverse();
    if (!Ninv.allFinite()) return std::numeric_limits<double>::infinity();
    return std::sqrt(std::max(0.0, Ninv.trace()));
}

struct SurplusValidationOutcome {
    bool evaluated = false;  ///< false: too few surplus sats at every fallback level (no verdict)
    bool pass = false;
    int fallback_level = -1;  ///< 0=GQEBR .. 5=GQ; the level that rendered the verdict
    int surplus_used = 0;     ///< surplus satellites in the deciding pool
    int surplus_available_full = 0;  ///< diagnostic only: total surplus sats before any fallback filtering
};

// Evaluates the surplus-satellite independent integrity test for ONE LAMBDA
// candidate (subset already re-differenced into fixed_cycles_by_index at
// candidate_ant). See FGOConfig::use_surplus_satellite_validation for the
// full rationale; the re-differencing identity used below:
//
//   For satellite S and the group's own DD reference R0 (both DD'd against
//   R0 in cp_by_epoch already), and an alternate in-fixed-set reference
//   Ralt (also DD'd against R0, with a KNOWN fixed integer N(Ralt,R0)):
//     geometry(S,R0)(ant) - geometry(Ralt,R0)(ant) == geometry(S,Ralt)(ant)
//   exactly (telescoping range terms), so
//     [obs(S,R0) - obs(Ralt,R0) - geometry(S,Ralt)(ant)] / lambda
//         == N(S,R0) - N(Ralt,R0)
//   The right-hand side must be (near) an integer if S's true ambiguity is
//   geometrically self-consistent with the candidate fixed position -- this
//   is evaluated with NO dependency on S's own float ambiguity estimate.
//
// epoch_cp_factors is the UNION of this epoch's active DD carrier factors
// (cp_by_epoch[i], includes both currently-fixed and FDE/gate-runtime-
// excluded arcs) and its build-time-excluded rows (cp_excluded_by_epoch[i],
// CMC level exclusions -- see FGOProblem::excluded_double_difference_
// carrier_factors). Wavelength is looked up from `signal` (not from an
// AmbiguityState) so build-time-excluded rows, which never got a segment,
// work identically to normal ones.
inline SurplusValidationOutcome evaluateSurplusSatelliteValidation(
    const std::vector<const FGOProcessor::DoubleDifferenceCarrierFactor*>& epoch_cp_factors,
    const std::map<std::size_t, int>& fixed_cycles_by_index,
    const Eigen::Vector3d& candidate_ant,
    const FGOProcessor::FGOConfig& config) {
    SurplusValidationOutcome outcome;

    const auto ddGeometry = [&](const FGOProcessor::DoubleDifferenceCarrierFactor* fp) {
        return ((fp->rover_satellite_position_ecef - candidate_ant).norm() -
                 (fp->base_satellite_position_ecef - fp->base_position_ecef).norm()) -
               ((fp->rover_reference_position_ecef - candidate_ant).norm() -
                 (fp->base_reference_position_ecef - fp->base_position_ecef).norm());
    };

    // Group by (system, signal): every factor in a group shares the same
    // primary DD reference satellite (fgo.cpp's select_reference groups by
    // exactly this key), so the alternate-reference re-differencing above is
    // only valid WITHIN a group.
    struct GroupEntry {
        const FGOProcessor::DoubleDifferenceCarrierFactor* fp;
        std::size_t amb_idx;
        bool is_fixed;
    };
    std::map<std::pair<GNSSSystem, SignalType>, std::vector<GroupEntry>> groups;
    for (const auto* fp : epoch_cp_factors) {
        if (!fp) continue;
        if (fp->use_ambiguity_difference) continue;  // only plain single-ref DD carriers supported
        // Build-time-excluded rows carry the sentinel ambiguity_index
        // (never a key in fixed_cycles_by_index), so they always land in
        // is_fixed=false -- i.e. always part of the surplus pool.
        const bool fixed = fixed_cycles_by_index.count(fp->ambiguity_index) > 0;
        groups[{fp->satellite.system, fp->signal}].push_back({fp, fp->ambiguity_index, fixed});
    }

    struct SurplusCandidate {
        GNSSSystem system;
        double distance_from_int_cycles;
    };
    std::vector<SurplusCandidate> candidates;
    for (auto& [key, entries] : groups) {
        const GNSSSystem system = key.first;
        // Alternate reference = highest-elevation FIXED-set satellite in
        // this (system,signal) group (spec: "a fixed-set satellite other
        // than the primary ref, or the next-best-elevation sat").
        const GroupEntry* alt_ref = nullptr;
        for (const auto& e : entries) {
            if (!e.is_fixed) continue;
            if (!alt_ref || e.fp->elevation_rad > alt_ref->fp->elevation_rad) alt_ref = &e;
        }
        if (!alt_ref) continue;  // no fixed-set satellite available in this group
        const auto alt_it = fixed_cycles_by_index.find(alt_ref->amb_idx);
        if (alt_it == fixed_cycles_by_index.end()) continue;
        const double wavelength_ref = signalWavelengthMeters(alt_ref->fp->signal);
        if (!(wavelength_ref > 0.0)) continue;
        const double geom_ref = ddGeometry(alt_ref->fp);
        const double n_ref_wrt_primary = static_cast<double>(alt_it->second);
        for (const auto& e : entries) {
            if (e.is_fixed || &e == alt_ref) continue;  // surplus (excluded) satellites only
            const double wavelength_s = signalWavelengthMeters(e.fp->signal);
            if (!(wavelength_s > 0.0) || std::abs(wavelength_s - wavelength_ref) > 1e-6) {
                continue;  // mixed wavelengths within one (system,signal) group should not happen
            }
            const double geom_s = ddGeometry(e.fp);
            const double value_cycles =
                (e.fp->observed_dd_carrier_m - alt_ref->fp->observed_dd_carrier_m -
                 (geom_s - geom_ref)) /
                    wavelength_s -
                n_ref_wrt_primary;
            if (!std::isfinite(value_cycles)) continue;
            const double dist = std::abs(value_cycles - std::round(value_cycles));
            candidates.push_back({system, dist});
        }
    }
    outcome.surplus_available_full = static_cast<int>(candidates.size());

    const double pdop = computeFixedSetPdop(epoch_cp_factors, fixed_cycles_by_index, candidate_ant);
    if (!std::isfinite(pdop)) return outcome;  // fixed-set geometry too weak (<4 sats): no verdict
    const double aperture_cycles = pdop < 1.0   ? config.surplus_validation_aperture_pdop_lt1_cycles
                                    : pdop <= 2.0 ? config.surplus_validation_aperture_pdop_1to2_cycles
                                                   : config.surplus_validation_aperture_pdop_gt2_cycles;
    const int min_n = std::max(1, config.surplus_validation_min_surplus_satellites);

    int last_level = -1, last_n = 0;
    bool last_pass = false;
    for (int level = 0; level < 6; ++level) {
        std::vector<double> pool;
        for (const auto& c : candidates) {
            if (surplusSystemAllowedAtLevel(c.system, level)) pool.push_back(c.distance_from_int_cycles);
        }
        if (static_cast<int>(pool.size()) < min_n) continue;
        int pass_count = 0;
        for (double d : pool) {
            if (d <= aperture_cycles) ++pass_count;
        }
        const bool level_pass =
            config.surplus_validation_require_all
                ? (pass_count == static_cast<int>(pool.size()))
                : (static_cast<double>(pass_count) / static_cast<double>(pool.size()) >=
                   config.surplus_validation_majority_fraction);
        last_level = level;
        last_n = static_cast<int>(pool.size());
        last_pass = level_pass;
        if (level_pass) {
            outcome.evaluated = true;
            outcome.pass = true;
            outcome.fallback_level = level;
            outcome.surplus_used = static_cast<int>(pool.size());
            return outcome;
        }
    }
    if (last_level >= 0) {
        outcome.evaluated = true;
        outcome.pass = last_pass;  // false here (the true case already returned above)
        outcome.fallback_level = last_level;
        outcome.surplus_used = last_n;
    }
    return outcome;
}

// Mirror of fgo.cpp's clockBiasGroup(): which receiver-clock group a system
// belongs to. GPS+QZSS share one clock; every other constellation gets its own
// inter-system-bias clock. This matches the per-constellation bias columns the
// native Eigen backend forms when config.use_inter_system_biases is set.
inline GNSSSystem clockBiasGroup(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            return GNSSSystem::GPS;
        case GNSSSystem::Galileo:
        case GNSSSystem::BeiDou:
        case GNSSSystem::GLONASS:
        case GNSSSystem::NavIC:
            return system;
        default:
            return GNSSSystem::UNKNOWN;
    }
}

// Key spaces: 'x' rover position per epoch, 'c' receiver base-clock bias [s]
// per epoch (the GPS/QZSS group), 'i' a GLOBAL (time-constant) inter-system
// bias [s] per non-GPS constellation shared across every epoch -- matching the
// native backend, which carries one bias column per constellation, not a fresh
// clock each epoch, 'a' per-ambiguity node (cycles for DD, meters for
// undifferenced), 'z' a single shared dummy ambiguity node pinned at 0.
inline gtsam::Key positionKey(std::size_t epoch) { return Symbol('x', epoch); }
inline gtsam::Key clockKey(std::size_t epoch) { return Symbol('c', epoch); }
inline gtsam::Key isbKey(int group_ordinal) { return Symbol('i', group_ordinal); }
// Static receiver secondary-signal code-bias states, in metres.  The sparse
// ordinal is derived from (GNSSSystem, SignalType), not input order.
inline gtsam::Key signalBiasKey(int signal_bias_ordinal) {
    return Symbol('f', static_cast<std::size_t>(signal_bias_ordinal));
}
// Per-epoch vertical L1 residual-ionosphere state [m].  The dedicated key
// space is intentionally separate from static signal-bias ('f') states.
inline gtsam::Key residualIonosphereKey(std::size_t epoch) {
    return Symbol('j', epoch);
}
inline gtsam::Key ambiguityKey(std::size_t index) { return Symbol('a', index); }
inline gtsam::Key dummyAmbiguityKey() { return Symbol('z', 0); }
// Milestone 2b IMU states: 'v' body velocity in nav (ENU), 'b' IMU bias, per epoch.
inline gtsam::Key velocityKey(std::size_t epoch) { return Symbol('v', epoch); }
inline gtsam::Key biasKey(std::size_t epoch) { return Symbol('b', epoch); }
// GNSS-only P+D states: 'd' is receiver clock range-rate [m/s] per epoch.
// This key space is used only when the opt-in GNSS velocity-state path is
// active, so it cannot collide with the Pose3 IMU bias keys above.
inline gtsam::Key dopplerClockDriftKey(std::size_t epoch) { return Symbol('d', epoch); }

struct IntegerConstrainedGraphCostOutcome {
    bool evaluated = false;
    bool pass = false;
    int base_factor_count = 0;
    double base_cost_before = 0.0;
    double base_cost_after = 0.0;
    std::optional<Pose3> optimized_pose;
    std::optional<gtsam::Matrix> optimized_pose_covariance;
};

inline IntegerConstrainedGraphCostOutcome evaluateIntegerConstrainedGraphCost(
    const gtsam::NonlinearFactorGraph& active_factors,
    const gtsam::Values& initial_values,
    const std::vector<std::pair<gtsam::Key, double>>& integer_constraints,
    std::optional<gtsam::Key> current_position_key,
    const FGOProcessor::FGOConfig& config) {
    IntegerConstrainedGraphCostOutcome outcome;
    gtsam::NonlinearFactorGraph base_graph;
    for (const auto& factor : active_factors) {
        if (factor) base_graph.push_back(factor);
    }
    outcome.base_factor_count = static_cast<int>(base_graph.size());
    if (base_graph.empty()) return outcome;

    gtsam::NonlinearFactorGraph constrained_graph = base_graph;
    const auto integer_noise = gtsam::noiseModel::Isotropic::Sigma(
        1, std::max(1e-9, config.integer_constrained_prior_sigma_cycles));
    int constraints_added = 0;
    for (const auto& [key, integer_cycles] : integer_constraints) {
        if (!initial_values.exists(key)) continue;
        constrained_graph.addPrior(key, integer_cycles, integer_noise);
        ++constraints_added;
    }
    if (constraints_added == 0) return outcome;

    outcome.evaluated = true;
    outcome.base_cost_before = base_graph.error(initial_values);
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(std::max(1, config.integer_constrained_max_iterations));
    params.setVerbosityLM("SILENT");
    const gtsam::Values optimized_values = gtsam::LevenbergMarquardtOptimizer(
        constrained_graph, initial_values, params).optimize();
    outcome.base_cost_after = base_graph.error(optimized_values);
    outcome.pass = std::isfinite(outcome.base_cost_before) &&
        std::isfinite(outcome.base_cost_after) &&
        outcome.base_cost_after <= outcome.base_cost_before +
            std::max(0.0, config.integer_constrained_cost_abs_tolerance);
    if (current_position_key && optimized_values.exists(*current_position_key)) {
        outcome.optimized_pose = optimized_values.at<Pose3>(*current_position_key);
        if (outcome.pass && config.compute_fixed_lag_position_covariance) {
            try {
                outcome.optimized_pose_covariance =
                    gtsam::Marginals(constrained_graph, optimized_values)
                        .marginalCovariance(*current_position_key);
            } catch (const std::exception&) {
                // Output uncertainty is optional evidence; its failure must
                // neither accept nor reject the existing integer candidate.
            }
        }
    }
    return outcome;
}

struct DdprGncCounterfactualOutcome {
    bool evaluated = false;
    bool succeeded = false;
    int factor_count = 0;
    int stages = 0;
    double base_cost_before = 0.0;
    double base_cost_after = 0.0;
    double ddpr_rms_before_m = 0.0;
    double ddpr_rms_after_m = 0.0;
    bool lambda_evaluated = false;
    int lambda_ambiguities = 0;
    double lambda_ratio = 0.0;
    bool lambda_ratio_pass = false;
    std::optional<Pose3> optimized_pose;
};

inline double scalarNoiseSigma(const gtsam::SharedNoiseModel& model) {
    if (!model) return 0.0;
    gtsam::SharedNoiseModel gaussian_model = model;
    if (const auto robust =
            std::dynamic_pointer_cast<gtsam::noiseModel::Robust>(model)) {
        gaussian_model = robust->noise();
    }
    const auto diagonal =
        std::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(gaussian_model);
    if (!diagonal || diagonal->dim() != 1) return 0.0;
    const gtsam::Vector sigmas = diagonal->sigmas();
    return sigmas.size() == 1 && std::isfinite(sigmas(0)) && sigmas(0) > 0.0
        ? sigmas(0)
        : 0.0;
}

inline DdprGncCounterfactualOutcome evaluateDdprGncCounterfactual(
    const gtsam::NonlinearFactorGraph& active_factors,
    const gtsam::Values& initial_values,
    std::optional<gtsam::Key> current_position_key,
    const std::vector<gtsam::Key>& current_ambiguity_keys,
    const FGOProcessor::FGOConfig& config) {
    DdprGncCounterfactualOutcome outcome;
    if (config.ddpr_gnc_counterfactual_max_stages <= 0 ||
        config.ddpr_gnc_counterfactual_iterations_per_stage <= 0 ||
        !std::isfinite(config.ddpr_gnc_shape) || config.ddpr_gnc_shape <= 0.0 ||
        !std::isfinite(config.ddpr_gnc_counterfactual_min_weight) ||
        config.ddpr_gnc_counterfactual_min_weight <= 0.0 ||
        config.ddpr_gnc_counterfactual_min_weight > 1.0) {
        return outcome;
    }

    try {
        gtsam::NonlinearFactorGraph base_graph;
        std::vector<std::size_t> ddpr_indices;
        std::vector<double> ddpr_sigmas;
        for (const auto& factor : active_factors) {
            if (!factor) continue;
            const std::size_t graph_index = base_graph.size();
            base_graph.push_back(factor);
            if (dynamic_cast<const gtsam::DoubleDifferencePseudorangeFactorArm*>(
                    factor.get()) == nullptr) {
                continue;
            }
            const auto noise_factor =
                std::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
            const double sigma = noise_factor
                ? scalarNoiseSigma(noise_factor->noiseModel())
                : 0.0;
            if (sigma <= 0.0) return outcome;
            ddpr_indices.push_back(graph_index);
            ddpr_sigmas.push_back(sigma);
        }
        if (base_graph.empty() || ddpr_indices.empty()) return outcome;

        auto residualsAt = [&](const gtsam::Values& values,
                               std::vector<double>* residuals,
                               double* rms) -> bool {
            residuals->clear();
            residuals->reserve(ddpr_indices.size());
            double sum_sq = 0.0;
            for (const std::size_t index : ddpr_indices) {
                const auto& factor = base_graph[index];
                const auto* ddpr = dynamic_cast<
                    const gtsam::DoubleDifferencePseudorangeFactorArm*>(factor.get());
                if (!ddpr || factor->keys().empty() ||
                    !values.exists(factor->keys().front())) {
                    return false;
                }
                const Pose3 pose = values.at<Pose3>(factor->keys().front());
                const double residual = ddpr->evaluateError(pose)(0);
                if (!std::isfinite(residual)) return false;
                residuals->push_back(residual);
                sum_sq += residual * residual;
            }
            *rms = std::sqrt(sum_sq / static_cast<double>(residuals->size()));
            return true;
        };

        std::vector<double> residuals;
        if (!residualsAt(initial_values, &residuals, &outcome.ddpr_rms_before_m)) {
            return outcome;
        }
        outcome.evaluated = true;
        outcome.factor_count = static_cast<int>(ddpr_indices.size());
        outcome.base_cost_before = base_graph.error(initial_values);

        const double shape_sq = config.ddpr_gnc_shape * config.ddpr_gnc_shape;
        double max_normalized_sq = 0.0;
        for (std::size_t i = 0; i < residuals.size(); ++i) {
            const double normalized = residuals[i] / ddpr_sigmas[i];
            max_normalized_sq = std::max(max_normalized_sq,
                                         normalized * normalized);
        }
        const double initial_mu = std::max(1.0, max_normalized_sq / shape_sq);
        const int requested_stages = config.ddpr_gnc_counterfactual_max_stages;
        const int stages = initial_mu <= 1.0 ? 1 : std::max(2, requested_stages);
        gtsam::Values candidate_values = initial_values;
        gtsam::NonlinearFactorGraph final_weighted_graph = base_graph;

        for (int stage = 0; stage < stages; ++stage) {
            if (!residualsAt(candidate_values, &residuals,
                             &outcome.ddpr_rms_after_m)) {
                return outcome;
            }
            const double progress = stages == 1
                ? 1.0
                : static_cast<double>(stage) / static_cast<double>(stages - 1);
            const double mu = std::exp((1.0 - progress) * std::log(initial_mu));
            const double scale = mu * shape_sq;
            gtsam::NonlinearFactorGraph weighted_graph = base_graph;
            for (std::size_t i = 0; i < ddpr_indices.size(); ++i) {
                const double normalized = residuals[i] / ddpr_sigmas[i];
                const double normalized_sq = normalized * normalized;
                const double weight = std::max(
                    config.ddpr_gnc_counterfactual_min_weight,
                    scale / (scale + normalized_sq));
                const auto noise_factor = std::dynamic_pointer_cast<
                    gtsam::NoiseModelFactor>(base_graph[ddpr_indices[i]]);
                if (!noise_factor) return outcome;
                const auto weighted_noise = gtsam::noiseModel::Isotropic::Sigma(
                    1, ddpr_sigmas[i] / std::sqrt(weight));
                weighted_graph[ddpr_indices[i]] =
                    noise_factor->cloneWithNewNoiseModel(weighted_noise);
            }
            gtsam::LevenbergMarquardtParams params;
            params.setMaxIterations(
                config.ddpr_gnc_counterfactual_iterations_per_stage);
            params.setVerbosityLM("SILENT");
            candidate_values = gtsam::LevenbergMarquardtOptimizer(
                weighted_graph, candidate_values, params).optimize();
            final_weighted_graph = std::move(weighted_graph);
            ++outcome.stages;
        }

        if (!residualsAt(candidate_values, &residuals,
                         &outcome.ddpr_rms_after_m)) {
            return outcome;
        }
        outcome.base_cost_after = base_graph.error(candidate_values);
        outcome.succeeded = std::isfinite(outcome.base_cost_before) &&
            std::isfinite(outcome.base_cost_after) &&
            std::isfinite(outcome.ddpr_rms_after_m);
        if (outcome.succeeded && current_position_key &&
            candidate_values.exists(*current_position_key)) {
            outcome.optimized_pose =
                candidate_values.at<Pose3>(*current_position_key);
        }
        if (outcome.succeeded && !current_ambiguity_keys.empty()) {
            try {
                gtsam::KeyVector lambda_keys;
                for (const gtsam::Key key : current_ambiguity_keys) {
                    if (candidate_values.exists(key)) lambda_keys.push_back(key);
                }
                const int n = static_cast<int>(lambda_keys.size());
                if (n > 0) {
                    const gtsam::Marginals marginals(final_weighted_graph,
                                                     candidate_values);
                    const gtsam::JointMarginal joint =
                        marginals.jointMarginalCovariance(lambda_keys);
                    Eigen::VectorXd float_ambiguities(n);
                    Eigen::MatrixXd covariance(n, n);
                    for (int row = 0; row < n; ++row) {
                        float_ambiguities(row) =
                            candidate_values.at<double>(lambda_keys[row]);
                        for (int col = 0; col < n; ++col) {
                            covariance(row, col) =
                                joint(lambda_keys[row], lambda_keys[col])(0, 0);
                        }
                    }
                    LambdaCandidateDiagnostics lambda;
                    if (float_ambiguities.allFinite() && covariance.allFinite() &&
                        lambdaSearchTopK(float_ambiguities, covariance, 2, lambda) &&
                        lambda.squared_residuals.size() >= 2) {
                        const double best = lambda.squared_residuals(0);
                        const double second = lambda.squared_residuals(1);
                        outcome.lambda_evaluated = std::isfinite(best) &&
                            std::isfinite(second) && best > 0.0;
                        outcome.lambda_ambiguities = n;
                        outcome.lambda_ratio = outcome.lambda_evaluated
                            ? second / best
                            : 0.0;
                        outcome.lambda_ratio_pass = outcome.lambda_evaluated &&
                            n >= 6 && outcome.lambda_ratio >=
                                config.lambda_ratio_threshold;
                    }
                }
            } catch (const std::exception&) {
                // Candidate positioning remains valid if its batch marginal
                // is rank-deficient; the LAMBDA counterfactual simply has no
                // verdict for this epoch.
            }
        }
    } catch (const std::exception&) {
        // Counterfactual diagnostics must fail closed and never affect the
        // live incremental solution.
    }
    return outcome;
}

// ENU-from-ECEF rotation whose columns are the East/North/Up basis vectors
// expressed in ECEF, i.e. ecef_vec = R_ecef_enu * enu_vec. Matches
// core/coordinates.hpp enu2ecef(). Used to build the ecef_T_nav Pose3 that
// the DD '...FactorArm' factors and the IMU factor share (nav = local ENU).
inline gtsam::Rot3 ecefFromEnuRotation(double lat, double lon) {
    const double sinlat = std::sin(lat), coslat = std::cos(lat);
    const double sinlon = std::sin(lon), coslon = std::cos(lon);
    gtsam::Matrix3 R;
    R << -sinlon, -sinlat * coslon, coslat * coslon,
          coslon, -sinlat * sinlon, coslat * sinlon,
          0.0,     coslat,          sinlat;
    return gtsam::Rot3(R);
}

// Stable small ordinal per clock group. GPS/QZSS are group 0 (the base clock,
// no ISB node); every other constellation gets its own global ISB node. UNKNOWN
// groups (SBAS etc.) fold into GPS(0). When inter-system biases are disabled
// everything is group 0 (single base clock, no ISB), matching the native
// backend with use_inter_system_biases=false.
inline int clockGroupOrdinal(GNSSSystem group, bool use_inter_system_biases) {
    if (!use_inter_system_biases) {
        return 0;
    }
    switch (group) {
        case GNSSSystem::GPS: return 0;
        case GNSSSystem::Galileo: return 1;
        case GNSSSystem::BeiDou: return 2;
        case GNSSSystem::GLONASS: return 3;
        case GNSSSystem::NavIC: return 4;
        default: return 0;
    }
}

// A scalar clock key is represented in seconds by the legacy GTSAM graph and
// in metres by the Phase92 source-parity graph.  Keep the conversion explicit
// at each factor boundary; magnitude-based unit guessing is forbidden.
inline double clockStateScale(bool meter_state) {
    return meter_state ? 1.0 : gtsam::gnss::C_LIGHT;
}

inline double clockStateTerm(double clock_state, bool meter_state) {
    return clockStateScale(meter_state) * clock_state;
}

// PositionSolution exposes receiver_clock_bias in seconds.  Keep the only
// meter-state conversion at this explicit graph/output boundary so residual
// diagnostics and all factor equations cannot accidentally divide twice (or
// leave a metre state exposed as seconds).
inline double publicClockSeconds(double clock_state, bool meter_state) {
    return meter_state ? clock_state / gtsam::gnss::C_LIGHT : clock_state;
}

// Phase101 source-parity clock topology.  This is the exact component
// numbering returned by gsdc2023/functions/sysfreq2sigtype.m.  The sentinel
// used by the MATLAB helper for unsupported systems/frequencies is deliberately
// represented as -1 here so an active candidate can fail closed instead of
// indexing a seventh component or silently folding it into the base clock.
constexpr std::size_t kNativeSourceClockVectorDimension = 7;

inline int sourceClockComponentFor(GNSSSystem system, SignalType signal) {
    return raw_p_seed::c7ClockComponentFor(system, signal);
}

inline gtsam::Vector sourceClockComponentJacobian(int component) {
    gtsam::Vector hc = gtsam::Vector::Zero(kNativeSourceClockVectorDimension);
    if (component >= 0 &&
        static_cast<std::size_t>(component) < kNativeSourceClockVectorDimension) {
        hc(0) = 1.0;
        // Assignment (rather than addition) is intentional for GPS L1:
        // sysfreq2sigtype returns zero, and the official XC factor sets
        // hc(0)=1 followed by hc(sysidx)=1, leaving one base-clock coefficient.
        hc(static_cast<std::size_t>(component)) = 1.0;
    }
    return hc;
}

inline bool finiteSourceClockVector(const gtsam::Vector& value) {
    return value.size() ==
               static_cast<Eigen::Index>(kNativeSourceClockVectorDimension) &&
           value.allFinite();
}

// Phase135 auxiliary official X key.  The historical Pose3 position key is
// already `x`; an upper-case Symbol keeps the official X namespace distinct
// and prevents a Pose3/Vector type collision in Values.
inline gtsam::Key affinePositionKey(std::size_t epoch) {
    return Symbol('X', epoch);
}

// Source geodist convention used by the official Gsat/RTKLIB factors.  RTKLIB
// geodist() emits e=(satellite-receiver)/range, but the official affine
// scripts pass -e to PseudorangeFactor_XC, DopplerFactor_VD, and
// TDCPFactor_XXCC (the receiver derivative).  This adapter therefore exposes
// that factor LOS directly.  Sagnac is evaluated once from the unrotated
// transmit-time satellite state; callers must not feed an already rotated
// state here.
struct Phase135SourceGeometry {
    gtsam::Vector3 los = gtsam::Vector3::Zero();
    double range_m = std::numeric_limits<double>::quiet_NaN();
};

inline bool phase135SourceGeodist(const Point3& satellite_transmit,
                                  const Point3& receiver_ecef,
                                  Phase135SourceGeometry& geometry) {
    if (!satellite_transmit.allFinite() || !receiver_ecef.allFinite()) {
        return false;
    }
    // Preserve RTKLIB geodist()'s physical-state guard: a broadcast state
    // below the WGS-84 surface is not a satellite geometry and must never be
    // admitted to the all-or-nothing affine family.
    const double satellite_radius = satellite_transmit.norm();
    if (!(satellite_radius >= constants::WGS84_A) ||
        !std::isfinite(satellite_radius)) {
        return false;
    }
    const Point3 delta = satellite_transmit - receiver_ecef;
    const double geometric_range = delta.norm();
    if (!(geometric_range > 0.0) || !std::isfinite(geometric_range)) {
        return false;
    }
    // RTKLIB geodist() uses rs[0]*rr[1]-rs[1]*rr[0] and emits
    // e=(rs-rr)/|rs-rr|.  The official affine factor scripts explicitly pass
    // -e as their receiver-derivative LOS.
    const double sagnac = constants::OMEGA_E /
                          constants::SPEED_OF_LIGHT *
                          (satellite_transmit.x() * receiver_ecef.y() -
                           satellite_transmit.y() * receiver_ecef.x());
    geometry.range_m = geometric_range + sagnac;
    geometry.los = -delta / geometric_range;
    return std::isfinite(geometry.range_m) && geometry.los.allFinite() &&
           std::abs(geometry.los.norm() - 1.0) < 1e-9;
}

// Exact Gsat/Gobs Doppler preparation for the Phase135 adapter.  Inputs use
// the source units: satellite/receiver positions in ECEF metres, velocities
// and measured range rate in metres/second, and satellite clock drift already
// converted to metres/second.  Gsat's e remains positive internally; the
// factor LOS is -e and is provided by phase135SourceGeodist().
inline bool phase135OfficialDopplerResidual(
    const Point3& satellite_transmit, const gtsam::Vector3& satellite_velocity,
    const Point3& receiver_ecef, const gtsam::Vector3& receiver_velocity,
    double measured_range_rate_mps, double satellite_clock_drift_mps,
    double& modeled_range_rate_mps, double& residual_mps) {
    if (!satellite_velocity.allFinite() || !receiver_velocity.allFinite() ||
        !std::isfinite(measured_range_rate_mps) ||
        !std::isfinite(satellite_clock_drift_mps)) {
        return false;
    }
    Phase135SourceGeometry geometry;
    if (!phase135SourceGeodist(satellite_transmit, receiver_ecef, geometry)) {
        return false;
    }
    // phase135SourceGeodist returns the official factor LOS -e.  Recover e
    // only for the Gsat known range-rate expression; no second geometry or
    // Sagnac evaluation is introduced.
    const gtsam::Vector3 e = -geometry.los;
    const double explicit_sagnac =
        constants::OMEGA_E / constants::SPEED_OF_LIGHT *
        (satellite_velocity.y() * receiver_ecef.x() +
         satellite_transmit.y() * receiver_velocity.x() -
         satellite_velocity.x() * receiver_ecef.y() -
         satellite_transmit.x() * receiver_velocity.y());
    modeled_range_rate_mps =
        (satellite_velocity - receiver_velocity).dot(e) + explicit_sagnac;
    residual_mps = measured_range_rate_mps -
                   (modeled_range_rate_mps - satellite_clock_drift_mps);
    return std::isfinite(modeled_range_rate_mps) &&
           std::isfinite(residual_mps);
}

// Official Pose3Point3Factor_PX, kept local so the candidate does not depend
// on the optional reproducibility-cache header.  The exact residual and
// GTSAM Pose3 tangent Jacobian match the pinned source factor.
class Phase135Pose3Point3FactorPX
    : public gtsam::NoiseModelFactorN<Pose3, gtsam::Vector> {
 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, gtsam::Vector>;
    using Base::evaluateError;

    Phase135Pose3Point3FactorPX(gtsam::Key pose, gtsam::Key point,
                                const gtsam::SharedNoiseModel& model)
        : Base(model, pose, point) {}

    gtsam::Vector evaluateError(
        const Pose3& pose, const gtsam::Vector& point,
        gtsam::OptionalMatrixType H_pose,
        gtsam::OptionalMatrixType H_point) const override {
        if (point.size() != 3 || !point.allFinite()) {
            return gtsam::Vector::Constant(
                3, std::numeric_limits<double>::quiet_NaN());
        }
        gtsam::Matrix H_translation;
        const gtsam::Vector3 error =
            pose.translation(H_translation) - gtsam::Vector3(point);
        if (H_pose) *H_pose = H_translation;
        if (H_point) *H_point = -gtsam::I_3x3;
        return error;
    }
};

// Official fixed-initial-LOS P factor for the GNSS-first Point3 state.  The
// public source factor uses a dynamic Vector X; the native staging graph has
// historically exposed Point3, so this adapter preserves that value type
// while retaining the exact residual/Jacobian/key order [X,C].
class Phase135PseudorangeAffinePointFactor
    : public gtsam::NoiseModelFactorN<Point3, gtsam::Vector> {
    gtsam::Vector los_;
    double residual_m_ = 0.0;
    gtsam::Vector initial_position_;
    gtsam::Vector clock_selector_;

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, gtsam::Vector>;
    using Base::evaluateError;
    Phase135PseudorangeAffinePointFactor(
        gtsam::Key point, gtsam::Key clock, const gtsam::Vector& los,
        double residual_m, int component, const gtsam::Vector& initial_position,
        const gtsam::SharedNoiseModel& model)
        : Base(model, point, clock),
          los_(los),
          residual_m_(residual_m),
          initial_position_(initial_position),
          clock_selector_(sourceClockComponentJacobian(component)) {}

    gtsam::Vector evaluateError(
        const Point3& point, const gtsam::Vector& clock,
        gtsam::OptionalMatrixType H_point,
        gtsam::OptionalMatrixType H_clock) const override {
        if (los_.size() != 3 || initial_position_.size() != 3 ||
            clock.size() != static_cast<Eigen::Index>(kNativeSourceClockVectorDimension) ||
            !los_.allFinite() || !initial_position_.allFinite() ||
            !clock.allFinite() || !std::isfinite(residual_m_)) {
            return gtsam::Vector::Constant(
                1, std::numeric_limits<double>::quiet_NaN());
        }
        if (H_point) *H_point = los_.transpose();
        if (H_clock) *H_clock = clock_selector_.transpose();
        return (gtsam::Vector(1) <<
                los_.dot(gtsam::Vector3(point) - initial_position_) +
                    clock_selector_.dot(clock) - residual_m_)
            .finished();
    }
};

// Official PseudorangeFactor_XC with a dynamic auxiliary X Vector.
class Phase135PseudorangeAffineVectorFactor
    : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector> {
    gtsam::Vector los_;
    double residual_m_ = 0.0;
    gtsam::Vector initial_position_;
    gtsam::Vector clock_selector_;

 public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector>;
    using Base::evaluateError;
    Phase135PseudorangeAffineVectorFactor(
        gtsam::Key point, gtsam::Key clock, const gtsam::Vector& los,
        double residual_m, int component, const gtsam::Vector& initial_position,
        const gtsam::SharedNoiseModel& model)
        : Base(model, point, clock),
          los_(los),
          residual_m_(residual_m),
          initial_position_(initial_position),
          clock_selector_(sourceClockComponentJacobian(component)) {}

    gtsam::Vector evaluateError(
        const gtsam::Vector& point, const gtsam::Vector& clock,
        gtsam::OptionalMatrixType H_point,
        gtsam::OptionalMatrixType H_clock) const override {
        if (point.size() != 3 || initial_position_.size() != 3 ||
            clock.size() != static_cast<Eigen::Index>(kNativeSourceClockVectorDimension) ||
            !point.allFinite() || !clock.allFinite() ||
            !std::isfinite(residual_m_)) {
            return gtsam::Vector::Constant(
                1, std::numeric_limits<double>::quiet_NaN());
        }
        if (H_point) *H_point = los_.transpose();
        if (H_clock) *H_clock = clock_selector_.transpose();
        return (gtsam::Vector(1) <<
                los_.dot(point - initial_position_) +
                    clock_selector_.dot(clock) - residual_m_)
            .finished();
    }
};

// Official DopplerFactor_VD with the source initial velocity and residual
// convention.  D is represented as a one-element Vector so it has the same
// Values type as the CCDD drift key.
class Phase135DopplerAffineFactor
    : public gtsam::NoiseModelFactorN<gtsam::Vector3, gtsam::Vector> {
    gtsam::Vector3 los_ = gtsam::Vector3::Zero();
    double residual_mps_ = 0.0;
    gtsam::Vector3 initial_velocity_ = gtsam::Vector3::Zero();

 public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Vector3, gtsam::Vector>;
    using Base::evaluateError;
    Phase135DopplerAffineFactor(
        gtsam::Key velocity, gtsam::Key drift, const gtsam::Vector3& los,
        double residual_mps, const gtsam::Vector3& initial_velocity,
        const gtsam::SharedNoiseModel& model)
        : Base(model, velocity, drift),
          los_(los), residual_mps_(residual_mps),
          initial_velocity_(initial_velocity) {}

    gtsam::Vector evaluateError(
        const gtsam::Vector3& velocity, const gtsam::Vector& drift,
        gtsam::OptionalMatrixType H_velocity,
        gtsam::OptionalMatrixType H_drift) const override {
        if (!los_.allFinite() || !initial_velocity_.allFinite() ||
            drift.size() != 1 || !drift.allFinite() ||
            !std::isfinite(residual_mps_)) {
            return gtsam::Vector::Constant(
                1, std::numeric_limits<double>::quiet_NaN());
        }
        if (H_velocity) *H_velocity = los_.transpose();
        if (H_drift) *H_drift = gtsam::Matrix::Constant(1, 1, 1.0);
        return (gtsam::Vector(1) <<
                los_.dot(velocity - initial_velocity_) + drift(0) -
                    residual_mps_)
            .finished();
    }
};

// Official TDCPFactor_XXCC.  The source uses the previous endpoint LOS and
// C[0] only; C7 mapping for pseudorange remains in the P factor.
class Phase135TdcpAffinePointFactor
    : public gtsam::NoiseModelFactorN<Point3, Point3, gtsam::Vector,
                                      gtsam::Vector> {
    gtsam::Vector3 los_ = gtsam::Vector3::Zero();
    double tdcp_m_ = 0.0;
    gtsam::Vector3 initial_previous_ = gtsam::Vector3::Zero();
    gtsam::Vector3 initial_current_ = gtsam::Vector3::Zero();
    gtsam::Vector clock_selector_ = gtsam::Vector::Zero(7);

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, Point3, gtsam::Vector,
                                          gtsam::Vector>;
    using Base::evaluateError;
    Phase135TdcpAffinePointFactor(
        gtsam::Key previous_point, gtsam::Key current_point,
        gtsam::Key previous_clock, gtsam::Key current_clock,
        const gtsam::Vector3& los, double tdcp_m,
        const gtsam::Vector3& initial_previous,
        const gtsam::Vector3& initial_current,
        const gtsam::SharedNoiseModel& model)
        : Base(model, previous_point, current_point, previous_clock,
               current_clock),
          los_(los), tdcp_m_(tdcp_m), initial_previous_(initial_previous),
          initial_current_(initial_current),
          clock_selector_(sourceClockComponentJacobian(0)) {}

    gtsam::Vector evaluateError(
        const Point3& previous_point, const Point3& current_point,
        const gtsam::Vector& previous_clock, const gtsam::Vector& current_clock,
        gtsam::OptionalMatrixType H_previous_point,
        gtsam::OptionalMatrixType H_current_point,
        gtsam::OptionalMatrixType H_previous_clock,
        gtsam::OptionalMatrixType H_current_clock) const override {
        if (previous_clock.size() != 7 || current_clock.size() != 7 ||
            !previous_clock.allFinite() || !current_clock.allFinite() ||
            !los_.allFinite() || !initial_previous_.allFinite() ||
            !initial_current_.allFinite() || !std::isfinite(tdcp_m_)) {
            return gtsam::Vector::Constant(
                1, std::numeric_limits<double>::quiet_NaN());
        }
        if (H_previous_point) *H_previous_point = -los_.transpose();
        if (H_current_point) *H_current_point = los_.transpose();
        if (H_previous_clock) *H_previous_clock = -clock_selector_.transpose();
        if (H_current_clock) *H_current_clock = clock_selector_.transpose();
        const gtsam::Vector3 delta =
            (gtsam::Vector3(current_point) - initial_current_) -
            (gtsam::Vector3(previous_point) - initial_previous_);
        return (gtsam::Vector(1) <<
                los_.dot(delta) +
                    clock_selector_.dot(current_clock - previous_clock) - tdcp_m_)
            .finished();
    }
};

class Phase135TdcpAffineVectorFactor
    : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                      gtsam::Vector, gtsam::Vector> {
    gtsam::Vector3 los_ = gtsam::Vector3::Zero();
    double tdcp_m_ = 0.0;
    gtsam::Vector3 initial_previous_ = gtsam::Vector3::Zero();
    gtsam::Vector3 initial_current_ = gtsam::Vector3::Zero();
    gtsam::Vector clock_selector_ = gtsam::Vector::Zero(7);

 public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                          gtsam::Vector, gtsam::Vector>;
    using Base::evaluateError;
    Phase135TdcpAffineVectorFactor(
        gtsam::Key previous_point, gtsam::Key current_point,
        gtsam::Key previous_clock, gtsam::Key current_clock,
        const gtsam::Vector3& los, double tdcp_m,
        const gtsam::Vector3& initial_previous,
        const gtsam::Vector3& initial_current,
        const gtsam::SharedNoiseModel& model)
        : Base(model, previous_point, current_point, previous_clock,
               current_clock),
          los_(los), tdcp_m_(tdcp_m), initial_previous_(initial_previous),
          initial_current_(initial_current),
          clock_selector_(sourceClockComponentJacobian(0)) {}

    gtsam::Vector evaluateError(
        const gtsam::Vector& previous_point, const gtsam::Vector& current_point,
        const gtsam::Vector& previous_clock, const gtsam::Vector& current_clock,
        gtsam::OptionalMatrixType H_previous_point,
        gtsam::OptionalMatrixType H_current_point,
        gtsam::OptionalMatrixType H_previous_clock,
        gtsam::OptionalMatrixType H_current_clock) const override {
        if (previous_point.size() != 3 || current_point.size() != 3 ||
            previous_clock.size() != 7 || current_clock.size() != 7 ||
            !previous_point.allFinite() || !current_point.allFinite() ||
            !previous_clock.allFinite() || !current_clock.allFinite() ||
            !los_.allFinite() || !initial_previous_.allFinite() ||
            !initial_current_.allFinite() || !std::isfinite(tdcp_m_)) {
            return gtsam::Vector::Constant(
                1, std::numeric_limits<double>::quiet_NaN());
        }
        if (H_previous_point) *H_previous_point = -los_.transpose();
        if (H_current_point) *H_current_point = los_.transpose();
        if (H_previous_clock) *H_previous_clock = -clock_selector_.transpose();
        if (H_current_clock) *H_current_clock = clock_selector_.transpose();
        const gtsam::Vector3 delta =
            (gtsam::Vector3(current_point) - initial_current_) -
            (gtsam::Vector3(previous_point) - initial_previous_);
        return (gtsam::Vector(1) <<
                los_.dot(delta) +
                    clock_selector_.dot(current_clock - previous_clock) - tdcp_m_)
            .finished();
    }
};

// Source-exact ClockFactor_CCDD C0/D contract.  The source vector factor's
// active component is ported here as a scalar row.  Its metre form is the
// official row multiplied by C_LIGHT; the legacy seconds form remains the
// historical compatibility path.
constexpr double kNativeSourceClockC0DSigmaM = 0.1;
constexpr double kNativeSourceClockC0DMaxGapS = 1.5;

enum class NativeSourceClockC0DSkipReason {
    Eligible,
    InvalidDt,
    Gap,
    PhoneExcluded,
    ClockJump,
};

struct NativeSourceClockC0DEdgeDecision {
    bool eligible = false;
    NativeSourceClockC0DSkipReason reason =
        NativeSourceClockC0DSkipReason::InvalidDt;
};

// The source exclusion list is keyed by the exact Android phone model, not a
// route/path substring.  An absent model fails closed because the candidate's
// raw entry-point contract requires the dataset phone to be passed explicitly.
inline bool nativeSourceClockC0DPhoneExcluded(const std::string& phone) {
    return phone.empty() || phone == "sm-a205u" || phone == "sm-a505u" ||
           phone == "samsunga325g";
}

inline NativeSourceClockC0DEdgeDecision nativeSourceClockC0DEdgeDecision(
    double dt_s, bool clock_jump, const std::string& phone) {
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
        return {false, NativeSourceClockC0DSkipReason::InvalidDt};
    }
    if (dt_s >= kNativeSourceClockC0DMaxGapS) {
        return {false, NativeSourceClockC0DSkipReason::Gap};
    }
    if (nativeSourceClockC0DPhoneExcluded(phone)) {
        return {false, NativeSourceClockC0DSkipReason::PhoneExcluded};
    }
    if (clock_jump) {
        return {false, NativeSourceClockC0DSkipReason::ClockJump};
    }
    return {true, NativeSourceClockC0DSkipReason::Eligible};
}

// Library-side fail-closed check for the same backend constraints enforced by
// gnss_fgo_imu_no_base's CLI contract.  The CLI additionally verifies the
// exact direct-quality/no-bridge option spellings and dataset phone mapping;
// this helper prevents a direct library caller from silently selecting the
// legacy scalar row with an incompatible candidate configuration.
inline bool nativeSourceClockC0DBackendConfigurationAllowed(
    const FGOProcessor::FGOConfig& config) {
    const bool direct_wls_ephemeral_main_path =
        config.use_native_direct_wls_ephemeral_c7d_main_seed &&
        !config.use_native_source_clock_c0d_gnss_first_meter_state_handoff &&
        !config.use_native_source_clock_c0d_raw_drift_d_initializer &&
        !config.use_native_pdc_state_bridge &&
        !config.use_fixed_lag_smoother &&
        !config.use_inter_system_biases &&
        !config.native_source_clock_c0d_phone.empty();
    const bool pose3_imu_path =
        config.use_pose3_state && config.use_imu &&
        config.use_upstream_observable_quality &&
        config.use_undifferenced_doppler_factors &&
        config.use_motion_factors && config.use_clock_motion_factors &&
        !config.use_native_pdc_state_bridge &&
        !config.use_fixed_lag_smoother &&
        !config.native_source_clock_c0d_phone.empty();
    const bool phase171_no_doppler_pose3_imu_path =
        config.use_native_phase171_raw_p_no_doppler_imu_main &&
        config.use_pose3_state && config.use_imu &&
        config.use_upstream_observable_quality &&
        !config.use_undifferenced_doppler_factors &&
        config.use_motion_factors && config.use_clock_motion_factors &&
        config.use_native_source_clock_c0d_meter_state_parity &&
        config.use_native_source_clock_c0d_epoch_vector_parity &&
        config.use_native_source_clock_c0d_gnss_first_meter_state_handoff &&
        !config.use_native_source_clock_c0d_raw_drift_d_initializer &&
        !config.use_native_pdc_state_bridge &&
        !config.use_fixed_lag_smoother &&
        !config.use_inter_system_biases &&
        !config.use_receiver_signal_bias_states &&
        !config.use_residual_ionosphere_states &&
        !config.native_source_clock_c0d_phone.empty();
    const bool gnss_first_point3_velocity_path =
        config.use_native_source_clock_c0d_gnss_first_meter_state_handoff &&
        !config.use_pose3_state && !config.use_imu &&
        config.use_velocity_states && config.use_upstream_observable_quality &&
        config.use_undifferenced_doppler_factors &&
        config.use_motion_factors && config.use_clock_motion_factors &&
        !config.use_native_pdc_state_bridge &&
        !config.use_fixed_lag_smoother &&
        !config.native_source_clock_c0d_phone.empty();
    const bool raw_p_no_doppler_point3_velocity_path =
        config.use_native_raw_p_no_doppler_graph &&
        !config.use_pose3_state && !config.use_imu &&
        config.use_velocity_states && config.use_motion_factors &&
        config.use_velocity_motion_factors && config.use_clock_motion_factors &&
        config.use_native_source_clock_c0d_factor &&
        config.use_native_source_clock_c0d_meter_state_parity &&
        config.use_native_source_clock_c0d_epoch_vector_parity &&
        !config.use_native_pdc_state_bridge && !config.use_fixed_lag_smoother &&
        !config.use_inter_system_biases &&
        !config.native_source_clock_c0d_phone.empty();
    const bool raw_p_ecef_doppler_point3_velocity_path =
        config.use_native_raw_p_ecef_doppler_gnss_first &&
        !config.use_native_raw_p_no_doppler_graph &&
        !config.use_pose3_state && !config.use_imu &&
        config.use_velocity_states && config.use_motion_factors &&
        config.use_velocity_motion_factors && config.use_clock_motion_factors &&
        config.use_native_source_clock_c0d_factor &&
        config.use_native_source_clock_c0d_meter_state_parity &&
        config.use_native_source_clock_c0d_epoch_vector_parity &&
        config.use_undifferenced_doppler_factors &&
        config.use_corrected_undifferenced_doppler_factors &&
        config.use_upstream_observable_quality &&
        !config.use_native_pdc_state_bridge && !config.use_fixed_lag_smoother &&
        !config.use_inter_system_biases &&
        !config.native_source_clock_c0d_phone.empty();
    return !config.use_native_source_clock_c0d_factor ||
           (pose3_imu_path && !config.use_native_direct_wls_ephemeral_c7d_main_seed) ||
           phase171_no_doppler_pose3_imu_path ||
           gnss_first_point3_velocity_path ||
           raw_p_no_doppler_point3_velocity_path ||
           raw_p_ecef_doppler_point3_velocity_path ||
           direct_wls_ephemeral_main_path;
}

// Active C0/D row of taroz/gtsam_gnss/src/ClockFactor_CCDD.h, expressed in
// the native scalar clock units:
//   error = (c2-c1) - ((d1+d2)*dt/(2*C_LIGHT))       [seconds]
// with Jacobian order [c1,c2,d1,d2] equal to
//   [-1,+1,-dt/(2*C_LIGHT),-dt/(2*C_LIGHT)].
class SourceClockC0DFactor
    : public gtsam::NoiseModelFactorN<double, double, double, double> {
    double dt_s_ = 0.0;
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<double, double, double, double>;
    using Base::evaluateError;

    SourceClockC0DFactor(gtsam::Key clock_previous,
                         gtsam::Key clock_current,
                         gtsam::Key drift_previous,
                         gtsam::Key drift_current,
                         double dt_s,
                         const gtsam::SharedNoiseModel& model,
                         bool meter_state = false)
        : Base(model, clock_previous, clock_current, drift_previous,
               drift_current),
          dt_s_(dt_s),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const double& clock_previous, const double& clock_current,
        const double& drift_previous, const double& drift_current,
        gtsam::OptionalMatrixType H_clock_previous,
        gtsam::OptionalMatrixType H_clock_current,
        gtsam::OptionalMatrixType H_drift_previous,
        gtsam::OptionalMatrixType H_drift_current) const override {
        const double drift_coefficient =
            dt_s_ / (2.0 * clockStateScale(meter_state_));
        if (H_clock_previous) {
            *H_clock_previous = gtsam::Matrix::Constant(1, 1, -1.0);
        }
        if (H_clock_current) {
            *H_clock_current = gtsam::Matrix::Constant(1, 1, 1.0);
        }
        if (H_drift_previous) {
            *H_drift_previous =
                gtsam::Matrix::Constant(1, 1, -drift_coefficient);
        }
        if (H_drift_current) {
            *H_drift_current =
                gtsam::Matrix::Constant(1, 1, -drift_coefficient);
        }
        const double error =
            (clock_current - clock_previous) -
            (drift_previous + drift_current) * drift_coefficient;
        return (gtsam::Vector(1) << error).finished();
    }
};

// Official source ClockFactor_CCDD topology: C is a seven-dimensional
// epoch-local metre vector while D is a one-dimensional metre/second vector.
// Only C[0] participates in the clock-drift row; the other six C components
// are constrained to remain unchanged by the source's zero-sigma entries
// (hard constraints, NOT zero information). Keeping the complete vector in
// one factor is important: replacing it with a scalar C plus global `i` keys
// is a different state topology and is forbidden by Phase101.
class SourceClockVectorC0DFactor
    : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                      gtsam::Vector, gtsam::Vector> {
    double dt_s_ = 0.0;

 public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                          gtsam::Vector, gtsam::Vector>;
    using Base::evaluateError;

    SourceClockVectorC0DFactor(
        gtsam::Key clock_previous, gtsam::Key clock_current,
        gtsam::Key drift_previous, gtsam::Key drift_current, double dt_s,
        const gtsam::SharedNoiseModel& model)
        : Base(model, clock_previous, clock_current, drift_previous,
               drift_current),
          dt_s_(dt_s) {}

    gtsam::Vector evaluateError(
        const gtsam::Vector& clock_previous,
        const gtsam::Vector& clock_current,
        const gtsam::Vector& drift_previous,
        const gtsam::Vector& drift_current,
        gtsam::OptionalMatrixType H_clock_previous,
        gtsam::OptionalMatrixType H_clock_current,
        gtsam::OptionalMatrixType H_drift_previous,
        gtsam::OptionalMatrixType H_drift_current) const override {
        constexpr Eigen::Index kDimension =
            static_cast<Eigen::Index>(kNativeSourceClockVectorDimension);
        if (H_clock_previous) *H_clock_previous = -gtsam::Matrix::Identity(kDimension, kDimension);
        if (H_clock_current) *H_clock_current = gtsam::Matrix::Identity(kDimension, kDimension);
        gtsam::Matrix hd = gtsam::Matrix::Zero(kDimension, 1);
        hd(0, 0) = -dt_s_ / 2.0;
        if (H_drift_previous) *H_drift_previous = hd;
        if (H_drift_current) *H_drift_current = hd;
        if (!finiteSourceClockVector(clock_previous) ||
            !finiteSourceClockVector(clock_current) ||
            drift_previous.size() != 1 || drift_current.size() != 1 ||
            !drift_previous.allFinite() || !drift_current.allFinite() ||
            !std::isfinite(dt_s_)) {
            return gtsam::Vector::Constant(kDimension,
                                           std::numeric_limits<double>::quiet_NaN());
        }
        gtsam::Vector error = clock_current - clock_previous;
        error(0) -= (drift_previous(0) + drift_current(0)) * dt_s_ / 2.0;
        return error;
    }
};

// Phase96 factor-family labels are intentionally derived from the concrete
// factor type already present in the graph.  This is a read-only accounting
// view: no factor is replaced, reweighted, reordered, or evaluated at a
// different state.  Unknown types remain visible as "other" rather than being
// silently dropped from the family-cost accounting.
inline std::string phase96FactorFamily(const gtsam::NonlinearFactor& factor) {
    if (dynamic_cast<const SourceClockC0DFactor*>(&factor) != nullptr ||
        dynamic_cast<const SourceClockVectorC0DFactor*>(&factor) != nullptr) {
        return "clock_ccdd";
    }
    const std::string type_name = typeid(factor).name();
    if (type_name.find("SourceClockC0D") != std::string::npos ||
        type_name.find("ClockFactor_CCDD") != std::string::npos) {
        return "clock_ccdd";
    }
    if (type_name.find("Pseudorange") != std::string::npos) {
        return "gnss_code";
    }
    if (type_name.find("Doppler") != std::string::npos) {
        return "gnss_doppler";
    }
    if (type_name.find("CombinedImu") != std::string::npos ||
        type_name.find("ImuFactor") != std::string::npos ||
        type_name.find("ConstantBias") != std::string::npos) {
        return "imu_preintegration_bias";
    }
    if (type_name.find("Prior") != std::string::npos ||
        type_name.find("Pose3RotationPrior") != std::string::npos) {
        return "priors";
    }
    if (type_name.find("BetweenFactor") != std::string::npos ||
        type_name.find("Motion") != std::string::npos) {
        return "motion";
    }
    return "other";
}

inline std::string phase96VariableBucket(gtsam::Key key) {
    const char symbol = gtsam::Symbol(key).chr();
    switch (symbol) {
        case 'x':
        case 'p': return "position";
        case 'v': return "velocity";
        case 'b': return "imu_bias";
        case 'c': return "clock_c";
        case 'd': return "clock_d";
        case 'i': return "clock_isb";
        case 'f': return "signal_bias";
        case 'a': return "ambiguity";
        default: return "other";
    }
}

inline void phase96RecordException(
    FGOProcessor::FGOPhase96MainDiagnostics& diagnostics,
    const std::string& stage, const std::string& classification,
    const std::string& type, const std::string& message) {
    for (auto& existing : diagnostics.exceptions) {
        if (existing.stage == stage && existing.classification == classification &&
            existing.type == type && existing.message == message) {
            ++existing.count;
            return;
        }
    }
    diagnostics.exceptions.push_back(
        {stage, classification, type, message, 1});
}

// Collect factor-family costs and the blockwise gradient/normal-diagonal
// norms from the one diagnostic linearization.  The helper is only called by
// the explicit Phase96 flag.  It never feeds these values back into GTSAM.
inline void collectPhase96InitialGraphDiagnostics(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial,
    FGOProcessor::FGOPhase96MainDiagnostics& diagnostics) {
    diagnostics.graph_observed = true;
    diagnostics.graph_factor_count = graph.size();
    diagnostics.graph_value_count = initial.size();

    double family_sum = 0.0;
    bool family_sum_finite = true;
    try {
        diagnostics.graph_initial_cost = graph.error(initial);
    } catch (const std::exception& e) {
        diagnostics.graph_initial_cost =
            std::numeric_limits<double>::quiet_NaN();
        phase96RecordException(diagnostics, "initial_graph_error", "exception",
                               typeid(e).name(), e.what());
    } catch (...) {
        diagnostics.graph_initial_cost =
            std::numeric_limits<double>::quiet_NaN();
        phase96RecordException(diagnostics, "initial_graph_error", "exception",
                               "unknown", "unknown exception");
    }

    for (std::size_t index = 0; index < graph.size(); ++index) {
        const auto factor = graph[index];
        if (!factor) continue;
        const std::string family = phase96FactorFamily(*factor);
        auto family_it = std::find_if(
            diagnostics.factor_families.begin(),
            diagnostics.factor_families.end(),
            [&](const auto& value) { return value.family == family; });
        if (family_it == diagnostics.factor_families.end()) {
            diagnostics.factor_families.push_back({});
            family_it = std::prev(diagnostics.factor_families.end());
            family_it->family = family;
        }
        ++family_it->factor_count;
        try {
            const double cost = factor->error(initial);
            if (std::isfinite(cost)) {
                ++family_it->finite_factor_count;
                family_it->initial_cost += cost;
                family_sum += cost;
            } else {
                ++family_it->nonfinite_factor_count;
                family_sum_finite = false;
                phase96RecordException(
                    diagnostics, "initial_factor_error", "nonfinite",
                    family, "factor error is nonfinite");
            }
        } catch (const std::exception& e) {
            ++family_it->nonfinite_factor_count;
            family_sum_finite = false;
            phase96RecordException(diagnostics, "initial_factor_error",
                                   "exception", typeid(e).name(), e.what());
        } catch (...) {
            ++family_it->nonfinite_factor_count;
            family_sum_finite = false;
            phase96RecordException(diagnostics, "initial_factor_error",
                                   "exception", "unknown", "unknown exception");
        }
    }
    if (family_sum_finite && std::isfinite(family_sum)) {
        diagnostics.factor_family_cost_sum = family_sum;
    }

    std::shared_ptr<gtsam::GaussianFactorGraph> linearized;
    try {
        linearized = graph.linearize(initial);
        diagnostics.initial_linearization_observed = true;
    } catch (const std::exception& e) {
        phase96RecordException(diagnostics, "initial_linearization", "exception",
                               typeid(e).name(), e.what());
        return;
    } catch (...) {
        phase96RecordException(diagnostics, "initial_linearization", "exception",
                               "unknown", "unknown exception");
        return;
    }

    for (std::size_t index = 0; index < graph.size(); ++index) {
        const auto nonlinear_factor = graph[index];
        if (!nonlinear_factor || index >= linearized->size()) continue;
        const auto gaussian_factor = linearized->at(index);
        if (!gaussian_factor) continue;
        const std::string family = phase96FactorFamily(*nonlinear_factor);
        auto jacobian =
            std::dynamic_pointer_cast<gtsam::JacobianFactor>(gaussian_factor);
        if (!jacobian) {
            try {
                // The pinned GTSAM linearizer normally returns JacobianFactor
                // instances.  Preserve visibility if a future factor emits
                // a HessianFactor by using GTSAM's read-only conversion; the
                // converted factor is never passed back to the optimizer.
                jacobian = std::make_shared<gtsam::JacobianFactor>(
                    *gaussian_factor);
            } catch (const std::exception& e) {
                phase96RecordException(
                    diagnostics, "initial_linearization", "unsupported_factor",
                    typeid(*gaussian_factor).name(), e.what());
            } catch (...) {
                phase96RecordException(
                    diagnostics, "initial_linearization", "unsupported_factor",
                    typeid(*gaussian_factor).name(),
                    "unable to convert linearized factor to Jacobian");
            }
        }
        if (!jacobian) {
            phase96RecordException(
                diagnostics, "initial_linearization", "unsupported_factor",
                typeid(*gaussian_factor).name(),
                "linearized factor is not a JacobianFactor");
            continue;
        }
        const Eigen::VectorXd b = jacobian->getb();
        for (auto key_it = jacobian->begin(); key_it != jacobian->end();
             ++key_it) {
            const std::string bucket = phase96VariableBucket(*key_it);
            auto norm_it = std::find_if(
                diagnostics.variable_family_norms.begin(),
                diagnostics.variable_family_norms.end(),
                [&](const auto& value) {
                    return value.family == family &&
                           value.variable_bucket == bucket;
                });
            if (norm_it == diagnostics.variable_family_norms.end()) {
                diagnostics.variable_family_norms.push_back({});
                norm_it = std::prev(diagnostics.variable_family_norms.end());
                norm_it->family = family;
                norm_it->variable_bucket = bucket;
            }
            ++norm_it->contribution_count;
            const Eigen::MatrixXd A = jacobian->getA(key_it);
            const bool finite = A.allFinite() && b.allFinite();
            if (!finite) {
                ++norm_it->nonfinite_contribution_count;
                phase96RecordException(
                    diagnostics, "initial_linearization", "nonfinite",
                    family + ":" + bucket,
                    "Jacobian block or right-hand side is nonfinite");
                continue;
            }
            ++norm_it->finite_contribution_count;
            const Eigen::VectorXd gradient = -A.transpose() * b;
            const Eigen::VectorXd normal_diagonal =
                A.array().square().colwise().sum().transpose();
            if (!gradient.allFinite() || !normal_diagonal.allFinite()) {
                ++norm_it->nonfinite_contribution_count;
                phase96RecordException(
                    diagnostics, "initial_linearization", "nonfinite",
                    family + ":" + bucket,
                    "gradient or normal diagonal is nonfinite");
                continue;
            }
            norm_it->gradient_l2_norm = std::sqrt(
                norm_it->gradient_l2_norm * norm_it->gradient_l2_norm +
                gradient.squaredNorm());
            norm_it->normal_diagonal_l2_norm = std::sqrt(
                norm_it->normal_diagonal_l2_norm *
                    norm_it->normal_diagonal_l2_norm +
                normal_diagonal.squaredNorm());
            for (const double value : normal_diagonal) {
                if (!std::isfinite(norm_it->normal_diagonal_min) ||
                    value < norm_it->normal_diagonal_min) {
                    norm_it->normal_diagonal_min = value;
                }
                if (!std::isfinite(norm_it->normal_diagonal_max) ||
                    value > norm_it->normal_diagonal_max) {
                    norm_it->normal_diagonal_max = value;
                }
            }
        }
    }
}

// Phase97 singular-system diagnostics are deliberately a separate collector
// from Phase96.  It observes the exact graph and Values already handed to the
// pinned optimizer, plus one initial linearization.  No diagnostic ordering,
// damped system, factor, Value, or optimizer parameter is installed anywhere
// in this routine.
constexpr double kPhase97NearZeroNormalDiagonal = 1.0e-12;
constexpr std::size_t kPhase97DenseRankElementLimit = 4000000U;

inline FGOProcessor::FGOPhase97KeyReference phase97KeyReference(
    gtsam::Key key) {
    const gtsam::Symbol symbol(key);
    FGOProcessor::FGOPhase97KeyReference reference;
    reference.numeric_key = static_cast<std::uint64_t>(key);
    reference.symbol_character = static_cast<char>(symbol.chr());
    reference.symbol_index = static_cast<std::uint64_t>(symbol.index());
    return reference;
}

inline FGOProcessor::FGOPhase97KeyReference phase97NearbyVariableReference(
    const gtsam::IndeterminantLinearSystemException& exception) {
    return phase97KeyReference(exception.nearbyVariable());
}

inline void phase97RecordException(
    FGOProcessor::FGOPhase97MainDiagnostics& diagnostics,
    const std::string& stage, const std::string& classification,
    const std::string& type, const std::string& message) {
    for (auto& existing : diagnostics.exceptions) {
        if (existing.stage == stage &&
            existing.classification == classification &&
            existing.type == type && existing.message == message) {
            ++existing.count;
            return;
        }
    }
    diagnostics.exceptions.push_back({stage, classification, type, message, 1});
}

inline void phase97RecordNearbyVariable(
    FGOProcessor::FGOPhase97MainDiagnostics& diagnostics,
    const FGOProcessor::FGOPhase97KeyReference& reference) {
    const auto duplicate = std::find_if(
        diagnostics.nearby_variables.begin(), diagnostics.nearby_variables.end(),
        [&](const auto& existing) {
            return existing.numeric_key == reference.numeric_key;
        });
    if (duplicate == diagnostics.nearby_variables.end()) {
        diagnostics.nearby_variables.push_back(reference);
    }
}

inline void phase97RecordKeyOnce(
    std::vector<FGOProcessor::FGOPhase97KeyReference>& keys,
    const FGOProcessor::FGOPhase97KeyReference& reference) {
    const auto duplicate = std::find_if(
        keys.begin(), keys.end(), [&](const auto& existing) {
            return existing.numeric_key == reference.numeric_key;
        });
    if (duplicate == keys.end()) keys.push_back(reference);
}

inline void collectPhase97InitialGraphDiagnostics(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial,
    FGOProcessor::FGOPhase97MainDiagnostics& diagnostics) {
    using KeyDiagnostics = FGOProcessor::FGOPhase97KeyDiagnostics;
    using FactorDiagnostics = FGOProcessor::FGOPhase97FactorDiagnostics;

    diagnostics.graph_observed = true;
    diagnostics.graph_factor_count = graph.size();
    diagnostics.graph_value_count = initial.size();
    diagnostics.graph_value_dimension = initial.dim();
    diagnostics.near_zero_threshold = kPhase97NearZeroNormalDiagonal;
    double factor_error_sum = 0.0;
    bool factor_error_sum_finite = true;

    const auto initial_keys = initial.keys();
    std::map<gtsam::Key, std::size_t> key_indices;
    std::vector<std::size_t> parent;
    auto add_key = [&](gtsam::Key key) {
        const auto found = key_indices.find(key);
        if (found != key_indices.end()) return found->second;
        const std::size_t index = diagnostics.keys.size();
        key_indices.emplace(key, index);
        parent.push_back(index);
        diagnostics.keys.emplace_back();
        KeyDiagnostics& record = diagnostics.keys.back();
        record.key = phase97KeyReference(key);
        record.variable_bucket = phase96VariableBucket(key);
        if (initial.exists(key)) {
            const gtsam::Value& value = initial.at(key);
            record.value_present = true;
            record.value_dimension = value.dim();
            record.value_type = gtsam::demangle(typeid(value).name());
            record.gradient.assign(record.value_dimension, 0.0);
            record.normal_diagonal.assign(record.value_dimension, 0.0);
        } else {
            record.value_type = "missing-from-initial-values";
        }
        return index;
    };
    for (const gtsam::Key key : initial_keys) add_key(key);

    auto union_keys = [&parent](std::size_t left, std::size_t right) {
        auto root = [&parent](std::size_t value) {
            while (parent[value] != value) {
                parent[value] = parent[parent[value]];
                value = parent[value];
            }
            return value;
        };
        const std::size_t left_root = root(left);
        const std::size_t right_root = root(right);
        if (left_root != right_root) parent[right_root] = left_root;
    };

    for (std::size_t graph_index = 0; graph_index < graph.size();
         ++graph_index) {
        const auto factor = graph[graph_index];
        FactorDiagnostics record;
        record.graph_index = graph_index;
        if (!factor) {
            ++diagnostics.empty_factor_count;
            record.concrete_factor_type = "null";
            record.family = "empty";
            diagnostics.factors.push_back(std::move(record));
            continue;
        }
        record.concrete_factor_type =
            gtsam::demangle(typeid(*factor).name());
        record.family = phase96FactorFamily(*factor);
        record.is_prior_or_anchor = record.family == "priors" ||
                                    record.concrete_factor_type.find("Prior") !=
                                        std::string::npos ||
                                    record.concrete_factor_type.find(
                                        "Anchor") != std::string::npos;
        try {
            record.factor_dimension = factor->dim();
        } catch (const std::exception& exception) {
            phase97RecordException(diagnostics, "factor_dimension", "exception",
                                   typeid(exception).name(), exception.what());
        } catch (...) {
            phase97RecordException(diagnostics, "factor_dimension", "exception",
                                   "unknown", "unknown exception");
        }

        std::vector<std::size_t> factor_key_indices;
        std::set<gtsam::Key> unique_factor_keys;
        for (const gtsam::Key key : factor->keys()) {
            const auto insertion = unique_factor_keys.insert(key);
            if (!insertion.second) {
                ++diagnostics.duplicate_key_reference_count;
                phase97RecordKeyOnce(diagnostics.duplicate_key_reference_keys,
                                     phase97KeyReference(key));
            }
            const std::size_t key_index = add_key(key);
            record.keys.push_back(phase97KeyReference(key));
            if (insertion.second) factor_key_indices.push_back(key_index);
            if (!initial.exists(key)) {
                ++diagnostics.missing_factor_key_count;
                phase97RecordKeyOnce(diagnostics.missing_factor_keys,
                                     phase97KeyReference(key));
            }
        }
        for (std::size_t i = 1; i < factor_key_indices.size(); ++i) {
            union_keys(factor_key_indices.front(), factor_key_indices[i]);
        }
        for (const std::size_t key_index : factor_key_indices) {
            KeyDiagnostics& key_record = diagnostics.keys[key_index];
            ++key_record.factor_degree;
            if (record.is_prior_or_anchor) ++key_record.prior_factor_degree;
            auto family_it = std::find_if(
                key_record.family_degrees.begin(), key_record.family_degrees.end(),
                [&](const auto& family) { return family.family == record.family; });
            if (family_it == key_record.family_degrees.end()) {
                key_record.family_degrees.push_back({record.family, 1});
            } else {
                ++family_it->factor_degree;
            }
        }
        try {
            const double error = factor->error(initial);
            record.finite_error = std::isfinite(error);
            if (!record.finite_error) {
                factor_error_sum_finite = false;
                phase97RecordException(diagnostics, "factor_error", "nonfinite",
                                       record.concrete_factor_type,
                                       "factor error is nonfinite");
            } else {
                factor_error_sum += error;
            }
        } catch (const gtsam::IndeterminantLinearSystemException& exception) {
            factor_error_sum_finite = false;
            phase97RecordNearbyVariable(
                diagnostics, phase97NearbyVariableReference(exception));
            phase97RecordException(
                diagnostics, "factor_error", "indeterminate_linear_system",
                "IndeterminantLinearSystemException", exception.what());
        } catch (const std::exception& exception) {
            const std::string message = exception.what();
            if (message.find("type") != std::string::npos ||
                message.find("Value") != std::string::npos) {
                ++diagnostics.value_type_mismatch_count;
                for (const std::size_t key_index : factor_key_indices) {
                    phase97RecordKeyOnce(
                        diagnostics.value_type_mismatch_keys,
                        diagnostics.keys[key_index].key);
                }
            }
            factor_error_sum_finite = false;
            phase97RecordException(diagnostics, "factor_error", "exception",
                                   typeid(exception).name(), message);
        } catch (...) {
            factor_error_sum_finite = false;
            phase97RecordException(diagnostics, "factor_error", "exception",
                                   "unknown", "unknown exception");
        }
        diagnostics.factors.push_back(std::move(record));
    }
    if (factor_error_sum_finite && std::isfinite(factor_error_sum)) {
        diagnostics.graph_initial_cost = factor_error_sum;
    }

    std::map<std::size_t, std::size_t> component_by_root;
    for (std::size_t index = 0; index < diagnostics.keys.size(); ++index) {
        std::size_t root = index;
        while (parent[root] != root) {
            parent[root] = parent[parent[root]];
            root = parent[root];
        }
        auto component_it = component_by_root.find(root);
        if (component_it == component_by_root.end()) {
            const std::size_t component_id = diagnostics.components.size();
            component_by_root.emplace(root, component_id);
            diagnostics.components.push_back({});
            diagnostics.components.back().component_id = component_id;
            component_it = component_by_root.find(root);
        }
        const std::size_t component_id = component_it->second;
        KeyDiagnostics& key_record = diagnostics.keys[index];
        key_record.component_id = component_id;
        auto& component = diagnostics.components[component_id];
        ++component.key_count;
        component.keys.push_back(key_record.key);
        if (key_record.value_present && key_record.factor_degree == 0U) {
            ++diagnostics.isolated_value_key_count;
        }
    }
    for (const FactorDiagnostics& factor : diagnostics.factors) {
        if (factor.keys.empty()) continue;
        const auto key_it = key_indices.find(factor.keys.front().numeric_key);
        if (key_it == key_indices.end()) continue;
        const std::size_t component_id =
            diagnostics.keys[key_it->second].component_id;
        auto& component = diagnostics.components[component_id];
        ++component.factor_count;
        component.anchored = component.anchored || factor.is_prior_or_anchor;
    }
    diagnostics.connected_component_count = diagnostics.components.size();

    std::shared_ptr<gtsam::GaussianFactorGraph> linearized;
    std::vector<std::shared_ptr<gtsam::JacobianFactor>> jacobians(graph.size());
    std::vector<std::size_t> row_offsets(graph.size(), 0U);
    std::size_t row_count = 0;
    try {
        linearized = graph.linearize(initial);
        diagnostics.initial_linearization_observed = true;
    } catch (const gtsam::IndeterminantLinearSystemException& exception) {
        phase97RecordNearbyVariable(
            diagnostics, phase97NearbyVariableReference(exception));
        phase97RecordException(
            diagnostics, "initial_linearization", "indeterminate_linear_system",
            "IndeterminantLinearSystemException", exception.what());
    } catch (const std::exception& exception) {
        phase97RecordException(diagnostics, "initial_linearization", "exception",
                               typeid(exception).name(), exception.what());
    } catch (...) {
        phase97RecordException(diagnostics, "initial_linearization", "exception",
                               "unknown", "unknown exception");
    }

    if (linearized) {
        for (std::size_t graph_index = 0; graph_index < graph.size();
             ++graph_index) {
            if (graph_index >= linearized->size()) continue;
            const auto gaussian_factor = linearized->at(graph_index);
            if (!gaussian_factor) continue;
            auto jacobian =
                std::dynamic_pointer_cast<gtsam::JacobianFactor>(gaussian_factor);
            if (!jacobian) {
                try {
                    jacobian = std::make_shared<gtsam::JacobianFactor>(
                        *gaussian_factor);
                } catch (const std::exception& exception) {
                    phase97RecordException(
                        diagnostics, "initial_linearization", "unsupported_factor",
                        typeid(*gaussian_factor).name(), exception.what());
                } catch (...) {
                    phase97RecordException(
                        diagnostics, "initial_linearization", "unsupported_factor",
                        typeid(*gaussian_factor).name(),
                        "unable to convert linearized factor to Jacobian");
                }
            }
            if (!jacobian) continue;
            jacobians[graph_index] = jacobian;
            row_offsets[graph_index] = row_count;
            row_count += jacobian->rows();

            const Eigen::VectorXd b = jacobian->getb();
            for (auto key_it = jacobian->begin(); key_it != jacobian->end();
                 ++key_it) {
                const auto index_it = key_indices.find(*key_it);
                if (index_it == key_indices.end()) {
                    ++diagnostics.missing_factor_key_count;
                    phase97RecordKeyOnce(diagnostics.missing_factor_keys,
                                         phase97KeyReference(*key_it));
                    continue;
                }
                KeyDiagnostics& key_record = diagnostics.keys[index_it->second];
                ++key_record.linearized_contribution_count;
                const Eigen::MatrixXd A = jacobian->getA(key_it);
                const bool finite = A.allFinite() && b.allFinite();
                if (!finite) {
                    ++key_record.nonfinite_linearized_contribution_count;
                    phase97RecordException(
                        diagnostics, "initial_linearization", "nonfinite",
                        phase96VariableBucket(*key_it),
                        "Jacobian block or right-hand side is nonfinite");
                    continue;
                }
                ++key_record.finite_linearized_contribution_count;
                const Eigen::VectorXd gradient = -A.transpose() * b;
                const Eigen::VectorXd normal_diagonal =
                    A.array().square().colwise().sum().transpose();
                if (key_record.value_dimension !=
                    static_cast<std::size_t>(A.cols())) {
                    ++diagnostics.value_type_mismatch_count;
                    phase97RecordKeyOnce(diagnostics.value_type_mismatch_keys,
                                         key_record.key);
                    phase97RecordException(
                        diagnostics, "initial_linearization", "dimension_mismatch",
                        phase96VariableBucket(*key_it),
                        "Jacobian block dimension differs from Values dimension");
                    continue;
                }
                if (!gradient.allFinite() || !normal_diagonal.allFinite()) {
                    ++key_record.nonfinite_linearized_contribution_count;
                    phase97RecordException(
                        diagnostics, "initial_linearization", "nonfinite",
                        phase96VariableBucket(*key_it),
                        "keyed gradient or normal diagonal is nonfinite");
                    continue;
                }
                if (key_record.gradient.size() !=
                    static_cast<std::size_t>(gradient.size())) {
                    key_record.gradient.assign(gradient.size(), 0.0);
                    key_record.normal_diagonal.assign(normal_diagonal.size(),
                                                       0.0);
                }
                for (Eigen::Index scalar = 0; scalar < gradient.size(); ++scalar) {
                    key_record.gradient[static_cast<std::size_t>(scalar)] +=
                        gradient(scalar);
                    key_record.normal_diagonal[static_cast<std::size_t>(scalar)] +=
                        normal_diagonal(scalar);
                }
            }
        }
    }

    for (auto& key_record : diagnostics.keys) {
        const bool has_nonfinite =
            key_record.nonfinite_linearized_contribution_count > 0U;
        if (!has_nonfinite && !key_record.gradient.empty() &&
            key_record.gradient.size() == key_record.normal_diagonal.size()) {
            double gradient_squared_norm = 0.0;
            double normal_squared_norm = 0.0;
            bool finite = true;
            for (std::size_t scalar = 0; scalar < key_record.gradient.size();
                 ++scalar) {
                const double gradient_value = key_record.gradient[scalar];
                const double normal_value = key_record.normal_diagonal[scalar];
                finite = finite && std::isfinite(gradient_value) &&
                         std::isfinite(normal_value);
                if (finite) {
                    gradient_squared_norm += gradient_value * gradient_value;
                    normal_squared_norm += normal_value * normal_value;
                }
                if (normal_value == 0.0) {
                    ++key_record.exact_zero_normal_diagonal_count;
                } else if (std::isfinite(normal_value) &&
                           std::abs(normal_value) <=
                               kPhase97NearZeroNormalDiagonal) {
                    ++key_record.near_zero_normal_diagonal_count;
                }
            }
            if (finite) {
                key_record.gradient_l2_norm = std::sqrt(gradient_squared_norm);
                key_record.normal_diagonal_l2_norm = std::sqrt(normal_squared_norm);
            }
        }
        const bool no_contribution =
            key_record.linearized_contribution_count == 0U;
        key_record.exact_zero_column =
            no_contribution ||
            (!key_record.normal_diagonal.empty() &&
             key_record.exact_zero_normal_diagonal_count ==
                 key_record.normal_diagonal.size());
        key_record.near_zero_column =
            key_record.near_zero_normal_diagonal_count > 0U;
        if (key_record.exact_zero_column) ++diagnostics.exact_zero_column_count;
        if (key_record.near_zero_column) ++diagnostics.near_zero_column_count;
        if (key_record.value_present && key_record.normal_diagonal.empty()) {
            key_record.normal_diagonal.assign(key_record.value_dimension, 0.0);
            key_record.exact_zero_column = true;
        }
        if (key_record.value_present && key_record.normal_diagonal_min !=
                                             key_record.normal_diagonal_min) {
            if (!key_record.normal_diagonal.empty()) {
                key_record.normal_diagonal_min =
                    *std::min_element(key_record.normal_diagonal.begin(),
                                      key_record.normal_diagonal.end());
                key_record.normal_diagonal_max =
                    *std::max_element(key_record.normal_diagonal.begin(),
                                      key_record.normal_diagonal.end());
            }
        }
    }

    FGOProcessor::FGOPhase97RankDiagnostics rank;
    rank.attempted = linearized != nullptr;
    rank.row_count = row_count;
    for (const auto& key_record : diagnostics.keys) {
        if (key_record.value_present) rank.column_count += key_record.value_dimension;
    }
    rank.threshold = kPhase97NearZeroNormalDiagonal;
    rank.method = "Eigen::ColPivHouseholderQR keyed Jacobian";
    for (const auto& key_record : diagnostics.keys) {
        if (key_record.nonfinite_linearized_contribution_count > 0U) {
            rank.finite = false;
            break;
        }
    }
    if (!linearized) {
        rank.status = "unavailable_initial_linearization_failed";
    } else if (rank.column_count == 0U) {
        rank.rank_known = true;
        rank.nullity_known = true;
        rank.status = row_count == 0U ? "empty_keyed_system" : "zero_column_system";
    } else if (rank.row_count > 0U &&
               rank.column_count >
                   kPhase97DenseRankElementLimit / rank.row_count) {
        rank.status = "rank_revealing_unavailable_due_to_size";
        rank.method = "keyed structural support only";
    } else {
        Eigen::MatrixXd keyed_jacobian =
            Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(rank.row_count),
                                  static_cast<Eigen::Index>(rank.column_count));
        std::map<gtsam::Key, std::size_t> column_offsets;
        std::size_t column_offset = 0;
        for (const auto& key : initial_keys) {
            column_offsets.emplace(key, column_offset);
            column_offset += initial.at(key).dim();
        }
        bool finite_jacobian = true;
        bool complete_jacobian = diagnostics.missing_factor_key_count == 0U;
        for (std::size_t graph_index = 0; graph_index < jacobians.size();
             ++graph_index) {
            const auto& jacobian = jacobians[graph_index];
            if (!jacobian) {
                complete_jacobian = false;
                continue;
            }
            const std::size_t row_offset = row_offsets[graph_index];
            const Eigen::VectorXd b = jacobian->getb();
            if (!b.allFinite()) finite_jacobian = false;
            for (auto key_it = jacobian->begin(); key_it != jacobian->end();
                 ++key_it) {
                const Eigen::MatrixXd A = jacobian->getA(key_it);
                if (!A.allFinite()) finite_jacobian = false;
                const auto column_it = column_offsets.find(*key_it);
                if (column_it == column_offsets.end() ||
                    row_offset + static_cast<std::size_t>(A.rows()) >
                        rank.row_count ||
                    column_it->second + static_cast<std::size_t>(A.cols()) >
                        rank.column_count) {
                    complete_jacobian = false;
                    continue;
                }
                keyed_jacobian.block(
                    static_cast<Eigen::Index>(row_offset),
                    static_cast<Eigen::Index>(column_it->second), A.rows(),
                    A.cols()) += A;
            }
        }
        rank.finite = finite_jacobian;
        if (!finite_jacobian) {
            rank.status = "rank_revealing_blocked_nonfinite_jacobian";
        } else {
            if (rank.row_count == 0U) {
                rank.rank = 0U;
            } else {
                Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(keyed_jacobian);
                qr.setThreshold(kPhase97NearZeroNormalDiagonal);
                rank.rank = static_cast<std::size_t>(qr.rank());
            }
            rank.rank_known = true;
            rank.nullity_known = true;
            rank.nullity = rank.column_count - rank.rank;
            rank.status = rank.nullity > 0U ? "rank_deficient" :
                                               "full_column_rank";
            if (!complete_jacobian) rank.status += "_partial_support";
        }
    }

    std::set<std::uint64_t> attributed_keys;
    for (const auto& key_record : diagnostics.keys) {
        if (key_record.exact_zero_column) {
            rank.nullspace_attribution.push_back(key_record.key);
            attributed_keys.insert(key_record.key.numeric_key);
        }
    }
    for (const auto& component : diagnostics.components) {
        if (component.anchored) continue;
        for (const auto& key : component.keys) {
            if (attributed_keys.insert(key.numeric_key).second) {
                rank.nullspace_attribution.push_back(key);
            }
        }
    }
    diagnostics.rank_decompositions.push_back(std::move(rank));
    diagnostics.nearby_variable_capture_status =
        diagnostics.nearby_variables.empty()
            ? "nearby_variable_unavailable"
            : "captured_from_indeterminant_exception";
    diagnostics.diagnostic_complete = diagnostics.initial_linearization_observed;
}

// Plain-Euclidean geometric range and its 1x3 Jacobian w.r.t. the receiver.
//
// NOTE ON SAGNAC: the native backend earth-rotation-corrects each satellite
// position at build time (earthRotationCorrected, using the seed position) and
// then forms the pseudorange residual with a *plain* Euclidean norm. GTSAM's
// gtsam::gnss::geodist would apply its OWN first-order Sagnac correction on top
// of those already-corrected positions -- a double correction that biases the
// undifferenced-pseudorange position solution by ~1 m on multi-GNSS data (it
// mostly cancels in the double-difference, which is why the stock DD factors
// still reach cm parity). So these undifferenced factors use a plain norm to
// match the native backend exactly.
inline double plainRange(const Point3& satellite, const Point3& receiver,
                         gtsam::Matrix13* H_receiver) {
    const Point3 delta = receiver - satellite;
    const double range = delta.norm();
    if (H_receiver) {
        if (range > 0.0) {
            *H_receiver = (delta / range).transpose();
        } else {
            H_receiver->setZero();
        }
    }
    return range;
}

// Undifferenced pseudorange factor with a global inter-system bias node.
//   error = ||rcv - sat|| + clockTerm(baseClock + isb) - measuredPseudorange
// Base clock is per epoch; isb is one global (time-constant) node per non-GPS
// constellation -- mirroring the native backend's per-epoch clock + shared
// per-constellation bias column.
class PseudorangeFactorISB : public gtsam::NoiseModelFactorN<Point3, double, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, double, double>;
    using Base::evaluateError;
    PseudorangeFactorISB(gtsam::Key position, gtsam::Key base_clock, gtsam::Key isb,
                         double measured_pseudorange, const Point3& satellite_position,
                         const gtsam::SharedNoiseModel& model,
                         bool meter_state = false)
        : Base(model, position, base_clock, isb),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(const Point3& position, const double& base_clock,
                                const double& isb, gtsam::OptionalMatrixType H_position,
                                gtsam::OptionalMatrixType H_base_clock,
                                gtsam::OptionalMatrixType H_isb) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(satellite_position_, position, H_position ? &H_range : nullptr);
        const double error = range +
                             clockStateTerm(base_clock + isb, meter_state_) -
                             measurement_;
        if (H_position) {
            *H_position = H_range;
        }
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_isb) {
            *H_isb =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

// GPS/base-group undifferenced pseudorange factor (no ISB node).
//   error = ||rcv - sat|| + clockTerm(baseClock) - measuredPseudorange
class PseudorangeFactorPlain : public gtsam::NoiseModelFactorN<Point3, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, double>;
    using Base::evaluateError;
    PseudorangeFactorPlain(gtsam::Key position, gtsam::Key base_clock,
                           double measured_pseudorange, const Point3& satellite_position,
                           const gtsam::SharedNoiseModel& model,
                           bool meter_state = false)
        : Base(model, position, base_clock),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(const Point3& position, const double& base_clock,
                                gtsam::OptionalMatrixType H_position,
                                gtsam::OptionalMatrixType H_base_clock) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(satellite_position_, position, H_position ? &H_range : nullptr);
        const double error = range + clockStateTerm(base_clock, meter_state_) -
                             measurement_;
        if (H_position) {
            *H_position = H_range;
        }
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

// Phase101 equivalent of the official PseudorangeFactor_XC.  The position is
// kept as the native Point3 state, while the receiver clock/ISB contribution
// is selected from the same seven-component epoch-local C vector used by the
// CCDD factor.  `component` is the sysfreq2sigtype.m result and is validated
// before insertion by the backend.
class PseudorangeFactorSourceClock
    : public gtsam::NoiseModelFactorN<Point3, gtsam::Vector> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    int component_ = -1;

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, gtsam::Vector>;
    using Base::evaluateError;

    PseudorangeFactorSourceClock(
        gtsam::Key position, gtsam::Key clock,
        double measured_pseudorange, const Point3& satellite_position,
        int component, const gtsam::SharedNoiseModel& model)
        : Base(model, position, clock),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          component_(component) {}

    gtsam::Vector evaluateError(
        const Point3& position, const gtsam::Vector& clock,
        gtsam::OptionalMatrixType H_position,
        gtsam::OptionalMatrixType H_clock) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, position, H_position ? &H_range : nullptr);
        const gtsam::Vector hc = sourceClockComponentJacobian(component_);
        if (H_position) *H_position = H_range;
        if (H_clock) *H_clock = hc.transpose();
        if (!finiteSourceClockVector(clock) || component_ < 0) {
            return gtsam::Vector::Constant(
                1, std::numeric_limits<double>::quiet_NaN());
        }
        return (gtsam::Vector(1) << range + hc.dot(clock) - measurement_)
            .finished();
    }
};

// Secondary-signal analogue of PseudorangeFactorPlain.  The third state is a
// static receiver signal bias in metres; it is deliberately separate from
// the epoch clock and from the constellation ISB so a primary/secondary pair
// can identify a receiver inter-frequency delay without silently changing the
// clock-group contract.
class PseudorangeFactorPlainSignalBias
    : public gtsam::NoiseModelFactorN<Point3, double, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, double, double>;
    using Base::evaluateError;
    PseudorangeFactorPlainSignalBias(
        gtsam::Key position, gtsam::Key base_clock, gtsam::Key signal_bias,
        double measured_pseudorange, const Point3& satellite_position,
        const gtsam::SharedNoiseModel& model, bool meter_state = false)
        : Base(model, position, base_clock, signal_bias),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Point3& position, const double& base_clock,
        const double& signal_bias, gtsam::OptionalMatrixType H_position,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_signal_bias) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, position, H_position ? &H_range : nullptr);
        const double error = range + clockStateTerm(base_clock, meter_state_) +
                             signal_bias - measurement_;
        if (H_position) *H_position = H_range;
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_signal_bias) {
            *H_signal_bias = (gtsam::Matrix(1, 1) << 1.0).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

// Secondary-signal factor with both the constellation ISB (seconds) and the
// receiver signal bias (metres).
class PseudorangeFactorISBSignalBias
    : public gtsam::NoiseModelFactorN<Point3, double, double, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, double, double, double>;
    using Base::evaluateError;
    PseudorangeFactorISBSignalBias(
        gtsam::Key position, gtsam::Key base_clock, gtsam::Key isb,
        gtsam::Key signal_bias, double measured_pseudorange,
        const Point3& satellite_position, const gtsam::SharedNoiseModel& model,
        bool meter_state = false)
        : Base(model, position, base_clock, isb, signal_bias),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Point3& position, const double& base_clock, const double& isb,
        const double& signal_bias, gtsam::OptionalMatrixType H_position,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_isb,
        gtsam::OptionalMatrixType H_signal_bias) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, position, H_position ? &H_range : nullptr);
        const double error = range + clockStateTerm(base_clock + isb, meter_state_) +
                             signal_bias - measurement_;
        if (H_position) *H_position = H_range;
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_isb) {
            *H_isb =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_signal_bias) {
            *H_signal_bias = (gtsam::Matrix(1, 1) << 1.0).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

// Opt-in residual-ionosphere variants of the undifferenced pseudorange
// factors.  The last scalar is a shared vertical L1 residual state in metres;
// its coefficient is supplied by the raw problem builder as
// mapping(elevation)*(f_L1/f_signal)^2.  Keeping these as separate factor
// types makes the additional state key explicit and prevents the Phase11
// static receiver-bias state from being accidentally reused as ionosphere.
class PseudorangeFactorPlainResidualIonosphere
    : public gtsam::NoiseModelFactorN<Point3, double, double> {
    double measurement_ = 0.0;
    double ionosphere_coefficient_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, double, double>;
    using Base::evaluateError;
    PseudorangeFactorPlainResidualIonosphere(
        gtsam::Key position, gtsam::Key base_clock, gtsam::Key ionosphere,
        double measured_pseudorange, const Point3& satellite_position,
        double ionosphere_coefficient, const gtsam::SharedNoiseModel& model,
        bool meter_state = false)
        : Base(model, position, base_clock, ionosphere),
          measurement_(measured_pseudorange),
          ionosphere_coefficient_(ionosphere_coefficient),
          satellite_position_(satellite_position),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Point3& position, const double& base_clock,
        const double& ionosphere, gtsam::OptionalMatrixType H_position,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_ionosphere) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, position, H_position ? &H_range : nullptr);
        const double error = range + clockStateTerm(base_clock, meter_state_) +
                             ionosphere_coefficient_ * ionosphere - measurement_;
        if (H_position) *H_position = H_range;
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_ionosphere) {
            *H_ionosphere =
                (gtsam::Matrix(1, 1) << ionosphere_coefficient_).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

class PseudorangeFactorISBResidualIonosphere
    : public gtsam::NoiseModelFactorN<Point3, double, double, double> {
    double measurement_ = 0.0;
    double ionosphere_coefficient_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, double, double, double>;
    using Base::evaluateError;
    PseudorangeFactorISBResidualIonosphere(
        gtsam::Key position, gtsam::Key base_clock, gtsam::Key isb,
        gtsam::Key ionosphere, double measured_pseudorange,
        const Point3& satellite_position, double ionosphere_coefficient,
        const gtsam::SharedNoiseModel& model, bool meter_state = false)
        : Base(model, position, base_clock, isb, ionosphere),
          measurement_(measured_pseudorange),
          ionosphere_coefficient_(ionosphere_coefficient),
          satellite_position_(satellite_position),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Point3& position, const double& base_clock, const double& isb,
        const double& ionosphere, gtsam::OptionalMatrixType H_position,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_isb,
        gtsam::OptionalMatrixType H_ionosphere) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, position, H_position ? &H_range : nullptr);
        const double error = range + clockStateTerm(base_clock + isb, meter_state_) +
                             ionosphere_coefficient_ * ionosphere - measurement_;
        if (H_position) *H_position = H_range;
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_isb) {
            *H_isb =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_ionosphere) {
            *H_ionosphere =
                (gtsam::Matrix(1, 1) << ionosphere_coefficient_).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

class PseudorangeFactorPlainSignalBiasResidualIonosphere
    : public gtsam::NoiseModelFactorN<Point3, double, double, double> {
    double measurement_ = 0.0;
    double ionosphere_coefficient_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, double, double, double>;
    using Base::evaluateError;
    PseudorangeFactorPlainSignalBiasResidualIonosphere(
        gtsam::Key position, gtsam::Key base_clock, gtsam::Key signal_bias,
        gtsam::Key ionosphere, double measured_pseudorange,
        const Point3& satellite_position, double ionosphere_coefficient,
        const gtsam::SharedNoiseModel& model, bool meter_state = false)
        : Base(model, position, base_clock, signal_bias, ionosphere),
          measurement_(measured_pseudorange),
          ionosphere_coefficient_(ionosphere_coefficient),
          satellite_position_(satellite_position),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Point3& position, const double& base_clock,
        const double& signal_bias, const double& ionosphere,
        gtsam::OptionalMatrixType H_position,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_signal_bias,
        gtsam::OptionalMatrixType H_ionosphere) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, position, H_position ? &H_range : nullptr);
        const double error = range + clockStateTerm(base_clock, meter_state_) +
                             signal_bias + ionosphere_coefficient_ * ionosphere -
                             measurement_;
        if (H_position) *H_position = H_range;
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_signal_bias) {
            *H_signal_bias = (gtsam::Matrix(1, 1) << 1.0).finished();
        }
        if (H_ionosphere) {
            *H_ionosphere =
                (gtsam::Matrix(1, 1) << ionosphere_coefficient_).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

class PseudorangeFactorISBSignalBiasResidualIonosphere
    : public gtsam::NoiseModelFactorN<Point3, double, double, double, double> {
    double measurement_ = 0.0;
    double ionosphere_coefficient_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    bool meter_state_ = false;

 public:
    using Base =
        gtsam::NoiseModelFactorN<Point3, double, double, double, double>;
    using Base::evaluateError;
    PseudorangeFactorISBSignalBiasResidualIonosphere(
        gtsam::Key position, gtsam::Key base_clock, gtsam::Key isb,
        gtsam::Key signal_bias, gtsam::Key ionosphere,
        double measured_pseudorange, const Point3& satellite_position,
        double ionosphere_coefficient, const gtsam::SharedNoiseModel& model,
        bool meter_state = false)
        : Base(model, position, base_clock, isb, signal_bias, ionosphere),
          measurement_(measured_pseudorange),
          ionosphere_coefficient_(ionosphere_coefficient),
          satellite_position_(satellite_position),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Point3& position, const double& base_clock, const double& isb,
        const double& signal_bias, const double& ionosphere,
        gtsam::OptionalMatrixType H_position,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_isb,
        gtsam::OptionalMatrixType H_signal_bias,
        gtsam::OptionalMatrixType H_ionosphere) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, position, H_position ? &H_range : nullptr);
        const double error = range + clockStateTerm(base_clock + isb, meter_state_) +
                             signal_bias + ionosphere_coefficient_ * ionosphere -
                             measurement_;
        if (H_position) *H_position = H_range;
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_isb) {
            *H_isb =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_signal_bias) {
            *H_signal_bias = (gtsam::Matrix(1, 1) << 1.0).finished();
        }
        if (H_ionosphere) {
            *H_ionosphere =
                (gtsam::Matrix(1, 1) << ionosphere_coefficient_).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

// --- Milestone 2a: Pose3 + lever-arm plumbing (docs/gtsam_backend_design.md) ---
//
// Undifferenced plain-norm pseudorange factors are Pose3 analogs of
// PseudorangeFactorPlain/ISB above: same "no double Sagnac correction"
// reasoning (the satellite position is already earth-rotation-corrected by
// the native builder), just evaluated at the ANTENNA position derived from
// a body Pose3 + lever arm via gtsam::gnss::LeverArm (antenna_ecef =
// pose.translation() + pose.rotation() * leverArm, exactly the convention
// used by the stock '...FactorArm' factors and by this project's own
// Stage-1 ESKF, see fusion_measurement.cpp). GTSAM's navigation module has
// no stock plain-norm Arm factor (only geodist-based ones), hence these.

class PseudorangeFactorPlainArm : public gtsam::NoiseModelFactorN<Pose3, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    gtsam::gnss::LeverArm arm_;
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double>;
    using Base::evaluateError;
    PseudorangeFactorPlainArm(gtsam::Key pose, gtsam::Key base_clock,
                              double measured_pseudorange, const Point3& satellite_position,
                              const gtsam::gnss::LeverArm& arm,
                              const gtsam::SharedNoiseModel& model,
                              bool meter_state = false)
        : Base(model, pose, base_clock),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          arm_(arm),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(const Pose3& pose, const double& base_clock,
                                gtsam::OptionalMatrixType H_pose,
                                gtsam::OptionalMatrixType H_base_clock) const override {
        gtsam::gnss::LeverArm::PoseFrame frame;
        const Point3 antenna = arm_.antennaPosition(pose, H_pose ? &frame : nullptr);
        gtsam::Matrix13 H_range;
        const double range = plainRange(satellite_position_, antenna, H_pose ? &H_range : nullptr);
        const double error = range + clockStateTerm(base_clock, meter_state_) -
                             measurement_;
        if (H_pose) {
            *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        }
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

// Pose3/lever-arm counterpart of PseudorangeFactorSourceClock.  It is used by
// the Phase101 main meter-state graph so the same epoch-local C key and
// component mapping survive the GNSS-first -> IMU handoff.
class PseudorangeFactorSourceClockArm
    : public gtsam::NoiseModelFactorN<Pose3, gtsam::Vector> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    gtsam::gnss::LeverArm arm_;
    int component_ = -1;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, gtsam::Vector>;
    using Base::evaluateError;

    PseudorangeFactorSourceClockArm(
        gtsam::Key pose, gtsam::Key clock,
        double measured_pseudorange, const Point3& satellite_position,
        const gtsam::gnss::LeverArm& arm, int component,
        const gtsam::SharedNoiseModel& model)
        : Base(model, pose, clock),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          arm_(arm),
          component_(component) {}

    gtsam::Vector evaluateError(
        const Pose3& pose, const gtsam::Vector& clock,
        gtsam::OptionalMatrixType H_pose,
        gtsam::OptionalMatrixType H_clock) const override {
        gtsam::gnss::LeverArm::PoseFrame frame;
        const Point3 antenna = arm_.antennaPosition(
            pose, H_pose ? &frame : nullptr);
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, antenna, H_pose ? &H_range : nullptr);
        const gtsam::Vector hc = sourceClockComponentJacobian(component_);
        if (H_pose) *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        if (H_clock) *H_clock = hc.transpose();
        if (!finiteSourceClockVector(clock) || component_ < 0) {
            return gtsam::Vector::Constant(
                1, std::numeric_limits<double>::quiet_NaN());
        }
        return (gtsam::Vector(1) << range + hc.dot(clock) - measurement_)
            .finished();
    }
};

class PseudorangeFactorISBArm : public gtsam::NoiseModelFactorN<Pose3, double, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    gtsam::gnss::LeverArm arm_;
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double, double>;
    using Base::evaluateError;
    PseudorangeFactorISBArm(gtsam::Key pose, gtsam::Key base_clock, gtsam::Key isb,
                            double measured_pseudorange, const Point3& satellite_position,
                            const gtsam::gnss::LeverArm& arm,
                            const gtsam::SharedNoiseModel& model,
                            bool meter_state = false)
        : Base(model, pose, base_clock, isb),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          arm_(arm),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(const Pose3& pose, const double& base_clock, const double& isb,
                                gtsam::OptionalMatrixType H_pose,
                                gtsam::OptionalMatrixType H_base_clock,
                                gtsam::OptionalMatrixType H_isb) const override {
        gtsam::gnss::LeverArm::PoseFrame frame;
        const Point3 antenna = arm_.antennaPosition(pose, H_pose ? &frame : nullptr);
        gtsam::Matrix13 H_range;
        const double range = plainRange(satellite_position_, antenna, H_pose ? &H_range : nullptr);
        const double error = range +
                             clockStateTerm(base_clock + isb, meter_state_) -
                             measurement_;
        if (H_pose) {
            *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        }
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_isb) {
            *H_isb =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

class PseudorangeFactorPlainSignalBiasArm
    : public gtsam::NoiseModelFactorN<Pose3, double, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    gtsam::gnss::LeverArm arm_;
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double, double>;
    using Base::evaluateError;
    PseudorangeFactorPlainSignalBiasArm(
        gtsam::Key pose, gtsam::Key base_clock, gtsam::Key signal_bias,
        double measured_pseudorange, const Point3& satellite_position,
        const gtsam::gnss::LeverArm& arm, const gtsam::SharedNoiseModel& model,
        bool meter_state = false)
        : Base(model, pose, base_clock, signal_bias),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          arm_(arm),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Pose3& pose, const double& base_clock, const double& signal_bias,
        gtsam::OptionalMatrixType H_pose,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_signal_bias) const override {
        gtsam::gnss::LeverArm::PoseFrame frame;
        const Point3 antenna = arm_.antennaPosition(
            pose, H_pose ? &frame : nullptr);
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, antenna, H_pose ? &H_range : nullptr);
        const double error = range + clockStateTerm(base_clock, meter_state_) +
                             signal_bias - measurement_;
        if (H_pose) *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_signal_bias) {
            *H_signal_bias = (gtsam::Matrix(1, 1) << 1.0).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

class PseudorangeFactorISBSignalBiasArm
    : public gtsam::NoiseModelFactorN<Pose3, double, double, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    gtsam::gnss::LeverArm arm_;
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double, double, double>;
    using Base::evaluateError;
    PseudorangeFactorISBSignalBiasArm(
        gtsam::Key pose, gtsam::Key base_clock, gtsam::Key isb,
        gtsam::Key signal_bias, double measured_pseudorange,
        const Point3& satellite_position, const gtsam::gnss::LeverArm& arm,
        const gtsam::SharedNoiseModel& model, bool meter_state = false)
        : Base(model, pose, base_clock, isb, signal_bias),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          arm_(arm),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Pose3& pose, const double& base_clock, const double& isb,
        const double& signal_bias, gtsam::OptionalMatrixType H_pose,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_isb,
        gtsam::OptionalMatrixType H_signal_bias) const override {
        gtsam::gnss::LeverArm::PoseFrame frame;
        const Point3 antenna = arm_.antennaPosition(
            pose, H_pose ? &frame : nullptr);
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, antenna, H_pose ? &H_range : nullptr);
        const double error = range +
                             clockStateTerm(base_clock + isb, meter_state_) +
                             signal_bias - measurement_;
        if (H_pose) *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_isb) {
            *H_isb =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_signal_bias) {
            *H_signal_bias = (gtsam::Matrix(1, 1) << 1.0).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

class PseudorangeFactorPlainResidualIonosphereArm
    : public gtsam::NoiseModelFactorN<Pose3, double, double> {
    double measurement_ = 0.0;
    double ionosphere_coefficient_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    gtsam::gnss::LeverArm arm_;
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double, double>;
    using Base::evaluateError;
    PseudorangeFactorPlainResidualIonosphereArm(
        gtsam::Key pose, gtsam::Key base_clock, gtsam::Key ionosphere,
        double measured_pseudorange, const Point3& satellite_position,
        const gtsam::gnss::LeverArm& arm, double ionosphere_coefficient,
        const gtsam::SharedNoiseModel& model, bool meter_state = false)
        : Base(model, pose, base_clock, ionosphere),
          measurement_(measured_pseudorange),
          ionosphere_coefficient_(ionosphere_coefficient),
          satellite_position_(satellite_position),
          arm_(arm),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Pose3& pose, const double& base_clock,
        const double& ionosphere, gtsam::OptionalMatrixType H_pose,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_ionosphere) const override {
        gtsam::gnss::LeverArm::PoseFrame frame;
        const Point3 antenna = arm_.antennaPosition(
            pose, H_pose ? &frame : nullptr);
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, antenna, H_pose ? &H_range : nullptr);
        const double error = range + clockStateTerm(base_clock, meter_state_) +
                             ionosphere_coefficient_ * ionosphere - measurement_;
        if (H_pose) *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_ionosphere) {
            *H_ionosphere =
                (gtsam::Matrix(1, 1) << ionosphere_coefficient_).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

class PseudorangeFactorISBResidualIonosphereArm
    : public gtsam::NoiseModelFactorN<Pose3, double, double, double> {
    double measurement_ = 0.0;
    double ionosphere_coefficient_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    gtsam::gnss::LeverArm arm_;
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double, double, double>;
    using Base::evaluateError;
    PseudorangeFactorISBResidualIonosphereArm(
        gtsam::Key pose, gtsam::Key base_clock, gtsam::Key isb,
        gtsam::Key ionosphere, double measured_pseudorange,
        const Point3& satellite_position, const gtsam::gnss::LeverArm& arm,
        double ionosphere_coefficient, const gtsam::SharedNoiseModel& model,
        bool meter_state = false)
        : Base(model, pose, base_clock, isb, ionosphere),
          measurement_(measured_pseudorange),
          ionosphere_coefficient_(ionosphere_coefficient),
          satellite_position_(satellite_position),
          arm_(arm),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Pose3& pose, const double& base_clock, const double& isb,
        const double& ionosphere, gtsam::OptionalMatrixType H_pose,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_isb,
        gtsam::OptionalMatrixType H_ionosphere) const override {
        gtsam::gnss::LeverArm::PoseFrame frame;
        const Point3 antenna = arm_.antennaPosition(
            pose, H_pose ? &frame : nullptr);
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, antenna, H_pose ? &H_range : nullptr);
        const double error = range +
                             clockStateTerm(base_clock + isb, meter_state_) +
                             ionosphere_coefficient_ * ionosphere - measurement_;
        if (H_pose) *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_isb) {
            *H_isb =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_ionosphere) {
            *H_ionosphere =
                (gtsam::Matrix(1, 1) << ionosphere_coefficient_).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

class PseudorangeFactorPlainSignalBiasResidualIonosphereArm
    : public gtsam::NoiseModelFactorN<Pose3, double, double, double> {
    double measurement_ = 0.0;
    double ionosphere_coefficient_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    gtsam::gnss::LeverArm arm_;
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double, double, double>;
    using Base::evaluateError;
    PseudorangeFactorPlainSignalBiasResidualIonosphereArm(
        gtsam::Key pose, gtsam::Key base_clock, gtsam::Key signal_bias,
        gtsam::Key ionosphere, double measured_pseudorange,
        const Point3& satellite_position, const gtsam::gnss::LeverArm& arm,
        double ionosphere_coefficient, const gtsam::SharedNoiseModel& model,
        bool meter_state = false)
        : Base(model, pose, base_clock, signal_bias, ionosphere),
          measurement_(measured_pseudorange),
          ionosphere_coefficient_(ionosphere_coefficient),
          satellite_position_(satellite_position),
          arm_(arm),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Pose3& pose, const double& base_clock,
        const double& signal_bias, const double& ionosphere,
        gtsam::OptionalMatrixType H_pose,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_signal_bias,
        gtsam::OptionalMatrixType H_ionosphere) const override {
        gtsam::gnss::LeverArm::PoseFrame frame;
        const Point3 antenna = arm_.antennaPosition(
            pose, H_pose ? &frame : nullptr);
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, antenna, H_pose ? &H_range : nullptr);
        const double error = range + clockStateTerm(base_clock, meter_state_) +
                             signal_bias + ionosphere_coefficient_ * ionosphere -
                             measurement_;
        if (H_pose) *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_signal_bias) {
            *H_signal_bias = (gtsam::Matrix(1, 1) << 1.0).finished();
        }
        if (H_ionosphere) {
            *H_ionosphere =
                (gtsam::Matrix(1, 1) << ionosphere_coefficient_).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

class PseudorangeFactorISBSignalBiasResidualIonosphereArm
    : public gtsam::NoiseModelFactorN<Pose3, double, double, double, double> {
    double measurement_ = 0.0;
    double ionosphere_coefficient_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    gtsam::gnss::LeverArm arm_;
    bool meter_state_ = false;

 public:
    using Base =
        gtsam::NoiseModelFactorN<Pose3, double, double, double, double>;
    using Base::evaluateError;
    PseudorangeFactorISBSignalBiasResidualIonosphereArm(
        gtsam::Key pose, gtsam::Key base_clock, gtsam::Key isb,
        gtsam::Key signal_bias, gtsam::Key ionosphere,
        double measured_pseudorange, const Point3& satellite_position,
        const gtsam::gnss::LeverArm& arm, double ionosphere_coefficient,
        const gtsam::SharedNoiseModel& model, bool meter_state = false)
        : Base(model, pose, base_clock, isb, signal_bias, ionosphere),
          measurement_(measured_pseudorange),
          ionosphere_coefficient_(ionosphere_coefficient),
          satellite_position_(satellite_position),
          arm_(arm),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Pose3& pose, const double& base_clock, const double& isb,
        const double& signal_bias, const double& ionosphere,
        gtsam::OptionalMatrixType H_pose,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_isb,
        gtsam::OptionalMatrixType H_signal_bias,
        gtsam::OptionalMatrixType H_ionosphere) const override {
        gtsam::gnss::LeverArm::PoseFrame frame;
        const Point3 antenna = arm_.antennaPosition(
            pose, H_pose ? &frame : nullptr);
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, antenna, H_pose ? &H_range : nullptr);
        const double error = range +
                             clockStateTerm(base_clock + isb, meter_state_) +
                             signal_bias + ionosphere_coefficient_ * ionosphere -
                             measurement_;
        if (H_pose) *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_isb) {
            *H_isb =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_signal_bias) {
            *H_signal_bias = (gtsam::Matrix(1, 1) << 1.0).finished();
        }
        if (H_ionosphere) {
            *H_ionosphere =
                (gtsam::Matrix(1, 1) << ionosphere_coefficient_).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

// Undifferenced carrier phase factors with an explicit clock-unit boundary.
// GTSAM's stock CarrierPhaseFactor accepts a seconds-valued clock key.  The
// Phase92 candidate owns a metre-valued clock key, so using the stock factor
// would silently mix units.  These two small equivalents preserve the stock
// equation (range + clock + ambiguity - measurement) while scaling only the
// clock term/Jacobian for the legacy seconds graph.
class UndifferencedCarrierPhaseFactorPlain
    : public gtsam::NoiseModelFactorN<Point3, double, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, double, double>;
    using Base::evaluateError;
    UndifferencedCarrierPhaseFactorPlain(
        gtsam::Key position, gtsam::Key base_clock, gtsam::Key ambiguity,
        double measured_carrier_m, const Point3& satellite_position,
        const gtsam::SharedNoiseModel& model, bool meter_state = false)
        : Base(model, position, base_clock, ambiguity),
          measurement_(measured_carrier_m),
          satellite_position_(satellite_position),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Point3& position, const double& base_clock,
        const double& ambiguity, gtsam::OptionalMatrixType H_position,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_ambiguity) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, position, H_position ? &H_range : nullptr);
        if (H_position) *H_position = H_range;
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_ambiguity) {
            *H_ambiguity = (gtsam::Matrix(1, 1) << 1.0).finished();
        }
        return (gtsam::Vector(1) <<
                range + clockStateTerm(base_clock, meter_state_) + ambiguity -
                    measurement_)
            .finished();
    }
};

class UndifferencedCarrierPhaseFactorArm
    : public gtsam::NoiseModelFactorN<Pose3, double, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    gtsam::gnss::LeverArm arm_;
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double, double>;
    using Base::evaluateError;
    UndifferencedCarrierPhaseFactorArm(
        gtsam::Key pose, gtsam::Key base_clock, gtsam::Key ambiguity,
        double measured_carrier_m, const Point3& satellite_position,
        const gtsam::gnss::LeverArm& arm,
        const gtsam::SharedNoiseModel& model, bool meter_state = false)
        : Base(model, pose, base_clock, ambiguity),
          measurement_(measured_carrier_m),
          satellite_position_(satellite_position),
          arm_(arm),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Pose3& pose, const double& base_clock,
        const double& ambiguity, gtsam::OptionalMatrixType H_pose,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_ambiguity) const override {
        gtsam::gnss::LeverArm::PoseFrame frame;
        const Point3 antenna = arm_.antennaPosition(
            pose, H_pose ? &frame : nullptr);
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, antenna, H_pose ? &H_range : nullptr);
        if (H_pose) {
            *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        }
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        if (H_ambiguity) {
            *H_ambiguity = (gtsam::Matrix(1, 1) << 1.0).finished();
        }
        return (gtsam::Vector(1) <<
                range + clockStateTerm(base_clock, meter_state_) + ambiguity -
                    measurement_)
            .finished();
    }
};

// Ordinary (single-receiver) time-differenced carrier-phase factor for a
// Pose3/IMU graph.  The native Eigen v1 path uses this same undifferenced
// range-difference model, including the per-epoch receiver-clock states and
// the existing Huber noise wrapper.  GTSAM previously omitted this factor in
// its Pose3 path, so enabling IMU silently replaced the v1 temporal carrier
// constraint with pseudorange + IMU only.  Keep the factor local to the
// backend: no base, ambiguity, or double-difference state is required.
//
// `LeverArm` maps each body-in-nav Pose3 to the antenna ECEF position.  The
// error convention is predicted-minus-measured (the sign is immaterial to a
// squared robust cost, while the Jacobians below are consistent with it):
//
//   e = (rho_k + c*b_k) - (rho_j + c*b_j) - delta_carrier.
class TimeDifferencedCarrierFactorArm
    : public gtsam::NoiseModelFactorN<Pose3, double, Pose3, double> {
    Point3 previous_satellite_position_{0, 0, 0};
    Point3 current_satellite_position_{0, 0, 0};
    double delta_carrier_m_ = 0.0;
    gtsam::gnss::LeverArm arm_;
    bool meter_state_ = false;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double, Pose3, double>;
    using Base::evaluateError;

    TimeDifferencedCarrierFactorArm(
        gtsam::Key previous_pose, gtsam::Key previous_clock,
        gtsam::Key current_pose, gtsam::Key current_clock,
        const Point3& previous_satellite_position,
        const Point3& current_satellite_position, double delta_carrier_m,
        const gtsam::gnss::LeverArm& arm,
        const gtsam::SharedNoiseModel& model, bool meter_state = false)
        : Base(model, previous_pose, previous_clock, current_pose, current_clock),
          previous_satellite_position_(previous_satellite_position),
          current_satellite_position_(current_satellite_position),
          delta_carrier_m_(delta_carrier_m),
          arm_(arm),
          meter_state_(meter_state) {}

    gtsam::Vector evaluateError(
        const Pose3& previous_pose, const double& previous_clock,
        const Pose3& current_pose, const double& current_clock,
        gtsam::OptionalMatrixType H_previous_pose,
        gtsam::OptionalMatrixType H_previous_clock,
        gtsam::OptionalMatrixType H_current_pose,
        gtsam::OptionalMatrixType H_current_clock) const override {
        gtsam::gnss::LeverArm::PoseFrame previous_frame;
        gtsam::gnss::LeverArm::PoseFrame current_frame;
        const Point3 previous_antenna = arm_.antennaPosition(
            previous_pose, H_previous_pose ? &previous_frame : nullptr);
        const Point3 current_antenna = arm_.antennaPosition(
            current_pose, H_current_pose ? &current_frame : nullptr);

        gtsam::Matrix13 H_previous_range;
        gtsam::Matrix13 H_current_range;
        const double previous_range = plainRange(
            previous_satellite_position_, previous_antenna,
            H_previous_pose ? &H_previous_range : nullptr);
        const double current_range = plainRange(
            current_satellite_position_, current_antenna,
            H_current_pose ? &H_current_range : nullptr);
        const double error = current_range +
                             clockStateTerm(current_clock, meter_state_) -
                             previous_range -
                             clockStateTerm(previous_clock, meter_state_) -
                             delta_carrier_m_;

        if (H_previous_pose) {
            *H_previous_pose =
                -arm_.antennaPoseJacobian(H_previous_range, previous_frame);
        }
        if (H_previous_clock) {
            *H_previous_clock =
                (gtsam::Matrix(1, 1) << -clockStateScale(meter_state_)).finished();
        }
        if (H_current_pose) {
            *H_current_pose =
                arm_.antennaPoseJacobian(H_current_range, current_frame);
        }
        if (H_current_clock) {
            *H_current_clock =
                (gtsam::Matrix(1, 1) << clockStateScale(meter_state_)).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

// Source-parity TDCP counterpart.  The official TDCPFactor_XXCC carries the
// complete epoch-local C vectors at both endpoints but selects only C[0] for
// the receiver-clock difference.  The class is kept separate from the scalar
// legacy factor so an active Phase101 graph cannot accidentally bind a Vector
// C key to a scalar factor; no signal-specific ISB component is introduced.
class TimeDifferencedCarrierFactorSourceClockArm
    : public gtsam::NoiseModelFactorN<Pose3, gtsam::Vector, Pose3,
                                      gtsam::Vector> {
    Point3 previous_satellite_position_{0, 0, 0};
    Point3 current_satellite_position_{0, 0, 0};
    double delta_carrier_m_ = 0.0;
    gtsam::gnss::LeverArm arm_;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, gtsam::Vector, Pose3,
                                          gtsam::Vector>;
    using Base::evaluateError;

    TimeDifferencedCarrierFactorSourceClockArm(
        gtsam::Key previous_pose, gtsam::Key previous_clock,
        gtsam::Key current_pose, gtsam::Key current_clock,
        const Point3& previous_satellite_position,
        const Point3& current_satellite_position, double delta_carrier_m,
        const gtsam::gnss::LeverArm& arm, const gtsam::SharedNoiseModel& model)
        : Base(model, previous_pose, previous_clock, current_pose,
               current_clock),
          previous_satellite_position_(previous_satellite_position),
          current_satellite_position_(current_satellite_position),
          delta_carrier_m_(delta_carrier_m),
          arm_(arm) {}

    gtsam::Vector evaluateError(
        const Pose3& previous_pose, const gtsam::Vector& previous_clock,
        const Pose3& current_pose, const gtsam::Vector& current_clock,
        gtsam::OptionalMatrixType H_previous_pose,
        gtsam::OptionalMatrixType H_previous_clock,
        gtsam::OptionalMatrixType H_current_pose,
        gtsam::OptionalMatrixType H_current_clock) const override {
        gtsam::gnss::LeverArm::PoseFrame previous_frame;
        gtsam::gnss::LeverArm::PoseFrame current_frame;
        const Point3 previous_antenna = arm_.antennaPosition(
            previous_pose, H_previous_pose ? &previous_frame : nullptr);
        const Point3 current_antenna = arm_.antennaPosition(
            current_pose, H_current_pose ? &current_frame : nullptr);
        gtsam::Matrix13 H_previous_range;
        gtsam::Matrix13 H_current_range;
        const double previous_range = plainRange(
            previous_satellite_position_, previous_antenna,
            H_previous_pose ? &H_previous_range : nullptr);
        const double current_range = plainRange(
            current_satellite_position_, current_antenna,
            H_current_pose ? &H_current_range : nullptr);
        const gtsam::Vector hc = sourceClockComponentJacobian(0);
        if (H_previous_pose) {
            *H_previous_pose =
                -arm_.antennaPoseJacobian(H_previous_range, previous_frame);
        }
        if (H_current_pose) {
            *H_current_pose =
                arm_.antennaPoseJacobian(H_current_range, current_frame);
        }
        if (H_previous_clock) *H_previous_clock = -hc.transpose();
        if (H_current_clock) *H_current_clock = hc.transpose();
        if (!finiteSourceClockVector(previous_clock) ||
            !finiteSourceClockVector(current_clock)) {
            return gtsam::Vector::Constant(
                1, std::numeric_limits<double>::quiet_NaN());
        }
        const double error =
            current_range + hc.dot(current_clock) - previous_range -
            hc.dot(previous_clock) - delta_carrier_m_;
        return (gtsam::Vector(1) << error).finished();
    }
};

// TDCP-only affine geometry in ECEF, connected to native Pose3 via LeverArm.
// The caller supplies delta(carrier + clock) - delta(anchor range), in metres.
// Not selected by any production graph until the integration is validated.
// Equal local-up antenna height; the fixed up axis is expressed in ECEF.
// No absolute height, external reference, or horizontal constraint.
class RelativeHeightPoseFactor : public gtsam::NoiseModelFactorN<Pose3, Pose3> {
    gtsam::Vector3 up_;
    gtsam::gnss::LeverArm arm_;
 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, Pose3>;
    using Base::evaluateError;
    RelativeHeightPoseFactor(gtsam::Key x1, gtsam::Key x2,
                             const gtsam::Vector3& up_ecef,
                             const gtsam::gnss::LeverArm& arm,
                             const gtsam::SharedNoiseModel& noise)
        : Base(noise, x1, x2), up_(up_ecef), arm_(arm) {
        if (!up_.allFinite() || std::abs(up_.norm() - 1.0) > 1e-9 ||
            !noise || noise->dim() != 1)
            throw std::invalid_argument("invalid relative-height factor");
    }
    gtsam::Vector evaluateError(const Pose3& x1, const Pose3& x2,
                               gtsam::OptionalMatrixType H1,
                               gtsam::OptionalMatrixType H2) const override {
        gtsam::gnss::LeverArm::PoseFrame f1, f2;
        const Point3 p1 = arm_.antennaPosition(x1, H1 ? &f1 : nullptr);
        const Point3 p2 = arm_.antennaPosition(x2, H2 ? &f2 : nullptr);
        const gtsam::Matrix13 h = up_.transpose();
        if (H1) *H1 = -arm_.antennaPoseJacobian(h, f1);
        if (H2) *H2 = arm_.antennaPoseJacobian(h, f2);
        return gtsam::Vector::Constant(1, up_.dot(p2 - p1));
    }
};

class SourceAffineTdcpPoseFactor
    : public gtsam::NoiseModelFactorN<Pose3, gtsam::Vector, Pose3, gtsam::Vector> {
    gtsam::Vector3 los_;
    Point3 anchor1_, anchor2_;
    double measurement_;
    gtsam::gnss::LeverArm arm_;
 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, gtsam::Vector, Pose3, gtsam::Vector>;
    using Base::evaluateError;
    SourceAffineTdcpPoseFactor(gtsam::Key x1, gtsam::Key c1,
                              gtsam::Key x2, gtsam::Key c2,
                              const gtsam::Vector3& previous_los,
                              const Point3& anchor1, const Point3& anchor2,
                              double residual_measurement_m,
                              const gtsam::gnss::LeverArm& arm,
                              const gtsam::SharedNoiseModel& noise)
        : Base(noise, x1, c1, x2, c2), los_(previous_los),
          anchor1_(anchor1), anchor2_(anchor2),
          measurement_(residual_measurement_m), arm_(arm) {
        if (!los_.allFinite() || std::abs(los_.norm() - 1.0) > 1e-6 ||
            !anchor1_.allFinite() || !anchor2_.allFinite() ||
            !std::isfinite(measurement_)) {
            throw std::invalid_argument("Invalid source affine TDCP geometry");
        }
    }
    gtsam::Vector evaluateError(
        const Pose3& x1, const gtsam::Vector& c1,
        const Pose3& x2, const gtsam::Vector& c2,
        gtsam::OptionalMatrixType Hx1, gtsam::OptionalMatrixType Hc1,
        gtsam::OptionalMatrixType Hx2, gtsam::OptionalMatrixType Hc2) const override {
        gtsam::gnss::LeverArm::PoseFrame f1, f2;
        const Point3 p1 = arm_.antennaPosition(x1, Hx1 ? &f1 : nullptr);
        const Point3 p2 = arm_.antennaPosition(x2, Hx2 ? &f2 : nullptr);
        const gtsam::Matrix13 h = los_.transpose();
        const gtsam::Vector hc = sourceClockComponentJacobian(0);
        if (Hx1) *Hx1 = -arm_.antennaPoseJacobian(h, f1);
        if (Hx2) *Hx2 = arm_.antennaPoseJacobian(h, f2);
        if (Hc1) *Hc1 = -hc.transpose();
        if (Hc2) *Hc2 = hc.transpose();
        if (!finiteSourceClockVector(c1) || !finiteSourceClockVector(c2) ||
            !p1.allFinite() || !p2.allFinite()) {
            return gtsam::Vector::Constant(1, std::numeric_limits<double>::quiet_NaN());
        }
        return gtsam::Vector::Constant(1,
            los_.dot((p2 - anchor2_) - (p1 - anchor1_)) +
            hc.dot(c2 - c1) - measurement_);
    }
};

// Point3/V source-parity TDCP factor for the dedicated GNSS-only graph.  It
// is the same XXCC equation as the Pose3 arm factor above, with C[0] as the
// only clock component and no lever-arm/attitude state.
class TimeDifferencedCarrierFactorSourceClockPoint
    : public gtsam::NoiseModelFactorN<Point3, gtsam::Vector, Point3,
                                      gtsam::Vector> {
    Point3 previous_satellite_position_{0, 0, 0};
    Point3 current_satellite_position_{0, 0, 0};
    double delta_carrier_m_ = 0.0;

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, gtsam::Vector, Point3,
                                          gtsam::Vector>;
    using Base::evaluateError;

    TimeDifferencedCarrierFactorSourceClockPoint(
        gtsam::Key previous_point, gtsam::Key previous_clock,
        gtsam::Key current_point, gtsam::Key current_clock,
        const Point3& previous_satellite_position,
        const Point3& current_satellite_position, double delta_carrier_m,
        const gtsam::SharedNoiseModel& model)
        : Base(model, previous_point, previous_clock, current_point,
               current_clock),
          previous_satellite_position_(previous_satellite_position),
          current_satellite_position_(current_satellite_position),
          delta_carrier_m_(delta_carrier_m) {}

    gtsam::Vector evaluateError(
        const Point3& previous_point, const gtsam::Vector& previous_clock,
        const Point3& current_point, const gtsam::Vector& current_clock,
        gtsam::OptionalMatrixType H_previous_point,
        gtsam::OptionalMatrixType H_previous_clock,
        gtsam::OptionalMatrixType H_current_point,
        gtsam::OptionalMatrixType H_current_clock) const override {
        gtsam::Matrix13 H_previous_range;
        gtsam::Matrix13 H_current_range;
        const double previous_range = plainRange(
            previous_satellite_position_, previous_point,
            H_previous_point ? &H_previous_range : nullptr);
        const double current_range = plainRange(
            current_satellite_position_, current_point,
            H_current_point ? &H_current_range : nullptr);
        const gtsam::Vector hc = sourceClockComponentJacobian(0);
        if (H_previous_point) *H_previous_point = -H_previous_range;
        if (H_current_point) *H_current_point = H_current_range;
        if (H_previous_clock) *H_previous_clock = -hc.transpose();
        if (H_current_clock) *H_current_clock = hc.transpose();
        if (!finiteSourceClockVector(previous_clock) ||
            !finiteSourceClockVector(current_clock) ||
            !std::isfinite(previous_range) || !std::isfinite(current_range) ||
            !std::isfinite(delta_carrier_m_)) {
            return gtsam::Vector::Constant(
                1, std::numeric_limits<double>::quiet_NaN());
        }
        return (gtsam::Vector(1) << current_range +
                    hc.dot(current_clock) - previous_range -
                    hc.dot(previous_clock) - delta_carrier_m_)
            .finished();
    }
};

// Official source MotionFactor_XXVV for Point3/V states:
//   (x2-x1) - (v1+v2)*dt/2.
// The factor is dedicated to Phase164; the legacy Point3 position random
// walk remains unchanged for every other configuration.
class MotionFactorXXVV
    : public gtsam::NoiseModelFactorN<Point3, Point3, gtsam::Vector3,
                                      gtsam::Vector3> {
    double dt_s_ = 0.0;

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, Point3, gtsam::Vector3,
                                          gtsam::Vector3>;
    using Base::evaluateError;

    MotionFactorXXVV(gtsam::Key previous_point, gtsam::Key current_point,
                     gtsam::Key previous_velocity, gtsam::Key current_velocity,
                     double dt_s, const gtsam::SharedNoiseModel& model)
        : Base(model, previous_point, current_point, previous_velocity,
               current_velocity),
          dt_s_(dt_s) {}

    gtsam::Vector evaluateError(
        const Point3& previous_point, const Point3& current_point,
        const gtsam::Vector3& previous_velocity,
        const gtsam::Vector3& current_velocity,
        gtsam::OptionalMatrixType H_previous_point,
        gtsam::OptionalMatrixType H_current_point,
        gtsam::OptionalMatrixType H_previous_velocity,
        gtsam::OptionalMatrixType H_current_velocity) const override {
        if (H_previous_point) *H_previous_point = -gtsam::I_3x3;
        if (H_current_point) *H_current_point = gtsam::I_3x3;
        if (H_previous_velocity) {
            *H_previous_velocity =
                -0.5 * dt_s_ * gtsam::I_3x3;
        }
        if (H_current_velocity) {
            *H_current_velocity =
                -0.5 * dt_s_ * gtsam::I_3x3;
        }
        if (!previous_point.allFinite() || !current_point.allFinite() ||
            !previous_velocity.allFinite() || !current_velocity.allFinite() ||
            !std::isfinite(dt_s_) || dt_s_ <= 0.0) {
            return gtsam::Vector::Constant(
                3, std::numeric_limits<double>::quiet_NaN());
        }
        return (gtsam::Vector3(current_point) -
                gtsam::Vector3(previous_point) -
                0.5 * dt_s_ * (previous_velocity + current_velocity));
    }
};

// Phase216 building block: source XXVV expressed at Pose3 translations.
// ENU velocity and translation share a frame. Zero lever arm is required by
// the intended main lane; this is not an antenna-offset model.
class MotionFactorPose3XXVV
    : public gtsam::NoiseModelFactorN<Pose3, Pose3, gtsam::Vector3, gtsam::Vector3> {
    double dt_s_;
 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, Pose3, gtsam::Vector3, gtsam::Vector3>;
    using Base::evaluateError;
    MotionFactorPose3XXVV(gtsam::Key p1, gtsam::Key p2, gtsam::Key v1,
                         gtsam::Key v2, double dt_s, const gtsam::SharedNoiseModel& model)
        : Base(model, p1, p2, v1, v2), dt_s_(dt_s) {
        if (!std::isfinite(dt_s_) || dt_s_ <= 0.0) {
            throw std::invalid_argument("Pose3 XXVV requires positive finite duration");
        }
    }
    gtsam::Vector evaluateError(const Pose3& p1, const Pose3& p2,
        const gtsam::Vector3& v1, const gtsam::Vector3& v2,
        gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
        gtsam::OptionalMatrixType H3, gtsam::OptionalMatrixType H4) const override {
        gtsam::Matrix j1, j2;
        const gtsam::Vector3 x1 = p1.translation(j1), x2 = p2.translation(j2);
        if (H1) *H1 = -j1;
        if (H2) *H2 = j2;
        if (H3) *H3 = -0.5 * dt_s_ * gtsam::I_3x3;
        if (H4) *H4 = -0.5 * dt_s_ * gtsam::I_3x3;
        if (!x1.allFinite() || !x2.allFinite() || !v1.allFinite() || !v2.allFinite() ||
            !j1.allFinite() || !j2.allFinite()) {
            return gtsam::Vector::Constant(3, std::numeric_limits<double>::quiet_NaN());
        }
        return x2 - x1 - 0.5 * dt_s_ * (v1 + v2);
    }
};

// Rotation-only gauge-fixing prior for a Pose3 state. Needed because the
// DD/undifferenced '...Arm' factors only observe the ANTENNA position
// (translation + R*leverArm): for a lever arm != 0 that is a rank<=3
// function of the full 6-dim pose tangent, so every pose has an exact
// (generic) 3-dim rotational null space that leaves gtsam::Marginals'
// Cholesky factorization singular. Since attitude is genuinely unobservable
// here (no IMU until milestone 2b), pinning it at its identity seed simply
// resolves the gauge freedom -- it is not a modeling approximation on top of
// real information, there is none to lose. Uses Pose3::rotation(H), whose
// Jacobian w.r.t. the pose tangent is structurally [I3 | 0] (rotation never
// depends on the translation tangent component, in any Pose3 chart), so the
// translation block of this factor's Jacobian is exactly zero and it cannot
// bias the estimated antenna position.
class Pose3RotationPrior : public gtsam::NoiseModelFactorN<Pose3> {
    Rot3 prior_;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3>;
    using Base::evaluateError;
    Pose3RotationPrior(gtsam::Key pose, const Rot3& prior_rotation,
                       const gtsam::SharedNoiseModel& model)
        : Base(model, pose), prior_(prior_rotation) {}

    gtsam::Vector evaluateError(const Pose3& pose,
                                gtsam::OptionalMatrixType H) const override {
        gtsam::Matrix36 H_rot;
        const Rot3 rot = pose.rotation(H_rot);
        if (!H) {
            return prior_.localCoordinates(rot);
        }
        gtsam::Matrix3 H_local;
        const gtsam::Vector3 error = prior_.localCoordinates(rot, {}, H_local);
        *H = H_local * H_rot;
        return error;
    }
};

// --- Milestone 2d: Non-Holonomic Constraint factor -------------------------
// Mirrors inuex35 buildfactor/nhc.py: a ground vehicle's body-frame lateral
// (Left, y) and vertical (Up, z) velocity is ~0. Binary factor on
// (Pose3 body-in-nav, Vector3 velocity-in-nav) with the 2-D residual
//   err = [ (R^T v_nav).y , (R^T v_nav).z ]      (R = nav_R_body = pose.rotation)
// Exact Jacobians come from GTSAM's own Rot3::unrotate / Pose3::rotation, so
// the linearization is correct in GTSAM's retract convention (no hand-derived
// skew term as in the Python CustomFactor). Caller gates it to moving,
// non-turning epochs. The NHC lever arm (evaluate at the rear axle) is left at
// zero -- the antenna lever is separate and the omega x lever term is a small
// correction inuex35 also defaults off (nhc_lever = 0).
class NonHolonomicFactor : public gtsam::NoiseModelFactorN<Pose3, gtsam::Vector3> {
 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, gtsam::Vector3>;
    using Base::evaluateError;
    NonHolonomicFactor(gtsam::Key pose, gtsam::Key velocity,
                       const gtsam::SharedNoiseModel& model)
        : Base(model, pose, velocity) {}

    gtsam::Vector evaluateError(const Pose3& pose, const gtsam::Vector3& v_nav,
                                gtsam::OptionalMatrixType H_pose,
                                gtsam::OptionalMatrixType H_vel) const override {
        gtsam::Matrix36 H_rot_pose;  // d(rot tangent)/d(pose tangent) = [I3|0]
        const Rot3 R = pose.rotation(H_pose ? &H_rot_pose : nullptr);
        gtsam::Matrix3 H_unrot_rot, H_unrot_v;
        const gtsam::Point3 v_body =
            R.unrotate(gtsam::Point3(v_nav), H_pose ? &H_unrot_rot : nullptr,
                       H_vel ? &H_unrot_v : nullptr);
        if (H_pose) {
            const gtsam::Matrix36 dvb_dpose = H_unrot_rot * H_rot_pose;  // 3x6
            gtsam::Matrix H = gtsam::Matrix::Zero(2, 6);
            H.row(0) = dvb_dpose.row(1);  // d(v_body.y)/d(pose)
            H.row(1) = dvb_dpose.row(2);  // d(v_body.z)/d(pose)
            *H_pose = H;
        }
        if (H_vel) {
            gtsam::Matrix H = gtsam::Matrix::Zero(2, 3);
            H.row(0) = H_unrot_v.row(1);
            H.row(1) = H_unrot_v.row(2);
            *H_vel = H;
        }
        return (gtsam::Vector(2) << v_body.y(), v_body.z()).finished();
    }
};

// Single-difference Doppler directly observes the ENU velocity state.  The
// problem builder stores LOS in ECEF, so the caller rotates it into ENU before
// constructing this factor.  Sign follows the Eigen backend's model:
// measured_sd_doppler = los_sd dot velocity.
class SingleDifferenceDopplerVelocityFactor
    : public gtsam::NoiseModelFactorN<gtsam::Vector3> {
    gtsam::Vector3 los_nav_;
    double measured_mps_ = 0.0;

 public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Vector3>;
    using Base::evaluateError;
    SingleDifferenceDopplerVelocityFactor(
        gtsam::Key velocity, const gtsam::Vector3& los_nav, double measured_mps,
        const gtsam::SharedNoiseModel& model)
        : Base(model, velocity), los_nav_(los_nav), measured_mps_(measured_mps) {}

    gtsam::Vector evaluateError(const gtsam::Vector3& velocity_nav,
                                gtsam::OptionalMatrixType H) const override {
        if (H) {
            *H = los_nav_.transpose();
        }
        return (gtsam::Vector(1) << los_nav_.dot(velocity_nav) - measured_mps_).finished();
    }
};

// Receiver-only undifferenced Doppler factor for the upstream GNSS-first
// graph.  The prepared `los` follows the existing native contract (satellite
// to receiver, i.e. `-doppler_los`), so the prediction is `los dot v + d`.
// `los_nav` is that vector in the local ENU frame and
// `residual_mps` is the known-satellite-terms-removed Android/RINEX range-rate
// residual prepared by fgo_problems.cpp.  The factor estimates ENU velocity
// and receiver clock range-rate in metres/second, matching
// taroz/gsdc2023's DopplerFactor_VD contract.
class UndifferencedDopplerVelocityFactor
    : public gtsam::NoiseModelFactorN<gtsam::Vector3, double> {
    gtsam::Vector3 los_nav_;
    double measured_mps_ = 0.0;

 public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Vector3, double>;
    using Base::evaluateError;
    UndifferencedDopplerVelocityFactor(
        gtsam::Key velocity, gtsam::Key clock_drift,
        const gtsam::Vector3& los_nav, double measured_mps,
        const gtsam::SharedNoiseModel& model)
        : Base(model, velocity, clock_drift),
          los_nav_(los_nav), measured_mps_(measured_mps) {}

    gtsam::Vector evaluateError(const gtsam::Vector3& velocity_nav,
                                const double& clock_drift_mps,
                                gtsam::OptionalMatrixType H_velocity,
                                gtsam::OptionalMatrixType H_clock_drift) const override {
        if (H_velocity) *H_velocity = los_nav_.transpose();
        if (H_clock_drift) *H_clock_drift = gtsam::Matrix::Constant(1, 1, 1.0);
        return (gtsam::Vector(1) << los_nav_.dot(velocity_nav) +
                                      clock_drift_mps - measured_mps_)
            .finished();
    }
};

// The official source D state is represented as a one-element Vector. Keep a
// dedicated factor for the Phase101 graph so the CCDD D key and Doppler D key
// have the same value type; the legacy scalar factor remains untouched.
class UndifferencedDopplerVelocityFactorSourceClock
    : public gtsam::NoiseModelFactorN<gtsam::Vector3, gtsam::Vector> {
    gtsam::Vector3 los_nav_;
    double measured_mps_ = 0.0;

 public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Vector3, gtsam::Vector>;
    using Base::evaluateError;
    UndifferencedDopplerVelocityFactorSourceClock(
        gtsam::Key velocity, gtsam::Key clock_drift,
        const gtsam::Vector3& los_nav, double measured_mps,
        const gtsam::SharedNoiseModel& model)
        : Base(model, velocity, clock_drift),
          los_nav_(los_nav),
          measured_mps_(measured_mps) {}

    gtsam::Vector evaluateError(
        const gtsam::Vector3& velocity_nav,
        const gtsam::Vector& clock_drift_mps,
        gtsam::OptionalMatrixType H_velocity,
        gtsam::OptionalMatrixType H_clock_drift) const override {
        if (H_velocity) *H_velocity = los_nav_.transpose();
        if (H_clock_drift) *H_clock_drift = gtsam::Matrix::Constant(1, 1, 1.0);
        if (clock_drift_mps.size() != 1 || !clock_drift_mps.allFinite()) {
            return gtsam::Vector::Constant(
                1, std::numeric_limits<double>::quiet_NaN());
        }
        return (gtsam::Vector(1) << los_nav_.dot(velocity_nav) +
                                      clock_drift_mps(0) - measured_mps_)
            .finished();
    }
};

// Source-clock Doppler factor for the dedicated raw-P GNSS-first graph.  The
// equation is identical to the source-clock factor above, but the type makes
// the frame contract explicit: both `los` and the velocity state are ECEF.
// The generic raw-D path must continue to rotate its ECEF LOS into ENU before
// using its ENU velocity state, so it intentionally uses the base class.
class UndifferencedDopplerVelocityFactorSourceClockEcef
    : public UndifferencedDopplerVelocityFactorSourceClock {
 public:
    using Base = UndifferencedDopplerVelocityFactorSourceClock;

    UndifferencedDopplerVelocityFactorSourceClockEcef(
        gtsam::Key velocity, gtsam::Key clock_drift,
        const gtsam::Vector3& los_ecef, double measured_mps,
        const gtsam::SharedNoiseModel& model)
        : Base(velocity, clock_drift, los_ecef, measured_mps, model) {}
};

// Stationarity stats over an IMU sample sub-range [begin, end), mirroring the
// Vector-deviation RMS plus the median bias-referenced gyro norm, matching
// inuex35's compute_zupt_stats. The older std-of-norms formulation could hide
// directional vibration whose magnitude stayed nearly constant.
struct ImuWindowStats {
    int n = 0;
    double accel_std = 0.0;
    double gyro_std = 0.0;
    double gyro_median = 0.0;
    double yaw_rate_abs = 0.0;  ///< |mean(gyro_z - bias_z)|, for the NHC turn gate
};

inline ImuWindowStats imuWindowStats(const std::vector<ImuSample>& samples, std::size_t begin,
                                     std::size_t end, const Vector3d& accel_bias,
                                     const Vector3d& gyro_bias) {
    ImuWindowStats s;
    const std::size_t n = end > begin ? end - begin : 0;
    s.n = static_cast<int>(n);
    if (n == 0) {
        return s;
    }
    Vector3d accel_mean = Vector3d::Zero();
    Vector3d gyro_mean = Vector3d::Zero();
    std::vector<double> gyro_residual_norms;
    gyro_residual_norms.reserve(n);
    double gyro_z_sum = 0.0;
    for (std::size_t k = begin; k < end; ++k) {
        accel_mean += samples[k].accel_raw;
        gyro_mean += samples[k].gyro_raw_radps;
        gyro_residual_norms.push_back(
            (samples[k].gyro_raw_radps - gyro_bias).norm());
        gyro_z_sum += samples[k].gyro_raw_radps.z() - gyro_bias.z();
    }
    accel_mean /= static_cast<double>(n);
    gyro_mean /= static_cast<double>(n);
    double accel_variance = 0.0;
    double gyro_variance = 0.0;
    for (std::size_t k = begin; k < end; ++k) {
        accel_variance += (samples[k].accel_raw - accel_mean).squaredNorm();
        gyro_variance +=
            (samples[k].gyro_raw_radps - gyro_mean).squaredNorm();
    }
    std::sort(gyro_residual_norms.begin(), gyro_residual_norms.end());
    s.accel_std = std::sqrt(accel_variance / static_cast<double>(n));
    s.gyro_std = std::sqrt(gyro_variance / static_cast<double>(n));
    s.gyro_median = gyro_residual_norms[gyro_residual_norms.size() / 2];
    s.yaw_rate_abs = std::abs(gyro_z_sum / static_cast<double>(n));
    return s;
}

inline SharedNoise makeNoise(double sigma_m, bool robust, double huber_threshold_sigma) {
    const double safe_sigma = std::max(1e-9, sigma_m);
    SharedNoise base = gtsam::noiseModel::Isotropic::Sigma(1, safe_sigma);
    if (robust && huber_threshold_sigma > 0.0) {
        return gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(huber_threshold_sigma),
            base);
    }
    return base;
}


// Result of the mini DDPR-only LS anchor solve (FGOConfig::use_ddpr_anchor;
// port of the inuex35 reference's utils/ls_solvers.py ddpr_only_position).
// `pose` is a body-in-nav-ENU Pose3, the SAME convention as the main graph's
// positionKey(i) -- i.e. translation() is the body position in nav-ENU, NOT
// the antenna position (apply the caller's antennaOf()/lever-arm convention
// to get the antenna ECEF, exactly like the main loop does for pose_i).
struct DdprAnchorResult {
    bool ok = false;
    Pose3 pose;
    int n_active = 0;
    double res_rms = std::numeric_limits<double>::infinity();
};

// Cross-checks that gtsam::gnss::DoubleDifferenceData::observed() reproduces
// libgnss's precomputed observed DD value for the mapping used below. Cheap
// (a handful of subtractions) so it is left active in all builds rather than
// only in debug; a mismatch indicates the obs-convention mapping regressed.
inline void checkObservedDdMatches(double gtsam_observed,
                            double libgnss_observed,
                            const char* what) {
    const double diff = std::abs(gtsam_observed - libgnss_observed);
    if (diff > 1e-6) {
        std::fprintf(stderr,
                     "[fgo_gtsam_backend] WARNING: %s observed-DD mismatch: "
                     "gtsam=%.9f libgnss=%.9f diff=%.9e\n",
                     what, gtsam_observed, libgnss_observed, diff);
        assert(false && "GTSAM/libgnss DD observation convention mismatch");
    }
}

}  // namespace fgo_gtsam_internal
}  // namespace libgnss
