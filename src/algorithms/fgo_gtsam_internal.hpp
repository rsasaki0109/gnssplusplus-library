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

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
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
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>


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
//   error = ||rcv - sat|| + c*(baseClock + isb) - measuredPseudorange
// Base clock is per epoch; isb is one global (time-constant) node per non-GPS
// constellation -- mirroring the native backend's per-epoch clock + shared
// per-constellation bias column.
class PseudorangeFactorISB : public gtsam::NoiseModelFactorN<Point3, double, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, double, double>;
    using Base::evaluateError;
    PseudorangeFactorISB(gtsam::Key position, gtsam::Key base_clock, gtsam::Key isb,
                         double measured_pseudorange, const Point3& satellite_position,
                         const gtsam::SharedNoiseModel& model)
        : Base(model, position, base_clock, isb),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position) {}

    gtsam::Vector evaluateError(const Point3& position, const double& base_clock,
                                const double& isb, gtsam::OptionalMatrixType H_position,
                                gtsam::OptionalMatrixType H_base_clock,
                                gtsam::OptionalMatrixType H_isb) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(satellite_position_, position, H_position ? &H_range : nullptr);
        const double error =
            range + gtsam::gnss::C_LIGHT * (base_clock + isb) - measurement_;
        if (H_position) {
            *H_position = H_range;
        }
        if (H_base_clock) {
            *H_base_clock = (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        if (H_isb) {
            *H_isb = (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

// GPS/base-group undifferenced pseudorange factor (no ISB node).
//   error = ||rcv - sat|| + c*baseClock - measuredPseudorange
class PseudorangeFactorPlain : public gtsam::NoiseModelFactorN<Point3, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, double>;
    using Base::evaluateError;
    PseudorangeFactorPlain(gtsam::Key position, gtsam::Key base_clock,
                           double measured_pseudorange, const Point3& satellite_position,
                           const gtsam::SharedNoiseModel& model)
        : Base(model, position, base_clock),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position) {}

    gtsam::Vector evaluateError(const Point3& position, const double& base_clock,
                                gtsam::OptionalMatrixType H_position,
                                gtsam::OptionalMatrixType H_base_clock) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(satellite_position_, position, H_position ? &H_range : nullptr);
        const double error = range + gtsam::gnss::C_LIGHT * base_clock - measurement_;
        if (H_position) {
            *H_position = H_range;
        }
        if (H_base_clock) {
            *H_base_clock = (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        return (gtsam::Vector(1) << error).finished();
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

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, double, double>;
    using Base::evaluateError;
    PseudorangeFactorPlainSignalBias(
        gtsam::Key position, gtsam::Key base_clock, gtsam::Key signal_bias,
        double measured_pseudorange, const Point3& satellite_position,
        const gtsam::SharedNoiseModel& model)
        : Base(model, position, base_clock, signal_bias),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position) {}

    gtsam::Vector evaluateError(
        const Point3& position, const double& base_clock,
        const double& signal_bias, gtsam::OptionalMatrixType H_position,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_signal_bias) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, position, H_position ? &H_range : nullptr);
        const double error = range + gtsam::gnss::C_LIGHT * base_clock +
                             signal_bias - measurement_;
        if (H_position) *H_position = H_range;
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
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

 public:
    using Base = gtsam::NoiseModelFactorN<Point3, double, double, double>;
    using Base::evaluateError;
    PseudorangeFactorISBSignalBias(
        gtsam::Key position, gtsam::Key base_clock, gtsam::Key isb,
        gtsam::Key signal_bias, double measured_pseudorange,
        const Point3& satellite_position, const gtsam::SharedNoiseModel& model)
        : Base(model, position, base_clock, isb, signal_bias),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position) {}

    gtsam::Vector evaluateError(
        const Point3& position, const double& base_clock, const double& isb,
        const double& signal_bias, gtsam::OptionalMatrixType H_position,
        gtsam::OptionalMatrixType H_base_clock,
        gtsam::OptionalMatrixType H_isb,
        gtsam::OptionalMatrixType H_signal_bias) const override {
        gtsam::Matrix13 H_range;
        const double range = plainRange(
            satellite_position_, position, H_position ? &H_range : nullptr);
        const double error = range + gtsam::gnss::C_LIGHT * (base_clock + isb) +
                             signal_bias - measurement_;
        if (H_position) *H_position = H_range;
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        if (H_isb) {
            *H_isb = (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        if (H_signal_bias) {
            *H_signal_bias = (gtsam::Matrix(1, 1) << 1.0).finished();
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

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double>;
    using Base::evaluateError;
    PseudorangeFactorPlainArm(gtsam::Key pose, gtsam::Key base_clock,
                              double measured_pseudorange, const Point3& satellite_position,
                              const gtsam::gnss::LeverArm& arm,
                              const gtsam::SharedNoiseModel& model)
        : Base(model, pose, base_clock),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          arm_(arm) {}

    gtsam::Vector evaluateError(const Pose3& pose, const double& base_clock,
                                gtsam::OptionalMatrixType H_pose,
                                gtsam::OptionalMatrixType H_base_clock) const override {
        gtsam::gnss::LeverArm::PoseFrame frame;
        const Point3 antenna = arm_.antennaPosition(pose, H_pose ? &frame : nullptr);
        gtsam::Matrix13 H_range;
        const double range = plainRange(satellite_position_, antenna, H_pose ? &H_range : nullptr);
        const double error = range + gtsam::gnss::C_LIGHT * base_clock - measurement_;
        if (H_pose) {
            *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        }
        if (H_base_clock) {
            *H_base_clock = (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

class PseudorangeFactorISBArm : public gtsam::NoiseModelFactorN<Pose3, double, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    gtsam::gnss::LeverArm arm_;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double, double>;
    using Base::evaluateError;
    PseudorangeFactorISBArm(gtsam::Key pose, gtsam::Key base_clock, gtsam::Key isb,
                            double measured_pseudorange, const Point3& satellite_position,
                            const gtsam::gnss::LeverArm& arm,
                            const gtsam::SharedNoiseModel& model)
        : Base(model, pose, base_clock, isb),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          arm_(arm) {}

    gtsam::Vector evaluateError(const Pose3& pose, const double& base_clock, const double& isb,
                                gtsam::OptionalMatrixType H_pose,
                                gtsam::OptionalMatrixType H_base_clock,
                                gtsam::OptionalMatrixType H_isb) const override {
        gtsam::gnss::LeverArm::PoseFrame frame;
        const Point3 antenna = arm_.antennaPosition(pose, H_pose ? &frame : nullptr);
        gtsam::Matrix13 H_range;
        const double range = plainRange(satellite_position_, antenna, H_pose ? &H_range : nullptr);
        const double error =
            range + gtsam::gnss::C_LIGHT * (base_clock + isb) - measurement_;
        if (H_pose) {
            *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        }
        if (H_base_clock) {
            *H_base_clock = (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        if (H_isb) {
            *H_isb = (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

class PseudorangeFactorPlainSignalBiasArm
    : public gtsam::NoiseModelFactorN<Pose3, double, double> {
    double measurement_ = 0.0;
    Point3 satellite_position_{0, 0, 0};
    gtsam::gnss::LeverArm arm_;

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double, double>;
    using Base::evaluateError;
    PseudorangeFactorPlainSignalBiasArm(
        gtsam::Key pose, gtsam::Key base_clock, gtsam::Key signal_bias,
        double measured_pseudorange, const Point3& satellite_position,
        const gtsam::gnss::LeverArm& arm, const gtsam::SharedNoiseModel& model)
        : Base(model, pose, base_clock, signal_bias),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          arm_(arm) {}

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
        const double error = range + gtsam::gnss::C_LIGHT * base_clock +
                             signal_bias - measurement_;
        if (H_pose) *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
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

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double, double, double>;
    using Base::evaluateError;
    PseudorangeFactorISBSignalBiasArm(
        gtsam::Key pose, gtsam::Key base_clock, gtsam::Key isb,
        gtsam::Key signal_bias, double measured_pseudorange,
        const Point3& satellite_position, const gtsam::gnss::LeverArm& arm,
        const gtsam::SharedNoiseModel& model)
        : Base(model, pose, base_clock, isb, signal_bias),
          measurement_(measured_pseudorange),
          satellite_position_(satellite_position),
          arm_(arm) {}

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
        const double error = range + gtsam::gnss::C_LIGHT * (base_clock + isb) +
                             signal_bias - measurement_;
        if (H_pose) *H_pose = arm_.antennaPoseJacobian(H_range, frame);
        if (H_base_clock) {
            *H_base_clock =
                (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        if (H_isb) {
            *H_isb = (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        if (H_signal_bias) {
            *H_signal_bias = (gtsam::Matrix(1, 1) << 1.0).finished();
        }
        return (gtsam::Vector(1) << error).finished();
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

 public:
    using Base = gtsam::NoiseModelFactorN<Pose3, double, Pose3, double>;
    using Base::evaluateError;

    TimeDifferencedCarrierFactorArm(
        gtsam::Key previous_pose, gtsam::Key previous_clock,
        gtsam::Key current_pose, gtsam::Key current_clock,
        const Point3& previous_satellite_position,
        const Point3& current_satellite_position, double delta_carrier_m,
        const gtsam::gnss::LeverArm& arm,
        const gtsam::SharedNoiseModel& model)
        : Base(model, previous_pose, previous_clock, current_pose, current_clock),
          previous_satellite_position_(previous_satellite_position),
          current_satellite_position_(current_satellite_position),
          delta_carrier_m_(delta_carrier_m),
          arm_(arm) {}

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
        const double error = current_range + gtsam::gnss::C_LIGHT * current_clock -
                             previous_range - gtsam::gnss::C_LIGHT * previous_clock -
                             delta_carrier_m_;

        if (H_previous_pose) {
            *H_previous_pose =
                -arm_.antennaPoseJacobian(H_previous_range, previous_frame);
        }
        if (H_previous_clock) {
            *H_previous_clock =
                (gtsam::Matrix(1, 1) << -gtsam::gnss::C_LIGHT).finished();
        }
        if (H_current_pose) {
            *H_current_pose =
                arm_.antennaPoseJacobian(H_current_range, current_frame);
        }
        if (H_current_clock) {
            *H_current_clock =
                (gtsam::Matrix(1, 1) << gtsam::gnss::C_LIGHT).finished();
        }
        return (gtsam::Vector(1) << error).finished();
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
