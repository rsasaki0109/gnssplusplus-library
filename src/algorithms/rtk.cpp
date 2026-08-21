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

RTKProcessor::RTKProcessor()
    : spp_processor_(makeRTKSppProcessor(rtk_config_)),
      l1_l5_mw_arc_bank_(
          rtk_config_.lambda_l1_l5_wlnl_causal_arc_config),
      ambiguity_arc_bank_(
          rtk_config_.lambda_causal_arc_readiness_config) {
    filter_initialized_ = false;
}
RTKProcessor::RTKProcessor(const RTKConfig& rtk_config)
    : rtk_config_(rtk_config),
      spp_processor_(makeRTKSppProcessor(rtk_config_)),
      l1_l5_mw_arc_bank_(
          rtk_config_.lambda_l1_l5_wlnl_causal_arc_config),
      ambiguity_arc_bank_(
          rtk_config_.lambda_causal_arc_readiness_config) {
    filter_initialized_ = false;
}

void RTKProcessor::setRTKConfig(const RTKConfig& config) {
    rtk_config_ = config;
    l1_l5_mw_arc_bank_ = causal_ambiguity_arc::Bank(
        rtk_config_.lambda_l1_l5_wlnl_causal_arc_config);
    ambiguity_arc_bank_ = causal_ambiguity_arc::Bank(
        rtk_config_.lambda_causal_arc_readiness_config);
    syncSPPConfig();
    // Phase 2a: cmc_suspect_tracker_ is lazily constructed with the
    // cmc_ref_level_m threshold captured at construction time; drop it so a
    // config change (e.g. --cmc-ref-level) takes effect on the next epoch
    // instead of being silently ignored.
    cmc_suspect_tracker_.reset();
}

void RTKProcessor::syncSPPConfig() {
    spp_processor_.setSPPConfig(makeRTKSppConfig(rtk_config_));
}

bool RTKProcessor::initialize(const ProcessorConfig& config) {
    config_ = config;
    syncSPPConfig();
    spp_processor_.initialize(config);
    reset();
    return true;
}

PositionSolution RTKProcessor::processEpoch(const ObservationData& rover_obs, const NavigationData& nav) {
    return spp_processor_.processEpoch(rover_obs, nav);
}

void RTKProcessor::reset() {
    filter_initialized_ = false;
    adaptive_noise_tracker_.clear();
    debug_telemetry_ = EpochDebugTelemetry{};
    debug_telemetry_.prior_held_integer_count = static_cast<int>(last_dd_fixed_.size());
    debug_telemetry_.prior_held_pair_count = static_cast<int>(last_best_subset_.size());
    debug_telemetry_.prior_consecutive_fix_count = consecutive_fix_count_;
    debug_telemetry_.prior_tracked_ambiguity_count =
        static_cast<int>(lock_count_l1_.size() +
                         lock_count_l2_.size() +
                         lock_count_l5_.size());
    safe_fix_shadow_state_machine_.reset();
    disjoint_consensus_state_machine_.reset();
    causal_arc_consensus_state_machine_.reset();
    satellite_par_consensus_state_machine_.reset();
    src_par_consensus_state_machine_.reset();
    inertial_referenced_consensus_state_machine_.reset();
    multifrequency_consensus_state_machine_.reset();
    l1_l2_multifrequency_consensus_state_machine_.reset();
    satellite_par_persistent_satellites_.clear();
    l1_l5_mw_arc_bank_.reset();
    filter_state_ = RTKState{};
    filter_state_.next_state_idx = REAL_STATES + IONO_STATES;
    ambiguity_states_.clear();
    lock_count_l1_.clear();
    lock_count_l2_.clear();
    lock_count_l5_.clear();  // Phase 18 Step 2: clear L5 lock counts on reset
    has_fixed_solution_ = false;
    has_last_fixed_position_ = false;
    has_last_fixed_time_ = false;
    has_last_solution_position_ = false;
    has_last_trusted_position_ = false;
    has_fixed_update_gate_previous_solution_ = false;
    has_ref_satellite_ = false;
    has_last_epoch_ = false;
    has_last_trusted_time_ = false;
    has_prev_trusted_position_ = false;
    fixed_anchor_float_stabilizer_armed_ = false;
    fixed_anchor_float_history_.clear();
    has_last_doppler_velocity_ = false;
    has_doppler_continuity_position_ = false;
    current_epoch_nlos_fraction_ = std::numeric_limits<double>::quiet_NaN();
    current_sat_data_.clear();
    gf_l1l2_history_.clear();
    gf_l1l5_history_.clear();
    doppler_phase_history_l1_m_.clear();
    doppler_phase_history_l2_m_.clear();
    doppler_phase_history_l5_m_.clear();
    code_phase_history_l1_m_.clear();
    code_phase_history_l2_m_.clear();
    tdcp_history_l1_.clear();
    tdcp_history_l2_.clear();
    tdcp_history_l5_.clear();
    cmc_suspect_tracker_.reset();
    cmc_ref_hysteresis_by_system_.clear();
    cmc_aware_ref_by_system_.clear();
    cmc_ref_suspect_epoch_count_ = 0;
    cmc_ref_switch_count_ = 0;
    has_external_position_time_update_ = false;
    has_external_velocity_time_update_ = false;
    ins_time_update_applied_count_ = 0;
    ins_time_update_rejected_count_ = 0;
    ins_time_update_applied_last_epoch_ = false;
    position_correction_count_ = 0;
    position_correction_sum_sq_m2_ = 0.0;
    consecutive_cp_pr_gate_rejections_ = 0;
    has_last_ddpr_anchor_ = false;
    code_phase_history_l5_m_.clear();
    current_epoch_slips_l1_.clear();
    current_epoch_slips_l2_.clear();
    current_epoch_slips_l5_.clear();
    ambiguity_arc_bank_.reset();
    consecutive_fix_count_ = 0;
    consecutive_float_count_ = 0;
    consecutive_nonfix_count_ = 0;
    consecutive_high_float_residual_count_ = 0;
    consecutive_high_fixed_residual_count_ = 0;
    adaptive_dynamic_slip_hold_count_ = 0;
    last_ar_ratio_ = 0.0;
    last_num_fixed_ambiguities_ = 0;
    current_update_diagnostics_ = RTKUpdateDiagnostics{};
    std::lock_guard<std::mutex> lock(stats_mutex_);
    total_epochs_processed_ = 0;
    fixed_solutions_ = 0;
    float_solutions_ = 0;
}

ProcessorStats RTKProcessor::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ProcessorStats stats;
    stats.total_epochs = total_epochs_processed_;
    stats.valid_solutions = fixed_solutions_ + float_solutions_;
    stats.fixed_solutions = fixed_solutions_;
    return stats;
}

bool RTKProcessor::getFloatPosteriorPosition(
    Vector3d& position_ecef, Matrix3d& position_covariance_ecef) const {
    if (!filter_initialized_ || !base_position_known_ ||
        filter_state_.state.size() < BASE_STATES ||
        filter_state_.covariance.rows() < BASE_STATES ||
        filter_state_.covariance.cols() < BASE_STATES) {
        return false;
    }
    position_ecef = base_position_ + filter_state_.state.head<BASE_STATES>();
    position_covariance_ecef =
        filter_state_.covariance.topLeftCorner<BASE_STATES, BASE_STATES>();
    return position_ecef.allFinite() && position_covariance_ecef.allFinite();
}

bool RTKProcessor::getLastDdPrAnchor(
    Vector3d& position_ecef, Matrix3d& position_covariance_ecef, GNSSTime& time) const {
    if (!has_last_ddpr_anchor_) {
        return false;
    }
    position_ecef = last_ddpr_anchor_position_ecef_;
    position_covariance_ecef = last_ddpr_anchor_covariance_ecef_;
    time = last_ddpr_anchor_time_;
    return true;
}

// RTKLIB varerr: SD measurement error variance

PositionSolution RTKProcessor::generateSolution(const GNSSTime& time, SolutionStatus status, int num_satellites) {
    PositionSolution solution;
    solution.time = time;
    solution.status = status;
    solution.position_ecef = (filter_state_.state.size() >= 3) ?
        Vector3d(base_position_ + filter_state_.state.head<3>()) : base_position_;
    const double a = constants::WGS84_A, f = constants::WGS84_F, b_axis = a*(1-f), e2 = constants::WGS84_E2;
    double x = solution.position_ecef(0), y = solution.position_ecef(1), z = solution.position_ecef(2);
    double p = std::sqrt(x*x + y*y), theta = std::atan2(z*a, p*b_axis);
    solution.position_geodetic.longitude = std::atan2(y, x);
    solution.position_geodetic.latitude = std::atan2(z + e2*b_axis/(1-e2)*std::pow(std::sin(theta),3), p - e2*a*std::pow(std::cos(theta),3));
    double N = a / std::sqrt(1 - e2*std::sin(solution.position_geodetic.latitude)*std::sin(solution.position_geodetic.latitude));
    solution.position_geodetic.height = p / std::cos(solution.position_geodetic.latitude) - N;
    solution.num_satellites = num_satellites;
    solution.position_covariance = Matrix3d::Identity() * 0.01;
    solution.pdop = 2.0; solution.hdop = 1.5; solution.vdop = 2.5;
    if (filter_state_.state.size() >= 3) solution.baseline_length = filter_state_.state.head<3>().norm();
    solution.ratio = last_ar_ratio_;
    solution.num_fixed_ambiguities = last_num_fixed_ambiguities_;
    solution.iterations = current_update_diagnostics_.iterations;
    solution.residual_rms = current_update_diagnostics_.post_suppression_residual_rms_m;
    solution.rtk_update_observations = current_update_diagnostics_.observation_count;
    solution.rtk_update_phase_observations = current_update_diagnostics_.phase_observation_count;
    solution.rtk_update_code_observations = current_update_diagnostics_.code_observation_count;
    solution.rtk_update_suppressed_outliers = current_update_diagnostics_.suppressed_outliers;
    solution.rtk_update_prefit_residual_rms_m =
        current_update_diagnostics_.prefit_residual_rms_m;
    solution.rtk_update_prefit_residual_max_m =
        current_update_diagnostics_.prefit_residual_max_m;
    solution.rtk_update_post_suppression_residual_rms_m =
        current_update_diagnostics_.post_suppression_residual_rms_m;
    solution.rtk_update_post_suppression_residual_max_m =
        current_update_diagnostics_.post_suppression_residual_max_m;
    solution.rtk_update_normalized_innovation_squared =
        current_update_diagnostics_.normalized_innovation_squared;
    solution.rtk_update_normalized_innovation_squared_per_observation =
        current_update_diagnostics_.normalized_innovation_squared_per_observation;
    solution.rtk_update_rejected_by_innovation_gate =
        current_update_diagnostics_.rejected_by_innovation_gate ? 1 : 0;
    if (rtk_config_.enable_velocity_states &&
        filter_state_.state.size() >= VELOCITY_STATE_INDEX + VELOCITY_STATES &&
        filter_state_.covariance.rows() == filter_state_.state.size()) {
        const Vector3d velocity = filter_state_.state.segment<3>(VELOCITY_STATE_INDEX);
        const Matrix3d velocity_covariance =
            filter_state_.covariance.block<3, 3>(
                VELOCITY_STATE_INDEX, VELOCITY_STATE_INDEX);
        if (velocity.allFinite() && velocity_covariance.allFinite() &&
            velocity_covariance.diagonal().minCoeff() > 0.0) {
            solution.velocity_ecef = velocity;
            solution.velocity_covariance = velocity_covariance;
            solution.has_velocity = true;
        }
    }
    rememberSolution(solution);
    return solution;
}

void RTKProcessor::rememberSolution(const PositionSolution& solution) {
    if (!solution.isValid()) return;
    last_solution_position_ = solution.position_ecef;
    has_last_solution_position_ = true;
    last_epoch_time_ = solution.time;
    has_last_epoch_ = true;
    bool refresh_trusted = solution.status == SolutionStatus::FIXED;
    if (!refresh_trusted &&
        solution.status == SolutionStatus::FLOAT &&
        solution.num_satellites >= 5) {
        if (isMovingBasePositionMode(rtk_config_)) {
            refresh_trusted = true;
        } else if (!has_last_trusted_position_ || !has_last_trusted_time_) {
            refresh_trusted = true;
        } else {
            double dt = solution.time - last_trusted_time_;
            if (!std::isfinite(dt) || dt < 0.5) {
                dt = 1.0;
            }
            const double trusted_jump =
                (solution.position_ecef - last_trusted_position_).norm();
            // WP9 optional lever: relax the jump gate 2x when a majority of
            // this epoch's tracked satellites are NLOS-flagged (per the
            // loaded weight table). No-op (multiplier stays 1.0) unless
            // both --trust-gate-nlos-relax is set and current_epoch_nlos_
            // fraction_ was actually computed this epoch (finite).
            double jump_gate_multiplier = 1.0;
            if (rtk_config_.trust_gate_nlos_relax &&
                std::isfinite(current_epoch_nlos_fraction_) &&
                current_epoch_nlos_fraction_ > 0.5) {
                jump_gate_multiplier = 2.0;
            }
            refresh_trusted = trusted_jump <= std::max(3.0, 6.0 * dt) * jump_gate_multiplier;
        }
    }
    if (refresh_trusted) {
        // WP9: remember the trust sample being replaced so CV_PREDICT can
        // derive a two-point constant-velocity estimate from the last two
        // trusted deltas. No effect on any exported solution.
        if (has_last_trusted_position_ && has_last_trusted_time_) {
            prev_trusted_position_ = last_trusted_position_;
            prev_trusted_time_ = last_trusted_time_;
            has_prev_trusted_position_ = true;
        }
        last_trusted_position_ = solution.position_ecef;
        has_last_trusted_position_ = true;
        last_trusted_time_ = solution.time;
        has_last_trusted_time_ = true;
    }
}

PositionSolution RTKProcessor::makeSafeFloatContinuity(
    const GNSSTime& time) {
    PositionSolution solution;
    solution.time = time;
    solution.status = SolutionStatus::NONE;

    const double trusted_anchor_age_s =
        has_last_trusted_time_
            ? time - last_trusted_time_
            : std::numeric_limits<double>::quiet_NaN();
    const double velocity_age_s =
        has_last_doppler_velocity_
            ? time - last_doppler_velocity_time_
            : std::numeric_limits<double>::quiet_NaN();
    debug_telemetry_.safe_float_continuity_attempted =
        rtk_config_.safe_float_continuity.enabled;
    debug_telemetry_.safe_float_continuity_anchor_age_s =
        trusted_anchor_age_s;
    debug_telemetry_.safe_float_continuity_velocity_age_s =
        velocity_age_s;
    auto continuity = safe_float_continuity::propagate(
        rtk_config_.safe_float_continuity,
        last_trusted_position_,
        trusted_anchor_age_s,
        last_doppler_velocity_ecef_,
        velocity_age_s);
    double anchor_age_s = trusted_anchor_age_s;
    bool solver_gap_anchor = false;

    // If the trusted anchor is too old, bridge only an isolated sub-second
    // solver gap from the last real solver output. Continuity outputs are
    // never remembered, so this path cannot recursively dead-reckon.
    if (!continuity.valid &&
        has_last_solution_position_ &&
        has_last_epoch_) {
        anchor_age_s = time - last_epoch_time_;
        auto short_gap_config = rtk_config_.safe_float_continuity;
        short_gap_config.maximum_anchor_age_s =
            std::min(
                short_gap_config.maximum_anchor_age_s,
                short_gap_config.maximum_solver_gap_anchor_age_s);
        continuity = safe_float_continuity::propagate(
            short_gap_config,
            last_solution_position_,
            anchor_age_s,
            last_doppler_velocity_ecef_,
            velocity_age_s);
        solver_gap_anchor = continuity.valid;
    }
    if (!has_last_doppler_velocity_ || !continuity.valid) {
        return solution;
    }

    solution.status = SolutionStatus::FLOAT;
    solution.position_ecef = continuity.position_ecef;
    solution.position_geodetic =
        spp_utils::ecefToGeodetic(solution.position_ecef);
    solution.position_covariance =
        Matrix3d::Identity() * continuity.position_variance_m2;
    solution.velocity_ecef = last_doppler_velocity_ecef_;
    solution.velocity_covariance =
        Matrix3d::Identity() *
        std::pow(
            rtk_config_.safe_float_continuity.velocity_sigma_mps,
            2.0);
    solution.has_velocity = true;
    // Four is the minimum valid FLOAT output and deliberately prevents
    // rememberSolution() from refreshing trust.
    solution.num_satellites = 4;
    solution.ratio = 0.0;
    solution.num_fixed_ambiguities = 0;
    debug_telemetry_.safe_float_continuity_used = true;
    debug_telemetry_.safe_float_continuity_solver_gap_anchor =
        solver_gap_anchor;
    debug_telemetry_.safe_float_continuity_anchor_age_s =
        anchor_age_s;
    debug_telemetry_.safe_float_continuity_velocity_age_s =
        velocity_age_s;
    return solution;
}

void RTKProcessor::updateStatistics(SolutionStatus status) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    const_cast<size_t&>(total_epochs_processed_)++;
    if (status == SolutionStatus::FIXED) const_cast<size_t&>(fixed_solutions_)++;
    else if (status == SolutionStatus::FLOAT) const_cast<size_t&>(float_solutions_)++;
}

// ============================================================
// LAMBDA - C++ implementation (ported from RTKLIB lambda.c)
// ============================================================
bool RTKProcessor::lambdaMethod(const VectorXd& float_ambiguities, const MatrixXd& covariance,
    VectorXd& fixed_ambiguities, double& success_rate) {
    int n = float_ambiguities.size();
    if (n == 0 || n > MAXSAT * 2) return false;

    if (rtk_config_.ar_policy == RTKConfig::ARPolicy::DEMO5_CONTINUOUS) {
        // demo5-continuous: pass raw covariance directly to LAMBDA (no regularization)
        return lambdaSearch(float_ambiguities, covariance, fixed_ambiguities, success_rate);
    }

    // Regularize covariance to ensure positive-definiteness for LAMBDA
    MatrixXd Q_reg = covariance;
    Q_reg = (Q_reg + Q_reg.transpose()) * 0.5;

    // Ensure minimum diagonal values
    constexpr double MIN_VAR = 1e-6;
    for (int i = 0; i < n; ++i) {
        if (Q_reg(i, i) < MIN_VAR) Q_reg(i, i) = MIN_VAR;
    }

    // Prefer cheap diagonal ridge retries over an eigensolver fallback.
    double ridge = 0.0;
    Eigen::LLT<MatrixXd> llt;
    bool ok = false;
    for (int attempt = 0; attempt < 5; ++attempt) {
        MatrixXd candidate = Q_reg;
        if (ridge > 0.0) {
            candidate.diagonal().array() += ridge;
        }
        llt.compute(candidate);
        if (llt.info() == Eigen::Success) {
            Q_reg.swap(candidate);
            ok = true;
            break;
        }
        ridge = (ridge == 0.0) ? MIN_VAR : ridge * 10.0;
    }
    if (!ok) {
        Eigen::SelfAdjointEigenSolver<MatrixXd> eig(Q_reg);
        if (eig.info() != Eigen::Success) {
            return false;
        }
        const double min_eig = eig.eigenvalues().minCoeff();
        if (!std::isfinite(min_eig)) {
            return false;
        }
        if (min_eig < MIN_VAR) {
            Q_reg.diagonal().array() += (MIN_VAR - min_eig);
        }
    }

    return lambdaSearch(float_ambiguities, Q_reg, fixed_ambiguities, success_rate);
}

// Old updateFilter signature stub
bool RTKProcessor::updateFilter(const ObservationData&, const ObservationData&, const NavigationData&) {
    return updateFilter(current_sat_data_);
}

// Legacy stubs
std::vector<RTKProcessor::DoubleDifference> RTKProcessor::formDoubleDifferences(
    const ObservationData& rover_obs, const ObservationData& base_obs, const NavigationData& nav,
    const Vector3d* evaluation_rover_position_ecef) {
    if (!base_position_known_) {
        if (base_obs.receiver_position.norm() > 1e6) {
            const_cast<RTKProcessor*>(this)->setBasePosition(base_obs.receiver_position);
        } else {
            return {};
        }
    }

    if (!filter_initialized_) {
        if (!const_cast<RTKProcessor*>(this)->initializeFilter(rover_obs, base_obs, nav)) {
            return {};
        }
    }

    const bool reuse_processed_epoch = !current_sat_data_.empty() &&
        std::abs(current_epoch_time_ - rover_obs.time) <= 1e-6;
    const auto sat_data = reuse_processed_epoch
        ? current_sat_data_
        : const_cast<RTKProcessor*>(this)->collectSatelliteData(rover_obs, base_obs, nav);
    if (sat_data.size() < 4) {
        return {};
    }

    if (!reuse_processed_epoch) {
        const_cast<RTKProcessor*>(this)->current_sat_data_ = sat_data;
        const double bias_dt =
            has_last_epoch_ ? std::max(rover_obs.time - last_epoch_time_, 1e-3) : 1.0;
        const_cast<RTKProcessor*>(this)->updateBias(sat_data, bias_dt);
    }

    std::vector<DoubleDifference> measurements;
    const auto dd_pairs = buildDoubleDifferencePairs(sat_data, 0);
    if (dd_pairs.empty()) {
        return measurements;
    }

    const Vector3d rover_pos = evaluation_rover_position_ecef != nullptr
        ? *evaluation_rover_position_ecef
        : base_position_ + filter_state_.state.head<3>();
    for (const auto& pair : dd_pairs) {
        const auto ref_it = sat_data.find(pair.ref_sat);
        const auto sat_it = sat_data.find(pair.sat);
        if (ref_it == sat_data.end() || sat_it == sat_data.end()) {
            continue;
        }

        const auto& ref_sd = ref_it->second;
        const auto& sd = sat_it->second;
        const bool use_l1 = pair.freq == 0;
        if ((use_l1 && (!ref_sd.has_l1 || !sd.has_l1)) ||
            (!use_l1 && (!ref_sd.has_l2 || !sd.has_l2))) {
            continue;
        }

        const double ref_code = use_l1 ? ref_sd.rover_l1_code - ref_sd.base_l1_code
                                       : ref_sd.rover_l2_code - ref_sd.base_l2_code;
        const double sat_code = use_l1 ? sd.rover_l1_code - sd.base_l1_code
                                       : sd.rover_l2_code - sd.base_l2_code;
        const double ref_phase = use_l1 ? ref_sd.rover_l1_phase - ref_sd.base_l1_phase
                                        : ref_sd.rover_l2_phase - ref_sd.base_l2_phase;
        const double sat_phase = use_l1 ? sd.rover_l1_phase - sd.base_l1_phase
                                        : sd.rover_l2_phase - sd.base_l2_phase;

        const double rr_ref =
            geodist_range(ref_sd.sat_pos, rover_pos) + tropModel(rover_pos, ref_sd.elevation);
        const double br_ref = geodist_range(ref_sd.sat_pos_base, base_position_) +
                              tropModel(base_position_, ref_sd.base_elevation);
        const double rr =
            geodist_range(sd.sat_pos, rover_pos) + tropModel(rover_pos, sd.elevation);
        const double br = geodist_range(sd.sat_pos_base, base_position_) +
                          tropModel(base_position_, sd.base_elevation);

        DoubleDifference measurement;
        measurement.reference_satellite = pair.ref_sat;
        measurement.satellite = pair.sat;
        measurement.signal = use_l1 ? sd.l1_signal : sd.l2_signal;
        measurement.pseudorange_dd = ref_code - sat_code;
        measurement.carrier_phase_dd = ref_phase - sat_phase;
        measurement.geometric_range = (rr_ref - br_ref) - (rr - br);
        measurement.unit_vector =
            -(ref_sd.sat_pos - rover_pos).normalized() + (sd.sat_pos - rover_pos).normalized();
        measurement.elevation = std::min(ref_sd.elevation, sd.elevation);
        const double ref_snr = use_l1
            ? combinedSnrDbHz(ref_sd.rover_l1_snr, ref_sd.base_l1_snr)
            : combinedSnrDbHz(ref_sd.rover_l2_snr, ref_sd.base_l2_snr);
        const double sat_snr = use_l1
            ? combinedSnrDbHz(sd.rover_l1_snr, sd.base_l1_snr)
            : combinedSnrDbHz(sd.rover_l2_snr, sd.base_l2_snr);
        const double dd_snr = combinedSnrDbHz(ref_snr, sat_snr);
        measurement.variance = varerr(measurement.elevation, true, dd_snr);
        measurement.code_variance = varerr(measurement.elevation, false, dd_snr);
        measurement.wavelength = use_l1 ? sd.l1_wavelength : sd.l2_wavelength;
        measurement.frequency_index = pair.freq;
        const auto& locks = use_l1 ? lock_count_l1_ : lock_count_l2_;
        const auto sat_lock = locks.find(pair.sat);
        const auto ref_lock = locks.find(pair.ref_sat);
        measurement.lock_count = std::min(
            sat_lock == locks.end() ? 0 : sat_lock->second,
            ref_lock == locks.end() ? 0 : ref_lock->second);
        measurement.cycle_slip =
            (use_l1 ? (sd.l1_lli | ref_sd.l1_lli) : (sd.l2_lli | ref_sd.l2_lli)) != 0 ||
            measurement.lock_count <= 0;
        measurement.valid = true;
        measurements.push_back(std::move(measurement));
    }

    return measurements;
}

std::vector<dd_imu_bridge::DDObservation> RTKProcessor::formTightlyCoupledObservations(
    const ObservationData& rover_obs, const ObservationData& base_obs,
    const NavigationData& nav, const Vector3d& rover_position_ecef,
    const Matrix3d& ecef_to_enu, const Eigen::Quaterniond& attitude_body_to_enu) {
    std::vector<dd_imu_bridge::DDObservation> rows;
    if (!rover_position_ecef.allFinite() || !ecef_to_enu.allFinite() ||
        !attitude_body_to_enu.coeffs().allFinite()) {
        return rows;
    }
    const auto measurements = formDoubleDifferences(
        rover_obs, base_obs, nav, &rover_position_ecef);
    rows.reserve(measurements.size());
    for (const auto& measurement : measurements) {
        if (!measurement.valid || !(measurement.wavelength > 0.0) ||
            !std::isfinite(measurement.geometric_range)) {
            continue;
        }
        dd_imu_bridge::DDObservation row;
        row.key.satellite_prn = measurement.satellite.prn;
        row.key.frequency_index = measurement.frequency_index;
        row.key.satellite_system = static_cast<int>(measurement.satellite.system);
        row.key.reference_satellite_prn = measurement.reference_satellite.prn;
        row.key.reference_satellite_system =
            static_cast<int>(measurement.reference_satellite.system);
        row.key.signal_type = static_cast<int>(measurement.signal);
        const Vector3d geometry_enu = ecef_to_enu * measurement.unit_vector;
        row.geometry_enu = geometry_enu.transpose();
        row.code_residual_m = measurement.pseudorange_dd - measurement.geometric_range;
        row.code_variance_m2 = measurement.code_variance;
        row.carrier_residual_m =
            measurement.carrier_phase_dd * measurement.wavelength -
            measurement.geometric_range;
        // A newly formed DD arc can appear numerically precise while its
        // reference relationship and slip history are not yet stable. Keep
        // code rows active immediately, but delay carrier/ambiguity state
        // augmentation until roughly one minute at 5 Hz.
        constexpr int kTightCarrierMinLockCount = 300;
        row.carrier_variance_m2 = measurement.lock_count >= kTightCarrierMinLockCount
            ? measurement.variance
            : 0.0;
        row.wavelength_m = measurement.wavelength;
        row.elevation_rad = measurement.elevation;
        const Vector3d geometry_body = attitude_body_to_enu.conjugate() * geometry_enu;
        row.body_azimuth_rad = std::atan2(geometry_body.y(), geometry_body.x());
        const double ambiguity_cycles = row.carrier_residual_m / row.wavelength_m;
        row.posterior_abs_residual_m = row.wavelength_m *
            std::abs(ambiguity_cycles - std::round(ambiguity_cycles));
        row.lock_count = measurement.lock_count;
        row.cycle_slip = measurement.cycle_slip;
        rows.push_back(std::move(row));
    }
    return rows;
}
void RTKProcessor::detectCycleSlips(const ObservationData&, const ObservationData&) {}
bool RTKProcessor::resolveAmbiguities(int) { return resolveAmbiguities(); }
RTKProcessor::LAMBDAResult RTKProcessor::solveLAMBDA(const VectorXd& float_ambiguities,
                                                     const MatrixXd& ambiguity_covariance) {
    LAMBDAResult result;
    result.success = lambdaMethod(float_ambiguities, ambiguity_covariance,
                                  result.fixed_ambiguities, result.ratio);
    return result;
}
bool RTKProcessor::validateAmbiguityResolution(const VectorXd& fixed_ambiguities,
                                               const VectorXd& float_ambiguities,
                                               const MatrixXd& covariance,
                                               double ratio) {
    if (fixed_ambiguities.size() == 0 || fixed_ambiguities.size() != float_ambiguities.size()) {
        return false;
    }
    if (covariance.rows() != covariance.cols() ||
        covariance.rows() != fixed_ambiguities.size()) {
        return false;
    }
    if (!std::isfinite(ratio) || ratio < rtk_config_.ambiguity_ratio_threshold) {
        return false;
    }

    for (int i = 0; i < fixed_ambiguities.size(); ++i) {
        if (!std::isfinite(fixed_ambiguities(i)) || !std::isfinite(float_ambiguities(i))) {
            return false;
        }
        if (std::abs(fixed_ambiguities(i) - std::round(fixed_ambiguities(i))) > 1e-6) {
            return false;
        }
    }

    MatrixXd Q = (covariance + covariance.transpose()) * 0.5;
    for (int i = 0; i < Q.rows(); ++i) {
        if (!std::isfinite(Q(i, i)) || Q(i, i) <= 0.0) {
            return false;
        }
    }
    Eigen::LDLT<MatrixXd> solver(Q);
    if (solver.info() != Eigen::Success) {
        return false;
    }
    const VectorXd diff = float_ambiguities - fixed_ambiguities;
    const double norm = diff.transpose() * solver.solve(diff);
    return std::isfinite(norm) && norm >= 0.0;
}
void RTKProcessor::updateAmbiguityStates(const std::vector<DoubleDifference>&) {}
Vector3d RTKProcessor::calculateBaseline() const {
    if (!filter_initialized_ || filter_state_.state.size() < 3) {
        return Vector3d::Zero();
    }
    return filter_state_.state.head<3>();
}
VectorXd RTKProcessor::calculateResiduals(const std::vector<DoubleDifference>& measurements,
                                          const Vector3d& baseline) const {
    VectorXd residuals = VectorXd::Zero(static_cast<int>(measurements.size()));
    for (int i = 0; i < static_cast<int>(measurements.size()); ++i) {
        const auto& measurement = measurements[i];
        const double observed =
            std::abs(measurement.pseudorange_dd) > 0.0 ? measurement.pseudorange_dd
                                                       : measurement.carrier_phase_dd;
        const double predicted =
            measurement.geometric_range + measurement.unit_vector.dot(baseline);
        residuals(i) = observed - predicted;
    }
    return residuals;
}
MatrixXd RTKProcessor::formMeasurementMatrix(const std::vector<DoubleDifference>& measurements,
                                             const NavigationData&,
                                             const GNSSTime&) const {
    MatrixXd H = MatrixXd::Zero(static_cast<int>(measurements.size()), 3);
    for (int i = 0; i < static_cast<int>(measurements.size()); ++i) {
        H.row(i) = measurements[i].unit_vector.transpose();
    }
    return H;
}
MatrixXd RTKProcessor::calculateMeasurementWeights(
    const std::vector<DoubleDifference>& measurements) const {
    MatrixXd W = MatrixXd::Zero(static_cast<int>(measurements.size()),
                                static_cast<int>(measurements.size()));
    for (int i = 0; i < static_cast<int>(measurements.size()); ++i) {
        const double variance = std::max(measurements[i].variance, 1e-6);
        W(i, i) = 1.0 / variance;
    }
    return W;
}
bool RTKProcessor::hasSufficientSatellites(
    const std::vector<DoubleDifference>& measurements) const {
    std::set<SatelliteId> satellites;
    for (const auto& measurement : measurements) {
        satellites.insert(measurement.reference_satellite);
        satellites.insert(measurement.satellite);
    }
    return satellites.size() >= 4;
}
void RTKProcessor::resetAmbiguity(const SatelliteId&, SignalType) {}
bool RTKProcessor::applyFixedAmbiguities(const VectorXd& fixed_n1,
                                         const VectorXd& fixed_n2,
                                         const std::map<SatelliteId, SatelliteData>& sat_data) {
    if (!filter_initialized_) {
        return false;
    }

    std::vector<SatelliteId> satellites;
    for (const auto& [sat, sd] : sat_data) {
        if (sd.has_l1 || sd.has_l2) {
            satellites.push_back(sat);
        }
    }
    std::sort(satellites.begin(), satellites.end());

    int n1_applied = 0;
    int n2_applied = 0;
    for (const auto& sat : satellites) {
        if (n1_applied < fixed_n1.size()) {
            auto it = filter_state_.n1_indices.find(sat);
            if (it != filter_state_.n1_indices.end()) {
                filter_state_.state(it->second) = fixed_n1(n1_applied++);
            }
        }
        if (n2_applied < fixed_n2.size()) {
            auto it = filter_state_.n2_indices.find(sat);
            if (it != filter_state_.n2_indices.end()) {
                filter_state_.state(it->second) = fixed_n2(n2_applied++);
            }
        }
    }
    return n1_applied == fixed_n1.size() && n2_applied == fixed_n2.size();
}
void RTKProcessor::solvePositionWithAmbiguities(const std::map<SatelliteId, SatelliteData>&) {
    if (has_fixed_solution_ && filter_initialized_ && filter_state_.state.size() >= 3) {
        filter_state_.state.head<3>() = fixed_baseline_;
    }
}
bool RTKProcessor::trySingleEpochAR(const std::map<SatelliteId, SatelliteData>& sat_data) {
    if (!filter_initialized_) {
        return false;
    }
    current_sat_data_ = sat_data;
    if (!resolveAmbiguities(buildDoubleDifferencePairs(sat_data, 0))) {
        return false;
    }
    return has_fixed_solution_ && validateFixedSolution(sat_data, last_epoch_time_);
}

} // namespace libgnss
