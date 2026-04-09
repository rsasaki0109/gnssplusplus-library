// CLAS-PPP epoch processing method for PPPProcessor.
// Split from ppp.cpp for modularity.

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/kalman.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>

#include <cmath>
#include <iostream>

namespace libgnss {

using PPPConfig = ppp_shared::PPPConfig;
using PPPState = ppp_shared::PPPState;
using PPPAmbiguityInfo = ppp_shared::PPPAmbiguityInfo;

namespace {

bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

}  // namespace

PositionSolution PPPProcessor::processEpochCLAS(const ObservationData& obs,
                                                 const NavigationData& nav) {
    PositionSolution solution;
    solution.time = obs.time;
    solution.status = SolutionStatus::NONE;
    last_clas_hybrid_fallback_used_ = false;
    last_clas_hybrid_fallback_reason_.clear();
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;
    // CLAS per-frequency mode uses WL-NL AR: MW averaging resolves WL integers,
    // then NL integers are extracted from OSR-corrected dual-freq observations.
    if (ppp_config_.enable_ambiguity_resolution && !ppp_config_.use_ionosphere_free) {
        ppp_config_.ar_method = PPPConfig::ARMethod::DD_WLNL;
        // CLASLIB attempts WL fixing without a hard multi-epoch warmup.
        // Let the DD-WL/NL ratio test decide from the first epoch onward.
        if (ppp_config_.wl_min_averaging_epochs > 1) {
            ppp_config_.wl_min_averaging_epochs = 1;
        }
    }

    struct ClasFallbackSnapshot {
        PPPState filter_state;
        bool filter_initialized = false;
        GNSSTime convergence_start_time;
        Vector3d static_anchor_position = Vector3d::Zero();
        bool has_static_anchor_position = false;
        std::map<SatelliteId, PPPAmbiguityInfo> ambiguity_states;
        std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion_compensation;
        std::map<SatelliteId, CLASSisContinuityInfo> sis_continuity;
        std::map<SatelliteId, double> windup_cache;
        std::map<SatelliteId, CLASPhaseBiasRepairInfo> phase_bias_repair;
        bool has_last_processed_time = false;
        GNSSTime last_processed_time;
    };
    const ClasFallbackSnapshot fallback_snapshot{
        filter_state_,
        filter_initialized_,
        convergence_start_time_,
        static_anchor_position_,
        has_static_anchor_position_,
        ambiguity_states_,
        clas_dispersion_compensation_,
        clas_sis_continuity_,
        windup_cache_,
        clas_phase_bias_repair_,
        has_last_processed_time_,
        last_processed_time_,
    };
    const auto restore_clas_snapshot = [&]() {
        filter_state_ = fallback_snapshot.filter_state;
        filter_initialized_ = fallback_snapshot.filter_initialized;
        convergence_start_time_ = fallback_snapshot.convergence_start_time;
        static_anchor_position_ = fallback_snapshot.static_anchor_position;
        has_static_anchor_position_ = fallback_snapshot.has_static_anchor_position;
        ambiguity_states_ = fallback_snapshot.ambiguity_states;
        clas_dispersion_compensation_ = fallback_snapshot.dispersion_compensation;
        clas_sis_continuity_ = fallback_snapshot.sis_continuity;
        windup_cache_ = fallback_snapshot.windup_cache;
        clas_phase_bias_repair_ = fallback_snapshot.phase_bias_repair;
        has_last_processed_time_ = fallback_snapshot.has_last_processed_time;
        last_processed_time_ = fallback_snapshot.last_processed_time;
    };
    const bool allow_hybrid_fallback =
        ppp_config_.clas_epoch_policy ==
        PPPConfig::ClasEpochPolicy::HYBRID_STANDARD_PPP_FALLBACK;
    const auto fallback_to_standard = [&](const char* reason) {
        restore_clas_snapshot();
        return processEpochStandard(obs, nav, reason);
    };

    PositionSolution seed = spp_processor_.processEpoch(obs, nav);
    detectCycleSlips(obs);
    const double ambiguity_reset_variance =
        ppp_config_.initial_ambiguity_variance <
                ppp_config_.clas_ambiguity_reinit_threshold * 0.1
            ? ppp_config_.initial_ambiguity_variance
            : ppp_config_.clas_ambiguity_reinit_threshold * 0.1;
    const auto epoch_preparation = ppp_clas::prepareEpochState(
        obs,
        seed,
        ssr_products_,
        filter_state_,
        filter_initialized_,
        convergence_start_time_,
        static_anchor_position_,
        has_static_anchor_position_,
        ppp_config_,
        modeledZenithTroposphereDelayMeters(seed.position_ecef, obs.time),
        has_last_processed_time_,
        last_processed_time_,
        ambiguity_states_,
        clas_dispersion_compensation_,
        clas_phase_bias_repair_,
        ambiguity_reset_variance);
    if (!epoch_preparation.ready) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("prepare_epoch_state");
        }
        return solution;
    }

    const auto epoch_context = prepareClasEpochContext(
        obs,
        nav,
        ssr_products_,
        filter_state_.state.segment(0, 3),
        filter_state_.state(filter_state_.clock_index),
        filter_state_.state(filter_state_.trop_index),
        ppp_config_,
        windup_cache_,
        clas_dispersion_compensation_,
        clas_sis_continuity_,
        clas_phase_bias_repair_);
    const auto& epoch_atmos = epoch_context.epoch_atmos_tokens;
    const auto& osr_corrections = epoch_context.osr_corrections;

    if (osr_corrections.size() < 4) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("insufficient_osr");
        }
        solution = seed;
        return solution;
    }

    ppp_clas::ensureAmbiguityStates(filter_state_, osr_corrections);
    ppp_clas::applyPendingPhaseBiasStateShifts(
        filter_state_, osr_corrections, clas_phase_bias_repair_, pppDebugEnabled());

    const auto epoch_update = ppp_clas::runEpochMeasurementUpdate(
        obs,
        epoch_context,
        filter_state_,
        ppp_config_,
        seed,
        ambiguity_states_,
        [&](const Vector3d& receiver_pos, double elevation, const GNSSTime& time) {
            return calculateMappingFunction(receiver_pos, elevation, time);
        },
        [&](const SatelliteId& satellite, SignalType signal) {
            resetAmbiguity(satellite, signal);
        },
        [&](const SatelliteId& satellite) {
            return ambiguityStateIndex(satellite);
        },
        pppDebugEnabled());
    if (!epoch_update.updated) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("measurement_update");
        }
        solution = seed;
        return solution;
    }
    const auto& update_stats = epoch_update.update_stats;
    pre_anchor_covariance_ = update_stats.pre_anchor_covariance;

    // Accumulate Melbourne-Wübbena for WL-NL AR in CLAS per-frequency mode.
    if (ppp_config_.enable_ambiguity_resolution &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
        for (const auto& osr : osr_corrections) {
            if (!osr.valid || osr.num_frequencies < 2) continue;
            const Observation* l1_raw = obs.getObservation(osr.satellite, osr.signals[0]);
            const Observation* l2_raw = obs.getObservation(osr.satellite, osr.signals[1]);
            if (!l1_raw || !l2_raw || !l1_raw->valid || !l2_raw->valid) continue;
            if (!l1_raw->has_carrier_phase || !l2_raw->has_carrier_phase) continue;
            if (!l1_raw->has_pseudorange || !l2_raw->has_pseudorange) continue;
            const double f1 = osr.frequencies[0];
            const double f2 = osr.frequencies[1];
            if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1e6) continue;
            const double l1_m = l1_raw->carrier_phase * osr.wavelengths[0]
                              - osr.phase_bias_m[0];
            const double l2_m = l2_raw->carrier_phase * osr.wavelengths[1]
                              - osr.phase_bias_m[1];
            const double p1 = l1_raw->pseudorange - osr.code_bias_m[0];
            const double p2 = l2_raw->pseudorange - osr.code_bias_m[1];
            const double mw_m = (f1 * l1_m - f2 * l2_m) / (f1 - f2)
                              - (f1 * p1 + f2 * p2) / (f1 + f2);
            const double lambda_wl = constants::SPEED_OF_LIGHT / std::abs(f1 - f2);
            if (lambda_wl <= 0.0 || !std::isfinite(lambda_wl)) {
                continue;
            }
            const double mw_cycles = mw_m / lambda_wl;
            auto& amb = ambiguity_states_[osr.satellite];
            const bool mw_slip = l1_raw->loss_of_lock || l2_raw->loss_of_lock;
            if (!mw_slip && amb.mw_count > 0) {
                amb.mw_sum_cycles += mw_cycles;
                amb.mw_count += 1;
                amb.mw_mean_cycles = amb.mw_sum_cycles / amb.mw_count;
            } else {
                amb.mw_sum_cycles = mw_cycles;
                amb.mw_count = 1;
                amb.mw_mean_cycles = mw_cycles;
                amb.wl_is_fixed = false;
            }
        }
    }

    const auto trop_mapping_for_validation =
        [&](const Vector3d& receiver_pos, double elevation, const GNSSTime& time) {
            return calculateMappingFunction(receiver_pos, elevation, time);
        };
    const auto ambiguity_index_for_validation = [&](const SatelliteId& satellite) {
        return ambiguityStateIndex(satellite);
    };

    const auto ambiguity_resolution =
        ppp_clas::resolveAndValidateAmbiguities(
            filter_state_,
            ambiguity_states_,
            [&]() {
                return ppp_config_.enable_ambiguity_resolution &&
                       resolveAmbiguities(obs, nav);
            },
            (ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL)
                ? ppp_clas::ValidateFixedSolutionFunction{}
                : ppp_clas::ValidateFixedSolutionFunction{[&]() {
                      return ppp_clas::validateFixedSolution(
                          obs, osr_corrections, filter_state_, ppp_config_,
                          trop_mapping_for_validation,
                          ambiguity_index_for_validation, pppDebugEnabled());
                  }},
            pppDebugEnabled());
    if (ambiguity_resolution.rejected_after_fix) {
        last_ar_ratio_ = 0.0;
        last_fixed_ambiguities_ = 0;
    }

    if (pppDebugEnabled()) {
        ppp_clas::logUpdateSummary(update_stats, osr_corrections.size());
    }
    bool wlnl_fixed_position_ok = false;
    Vector3d wlnl_fixed_position = Vector3d::Zero();
    if (ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
        const PPPState& base_state = filter_state_;
        auto& beta_state = clas_iflc_beta_state_;

        auto global_clock_index_for_system = [&](GNSSSystem system) {
            if (system == GNSSSystem::Galileo && base_state.gal_clock_index >= 0) {
                return base_state.gal_clock_index;
            }
            if (system == GNSSSystem::GLONASS && base_state.glo_clock_index >= 0) {
                return base_state.glo_clock_index;
            }
            return base_state.clock_index;
        };
        auto ensure_beta_size = [&](int new_size) {
            if (beta_state.state.size() >= new_size) {
                return;
            }
            const int old_size = beta_state.state.size();
            VectorXd expanded_state = VectorXd::Zero(new_size);
            MatrixXd expanded_covariance = MatrixXd::Zero(new_size, new_size);
            if (old_size > 0) {
                expanded_state.head(old_size) = beta_state.state;
                expanded_covariance.topLeftCorner(old_size, old_size) =
                    beta_state.covariance;
            }
            beta_state.state = std::move(expanded_state);
            beta_state.covariance = std::move(expanded_covariance);
        };
        if (!beta_state.initialized) {
            ensure_beta_size(3);
            beta_state.state.head(3) =
                base_state.state.segment(base_state.pos_index, 3);
            beta_state.covariance.topLeftCorner(3, 3) =
                MatrixXd::Identity(3, 3) *
                std::max(ppp_config_.clas_initial_position_variance, 1.0);
            beta_state.initialized = true;
        }

        struct LocalIfRow {
            Eigen::RowVectorXd H;
            double residual = 0.0;
            double variance = 0.0;
            bool is_phase = false;
            SatelliteId satellite{};
        };
        struct IfObservationSeed {
            const OSRCorrection* osr = nullptr;
            int clock_column = -1;
            int ambiguity_column = -1;
            int l1_ambiguity_index = -1;
            int l2_ambiguity_index = -1;
            double alpha1 = 0.0;
            double alpha2 = 0.0;
        };

        std::map<SatelliteId, IfObservationSeed> if_seeds;
        std::vector<LocalIfRow> local_rows;
        local_rows.reserve(osr_corrections.size() * 2);

        for (const auto& osr : osr_corrections) {
            if (!osr.valid || osr.num_frequencies < 2) {
                continue;
            }
            const Observation* l1 = obs.getObservation(osr.satellite, osr.signals[0]);
            const Observation* l2 = obs.getObservation(osr.satellite, osr.signals[1]);
            if (l1 == nullptr || l2 == nullptr ||
                !l1->valid || !l2->valid ||
                !l1->has_carrier_phase || !l2->has_carrier_phase ||
                !std::isfinite(l1->carrier_phase) || !std::isfinite(l2->carrier_phase) ||
                osr.wavelengths[0] <= 0.0 || osr.wavelengths[1] <= 0.0) {
                continue;
            }
            const double f1 = osr.frequencies[0];
            const double f2 = osr.frequencies[1];
            const double denom = f1 * f1 - f2 * f2;
            if (f1 <= 0.0 || f2 <= 0.0 || std::abs(denom) < 1.0) {
                continue;
            }
            const double alpha1 = (f1 * f1) / denom;
            const double alpha2 = -(f2 * f2) / denom;

            const int global_clock_index =
                global_clock_index_for_system(osr.satellite.system);
            auto clock_it = beta_state.clock_columns.find(osr.satellite.system);
            if (clock_it == beta_state.clock_columns.end()) {
                const int clock_column = beta_state.state.size();
                ensure_beta_size(clock_column + 1);
                beta_state.clock_columns.emplace(osr.satellite.system, clock_column);
                beta_state.state(clock_column) = base_state.state(global_clock_index);
                beta_state.covariance(clock_column, clock_column) =
                    std::max(ppp_config_.clas_clock_variance, 1.0);
                clock_it = beta_state.clock_columns.find(osr.satellite.system);
            } else {
                beta_state.covariance(clock_it->second, clock_it->second) =
                    std::max(beta_state.covariance(clock_it->second, clock_it->second) +
                                 std::max(ppp_config_.process_noise_clock, 100.0),
                             1e-6);
            }

            const SatelliteId l1_amb_sat(osr.satellite.system, osr.satellite.prn);
            const SatelliteId l2_amb_sat(
                osr.satellite.system,
                static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100)));
            const auto l1_amb_it = base_state.ambiguity_indices.find(l1_amb_sat);
            const auto l2_amb_it = base_state.ambiguity_indices.find(l2_amb_sat);
            if (l1_amb_it == base_state.ambiguity_indices.end() ||
                l2_amb_it == base_state.ambiguity_indices.end()) {
                continue;
            }

            auto amb_it = beta_state.ambiguity_columns.find(osr.satellite);
            if (amb_it == beta_state.ambiguity_columns.end()) {
                const int ambiguity_column = beta_state.state.size();
                ensure_beta_size(ambiguity_column + 1);
                beta_state.ambiguity_columns.emplace(osr.satellite, ambiguity_column);
                beta_state.state(ambiguity_column) =
                    alpha1 * base_state.state(l1_amb_it->second) +
                    alpha2 * base_state.state(l2_amb_it->second);
                beta_state.covariance(ambiguity_column, ambiguity_column) =
                    std::max(ppp_config_.initial_ambiguity_variance, 1.0);
                amb_it = beta_state.ambiguity_columns.find(osr.satellite);
            } else {
                beta_state.covariance(amb_it->second, amb_it->second) =
                    std::max(beta_state.covariance(amb_it->second, amb_it->second) +
                                 ppp_config_.process_noise_ambiguity,
                             1e-6);
            }
            beta_state.state(amb_it->second) =
                alpha1 * base_state.state(l1_amb_it->second) +
                alpha2 * base_state.state(l2_amb_it->second);

            if_seeds.emplace(osr.satellite,
                             IfObservationSeed{&osr,
                                               clock_it->second,
                                               amb_it->second,
                                               l1_amb_it->second,
                                               l2_amb_it->second,
                                               alpha1,
                                               alpha2});
        }
        PPPState measurement_state = base_state;
        measurement_state.state.segment(measurement_state.pos_index, 3) =
            beta_state.state.head(3);
        for (const auto& [system, clock_column] : beta_state.clock_columns) {
            const int global_clock_index = global_clock_index_for_system(system);
            if (global_clock_index >= 0 &&
                global_clock_index < measurement_state.state.size()) {
                measurement_state.state(global_clock_index) =
                    beta_state.state(clock_column);
            }
        }
        const auto beta_measurements = ppp_clas::buildEpochMeasurements(
            obs,
            osr_corrections,
            measurement_state,
            ppp_config_,
            measurement_state.state.segment(measurement_state.pos_index, 3),
            measurement_state.state(measurement_state.clock_index),
            measurement_state.state(measurement_state.trop_index),
            epoch_atmos,
            trop_mapping_for_validation,
            {},
            false);
        struct PerFreqRows {
            const ppp_clas::MeasurementRow* code[2]{nullptr, nullptr};
            const ppp_clas::MeasurementRow* phase[2]{nullptr, nullptr};
        };
        std::map<SatelliteId, PerFreqRows> rows_by_satellite;
        for (const auto& row : beta_measurements.measurements) {
            if (row.freq_index < 0 || row.freq_index >= 2) {
                continue;
            }
            auto& slot = rows_by_satellite[row.satellite];
            if (row.is_phase) {
                slot.phase[row.freq_index] = &row;
            } else {
                slot.code[row.freq_index] = &row;
            }
        }

        for (const auto& [satellite, seed] : if_seeds) {
            const auto rows_it = rows_by_satellite.find(satellite);
            if (rows_it == rows_by_satellite.end()) {
                continue;
            }
            const auto& rows = rows_it->second;

            auto build_row = [&](bool phase) {
                LocalIfRow row;
                row.H = Eigen::RowVectorXd::Zero(beta_state.state.size());
                if (phase) {
                    row.H.segment(0, 3) =
                        seed.alpha1 * rows.phase[0]->H.segment(0, 3) +
                        seed.alpha2 * rows.phase[1]->H.segment(0, 3);
                } else {
                    row.H.segment(0, 3) =
                        seed.alpha1 * rows.code[0]->H.segment(0, 3) +
                        seed.alpha2 * rows.code[1]->H.segment(0, 3);
                }
                row.H(seed.clock_column) = 1.0;
                row.is_phase = phase;
                row.satellite = satellite;
                return row;
            };

            if (rows.phase[0] == nullptr || rows.phase[1] == nullptr) {
                continue;
            }
            const double base_if_ambiguity =
                seed.alpha1 * measurement_state.state(seed.l1_ambiguity_index) +
                seed.alpha2 * measurement_state.state(seed.l2_ambiguity_index);
            const double ambiguity_gap =
                base_if_ambiguity - beta_state.state(seed.ambiguity_column);
            auto phase_row = build_row(true);
            phase_row.H(seed.ambiguity_column) = 1.0;
            phase_row.residual =
                seed.alpha1 * rows.phase[0]->residual +
                seed.alpha2 * rows.phase[1]->residual +
                ambiguity_gap;
            phase_row.variance =
                std::max(
                    seed.alpha1 * seed.alpha1 * rows.phase[0]->variance +
                        seed.alpha2 * seed.alpha2 * rows.phase[1]->variance,
                    1e-8);
            if (pppDebugEnabled() &&
                phase_row.satellite.system == GNSSSystem::GPS &&
                (phase_row.satellite.prn == 25 || phase_row.satellite.prn == 26 ||
                 phase_row.satellite.prn == 29 || phase_row.satellite.prn == 31)) {
                std::cerr << "[CLAS-IF-PHASE] sat="
                          << phase_row.satellite.toString()
                          << " if_raw="
                          << (seed.alpha1 * rows.phase[0]->residual +
                              seed.alpha2 * rows.phase[1]->residual)
                          << " amb_gap=" << ambiguity_gap
                          << " beta_amb=" << beta_state.state(seed.ambiguity_column)
                          << " base_if_amb=" << base_if_ambiguity
                          << " if_residual=" << phase_row.residual
                          << "\n";
            }
            local_rows.push_back(std::move(phase_row));
        }

        for (const auto& [system, clock_column] : beta_state.clock_columns) {
            const int global_clock_index = global_clock_index_for_system(system);
            if (global_clock_index < 0 || global_clock_index >= base_state.state.size()) {
                continue;
            }
            LocalIfRow row;
            row.H = Eigen::RowVectorXd::Zero(beta_state.state.size());
            row.H(clock_column) = 1.0;
            row.residual = base_state.state(global_clock_index) - beta_state.state(clock_column);
            row.variance = 1e4;
            row.is_phase = false;
            local_rows.push_back(std::move(row));
        }

        bool fixed_solved = false;
        double fixed_shift = 0.0;
        double fixed_phase_rms = 0.0;
        double fixed_constraint_mean_abs = 1e9;
        double fixed_constraint_max_abs = 1e9;
        int fixed_constraint_rows = 0;
        int minimum_constraint_rows = 3;
        VectorXd linearized_delta_debug;
        bool have_linearized_delta_debug = false;

        if (local_rows.size() >= 6 && beta_state.state.size() >= 4) {
            const VectorXd linearization_state = beta_state.state;
            const int nrows = static_cast<int>(local_rows.size());
            const int nstate = beta_state.state.size();
            MatrixXd H = MatrixXd::Zero(nrows, nstate);
            MatrixXd R = MatrixXd::Zero(nrows, nrows);
            VectorXd v = VectorXd::Zero(nrows);
            for (int i = 0; i < nrows; ++i) {
                H.row(i) = local_rows[static_cast<size_t>(i)].H;
                R(i, i) = local_rows[static_cast<size_t>(i)].variance;
                v(i) = local_rows[static_cast<size_t>(i)].residual;
            }
            const MatrixXd S = H * beta_state.covariance * H.transpose() + R;
            Eigen::LDLT<MatrixXd> s_ldlt(S);
            if (s_ldlt.info() == Eigen::Success) {
                const MatrixXd K =
                    beta_state.covariance * H.transpose() *
                    s_ldlt.solve(MatrixXd::Identity(nrows, nrows));
                beta_state.state += K * v;
                beta_state.covariance =
                    (MatrixXd::Identity(nstate, nstate) - K * H) * beta_state.covariance;
            }
            const Vector3d pre_constraint_position = beta_state.state.head(3);
            if (ambiguity_resolution.accepted) {
                std::vector<Eigen::RowVectorXd> constraint_H_rows;
                std::vector<double> constraint_residuals;
                std::vector<GNSSSystem> constraint_systems;
                for (const auto& dd_constraint : wlnl_dd_constraints_) {
                    const auto ref_it = if_seeds.find(dd_constraint.ref_satellite);
                    const auto sat_it = if_seeds.find(dd_constraint.sat_satellite);
                    if (ref_it == if_seeds.end() || sat_it == if_seeds.end()) {
                        continue;
                    }
                    const auto& sat_osr = *sat_it->second.osr;
                    const double f1 = sat_osr.frequencies[0];
                    const double f2 = sat_osr.frequencies[1];
                    const double denom = f1 * f1 - f2 * f2;
                    if (std::abs(denom) < 1.0) {
                        continue;
                    }
                    const double lambda1 = sat_osr.wavelengths[0];
                    const double lambda2 = sat_osr.wavelengths[1];
                    const double lambda_nl = constants::SPEED_OF_LIGHT / (f1 + f2);
                    const double lambda_wl =
                        constants::SPEED_OF_LIGHT / std::abs(f1 - f2);
                    const double beta = f1 * f2 / denom;
                    if (lambda1 <= 0.0 || lambda2 <= 0.0 ||
                        lambda_nl <= 0.0 || lambda_wl <= 0.0 ||
                        std::abs(beta * lambda_wl) < 1e-9 ||
                        std::abs(lambda2 * lambda2 - lambda1 * lambda1) < 1e-9) {
                        continue;
                    }
                    const double n_nl =
                        (dd_constraint.l1_dd_ambiguity_m +
                         dd_constraint.l2_dd_ambiguity_m) /
                        (2.0 * lambda_nl);
                    const double n_wl =
                        (dd_constraint.l1_dd_ambiguity_m -
                         dd_constraint.l2_dd_ambiguity_m) /
                        (2.0 * beta * lambda_wl);
                    const double n1 = 0.5 * (n_nl + n_wl);
                    const double n2 = 0.5 * (n_nl - n_wl);
                    const double c1 =
                        (lambda2 * lambda2) /
                        (lambda2 * lambda2 - lambda1 * lambda1);
                    const double c2 =
                        -(lambda1 * lambda1) /
                        (lambda2 * lambda2 - lambda1 * lambda1);
                    const double fixed_dd_if_m =
                        c1 * lambda1 * n1 + c2 * lambda2 * n2;
                    Eigen::RowVectorXd h =
                        Eigen::RowVectorXd::Zero(beta_state.state.size());
                    h(ref_it->second.ambiguity_column) = 1.0;
                    h(sat_it->second.ambiguity_column) = -1.0;
                    const double float_dd_if_m =
                        beta_state.state(ref_it->second.ambiguity_column) -
                        beta_state.state(sat_it->second.ambiguity_column);
                    const double dd_if_residual = fixed_dd_if_m - float_dd_if_m;
                    if (pppDebugEnabled() && constraint_H_rows.size() < 3) {
                        std::cerr << "[CLAS-WLNL-BETA] ref="
                                  << dd_constraint.ref_satellite.toString()
                                  << " sat=" << dd_constraint.sat_satellite.toString()
                                  << " fixed_if=" << fixed_dd_if_m
                                  << " float_if=" << float_dd_if_m
                                  << " seed_dd=0\n";
                    }
                    constraint_H_rows.push_back(std::move(h));
                    constraint_residuals.push_back(dd_if_residual);
                    constraint_systems.push_back(dd_constraint.sat_satellite.system);
                }

                if (constraint_residuals.size() >= 3) {
                    int gps_constraints = 0;
                    double max_gps_abs_residual = 0.0;
                    int non_gps_outlier_index = -1;
                    double non_gps_outlier_abs_residual = 0.0;
                    for (size_t i = 0; i < constraint_residuals.size(); ++i) {
                        const double abs_residual = std::abs(constraint_residuals[i]);
                        if (constraint_systems[i] == GNSSSystem::GPS) {
                            ++gps_constraints;
                            max_gps_abs_residual =
                                std::max(max_gps_abs_residual, abs_residual);
                        } else if (abs_residual > non_gps_outlier_abs_residual) {
                            non_gps_outlier_abs_residual = abs_residual;
                            non_gps_outlier_index = static_cast<int>(i);
                        }
                    }
                    if (gps_constraints >= 2 &&
                        non_gps_outlier_index >= 0 &&
                        non_gps_outlier_abs_residual > 0.5 &&
                        non_gps_outlier_abs_residual >
                            2.0 * std::max(max_gps_abs_residual, 1e-6)) {
                        constraint_H_rows.erase(
                            constraint_H_rows.begin() + non_gps_outlier_index);
                        constraint_residuals.erase(
                            constraint_residuals.begin() + non_gps_outlier_index);
                        constraint_systems.erase(
                            constraint_systems.begin() + non_gps_outlier_index);
                        minimum_constraint_rows = 2;
                        if (pppDebugEnabled()) {
                            std::cerr << "[CLAS-WLNL-FIX] drop non-gps outlier="
                                      << non_gps_outlier_abs_residual << "\n";
                        }
                    }
                }

                fixed_constraint_rows = static_cast<int>(constraint_H_rows.size());
                if (fixed_constraint_rows > 0) {
                    double sum_abs = 0.0;
                    double max_abs = 0.0;
                    for (const double residual : constraint_residuals) {
                        const double abs_residual = std::abs(residual);
                        sum_abs += abs_residual;
                        max_abs = std::max(max_abs, abs_residual);
                    }
                    fixed_constraint_mean_abs =
                        sum_abs / static_cast<double>(fixed_constraint_rows);
                    fixed_constraint_max_abs = max_abs;
                }
                if (fixed_constraint_rows >= minimum_constraint_rows) {
                    MatrixXd Hc = MatrixXd::Zero(fixed_constraint_rows, nstate);
                    MatrixXd Rc =
                        MatrixXd::Identity(fixed_constraint_rows, fixed_constraint_rows) *
                        1e-6;
                    VectorXd vc = VectorXd::Zero(fixed_constraint_rows);
                    for (int i = 0; i < fixed_constraint_rows; ++i) {
                        Hc.row(i) = constraint_H_rows[static_cast<size_t>(i)];
                        vc(i) = constraint_residuals[static_cast<size_t>(i)];
                    }
                    const MatrixXd Sc =
                        Hc * beta_state.covariance * Hc.transpose() + Rc;
                    Eigen::LDLT<MatrixXd> sc_ldlt(Sc);
                    if (sc_ldlt.info() == Eigen::Success) {
                        const MatrixXd Kc =
                            beta_state.covariance * Hc.transpose() *
                            sc_ldlt.solve(MatrixXd::Identity(
                                fixed_constraint_rows, fixed_constraint_rows));
                        beta_state.state += Kc * vc;
                        beta_state.covariance =
                            (MatrixXd::Identity(nstate, nstate) - Kc * Hc) *
                            beta_state.covariance;
                    }
                }
                fixed_shift =
                    (beta_state.state.head(3) - pre_constraint_position).norm();
                wlnl_fixed_position = beta_state.state.head(3);
            }

            double phase_sum_sq = 0.0;
            int phase_count = 0;
            const VectorXd linearized_delta = beta_state.state - linearization_state;
            for (const auto& row : local_rows) {
                if (!row.is_phase) {
                    continue;
                }
                const double postfit = row.residual - row.H.dot(linearized_delta);
                phase_sum_sq += postfit * postfit;
                ++phase_count;
            }
            if (phase_count > 0) {
                fixed_phase_rms = std::sqrt(phase_sum_sq / phase_count);
            }
            linearized_delta_debug = linearized_delta;
            have_linearized_delta_debug = true;

            fixed_solved =
                ambiguity_resolution.accepted &&
                std::isfinite(fixed_shift) &&
                std::isfinite(fixed_phase_rms) &&
                fixed_constraint_rows >= minimum_constraint_rows &&
                fixed_shift < 10.0 &&
                fixed_phase_rms < 0.40 &&
                fixed_constraint_mean_abs < 1.5 &&
                fixed_constraint_max_abs < 2.0;
        }
        if (pppDebugEnabled()) {
            std::cerr << "[CLAS-WLNL-FIX] rows=" << local_rows.size()
                      << " constraints=" << fixed_constraint_rows
                      << " solved=" << (fixed_solved ? 1 : 0)
                      << " pos_shift=" << fixed_shift
                      << " phase_rms=" << fixed_phase_rms
                      << " dd_mean=" << fixed_constraint_mean_abs
                      << " dd_max=" << fixed_constraint_max_abs
                      << "\n";
            if (ambiguity_resolution.accepted && have_linearized_delta_debug) {
                int logged_phase_rows = 0;
                for (const auto& row : local_rows) {
                    if (!row.is_phase) {
                        continue;
                    }
                    const double postfit =
                        row.residual - row.H.dot(linearized_delta_debug);
                    std::cerr << "[CLAS-WLNL-PHASE] sat="
                              << row.satellite.toString()
                              << " prefit=" << row.residual
                              << " postfit=" << postfit
                              << " sigma=" << std::sqrt(std::max(row.variance, 0.0))
                              << "\n";
                    if (++logged_phase_rows >= 4) {
                        break;
                    }
                }
            }
        }
        if (fixed_solved) {
            wlnl_fixed_position_ok = true;
        }
    }

    const ppp_shared::PPPState& solution_state =
        (ambiguity_resolution.accepted &&
         ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
         has_wlnl_fixed_state_)
            ? wlnl_fixed_state_
            : filter_state_;
    solution = ppp_clas::finalizeEpochSolution(
        solution_state,
        ambiguity_resolution.accepted,
        last_ar_ratio_,
        last_fixed_ambiguities_,
        static_cast<int>(osr_corrections.size()));

    if (wlnl_fixed_position_ok) {
        solution.position_ecef = wlnl_fixed_position;
    }

    // Multi-epoch SD AR: accumulate DD float ambiguities over epochs,
    // then fix with LAMBDA when variance is small enough.
    {
        const auto sd_ar_result = ppp_clas_sd::solveMultiEpochSdAr(
            clas_dd_accumulator_,
            obs,
            osr_corrections,
            solution.position_ecef,
            3.0,   // AR ratio threshold
            20,    // Min accumulation epochs before attempting LAMBDA
            pppDebugEnabled());
        constexpr bool kEnableClasSdMarPositionOverride = false;
        if (kEnableClasSdMarPositionOverride &&
            sd_ar_result.valid && sd_ar_result.code_rms <= 3.0) {
            solution.position_ecef = sd_ar_result.position;
            solution.status = SolutionStatus::PPP_FIXED;
        }
    }

    solution.time = obs.time;
    solution.receiver_clock_bias =
        filter_state_.state(filter_state_.clock_index) / constants::SPEED_OF_LIGHT;
    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    ecef2geodetic(solution.position_ecef, latitude, longitude, height);
    solution.position_geodetic = GeodeticCoord(latitude, longitude, height);
    return solution;
}

}  // namespace libgnss
