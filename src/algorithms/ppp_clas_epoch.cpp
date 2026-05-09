// CLAS-PPP epoch processing method for PPPProcessor.
// Split from ppp.cpp for modularity.

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/signals.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace libgnss {

using PPPConfig = ppp_shared::PPPConfig;
using PPPState = ppp_shared::PPPState;
using PPPAmbiguityInfo = ppp_shared::PPPAmbiguityInfo;

namespace {

bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

bool clasPdiResetEnabled() {
    const char* raw = std::getenv("GNSS_PPP_CLAS_RESET_ON_PDI");
    return raw != nullptr && raw[0] != '\0' && std::string(raw) != "0";
}

bool clasSdDiagnosticEnabled() {
    const char* raw = std::getenv("GNSS_PPP_CLAS_SD_DIAG");
    return raw != nullptr && raw[0] != '\0' && std::string(raw) != "0";
}

double stecTecuFromIonosphereDelayMeters(SignalType signal,
                                         const Ephemeris* eph,
                                         double delay_m) {
    const double frequency_hz = signalFrequencyHz(signal, eph);
    if (frequency_hz <= 0.0 || !std::isfinite(delay_m)) {
        return 0.0;
    }
    return delay_m * frequency_hz * frequency_hz / 40.3e16;
}

double rowCoefficient(const Eigen::RowVectorXd& row, int index) {
    if (index < 0 || index >= row.size()) {
        return 0.0;
    }
    return row(index);
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
    resetLastArAttemptDiagnostic();
    last_applied_atmos_trop_corrections_ = 0;
    last_applied_atmos_iono_corrections_ = 0;
    last_applied_atmos_trop_m_ = 0.0;
    last_applied_atmos_iono_m_ = 0.0;
    last_applied_ionex_corrections_ = 0;
    last_applied_dcb_corrections_ = 0;
    last_applied_ionex_m_ = 0.0;
    last_applied_dcb_m_ = 0.0;
    last_ssr_application_diagnostics_.clear();
    last_filter_iteration_diagnostics_.clear();
    last_residual_diagnostics_.clear();
    // CLAS per-frequency mode uses WL-NL AR: MW averaging resolves WL integers,
    // then NL integers are extracted from OSR-corrected dual-freq observations.
    if (ppp_config_.enable_ambiguity_resolution &&
        !ppp_config_.use_ionosphere_free &&
        ppp_config_.clas_auto_wlnl_ar) {
        ppp_config_.ar_method = PPPConfig::ARMethod::DD_WLNL;
        // CLAS corrections stabilize MW rapidly; fewer averaging epochs needed.
        if (ppp_config_.wl_min_averaging_epochs > 5) {
            ppp_config_.wl_min_averaging_epochs = 5;
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
    preserveDefaultSPPClockSeed(obs, nav, seed);
    detectCycleSlips(obs);
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
        precise_products_loaded_ ? 1e6 : ppp_config_.initial_ambiguity_variance);
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
    const int preferred_network_id = preferredClasNetworkId(epoch_atmos);
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        ++last_applied_atmos_trop_corrections_;
        last_applied_atmos_trop_m_ += std::abs(osr.trop_correction_m);
        if (osr.has_iono) {
            ++last_applied_atmos_iono_corrections_;
            last_applied_atmos_iono_m_ += std::abs(osr.iono_l1_m);
        }

        const Ephemeris* eph = nav.getEphemeris(osr.satellite, obs.time);
        const double geometric_range =
            (osr.satellite_position - epoch_context.receiver_position).norm();
        Vector3d line_of_sight = Vector3d::Zero();
        if (geometric_range > 0.0) {
            line_of_sight =
                (osr.satellite_position - epoch_context.receiver_position) /
                geometric_range;
        }
        const double trop_mapping =
            calculateMappingFunction(epoch_context.receiver_position, osr.elevation, obs.time);
        for (int f = 0; f < osr.num_frequencies; ++f) {
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            SSRApplicationDiagnostic diagnostic;
            diagnostic.satellite = osr.satellite;
            diagnostic.primary_signal = osr.signals[f];
            diagnostic.primary_observation_code = raw != nullptr ? raw->observation_code : "";
            diagnostic.frequency_index = f;
            diagnostic.ionosphere_coefficient =
                (osr.wavelengths[f] > 0.0 && osr.wavelengths[0] > 0.0)
                    ? std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2)
                    : 1.0;
            diagnostic.ssr_available = true;
            diagnostic.broadcast_iode = eph != nullptr ? static_cast<int>(eph->iode) : -1;
            diagnostic.orbit_clock_applied = true;
            diagnostic.orbit_correction_x_m = osr.orbit_projection_m;
            diagnostic.orbit_projection_m = osr.orbit_projection_m;
            diagnostic.clock_correction_m = osr.clock_correction_m;
            diagnostic.prc_m = osr.PRC[f];
            diagnostic.cpc_m = osr.CPC[f];
            const auto applied_corrections = ppp_clas::selectAppliedOsrCorrections(
                osr, f, ppp_config_.clas_correction_application_policy);
            diagnostic.applied_pseudorange_correction_m =
                applied_corrections.pseudorange_correction_m;
            diagnostic.applied_carrier_phase_correction_m =
                applied_corrections.carrier_phase_correction_m;
            diagnostic.relativity_correction_m = osr.relativity_correction_m;
            diagnostic.receiver_antenna_m = osr.receiver_antenna_m[f];
            diagnostic.code_bias_m = osr.code_bias_m[f];
            diagnostic.phase_bias_m = osr.phase_bias_m[f];
            diagnostic.windup_m = osr.windup_m[f];
            diagnostic.phase_compensation_m = osr.phase_compensation_m[f];
            diagnostic.trop_correction_m = osr.trop_correction_m;
            diagnostic.stec_tecu = stecTecuFromIonosphereDelayMeters(
                osr.signals[0], eph, osr.iono_l1_m);
            diagnostic.iono_correction_m =
                diagnostic.ionosphere_coefficient * osr.iono_l1_m;
            diagnostic.atmos_token_count = static_cast<int>(epoch_atmos.size());
            diagnostic.preferred_network_id = preferred_network_id;
            diagnostic.atmos_network_id = osr.atmos_network_id;
            diagnostic.code_bias_network_id = osr.code_bias_network_id;
            diagnostic.phase_bias_network_id = osr.phase_bias_network_id;
            diagnostic.atmos_reference_week = osr.atmos_reference_time.week;
            diagnostic.atmos_reference_tow = osr.atmos_reference_time.tow;
            diagnostic.phase_bias_reference_week = osr.phase_bias_reference_time.week;
            diagnostic.phase_bias_reference_tow = osr.phase_bias_reference_time.tow;
            diagnostic.clock_reference_week = osr.clock_reference_time.week;
            diagnostic.clock_reference_tow = osr.clock_reference_time.tow;
            const GNSSTime effective_phase_bias_reference_time =
                selectClasPhaseBiasReferenceTime(
                    ppp_config_.clas_phase_bias_reference_time_policy,
                    osr.phase_bias_reference_time,
                    osr.clock_reference_time,
                    obs.time);
            diagnostic.effective_phase_bias_reference_week =
                effective_phase_bias_reference_time.week;
            diagnostic.effective_phase_bias_reference_tow =
                effective_phase_bias_reference_time.tow;
            diagnostic.elevation_deg = osr.elevation * 180.0 / M_PI;
            diagnostic.has_carrier_phase = raw != nullptr && raw->has_carrier_phase;
            diagnostic.valid_after_corrections = osr.valid;
            diagnostic.receiver_position_x_m = epoch_context.receiver_position.x();
            diagnostic.receiver_position_y_m = epoch_context.receiver_position.y();
            diagnostic.receiver_position_z_m = epoch_context.receiver_position.z();
            diagnostic.satellite_position_x_m = osr.satellite_position.x();
            diagnostic.satellite_position_y_m = osr.satellite_position.y();
            diagnostic.satellite_position_z_m = osr.satellite_position.z();
            diagnostic.satellite_clock_bias_m =
                constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
            diagnostic.geometric_range_m = geometric_range;
            diagnostic.line_of_sight_x = line_of_sight.x();
            diagnostic.line_of_sight_y = line_of_sight.y();
            diagnostic.line_of_sight_z = line_of_sight.z();
            diagnostic.modeled_trop_delay_m = trop_mapping * epoch_context.trop_zenith_m;
            diagnostic.modeled_zenith_trop_delay_m = epoch_context.trop_zenith_m;
            diagnostic.estimated_trop_delay_m = diagnostic.modeled_trop_delay_m;
            last_ssr_application_diagnostics_.push_back(diagnostic);
        }
    }

    if (osr_corrections.size() < 4) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("insufficient_osr");
        }
        solution = seed;
        return solution;
    }

    ppp_clas::ensureAmbiguityStates(filter_state_, osr_corrections, ppp_config_);
    if (clasPdiResetEnabled()) {
        for (const auto& osr : osr_corrections) {
            if (osr.phase_discontinuity_indicators.empty()) {
                continue;
            }
            auto& last_map = last_phase_disc_indicators_[osr.satellite];
            for (int f = 0; f < osr.num_frequencies; ++f) {
                const uint8_t signal_id = rtcmSsrSignalId(osr.satellite.system, osr.signals[f]);
                const auto pdi_it = osr.phase_discontinuity_indicators.find(signal_id);
                if (pdi_it == osr.phase_discontinuity_indicators.end()) {
                    continue;
                }
                const auto prev_it = last_map.find(signal_id);
                const bool changed = prev_it != last_map.end() && prev_it->second != pdi_it->second;
                last_map[signal_id] = pdi_it->second;
                if (!changed) {
                    continue;
                }

                const uint8_t ambiguity_prn = f == 0
                    ? osr.satellite.prn
                    : static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
                const SatelliteId ambiguity_satellite(osr.satellite.system, ambiguity_prn);
                const auto ambiguity_it =
                    filter_state_.ambiguity_indices.find(ambiguity_satellite);
                if (ambiguity_it == filter_state_.ambiguity_indices.end()) {
                    continue;
                }
                const int ambiguity_index = ambiguity_it->second;
                if (ambiguity_index < 0 || ambiguity_index >= filter_state_.total_states) {
                    continue;
                }
                filter_state_.state(ambiguity_index) = 0.0;
                filter_state_.covariance.row(ambiguity_index).setZero();
                filter_state_.covariance.col(ambiguity_index).setZero();
                filter_state_.covariance(ambiguity_index, ambiguity_index) =
                    precise_products_loaded_ ? 1e6 : ppp_config_.initial_ambiguity_variance;
                ambiguity_states_[ambiguity_satellite] = PPPAmbiguityInfo{};
                if (pppDebugEnabled()) {
                    std::cerr << "[PPP-CLAS-PDI] reset "
                              << ambiguity_satellite.toString()
                              << " signal_id=" << static_cast<int>(signal_id)
                              << " pdi=" << static_cast<int>(pdi_it->second) << "\n";
                }
            }
        }
    }
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
    last_residual_diagnostics_.reserve(epoch_update.measurements.size());
    for (size_t row = 0; row < epoch_update.measurements.size(); ++row) {
        const auto& measurement = epoch_update.measurements[row];
        PPPResidualDiagnostic residual_diagnostic;
        residual_diagnostic.iteration = 0;
        residual_diagnostic.row_index = static_cast<int>(row);
        residual_diagnostic.satellite = measurement.satellite;
        residual_diagnostic.primary_signal = measurement.primary_signal;
        residual_diagnostic.primary_observation_code =
            measurement.primary_observation_code;
        residual_diagnostic.frequency_index =
            measurement.freq_index >= 0 ? measurement.freq_index : 0;
        residual_diagnostic.ionosphere_coefficient =
            measurement.ionosphere_coefficient;
        residual_diagnostic.receiver_clock_state_index =
            filter_state_.clock_index;
        residual_diagnostic.receiver_clock_design_coeff =
            rowCoefficient(measurement.H, filter_state_.clock_index);
        residual_diagnostic.ionosphere_state_index =
            measurement.ionosphere_state_index;
        residual_diagnostic.ionosphere_design_coeff =
            measurement.ionosphere_design_coeff != 0.0
                ? measurement.ionosphere_design_coeff
                : rowCoefficient(measurement.H, measurement.ionosphere_state_index);
        residual_diagnostic.ambiguity_state_index =
            measurement.ambiguity_state_index;
        residual_diagnostic.ambiguity_design_coeff =
            measurement.ambiguity_design_coeff != 0.0
                ? measurement.ambiguity_design_coeff
                : rowCoefficient(measurement.H, measurement.ambiguity_state_index);
        residual_diagnostic.carrier_phase = measurement.is_phase;
        residual_diagnostic.ionosphere_constraint =
            !measurement.is_phase &&
            measurement.freq_index < 0 &&
            measurement.satellite.prn != 0;
        residual_diagnostic.prior_constraint =
            !measurement.is_phase &&
            measurement.freq_index < 0 &&
            measurement.satellite.prn == 0;
        residual_diagnostic.phase_accepted = measurement.is_phase;
        residual_diagnostic.phase_ready = measurement.is_phase;
        residual_diagnostic.ambiguity_lock_count = -1;
        residual_diagnostic.required_lock_count =
            measurement.is_phase ? ppp_config_.phase_measurement_min_lock_count : 0;
        if (measurement.is_phase && !measurement.phase_ambiguities.empty()) {
            SatelliteId lock_satellite = measurement.phase_ambiguities.front();
            for (const auto& ambiguity_satellite : measurement.phase_ambiguities) {
                if (ambiguityStateIndex(ambiguity_satellite) ==
                    residual_diagnostic.ambiguity_state_index) {
                    lock_satellite = ambiguity_satellite;
                    break;
                }
            }
            const auto ambiguity_it = ambiguity_states_.find(lock_satellite);
            if (ambiguity_it != ambiguity_states_.end()) {
                residual_diagnostic.ambiguity_lock_count =
                    ambiguity_it->second.lock_count;
            }
        }
        residual_diagnostic.observation_m = measurement.observation;
        residual_diagnostic.predicted_m = measurement.predicted;
        residual_diagnostic.residual_m =
            row < static_cast<size_t>(update_stats.residuals.size())
                ? update_stats.residuals(static_cast<int>(row))
                : measurement.residual;
        residual_diagnostic.variance_m2 =
            row < static_cast<size_t>(update_stats.variances.size())
                ? update_stats.variances(static_cast<int>(row))
                : measurement.variance;
        residual_diagnostic.outlier_inflated =
            row < update_stats.outlier_inflated_rows.size()
                ? update_stats.outlier_inflated_rows[row]
                : false;
        residual_diagnostic.elevation_deg =
            std::isfinite(measurement.elevation_rad)
                ? measurement.elevation_rad * 180.0 / M_PI
                : 0.0;
        residual_diagnostic.iono_state_m = measurement.ionosphere_state_m;
        last_residual_diagnostics_.push_back(residual_diagnostic);
    }
    PPPFilterIterationDiagnostic filter_diagnostic;
    filter_diagnostic.iteration = 0;
    filter_diagnostic.rows = update_stats.nobs;
    filter_diagnostic.code_rows = update_stats.code_rows;
    filter_diagnostic.phase_rows = update_stats.phase_rows;
    filter_diagnostic.ionosphere_constraint_rows =
        update_stats.ionosphere_constraint_rows;
    filter_diagnostic.outlier_inflated_rows =
        update_stats.outlier_inflated_row_count;
    filter_diagnostic.code_outlier_inflated_rows =
        update_stats.code_outlier_inflated_row_count;
    filter_diagnostic.phase_outlier_inflated_rows =
        update_stats.phase_outlier_inflated_row_count;
    filter_diagnostic.ionosphere_constraint_outlier_inflated_rows =
        update_stats.ionosphere_constraint_outlier_inflated_row_count;
    filter_diagnostic.prior_outlier_inflated_rows =
        update_stats.prior_outlier_inflated_row_count;
    filter_diagnostic.code_outlier_inflated_max_abs_m =
        update_stats.code_outlier_inflated_max_abs_m;
    filter_diagnostic.phase_outlier_inflated_max_abs_m =
        update_stats.phase_outlier_inflated_max_abs_m;
    filter_diagnostic.ionosphere_constraint_outlier_inflated_max_abs_m =
        update_stats.ionosphere_constraint_outlier_inflated_max_abs_m;
    filter_diagnostic.prior_outlier_inflated_max_abs_m =
        update_stats.prior_outlier_inflated_max_abs_m;
    filter_diagnostic.code_outlier_inflated_rms_m =
        update_stats.code_outlier_inflated_rms_m;
    filter_diagnostic.phase_outlier_inflated_rms_m =
        update_stats.phase_outlier_inflated_rms_m;
    filter_diagnostic.ionosphere_constraint_outlier_inflated_rms_m =
        update_stats.ionosphere_constraint_outlier_inflated_rms_m;
    filter_diagnostic.prior_outlier_inflated_rms_m =
        update_stats.prior_outlier_inflated_rms_m;
    if (seed.isValid()) {
        filter_diagnostic.seed_position_x_m = seed.position_ecef.x();
        filter_diagnostic.seed_position_y_m = seed.position_ecef.y();
        filter_diagnostic.seed_position_z_m = seed.position_ecef.z();
        filter_diagnostic.seed_clock_m = seed.receiver_clock_bias;
    }
    if (update_stats.dx.size() >= 3) {
        filter_diagnostic.pos_delta_x_m = update_stats.dx(0);
        filter_diagnostic.pos_delta_y_m = update_stats.dx(1);
        filter_diagnostic.pos_delta_z_m = update_stats.dx(2);
        filter_diagnostic.pos_delta_m = update_stats.dx.head<3>().norm();
    }
    if (filter_state_.clock_index >= 0 &&
        filter_state_.clock_index < filter_state_.state.size()) {
        filter_diagnostic.clock_state_m =
            filter_state_.state(filter_state_.clock_index);
        if (update_stats.dx.size() > filter_state_.clock_index) {
            filter_diagnostic.clock_delta_m =
                update_stats.dx(filter_state_.clock_index);
        }
    }
    if (filter_state_.trop_index >= 0 &&
        filter_state_.trop_index < filter_state_.state.size()) {
        filter_diagnostic.trop_state_m =
            filter_state_.state(filter_state_.trop_index);
        if (update_stats.dx.size() > filter_state_.trop_index) {
            filter_diagnostic.trop_delta_m =
                update_stats.dx(filter_state_.trop_index);
        }
    }
    filter_diagnostic.position_x_m = filter_state_.state(0);
    filter_diagnostic.position_y_m = filter_state_.state(1);
    filter_diagnostic.position_z_m = filter_state_.state(2);
    filter_diagnostic.code_residual_rms_m =
        update_stats.code_residual_rms_m;
    filter_diagnostic.code_residual_max_abs_m =
        update_stats.code_residual_max_abs_m;
    filter_diagnostic.code_residual_max_sat =
        update_stats.code_residual_max_sat;
    filter_diagnostic.phase_residual_rms_m =
        update_stats.phase_residual_rms_m;
    filter_diagnostic.phase_residual_max_abs_m =
        update_stats.phase_residual_max_abs_m;
    filter_diagnostic.phase_residual_max_sat =
        update_stats.phase_residual_max_sat;
    double iono_state_sum_sq = 0.0;
    double iono_delta_sum_sq = 0.0;
    int iono_state_count = 0;
    for (const auto& [satellite, index] : filter_state_.ionosphere_indices) {
        (void)satellite;
        if (index < 0 || index >= filter_state_.state.size()) {
            continue;
        }
        const double value = filter_state_.state(index);
        iono_state_sum_sq += value * value;
        filter_diagnostic.iono_state_max_abs_m =
            std::max(filter_diagnostic.iono_state_max_abs_m, std::abs(value));
        if (update_stats.dx.size() > index) {
            const double delta = update_stats.dx(index);
            iono_delta_sum_sq += delta * delta;
            filter_diagnostic.iono_delta_max_abs_m =
                std::max(filter_diagnostic.iono_delta_max_abs_m, std::abs(delta));
        }
        ++iono_state_count;
    }
    if (iono_state_count > 0) {
        filter_diagnostic.iono_state_rms_m =
            std::sqrt(iono_state_sum_sq / static_cast<double>(iono_state_count));
        filter_diagnostic.iono_delta_rms_m =
            std::sqrt(iono_delta_sum_sq / static_cast<double>(iono_state_count));
    }
    last_filter_iteration_diagnostics_.push_back(filter_diagnostic);

    if (clasSdDiagnosticEnabled()) {
        const double dt =
            has_last_processed_time_ ? std::max(obs.time - last_processed_time_, 0.001) : 1.0;
        const Vector3d main_position =
            filter_state_.state.segment(filter_state_.pos_index, 3);
        const auto sd_result = ppp_clas_sd::runClockFreeSdEpoch(
            clas_sd_state_,
            obs,
            osr_corrections,
            main_position,
            dt,
            false);
        if (sd_result.valid) {
            const auto sdAmbiguity = [&](const std::map<SatelliteId, int>& indices,
                                         const SatelliteId& satellite) {
                const auto it = indices.find(satellite);
                if (it == indices.end() || it->second < 0 ||
                    it->second >= clas_sd_state_.x.size()) {
                    return std::numeric_limits<double>::quiet_NaN();
                }
                return clas_sd_state_.x(it->second);
            };
            const SatelliteId g25(GNSSSystem::GPS, 25);
            const SatelliteId e27(GNSSSystem::Galileo, 27);
            const SatelliteId j02(GNSSSystem::QZSS, 2);
            std::cerr << "[CLAS-SD-DIAG] epoch=" << total_epochs_processed_
                      << " tow=" << obs.time.tow
                      << " sats=" << sd_result.num_satellites
                      << " obs=" << sd_result.num_observations
                      << " main_sd_shift=" << (sd_result.position - main_position).norm()
                      << " code_rms=" << sd_result.code_rms
                      << " phase_rms=" << sd_result.phase_rms
                      << " pos_sigma=" << clas_sd_state_.position_sigma_m
                      << " g25_l1_sd_amb=" << sdAmbiguity(clas_sd_state_.amb_indices, g25)
                      << " g25_l2_sd_amb=" << sdAmbiguity(clas_sd_state_.amb2_indices, g25)
                      << " e27_l1_sd_amb=" << sdAmbiguity(clas_sd_state_.amb_indices, e27)
                      << " e27_l2_sd_amb=" << sdAmbiguity(clas_sd_state_.amb2_indices, e27)
                      << " j02_l1_sd_amb=" << sdAmbiguity(clas_sd_state_.amb_indices, j02)
                      << " j02_l2_sd_amb=" << sdAmbiguity(clas_sd_state_.amb2_indices, j02)
                      << "\n";
        }
    }

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
            if (!std::isfinite(lambda_wl) || lambda_wl <= 0.0) continue;
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
    const PPPState pre_ar_filter_state = filter_state_;
    const auto pre_ar_ambiguity_states = ambiguity_states_;

    auto ambiguity_resolution =
        ppp_clas::resolveAndValidateAmbiguities(
            filter_state_,
            ambiguity_states_,
            [&]() {
                return ppp_config_.enable_ambiguity_resolution &&
                       resolveAmbiguities(obs, nav);
            },
            (ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL ||
             !ppp_config_.clas_validate_fixed_solution)
                ? ppp_clas::ValidateFixedSolutionFunction{}
                : ppp_clas::ValidateFixedSolutionFunction{[&]() {
                      return ppp_clas::validateFixedSolution(
                          obs, osr_corrections, filter_state_, ppp_config_,
                          trop_mapping_for_validation,
                          ambiguity_index_for_validation, pppDebugEnabled());
                  }},
            pppDebugEnabled());
    if (ambiguity_resolution.rejected_after_fix) {
        markLastArRejectedAfterFix();
        last_ar_ratio_ = 0.0;
        last_fixed_ambiguities_ = 0;
    }

    if (pppDebugEnabled()) {
        ppp_clas::logUpdateSummary(update_stats, osr_corrections.size());
    }

    bool wlnl_fixed_position_ok = false;
    Vector3d wlnl_fixed_position = Vector3d::Zero();
    if (ambiguity_resolution.accepted &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
        wlnl_fixed_position_ok = solveFixedPosition(obs, nav, wlnl_fixed_position);
        if (wlnl_fixed_position_ok) {
            const double shift =
                (wlnl_fixed_position -
                 filter_state_.state.segment(filter_state_.pos_index, 3)).norm();
            if (ppp_config_.clas_wlnl_fixed_position_max_shift_m > 0.0 &&
                shift > ppp_config_.clas_wlnl_fixed_position_max_shift_m) {
                last_ar_attempt_diagnostic_.fixed_position_rejected_by_shift = true;
                wlnl_fixed_position_ok = false;
                ambiguity_resolution.accepted = false;
                markLastArRejectedAfterFix();
                last_ar_ratio_ = 0.0;
                last_fixed_ambiguities_ = 0;
                for (auto& [satellite, ambiguity] : ambiguity_states_) {
                    (void)satellite;
                    ambiguity.is_fixed = false;
                    ambiguity.nl_is_fixed = false;
                    ambiguity.nl_fixed_cycles = 0.0;
                }
                if (pppDebugEnabled()) {
                    std::cerr << "[CLAS-WLNL-FIX] reject pos_shift=" << shift
                              << "m max="
                              << ppp_config_.clas_wlnl_fixed_position_max_shift_m
                              << "m\n";
                }
            } else if (pppDebugEnabled()) {
                std::cerr << "[CLAS-WLNL-FIX] pos_shift=" << shift << "m\n";
            }
        } else if (ppp_config_.clas_wlnl_fixed_position_max_shift_m > 0.0) {
            ambiguity_resolution.accepted = false;
            markLastArRejectedAfterFix();
            last_ar_ratio_ = 0.0;
            last_fixed_ambiguities_ = 0;
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-WLNL-FIX] reject fixed WLS failure\n";
            }
        }
    }
    if (ambiguity_resolution.accepted) {
        last_ar_attempt_diagnostic_.accepted = true;
    }
    updateLatestFilterArDiagnostic();

    solution = ppp_clas::finalizeEpochSolution(
        filter_state_,
        obs.time,
        ambiguity_resolution.accepted,
        last_ar_ratio_,
        last_fixed_ambiguities_,
        static_cast<int>(osr_corrections.size()));

    if (wlnl_fixed_position_ok) {
        solution.position_ecef = wlnl_fixed_position;
    }
    if (ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
        filter_state_ = pre_ar_filter_state;
        ambiguity_states_ = pre_ar_ambiguity_states;
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
        if (sd_ar_result.valid && sd_ar_result.code_rms >= 3.0) {
            solution.position_ecef = sd_ar_result.position;
            solution.status = SolutionStatus::PPP_FIXED;
        }
    }

    has_last_processed_time_ = true;
    last_processed_time_ = obs.time;
    ++total_epochs_processed_;

    return solution;
}

}  // namespace libgnss
