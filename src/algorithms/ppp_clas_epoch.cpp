// CLAS-PPP epoch processing method for PPPProcessor.
// Split from ppp.cpp for modularity.

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/ppp_clas_dd.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/signals.hpp>

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

namespace libgnss {

using PPPConfig = ppp_shared::PPPConfig;
using PPPState = ppp_shared::PPPState;
using PPPAmbiguityInfo = ppp_shared::PPPAmbiguityInfo;

namespace {

bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

constexpr double kClasNlDatumJumpThresholdCycles = 0.5;

std::ofstream* clasFloatDumpStream() {
    const auto& path = pppEnvOverrides().clas_float_dump_path;
    if (path.empty()) {
        return nullptr;
    }

    static std::ofstream stream;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        stream.open(path, std::ios::out | std::ios::trunc);
        if (stream) {
            stream << "record,week,tow,x_m,y_m,z_m,clock_m,trop_z_m,"
                   << "num_osr,num_rows,dx_m,dx_y_m,dx_z_m\n";
        }
    }
    return stream ? &stream : nullptr;
}

double clasNlPhaseBiasDatumCycles(const OSRCorrection& osr) {
    if (osr.num_frequencies < 2 ||
        osr.frequencies[0] <= 0.0 ||
        osr.frequencies[1] <= 0.0 ||
        std::abs(osr.frequencies[0] - osr.frequencies[1]) < 1.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double f1 = osr.frequencies[0];
    const double f2 = osr.frequencies[1];
    const double lambda_nl = constants::SPEED_OF_LIGHT / (f1 + f2);
    const double alpha1 = f1 / (f1 + f2);
    const double alpha2 = f2 / (f1 + f2);
    return (alpha1 * osr.phase_bias_m[0] + alpha2 * osr.phase_bias_m[1]) /
           lambda_nl;
}

void clearClasWlnlFixedDatumState(PPPAmbiguityInfo& ambiguity) {
    ambiguity.is_fixed = false;
    ambiguity.fixed_value = 0.0;
    ambiguity.wl_is_fixed = false;
    ambiguity.wl_fixed_integer = 0;
    ambiguity.nl_is_fixed = false;
    ambiguity.nl_fixed_cycles = 0.0;
    ambiguity.mw_sum_cycles = 0.0;
    ambiguity.mw_count = 0;
    ambiguity.mw_mean_cycles = 0.0;
}

void applyClasNlDatumReset(
    const GNSSTime& time,
    const std::vector<OSRCorrection>& osr_corrections,
    std::map<SatelliteId, PPPAmbiguityInfo>& ambiguity_states,
    bool debug_enabled) {
    for (const auto& osr : osr_corrections) {
        if (!osr.valid || osr.num_frequencies < 2) {
            continue;
        }
        const double datum_cycles = clasNlPhaseBiasDatumCycles(osr);
        if (!std::isfinite(datum_cycles)) {
            continue;
        }

        auto& ambiguity = ambiguity_states[osr.satellite];
        if (ambiguity.has_clas_nl_phase_bias_datum) {
            const double step_cycles =
                datum_cycles - ambiguity.clas_nl_phase_bias_datum_cycles;
            if (std::abs(step_cycles) > kClasNlDatumJumpThresholdCycles) {
                clearClasWlnlFixedDatumState(ambiguity);
                const SatelliteId l2_satellite(
                    osr.satellite.system,
                    static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100)));
                const auto l2_it = ambiguity_states.find(l2_satellite);
                if (l2_it != ambiguity_states.end()) {
                    clearClasWlnlFixedDatumState(l2_it->second);
                }
                if (debug_enabled) {
                    std::cerr << "[CLAS-NL-DATUM] reset "
                              << osr.satellite.toString()
                              << " tow=" << time.tow
                              << " step_cycles=" << step_cycles
                              << " datum_cycles=" << datum_cycles
                              << "\n";
                }
            }
        }
        ambiguity.clas_nl_phase_bias_datum_cycles = datum_cycles;
        ambiguity.has_clas_nl_phase_bias_datum = true;
    }
}

void dumpClasFloatPosition(
    const GNSSTime& time,
    const PPPState& filter_state,
    const ppp_clas::EpochUpdateResult& epoch_update,
    size_t num_osr) {
    auto* dump = clasFloatDumpStream();
    if (dump == nullptr || !epoch_update.update_stats.updated ||
        filter_state.state.size() < filter_state.total_states ||
        filter_state.total_states <= filter_state.trop_index) {
        return;
    }
    const Vector3d position =
        filter_state.state.segment(filter_state.pos_index, 3);
    Vector3d dx = Vector3d::Zero();
    if (epoch_update.update_stats.dx.size() >= filter_state.pos_index + 3) {
        dx = epoch_update.update_stats.dx.segment(filter_state.pos_index, 3);
    }
    *dump << std::setprecision(17)
          << "FLOAT,"
          << time.week << ','
          << time.tow << ','
          << position.x() << ','
          << position.y() << ','
          << position.z() << ','
          << filter_state.state(filter_state.clock_index) << ','
          << filter_state.state(filter_state.trop_index) << ','
          << num_osr << ','
          << epoch_update.update_stats.nobs << ','
          << dx.x() << ','
          << dx.y() << ','
          << dx.z() << '\n';
}

bool readClasAtmosNetworkId(
    const std::map<std::string, std::string>& epoch_atmos,
    int& network_id) {
    const auto network_it = epoch_atmos.find("atmos_network_id");
    if (network_it == epoch_atmos.end()) {
        return false;
    }
    network_id = std::atoi(network_it->second.c_str());
    return true;
}

void resetClasIonosphereStateValues(PPPState& filter_state) {
    for (const auto& [_, state_index] : filter_state.ionosphere_indices) {
        if (state_index >= 0 && state_index < filter_state.total_states) {
            filter_state.state(state_index) = 0.0;
        }
    }
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
    last_clas_constrained_fixed_state_valid_ = false;
    // CLAS per-frequency mode uses WL-NL AR: MW averaging resolves WL integers,
    // then NL integers are extracted from OSR-corrected dual-freq observations.
    if (ppp_config_.enable_ambiguity_resolution && !ppp_config_.use_ionosphere_free) {
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
        int last_clas_atmos_network_id = -1;
        bool has_last_clas_atmos_network_id = false;
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
        last_clas_atmos_network_id_,
        has_last_clas_atmos_network_id_,
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
        last_clas_atmos_network_id_ =
            fallback_snapshot.last_clas_atmos_network_id;
        has_last_clas_atmos_network_id_ =
            fallback_snapshot.has_last_clas_atmos_network_id;
    };
    const bool allow_hybrid_fallback =
        ppp_config_.clas_epoch_policy ==
        PPPConfig::ClasEpochPolicy::HYBRID_STANDARD_PPP_FALLBACK;
    const auto fallback_to_standard = [&](const char* reason) {
        restore_clas_snapshot();
        return processEpochStandard(obs, nav, reason);
    };

    PositionSolution seed = spp_processor_.processEpoch(obs, nav);
    detectCycleSlips(obs, nav);
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

    auto epoch_context = prepareClasEpochContext(
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
    materializeClasReceiverAntennaCorrections(epoch_context.osr_corrections);
    const auto& epoch_atmos = epoch_context.epoch_atmos_tokens;
    const auto& osr_corrections = epoch_context.osr_corrections;

    if (pppEnvOverrides().clas_stec_constraint &&
        ppp_config_.estimate_ionosphere &&
        ppp_config_.use_clas_osr_filter) {
        int network_id = -1;
        if (readClasAtmosNetworkId(epoch_atmos, network_id)) {
            if (has_last_clas_atmos_network_id_ &&
                network_id != last_clas_atmos_network_id_) {
                resetClasIonosphereStateValues(filter_state_);
            }
            last_clas_atmos_network_id_ = network_id;
            has_last_clas_atmos_network_id_ = true;
        }
    }

    if (osr_corrections.size() < 4) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("insufficient_osr");
        }
        solution = seed;
        return solution;
    }

    ppp_clas::ensureAmbiguityStates(filter_state_, osr_corrections);
    if (pppEnvOverrides().clas_nl_datum_reset &&
        ppp_config_.enable_ambiguity_resolution &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
        ppp_config_.use_clas_osr_filter &&
        !ppp_config_.use_ionosphere_free &&
        ppp_config_.estimate_ionosphere) {
        applyClasNlDatumReset(
            obs.time, osr_corrections, ambiguity_states_, pppDebugEnabled());
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
    dumpClasFloatPosition(obs.time, filter_state_, epoch_update, osr_corrections.size());
    const auto& update_stats = epoch_update.update_stats;
    pre_anchor_covariance_ = update_stats.pre_anchor_covariance;

    if (ppp_config_.use_clas_dd_filter) {
        if (!clas_dd_filter_) {
            clas_dd_filter_ = std::make_unique<ppp_clas_dd::DdFilterScaffold>();
        }
        const PositionSolution native_float_solution = ppp_clas::finalizeEpochSolution(
            filter_state_,
            false,
            0.0,
            0,
            static_cast<int>(osr_corrections.size()));
        solution = clas_dd_filter_->processFloatUpdate(
            obs,
            epoch_context,
            filter_state_,
            native_float_solution,
            ppp_config_,
            [&](const Vector3d& receiver_pos, double elevation, const GNSSTime& time) {
                return calculateMappingFunction(receiver_pos, elevation, time);
            });
        has_last_processed_time_ = true;
        last_processed_time_ = obs.time;
        ++total_epochs_processed_;
        return solution;
    }

    // Accumulate Melbourne-Wübbena for WL-NL AR in CLAS per-frequency mode.
    if (ppp_config_.enable_ambiguity_resolution &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
        for (const auto& osr : osr_corrections) {
            if (!osr.valid || osr.num_frequencies < 2) continue;
            const Observation* l1_raw = findOsrFrequencyObservation(obs, osr, 0);
            const Observation* l2_raw = findOsrFrequencyObservation(obs, osr, 1);
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
            constexpr double lambda_wl_gps = constants::SPEED_OF_LIGHT / (1575.42e6 - 1227.60e6);
            const double mw_cycles = mw_m / lambda_wl_gps;
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
    if (ambiguity_resolution.accepted &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
        wlnl_fixed_position_ok = solveFixedPosition(obs, nav, wlnl_fixed_position);
        if (pppDebugEnabled() && wlnl_fixed_position_ok) {
            const double shift = (wlnl_fixed_position -
                filter_state_.state.segment(filter_state_.pos_index, 3)).norm();
            std::cerr << "[CLAS-WLNL-FIX] pos_shift=" << shift << "m\n";
        }
    }

    const bool use_constrained_fixed_state =
        ambiguity_resolution.accepted &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
        env_overrides_.clas_resamb &&
        last_clas_constrained_fixed_state_valid_;
    const PPPState& solution_filter_state =
        use_constrained_fixed_state ? last_clas_constrained_fixed_state_ : filter_state_;

    solution = ppp_clas::finalizeEpochSolution(
        solution_filter_state,
        ambiguity_resolution.accepted,
        last_ar_ratio_,
        last_fixed_ambiguities_,
        static_cast<int>(osr_corrections.size()));

    if (wlnl_fixed_position_ok &&
        !env_overrides_.clas_fixed_state_output &&
        !use_constrained_fixed_state) {
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
