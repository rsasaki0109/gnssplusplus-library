// CLAS-PPP epoch processing method for PPPProcessor.
// Split from ppp.cpp for modularity.

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/ppp_clas_dd.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/algorithms/rtk_validation.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
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
// Good WL-NL / SD-MAR fixes stay below ~0.7 m; parity-path blunders were 38–595 m.
constexpr double kClasBaseClockParitySdMarMaxPositionShiftM = 2.0;
// Kinematic CLAS: reject fixed positions that jump beyond float prediction.
// RTK PPC gates (PR #177/#178/#179) use 20 m min / 25 m/s adaptive trusted jump;
// at 5 Hz urban (~15 m/s) we tighten to max(1 m, 15 m/s·dt) capped at 8 m, and
// max(1 m, 3σ_float) when covariance is available (ppp.cpp uses 25 m kinematic).
constexpr double kClasKinematicFixedJumpMinM = 1.0;
constexpr double kClasKinematicFixedJumpRateMps = 10.0;
constexpr double kClasKinematicFixedJumpMaxM = 2.0;
constexpr double kClasKinematicFixedJumpSigmaScale = 3.0;
constexpr double kClasKinematicWlnlMaxPositionShiftM = 2.0;
constexpr double kClasKinematicMinFixRatio = 3.0;
constexpr int kClasKinematicMinFixCount = 1;
// MRTKLIB clas.toml rejection.hold_chi_square / fix_chi_square (mrtk_ppp_rtk.c:2312-2315)
constexpr double kMrtklibHoldChiSquareGate = 0.5;
constexpr double kMrtklibFixChiSquareGate = 5.0;
// MRTKLIB clas.toml rejection.l1_l2_residual (pos2-rejionno1) sigma gate used
// by residual_test() to drop individual outlier carrier residuals.
constexpr double kMrtklibPhaseResidualSigmaGate = 2.0;
// MRTKLIB clas.toml rejection.pseudorange_diff / position_error_count
// (pos2-rejdiffpse -> opt.maxdiffp, pos2-poserrcnt; mrtk_ppp_rtk.c:2333-2352)
constexpr double kMrtklibMaxSppDivergenceM = 10.0;
constexpr int kMrtklibMaxSppDivergenceEpochs = 5;

double clasKinematicHorizontalPositionSigmaM(const PPPState& filter_state) {
    if (filter_state.covariance.rows() < 3 ||
        filter_state.covariance.cols() < 3 ||
        filter_state.pos_index < 0 ||
        filter_state.pos_index + 2 >= filter_state.covariance.rows()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const int base = filter_state.pos_index;
    const double var_xy =
        filter_state.covariance(base, base) + filter_state.covariance(base + 1, base + 1);
    if (!std::isfinite(var_xy) || var_xy <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::sqrt(var_xy);
}

double clasKinematicMaxFixedFloatJumpM(
    double dt_seconds,
    double horizontal_position_sigma_m) {
    const double adaptive_limit = rtk_validation::adaptiveJumpLimit(
        dt_seconds,
        kClasKinematicFixedJumpMinM,
        kClasKinematicFixedJumpRateMps);
    double limit = adaptive_limit;
    if (std::isfinite(horizontal_position_sigma_m) && horizontal_position_sigma_m > 0.0) {
        limit = std::max(
            limit,
            kClasKinematicFixedJumpMinM +
                kClasKinematicFixedJumpSigmaScale * horizontal_position_sigma_m);
    }
    return std::min(kClasKinematicFixedJumpMaxM, limit);
}

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

bool applyClasNlDatumReset(
    const GNSSTime& time,
    const std::vector<OSRCorrection>& osr_corrections,
    std::map<SatelliteId, PPPAmbiguityInfo>& ambiguity_states,
    bool debug_enabled) {
    bool any_reset = false;
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
                any_reset = true;
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
    return any_reset;
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

void applyOptionalSolutionEpochMetadata(
    PositionSolution& solution,
    const GNSSTime& time,
    const PPPConfig& config) {
    if (!config.emit_solution_epoch_time) {
        return;
    }
    solution.time = time;
    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    ecef2geodetic(solution.position_ecef, latitude, longitude, height);
    solution.position_geodetic = GeodeticCoord(latitude, longitude, height);
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

    const double clas_dt_seconds =
        has_last_processed_time_
            ? std::max(obs.time - last_processed_time_, 0.001)
            : 1.0;
    if (ppp_config_.kinematic_mode && ppp_config_.enable_cycle_slip_detection) {
        const auto slip_stats = ppp_clas::detectClasCycleSlips(
            obs,
            osr_corrections,
            ppp_config_,
            clas_dt_seconds,
            filter_state_,
            ambiguity_states_,
            clas_dispersion_compensation_,
            clas_phase_bias_repair_,
            [&](const SatelliteId& satellite, SignalType signal) {
                resetAmbiguity(satellite, signal);
            },
            precise_products_loaded_ ? 1e6 : ppp_config_.initial_ambiguity_variance,
            pppDebugEnabled());
        if (slip_stats.total_resets > 0) {
            clas_dd_accumulator_ = {};
        }
    }

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
        // Dynamics mode: prepareEpochState already ran predictFilterState,
        // which propagated pos += vel*dt and inflated the covariance for
        // this interval, so the processed-time bookkeeping MUST advance
        // before this early return. Otherwise the next epoch's dt spans
        // this interval AGAIN and the same coast is re-applied every epoch:
        // across a low-satellite stretch dt grows unboundedly (observed
        // 2.2 s -> 15+ s at 5 Hz approaching the tokyo_run2 bridge outage)
        // and the repeated vel*dt over-propagation drags the float
        // kilometers away (the 4.5 km post-bridge blunder). White-noise
        // mode keeps historical behavior (its per-epoch SPP re-anchor of
        // position and clock bounds the damage of a stale dt).
        if (ppp_config_.use_dynamics_model) {
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
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
        if (applyClasNlDatumReset(
                obs.time, osr_corrections, ambiguity_states_, pppDebugEnabled())) {
            clas_dd_accumulator_ = {};
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
        // Same dt-bookkeeping requirement as the insufficient_osr early
        // return above: the predict for this interval already happened.
        if (ppp_config_.use_dynamics_model) {
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
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
            obs.time,
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
        applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
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
            if (ppp_config_.kinematic_mode &&
                ppp_config_.use_clas_osr_filter &&
                ((l1_raw->snr > 0.0 &&
                  ppp_ar::clasKinematicSnrMasked(0, osr.elevation, l1_raw->snr)) ||
                 (l2_raw->snr > 0.0 &&
                  ppp_ar::clasKinematicSnrMasked(1, osr.elevation, l2_raw->snr)))) {
                continue;
            }
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
            const bool mw_slip = amb.needs_reinitialization;
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

    const Vector3d clas_float_position_ecef =
        filter_state_.state.segment(filter_state_.pos_index, 3);
    const double clas_float_horizontal_sigma_m =
        clasKinematicHorizontalPositionSigmaM(filter_state_);
    const PPPState clas_float_filter_state = filter_state_;
    const auto clas_float_ambiguity_states = ambiguity_states_;

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

    // MRTKLIB mrtk_ppp_rtk.c:2296-2330 parity: validate the fixed solution with
    // the post-fix DD phase chi-square. Publish FIX only when chisq < thres_fix
    // (5.0) and hold (constrain the float filter toward the fixed DD
    // ambiguities, holdamb()) only when chisq < thres_hold (0.5). AR-failed
    // epochs publish FLOAT and reset the nfix counter; there is no hold-driven
    // FIX publication.
    const bool kinematic_clas_wlnl_hold_path =
        ppp_config_.kinematic_mode &&
        ppp_config_.use_clas_osr_filter &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL;
    bool clas_kinematic_chisq_rejected = false;
    if (kinematic_clas_wlnl_hold_path &&
        ppp_config_.enable_ambiguity_resolution) {
        if (!ppp_ar::wlnlHoldStillValid(clas_wlnl_hold_, ambiguity_states_)) {
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        }

        if (ambiguity_resolution.accepted &&
            !last_clas_constrained_fixed_state_valid_) {
            // MRTKLIB publishes FIX only from the constrained xa solution
            // (sol.rr = xa). Without a validated state-DD LAMBDA fix the epoch
            // stays FLOAT instead of labelling the float state as fixed.
            clas_kinematic_chisq_rejected = true;
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-KIN-CHISQ] no constrained state, demote"
                          << " ratio=" << last_ar_ratio_ << "\n";
            }
        } else if (ambiguity_resolution.accepted) {
            const PPPState& fixed_state_for_validation =
                last_clas_constrained_fixed_state_;
            ppp_clas::FixValidationOptions fix_validation_options;
            fix_validation_options.outlier_sigma_gate =
                kMrtklibPhaseResidualSigmaGate;
            fix_validation_options.mrtklib_chisq_fallback = true;
            const auto fix_validation = ppp_clas::validateFixedSolution(
                obs,
                osr_corrections,
                fixed_state_for_validation,
                ppp_config_,
                trop_mapping_for_validation,
                ambiguity_index_for_validation,
                pppDebugEnabled(),
                fix_validation_options);
            const double phase_chisq = fix_validation.phase_chisq;
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-KIN-CHISQ] phase_chisq=" << phase_chisq
                          << " rows=" << fix_validation.phase_rows
                          << " outliers=" << fix_validation.phase_outlier_rows
                          << " ratio=" << last_ar_ratio_ << "\n";
            }
            if (!std::isfinite(phase_chisq) ||
                phase_chisq >= kMrtklibFixChiSquareGate) {
                clas_kinematic_chisq_rejected = true;
            } else if (phase_chisq < kMrtklibHoldChiSquareGate) {
                std::map<SatelliteId, double> clas_satellite_elevations_rad;
                for (const auto& osr : osr_corrections) {
                    if (osr.valid) {
                        clas_satellite_elevations_rad[osr.satellite] =
                            osr.elevation;
                    }
                }
                ++clas_wlnl_hold_.consecutive_fix_count;
                std::vector<ppp_ar::WlnlHoldConstraint> hold_constraints;
                if (clas_wlnl_hold_.consecutive_fix_count >=
                        ppp_ar::kMrtklibMinFixCount &&
                    ppp_ar::buildWlnlHoldConstraints(
                        last_clas_constrained_fixed_state_,
                        ambiguity_states_,
                        clas_satellite_elevations_rad,
                        hold_constraints)) {
                    clas_wlnl_hold_.constraints = std::move(hold_constraints);
                    clas_wlnl_hold_.active = true;
                    ppp_ar::applyWlnlHoldAmbiguity(
                        filter_state_, clas_wlnl_hold_.constraints);
                }
            }
        } else {
            // MRTKLIB resets rtk->nfix on every non-FIX epoch
            // (mrtk_ppp_rtk.c:2371).
            clas_wlnl_hold_.consecutive_fix_count = 0;
        }
    }

    bool ambiguity_fixed_epoch =
        ambiguity_resolution.accepted && !clas_kinematic_chisq_rejected;
    if (clas_kinematic_chisq_rejected) {
        last_ar_ratio_ = 0.0;
        last_fixed_ambiguities_ = 0;
        ambiguity_states_ = clas_float_ambiguity_states;
        ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        if (pppDebugEnabled()) {
            std::cerr << "[CLAS-KIN-CHISQ] reject fix (chisq >= "
                      << kMrtklibFixChiSquareGate << ")\n";
        }
    }
    if (ambiguity_resolution.rejected_after_fix) {
        last_ar_ratio_ = 0.0;
        last_fixed_ambiguities_ = 0;
        if (ppp_config_.kinematic_mode && ppp_config_.use_clas_osr_filter) {
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        }
    }

    if (pppDebugEnabled()) {
        ppp_clas::logUpdateSummary(update_stats, osr_corrections.size());
    }

    bool wlnl_fixed_position_ok = false;
    Vector3d wlnl_fixed_position = Vector3d::Zero();
    if (ambiguity_fixed_epoch &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
        !kinematic_clas_wlnl_hold_path) {
        wlnl_fixed_position_ok = solveFixedPosition(obs, nav, wlnl_fixed_position);
        if (pppDebugEnabled() && wlnl_fixed_position_ok) {
            const double shift = (wlnl_fixed_position -
                filter_state_.state.segment(filter_state_.pos_index, 3)).norm();
            std::cerr << "[CLAS-WLNL-FIX] pos_shift=" << shift << "m\n";
        }
    }

    // MRTKLIB publishes the fixed solution from xa/Pa while the float filter
    // x/P is only nudged by holdamb() (mrtk_ppp_rtk.c:2355-2361).
    const bool use_constrained_fixed_state =
        ambiguity_fixed_epoch &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
        last_clas_constrained_fixed_state_valid_ &&
        (env_overrides_.clas_resamb ||
         (ppp_config_.kinematic_mode && ppp_config_.use_clas_osr_filter));
    const PPPState& solution_filter_state =
        use_constrained_fixed_state ? last_clas_constrained_fixed_state_ : filter_state_;

    solution = ppp_clas::finalizeEpochSolution(
        solution_filter_state,
        obs.time,
        ambiguity_fixed_epoch,
        last_ar_ratio_,
        last_fixed_ambiguities_,
        static_cast<int>(osr_corrections.size()));

    if (wlnl_fixed_position_ok &&
        !ppp_config_.kinematic_mode &&
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
        const bool sd_ar_fixed =
            sd_ar_result.valid && sd_ar_result.ar_ratio >= 3.0;
        const bool apply_sd_mar_shift_gate =
            ppp_config_.kinematic_mode ||
            env_overrides_.clas_base_clock_parity;
        const bool sd_ar_position_ok =
            !apply_sd_mar_shift_gate ||
            sd_ar_result.position_shift_m <=
                kClasBaseClockParitySdMarMaxPositionShiftM;
        if (sd_ar_fixed && sd_ar_position_ok && !ppp_config_.kinematic_mode) {
            solution.position_ecef = sd_ar_result.position;
            solution.status = SolutionStatus::PPP_FIXED;
        } else if (sd_ar_fixed && pppDebugEnabled()) {
            std::cerr << "[CLAS-SD-MAR] reject"
                      << (ppp_config_.kinematic_mode ? " kinematic" : " parity")
                      << " pos_shift="
                      << sd_ar_result.position_shift_m
                      << " ratio=" << sd_ar_result.ar_ratio << "\n";
        }
    }

    // On the kinematic CLAS WLNL path the MRTKLIB-parity post-fix chi-square
    // gate (plus the maxdiffp guard below) already validates every fixed
    // publication, so the custom float-jump/continuity gates are skipped: the
    // float filter itself carries meter-level error, and a correct fix
    // legitimately jumps away from the previously published float position.
    bool clas_kinematic_fix_rejected = false;
    if (ppp_config_.kinematic_mode &&
        solution.status == SolutionStatus::PPP_FIXED &&
        !(kinematic_clas_wlnl_hold_path && ambiguity_fixed_epoch)) {
        const double dt_seconds =
            has_last_processed_time_ ? obs.time - last_processed_time_ : 0.2;
        const double max_fixed_float_jump_m = clasKinematicMaxFixedFloatJumpM(
            dt_seconds,
            clas_float_horizontal_sigma_m);
        const Vector3d post_ar_filter_position =
            solution_filter_state.state.segment(solution_filter_state.pos_index, 3);
        double fixed_float_jump_m =
            (solution.position_ecef - clas_float_position_ecef).norm();
        if (wlnl_fixed_position_ok) {
            fixed_float_jump_m = std::max(
                fixed_float_jump_m,
                (wlnl_fixed_position - post_ar_filter_position).norm());
        }
        const bool ratio_ok =
            solution.ratio <= 0.0 ||
            solution.ratio >= (kinematic_clas_wlnl_hold_path
                 ? ppp_config_.ar_ratio_threshold
                 : std::max(
                       kClasKinematicMinFixRatio,
                       ppp_config_.ar_ratio_threshold + 0.5));
        const bool wlnl_shift_ok =
            !wlnl_fixed_position_ok ||
            (wlnl_fixed_position - post_ar_filter_position).norm() <=
                kClasKinematicWlnlMaxPositionShiftM;
        double continuity_jump_m = 0.0;
        if (has_last_published_solution_position_) {
            continuity_jump_m = (solution.position_ecef -
                last_published_solution_position_ecef_).norm();
        }
        const bool continuity_ok =
            !has_last_published_solution_position_ ||
            continuity_jump_m <= max_fixed_float_jump_m;
        const bool float_jump_ok =
            std::isfinite(fixed_float_jump_m) &&
            fixed_float_jump_m <= max_fixed_float_jump_m;
        const bool jump_ok =
            wlnl_shift_ok &&
            continuity_ok &&
            float_jump_ok &&
            ratio_ok;
        if (jump_ok) {
            ++clas_kinematic_fix_candidate_streak_;
        } else {
            clas_kinematic_fix_candidate_streak_ = 0;
        }
        const bool minfix_ok =
            clas_kinematic_fix_candidate_streak_ >= kClasKinematicMinFixCount;
        if (!jump_ok || !minfix_ok) {
            clas_kinematic_fix_rejected = true;
            solution.position_ecef = clas_float_position_ecef;
            solution.status = SolutionStatus::PPP_FLOAT;
            solution.ratio = 0.0;
            solution.num_fixed_ambiguities = 0;
            filter_state_ = clas_float_filter_state;
            ambiguity_states_ = clas_float_ambiguity_states;
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
            if (!jump_ok) {
                clas_kinematic_fix_candidate_streak_ = 0;
            }
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-KIN-FIX] reject"
                          << (!wlnl_shift_ok ? " wlnl_shift" :
                              !continuity_ok ? " continuity" :
                              !float_jump_ok ? " float_jump" :
                              !ratio_ok ? " ratio" :
                              " minfix")
                          << " jump=" << fixed_float_jump_m
                          << " continuity=" << continuity_jump_m
                          << " limit=" << max_fixed_float_jump_m
                          << " ratio=" << solution.ratio
                          << " streak=" << clas_kinematic_fix_candidate_streak_
                          << "\n";
            }
        }
    } else if (ppp_config_.kinematic_mode) {
        clas_kinematic_fix_candidate_streak_ = 0;
    }

    // MRTKLIB maxdiffp guard (mrtk_ppp_rtk.c:2333-2352). MRTKLIB applies it
    // with dynamics on (benchmark clas.toml: dynamics=true + pseudorange_diff);
    // without it the dynamics filter can enter a rejection spiral and coast
    // kilometers away on the velocity states.
    if (ppp_config_.kinematic_mode &&
        ppp_config_.use_clas_osr_filter &&
        seed.isValid()) {
        const Vector3d float_position =
            filter_state_.state.segment(filter_state_.pos_index, 3);
        const double spp_divergence_m =
            (float_position - seed.position_ecef).norm();
        if (spp_divergence_m > kMrtklibMaxSppDivergenceM) {
            ++clas_kinematic_spp_divergence_count_;
            if (clas_kinematic_spp_divergence_count_ >
                    kMrtklibMaxSppDivergenceEpochs) {
                if (pppDebugEnabled()) {
                    std::cerr << "[CLAS-KIN-MAXDIFFP] reset to SPP dist="
                              << spp_divergence_m << "\n";
                }
                filter_state_.state.segment(filter_state_.pos_index, 3) =
                    seed.position_ecef;
                for (int i = 0; i < 3; ++i) {
                    const int idx = filter_state_.pos_index + i;
                    filter_state_.covariance.row(idx).setZero();
                    filter_state_.covariance.col(idx).setZero();
                    const double spp_variance =
                        seed.position_covariance(i, i) > 0.0
                            ? seed.position_covariance(i, i)
                            : 100.0;
                    filter_state_.covariance(idx, idx) = spp_variance;
                }
                if (ppp_config_.use_dynamics_model &&
                    filter_state_.vel_index >= 0 &&
                    filter_state_.vel_index + 2 < filter_state_.covariance.rows()) {
                    filter_state_.state.segment(filter_state_.vel_index, 3).setZero();
                    for (int i = 0; i < 3; ++i) {
                        const int idx = filter_state_.vel_index + i;
                        filter_state_.covariance.row(idx).setZero();
                        filter_state_.covariance.col(idx).setZero();
                        filter_state_.covariance(idx, idx) =
                            ppp_config_.initial_velocity_variance;
                    }
                }
                solution.position_ecef = seed.position_ecef;
                solution.status = SolutionStatus::SPP;
                solution.ratio = 0.0;
                solution.num_fixed_ambiguities = 0;
                clas_kinematic_fix_rejected = false;
                clas_kinematic_fix_candidate_streak_ = 0;
                clas_kinematic_spp_divergence_count_ = 0;
                ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
            }
        } else {
            clas_kinematic_spp_divergence_count_ = 0;
        }
    }

    had_fixed_last_epoch_ =
        solution.status == SolutionStatus::PPP_FIXED && !clas_kinematic_fix_rejected;

    has_last_processed_time_ = true;
    last_processed_time_ = obs.time;
    ++total_epochs_processed_;

    applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
    if (ppp_config_.kinematic_mode && solution.isValid()) {
        last_published_solution_position_ecef_ = solution.position_ecef;
        has_last_published_solution_position_ = true;
    }
    return solution;
}

}  // namespace libgnss
