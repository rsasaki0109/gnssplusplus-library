#include "ppp_internal.hpp"

#include <libgnss++/algorithms/ppp_utils.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <vector>

namespace libgnss {

using namespace ppp_internal;

namespace {

bool validReceiverSeed(const Vector3d& receiver_position) {
    return std::isfinite(receiver_position.x()) &&
           std::isfinite(receiver_position.y()) &&
           std::isfinite(receiver_position.z()) &&
           receiver_position.norm() > 1000.0;
}

double wrapFractionalCycle(double value) {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    double wrapped = std::fmod(value + 0.5, 1.0);
    if (wrapped < 0.0) {
        wrapped += 1.0;
    }
    return wrapped - 0.5;
}

}  // namespace

bool PPPProcessor::initializeFilter(const ObservationData& obs,
                                    const NavigationData& nav,
                                    const PositionSolution* seed_solution) {
    Vector3d initial_position = Vector3d::Zero();
    double initial_clock_bias_m = 0.0;
    if (seed_solution != nullptr && seed_solution->isValid()) {
        initial_position = seed_solution->position_ecef;
        initial_clock_bias_m = seed_solution->receiver_clock_bias;
    } else {
        auto spp_solution = spp_processor_.processEpoch(obs, nav);
        if (spp_solution.isValid()) {
            initial_position = spp_solution.position_ecef;
            initial_clock_bias_m = spp_solution.receiver_clock_bias;
        } else if (validReceiverSeed(obs.receiver_position)) {
            initial_position = obs.receiver_position;
            initial_clock_bias_m = obs.receiver_clock_bias * constants::SPEED_OF_LIGHT;
        } else {
            return false;
        }
    }

    // Allocate per-system receiver clocks after the base states:
    // pos(3) + vel(3) + gps_clk(1) + glo_clk(1) + trop(1).
    filter_state_.gal_clock_index = -1;
    filter_state_.qzs_clock_index = -1;
    filter_state_.bds_clock_index = -1;
    // Without these states QZSS/Galileo/BeiDou share the GPS clock and any
    // receiver inter-system hardware bias leaks into code residuals. Native
    // MADOCA coherent mode needs QZSS by default because the L6 stream carries
    // J02/J03/J04 orbit/clock/bias corrections used by the bridge. Other
    // systems stay explicit env opt-ins so non-MADOCA paths remain unchanged.
    // Accepts a comma/space-separated system list ("qzs", "gal", "bds", "all")
    // for diagnostics or broader experiments.
    const bool default_madoca_qzs_clock =
        require_coherent_ssr_ && env_overrides_.madoca_qzss_clock;
    std::set<GNSSSystem> visible_systems;
    for (const auto& sat : obs.getSatellites()) {
        visible_systems.insert(sat.system);
    }
    filter_state_.trop_index = 8;  // keep original
    int isb_start = 9;
    if (env_overrides_.estimate_isb_gal && visible_systems.count(GNSSSystem::Galileo)) {
        filter_state_.gal_clock_index = isb_start++;
    }
    if ((env_overrides_.estimate_isb_qzs || default_madoca_qzs_clock) &&
        visible_systems.count(GNSSSystem::QZSS)) {
        filter_state_.qzs_clock_index = isb_start++;
    }
    if (env_overrides_.estimate_isb_bds && visible_systems.count(GNSSSystem::BeiDou)) {
        filter_state_.bds_clock_index = isb_start++;
    }

    // Reserve ionosphere states for visible satellites.
    //
    // In MADOCA per-frequency (est-stec, non-CLAS) mode the ionosphere states
    // are allocated lazily and pruned, so every per-satellite state lives above
    // amb_index. Skipping the fixed reservation here keeps the pruning/compaction
    // re-indexing simple (it only touches indices >= amb_index).
    const bool madoca_per_freq_est_stec =
        ppp_config_.estimate_ionosphere && !ppp_config_.use_ionosphere_free &&
        !ppp_config_.use_clas_osr_filter;
    int n_iono_states = 0;
    filter_state_.ionosphere_indices.clear();
    if (ppp_config_.estimate_ionosphere && !madoca_per_freq_est_stec) {
        filter_state_.iono_index = isb_start;
        for (const auto& sat : obs.getSatellites()) {
            filter_state_.ionosphere_indices[sat] = filter_state_.iono_index + n_iono_states;
            ++n_iono_states;
        }
    }
    const int n_isb = isb_start - 9;
    const int n_states = 9 + n_isb + n_iono_states;
    filter_state_.amb_index = 9 + n_isb + n_iono_states;
    filter_state_.state = VectorXd::Zero(n_states);
    filter_state_.covariance = MatrixXd::Identity(n_states, n_states);
    filter_state_.state.segment(filter_state_.pos_index, 3) = initial_position;
    filter_state_.state.segment(filter_state_.vel_index, 3).setZero();
    filter_state_.state(filter_state_.clock_index) = initial_clock_bias_m;
    filter_state_.state(filter_state_.glo_clock_index) = initial_clock_bias_m;
    // Initialize per-system clock states.
    if (filter_state_.gal_clock_index >= 0)
        filter_state_.state(filter_state_.gal_clock_index) = initial_clock_bias_m;
    if (filter_state_.qzs_clock_index >= 0)
        filter_state_.state(filter_state_.qzs_clock_index) = initial_clock_bias_m;
    if (filter_state_.bds_clock_index >= 0)
        filter_state_.state(filter_state_.bds_clock_index) = initial_clock_bias_m;
    filter_state_.state(filter_state_.trop_index) =
        ppp_config_.estimate_troposphere ?
            modeledZenithTroposphereDelayMeters(initial_position, obs.time) :
            kDefaultZenithDelayMeters;
    const bool use_broadcast_rtklib_model = !precise_products_loaded_;
    filter_state_.covariance.block(filter_state_.pos_index, filter_state_.pos_index, 3, 3) *=
        use_broadcast_rtklib_model ? ppp_config_.initial_position_variance : 100.0;
    filter_state_.covariance.block(filter_state_.vel_index, filter_state_.vel_index, 3, 3) *=
        use_broadcast_rtklib_model ? ppp_config_.initial_velocity_variance : 25.0;
    filter_state_.covariance(filter_state_.clock_index, filter_state_.clock_index) =
        use_broadcast_rtklib_model ? ppp_config_.initial_clock_variance : 1e8;
    filter_state_.covariance(filter_state_.glo_clock_index, filter_state_.glo_clock_index) =
        use_broadcast_rtklib_model ? ppp_config_.initial_clock_variance : 1e8;
    filter_state_.covariance(filter_state_.trop_index, filter_state_.trop_index) =
        use_broadcast_rtklib_model ? ppp_config_.initial_troposphere_variance : 25.0;
    // Per-system receiver clocks use the same epoch prior as GPS by default.
    const double system_clock_initial_variance =
        use_broadcast_rtklib_model ? ppp_config_.initial_clock_variance : 1e8;
    if (filter_state_.gal_clock_index >= 0)
        filter_state_.covariance(filter_state_.gal_clock_index, filter_state_.gal_clock_index) =
            system_clock_initial_variance;
    if (filter_state_.qzs_clock_index >= 0)
        filter_state_.covariance(filter_state_.qzs_clock_index, filter_state_.qzs_clock_index) =
            system_clock_initial_variance;
    if (filter_state_.bds_clock_index >= 0)
        filter_state_.covariance(filter_state_.bds_clock_index, filter_state_.bds_clock_index) =
            system_clock_initial_variance;
    // Initialize per-satellite ionosphere states. GNSS_PPP_INIT_IONO_VAR (>0)
    // overrides the per-satellite initial ionosphere covariance. The est-stec
    // KF has a rank-deficient gauge in (iono, N1, N2, cdtr) -- a uniform shift
    // (iono+Δ, λN+Δ, cdtr−Δ) leaves the phase observations invariant -- so
    // when the constellation membership shifts (e.g. orbit-fix dropping a sat
    // missing its SSR orbit correction) the null-space anchor moves and the
    // filter settles to a different common-mode iono level. A tight prior pins
    // this gauge: on the MADOCA est-stec parity workload, var=1 lets a
    // low-latitude station (ALIC) stack cleanly with the orbit-fix lever for
    // bridge-parity float, while the default (100) is correct at mid-latitude
    // (MIZU); the right value is station-dependent, so this is opt-in.
    const double iono_init_var = env_overrides_.init_iono_var > 0.0
                                     ? env_overrides_.init_iono_var
                                     : ppp_config_.initial_ionosphere_variance;
    if (ppp_config_.estimate_ionosphere) {
        for (const auto& [sat, idx] : filter_state_.ionosphere_indices) {
            filter_state_.state(idx) = 0.0;  // Initial iono delay = 0 (will be constrained by STEC)
            filter_state_.covariance(idx, idx) = iono_init_var;
        }
    }
    filter_state_.total_states = n_states;
    if (!ppp_config_.kinematic_mode || ppp_config_.low_dynamics_mode) {
        static_anchor_position_ = initial_position;
        has_static_anchor_position_ = true;
    }
    constrainStaticVelocityStates();
    constrainStaticAnchorPosition();
    return true;
}

void PPPProcessor::predictState(double dt, const PositionSolution* seed_solution) {
    if (!filter_initialized_) {
        return;
    }

    const bool use_broadcast_rtklib_model = !precise_products_loaded_;
    const bool use_dynamic_prediction =
        ppp_config_.kinematic_mode && (!use_broadcast_rtklib_model || ppp_config_.use_dynamics_model);
    MatrixXd F = MatrixXd::Identity(filter_state_.total_states, filter_state_.total_states);
    if (use_dynamic_prediction) {
        F.block(filter_state_.pos_index, filter_state_.vel_index, 3, 3) =
            MatrixXd::Identity(3, 3) * dt;
    }

    filter_state_.state = F * filter_state_.state;
    if (!ppp_config_.kinematic_mode || !use_dynamic_prediction) {
        filter_state_.state.segment(filter_state_.vel_index, 3).setZero();
    }

    MatrixXd Q = MatrixXd::Zero(filter_state_.total_states, filter_state_.total_states);
    if (!ppp_config_.kinematic_mode) {
        Q.block(filter_state_.pos_index, filter_state_.pos_index, 3, 3) =
            MatrixXd::Identity(3, 3) * ppp_config_.process_noise_position * dt;
    } else if (use_dynamic_prediction) {
        Q.block(filter_state_.pos_index, filter_state_.pos_index, 3, 3) =
            MatrixXd::Identity(3, 3) * ppp_config_.process_noise_position * dt;
    }
    if (use_dynamic_prediction) {
        Q.block(filter_state_.vel_index, filter_state_.vel_index, 3, 3) =
            MatrixXd::Identity(3, 3) * ppp_config_.process_noise_velocity * dt;
    } else {
        Q.block(filter_state_.vel_index, filter_state_.vel_index, 3, 3) =
            MatrixXd::Identity(3, 3) * 1e-12;
    }
    const double clock_process_noise =
        use_broadcast_rtklib_model ? ppp_config_.process_noise_clock : 100.0;
    Q(filter_state_.clock_index, filter_state_.clock_index) = clock_process_noise * dt;
    Q(filter_state_.glo_clock_index, filter_state_.glo_clock_index) = clock_process_noise * dt;
    // ISB process noise: an inter-system receiver hardware bias is essentially a
    // constant, so it must be a slow random walk (NOT white noise like the
    // receiver clock). A white-noise ISB re-estimates a free QZSS/Galileo clock
    // every epoch, which strips the high-elevation QZSS geometry of its absolute
    // (vertical) leverage and degrades the solution. Tight Q (override-tunable)
    // lets the ISB converge to the true bias and then hold.
    if (filter_state_.gal_clock_index >= 0)
        Q(filter_state_.gal_clock_index, filter_state_.gal_clock_index) =
            env_overrides_.isb_process_noise * dt;
    if (filter_state_.qzs_clock_index >= 0)
        Q(filter_state_.qzs_clock_index, filter_state_.qzs_clock_index) =
            env_overrides_.isb_process_noise * dt;
    if (filter_state_.bds_clock_index >= 0)
        Q(filter_state_.bds_clock_index, filter_state_.bds_clock_index) =
            env_overrides_.isb_process_noise * dt;
    Q(filter_state_.trop_index, filter_state_.trop_index) =
        ppp_config_.process_noise_troposphere * dt;
    // Ambiguity process noise for every appended state. Ionosphere states may
    // be lazily appended above amb_index (est-stec), so apply the ionosphere
    // process noise AFTERWARDS to overwrite those indices with the correct value.
    for (int idx = filter_state_.amb_index; idx < filter_state_.total_states; ++idx) {
        Q(idx, idx) = ppp_config_.process_noise_ambiguity * dt;
    }
    // Ionosphere process noise
    if (ppp_config_.estimate_ionosphere) {
        for (const auto& [sat, idx] : filter_state_.ionosphere_indices) {
            Q(idx, idx) = ppp_config_.process_noise_ionosphere * dt;
        }
    }

    filter_state_.covariance = F * filter_state_.covariance * F.transpose() + Q;
    if (use_broadcast_rtklib_model && seed_solution != nullptr && seed_solution->isValid()) {
        if (ppp_config_.reset_clock_to_spp_each_epoch || useLowDynamicsBroadcastSeedAssist()) {
            const double gps_clock_before_reset = filter_state_.state(filter_state_.clock_index);
            const auto reinitializeSystemClock = [&](int index) {
                if (index < 0) {
                    return;
                }
                const double inter_system_offset =
                    filter_state_.state(index) - gps_clock_before_reset;
                reinitializeScalarState(
                    index,
                    seed_solution->receiver_clock_bias + inter_system_offset,
                    ppp_config_.initial_clock_variance);
            };
            reinitializeScalarState(
                filter_state_.clock_index,
                seed_solution->receiver_clock_bias,
                ppp_config_.initial_clock_variance);
            reinitializeScalarState(
                filter_state_.glo_clock_index,
                seed_solution->receiver_clock_bias,
                ppp_config_.initial_clock_variance);
            reinitializeSystemClock(filter_state_.gal_clock_index);
            reinitializeSystemClock(filter_state_.qzs_clock_index);
            reinitializeSystemClock(filter_state_.bds_clock_index);
        }
        if (ppp_config_.kinematic_mode &&
            !ppp_config_.low_dynamics_mode &&
            !use_dynamic_prediction &&
            ppp_config_.reset_kinematic_position_to_spp_each_epoch) {
            reinitializeVectorState(
                filter_state_.pos_index,
                seed_solution->position_ecef,
                ppp_config_.initial_position_variance);
        }
    }
    constrainStaticVelocityStates();
    constrainStaticAnchorPosition();
}

bool PPPProcessor::updateFilter(const ObservationData& obs, const NavigationData& nav) {
    auto if_obs = formIonosphereFree(obs, nav);
    if (if_obs.size() < static_cast<size_t>(config_.min_satellites)) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP] insufficient IF observations: " << if_obs.size()
                      << " < " << config_.min_satellites << "\n";
        }
        return false;
    }

    applyPreciseCorrections(if_obs, nav, obs.time);
    ensureAmbiguityStates(if_obs);
    if (!ppp_config_.use_ionosphere_free && ppp_config_.estimate_ionosphere &&
        !ppp_config_.use_clas_osr_filter) {
        std::set<SatelliteId> observed_now;
        for (const auto& entry : if_obs) {
            if (entry.valid && entry.has_carrier_phase) {
                observed_now.insert(entry.satellite);
            }
        }
        pruneStaleEstStecStates(observed_now);
    }
    size_t valid_count = 0;
    for (const auto& entry : if_obs) {
        valid_count += entry.valid ? 1U : 0U;
    }
    if (valid_count < static_cast<size_t>(config_.min_satellites)) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP] insufficient corrected observations: " << valid_count
                      << " < " << config_.min_satellites << "\n";
        }
        return false;
    }

    const int filter_iterations = precise_products_loaded_ ? 3 : ppp_config_.filter_iterations;
    for (int iteration = 0; iteration < filter_iterations; ++iteration) {
        MeasurementEquation meas_eq = formMeasurementEquations(if_obs, nav, obs.time);

        if (meas_eq.observations.size() < config_.min_satellites) {
            if (pppDebugEnabled()) {
                std::cerr << "[PPP] insufficient measurement rows: " << meas_eq.observations.size()
                          << " < " << config_.min_satellites << "\n";
            }
            return false;
        }

        const MatrixXd innovation_covariance =
            meas_eq.design_matrix * filter_state_.covariance * meas_eq.design_matrix.transpose() +
            meas_eq.weight_matrix;
        const MatrixXd innovation_inverse =
            innovation_covariance.ldlt().solve(MatrixXd::Identity(
                innovation_covariance.rows(), innovation_covariance.cols()));
        const MatrixXd gain =
            filter_state_.covariance * meas_eq.design_matrix.transpose() * innovation_inverse;
        const VectorXd delta_state = gain * meas_eq.residuals;

        if (pppDebugEnabled()) {
            std::cerr << "[PPP] iter=" << iteration
                      << " rows=" << meas_eq.observations.size()
                      << " pos_delta=" << delta_state.segment(filter_state_.pos_index, 3).norm()
                      << " clock_delta=" << delta_state(filter_state_.clock_index)
                      << " trop_delta=" << delta_state(filter_state_.trop_index)
                      << "\n";
        }

        filter_state_.state += delta_state;
        constrainStaticAnchorPosition();

        const MatrixXd identity =
            MatrixXd::Identity(filter_state_.total_states, filter_state_.total_states);
        const MatrixXd kh = gain * meas_eq.design_matrix;
        filter_state_.covariance =
            (identity - kh) * filter_state_.covariance * (identity - kh).transpose() +
            gain * meas_eq.weight_matrix * gain.transpose();

        if (delta_state.segment(filter_state_.pos_index, 3).norm() < 1e-4 &&
            std::abs(delta_state(filter_state_.clock_index)) < 1e-3 &&
            std::abs(delta_state(filter_state_.trop_index)) < 1e-3) {
            break;
        }
    }

    if (pppDebugEnabled()) {
        std::cerr << "[PPP] state clock=" << filter_state_.state(filter_state_.clock_index)
                  << " trop=" << filter_state_.state(filter_state_.trop_index)
                  << " pos_norm=" << filter_state_.state.segment(filter_state_.pos_index, 3).norm()
                  << "\n";
    }

    // Save covariance before anchor/velocity constraints destroy cross-terms.
    pre_anchor_covariance_ = filter_state_.covariance;

    updateAmbiguityStates(obs);
    constrainStaticVelocityStates();
    constrainStaticAnchorPosition();
    return true;
}

void PPPProcessor::detectCycleSlips(const ObservationData& obs) {
    if (!ppp_config_.enable_cycle_slip_detection) {
        return;
    }

    constexpr double kMinimumGeometryFreeSlipThresholdMeters = 0.5;
    constexpr double kMinimumMwSlipThresholdMeters = 10.0;
    // Enable combination (GF/MW) slip detection in SSR mode even for static,
    // because MW averaging is needed for Wide-Lane AR.
    const bool use_combination_slip_detection =
        ppp_config_.kinematic_mode || (ssr_products_loaded_ && ppp_config_.use_ionosphere_free);

    for (const auto& satellite : obs.getSatellites()) {
        const std::vector<SignalType> primary_candidates =
            satellite.system == GNSSSystem::GPS ?
                std::vector<SignalType>{SignalType::GPS_L1CA, SignalType::GPS_L1P} :
                primarySignals(satellite.system);
        const std::vector<SignalType> secondary_candidates =
            satellite.system == GNSSSystem::GPS ?
                std::vector<SignalType>{SignalType::GPS_L2C, SignalType::GPS_L2P, SignalType::GPS_L5} :
                secondarySignals(satellite.system);
        const Observation* primary =
            findCarrierObservationForSignals(obs, satellite, primary_candidates);
        if (primary == nullptr) {
            if (pppDebugEnabled() && satellite.system == GNSSSystem::GPS) {
                // Debug: list all observations for this satellite
                std::cerr << "[PPP-SLIP] " << satellite.toString() << " primary=null, obs: ";
                for (const auto& m : obs.observations) {
                    if (m.satellite == satellite) {
                        std::cerr << static_cast<int>(m.signal) << "(cp=" << m.has_carrier_phase
                                  << ",v=" << m.valid << ",L=" << m.carrier_phase << ") ";
                    }
                }
                std::cerr << "\n";
            }
            continue;
        }

        // SSR phase-bias discontinuity slip: when the MADOCA Compact SSR phase
        // bias discontinuity counter changes, the satellite phase bias has been
        // re-referenced and the carrier ambiguity consistent with the old bias
        // is stale (MADOCALIB detslip_cssr, ppp.c:558). env-gated, default OFF.
        bool discnt_slip = false;
        if (env_overrides_.ssr_discnt_slip && ssr_products_loaded_) {
            Vector3d disc_orbit;
            double disc_clock = 0.0;
            std::map<uint8_t, double> disc_pbias;
            std::map<uint8_t, int> cur_discnt;
            ssr_products_.interpolateCorrection(
                satellite, obs.time, disc_orbit, disc_clock,
                nullptr, nullptr, &disc_pbias, nullptr, nullptr, nullptr, nullptr,
                0, nullptr, &cur_discnt);
            const auto prev_it = prev_phase_bias_discnt_.find(satellite);
            if (prev_it != prev_phase_bias_discnt_.end()) {
                for (const auto& [sig, dc] : cur_discnt) {
                    const auto pf = prev_it->second.find(sig);
                    if (pf != prev_it->second.end() && pf->second != dc) {
                        discnt_slip = true;
                        break;
                    }
                }
            }
            if (!cur_discnt.empty()) {
                prev_phase_bias_discnt_[satellite] = std::move(cur_discnt);
            }
        }

        auto& ambiguity = ambiguity_states_[satellite];
        bool lli_slip = false;
        for (const auto& measurement : obs.observations) {
            if (!(measurement.satellite == satellite) ||
                !measurement.valid ||
                !measurement.has_carrier_phase) {
                continue;
            }
            if (measurement.loss_of_lock) {
                lli_slip = true;
                break;
            }
        }

        bool gf_slip = false;
        bool mw_slip = false;
        bool have_gf = false;
        bool have_mw = false;
        double gf_m = 0.0;
        double mw_m = 0.0;
        double mw_lambda_wl = 0.0;

        const Observation* secondary =
            use_combination_slip_detection ?
                findCarrierObservationForSignals(obs, satellite, secondary_candidates) :
                nullptr;
        if (secondary != nullptr) {
            const double lambda1 = signalWavelengthMeters(*primary);
            const double lambda2 = signalWavelengthMeters(*secondary);
            if (lambda1 > 0.0 && lambda2 > 0.0) {
                gf_m = primary->carrier_phase * lambda1 - secondary->carrier_phase * lambda2;
                have_gf = std::isfinite(gf_m);
                if (have_gf &&
                    ambiguity.has_last_geometry_free &&
                    std::abs(gf_m - ambiguity.last_geometry_free_m) >
                        std::max(ppp_config_.cycle_slip_threshold,
                                 kMinimumGeometryFreeSlipThresholdMeters)) {
                    gf_slip = true;
                }
            }

            if (satellite.system != GNSSSystem::GLONASS &&
                primary->has_pseudorange && secondary->has_pseudorange &&
                std::isfinite(primary->pseudorange) && std::isfinite(secondary->pseudorange)) {
                const double f1 = signalFrequencyHz(*primary);
                const double f2 = signalFrequencyHz(*secondary);
                if (f1 > 0.0 && f2 > 0.0 && std::abs(f1 - f2) >= 1.0) {
                    mw_m = ppp_utils::calculateMelbourneWubbena(
                        primary->carrier_phase,
                        secondary->carrier_phase,
                        primary->pseudorange,
                        secondary->pseudorange,
                        f1,
                        f2);
                    mw_lambda_wl = constants::SPEED_OF_LIGHT / std::abs(f1 - f2);
                    have_mw = std::isfinite(mw_m);
                    if (have_mw &&
                        ambiguity.has_last_melbourne_wubbena &&
                        std::abs(mw_m - ambiguity.last_melbourne_wubbena_m) >
                            std::max(kMinimumMwSlipThresholdMeters,
                                     ppp_config_.cycle_slip_threshold * 100.0)) {
                        mw_slip = true;
                    }
                }
            }

        }

        if (lli_slip || gf_slip || mw_slip || discnt_slip) {
            if (pppDebugEnabled()) {
                std::string reason;
                if (lli_slip) {
                    reason += "lli";
                }
                if (gf_slip) {
                    if (!reason.empty()) {
                        reason += "+";
                    }
                    reason += "geometry-free";
                }
                if (mw_slip) {
                    if (!reason.empty()) {
                        reason += "+";
                    }
                    reason += "melbourne-wubbena";
                }
                if (discnt_slip) {
                    if (!reason.empty()) {
                        reason += "+";
                    }
                    reason += "ssr-discontinuity";
                }
                std::cerr << "[PPP] cycle slip reset " << satellite.toString()
                          << " reason=" << reason;
                if (have_gf) {
                    std::cerr << " gf_m=" << gf_m;
                }
                if (have_mw) {
                    std::cerr << " mw_m=" << mw_m;
                }
                std::cerr << "\n";
            }
            resetAmbiguity(satellite, primary->signal);
        }

        auto& updated_ambiguity = ambiguity_states_[satellite];
        if (have_gf) {
            updated_ambiguity.last_geometry_free_m = gf_m;
            updated_ambiguity.has_last_geometry_free = true;
        }
        if (have_mw) {
            updated_ambiguity.last_melbourne_wubbena_m = mw_m;
            updated_ambiguity.has_last_melbourne_wubbena = true;
            // Accumulate MW average for Wide-Lane AR.  Use the per-system
            // wide-lane wavelength (c / |f1 - f2|) rather than a hardcoded GPS
            // value so Galileo E1-E5a, BeiDou, and QZSS L1-L5 average in the
            // correct cycle units.
            const double lambda_wl = mw_lambda_wl > 0.0
                ? mw_lambda_wl
                : constants::SPEED_OF_LIGHT / (1575.42e6 - 1227.60e6);
            const double mw_cycles = mw_m / lambda_wl;
            if (!mw_slip) {
                updated_ambiguity.mw_sum_cycles += mw_cycles;
                updated_ambiguity.mw_count += 1;
                updated_ambiguity.mw_mean_cycles =
                    updated_ambiguity.mw_sum_cycles / updated_ambiguity.mw_count;
            } else {
                // Reset MW averaging on cycle slip
                updated_ambiguity.mw_sum_cycles = mw_cycles;
                updated_ambiguity.mw_count = 1;
                updated_ambiguity.mw_mean_cycles = mw_cycles;
                updated_ambiguity.wl_is_fixed = false;
            }
        }
    }
}

void PPPProcessor::ensureAmbiguityStates(const std::vector<IonosphereFreeObs>& observations) {
    const bool per_freq =
        !ppp_config_.use_ionosphere_free && ppp_config_.estimate_ionosphere;
    for (const auto& observation : observations) {
        if (!observation.valid || !observation.has_carrier_phase) {
            continue;
        }
        if (per_freq) {
            // Ensure the ionosphere state exists, then init L2 BEFORE L1 so the
            // shared needs_reinitialization flag (cleared by the L1 init) does
            // not skip the L2 re-init on a cycle slip.
            getOrCreateIonosphereState(observation);
            if (observation.has_carrier_phase_l2) {
                getOrCreateAmbiguityStateL2(observation);
            }
        }
        getOrCreateAmbiguityState(observation);
    }
}

void PPPProcessor::reinitializeVectorState(int start_index,
                                           const Vector3d& value,
                                           double variance) {
    for (int offset = 0; offset < 3; ++offset) {
        const int index = start_index + offset;
        filter_state_.state(index) = value(offset);
        filter_state_.covariance.row(index).setZero();
        filter_state_.covariance.col(index).setZero();
        filter_state_.covariance(index, index) = variance;
    }
}

void PPPProcessor::reinitializeScalarState(int index, double value, double variance) {
    filter_state_.state(index) = value;
    filter_state_.covariance.row(index).setZero();
    filter_state_.covariance.col(index).setZero();
    filter_state_.covariance(index, index) = variance;
}

bool PPPProcessor::useLowDynamicsBroadcastSeedAssist() const {
    return ppp_config_.kinematic_mode &&
           ppp_config_.low_dynamics_mode &&
           !precise_products_loaded_;
}

void PPPProcessor::recoverLowDynamicsBroadcastState(const ObservationData& obs,
                                                    const PositionSolution* seed_solution) {
    if (!useLowDynamicsBroadcastSeedAssist()) {
        return;
    }

    Vector3d recovered_position = static_anchor_position_;
    double recovered_clock_bias_m = 0.0;
    bool have_seed = false;
    if (seed_solution != nullptr && seed_solution->isValid()) {
        recovered_position = seed_solution->position_ecef;
        recovered_clock_bias_m = seed_solution->receiver_clock_bias;
        have_seed = true;
    } else if (validReceiverSeed(obs.receiver_position)) {
        recovered_position = obs.receiver_position;
        recovered_clock_bias_m = obs.receiver_clock_bias * constants::SPEED_OF_LIGHT;
        have_seed = true;
    }
    if (!have_seed) {
        return;
    }

    if (!has_static_anchor_position_) {
        static_anchor_position_ = recovered_position;
        has_static_anchor_position_ = true;
    }

    const Vector3d anchored_position =
        has_static_anchor_position_ ?
            0.85 * static_anchor_position_ + 0.15 * recovered_position :
            recovered_position;
    const double gps_clock_before_reset = filter_state_.state(filter_state_.clock_index);
    const auto reinitializeSystemClock = [&](int index) {
        if (index < 0) {
            return;
        }
        const double inter_system_offset =
            filter_state_.state(index) - gps_clock_before_reset;
        reinitializeScalarState(
            index,
            recovered_clock_bias_m + inter_system_offset,
            ppp_config_.initial_clock_variance);
    };
    reinitializeVectorState(
        filter_state_.pos_index,
        anchored_position,
        std::min(ppp_config_.initial_position_variance, 36.0));
    reinitializeScalarState(
        filter_state_.clock_index,
        recovered_clock_bias_m,
        ppp_config_.initial_clock_variance);
    reinitializeScalarState(
        filter_state_.glo_clock_index,
        recovered_clock_bias_m,
        ppp_config_.initial_clock_variance);
    reinitializeSystemClock(filter_state_.gal_clock_index);
    reinitializeSystemClock(filter_state_.qzs_clock_index);
    reinitializeSystemClock(filter_state_.bds_clock_index);
    reinitializeScalarState(
        filter_state_.trop_index,
        modeledZenithTroposphereDelayMeters(anchored_position, obs.time),
        ppp_config_.initial_troposphere_variance);

    std::vector<SatelliteId> satellites_to_reset;
    satellites_to_reset.reserve(ambiguity_states_.size());
    for (const auto& [satellite, ambiguity] : ambiguity_states_) {
        (void)ambiguity;
        satellites_to_reset.push_back(satellite);
    }
    for (const auto& satellite : satellites_to_reset) {
        resetAmbiguity(satellite, SignalType::GPS_L1CA);
    }

    recent_positions_.clear();
    converged_ = false;
    convergence_start_time_ = obs.time;
    constrainStaticVelocityStates();
    constrainStaticAnchorPosition();
}

int PPPProcessor::receiverClockStateIndex(const SatelliteId& satellite) const {
    switch (satellite.system) {
        case GNSSSystem::GLONASS:
            return filter_state_.glo_clock_index;
        case GNSSSystem::Galileo:
            return filter_state_.gal_clock_index >= 0
                ? filter_state_.gal_clock_index : filter_state_.clock_index;
        case GNSSSystem::QZSS:
            return filter_state_.qzs_clock_index >= 0
                ? filter_state_.qzs_clock_index : filter_state_.clock_index;
        case GNSSSystem::BeiDou:
            return filter_state_.bds_clock_index >= 0
                ? filter_state_.bds_clock_index : filter_state_.clock_index;
        default:
            return filter_state_.clock_index;
    }
}

double PPPProcessor::receiverClockBiasMeters(const SatelliteId& satellite) const {
    return filter_state_.state(receiverClockStateIndex(satellite));
}

int PPPProcessor::ambiguityStateIndex(const SatelliteId& satellite) const {
    const auto it = filter_state_.ambiguity_indices.find(satellite);
    return it == filter_state_.ambiguity_indices.end() ? -1 : it->second;
}

int PPPProcessor::getOrCreateAmbiguityState(const IonosphereFreeObs& observation) {
    const auto existing = filter_state_.ambiguity_indices.find(observation.satellite);
    if (existing != filter_state_.ambiguity_indices.end()) {
        auto& ambiguity = ambiguity_states_[observation.satellite];
        if (ambiguity.needs_reinitialization) {
            initializeAmbiguityState(observation, existing->second);
        }
        return existing->second;
    }

    const int new_index = filter_state_.total_states;
    filter_state_.total_states += 1;
    filter_state_.state.conservativeResize(filter_state_.total_states);
    filter_state_.state(new_index) = 0.0;
    filter_state_.covariance.conservativeResize(filter_state_.total_states, filter_state_.total_states);
    filter_state_.covariance.row(new_index).setZero();
    filter_state_.covariance.col(new_index).setZero();
    filter_state_.covariance(new_index, new_index) =
        precise_products_loaded_ ? 100.0 : ppp_config_.initial_ambiguity_variance;
    filter_state_.ambiguity_indices[observation.satellite] = new_index;
    initializeAmbiguityState(observation, new_index);
    return new_index;
}

void PPPProcessor::initializeAmbiguityState(const IonosphereFreeObs& observation, int state_index) {
    const Vector3d receiver_position =
        observation.receiver_position.norm() > 1000.0 ?
            observation.receiver_position :
            filter_state_.state.segment(filter_state_.pos_index, 3);
    const double clock_bias_m = receiverClockBiasMeters(observation.satellite);
    const double zenith_delay = filter_state_.state(filter_state_.trop_index);
    const double predicted =
        geodist(observation.satellite_position, receiver_position) +
        clock_bias_m -
        constants::SPEED_OF_LIGHT * observation.satellite_clock_bias +
        (ppp_config_.estimate_troposphere ?
            observation.trop_mapping * zenith_delay :
            observation.modeled_trop_delay_m);
    auto& ambiguity = ambiguity_states_[observation.satellite];
    if (!ppp_config_.use_ionosphere_free && ppp_config_.estimate_ionosphere) {
        // Per-frequency L1 ambiguity in meters, seeded geometry-free consistent
        // with the ionosphere state: bias_1 = L1 - P1 + 2*ion*(f1/f1)^2.
        // (f1/f1)^2 = 1; this removes the +/-iono on code/phase so the residual
        // is the integer-recoverable phase bias lambda1*N1.
        const double ion = observation.has_iono_init ? observation.iono_init_m : 0.0;
        filter_state_.state(state_index) =
            observation.carrier_phase_l1 - observation.pseudorange_l1 + 2.0 * ion;
        filter_state_.covariance(state_index, state_index) =
            ppp_config_.initial_ambiguity_variance;
        ambiguity.wavelength_l1 = observation.wavelength_l1;
        ambiguity.float_value_l1 = filter_state_.state(state_index);
    } else {
        filter_state_.state(state_index) = observation.carrier_phase_if - predicted;
        filter_state_.covariance(state_index, state_index) =
            precise_products_loaded_ ? 25.0 : ppp_config_.initial_ambiguity_variance;
    }
    ambiguity.float_value = filter_state_.state(state_index);
    ambiguity.is_fixed = false;
    ambiguity.lock_count = 0;
    ambiguity.quality_indicator = 0.0;
    ambiguity.ambiguity_scale_m = observation.ambiguity_scale_m;
    ambiguity.needs_reinitialization = false;
}

int PPPProcessor::getOrCreateIonosphereState(const IonosphereFreeObs& observation) {
    const auto existing =
        filter_state_.ionosphere_indices.find(observation.satellite);
    if (existing != filter_state_.ionosphere_indices.end()) {
        return existing->second;
    }
    const int new_index = filter_state_.total_states;
    filter_state_.total_states += 1;
    filter_state_.state.conservativeResize(filter_state_.total_states);
    filter_state_.covariance.conservativeResize(
        filter_state_.total_states, filter_state_.total_states);
    filter_state_.covariance.row(new_index).setZero();
    filter_state_.covariance.col(new_index).setZero();
    filter_state_.state(new_index) =
        observation.has_iono_init ? observation.iono_init_m : 0.0;
    // Mirror the GNSS_PPP_INIT_IONO_VAR override at the lazy-create site so a
    // satellite appearing mid-run lands in the same anchored gauge.
    filter_state_.covariance(new_index, new_index) =
        env_overrides_.init_iono_var > 0.0
            ? env_overrides_.init_iono_var
            : ppp_config_.initial_ionosphere_variance;
    filter_state_.ionosphere_indices[observation.satellite] = new_index;
    return new_index;
}

int PPPProcessor::getOrCreateAmbiguityStateL2(const IonosphereFreeObs& observation) {
    auto& ambiguity = ambiguity_states_[observation.satellite];
    const auto existing =
        filter_state_.ambiguity_l2_indices.find(observation.satellite);
    int index = -1;
    if (existing != filter_state_.ambiguity_l2_indices.end()) {
        index = existing->second;
        if (!ambiguity.needs_reinitialization) {
            return index;
        }
    } else {
        index = filter_state_.total_states;
        filter_state_.total_states += 1;
        filter_state_.state.conservativeResize(filter_state_.total_states);
        filter_state_.covariance.conservativeResize(
            filter_state_.total_states, filter_state_.total_states);
        filter_state_.covariance.row(index).setZero();
        filter_state_.covariance.col(index).setZero();
        filter_state_.ambiguity_l2_indices[observation.satellite] = index;
    }
    // Per-frequency L2 ambiguity in meters: bias_2 = L2 - P2 + 2*ion*(f1/f2)^2.
    const double ion = observation.has_iono_init ? observation.iono_init_m : 0.0;
    const double ratio =
        observation.freq_l2 > 0.0 ? (observation.freq_l1 / observation.freq_l2) : 0.0;
    filter_state_.state(index) = observation.carrier_phase_l2 -
                                 observation.pseudorange_l2 +
                                 2.0 * ion * ratio * ratio;
    filter_state_.covariance(index, index) = ppp_config_.initial_ambiguity_variance;
    ambiguity.wavelength_l2 = observation.wavelength_l2;
    ambiguity.float_value_l2 = filter_state_.state(index);
    return index;
}

void PPPProcessor::pruneStaleEstStecStates(const std::set<SatelliteId>& observed) {
    if (ppp_config_.use_ionosphere_free || !ppp_config_.estimate_ionosphere ||
        ppp_config_.use_clas_osr_filter) {
        return;
    }
    constexpr int kMaxOutageEpochs = 5;
    std::set<SatelliteId> drop;
    for (const auto& [sat, idx] : filter_state_.ambiguity_indices) {
        (void)idx;
        if (observed.count(sat)) {
            est_stec_outage_[sat] = 0;
        } else if (++est_stec_outage_[sat] > kMaxOutageEpochs) {
            drop.insert(sat);
        }
    }
    if (drop.empty()) {
        return;
    }

    // Collect the state indices to remove (all >= amb_index in this mode).
    std::set<int> remove;
    auto collect = [&](const std::map<SatelliteId, int>& m, const SatelliteId& s) {
        const auto it = m.find(s);
        if (it != m.end() && it->second >= filter_state_.amb_index &&
            it->second < filter_state_.total_states) {
            remove.insert(it->second);
        }
    };
    for (const auto& sat : drop) {
        collect(filter_state_.ambiguity_indices, sat);
        collect(filter_state_.ambiguity_l2_indices, sat);
        collect(filter_state_.ionosphere_indices, sat);
    }
    const int old_n = filter_state_.total_states;
    std::vector<int> new_index(static_cast<size_t>(old_n), -1);
    int next = 0;
    for (int i = 0; i < old_n; ++i) {
        if (remove.count(i) == 0) {
            new_index[static_cast<size_t>(i)] = next++;
        }
    }
    const int new_n = next;
    if (new_n != old_n) {
        VectorXd new_state(new_n);
        MatrixXd new_cov = MatrixXd::Zero(new_n, new_n);
        for (int i = 0; i < old_n; ++i) {
            const int ni = new_index[static_cast<size_t>(i)];
            if (ni < 0) {
                continue;
            }
            new_state(ni) = filter_state_.state(i);
            for (int j = 0; j < old_n; ++j) {
                const int nj = new_index[static_cast<size_t>(j)];
                if (nj < 0) {
                    continue;
                }
                new_cov(ni, nj) = filter_state_.covariance(i, j);
            }
        }
        filter_state_.state = new_state;
        filter_state_.covariance = new_cov;
        filter_state_.total_states = new_n;
    }
    auto reindex = [&](std::map<SatelliteId, int>& m) {
        for (auto it = m.begin(); it != m.end();) {
            const int ni = (it->second >= 0 && it->second < old_n)
                               ? new_index[static_cast<size_t>(it->second)]
                               : -1;
            if (ni < 0) {
                it = m.erase(it);
            } else {
                it->second = ni;
                ++it;
            }
        }
    };
    reindex(filter_state_.ambiguity_indices);
    reindex(filter_state_.ambiguity_l2_indices);
    reindex(filter_state_.ionosphere_indices);
    for (const auto& sat : drop) {
        ambiguity_states_.erase(sat);
        est_stec_outage_.erase(sat);
    }
}

void PPPProcessor::updateAmbiguityStates(const ObservationData& obs) {
    for (const auto& measurement : obs.observations) {
        if (!measurement.valid || !measurement.has_carrier_phase) {
            continue;
        }
        auto& ambiguity = ambiguity_states_[measurement.satellite];
        ambiguity.last_phase = measurement.carrier_phase;
        ambiguity.last_time = obs.time;
        ambiguity.lock_count++;
        ambiguity.quality_indicator = measurement.snr;
        const int ambiguity_index = ambiguityStateIndex(measurement.satellite);
        if (ambiguity_index >= 0 && ambiguity_index < filter_state_.total_states) {
            ambiguity.float_value = filter_state_.state(ambiguity_index);
            if (!ppp_config_.kinematic_mode &&
                precise_products_loaded_ &&
                ambiguity.ambiguity_scale_m > 0.0) {
                const double cycles = ambiguity.float_value / ambiguity.ambiguity_scale_m;
                const double fractional_cycles = wrapFractionalCycle(cycles);
                if (ambiguity.fractional_bias_samples == 0) {
                    ambiguity.fractional_bias_cycles = fractional_cycles;
                    ambiguity.fractional_bias_samples = 1;
                } else {
                    double aligned_fraction = fractional_cycles;
                    while (aligned_fraction - ambiguity.fractional_bias_cycles > 0.5) {
                        aligned_fraction -= 1.0;
                    }
                    while (aligned_fraction - ambiguity.fractional_bias_cycles < -0.5) {
                        aligned_fraction += 1.0;
                    }
                    ambiguity.fractional_bias_cycles =
                        wrapFractionalCycle(
                            (ambiguity.fractional_bias_cycles *
                                 static_cast<double>(ambiguity.fractional_bias_samples) +
                             aligned_fraction) /
                            static_cast<double>(ambiguity.fractional_bias_samples + 1));
                    ambiguity.fractional_bias_samples++;
                }
            }
        }
    }
}

void PPPProcessor::resetAmbiguity(const SatelliteId& satellite, SignalType signal) {
    (void)signal;
    auto& ambiguity = ambiguity_states_[satellite];
    ambiguity = PPPAmbiguityInfo{};
    ambiguity.needs_reinitialization = true;

    const int ambiguity_index = ambiguityStateIndex(satellite);
    if (ambiguity_index >= 0 && ambiguity_index < filter_state_.total_states) {
        filter_state_.state(ambiguity_index) = 0.0;
        filter_state_.covariance.row(ambiguity_index).setZero();
        filter_state_.covariance.col(ambiguity_index).setZero();
        filter_state_.covariance(ambiguity_index, ambiguity_index) =
            precise_products_loaded_ ? 1e6 : ppp_config_.initial_ambiguity_variance;
    }

    // Mirror the reset for the per-frequency (est-stec) L2 ambiguity state.
    // Without this the L2 bias is left at its pre-slip value with its tight
    // covariance: initializeAmbiguityState (L1) clears needs_reinitialization
    // before getOrCreateAmbiguityStateL2 runs, so the L2 re-seed is skipped and
    // the filter stays anchored to the now-wrong L2 ambiguity — the stuck
    // ~10 m L2 phase residuals observed on e.g. G04/G11/G31. Zeroing the state
    // and re-inflating the variance lets the filter re-converge L2 from the
    // post-slip observations.
    // DEFAULT OFF (opt-in via GNSS_PPP_L2_RESET_FIX=1): the L1/L2 asymmetry is a
    // genuine latent bug, but on the MADOCA parity datasets cycle slips are rare
    // so resetAmbiguity is seldom called and the fix is bit-identical there (the
    // stuck ~10 m L2 residuals come from frozen wrong initial ambiguities, not
    // slip resets — process noise 1e-8). Kept opt-in, default behaviour = HEAD.
    const auto l2_it = env_overrides_.l2_reset_fix
                           ? filter_state_.ambiguity_l2_indices.find(satellite)
                           : filter_state_.ambiguity_l2_indices.end();
    if (l2_it != filter_state_.ambiguity_l2_indices.end()) {
        const int l2_index = l2_it->second;
        if (l2_index >= 0 && l2_index < filter_state_.total_states) {
            filter_state_.state(l2_index) = 0.0;
            filter_state_.covariance.row(l2_index).setZero();
            filter_state_.covariance.col(l2_index).setZero();
            filter_state_.covariance(l2_index, l2_index) =
                precise_products_loaded_ ? 1e6 : ppp_config_.initial_ambiguity_variance;
        }
    }
}

void PPPProcessor::constrainStaticVelocityStates() {
    if ((ppp_config_.kinematic_mode && !ppp_config_.low_dynamics_mode) ||
        filter_state_.total_states < filter_state_.vel_index + 3) {
        return;
    }

    filter_state_.state.segment(filter_state_.vel_index, 3).setZero();
    for (int axis = 0; axis < 3; ++axis) {
        const int idx = filter_state_.vel_index + axis;
        filter_state_.covariance.row(idx).setZero();
        filter_state_.covariance.col(idx).setZero();
        filter_state_.covariance(idx, idx) = 1e-12;
    }
}

void PPPProcessor::constrainStaticAnchorPosition() {
    if ((ppp_config_.kinematic_mode && !ppp_config_.low_dynamics_mode) ||
        !has_static_anchor_position_) {
        return;
    }
    const bool madoca_static_anchor_blend =
        require_coherent_ssr_ &&
        !env_overrides_.disable_madoca_static_anchor;
    if (!ppp_config_.apply_static_anchor_blend && !madoca_static_anchor_blend) {
        return;
    }
    if (precise_products_loaded_ && !ppp_config_.estimate_troposphere) {
        return;
    }
    // SSR mode note: hard blend (70% SPP) caps accuracy at ~4m but prevents
    // divergence. Soft anchor (pseudo-observation) tested with sigma=3-20m:
    //   3m: 4.3m (worse than hard), 5m: 4.8m, 10m: 6.2m, inf: 10.6m (diverges)
    // Root cause of 4m floor is 20-40m SSR residual spread (light travel time).
    // Fix the residuals first, then loosen the anchor.

    const double default_anchor_blend =
        madoca_static_anchor_blend && !ppp_config_.apply_static_anchor_blend
            ? 0.30
            : (ppp_config_.low_dynamics_mode
                   ? (precise_products_loaded_ ? (converged_ ? 0.65 : 0.9) : 0.95)
                   : (precise_products_loaded_ ? (converged_ ? 0.5 : 0.85)
                      : (ssr_products_loaded_
                             ? (converged_ ? 0.5 : 0.7)
                             : 1.0)));
    const double configured_anchor_blend = env_overrides_.static_anchor_blend;
    const double anchor_blend =
        configured_anchor_blend >= 0.0 && configured_anchor_blend <= 1.0
            ? configured_anchor_blend
            : default_anchor_blend;
    filter_state_.state.segment(filter_state_.pos_index, 3) =
        anchor_blend * static_anchor_position_ +
        (1.0 - anchor_blend) * filter_state_.state.segment(filter_state_.pos_index, 3);
    if (precise_products_loaded_) {
        Vector3d deviation =
            filter_state_.state.segment(filter_state_.pos_index, 3) - static_anchor_position_;
        const double deviation_norm = deviation.norm();
        const double max_static_deviation_m =
            ppp_config_.low_dynamics_mode
                ? (converged_ ? 15.0 : 8.0)
                : (converged_ ? 100.0 : 25.0);
        if (std::isfinite(deviation_norm) && deviation_norm > max_static_deviation_m) {
            deviation *= max_static_deviation_m / deviation_norm;
            filter_state_.state.segment(filter_state_.pos_index, 3) =
                static_anchor_position_ + deviation;
        }
    } else if (ppp_config_.low_dynamics_mode) {
        Vector3d deviation =
            filter_state_.state.segment(filter_state_.pos_index, 3) - static_anchor_position_;
        const double deviation_norm = deviation.norm();
        const double max_static_deviation_m = converged_ ? 10.0 : 6.0;
        if (std::isfinite(deviation_norm) && deviation_norm > max_static_deviation_m) {
            deviation *= max_static_deviation_m / deviation_norm;
            filter_state_.state.segment(filter_state_.pos_index, 3) =
                static_anchor_position_ + deviation;
        }
    }
    filter_state_.covariance.block(filter_state_.pos_index, filter_state_.pos_index, 3, 3) =
        MatrixXd::Identity(3, 3) *
        (ppp_config_.low_dynamics_mode
             ? (precise_products_loaded_ ? 4.0 : 9.0)
             : (precise_products_loaded_ ? 9.0 : 4.0));
    for (int axis = 0; axis < 3; ++axis) {
        const int idx = filter_state_.pos_index + axis;
        for (int col = 0; col < filter_state_.total_states; ++col) {
            if (col >= filter_state_.pos_index && col < filter_state_.pos_index + 3) {
                continue;
            }
            filter_state_.covariance(idx, col) = 0.0;
            filter_state_.covariance(col, idx) = 0.0;
        }
    }

    const double min_trop = 0.5;
    const double max_trop = precise_products_loaded_ ? 30.0 : 100.0;
    filter_state_.state(filter_state_.trop_index) =
        std::clamp(filter_state_.state(filter_state_.trop_index), min_trop, max_trop);
    filter_state_.covariance(filter_state_.trop_index, filter_state_.trop_index) =
        std::min(filter_state_.covariance(filter_state_.trop_index, filter_state_.trop_index), 25.0);
}


}  // namespace libgnss
