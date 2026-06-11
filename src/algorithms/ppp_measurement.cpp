#include "ppp_internal.hpp"

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

namespace libgnss {

using namespace ppp_internal;

PPPProcessor::MeasurementEquation PPPProcessor::formMeasurementEquations(
    const std::vector<IonosphereFreeObs>& observations,
    const NavigationData& nav,
    const GNSSTime& time) {
    (void)nav;
    (void)time;

    std::vector<Eigen::RowVectorXd> rows;
    std::vector<double> measured_values;
    std::vector<double> predicted_values;
    std::vector<double> variances;
    std::vector<SatelliteId> row_satellites;
    std::vector<bool> row_is_phase;

    const double zenith_delay = filter_state_.state(filter_state_.trop_index);
    const bool use_phase_rows =
        ppp_config_.use_carrier_phase_without_precise_products || precise_products_loaded_ || ssr_products_loaded_;

    for (const auto& observation : observations) {
        if (!observation.valid) {
            continue;
        }

        const Vector3d receiver_position =
            observation.receiver_position.norm() > 1000.0 ?
                observation.receiver_position :
                filter_state_.state.segment(filter_state_.pos_index, 3);

        const Vector3d range_vector = observation.satellite_position - receiver_position;
        const double euclidean_range = range_vector.norm();
        if (!std::isfinite(euclidean_range) || euclidean_range <= 1.0) {
            continue;
        }

        const double geometric_range = geodist(observation.satellite_position, receiver_position);
        const Vector3d line_of_sight = range_vector / euclidean_range;
        // Troposphere: by default the estimated zenith total delay is mapped with
        // the hydrostatic function. In per-frequency (est-stec) mode use the
        // RTKLIB-style split trop = m_h*ZHD + m_w*(ZTD-ZHD) so the estimated wet
        // residual is mapped with the (steeper, low-elevation) wet function; the
        // partial w.r.t. the ZTD state is then m_w.
        // Wet-mapping split is physically correct but relies on an accurate a
        // priori ZHD; the climatology ZHD is inaccurate at high/dry stations
        // (helped MIZU 0.42->0.385 m but hurt ALIC 0.34->0.585 m), so it is OFF
        // by default and opt-in via GNSS_PPP_PF_WET_TROP for experimentation.
        const bool per_freq_wet_trop =
            env_overrides_.pf_wet_trop && ppp_config_.estimate_troposphere &&
            !ppp_config_.use_ionosphere_free && ppp_config_.estimate_ionosphere &&
            !ppp_config_.use_clas_osr_filter && observation.trop_mapping_wet > 0.0;
        double troposphere_delay;
        double trop_partial;
        if (per_freq_wet_trop) {
            const double zhd = observation.trop_zhd_m;
            troposphere_delay = observation.trop_mapping * zhd +
                                observation.trop_mapping_wet * (zenith_delay - zhd);
            trop_partial = observation.trop_mapping_wet;
        } else if (ppp_config_.estimate_troposphere) {
            troposphere_delay = observation.trop_mapping * zenith_delay;
            trop_partial = observation.trop_mapping;
        } else {
            troposphere_delay = observation.modeled_trop_delay_m;
            trop_partial = 0.0;
        }
        const double clock_bias_m = receiverClockBiasMeters(observation.satellite);

        Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(filter_state_.total_states);
        row.segment(filter_state_.pos_index, 3) = -line_of_sight;
        const int clock_state_index = receiverClockStateIndex(observation.satellite);
        row(clock_state_index) = 1.0;
        row(filter_state_.trop_index) = trop_partial;
        // Per-satellite ionosphere state: pseudorange has +iono contribution
        double iono_state_m = 0.0;
        if (ppp_config_.estimate_ionosphere) {
            const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
            if (iono_it != filter_state_.ionosphere_indices.end()) {
                row(iono_it->second) = 1.0;  // +iono for pseudorange
                iono_state_m = filter_state_.state(iono_it->second);
            }
        }

        const double predicted =
            geometric_range + clock_bias_m -
            constants::SPEED_OF_LIGHT * observation.satellite_clock_bias + troposphere_delay
            + iono_state_m + observation.rx_ant_corr_l1_m;
        const double residual = observation.pseudorange_if - predicted;

        // Env-gated pre-fit residual dump for native-vs-bridge measurement diff.
        // Matches the bridge ppp_res (post=0) lines so per-system/per-signal
        // residual structure can be compared at convergence. el in degrees.
        auto dumpRes = [&](const char* band, const char* type, double res) {
            std::cerr << "[PPP-RES] tow=" << std::fixed << std::setprecision(1)
                      << time.tow << " sat=" << observation.satellite.toString()
                      << " band=" << band << " type=" << type
                      << " res=" << std::setprecision(4) << res
                      << " el=" << std::setprecision(1)
                      << (observation.elevation * 57.295779513) << "\n";
        };
        if (env_overrides_.res_dump) dumpRes("L1", "code", residual);

        // Deep state dump: the per-satellite (iono, N1, N2) split that the
        // est-stec filter settles into. Compared against the bridge ppp_res
        // dion/bias to find where native admits a wrong-but-self-consistent
        // (ambiguity, ionosphere) solution that the bridge does not.
        if (env_overrides_.res_dump) {
            const int n1_idx = ambiguityStateIndex(observation.satellite);
            const auto n2_it =
                filter_state_.ambiguity_l2_indices.find(observation.satellite);
            const double n1 =
                (n1_idx >= 0 && n1_idx < filter_state_.total_states)
                    ? filter_state_.state(n1_idx)
                    : 0.0;
            const double n2 =
                (n2_it != filter_state_.ambiguity_l2_indices.end())
                    ? filter_state_.state(n2_it->second)
                    : 0.0;
            std::cerr << "[PPP-STATE] tow=" << std::fixed << std::setprecision(1)
                      << time.tow << " sat=" << observation.satellite.toString()
                      << " iono=" << std::setprecision(4) << iono_state_m
                      << " n1=" << n1 << " n2=" << n2
                      << " cdtr=" << clock_bias_m
                      << " el=" << std::setprecision(1)
                      << (observation.elevation * 57.295779513) << "\n";
        }

        if (pppDebugEnabled()) {
            std::cerr << "[PPP-OBS] " << observation.satellite.toString()
                      << " pr=" << observation.pseudorange_if
                      << " geo=" << geometric_range
                      << " clk=" << clock_bias_m
                      << " satclk=" << (constants::SPEED_OF_LIGHT * observation.satellite_clock_bias)
                      << " trop=" << troposphere_delay
                      << " pred=" << predicted
                      << " resid=" << residual
                      << "\n";
        }

        if (ppp_config_.enable_outlier_detection) {
            const double sigma = std::sqrt(std::max(
                safeVariance(observation.variance_pr, 1e-6),
                (row * filter_state_.covariance * row.transpose())(0, 0)));
            const double code_residual_floor =
                precise_products_loaded_
                    ? (converged_ ? 500.0 : 50000.0)
                    : 20000.0;
            const double residual_limit =
                std::max(
                    ppp_config_.outlier_threshold * sigma * 10.0,
                    code_residual_floor);
            if (std::abs(residual) > residual_limit) {
                if (pppDebugEnabled()) {
                    std::cerr << "[PPP] reject code " << observation.satellite.toString()
                              << " pr_if=" << observation.pseudorange_if
                              << " pred=" << predicted
                              << " residual=" << residual
                              << " limit=" << residual_limit << "\n";
                }
                continue;
            }
        }

        rows.push_back(row);
        measured_values.push_back(observation.pseudorange_if);
        predicted_values.push_back(predicted);
        variances.push_back(safeVariance(observation.variance_pr, 1e-6));
        row_satellites.push_back(observation.satellite);
        row_is_phase.push_back(false);

        const bool qzss_code_only =
            env_overrides_.qzss_code_only ||
            (require_coherent_ssr_ && !env_overrides_.madoca_qzss_phase);
        // GLONASS FDMA phase rows need an inter-channel phase-bias treatment
        // that native does not yet model; code rows improve MADOCA bridge parity.
        const bool glonass_code_only =
            require_coherent_ssr_ && env_overrides_.madoca_glonass &&
            observation.satellite.system == GNSSSystem::GLONASS;
        const bool use_observation_phase =
            use_phase_rows &&
            observation.has_carrier_phase &&
            !(qzss_code_only && observation.satellite.system == GNSSSystem::QZSS) &&
            !glonass_code_only;
        if (use_observation_phase) {
            const int ambiguity_index = ambiguityStateIndex(observation.satellite);
            if (ambiguity_index >= 0 && ambiguity_index < filter_state_.total_states) {
                const auto ambiguity_it = ambiguity_states_.find(observation.satellite);
                // Phase observations carry the cm-class signal that drives
                // PPP convergence; gating them behind convergence_min_epochs
                // (= 20 = 10 min at 30 s) for the precise-products path lets
                // the static anchor blend pin the position state to the SPP
                // floor before phase ever enters the filter, capping the
                // achievable absolute residual. RTKLIB-style PPP enables phase
                // from the second epoch (lock_count >= 1) regardless of
                // product source, so use the same threshold here.
                const int required_lock_count =
                    ppp_config_.phase_measurement_min_lock_count;
                const bool phase_ready =
                    ambiguity_it != ambiguity_states_.end() &&
                    !ambiguity_it->second.needs_reinitialization &&
                    ambiguity_it->second.lock_count >= required_lock_count;
                if (!phase_ready) {
                    continue;
                }
                // predicted for phase: geo + clk - satclk + trop - iono + amb
                // (predicted already includes +iono, so subtract 2*iono for phase)
                double iono_phase_correction = 0.0;
                if (ppp_config_.estimate_ionosphere) {
                    const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
                    if (iono_it != filter_state_.ionosphere_indices.end()) {
                        iono_phase_correction = -2.0 * filter_state_.state(iono_it->second);
                    }
                }
                const double predicted_phase = predicted + iono_phase_correction
                                               + filter_state_.state(ambiguity_index);
                const double phase_residual = observation.carrier_phase_if - predicted_phase;
                if (env_overrides_.res_dump) dumpRes("L1", "phase", phase_residual);
                const double phase_residual_floor =
                    ppp_config_.kinematic_mode
                        ? (converged_ ? 20.0 : 200.0)
                        : (converged_ ? 10.0 : 50.0);
                const double phase_limit =
                    std::max(
                        ppp_config_.outlier_threshold *
                            std::sqrt(safeVariance(observation.variance_cp, 1e-8)) * 10.0,
                        phase_residual_floor);
                if (!ppp_config_.enable_outlier_detection ||
                    std::abs(phase_residual) <= phase_limit) {
                    Eigen::RowVectorXd phase_row = Eigen::RowVectorXd::Zero(filter_state_.total_states);
                    phase_row.segment(filter_state_.pos_index, 3) = -line_of_sight;
                    phase_row(clock_state_index) = 1.0;
                    phase_row(filter_state_.trop_index) =
                        trop_partial;
                    // Per-satellite ionosphere: carrier phase has -iono contribution
                    if (ppp_config_.estimate_ionosphere) {
                        const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
                        if (iono_it != filter_state_.ionosphere_indices.end()) {
                            phase_row(iono_it->second) = -1.0;
                        }
                    }
                    phase_row(ambiguity_index) = 1.0;

                    rows.push_back(phase_row);
                    measured_values.push_back(observation.carrier_phase_if);
                    predicted_values.push_back(predicted_phase);
                    variances.push_back(safeVariance(observation.variance_cp, 1e-8));
                    row_satellites.push_back(observation.satellite);
                    row_is_phase.push_back(true);
                }
            }
        }

        // --- Per-frequency (est-stec) L2 code + phase rows ---
        // The ionosphere state is in L1-equivalent meters; the L2 delay scales
        // by (f1/f2)^2. Code carries +iono, phase carries -iono (oracle ppp.c).
        if (!ppp_config_.use_ionosphere_free && ppp_config_.estimate_ionosphere &&
            observation.has_l2 && observation.freq_l1 > 0.0 && observation.freq_l2 > 0.0) {
            const double ratio = observation.freq_l1 / observation.freq_l2;
            const double ratio2 = ratio * ratio;
            const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
            const int iono_index =
                iono_it != filter_state_.ionosphere_indices.end() ? iono_it->second : -1;
            const double iono_state_l1_m =
                iono_index >= 0 ? filter_state_.state(iono_index) : 0.0;

            // L2 code row.
            {
                Eigen::RowVectorXd l2_code = Eigen::RowVectorXd::Zero(filter_state_.total_states);
                l2_code.segment(filter_state_.pos_index, 3) = -line_of_sight;
                l2_code(clock_state_index) = 1.0;
                l2_code(filter_state_.trop_index) =
                    trop_partial;
                if (iono_index >= 0) {
                    l2_code(iono_index) = ratio2;
                }
                const double l2_code_pred = geometric_range + clock_bias_m -
                    constants::SPEED_OF_LIGHT * observation.satellite_clock_bias +
                    troposphere_delay + ratio2 * iono_state_l1_m +
                    observation.rx_ant_corr_l2_m;
                const double l2_code_resid = observation.pseudorange_l2 - l2_code_pred;
                if (env_overrides_.res_dump) dumpRes("L2", "code", l2_code_resid);
                // Use the elevation-weighted RTKLIB variance (same satellite, so
                // identical elevation weighting as L1) rather than the flat seed.
                const double var_pr_l2 = observation.variance_pr > 0.0
                    ? observation.variance_pr
                    : safeVariance(observation.variance_pr_l2, 1e-6);
                const double code_floor = converged_ ? 500.0 : 20000.0;
                const double code_limit = std::max(
                    ppp_config_.outlier_threshold * std::sqrt(var_pr_l2) * 10.0,
                    code_floor);
                if (!ppp_config_.enable_outlier_detection ||
                    std::abs(l2_code_resid) <= code_limit) {
                    rows.push_back(l2_code);
                    measured_values.push_back(observation.pseudorange_l2);
                    predicted_values.push_back(l2_code_pred);
                    variances.push_back(var_pr_l2);
                    row_satellites.push_back(observation.satellite);
                    row_is_phase.push_back(false);
                }
            }

            // L2 phase row (requires a ready L2 ambiguity state).
            const auto amb2_it = filter_state_.ambiguity_l2_indices.find(observation.satellite);
            const auto amb_it = ambiguity_states_.find(observation.satellite);
            const bool l2_phase_ready =
                use_phase_rows && observation.has_carrier_phase_l2 &&
                amb2_it != filter_state_.ambiguity_l2_indices.end() &&
                amb_it != ambiguity_states_.end() &&
                !amb_it->second.needs_reinitialization &&
                amb_it->second.lock_count >= ppp_config_.phase_measurement_min_lock_count;
            if (l2_phase_ready) {
                const int amb2_index = amb2_it->second;
                Eigen::RowVectorXd l2_phase = Eigen::RowVectorXd::Zero(filter_state_.total_states);
                l2_phase.segment(filter_state_.pos_index, 3) = -line_of_sight;
                l2_phase(clock_state_index) = 1.0;
                l2_phase(filter_state_.trop_index) =
                    trop_partial;
                if (iono_index >= 0) {
                    l2_phase(iono_index) = -ratio2;
                }
                l2_phase(amb2_index) = 1.0;
                const double l2_phase_pred = geometric_range + clock_bias_m -
                    constants::SPEED_OF_LIGHT * observation.satellite_clock_bias +
                    troposphere_delay - ratio2 * iono_state_l1_m +
                    filter_state_.state(amb2_index) + observation.rx_ant_corr_l2_m;
                const double l2_phase_resid = observation.carrier_phase_l2 - l2_phase_pred;
                if (env_overrides_.res_dump) dumpRes("L2", "phase", l2_phase_resid);
                const double var_cp_l2 = observation.variance_cp > 0.0
                    ? observation.variance_cp
                    : safeVariance(observation.variance_cp_l2, 1e-8);
                const double phase_floor = converged_ ? 10.0 : 50.0;
                const double phase_limit = std::max(
                    ppp_config_.outlier_threshold * std::sqrt(var_cp_l2) * 10.0,
                    phase_floor);
                if (!ppp_config_.enable_outlier_detection ||
                    std::abs(l2_phase_resid) <= phase_limit) {
                    rows.push_back(l2_phase);
                    measured_values.push_back(observation.carrier_phase_l2);
                    predicted_values.push_back(l2_phase_pred);
                    variances.push_back(var_cp_l2);
                    row_satellites.push_back(observation.satellite);
                    row_is_phase.push_back(true);
                }
            }
        }
    }

    MeasurementEquation equation;
    const int n_obs = static_cast<int>(rows.size());
    equation.design_matrix = MatrixXd::Zero(n_obs, filter_state_.total_states);
    equation.observations = VectorXd::Zero(n_obs);
    equation.predicted = VectorXd::Zero(n_obs);
    equation.weight_matrix = MatrixXd::Zero(n_obs, n_obs);
    equation.residuals = VectorXd::Zero(n_obs);

    for (int i = 0; i < n_obs; ++i) {
        equation.design_matrix.row(i) = rows[static_cast<size_t>(i)];
        equation.observations(i) = measured_values[static_cast<size_t>(i)];
        equation.predicted(i) = predicted_values[static_cast<size_t>(i)];
        equation.weight_matrix(i, i) = variances[static_cast<size_t>(i)];
        equation.residuals(i) = measured_values[static_cast<size_t>(i)] -
                                predicted_values[static_cast<size_t>(i)];
    }
    equation.row_satellites = row_satellites;
    equation.row_is_phase = row_is_phase;
    return equation;
}

void PPPProcessor::checkConvergence(const GNSSTime& current_time) {
    if (converged_) {
        return;
    }

    recent_positions_.push_back(filter_state_.state.segment(filter_state_.pos_index, 3));
    if (recent_positions_.size() > static_cast<size_t>(ppp_config_.convergence_min_epochs)) {
        recent_positions_.erase(recent_positions_.begin());
    }

    if (recent_positions_.size() < static_cast<size_t>(ppp_config_.convergence_min_epochs)) {
        return;
    }

    Vector3d mean = Vector3d::Zero();
    for (const auto& position : recent_positions_) {
        mean += position;
    }
    mean /= static_cast<double>(recent_positions_.size());

    double max_deviation = 0.0;
    for (const auto& position : recent_positions_) {
        max_deviation = std::max(max_deviation, (position - mean).norm());
    }

    if (max_deviation < ppp_config_.convergence_threshold_horizontal) {
        converged_ = true;
        convergence_time_ = current_time - convergence_start_time_;
    }
}

PositionSolution PPPProcessor::generateSolution(const GNSSTime& time,
                                                const std::vector<IonosphereFreeObs>& observations) {
    PositionSolution solution;
    solution.time = time;
    solution.status = SolutionStatus::PPP_FLOAT;
    solution.num_satellites = 0;
    solution.num_frequencies = ppp_config_.use_ionosphere_free ? 2 : 1;

    if (!filter_initialized_) {
        return solution;
    }

    solution.position_ecef = filter_state_.state.segment(filter_state_.pos_index, 3);
    solution.receiver_clock_bias =
        filter_state_.state(filter_state_.clock_index) / constants::SPEED_OF_LIGHT;
    solution.position_covariance =
        filter_state_.covariance.block(filter_state_.pos_index, filter_state_.pos_index, 3, 3);

    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    ecef2geodetic(solution.position_ecef, latitude, longitude, height);
    solution.position_geodetic = GeodeticCoord(latitude, longitude, height);

    for (const auto& observation : observations) {
        if (!observation.valid) {
            continue;
        }
        solution.num_satellites++;
        solution.satellites_used.push_back(observation.satellite);
        solution.satellite_elevations.push_back(observation.elevation);
        const double geometric_range = geodist(observation.satellite_position, solution.position_ecef);
        const double clock_bias_m = receiverClockBiasMeters(observation.satellite);
        const double predicted = geometric_range +
            clock_bias_m -
            constants::SPEED_OF_LIGHT * observation.satellite_clock_bias +
            (ppp_config_.estimate_troposphere
                ? observation.trop_mapping * filter_state_.state(filter_state_.trop_index)
                : observation.modeled_trop_delay_m);
        solution.satellite_residuals.push_back(observation.pseudorange_if - predicted);
    }

    if (!solution.satellite_residuals.empty()) {
        double sum_sq = 0.0;
        for (const double residual : solution.satellite_residuals) {
            sum_sq += residual * residual;
        }
        solution.residual_rms =
            std::sqrt(sum_sq / static_cast<double>(solution.satellite_residuals.size()));
    }

    return solution;
}

Vector3d PPPProcessor::calculatePositionAccuracy() const {
    if (!filter_initialized_) {
        return Vector3d::Constant(std::numeric_limits<double>::infinity());
    }
    return Vector3d(
        std::sqrt(std::max(0.0, filter_state_.covariance(filter_state_.pos_index + 0, filter_state_.pos_index + 0))),
        std::sqrt(std::max(0.0, filter_state_.covariance(filter_state_.pos_index + 1, filter_state_.pos_index + 1))),
        std::sqrt(std::max(0.0, filter_state_.covariance(filter_state_.pos_index + 2, filter_state_.pos_index + 2))));
}

VectorXd PPPProcessor::calculateResiduals(const std::vector<IonosphereFreeObs>& observations,
                                          const NavigationData& nav,
                                          const GNSSTime& time) const {
    (void)nav;
    (void)time;
    std::vector<double> residuals;
    residuals.reserve(observations.size());

    const Vector3d receiver_position =
        filter_state_.state.segment(filter_state_.pos_index, 3);
    const double zenith_delay = filter_state_.state(filter_state_.trop_index);

    for (const auto& observation : observations) {
        if (!observation.valid) {
            continue;
        }
        const double clock_bias_m = receiverClockBiasMeters(observation.satellite);
        const double predicted = geodist(observation.satellite_position, receiver_position) +
            clock_bias_m -
            constants::SPEED_OF_LIGHT * observation.satellite_clock_bias +
            (ppp_config_.estimate_troposphere ?
                observation.trop_mapping * zenith_delay :
                observation.modeled_trop_delay_m);
        residuals.push_back(observation.pseudorange_if - predicted);
    }

    VectorXd residual_vector = VectorXd::Zero(static_cast<int>(residuals.size()));
    for (size_t i = 0; i < residuals.size(); ++i) {
        residual_vector(static_cast<int>(i)) = residuals[i];
    }
    return residual_vector;
}

}  // namespace libgnss
