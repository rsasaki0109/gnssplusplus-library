#include <libgnss++/algorithms/ppp_clas.hpp>

#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <vector>

namespace libgnss::ppp_clas {

namespace {

constexpr std::array<double, 100> kClaslibChiSquare001 = {
    10.8,13.8,16.3,18.5,20.5,22.5,24.3,26.1,27.9,29.6,
    31.3,32.9,34.5,36.1,37.7,39.3,40.8,42.3,43.8,45.3,
    46.8,48.3,49.7,51.2,52.6,54.1,55.5,56.9,58.3,59.7,
    61.1,62.5,63.9,65.2,66.6,68.0,69.3,70.7,72.1,73.4,
    74.7,76.0,77.3,78.6,80.0,81.3,82.6,84.0,85.4,86.7,
    88.0,89.3,90.6,91.9,93.3,94.7,96.0,97.4,98.7,100.0,
    101.0,102.0,103.0,104.0,105.0,107.0,108.0,109.0,110.0,112.0,
    113.0,114.0,115.0,116.0,118.0,119.0,120.0,122.0,123.0,125.0,
    126.0,127.0,128.0,129.0,131.0,132.0,133.0,134.0,135.0,137.0,
    138.0,139.0,140.0,142.0,143.0,144.0,145.0,147.0,148.0,149.0
};

double claslibChiSquare001ForDof(int dof) {
    if (dof <= 0) {
        return 0.0;
    }
    if (dof <= static_cast<int>(kClaslibChiSquare001.size())) {
        return kClaslibChiSquare001[static_cast<size_t>(dof - 1)];
    }
    const double tail = std::max(0, dof - static_cast<int>(kClaslibChiSquare001.size()));
    return kClaslibChiSquare001.back() + 1.1 * tail;
}

struct PhaseResidualInfo {
    SatelliteId ambiguity_satellite;
    SatelliteId real_satellite;
    double residual_m = 0.0;
    double variance_m2 = 0.0;
    double frequency_hz = 0.0;
    double wavelength_m = 0.0;
};

struct PhasePairInfo {
    std::array<double, 2> residual_m{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    std::array<double, 2> variance_m2{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    std::array<double, 2> frequency_hz{0.0, 0.0};
    std::array<double, 2> wavelength_m{0.0, 0.0};
};

}  // namespace

std::vector<SatelliteId> collectResidualIonoSatellites(
    const ObservationData& obs,
    const SSRProducts& ssr_products) {
    auto supports_clas_residual_iono = [](const SatelliteId& satellite) {
        return satellite.system == GNSSSystem::GPS ||
               satellite.system == GNSSSystem::Galileo ||
               satellite.system == GNSSSystem::QZSS;
    };

    std::set<SatelliteId> unique_satellites;
    for (const auto& satellite : obs.getSatellites()) {
        if (supports_clas_residual_iono(satellite)) {
            unique_satellites.insert(satellite);
        }
    }
    for (const auto& [satellite, _] : ssr_products.orbit_clock_corrections) {
        if (supports_clas_residual_iono(satellite)) {
            unique_satellites.insert(satellite);
        }
    }
    return std::vector<SatelliteId>(unique_satellites.begin(), unique_satellites.end());
}

EpochPreparationResult prepareEpochState(
    const ObservationData& obs,
    const PositionSolution& seed_solution,
    const SSRProducts& ssr_products,
    ppp_shared::PPPState& filter_state,
    bool& filter_initialized,
    GNSSTime& convergence_start_time,
    Vector3d& static_anchor_position,
    bool& has_static_anchor_position,
    const ppp_shared::PPPConfig& config,
    double modeled_zenith_troposphere_delay_m,
    bool has_last_processed_time,
    const GNSSTime& last_processed_time,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    double ambiguity_reset_variance) {
    EpochPreparationResult result;
    if (!filter_initialized) {
        if (!seed_solution.isValid()) {
            return result;
        }
        const auto iono_satellites =
            config.estimate_ionosphere
                ? collectResidualIonoSatellites(obs, ssr_products)
                : std::vector<SatelliteId>{};
        initializeFilterState(
            filter_state,
            seed_solution,
            obs.time,
            iono_satellites,
            config,
            modeled_zenith_troposphere_delay_m);
        filter_initialized = true;
        convergence_start_time = obs.time;
        static_anchor_position = seed_solution.position_ecef;
        has_static_anchor_position = true;
    }

    const double dt =
        has_last_processed_time ? std::max(obs.time - last_processed_time, 0.001) : 1.0;
    syncSlipState(
        obs,
        filter_state,
        ambiguity_states,
        dispersion_compensation,
        phase_bias_repair,
        ambiguity_reset_variance);
    predictFilterState(
        filter_state,
        config,
        dt,
        seed_solution.receiver_clock_bias,
        seed_solution.isValid());
    markSlipCompensationFromAmbiguities(
        obs, ambiguity_states, dispersion_compensation);
    result.ready = true;
    return result;
}

void initializeFilterState(
    ppp_shared::PPPState& filter_state,
    const PositionSolution& seed_solution,
    const GNSSTime& /*time*/,
    const std::vector<SatelliteId>& iono_satellites,
    const ppp_shared::PPPConfig& config,
    double modeled_zenith_troposphere_delay_m) {
    const int base = 9 + static_cast<int>(iono_satellites.size());
    filter_state.ionosphere_indices.clear();
    filter_state.state = VectorXd::Zero(base);
    filter_state.covariance = MatrixXd::Identity(base, base);
    filter_state.state.segment(0, 3) = seed_solution.position_ecef;
    filter_state.state(filter_state.clock_index) = seed_solution.receiver_clock_bias;
    filter_state.state(filter_state.glo_clock_index) = seed_solution.receiver_clock_bias;
    filter_state.state(filter_state.trop_index) = modeled_zenith_troposphere_delay_m;
    filter_state.covariance.block(0, 0, 3, 3) *= 100.0;
    filter_state.covariance(6, 6) = 1e8;
    filter_state.covariance(7, 7) = 1e8;
    filter_state.covariance(8, 8) = 0.25;
    filter_state.iono_index = 9;
    for (size_t index = 0; index < iono_satellites.size(); ++index) {
        const int state_index = filter_state.iono_index + static_cast<int>(index);
        filter_state.ionosphere_indices[iono_satellites[index]] = state_index;
        filter_state.state(state_index) = 0.0;
        filter_state.covariance(state_index, state_index) =
            std::min(config.initial_ionosphere_variance, 1.0);
    }
    filter_state.amb_index = base;
    filter_state.total_states = base;
}

void syncSlipState(
    const ObservationData& obs,
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    double ambiguity_reset_variance) {
    for (const auto& satellite : obs.getSatellites()) {
        const auto slip_it = ambiguity_states.find(satellite);
        if (slip_it == ambiguity_states.end() || !slip_it->second.needs_reinitialization) {
            continue;
        }

        const SatelliteId l2_satellite(
            satellite.system,
            static_cast<uint8_t>(std::min(255, satellite.prn + 100)));
        auto reset_ambiguity = [&](const SatelliteId& ambiguity_satellite) {
            auto& ambiguity = ambiguity_states[ambiguity_satellite];
            ambiguity = ppp_shared::PPPAmbiguityInfo{};
            ambiguity.needs_reinitialization = true;

            const auto ambiguity_index_it =
                filter_state.ambiguity_indices.find(ambiguity_satellite);
            if (ambiguity_index_it == filter_state.ambiguity_indices.end()) {
                return;
            }
            const int ambiguity_index = ambiguity_index_it->second;
            if (ambiguity_index < 0 || ambiguity_index >= filter_state.total_states) {
                return;
            }
            filter_state.state(ambiguity_index) = 0.0;
            filter_state.covariance.row(ambiguity_index).setZero();
            filter_state.covariance.col(ambiguity_index).setZero();
            filter_state.covariance(ambiguity_index, ambiguity_index) =
                ambiguity_reset_variance;
        };

        if (filter_state.ambiguity_indices.find(l2_satellite) !=
            filter_state.ambiguity_indices.end()) {
            reset_ambiguity(l2_satellite);
        }

        dispersion_compensation[satellite].slip = {true, true};
        auto repair_it = phase_bias_repair.find(satellite);
        if (repair_it != phase_bias_repair.end()) {
            repair_it->second.reference_time = GNSSTime();
            repair_it->second.last_continuity_m = {0.0, 0.0, 0.0};
            repair_it->second.offset_cycles = {0.0, 0.0, 0.0};
            repair_it->second.pending_state_shift_cycles = {0.0, 0.0, 0.0};
            repair_it->second.has_last = {false, false, false};
        }
    }
}

void predictFilterState(
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    double dt,
    double seed_receiver_clock_bias_m,
    bool seed_valid) {
    const int nx = filter_state.total_states;
    MatrixXd Q = MatrixXd::Zero(nx, nx);
    Q(filter_state.clock_index, filter_state.clock_index) = 1e8;
    Q(filter_state.glo_clock_index, filter_state.glo_clock_index) = 1e8;
    Q(filter_state.trop_index, filter_state.trop_index) = 1e-8 * dt;
    if (config.estimate_ionosphere) {
        for (const auto& [satellite, state_index] : filter_state.ionosphere_indices) {
            (void)satellite;
            if (state_index >= 0 && state_index < nx) {
                Q(state_index, state_index) = config.process_noise_ionosphere * dt;
            }
        }
    }
    for (const auto& [satellite, state_index] : filter_state.ambiguity_indices) {
        (void)satellite;
        if (state_index >= 0 && state_index < nx) {
            Q(state_index, state_index) = 1e-8 * dt;
        }
    }
    filter_state.covariance += Q;
    if (seed_valid) {
        filter_state.state(filter_state.clock_index) = seed_receiver_clock_bias_m;
        filter_state.state(filter_state.glo_clock_index) = seed_receiver_clock_bias_m;
    }
}

void markSlipCompensationFromAmbiguities(
    const ObservationData& obs,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation) {
    for (const auto& satellite : obs.getSatellites()) {
        auto& compensation = dispersion_compensation[satellite];
        const auto l1_ambiguity_it = ambiguity_states.find(satellite);
        if (l1_ambiguity_it != ambiguity_states.end() &&
            l1_ambiguity_it->second.needs_reinitialization) {
            compensation.slip[0] = true;
        }
        const SatelliteId l2_satellite(
            satellite.system,
            static_cast<uint8_t>(std::min(255, satellite.prn + 100)));
        const auto l2_ambiguity_it = ambiguity_states.find(l2_satellite);
        if (l2_ambiguity_it != ambiguity_states.end() &&
            l2_ambiguity_it->second.needs_reinitialization) {
            compensation.slip[1] = true;
        }
    }
}

void ensureAmbiguityStates(
    ppp_shared::PPPState& filter_state,
    const std::vector<OSRCorrection>& osr_corrections,
    double initial_variance) {
    auto allocate_ambiguity = [&](const SatelliteId& ambiguity_satellite) {
        if (filter_state.ambiguity_indices.find(ambiguity_satellite) !=
            filter_state.ambiguity_indices.end()) {
            return;
        }
        const int new_index = filter_state.total_states;
        filter_state.ambiguity_indices[ambiguity_satellite] = new_index;
        filter_state.total_states++;

        VectorXd new_state = VectorXd::Zero(filter_state.total_states);
        new_state.head(new_index) = filter_state.state;
        filter_state.state = new_state;

        MatrixXd new_covariance =
            MatrixXd::Zero(filter_state.total_states, filter_state.total_states);
        new_covariance.topLeftCorner(new_index, new_index) = filter_state.covariance;
        new_covariance(new_index, new_index) = initial_variance;
        filter_state.covariance = new_covariance;
    };

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        allocate_ambiguity(osr.satellite);
        if (osr.num_frequencies >= 2) {
            const uint8_t l2_prn =
                static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
            allocate_ambiguity(SatelliteId(osr.satellite.system, l2_prn));
        }
    }
}

void applyPendingPhaseBiasStateShifts(
    ppp_shared::PPPState& filter_state,
    const std::vector<OSRCorrection>& osr_corrections,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    bool debug_enabled) {
    for (const auto& osr : osr_corrections) {
        auto repair_it = phase_bias_repair.find(osr.satellite);
        if (repair_it == phase_bias_repair.end()) {
            continue;
        }
        for (int f = 0; f < osr.num_frequencies; ++f) {
            const double shift_cycles =
                repair_it->second.pending_state_shift_cycles[static_cast<size_t>(f)];
            if (shift_cycles == 0.0 || osr.wavelengths[f] <= 0.0) {
                continue;
            }
            const uint8_t ambiguity_prn = f == 0 ? osr.satellite.prn :
                static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
            const SatelliteId ambiguity_satellite(osr.satellite.system, ambiguity_prn);
            const auto ambiguity_it =
                filter_state.ambiguity_indices.find(ambiguity_satellite);
            if (ambiguity_it == filter_state.ambiguity_indices.end()) {
                continue;
            }
            filter_state.state(ambiguity_it->second) += shift_cycles * osr.wavelengths[f];
            repair_it->second.pending_state_shift_cycles[static_cast<size_t>(f)] = 0.0;
            if (debug_enabled) {
                std::cerr << "[CLAS-PBIAS] state shift "
                          << ambiguity_satellite.toString()
                          << " f=" << f
                          << " cycles=" << shift_cycles
                          << " meters=" << shift_cycles * osr.wavelengths[f]
                          << "\n";
            }
        }
    }
}

MeasurementBuildResult buildEpochMeasurements(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const Vector3d& receiver_position,
    double receiver_clock_m,
    double trop_zenith,
    const std::map<std::string, std::string>& epoch_atmos,
    const TropMappingFunction& trop_mapping_function,
    const AmbiguityResetFunction& ambiguity_reset_function,
    bool debug_enabled) {
    MeasurementBuildResult result;

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }

        const double geo = geodist(osr.satellite_position, receiver_position);
        const Vector3d los =
            (osr.satellite_position - receiver_position).normalized();
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double trop_mapping =
            trop_mapping_function(receiver_position, osr.elevation, obs.time);
        const double trop_modeled = trop_mapping * trop_zenith;

        if (debug_enabled && osr.num_frequencies >= 2 &&
            result.measurements.size() < 10) {
            const Observation* l1_raw = obs.getObservation(osr.satellite, osr.signals[0]);
            const Observation* l2_raw = obs.getObservation(osr.satellite, osr.signals[1]);
            if (l1_raw && l2_raw && l1_raw->has_pseudorange && l2_raw->has_pseudorange) {
                const double f1 = osr.frequencies[0];
                const double f2 = osr.frequencies[1];
                const double c1 = f1 * f1 / (f1 * f1 - f2 * f2);
                const double c2 = -f2 * f2 / (f1 * f1 - f2 * f2);
                const double p_if = c1 * l1_raw->pseudorange + c2 * l2_raw->pseudorange;
                const double cb_if = c1 * osr.code_bias_m[0] + c2 * osr.code_bias_m[1];
                const double p_if_corr = p_if - cb_if - osr.relativity_correction_m;
                const double predicted =
                    geo - sat_clk_m + receiver_clock_m + trop_modeled;
                const double res_if = p_if_corr - predicted;
                const double prc_no_trop = osr.PRC[0] - osr.trop_correction_m;
                const double p1_corr = l1_raw->pseudorange - prc_no_trop;
                const double res_l1 = p1_corr - predicted;
                std::cerr << "[CLAS-CMP] " << osr.satellite.toString()
                          << " res_IF=" << res_if
                          << " res_L1=" << res_l1
                          << " iono_l1=" << osr.iono_l1_m
                          << " diff=" << res_l1 - res_if << "\n";
            }
        }

        std::array<const Observation*, OSR_MAX_FREQ> raw_observations{};
        const auto iono_state_it = filter_state.ionosphere_indices.find(osr.satellite);
        const int iono_state_index =
            iono_state_it != filter_state.ionosphere_indices.end() ?
                iono_state_it->second : -1;
        const bool use_residual_iono_state =
            config.estimate_ionosphere &&
            iono_state_index >= 0 &&
            iono_state_index < filter_state.total_states;
        for (int f = 0; f < osr.num_frequencies; ++f) {
            raw_observations[static_cast<size_t>(f)] =
                obs.getObservation(osr.satellite, osr.signals[f]);
        }

        for (int f = 0; f < osr.num_frequencies; ++f) {
            const Observation* raw = raw_observations[static_cast<size_t>(f)];
            if (!raw || !raw->valid) {
                continue;
            }
            const double iono_scale =
                (osr.frequencies[f] > 0.0 && osr.wavelengths[0] > 0.0)
                    ? std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2)
                    : 1.0;
            const double iono_state_m =
                use_residual_iono_state ? filter_state.state(iono_state_index) : 0.0;

            const double prc_no_trop = osr.PRC[f] - osr.trop_correction_m;
            const double cpc_no_trop = osr.CPC[f] - osr.trop_correction_m;

            if (raw->has_pseudorange && std::isfinite(raw->pseudorange)) {
                const double p_corr = raw->pseudorange - prc_no_trop;
                const double predicted =
                    geo - sat_clk_m + receiver_clock_m + trop_modeled +
                    iono_scale * iono_state_m;
                const double residual = p_corr - predicted;

                if (debug_enabled && f == 0 && result.measurements.size() < 5) {
                    std::cerr << "[CLAS-RES] " << osr.satellite.toString()
                              << " P_corr=" << p_corr - geo + sat_clk_m
                              << " trop=" << trop_modeled
                              << " iono=" << iono_scale * iono_state_m
                              << " rcv_clk=" << receiver_clock_m
                              << " res=" << residual << "\n";
                }

                const double el_weight =
                    1.0 / (std::sin(osr.elevation) * std::sin(osr.elevation));

                MeasurementRow row;
                row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
                row.H.segment(0, 3) = -los.transpose();
                row.H(filter_state.clock_index) = 1.0;
                row.H(filter_state.trop_index) = trop_mapping;
                if (use_residual_iono_state) {
                    row.H(iono_state_index) = iono_scale;
                    result.observed_iono_states.insert(osr.satellite);
                }
                row.residual = residual;
                row.variance = 3.0 * el_weight;
                row.satellite = osr.satellite;
                row.is_phase = false;
                row.freq_index = f;
                result.measurements.push_back(row);
            }

            if (raw->has_carrier_phase && std::isfinite(raw->carrier_phase)) {
                const double l_m = raw->carrier_phase * osr.wavelengths[f];
                const double l_corr = l_m - cpc_no_trop;

                const uint8_t amb_prn = f == 0 ? osr.satellite.prn
                    : static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
                const SatelliteId amb_sat(osr.satellite.system, amb_prn);
                const auto amb_it = filter_state.ambiguity_indices.find(amb_sat);
                if (amb_it == filter_state.ambiguity_indices.end()) {
                    continue;
                }
                const int amb_idx = amb_it->second;

                if (raw->loss_of_lock && ambiguity_reset_function) {
                    ambiguity_reset_function(amb_sat, raw->signal);
                }

                if (filter_state.covariance(amb_idx, amb_idx) >= 3000.0) {
                    filter_state.state(amb_idx) =
                        l_corr - (geo - sat_clk_m + receiver_clock_m + trop_modeled
                                  - iono_scale * iono_state_m);
                }

                const double predicted =
                    geo - sat_clk_m + receiver_clock_m + trop_modeled
                    - iono_scale * iono_state_m + filter_state.state(amb_idx);
                const double residual = l_corr - predicted;

                const double el_weight =
                    1.0 / (std::sin(osr.elevation) * std::sin(osr.elevation));

                MeasurementRow row;
                row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
                row.H.segment(0, 3) = -los.transpose();
                row.H(filter_state.clock_index) = 1.0;
                row.H(filter_state.trop_index) = trop_mapping;
                if (use_residual_iono_state) {
                    row.H(iono_state_index) = -iono_scale;
                    result.observed_iono_states.insert(osr.satellite);
                }
                row.H(amb_idx) = 1.0;
                row.residual = residual;
                row.variance = 0.01 * el_weight;
                row.satellite = osr.satellite;
                row.is_phase = true;
                row.freq_index = f;
                result.measurements.push_back(row);
                result.observed_ambiguities.push_back(
                    {amb_sat, raw->signal, osr.wavelengths[f], raw->carrier_phase, raw->snr});
            }
        }
    }

    if (config.estimate_troposphere && !epoch_atmos.empty()) {
        const double clas_trop_zenith =
            ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
                epoch_atmos, receiver_position, obs.time, M_PI_2);
        if (std::isfinite(clas_trop_zenith) && clas_trop_zenith > 0.0) {
            MeasurementRow row;
            row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
            row.H(filter_state.trop_index) = 1.0;
            row.residual = clas_trop_zenith - trop_zenith;
            row.variance = 0.25;
            row.satellite = SatelliteId{};
            row.is_phase = false;
            row.freq_index = -1;
            result.measurements.push_back(row);
            if (debug_enabled) {
                std::cerr << "[CLAS-TROP] prior=" << clas_trop_zenith
                          << " state=" << trop_zenith << "\n";
            }
        }
    }

    if (config.estimate_ionosphere) {
        for (const auto& satellite : result.observed_iono_states) {
            const auto iono_it = filter_state.ionosphere_indices.find(satellite);
            if (iono_it == filter_state.ionosphere_indices.end()) {
                continue;
            }
            const int iono_state_index = iono_it->second;
            if (iono_state_index < 0 || iono_state_index >= filter_state.total_states) {
                continue;
            }
            MeasurementRow row;
            row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
            row.H(iono_state_index) = 1.0;
            row.residual = -filter_state.state(iono_state_index);
            row.variance = 0.25;
            row.satellite = satellite;
            row.is_phase = false;
            row.freq_index = -1;
            result.measurements.push_back(row);
        }
    }

    return result;
}

KalmanUpdateStats applyMeasurementUpdate(
    ppp_shared::PPPState& filter_state,
    const std::vector<MeasurementRow>& measurements,
    const PositionSolution* seed_solution) {
    KalmanUpdateStats stats;
    stats.nobs = static_cast<int>(measurements.size());
    if (stats.nobs < 4) {
        return stats;
    }

    MatrixXd H = MatrixXd::Zero(stats.nobs, filter_state.total_states);
    VectorXd z = VectorXd::Zero(stats.nobs);
    MatrixXd R = MatrixXd::Zero(stats.nobs, stats.nobs);

    for (int i = 0; i < stats.nobs; ++i) {
        H.row(i) = measurements[static_cast<size_t>(i)].H;
        z(i) = measurements[static_cast<size_t>(i)].residual;
        R(i, i) = measurements[static_cast<size_t>(i)].variance;
    }

    for (int i = 0; i < stats.nobs; ++i) {
        const double sigma = std::sqrt(R(i, i));
        if (std::abs(z(i)) > 50.0 * sigma) {
            R(i, i) = 1e10;
        }
    }

    MatrixXd S = H * filter_state.covariance * H.transpose() + R;
    MatrixXd K = filter_state.covariance * H.transpose() * S.inverse();
    VectorXd dx = K * z;
    filter_state.state += dx;
    MatrixXd I_KH =
        MatrixXd::Identity(filter_state.total_states, filter_state.total_states) - K * H;
    filter_state.covariance =
        I_KH * filter_state.covariance * I_KH.transpose() + K * R * K.transpose();

    stats.updated = true;
    stats.dx = dx;
    stats.residuals = z;
    stats.variances = R.diagonal();
    stats.pre_anchor_covariance = filter_state.covariance;

    if (seed_solution != nullptr && seed_solution->isValid()) {
        const double anchor_sigma = 5.0;
        for (int axis = 0; axis < 3; ++axis) {
            const int idx = axis;
            const double innovation =
                seed_solution->position_ecef(axis) - filter_state.state(idx);
            const double innovation_covariance =
                filter_state.covariance(idx, idx) + anchor_sigma * anchor_sigma;
            if (innovation_covariance > 0.0) {
                VectorXd K_col = filter_state.covariance.col(idx) / innovation_covariance;
                filter_state.state += K_col * innovation;
                filter_state.covariance -= K_col * filter_state.covariance.row(idx);
            }
        }
    }

    return stats;
}

EpochUpdateResult runEpochMeasurementUpdate(
    const ObservationData& obs,
    const CLASEpochContext& epoch_context,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const PositionSolution& seed_solution,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const TropMappingFunction& trop_mapping_function,
    const AmbiguityResetFunction& ambiguity_reset_function,
    const AmbiguityIndexFunction& ambiguity_index_function,
    bool debug_enabled) {
    EpochUpdateResult result;
    auto measurement_build_result = buildEpochMeasurements(
        obs,
        epoch_context.osr_corrections,
        filter_state,
        config,
        epoch_context.receiver_position,
        epoch_context.receiver_clock_m,
        epoch_context.trop_zenith_m,
        epoch_context.epoch_atmos_tokens,
        trop_mapping_function,
        ambiguity_reset_function,
        debug_enabled);

    if (measurement_build_result.measurements.size() < 5) {
        return result;
    }

    result.update_stats = applyMeasurementUpdate(
        filter_state, measurement_build_result.measurements, &seed_solution);
    if (!result.update_stats.updated) {
        return result;
    }

    updateObservedAmbiguities(
        obs.time,
        measurement_build_result.observed_ambiguities,
        filter_state,
        ambiguity_states,
        ambiguity_index_function);
    result.updated = true;
    return result;
}

void updateObservedAmbiguities(
    const GNSSTime& time,
    const std::vector<AmbiguityObservation>& observed_ambiguities,
    const ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const AmbiguityIndexFunction& ambiguity_index_function) {
    for (const auto& ambiguity_obs : observed_ambiguities) {
        auto& ambiguity = ambiguity_states[ambiguity_obs.ambiguity_satellite];
        ambiguity.last_phase = ambiguity_obs.carrier_phase_cycles;
        ambiguity.last_time = time;
        ambiguity.lock_count += 1;
        ambiguity.quality_indicator = ambiguity_obs.snr;
        ambiguity.ambiguity_scale_m = ambiguity_obs.wavelength_m;
        ambiguity.needs_reinitialization = false;
        const int ambiguity_index =
            ambiguity_index_function(ambiguity_obs.ambiguity_satellite);
        if (ambiguity_index >= 0 && ambiguity_index < filter_state.total_states) {
            ambiguity.float_value = filter_state.state(ambiguity_index);
        }
    }
}

FixValidationStats validateFixedSolution(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function,
    const AmbiguityIndexFunction& ambiguity_index_function,
    bool debug_enabled) {
    FixValidationStats stats;
    double phase_sum_sq = 0.0;
    double code_sum_sq = 0.0;
    double phase_chi_sq = 0.0;
    bool pair_validation_ok = true;
    SatelliteId worst_reference_satellite;
    SatelliteId worst_satellite;
    int worst_freq_group = -1;
    double worst_dd_residual_m = 0.0;
    double worst_dd_sigma = 0.0;
    SatelliteId worst_pair_satellite;
    double worst_pair_dispersive = 0.0;
    double worst_pair_nondispersive = 0.0;
    double worst_pair_sigma = 0.0;

    std::vector<PhaseResidualInfo> phase_residuals;

    const Vector3d receiver_position =
        filter_state.state.segment(filter_state.pos_index, 3);
    const double receiver_clock_m = filter_state.state(filter_state.clock_index);
    const double trop_zenith = filter_state.state(filter_state.trop_index);
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }

        const double geo = geodist(osr.satellite_position, receiver_position);
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double trop_mapping =
            trop_mapping_function(receiver_position, osr.elevation, obs.time);
        const double trop_modeled = trop_mapping * trop_zenith;

        std::array<const Observation*, OSR_MAX_FREQ> raw_observations{};
        for (int f = 0; f < osr.num_frequencies; ++f) {
            raw_observations[static_cast<size_t>(f)] =
                obs.getObservation(osr.satellite, osr.signals[f]);
        }

        for (int f = 0; f < osr.num_frequencies; ++f) {
            const Observation* raw = raw_observations[static_cast<size_t>(f)];
            if (raw == nullptr || !raw->valid) {
                continue;
            }

            const auto iono_it = filter_state.ionosphere_indices.find(osr.satellite);
            const double iono_scale =
                (config.estimate_ionosphere &&
                 iono_it != filter_state.ionosphere_indices.end() &&
                 iono_it->second >= 0 &&
                 iono_it->second < filter_state.total_states &&
                 osr.frequencies[f] > 0.0 &&
                 osr.wavelengths[0] > 0.0)
                    ? std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2)
                    : 0.0;
            const double iono_state_m =
                iono_scale > 0.0 ? filter_state.state(iono_it->second) : 0.0;

            const double el_weight =
                1.0 / (std::sin(osr.elevation) * std::sin(osr.elevation));
            const double predicted =
                geo - sat_clk_m + receiver_clock_m + trop_modeled;
            const double prc_no_trop = osr.PRC[f] - osr.trop_correction_m;
            const double cpc_no_trop = osr.CPC[f] - osr.trop_correction_m;

            if (raw->has_pseudorange && std::isfinite(raw->pseudorange)) {
                const double residual =
                    (raw->pseudorange - prc_no_trop) -
                    (predicted + iono_scale * iono_state_m);
                code_sum_sq += residual * residual;
                ++stats.code_rows;
            }

            if (!raw->has_carrier_phase || !std::isfinite(raw->carrier_phase)) {
                continue;
            }

            const uint8_t ambiguity_prn = f == 0 ? osr.satellite.prn :
                static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
            const SatelliteId ambiguity_satellite(osr.satellite.system, ambiguity_prn);
            const int ambiguity_index = ambiguity_index_function(ambiguity_satellite);
            if (ambiguity_index < 0 || ambiguity_index >= filter_state.total_states) {
                continue;
            }

            const double carrier_phase_m = raw->carrier_phase * osr.wavelengths[f];
            const double residual =
                (carrier_phase_m - cpc_no_trop) -
                (predicted - iono_scale * iono_state_m +
                 filter_state.state(ambiguity_index));
            const double variance = 0.01 * el_weight;
            phase_residuals.push_back({
                ambiguity_satellite,
                osr.satellite,
                residual,
                variance,
                osr.frequencies[f],
                osr.wavelengths[f]});
        }
    }

    std::map<std::pair<GNSSSystem, int>, std::vector<PhaseResidualInfo>> dd_groups;
    for (const auto& phase_residual : phase_residuals) {
        dd_groups[ppp_ar::ambiguityDdGroup(phase_residual.ambiguity_satellite)]
            .push_back(phase_residual);
    }

    std::map<SatelliteId, PhasePairInfo> phase_pairs;
    for (auto& [group, residuals] : dd_groups) {
        std::sort(residuals.begin(), residuals.end(),
                  [](const PhaseResidualInfo& lhs, const PhaseResidualInfo& rhs) {
                      if (lhs.real_satellite.system != rhs.real_satellite.system) {
                          return static_cast<int>(lhs.real_satellite.system) <
                                 static_cast<int>(rhs.real_satellite.system);
                      }
                      return lhs.real_satellite.prn < rhs.real_satellite.prn;
                  });
        if (residuals.size() < 2) {
            continue;
        }

        const auto& reference = residuals.front();
        const size_t freq_slot = static_cast<size_t>(group.second);
        for (size_t index = 1; index < residuals.size(); ++index) {
            const auto& residual = residuals[index];
            const double dd_residual = reference.residual_m - residual.residual_m;
            const double dd_variance = reference.variance_m2 + residual.variance_m2;
            const double sigma = std::sqrt(std::max(dd_variance, 1e-12));

            phase_sum_sq += dd_residual * dd_residual;
            phase_chi_sq += dd_residual * dd_residual /
                            std::max(dd_variance, 1e-12);
            ++stats.phase_rows;
            stats.max_phase_sigma =
                std::max(stats.max_phase_sigma, std::abs(dd_residual) / sigma);
            if (sigma > 0.0 &&
                std::abs(dd_residual) / sigma >=
                    std::abs(worst_dd_residual_m) /
                        std::max(worst_dd_sigma, 1e-12)) {
                worst_reference_satellite = reference.real_satellite;
                worst_satellite = residual.real_satellite;
                worst_freq_group = static_cast<int>(freq_slot);
                worst_dd_residual_m = dd_residual;
                worst_dd_sigma = sigma;
            }

            if (freq_slot < 2) {
                auto& pair_info = phase_pairs[residual.real_satellite];
                pair_info.residual_m[freq_slot] = dd_residual;
                pair_info.variance_m2[freq_slot] = dd_variance;
                pair_info.frequency_hz[freq_slot] = residual.frequency_hz;
                pair_info.wavelength_m[freq_slot] = residual.wavelength_m;
            }
        }
    }

    for (const auto& [satellite, pair_info] : phase_pairs) {
        const bool has_l1 =
            std::isfinite(pair_info.residual_m[0]) &&
            std::isfinite(pair_info.variance_m2[0]) &&
            pair_info.frequency_hz[0] > 0.0 &&
            pair_info.wavelength_m[0] > 0.0;
        const bool has_l2 =
            std::isfinite(pair_info.residual_m[1]) &&
            std::isfinite(pair_info.variance_m2[1]) &&
            pair_info.frequency_hz[1] > 0.0 &&
            pair_info.wavelength_m[1] > 0.0;
        if (!has_l1 || !has_l2) {
            continue;
        }

        const double gamma =
            std::pow(pair_info.wavelength_m[1] / pair_info.wavelength_m[0], 2);
        const double denom = 1.0 - gamma;
        if (std::abs(denom) <= 1e-9) {
            continue;
        }

        const double max_variance =
            std::max(pair_info.variance_m2[0], pair_info.variance_m2[1]);
        const double dispersive =
            (pair_info.frequency_hz[0] / pair_info.frequency_hz[1]) *
            (pair_info.residual_m[0] - pair_info.residual_m[1]) / denom;
        const double nondispersive =
            (gamma * pair_info.residual_m[0] - pair_info.residual_m[1]) /
            (gamma - 1.0);
        worst_pair_satellite = satellite;
        worst_pair_dispersive = dispersive;
        worst_pair_nondispersive = nondispersive;
        worst_pair_sigma = std::sqrt(std::max(max_variance, 1e-12));
        if (dispersive * dispersive > 64.0 * max_variance ||
            nondispersive * nondispersive > 64.0 * max_variance) {
            pair_validation_ok = false;
            break;
        }
    }

    const int position_dof =
        (config.kinematic_mode || config.use_dynamics_model) ? 9 : 3;
    if (stats.phase_rows > 0) {
        stats.phase_rms = std::sqrt(phase_sum_sq / stats.phase_rows);
    }
    if (stats.code_rows > 0) {
        stats.code_rms = std::sqrt(code_sum_sq / stats.code_rows);
    }
    if (stats.phase_rows > position_dof) {
        const double chi_square_limit =
            claslibChiSquare001ForDof(stats.phase_rows - position_dof);
        if (chi_square_limit > 0.0) {
            stats.phase_chisq = phase_chi_sq / chi_square_limit;
        }
    }

    stats.accepted =
        stats.phase_rows > position_dof &&
        stats.max_phase_sigma < 4.0 &&
        pair_validation_ok &&
        stats.phase_chisq < 5.0;
    if (debug_enabled && !stats.accepted) {
        std::cerr << "[CLAS-FIX-DBG] worst_dd ref="
                  << worst_reference_satellite.toString()
                  << " sat=" << worst_satellite.toString()
                  << " freq=" << worst_freq_group
                  << " resid=" << worst_dd_residual_m
                  << " sigma=" << worst_dd_sigma
                  << " pair_sat=" << worst_pair_satellite.toString()
                  << " dispersive=" << worst_pair_dispersive
                  << " nondisp=" << worst_pair_nondispersive
                  << " pair_sigma=" << worst_pair_sigma
                  << "\n";
    }
    return stats;
}

AmbiguityResolutionResult resolveAndValidateAmbiguities(
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const ResolveAmbiguitiesFunction& resolve_ambiguities,
    const ValidateFixedSolutionFunction& validate_fixed_solution,
    bool debug_enabled) {
    AmbiguityResolutionResult result;
    if (!resolve_ambiguities) {
        return result;
    }

    const ppp_shared::PPPState pre_fix_state = filter_state;
    const auto pre_fix_ambiguities = ambiguity_states;

    result.attempted = true;
    if (!resolve_ambiguities()) {
        return result;
    }

    if (validate_fixed_solution) {
        result.validation_stats = validate_fixed_solution();
        result.accepted = result.validation_stats.accepted;
    } else {
        result.accepted = true;
    }

    if (!result.accepted) {
        filter_state = pre_fix_state;
        ambiguity_states = pre_fix_ambiguities;
        result.rejected_after_fix = true;
        if (debug_enabled) {
            std::cerr << "[CLAS-FIX] reject: phase_rows="
                      << result.validation_stats.phase_rows
                      << " phase_rms=" << result.validation_stats.phase_rms
                      << " phase_chisq=" << result.validation_stats.phase_chisq
                      << " max_phase_sigma=" << result.validation_stats.max_phase_sigma
                      << "\n";
        }
        return result;
    }

    if (debug_enabled) {
        std::cerr << "[CLAS-FIX] accept: phase_rows="
                  << result.validation_stats.phase_rows
                  << " phase_rms=" << result.validation_stats.phase_rms
                  << " phase_chisq=" << result.validation_stats.phase_chisq
                  << " max_phase_sigma=" << result.validation_stats.max_phase_sigma
                  << "\n";
    }

    return result;
}

void logUpdateSummary(
    const KalmanUpdateStats& update_stats,
    size_t satellite_count) {
    double code_rms = 0.0;
    double phase_rms = 0.0;
    int n_code = 0;
    int n_phase = 0;
    for (int i = 0; i < update_stats.nobs; ++i) {
        if (update_stats.variances(i) > 0.5) {
            code_rms += update_stats.residuals(i) * update_stats.residuals(i);
            ++n_code;
        } else {
            phase_rms += update_stats.residuals(i) * update_stats.residuals(i);
            ++n_phase;
        }
    }
    if (n_code > 0) {
        code_rms = std::sqrt(code_rms / n_code);
    }
    if (n_phase > 0) {
        phase_rms = std::sqrt(phase_rms / n_phase);
    }
    std::cerr << "[CLAS-PPP] rows=" << update_stats.nobs
              << " sats=" << satellite_count
              << " pos_delta=" << update_stats.dx.head(3).norm()
              << " code_rms=" << code_rms
              << " phase_rms=" << phase_rms
              << "\n";
}

PositionSolution finalizeEpochSolution(
    const ppp_shared::PPPState& filter_state,
    bool fixed,
    double ar_ratio,
    int fixed_ambiguities,
    int num_satellites) {
    PositionSolution solution;
    solution.position_ecef = filter_state.state.segment(0, 3);
    solution.receiver_clock_bias = filter_state.state(filter_state.clock_index);
    solution.status = fixed ? SolutionStatus::PPP_FIXED : SolutionStatus::PPP_FLOAT;
    solution.ratio = fixed ? ar_ratio : 0.0;
    solution.num_fixed_ambiguities = fixed ? fixed_ambiguities : 0;
    solution.num_satellites = num_satellites;
    return solution;
}

}  // namespace libgnss::ppp_clas
