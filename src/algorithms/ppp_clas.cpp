#include <libgnss++/algorithms/ppp_clas.hpp>

#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <string>
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

struct ClasDdresDebugPhaseRow {
    SatelliteId satellite;
    SatelliteId ambiguity_satellite;
    int freq_index = -1;
    int ar_freq_group = -1;
    int state_index = -1;
    double elevation_rad = std::numeric_limits<double>::quiet_NaN();
    double wavelength_m = std::numeric_limits<double>::quiet_NaN();
    double raw_phase_m = std::numeric_limits<double>::quiet_NaN();
    double carrier_phase_correction_m = std::numeric_limits<double>::quiet_NaN();
    double l_corr_m = std::numeric_limits<double>::quiet_NaN();
    double geometric_range_no_sagnac_m = std::numeric_limits<double>::quiet_NaN();
    double sagnac_m = std::numeric_limits<double>::quiet_NaN();
    double geometric_range_m = std::numeric_limits<double>::quiet_NaN();
    double satellite_clock_m = std::numeric_limits<double>::quiet_NaN();
    double receiver_clock_m = std::numeric_limits<double>::quiet_NaN();
    double trop_m = std::numeric_limits<double>::quiet_NaN();
    double iono_scale = std::numeric_limits<double>::quiet_NaN();
    double iono_state_m = std::numeric_limits<double>::quiet_NaN();
    double osr_trop_correction_m = std::numeric_limits<double>::quiet_NaN();
    double osr_relativity_m = std::numeric_limits<double>::quiet_NaN();
    double osr_receiver_antenna_m = std::numeric_limits<double>::quiet_NaN();
    double osr_iono_l1_m = std::numeric_limits<double>::quiet_NaN();
    double osr_code_bias_m = std::numeric_limits<double>::quiet_NaN();
    double osr_phase_bias_m = std::numeric_limits<double>::quiet_NaN();
    double osr_windup_m = std::numeric_limits<double>::quiet_NaN();
    double osr_prc_m = std::numeric_limits<double>::quiet_NaN();
    double osr_cpc_m = std::numeric_limits<double>::quiet_NaN();
    double osr_phase_compensation_m = std::numeric_limits<double>::quiet_NaN();
    double residual_without_ambiguity_m = std::numeric_limits<double>::quiet_NaN();
    double ambiguity_m = std::numeric_limits<double>::quiet_NaN();
    double residual_m = std::numeric_limits<double>::quiet_NaN();
    double variance_m2 = std::numeric_limits<double>::quiet_NaN();
    double pdiag_m2 = std::numeric_limits<double>::quiet_NaN();
    bool initialized_from_geometry = false;
    bool initialized_from_phase_code = false;
    bool reset_from_loss_of_lock = false;
};

double elevationWeight(double elevation_rad) {
    const double s = std::sin(elevation_rad);
    return 1.0 / (s * s);
}

int clasFrequencyGroupForSatellite(const SatelliteId& satellite, int freq_index) {
    if (freq_index <= 0) {
        return freq_index;
    }
    return satellite.system == GNSSSystem::Galileo ? 2 : freq_index;
}

double claslibDdUdVariance(double elevation_rad, bool is_phase, int freq_group) {
    constexpr double kPhaseErrorM = 0.010;
    constexpr double kPhaseElevationErrorM = 0.005;
    constexpr double kCodePhaseErrorRatio = 50.0;
    const double sin_el = std::max(std::sin(elevation_rad), 1e-6);
    double a = kPhaseErrorM;
    double b = kPhaseElevationErrorM;
    if (!is_phase) {
        a *= kCodePhaseErrorRatio;
        b *= kCodePhaseErrorRatio;
    }
    double variance = a * a + b * b / (sin_el * sin_el);
    if (is_phase && freq_group == 1) {
        variance *= std::pow(2.55 / 1.55, 2);
    }
    return variance;
}

SatelliteId ambiguitySatelliteForFrequency(const SatelliteId& satellite, int frequency_index) {
    const uint8_t ambiguity_prn = frequency_index == 0
        ? satellite.prn
        : static_cast<uint8_t>(std::min(255, satellite.prn + 100));
    return SatelliteId(satellite.system, ambiguity_prn);
}

bool clasDdresDumpTimeMatches(const GNSSTime& time) {
    const char* tow_text = std::getenv("GNSS_PPP_CLAS_DDRES_DUMP_TOW");
    if (tow_text == nullptr || tow_text[0] == '\0') {
        return true;
    }
    try {
        const double target_tow = std::stod(tow_text);
        return std::abs(time.tow - target_tow) < 5e-4;
    } catch (const std::exception&) {
        return false;
    }
}

const char* clasUpdateRowDumpPath() {
    return std::getenv("GNSS_PPP_CLAS_UPDATE_ROW_DUMP");
}

bool clasDdUpdateEnabled() {
    const char* value = std::getenv("GNSS_PPP_CLAS_DD_UPDATE");
    return value != nullptr && value[0] != '\0' && std::string(value) != "0";
}

bool clasDdOutlierInflationEnabled() {
    const char* value = std::getenv("GNSS_PPP_CLAS_DD_OUTLIER_INFLATION");
    return value != nullptr && value[0] != '\0' && std::string(value) != "0";
}

bool clasDdKeepPriorRowsEnabled() {
    const char* value = std::getenv("GNSS_PPP_CLAS_DD_KEEP_PRIORS");
    return value != nullptr && value[0] != '\0' && std::string(value) != "0";
}

double clasIonoDecayTauSeconds() {
    const char* value = std::getenv("GNSS_PPP_CLAS_IONO_DECAY_TAU");
    if (value == nullptr || value[0] == '\0') {
        return 0.0;
    }
    try {
        const double tau = std::stod(value);
        return std::isfinite(tau) && tau > 0.0 ? tau : 0.0;
    } catch (const std::exception&) {
        return 0.0;
    }
}

bool clasUpdateRowDumpEnabled(const GNSSTime* time) {
    const char* path = clasUpdateRowDumpPath();
    if (path == nullptr || path[0] == '\0' || time == nullptr) {
        return false;
    }
    return clasDdresDumpTimeMatches(*time);
}

double rowCoefficient(const Eigen::RowVectorXd& row, int index) {
    if (index < 0 || index >= row.size()) {
        return 0.0;
    }
    return row(index);
}

int clasUpdateFrequencyGroup(const MeasurementRow& measurement) {
    return clasFrequencyGroupForSatellite(measurement.satellite, measurement.freq_index);
}

void appendClasUpdateRowDump(
    const GNSSTime& time,
    const ppp_shared::PPPState& filter_state,
    const std::vector<MeasurementRow>& measurements,
    const Eigen::VectorXd& state_before,
    const Eigen::VectorXd& state_after_kf,
    const Eigen::VectorXd& state_after_anchor,
    const Eigen::MatrixXd& K,
    const Eigen::VectorXd& z,
    const Eigen::MatrixXd& R,
    const std::vector<bool>& row_inflated,
    const PositionSolution* seed_solution) {
    const char* path = clasUpdateRowDumpPath();
    if (path == nullptr || path[0] == '\0') {
        return;
    }
    std::ofstream dump(path, std::ios::app);
    if (!dump) {
        return;
    }
    dump << std::fixed << std::setprecision(12);

    const auto position_or_zero = [](const Eigen::VectorXd& state, int offset) {
        return offset >= 0 && offset < state.size() ? state(offset) : 0.0;
    };
    const Eigen::VectorXd dx_kf = state_after_kf - state_before;
    const Eigen::VectorXd dx_anchor = state_after_anchor - state_before;
    const double seed_x =
        seed_solution != nullptr && seed_solution->isValid()
            ? seed_solution->position_ecef.x()
            : std::numeric_limits<double>::quiet_NaN();
    const double seed_y =
        seed_solution != nullptr && seed_solution->isValid()
            ? seed_solution->position_ecef.y()
            : std::numeric_limits<double>::quiet_NaN();
    const double seed_z =
        seed_solution != nullptr && seed_solution->isValid()
            ? seed_solution->position_ecef.z()
            : std::numeric_limits<double>::quiet_NaN();
    dump << "[CLAS-UPDATE-SUM] source=libgnss week=" << time.week
         << " tow=" << time.tow
         << " nobs=" << measurements.size()
         << " pos_before_x=" << position_or_zero(state_before, 0)
         << " pos_before_y=" << position_or_zero(state_before, 1)
         << " pos_before_z=" << position_or_zero(state_before, 2)
         << " clock_before_m=" << position_or_zero(state_before, filter_state.clock_index)
         << " pos_after_kf_x=" << position_or_zero(state_after_kf, 0)
         << " pos_after_kf_y=" << position_or_zero(state_after_kf, 1)
         << " pos_after_kf_z=" << position_or_zero(state_after_kf, 2)
         << " clock_after_kf_m=" << position_or_zero(state_after_kf, filter_state.clock_index)
         << " pos_after_anchor_x=" << position_or_zero(state_after_anchor, 0)
         << " pos_after_anchor_y=" << position_or_zero(state_after_anchor, 1)
         << " pos_after_anchor_z=" << position_or_zero(state_after_anchor, 2)
         << " clock_after_anchor_m=" << position_or_zero(state_after_anchor, filter_state.clock_index)
         << " dx_kf_x=" << position_or_zero(dx_kf, 0)
         << " dx_kf_y=" << position_or_zero(dx_kf, 1)
         << " dx_kf_z=" << position_or_zero(dx_kf, 2)
         << " dx_kf_clock_m=" << position_or_zero(dx_kf, filter_state.clock_index)
         << " dx_anchor_x=" << position_or_zero(dx_anchor, 0)
         << " dx_anchor_y=" << position_or_zero(dx_anchor, 1)
         << " dx_anchor_z=" << position_or_zero(dx_anchor, 2)
         << " dx_anchor_clock_m=" << position_or_zero(dx_anchor, filter_state.clock_index)
         << " seed_x=" << seed_x
         << " seed_y=" << seed_y
         << " seed_z=" << seed_z
         << " seed_clock_m="
         << (seed_solution != nullptr && seed_solution->isValid()
                 ? seed_solution->receiver_clock_bias
                 : std::numeric_limits<double>::quiet_NaN())
         << "\n";

    for (size_t i = 0; i < measurements.size(); ++i) {
        const auto& measurement = measurements[i];
        const int row = static_cast<int>(i);
        const SatelliteId ambiguity_satellite =
            measurement.phase_ambiguities.empty()
                ? SatelliteId()
                : measurement.phase_ambiguities.front();
        const auto amb_it =
            filter_state.ambiguity_indices.find(ambiguity_satellite);
        const int amb_idx =
            amb_it != filter_state.ambiguity_indices.end() ? amb_it->second : -1;
        const auto iono_it =
            filter_state.ionosphere_indices.find(measurement.satellite);
        const int iono_idx =
            iono_it != filter_state.ionosphere_indices.end() ? iono_it->second : -1;
        const int clock_idx = filter_state.clock_index;
        const int trop_idx = filter_state.trop_index;
        const double hpx = rowCoefficient(measurement.H, 0);
        const double hpy = rowCoefficient(measurement.H, 1);
        const double hpz = rowCoefficient(measurement.H, 2);
        const double hpos = std::sqrt(hpx * hpx + hpy * hpy + hpz * hpz);
        const double state_before_m =
            amb_idx >= 0 && amb_idx < state_before.size()
                ? state_before(amb_idx)
                : std::numeric_limits<double>::quiet_NaN();
        const double state_after_kf_m =
            amb_idx >= 0 && amb_idx < state_after_kf.size()
                ? state_after_kf(amb_idx)
                : std::numeric_limits<double>::quiet_NaN();
        const double state_after_anchor_m =
            amb_idx >= 0 && amb_idx < state_after_anchor.size()
                ? state_after_anchor(amb_idx)
                : std::numeric_limits<double>::quiet_NaN();
        const double k_amb =
            amb_idx >= 0 && amb_idx < K.rows() && row < K.cols()
                ? K(amb_idx, row)
                : 0.0;
        dump << "[CLAS-UPDATE-ROW] source=libgnss week=" << time.week
             << " tow=" << time.tow
             << " row=" << row
             << " sat=" << measurement.satellite.toString()
             << " ref="
             << (measurement.reference_satellite.prn != 0
                     ? measurement.reference_satellite.toString()
                     : "none")
             << " amb_sat="
             << (measurement.is_phase ? ambiguity_satellite.toString() : "none")
             << " f=" << measurement.freq_index
             << " freq_group=" << clasUpdateFrequencyGroup(measurement)
             << " type=" << (measurement.is_phase ? "phase" : "code")
             << " h_pos_x=" << hpx
             << " h_pos_y=" << hpy
             << " h_pos_z=" << hpz
             << " h_pos_norm=" << hpos
             << " h_clk=" << rowCoefficient(measurement.H, clock_idx)
             << " h_trop=" << rowCoefficient(measurement.H, trop_idx)
             << " h_iono=" << rowCoefficient(measurement.H, iono_idx)
             << " h_amb=" << rowCoefficient(measurement.H, amb_idx)
             << " y=" << (row < z.size() ? z(row) : 0.0)
             << " R=" << (row < R.rows() ? R(row, row) : 0.0)
             << " R_ref=" << measurement.reference_variance
             << " cov_group=" << measurement.covariance_group
             << " row_inflated="
             << (i < row_inflated.size() && row_inflated[i] ? 1 : 0)
             << " state_before_m=" << state_before_m
             << " state_after_kf_m=" << state_after_kf_m
             << " state_after_anchor_m=" << state_after_anchor_m
             << " dx_state_kf_m=" << (state_after_kf_m - state_before_m)
             << " dx_state_anchor_m=" << (state_after_anchor_m - state_before_m)
             << " K_pos_x=" << (row < K.cols() && K.rows() > 0 ? K(0, row) : 0.0)
             << " K_pos_y=" << (row < K.cols() && K.rows() > 1 ? K(1, row) : 0.0)
             << " K_pos_z=" << (row < K.cols() && K.rows() > 2 ? K(2, row) : 0.0)
             << " K_amb=" << k_amb
             << " contrib_pos_x="
             << (row < K.cols() && K.rows() > 0 && row < z.size() ? K(0, row) * z(row) : 0.0)
             << " contrib_pos_y="
             << (row < K.cols() && K.rows() > 1 && row < z.size() ? K(1, row) * z(row) : 0.0)
             << " contrib_pos_z="
             << (row < K.cols() && K.rows() > 2 && row < z.size() ? K(2, row) * z(row) : 0.0)
             << " contrib_amb_m="
             << (row < z.size() ? k_amb * z(row) : 0.0)
             << "\n";
    }
}

std::set<int> initializeAmbiguitiesFromPhaseCode(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    bool debug_enabled) {
    struct Candidate {
        SatelliteId ambiguity_satellite;
        int state_index = -1;
        double bias_m = 0.0;
    };

    std::map<int, std::vector<Candidate>> candidates_by_frequency;
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        for (int f = 0; f < osr.num_frequencies; ++f) {
            if (f < 0 || f >= OSR_MAX_FREQ || osr.wavelengths[f] <= 0.0) {
                continue;
            }
            const Observation* raw = obs.getObservation(osr.satellite, osr.signals[f]);
            if (!raw || !raw->valid || !raw->has_carrier_phase ||
                !raw->has_pseudorange || !std::isfinite(raw->carrier_phase) ||
                !std::isfinite(raw->pseudorange)) {
                continue;
            }
            const SatelliteId ambiguity_satellite =
                ambiguitySatelliteForFrequency(osr.satellite, f);
            const auto ambiguity_it =
                filter_state.ambiguity_indices.find(ambiguity_satellite);
            if (ambiguity_it == filter_state.ambiguity_indices.end()) {
                continue;
            }
            const int state_index = ambiguity_it->second;
            if (state_index < 0 || state_index >= filter_state.total_states) {
                continue;
            }
            const double bias_m =
                raw->carrier_phase * osr.wavelengths[f] - raw->pseudorange;
            if (!std::isfinite(bias_m)) {
                continue;
            }
            candidates_by_frequency[f].push_back(
                Candidate{ambiguity_satellite, state_index, bias_m});
        }
    }

    std::set<int> initialized_indices;
    for (const auto& [frequency_index, candidates] : candidates_by_frequency) {
        double offset_m = 0.0;
        int offset_count = 0;
        for (const auto& candidate : candidates) {
            const double state_m = filter_state.state(candidate.state_index);
            const double covariance_m2 =
                filter_state.covariance(candidate.state_index, candidate.state_index);
            const bool needs_initialization =
                state_m == 0.0 || covariance_m2 >= config.clas_ambiguity_reinit_threshold;
            if (!needs_initialization && std::isfinite(state_m)) {
                offset_m += candidate.bias_m - state_m;
                ++offset_count;
            }
        }
        const double common_bias_m = offset_count > 0 ? offset_m / offset_count : 0.0;
        for (const auto& candidate : candidates) {
            const double state_m = filter_state.state(candidate.state_index);
            const double covariance_m2 =
                filter_state.covariance(candidate.state_index, candidate.state_index);
            const bool needs_initialization =
                state_m == 0.0 || covariance_m2 >= config.clas_ambiguity_reinit_threshold;
            if (!needs_initialization) {
                continue;
            }
            filter_state.state(candidate.state_index) = candidate.bias_m - common_bias_m;
            initialized_indices.insert(candidate.state_index);
            if (debug_enabled) {
                std::cerr << "[CLAS-AMB-INIT] phase-code "
                          << candidate.ambiguity_satellite.toString()
                          << " f=" << frequency_index
                          << " bias_m=" << candidate.bias_m
                          << " common_bias_m=" << common_bias_m
                          << " state_m=" << filter_state.state(candidate.state_index)
                          << "\n";
            }
        }
    }
    return initialized_indices;
}

}  // namespace

AppliedOsrCorrections selectAppliedOsrCorrections(
    const OSRCorrection& osr,
    int freq_index,
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy) {
    AppliedOsrCorrections corrections;
    if (freq_index < 0 || freq_index >= osr.num_frequencies) {
        return corrections;
    }

    const double relativity = osr.relativity_correction_m;
    const double receiver_antenna = osr.receiver_antenna_m[freq_index];
    const double code_bias = osr.code_bias_m[freq_index];
    const double phase_bias = osr.phase_bias_m[freq_index];
    const double windup = osr.windup_m[freq_index];
    const double phase_compensation = osr.phase_compensation_m[freq_index];

    switch (policy) {
    case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR:
        corrections.pseudorange_correction_m =
            osr.PRC[freq_index] - osr.trop_correction_m;
        corrections.carrier_phase_correction_m =
            osr.CPC[freq_index] - osr.trop_correction_m;
        break;
    case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_BIAS:
        corrections.pseudorange_correction_m =
            relativity + receiver_antenna + code_bias;
        corrections.carrier_phase_correction_m =
            relativity + receiver_antenna + phase_bias + windup + phase_compensation;
        break;
    case ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_ONLY:
        corrections.pseudorange_correction_m =
            relativity + receiver_antenna;
        corrections.carrier_phase_correction_m =
            relativity + receiver_antenna + windup;
        break;
    }

    return corrections;
}

bool usesClasTropospherePrior(
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy) {
    return policy ==
           ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
}

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
    // CLASLIB-faithful kinematic position re-seed: replace position state with
    // current SPP solution and reset variance, mirroring `udpos_ppp` in
    // RTKLIB/src/ppp.c which calls `initx(rtk, rtk->sol.rr[i], VAR_POS, i)`
    // every kinematic epoch. Required for KF to escape the wrong-ambiguity ⊗
    // tight-position Nash equilibrium documented in
    // clas_kf_overconfidence_state_diff_2026_05_02.md.
    if (config.clas_kinematic_position_reseed && config.kinematic_mode &&
        seed_solution.isValid()) {
        const double rms_gate = config.clas_kinematic_position_reseed_max_residual_rms_m;
        const bool seed_quality_ok =
            rms_gate <= 0.0 || seed_solution.residual_rms <= rms_gate;
        if (seed_quality_ok) {
            const int nx = filter_state.total_states;
            filter_state.state.segment(0, 3) = seed_solution.position_ecef;
            filter_state.covariance.block(0, 0, 3, nx).setZero();
            filter_state.covariance.block(0, 0, nx, 3).setZero();
            const double v = config.clas_kinematic_position_reseed_variance;
            filter_state.covariance(0, 0) = v;
            filter_state.covariance(1, 1) = v;
            filter_state.covariance(2, 2) = v;
        } else if (ppp_shared::pppDebugEnabled()) {
            std::cerr << "[CLAS-RESEED-SKIP] residual_rms="
                      << seed_solution.residual_rms
                      << " > gate=" << rms_gate << "\n";
        }
    }
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
    filter_state.covariance.block(0, 0, 3, 3) *= config.clas_initial_position_variance;
    filter_state.covariance(6, 6) = config.clas_clock_variance;
    filter_state.covariance(7, 7) = config.clas_clock_variance;
    filter_state.covariance(8, 8) = config.clas_trop_initial_variance;
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
    Q(filter_state.clock_index, filter_state.clock_index) = config.clas_clock_variance;
    Q(filter_state.glo_clock_index, filter_state.glo_clock_index) = config.clas_clock_variance;
    Q(filter_state.trop_index, filter_state.trop_index) = config.clas_trop_process_noise * dt;
    if (config.estimate_ionosphere) {
        const double iono_decay_tau = clasIonoDecayTauSeconds();
        if (iono_decay_tau > 0.0 && dt > 0.0) {
            const double ki = std::exp(-dt / iono_decay_tau);
            for (const auto& [_, state_index] : filter_state.ionosphere_indices) {
                if (state_index >= 0 && state_index < nx) {
                    filter_state.state(state_index) *= ki;
                    for (int j = 0; j < nx; ++j) {
                        filter_state.covariance(state_index, j) *= ki;
                        filter_state.covariance(j, state_index) *= ki;
                    }
                }
            }
        }
        for (const auto& [_, state_index] : filter_state.ionosphere_indices) {
            if (state_index >= 0 && state_index < nx) {
                Q(state_index, state_index) = config.process_noise_ionosphere * dt;
            }
        }
    }
    for (const auto& [_, state_index] : filter_state.ambiguity_indices) {
        if (state_index >= 0 && state_index < nx) {
            Q(state_index, state_index) = config.process_noise_ambiguity * dt;
        }
    }
    filter_state.covariance += Q;
    // Receiver-clock evolution policy. Default (`clas_spp_clock_overwrite=true`)
    // pins the KF clock state to SPP every epoch and zeros the variance via
    // the decouple block below; this is the historical behaviour that
    // empirically produces the +11m sat-uniform residual systematic
    // documented in clas_per_sat_residual_root_cause_2026_05_08.md (because
    // SPP clock and CLASLIB PPP-RTK clock differ by exactly that). Setting
    // `clas_spp_clock_overwrite=false` lets the KF track clock from
    // observations: state is left as predicted, and when the decouple block
    // runs we seed a working variance instead of zeroing it.
    if (seed_valid && config.clas_spp_clock_overwrite) {
        filter_state.state(filter_state.clock_index) = seed_receiver_clock_bias_m;
        filter_state.state(filter_state.glo_clock_index) = seed_receiver_clock_bias_m;
    }
    if (config.clas_decouple_clock_position) {
        const int ci = filter_state.clock_index;
        const int gi = filter_state.glo_clock_index;
        for (int i = 0; i < nx; ++i) {
            if (i != ci) { filter_state.covariance(ci, i) = 0; filter_state.covariance(i, ci) = 0; }
            if (i != gi) { filter_state.covariance(gi, i) = 0; filter_state.covariance(i, gi) = 0; }
        }
        if (config.clas_spp_clock_overwrite) {
            filter_state.covariance(ci, ci) = 0;
            filter_state.covariance(gi, gi) = 0;
        } else {
            const double v = config.clas_kf_clock_seed_variance;
            filter_state.covariance(ci, ci) = std::max(filter_state.covariance(ci, ci), v);
            filter_state.covariance(gi, gi) = std::max(filter_state.covariance(gi, gi), v);
        }
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
    const ppp_shared::PPPConfig& config) {
    auto initialVarianceForFrequency = [&](const OSRCorrection& osr, int frequency_index) {
        if (frequency_index >= 0 && frequency_index < OSR_MAX_FREQ) {
            const double wavelength = osr.wavelengths[static_cast<size_t>(frequency_index)];
            if (std::isfinite(wavelength) && wavelength > 0.0 &&
                std::isfinite(config.clas_initial_ambiguity_std_cycles) &&
                config.clas_initial_ambiguity_std_cycles > 0.0) {
                const double std_m =
                    config.clas_initial_ambiguity_std_cycles * wavelength;
                return std_m * std_m;
            }
        }
        return config.initial_ambiguity_variance;
    };
    auto allocate_ambiguity = [&](const SatelliteId& ambiguity_satellite,
                                  double initial_variance) {
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
        allocate_ambiguity(osr.satellite, initialVarianceForFrequency(osr, 0));
        if (osr.num_frequencies >= 2) {
            allocate_ambiguity(
                ambiguitySatelliteForFrequency(osr.satellite, 1),
                initialVarianceForFrequency(osr, 1));
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
    std::map<size_t, ClasDdresDebugPhaseRow> phase_debug_by_measurement_index;
    const std::set<int> phase_code_initialized_indices =
        config.clas_phase_code_ambiguity_initialization
            ? initializeAmbiguitiesFromPhaseCode(
                  obs, osr_corrections, filter_state, config, debug_enabled)
            : std::set<int>{};
    const bool dd_update_mode =
        config.use_clas_osr_filter && clasDdUpdateEnabled();

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
            const auto applied_corrections = selectAppliedOsrCorrections(
                osr, f, config.clas_correction_application_policy);
            const double iono_scale =
                (osr.frequencies[f] > 0.0 && osr.wavelengths[0] > 0.0)
                    ? std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2)
                    : 1.0;
            const double iono_state_m =
                use_residual_iono_state ? filter_state.state(iono_state_index) : 0.0;

            if (raw->has_pseudorange && std::isfinite(raw->pseudorange)) {
                const double p_corr =
                    raw->pseudorange - applied_corrections.pseudorange_correction_m;
                const double predicted =
                    geo - sat_clk_m + receiver_clock_m + trop_modeled +
                    iono_scale * iono_state_m;
                const double residual = p_corr - predicted;

                const double el_weight = elevationWeight(osr.elevation);

                MeasurementRow row;
                row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
                row.H.segment(0, 3) = -los.transpose();
                row.H(filter_state.clock_index) = 1.0;
                if (config.estimate_troposphere) {
                    row.H(filter_state.trop_index) = trop_mapping;
                }
                if (use_residual_iono_state) {
                    row.H(iono_state_index) = iono_scale;
                    result.observed_iono_states.insert(osr.satellite);
                }
                row.residual = residual;
                row.observation = p_corr;
                row.predicted = predicted;
                row.variance =
                    dd_update_mode
                        ? claslibDdUdVariance(
                              osr.elevation,
                              false,
                              clasFrequencyGroupForSatellite(osr.satellite, f))
                        : config.clas_code_variance_scale * el_weight;
                row.satellite = osr.satellite;
                row.primary_signal = raw->signal;
                row.primary_observation_code = raw->observation_code;
                row.is_phase = false;
                row.freq_index = f;
                row.elevation_rad = osr.elevation;
                row.ionosphere_coefficient = iono_scale;
                row.ionosphere_state_index = iono_state_index;
                row.ionosphere_design_coeff = use_residual_iono_state ? iono_scale : 0.0;
                row.ionosphere_state_m = iono_state_m;
                result.measurements.push_back(row);
            }

            if (raw->has_carrier_phase && std::isfinite(raw->carrier_phase)) {
                const double l_m = raw->carrier_phase * osr.wavelengths[f];
                const double l_corr =
                    l_m - applied_corrections.carrier_phase_correction_m;

                const SatelliteId amb_sat =
                    ambiguitySatelliteForFrequency(osr.satellite, f);
                const auto amb_it = filter_state.ambiguity_indices.find(amb_sat);
                if (amb_it == filter_state.ambiguity_indices.end()) {
                    continue;
                }
                const int amb_idx = amb_it->second;

                const bool reset_from_loss_of_lock =
                    raw->loss_of_lock && static_cast<bool>(ambiguity_reset_function);
                if (reset_from_loss_of_lock) {
                    ambiguity_reset_function(amb_sat, raw->signal);
                }

                const bool needs_ambiguity_initialization =
                    filter_state.state(amb_idx) == 0.0 ||
                    filter_state.covariance(amb_idx, amb_idx) >=
                        config.clas_ambiguity_reinit_threshold;
                const bool phase_code_initialized =
                    !reset_from_loss_of_lock &&
                    phase_code_initialized_indices.count(amb_idx) > 0;
                bool initialized_from_geometry = false;
                if (needs_ambiguity_initialization && !phase_code_initialized) {
                    filter_state.state(amb_idx) =
                        l_corr - (geo - sat_clk_m + receiver_clock_m + trop_modeled
                                  - iono_scale * iono_state_m);
                    initialized_from_geometry = true;
                }

                const double residual_without_ambiguity =
                    l_corr - (geo - sat_clk_m + receiver_clock_m + trop_modeled
                              - iono_scale * iono_state_m);
                const double ambiguity_m = filter_state.state(amb_idx);
                const double predicted =
                    geo - sat_clk_m + receiver_clock_m + trop_modeled
                    - iono_scale * iono_state_m + ambiguity_m;
                const double residual = l_corr - predicted;

                const double el_weight = elevationWeight(osr.elevation);
                const int phase_freq_group =
                    ppp_ar::ambiguityDdGroup(amb_sat).second;
                const double variance_m2 =
                    dd_update_mode
                        ? claslibDdUdVariance(
                              osr.elevation, true, phase_freq_group)
                        : config.clas_phase_variance * el_weight;

                MeasurementRow row;
                row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
                row.H.segment(0, 3) = -los.transpose();
                row.H(filter_state.clock_index) = 1.0;
                if (config.estimate_troposphere) {
                    row.H(filter_state.trop_index) = trop_mapping;
                }
                if (use_residual_iono_state) {
                    row.H(iono_state_index) = -iono_scale;
                    result.observed_iono_states.insert(osr.satellite);
                }
                row.H(amb_idx) = 1.0;
                row.residual = residual;
                row.observation = l_corr;
                row.predicted = predicted;
                row.variance = variance_m2;
                row.satellite = osr.satellite;
                row.phase_ambiguities.push_back(amb_sat);
                row.primary_signal = raw->signal;
                row.primary_observation_code = raw->observation_code;
                row.is_phase = true;
                row.freq_index = f;
                row.elevation_rad = osr.elevation;
                row.ionosphere_coefficient = iono_scale;
                row.ionosphere_state_index = iono_state_index;
                row.ionosphere_design_coeff = use_residual_iono_state ? -iono_scale : 0.0;
                row.ionosphere_state_m = iono_state_m;
                row.ambiguity_state_index = amb_idx;
                row.ambiguity_design_coeff = 1.0;
                const size_t measurement_index = result.measurements.size();
                result.measurements.push_back(row);
                ClasDdresDebugPhaseRow debug_row;
                debug_row.satellite = osr.satellite;
                debug_row.ambiguity_satellite = amb_sat;
                debug_row.freq_index = f;
                debug_row.ar_freq_group = phase_freq_group;
                debug_row.state_index = amb_idx;
                debug_row.elevation_rad = osr.elevation;
                debug_row.wavelength_m = osr.wavelengths[f];
                debug_row.raw_phase_m = l_m;
                debug_row.carrier_phase_correction_m =
                    applied_corrections.carrier_phase_correction_m;
                debug_row.l_corr_m = l_corr;
                debug_row.geometric_range_no_sagnac_m =
                    (osr.satellite_position - receiver_position).norm();
                debug_row.sagnac_m =
                    geo - debug_row.geometric_range_no_sagnac_m;
                debug_row.geometric_range_m = geo;
                debug_row.satellite_clock_m = sat_clk_m;
                debug_row.receiver_clock_m = receiver_clock_m;
                debug_row.trop_m = trop_modeled;
                debug_row.iono_scale = iono_scale;
                debug_row.iono_state_m = iono_state_m;
                debug_row.osr_trop_correction_m = osr.trop_correction_m;
                debug_row.osr_relativity_m = osr.relativity_correction_m;
                debug_row.osr_receiver_antenna_m = osr.receiver_antenna_m[f];
                debug_row.osr_iono_l1_m = osr.iono_l1_m;
                debug_row.osr_code_bias_m = osr.code_bias_m[f];
                debug_row.osr_phase_bias_m = osr.phase_bias_m[f];
                debug_row.osr_windup_m = osr.windup_m[f];
                debug_row.osr_prc_m = osr.PRC[f];
                debug_row.osr_cpc_m = osr.CPC[f];
                debug_row.osr_phase_compensation_m =
                    osr.phase_compensation_m[f];
                debug_row.residual_without_ambiguity_m = residual_without_ambiguity;
                debug_row.ambiguity_m = ambiguity_m;
                debug_row.residual_m = residual;
                debug_row.variance_m2 = variance_m2;
                debug_row.pdiag_m2 =
                    amb_idx >= 0 && amb_idx < filter_state.covariance.rows()
                        ? filter_state.covariance(amb_idx, amb_idx)
                        : std::numeric_limits<double>::quiet_NaN();
                debug_row.initialized_from_geometry = initialized_from_geometry;
                debug_row.initialized_from_phase_code = phase_code_initialized;
                debug_row.reset_from_loss_of_lock = reset_from_loss_of_lock;
                phase_debug_by_measurement_index[measurement_index] = debug_row;
                result.observed_ambiguities.push_back(
                    {amb_sat, raw->signal, osr.wavelengths[f], raw->carrier_phase, raw->snr});
            }
        }
    }

    if (config.estimate_troposphere && !epoch_atmos.empty() &&
        usesClasTropospherePrior(config.clas_correction_application_policy)) {
        const double clas_trop_zenith =
            ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
                epoch_atmos,
                receiver_position,
                obs.time,
                M_PI_2,
                config.clas_expanded_value_construction_policy,
                config.clas_subtype12_value_construction_policy,
                config.clas_expanded_residual_sampling_policy);
        if (std::isfinite(clas_trop_zenith) && clas_trop_zenith > 0.0) {
            MeasurementRow row;
            row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
            row.H(filter_state.trop_index) = 1.0;
            row.residual = clas_trop_zenith - trop_zenith;
            row.observation = clas_trop_zenith;
            row.predicted = trop_zenith;
            row.variance = config.clas_trop_prior_variance;
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
            row.observation = 0.0;
            row.predicted = filter_state.state(iono_state_index);
            row.variance = config.clas_iono_prior_variance;
            row.satellite = satellite;
            row.is_phase = false;
            row.freq_index = -1;
            row.ionosphere_state_index = iono_state_index;
            row.ionosphere_design_coeff = 1.0;
            row.ionosphere_state_m = filter_state.state(iono_state_index);
            result.measurements.push_back(row);
        }
    }

    if (config.use_clas_osr_filter) {
        // Build elevation map for reference satellite selection
        std::map<SatelliteId, double> elevation_map;
        for (const auto& osr : osr_corrections) {
            if (osr.valid) {
                elevation_map[osr.satellite] = osr.elevation;
            }
        }

        // Group measurements by GNSS system, frequency, and observable type.
        // Default CLAS OSR parity only differences phase rows; the diagnostic
        // DD mode also differences code rows and drops pseudo-priors so the row
        // set can be compared directly with CLASLIB ddres()/filter2().
        struct DifferenceGroupKey {
            GNSSSystem system;
            int freq_index;
            bool is_phase;
            bool operator<(const DifferenceGroupKey& rhs) const {
                if (system != rhs.system) return system < rhs.system;
                if (freq_index != rhs.freq_index) return freq_index < rhs.freq_index;
                if (is_phase != rhs.is_phase) return is_phase < rhs.is_phase;
                return false;
            }
        };
        std::map<DifferenceGroupKey, std::vector<size_t>> difference_groups;
        for (size_t i = 0; i < result.measurements.size(); ++i) {
            const auto& m = result.measurements[i];
            if (m.freq_index < 0) {
                continue;
            }
            if (!m.is_phase && !dd_update_mode) {
                continue;
            }
            difference_groups[{m.satellite.system, m.freq_index, m.is_phase}]
                .push_back(i);
        }

        std::ofstream ddres_dump;
        const char* ddres_dump_path = std::getenv("GNSS_PPP_CLAS_DDRES_DUMP");
        if (ddres_dump_path != nullptr && ddres_dump_path[0] != '\0' &&
            clasDdresDumpTimeMatches(obs.time)) {
            ddres_dump.open(ddres_dump_path, std::ios::app);
            if (ddres_dump) {
                ddres_dump << std::fixed << std::setprecision(12);
                ddres_dump << "strict_clas_ddres_dump=1\n"
                            << "time_week=" << obs.time.week << "\n"
                            << "time_tow=" << obs.time.tow << "\n"
                            << "receiver_x_m=" << receiver_position.x() << "\n"
                            << "receiver_y_m=" << receiver_position.y() << "\n"
                            << "receiver_z_m=" << receiver_position.z() << "\n"
                            << "receiver_clock_m=" << receiver_clock_m << "\n"
                            << "[ud_phase]\n";
                for (const auto& [measurement_index, debug_row] :
                     phase_debug_by_measurement_index) {
                    const double x_cycles =
                        std::isfinite(debug_row.wavelength_m) &&
                                std::abs(debug_row.wavelength_m) > 0.0
                            ? debug_row.ambiguity_m / debug_row.wavelength_m
                            : std::numeric_limits<double>::quiet_NaN();
                    const double pdiag_cycles2 =
                        std::isfinite(debug_row.wavelength_m) &&
                                std::abs(debug_row.wavelength_m) > 0.0
                            ? debug_row.pdiag_m2 /
                                  (debug_row.wavelength_m * debug_row.wavelength_m)
                            : std::numeric_limits<double>::quiet_NaN();
                    ddres_dump << "i=" << measurement_index
                                << " sat=" << debug_row.satellite.toString()
                                << " state_sat="
                                << debug_row.ambiguity_satellite.toString()
                                << " system="
                                << static_cast<int>(debug_row.satellite.system)
                                << " freq_index=" << debug_row.freq_index
                                << " freq_group=" << debug_row.ar_freq_group
                                << " state_index=" << debug_row.state_index
                                << " wavelength_m=" << debug_row.wavelength_m
                                << " elev_rad=" << debug_row.elevation_rad
                                << " raw_phase_m=" << debug_row.raw_phase_m
                                << " carrier_phase_correction_m="
                                << debug_row.carrier_phase_correction_m
                                << " l_corr_m=" << debug_row.l_corr_m
                                << " geometric_range_no_sagnac_m="
                                << debug_row.geometric_range_no_sagnac_m
                                << " sagnac_m=" << debug_row.sagnac_m
                                << " geometric_range_m="
                                << debug_row.geometric_range_m
                                << " satellite_clock_m="
                                << debug_row.satellite_clock_m
                                << " receiver_clock_m="
                                << debug_row.receiver_clock_m
                                << " trop_m=" << debug_row.trop_m
                                << " iono_scale=" << debug_row.iono_scale
                                << " iono_state_m=" << debug_row.iono_state_m
                                << " osr_trop_correction_m="
                                << debug_row.osr_trop_correction_m
                                << " osr_relativity_m="
                                << debug_row.osr_relativity_m
                                << " osr_receiver_antenna_m="
                                << debug_row.osr_receiver_antenna_m
                                << " osr_iono_l1_m=" << debug_row.osr_iono_l1_m
                                << " osr_code_bias_m="
                                << debug_row.osr_code_bias_m
                                << " osr_phase_bias_m="
                                << debug_row.osr_phase_bias_m
                                << " osr_windup_m=" << debug_row.osr_windup_m
                                << " osr_prc_m=" << debug_row.osr_prc_m
                                << " osr_cpc_m=" << debug_row.osr_cpc_m
                                << " osr_phase_compensation_m="
                                << debug_row.osr_phase_compensation_m
                                << " residual_noamb_m="
                                << debug_row.residual_without_ambiguity_m
                                << " ambiguity_m=" << debug_row.ambiguity_m
                                << " x_cycles=" << x_cycles
                                << " residual_m=" << debug_row.residual_m
                                << " variance_m2=" << debug_row.variance_m2
                                << " pdiag_m2=" << debug_row.pdiag_m2
                                << " pdiag_cycles2=" << pdiag_cycles2
                                << " init_geometry="
                                << (debug_row.initialized_from_geometry ? 1 : 0)
                                << " init_phase_code="
                                << (debug_row.initialized_from_phase_code ? 1 : 0)
                                << " reset_lli="
                                << (debug_row.reset_from_loss_of_lock ? 1 : 0)
                                << "\n";
                }
                ddres_dump << "[sd_phase]\n";
            }
        }

        std::vector<MeasurementRow> differenced_measurements;
        if (!dd_update_mode || clasDdKeepPriorRowsEnabled()) {
            // Preserve the production parity path: only carrier phase rows are
            // single-differenced; code and prior constraints remain UD.
            for (const auto& m : result.measurements) {
                if ((!dd_update_mode && !m.is_phase) || m.freq_index < 0) {
                    differenced_measurements.push_back(m);
                }
            }
        }

        // Form reference-minus-satellite differences for each group.
        int sd_debug_index = 0;
        int covariance_group_index = 0;
        for (auto& [group_key, member_indices] : difference_groups) {
            if (member_indices.size() < 2) continue;
            const int covariance_group =
                dd_update_mode ? covariance_group_index++ : -1;

            // Select reference satellite: highest elevation
            size_t ref_index = member_indices[0];
            double max_elevation = -1.0;
            for (size_t idx : member_indices) {
                auto el_it = elevation_map.find(result.measurements[idx].satellite);
                if (el_it != elevation_map.end() && el_it->second > max_elevation) {
                    max_elevation = el_it->second;
                    ref_index = idx;
                }
            }

            // Form SD: reference - satellite
            const auto& ref_row = result.measurements[ref_index];
            if (ddres_dump && group_key.is_phase) {
                ddres_dump << "kind=reference"
                            << " system=" << static_cast<int>(group_key.system)
                            << " freq_group=" << group_key.freq_index
                            << " ref=" << ref_row.satellite.toString()
                            << " ref_measurement_index=" << ref_index
                            << " member_count=" << member_indices.size()
                            << " ref_elev_rad=" << max_elevation
                            << "\n";
            }
            for (size_t idx : member_indices) {
                if (idx == ref_index) continue;
                const auto& sat_row = result.measurements[idx];
                MeasurementRow sd_row;
                sd_row.H = ref_row.H - sat_row.H;
                sd_row.observation = ref_row.observation - sat_row.observation;
                sd_row.predicted = ref_row.predicted - sat_row.predicted;
                sd_row.residual = ref_row.residual - sat_row.residual;
                sd_row.variance = ref_row.variance + sat_row.variance;
                sd_row.satellite = sat_row.satellite;
                sd_row.reference_satellite = ref_row.satellite;
                sd_row.primary_signal = sat_row.primary_signal;
                sd_row.primary_observation_code = sat_row.primary_observation_code;
                sd_row.covariance_group = covariance_group;
                sd_row.reference_variance =
                    covariance_group >= 0 ? ref_row.variance : 0.0;
                if (group_key.is_phase) {
                    sd_row.phase_ambiguities = ref_row.phase_ambiguities;
                    sd_row.phase_ambiguities.insert(
                        sd_row.phase_ambiguities.end(),
                        sat_row.phase_ambiguities.begin(),
                        sat_row.phase_ambiguities.end());
                }
                sd_row.is_phase = group_key.is_phase;
                sd_row.freq_index = group_key.freq_index;
                sd_row.elevation_rad = sat_row.elevation_rad;
                sd_row.ionosphere_coefficient = sat_row.ionosphere_coefficient;
                sd_row.ionosphere_state_index = sat_row.ionosphere_state_index;
                sd_row.ionosphere_design_coeff =
                    rowCoefficient(sd_row.H, sat_row.ionosphere_state_index);
                sd_row.ionosphere_state_m = sat_row.ionosphere_state_m;
                sd_row.ambiguity_state_index = sat_row.ambiguity_state_index;
                sd_row.ambiguity_design_coeff =
                    rowCoefficient(sd_row.H, sat_row.ambiguity_state_index);
                if (ddres_dump && group_key.is_phase) {
                    const auto ref_debug_it =
                        phase_debug_by_measurement_index.find(ref_index);
                    const auto sat_debug_it =
                        phase_debug_by_measurement_index.find(idx);
                    if (ref_debug_it != phase_debug_by_measurement_index.end() &&
                        sat_debug_it != phase_debug_by_measurement_index.end()) {
                        const auto& ref_debug = ref_debug_it->second;
                        const auto& sat_debug = sat_debug_it->second;
                        const double sd_noamb_m =
                            ref_debug.residual_without_ambiguity_m -
                            sat_debug.residual_without_ambiguity_m;
                        const double sd_amb_term_m =
                            ref_debug.ambiguity_m - sat_debug.ambiguity_m;
                        const double ref_x_cycles =
                            std::isfinite(ref_debug.wavelength_m) &&
                                    std::abs(ref_debug.wavelength_m) > 0.0
                                ? ref_debug.ambiguity_m / ref_debug.wavelength_m
                                : std::numeric_limits<double>::quiet_NaN();
                        const double sat_x_cycles =
                            std::isfinite(sat_debug.wavelength_m) &&
                                    std::abs(sat_debug.wavelength_m) > 0.0
                                ? sat_debug.ambiguity_m / sat_debug.wavelength_m
                                : std::numeric_limits<double>::quiet_NaN();
                        const double h0 = sd_row.H.size() > 0
                            ? sd_row.H(0)
                            : std::numeric_limits<double>::quiet_NaN();
                        const double h1 = sd_row.H.size() > 1
                            ? sd_row.H(1)
                            : std::numeric_limits<double>::quiet_NaN();
                        const double h2 = sd_row.H.size() > 2
                            ? sd_row.H(2)
                            : std::numeric_limits<double>::quiet_NaN();
                        ddres_dump << "k=" << sd_debug_index++
                                    << " ref=" << ref_debug.satellite.toString()
                                    << " sat=" << sat_debug.satellite.toString()
                                    << " freq_index=" << group_key.freq_index
                                    << " freq_group="
                                    << sat_debug.ar_freq_group
                                    << " system="
                                    << static_cast<int>(group_key.system)
                                    << " ref_measurement_index=" << ref_index
                                    << " sat_measurement_index=" << idx
                                    << " ref_state_sat="
                                    << ref_debug.ambiguity_satellite.toString()
                                    << " sat_state_sat="
                                    << sat_debug.ambiguity_satellite.toString()
                                    << " ref_residual_noamb_m="
                                    << ref_debug.residual_without_ambiguity_m
                                    << " sat_residual_noamb_m="
                                    << sat_debug.residual_without_ambiguity_m
                                    << " sd_noamb_m=" << sd_noamb_m
                                    << " ref_ambiguity_m="
                                    << ref_debug.ambiguity_m
                                    << " sat_ambiguity_m="
                                    << sat_debug.ambiguity_m
                                    << " sd_ambiguity_term_m="
                                    << sd_amb_term_m
                                    << " ref_x_cycles=" << ref_x_cycles
                                    << " sat_x_cycles=" << sat_x_cycles
                                    << " sd_x_cycles="
                                    << (ref_x_cycles - sat_x_cycles)
                                    << " sd_residual_m=" << sd_row.residual
                                    << " variance_m2=" << sd_row.variance
                                    << " ref_variance_m2=" << ref_row.variance
                                    << " sat_variance_m2=" << sat_row.variance
                                    << " h_pos_x=" << h0
                                    << " h_pos_y=" << h1
                                    << " h_pos_z=" << h2
                                    << "\n";
                    }
                }
                differenced_measurements.push_back(sd_row);
            }
        }
        result.measurements = std::move(differenced_measurements);
    }

    return result;
}

KalmanUpdateStats applyMeasurementUpdate(
    ppp_shared::PPPState& filter_state,
    const std::vector<MeasurementRow>& measurements,
    const ppp_shared::PPPConfig& config,
    const PositionSolution* seed_solution,
    const GNSSTime* time) {
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

    double code_sum_sq = 0.0;
    double phase_sum_sq = 0.0;
    for (int i = 0; i < stats.nobs; ++i) {
        const auto& measurement = measurements[static_cast<size_t>(i)];
        const double abs_residual = std::abs(z(i));
        if (measurement.is_phase) {
            phase_sum_sq += z(i) * z(i);
            ++stats.phase_rows;
            if (abs_residual >= stats.phase_residual_max_abs_m) {
                stats.phase_residual_max_abs_m = abs_residual;
                stats.phase_residual_max_sat = measurement.satellite;
            }
        } else if (measurement.freq_index >= 0) {
            code_sum_sq += z(i) * z(i);
            ++stats.code_rows;
            if (abs_residual >= stats.code_residual_max_abs_m) {
                stats.code_residual_max_abs_m = abs_residual;
                stats.code_residual_max_sat = measurement.satellite;
            }
        } else if (measurement.satellite.prn != 0) {
            ++stats.ionosphere_constraint_rows;
        }
    }
    if (stats.code_rows > 0) {
        stats.code_residual_rms_m =
            std::sqrt(code_sum_sq / static_cast<double>(stats.code_rows));
    }
    if (stats.phase_rows > 0) {
        stats.phase_residual_rms_m =
            std::sqrt(phase_sum_sq / static_cast<double>(stats.phase_rows));
    }

    if (clasDdUpdateEnabled()) {
        for (int i = 0; i < stats.nobs; ++i) {
            const auto& mi = measurements[static_cast<size_t>(i)];
            if (mi.covariance_group < 0 || mi.reference_variance <= 0.0) {
                continue;
            }
            for (int j = 0; j < stats.nobs; ++j) {
                if (i == j) {
                    continue;
                }
                const auto& mj = measurements[static_cast<size_t>(j)];
                if (mj.covariance_group == mi.covariance_group) {
                    R(i, j) = mi.reference_variance;
                }
            }
        }
    }

    std::vector<bool> row_inflated(static_cast<size_t>(stats.nobs), false);
    double inflated_code_sum_sq = 0.0;
    double inflated_phase_sum_sq = 0.0;
    double inflated_ionosphere_constraint_sum_sq = 0.0;
    double inflated_prior_sum_sq = 0.0;
    const bool suppress_outlier_inflation =
        clasDdUpdateEnabled() && !clasDdOutlierInflationEnabled();
    for (int i = 0; i < stats.nobs; ++i) {
        const auto& measurement = measurements[static_cast<size_t>(i)];
        double outlier_sigma_scale = config.clas_outlier_sigma_scale;
        if (measurement.is_phase && config.clas_phase_outlier_sigma_scale >= 0.0) {
            outlier_sigma_scale = config.clas_phase_outlier_sigma_scale;
        } else if (!measurement.is_phase &&
                   measurement.freq_index >= 0 &&
                   config.clas_code_outlier_sigma_scale >= 0.0) {
            outlier_sigma_scale = config.clas_code_outlier_sigma_scale;
        } else if (!measurement.is_phase &&
                   measurement.freq_index < 0 &&
                   measurement.satellite.prn == 0 &&
                   config.clas_prior_outlier_sigma_scale >= 0.0) {
            outlier_sigma_scale = config.clas_prior_outlier_sigma_scale;
        }
        const double sigma = std::sqrt(R(i, i));
        const bool code_row = !measurement.is_phase && measurement.freq_index >= 0;
        const bool phase_row = measurement.is_phase;
        const bool prior_row =
            !measurement.is_phase &&
            measurement.freq_index < 0 &&
            measurement.satellite.prn == 0;
        const bool passes_code_residual_floor =
            !code_row ||
            config.clas_code_outlier_min_residual_m <= 0.0 ||
            std::abs(z(i)) >= config.clas_code_outlier_min_residual_m;
        const bool passes_phase_residual_floor =
            !phase_row ||
            config.clas_phase_outlier_min_residual_m <= 0.0 ||
            std::abs(z(i)) >= config.clas_phase_outlier_min_residual_m;
        const bool passes_prior_residual_floor =
            !prior_row ||
            config.clas_prior_outlier_min_residual_m <= 0.0 ||
            std::abs(z(i)) >= config.clas_prior_outlier_min_residual_m;
        if (!suppress_outlier_inflation &&
            outlier_sigma_scale > 0.0 &&
            passes_code_residual_floor &&
            passes_phase_residual_floor &&
            passes_prior_residual_floor &&
            std::abs(z(i)) > outlier_sigma_scale * sigma) {
            const double abs_residual = std::abs(z(i));
            R(i, i) =
                measurement.is_phase &&
                config.clas_phase_outlier_inflated_variance_m2 > 0.0
                    ? config.clas_phase_outlier_inflated_variance_m2
                    : 1e10;
            row_inflated[static_cast<size_t>(i)] = true;
            ++stats.outlier_inflated_row_count;
            if (measurement.is_phase) {
                ++stats.phase_outlier_inflated_row_count;
                inflated_phase_sum_sq += abs_residual * abs_residual;
                stats.phase_outlier_inflated_max_abs_m =
                    std::max(stats.phase_outlier_inflated_max_abs_m, abs_residual);
                if (abs_residual >=
                    config.clas_reset_phase_ambiguity_outlier_min_residual_m) {
                    stats.phase_outlier_inflated_ambiguities.insert(
                        measurement.phase_ambiguities.begin(),
                        measurement.phase_ambiguities.end());
                }
            } else if (measurement.freq_index >= 0) {
                ++stats.code_outlier_inflated_row_count;
                inflated_code_sum_sq += abs_residual * abs_residual;
                stats.code_outlier_inflated_max_abs_m =
                    std::max(stats.code_outlier_inflated_max_abs_m, abs_residual);
            } else if (measurement.satellite.prn != 0) {
                ++stats.ionosphere_constraint_outlier_inflated_row_count;
                inflated_ionosphere_constraint_sum_sq += abs_residual * abs_residual;
                stats.ionosphere_constraint_outlier_inflated_max_abs_m =
                    std::max(
                        stats.ionosphere_constraint_outlier_inflated_max_abs_m,
                        abs_residual);
            } else {
                ++stats.prior_outlier_inflated_row_count;
                inflated_prior_sum_sq += abs_residual * abs_residual;
                stats.prior_outlier_inflated_max_abs_m =
                    std::max(stats.prior_outlier_inflated_max_abs_m, abs_residual);
            }
        }
    }
    if (stats.code_outlier_inflated_row_count > 0) {
        stats.code_outlier_inflated_rms_m =
            std::sqrt(inflated_code_sum_sq /
                      static_cast<double>(stats.code_outlier_inflated_row_count));
    }
    if (stats.phase_outlier_inflated_row_count > 0) {
        stats.phase_outlier_inflated_rms_m =
            std::sqrt(inflated_phase_sum_sq /
                      static_cast<double>(stats.phase_outlier_inflated_row_count));
    }
    if (stats.ionosphere_constraint_outlier_inflated_row_count > 0) {
        stats.ionosphere_constraint_outlier_inflated_rms_m =
            std::sqrt(
                inflated_ionosphere_constraint_sum_sq /
                static_cast<double>(stats.ionosphere_constraint_outlier_inflated_row_count));
    }
    if (stats.prior_outlier_inflated_row_count > 0) {
        stats.prior_outlier_inflated_rms_m =
            std::sqrt(inflated_prior_sum_sq /
                      static_cast<double>(stats.prior_outlier_inflated_row_count));
    }

    for (int i = 0; i < stats.nobs; ++i) {
        const auto& measurement = measurements[static_cast<size_t>(i)];
        if (!measurement.is_phase || row_inflated[static_cast<size_t>(i)]) {
            continue;
        }
        for (const auto& ambiguity_satellite : measurement.phase_ambiguities) {
            stats.active_phase_ambiguities.insert(ambiguity_satellite);
        }
    }

    const VectorXd state_before = filter_state.state;
    MatrixXd S = H * filter_state.covariance * H.transpose() + R;
    MatrixXd K = filter_state.covariance * H.transpose() * S.inverse();
    VectorXd dx = K * z;
    filter_state.state += dx;
    const VectorXd state_after_kf = filter_state.state;
    MatrixXd I_KH =
        MatrixXd::Identity(filter_state.total_states, filter_state.total_states) - K * H;
    filter_state.covariance =
        I_KH * filter_state.covariance * I_KH.transpose() + K * R * K.transpose();

    stats.updated = true;
    stats.dx = dx;
    stats.residuals = z;
    stats.variances = R.diagonal();
    stats.outlier_inflated_rows = std::move(row_inflated);
    stats.pre_anchor_covariance = filter_state.covariance;

    if (seed_solution != nullptr && seed_solution->isValid()) {
        const double anchor_sigma = config.clas_anchor_sigma;
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
    if (clasUpdateRowDumpEnabled(time)) {
        appendClasUpdateRowDump(
            *time,
            filter_state,
            measurements,
            state_before,
            state_after_kf,
            filter_state.state,
            K,
            z,
            R,
            row_inflated,
            seed_solution);
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
    result.measurements = measurement_build_result.measurements;

    result.update_stats = applyMeasurementUpdate(
        filter_state, result.measurements, config, &seed_solution, &obs.time);
    if (!result.update_stats.updated) {
        return result;
    }

    updateObservedAmbiguities(
        obs.time,
        measurement_build_result.observed_ambiguities,
        filter_state,
        ambiguity_states,
        ambiguity_index_function);
    if (config.clas_reset_phase_ambiguity_on_outlier_inflation &&
        result.update_stats.phase_outlier_inflated_row_count >=
            config.clas_reset_phase_ambiguity_outlier_min_rows) {
        for (const auto& ambiguity_satellite :
             result.update_stats.phase_outlier_inflated_ambiguities) {
            if (static_cast<bool>(ambiguity_reset_function)) {
                ambiguity_reset_function(
                    ambiguity_satellite,
                    SignalType::SIGNAL_TYPE_COUNT);
            }
        }
    }
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
            const auto applied_corrections = selectAppliedOsrCorrections(
                osr, f, config.clas_correction_application_policy);

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

            const double el_weight = elevationWeight(osr.elevation);
            const double predicted =
                geo - sat_clk_m + receiver_clock_m + trop_modeled;

            if (raw->has_pseudorange && std::isfinite(raw->pseudorange)) {
                const double residual =
                    (raw->pseudorange - applied_corrections.pseudorange_correction_m) -
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
                (carrier_phase_m - applied_corrections.carrier_phase_correction_m) -
                (predicted - iono_scale * iono_state_m +
                 filter_state.state(ambiguity_index));
            const double variance = config.clas_phase_variance * el_weight;
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
        constexpr double kPairValidationScale = 64.0;  // 8-sigma squared
        if (dispersive * dispersive > kPairValidationScale * max_variance ||
            nondispersive * nondispersive > kPairValidationScale * max_variance) {
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

    constexpr double kMaxPhaseSigma = 4.0;
    constexpr double kMaxPhaseChisq = 5.0;
    stats.accepted =
        stats.phase_rows > position_dof &&
        stats.max_phase_sigma < kMaxPhaseSigma &&
        pair_validation_ok &&
        stats.phase_chisq < kMaxPhaseChisq;
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
    constexpr double kCodePhaseVarianceBoundary = 0.5;
    for (int i = 0; i < update_stats.nobs; ++i) {
        if (update_stats.variances(i) > kCodePhaseVarianceBoundary) {
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
    const GNSSTime& time,
    bool fixed,
    double ar_ratio,
    int fixed_ambiguities,
    int num_satellites) {
    PositionSolution solution;
    solution.time = time;
    solution.position_ecef = filter_state.state.segment(0, 3);
    solution.receiver_clock_bias =
        filter_state.state(filter_state.clock_index) / constants::SPEED_OF_LIGHT;
    solution.receiver_clock_biases_m[ReceiverClockBiasGroup::GPS] =
        filter_state.state(filter_state.clock_index);
    solution.receiver_clock_biases_m[ReceiverClockBiasGroup::GLONASS] =
        filter_state.state(filter_state.glo_clock_index);
    if (filter_state.gal_clock_index >= 0) {
        solution.receiver_clock_biases_m[ReceiverClockBiasGroup::Galileo] =
            filter_state.state(filter_state.gal_clock_index);
    }
    if (filter_state.qzs_clock_index >= 0) {
        solution.receiver_clock_biases_m[ReceiverClockBiasGroup::QZSS] =
            filter_state.state(filter_state.qzs_clock_index);
    }
    if (filter_state.bds_clock_index >= 0) {
        solution.receiver_clock_biases_m[ReceiverClockBiasGroup::BeiDou] =
            filter_state.state(filter_state.bds_clock_index);
    }
    if (filter_state.bds2_clock_index >= 0) {
        solution.receiver_clock_biases_m[ReceiverClockBiasGroup::BeiDou2] =
            filter_state.state(filter_state.bds2_clock_index);
    }
    if (filter_state.bds3_clock_index >= 0) {
        solution.receiver_clock_biases_m[ReceiverClockBiasGroup::BeiDou3] =
            filter_state.state(filter_state.bds3_clock_index);
    }
    solution.position_covariance =
        filter_state.covariance.block(filter_state.pos_index, filter_state.pos_index, 3, 3);
    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    ecef2geodetic(solution.position_ecef, latitude, longitude, height);
    solution.position_geodetic = GeodeticCoord(latitude, longitude, height);
    solution.status = fixed ? SolutionStatus::PPP_FIXED : SolutionStatus::PPP_FLOAT;
    solution.ratio = fixed ? ar_ratio : 0.0;
    solution.num_fixed_ambiguities = fixed ? fixed_ambiguities : 0;
    solution.num_satellites = num_satellites;
    return solution;
}

}  // namespace libgnss::ppp_clas
