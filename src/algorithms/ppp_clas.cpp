#include <libgnss++/algorithms/ppp_clas.hpp>

#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_bias_identity.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include "ppp_internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
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
    // Sparse measurement partials for the MRTKLIB innovation-variance basis
    // (h-row entries; the receiver clock is omitted because it cancels in
    // the same-system DDs where these are consumed).
    Vector3d unit_vector = Vector3d::Zero();  ///< d(range)/d(receiver pos)
    double trop_mapping = 0.0;
    int iono_index = -1;
    double iono_scale = 0.0;  ///< mu_f = (lambda_f/lambda_1)^2, phase sign applied at use
    int ambiguity_index = -1;
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

double elevationWeight(double elevation_rad) {
    const double s = std::sin(elevation_rad);
    return 1.0 / (s * s);
}

// ---------------------------------------------------------------------------
// MRTKLIB varerr() parity (mrtk_ppp_rtk.c:322) with the CLAS benchmark
// constants from conf/benchmark/clas.toml [kalman_filter.measurement_error]:
//   phase = 0.01 m (err[1]), phase_elevation = 0.005 m (err[2]),
//   code_phase_ratio_L1 = 50 (eratio[0]);
//   var = a^2 + b^2 / sin^2(el),  code rows multiply a,b by eratio;
//   GPS/QZS L2-slot phase rows get x(2.55/1.55)^2 (mrtk_ppp_rtk.c:1633);
//   EFACT is 1.0 for GPS/Galileo/QZSS (mrtk_const.h) so it is omitted.
// Only active on the dynamics-model kinematic CLAS path (the MRTKLIB
// equivalence track); white-noise/static keep the historical flat
// clas_code_variance_scale / clas_phase_variance / (1/sin^2 el) model.
constexpr double kMrtklibPhaseErrConstM = 0.01;
constexpr double kMrtklibPhaseErrElevM = 0.005;
constexpr double kMrtklibCodePhaseRatio = 50.0;
constexpr double kMrtklibL2PhaseVarFactor = (2.55 / 1.55) * (2.55 / 1.55);
constexpr double kL2FrequencyHz = 1227.60e6;

bool clasMrtklibFloatParity(const ppp_shared::PPPConfig& config) {
    return config.clas_mrtklib_float_parity &&
           config.kinematic_mode && !config.low_dynamics_mode &&
           config.use_clas_osr_filter && config.use_dynamics_model;
}

bool isMrtklibL2Slot(int freq_index, double frequency_hz) {
    return freq_index > 0 && std::abs(frequency_hz - kL2FrequencyHz) < 1.0e6;
}

double clasPhaseVariance(const ppp_shared::PPPConfig& config,
                         double elevation_rad,
                         int freq_index,
                         double frequency_hz) {
    if (!clasMrtklibFloatParity(config)) {
        return config.clas_phase_variance * elevationWeight(elevation_rad);
    }
    const double s = std::sin(elevation_rad);
    double var = kMrtklibPhaseErrConstM * kMrtklibPhaseErrConstM +
                 (kMrtklibPhaseErrElevM * kMrtklibPhaseErrElevM) / (s * s);
    if (isMrtklibL2Slot(freq_index, frequency_hz)) {
        var *= kMrtklibL2PhaseVarFactor;
    }
    return var;
}

double clasCodeVariance(const ppp_shared::PPPConfig& config,
                        double elevation_rad) {
    if (!clasMrtklibFloatParity(config)) {
        return config.clas_code_variance_scale * elevationWeight(elevation_rad);
    }
    const double s = std::sin(elevation_rad);
    const double a = kMrtklibCodePhaseRatio * kMrtklibPhaseErrConstM;
    const double b = kMrtklibCodePhaseRatio * kMrtklibPhaseErrElevM;
    return a * a + (b * b) / (s * s);
}

int receiverClockStateIndex(const ppp_shared::PPPState& filter_state,
                            const SatelliteId& satellite) {
    switch (satellite.system) {
        case GNSSSystem::GLONASS:
            return filter_state.glo_clock_index;
        case GNSSSystem::Galileo:
            return filter_state.gal_clock_index >= 0
                ? filter_state.gal_clock_index : filter_state.clock_index;
        case GNSSSystem::QZSS:
            return filter_state.qzs_clock_index >= 0
                ? filter_state.qzs_clock_index : filter_state.clock_index;
        case GNSSSystem::BeiDou:
            return filter_state.bds_clock_index >= 0
                ? filter_state.bds_clock_index : filter_state.clock_index;
        default:
            return filter_state.clock_index;
    }
}

double receiverClockBiasMeters(const ppp_shared::PPPState& filter_state,
                               const SatelliteId& satellite) {
    const int clock_index = receiverClockStateIndex(filter_state, satellite);
    return filter_state.state(clock_index);
}

bool usesClasStecConstraint(const ppp_shared::PPPConfig& config) {
    return pppEnvOverrides().clas_stec_constraint &&
           config.estimate_ionosphere &&
           config.use_clas_osr_filter;
}

double cssrStecQualityStdTecu(int quality) {
    const int cls = (quality >> 3) & 0x7;
    const int val = quality & 0x7;
    if ((cls == 0 && val == 0) || (cls == 7 && val == 7)) {
        return std::numeric_limits<double>::infinity();
    }
    return (1.0 + 0.25 * static_cast<double>(val)) *
               std::pow(3.0, static_cast<double>(cls)) -
           1.0;
}

double tecuStdToDelayVarianceM2(double sigma_tecu, double frequency_hz) {
    if (!std::isfinite(sigma_tecu) || sigma_tecu < 0.0 || frequency_hz <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    constexpr double kTecuToElectrons = 1e16;
    const double sigma_m =
        40.3 * kTecuToElectrons * sigma_tecu / (frequency_hz * frequency_hz);
    return sigma_m * sigma_m;
}

double clasStecConstraintVariance(
    const std::map<std::string, std::string>& epoch_atmos,
    const OSRCorrection& osr,
    const ppp_shared::PPPConfig& config) {
    constexpr double kClaslibStdIonoM = 0.010;
    constexpr double kClaslibMinVarianceM2 = kClaslibStdIonoM * kClaslibStdIonoM;
    const std::string key = "atmos_stec_quality:" + osr.satellite.toString();
    const auto quality_it = epoch_atmos.find(key);
    if (quality_it != epoch_atmos.end()) {
        const int quality = std::atoi(quality_it->second.c_str());
        const double quality_variance =
            tecuStdToDelayVarianceM2(
                cssrStecQualityStdTecu(quality),
                osr.frequencies[0]);
        if (std::isfinite(quality_variance) && quality_variance > 0.0) {
            return std::max(kClaslibMinVarianceM2, quality_variance);
        }
    }
    if (config.clas_iono_prior_variance > 0.0) {
        return std::min(config.clas_iono_prior_variance, kClaslibMinVarianceM2);
    }
    return kClaslibMinVarianceM2;
}

double effectiveClasIonoProcessNoise(const ppp_shared::PPPConfig& config) {
    if (usesClasStecConstraint(config)) {
        constexpr double kClaslibPrnIonoStdM = 0.001;
        return kClaslibPrnIonoStdM * kClaslibPrnIonoStdM;
    }
    return config.process_noise_ionosphere;
}

std::ofstream* ambDatumDumpStream() {
    const auto& path = pppEnvOverrides().clas_amb_datum_dump_path;
    if (path.empty()) {
        return nullptr;
    }

    static std::ofstream stream;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        stream.open(path, std::ios::out | std::ios::trunc);
        if (stream) {
            stream << "record,week,tow,sat,freq,signal,carrier_rinex_code,"
                   << "carrier_rtklib_code,signal_family,"
                   << "requested_pseudorange_rinex_code,"
                   << "requested_carrier_rinex_code,"
                   << "observation_exact_identity_requested,"
                   << "observation_exact_match,observation_family_fallback,"
                   << "phase_bias_signal_id,bias_exact_identity,"
                   << "phase_bias_source_signal_id,phase_bias_present,"
                   << "phase_bias_fallback,lambda_m,"
                   << "raw_phase_cycles,raw_phase_m,carrier_correction_m,"
                   << "cpc_m,cpc_minus_trop_m,trop_correction_m,relativity_m,"
                   << "receiver_antenna_m,iono_cpc_m,phase_bias_m,windup_m,"
                   << "phase_compensation_m,l_corr_m,predicted_no_amb_m,"
                   << "amb_state_m,amb_state_cycles,residual_m\n";
        }
    }
    return stream ? &stream : nullptr;
}

std::ofstream* clasGeometryDumpStream() {
    const auto& path = pppEnvOverrides().clas_geom_dump_path;
    if (path.empty()) {
        return nullptr;
    }

    static std::ofstream stream;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        stream.open(path, std::ios::out | std::ios::trunc);
        if (stream) {
            stream << "record,week,tow,tx_tow,sat,freq,signal,"
                   << "rx_state_x_m,rx_state_y_m,rx_state_z_m,"
                   << "rx_row_x_m,rx_row_y_m,rx_row_z_m,"
                   << "rx_forced_x_m,rx_forced_y_m,rx_forced_z_m,"
                   << "tide_x_m,tide_y_m,tide_z_m,"
                   << "sat_x_m,sat_y_m,sat_z_m,sat_vx_mps,sat_vy_mps,sat_vz_mps,"
                   << "sat_clk_m,euclidean_m,sagnac_m,rho_m,az_rad,el_rad,"
                   << "cpc_m,model_phase_m\n";
        }
    }
    return stream ? &stream : nullptr;
}

bool clasPhaseRowDumpEnabled() {
    return pppEnvOverrides().clas_phase_row_dump;
}

const char* clasCodeDumpPhaseExtensionHeader() {
    return clasPhaseRowDumpEnabled()
        ? ",row_type,cpc_m,carrier_correction_m,cpc_minus_trop_m,phase_bias_m,"
          "windup_m,phase_compensation_m,raw_l_m,corrected_l_m"
        : "";
}

void writeClasCodeDumpCodePhaseExtension(std::ostream& out) {
    if (!clasPhaseRowDumpEnabled()) {
        return;
    }
    out << ",code,,,,,,,,";
}

void writeClasCodeDumpPhasePhaseExtension(
    std::ostream& out,
    double cpc_m,
    double carrier_correction_m,
    double cpc_minus_trop_m,
    double phase_bias_m,
    double windup_m,
    double phase_compensation_m,
    double raw_l_m,
    double corrected_l_m) {
    if (!clasPhaseRowDumpEnabled()) {
        return;
    }
    out << ",phase," << cpc_m << ',' << carrier_correction_m << ','
        << cpc_minus_trop_m << ',' << phase_bias_m << ',' << windup_m << ','
        << phase_compensation_m << ',' << raw_l_m << ',' << corrected_l_m;
}

std::ofstream* clasCodeDumpStream() {
    const auto& path = pppEnvOverrides().clas_code_dump_path;
    if (path.empty()) {
        return nullptr;
    }

    static std::ofstream stream;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        stream.open(path, std::ios::out | std::ios::trunc);
        if (stream) {
            stream << "record,stage,week,tow,sat,freq,signal,"
                   << "pseudorange_rinex_code,carrier_rinex_code,"
                   << "pseudorange_rtklib_code,carrier_rtklib_code,signal_family,"
                   << "requested_pseudorange_rinex_code,"
                   << "requested_carrier_rinex_code,"
                   << "observation_exact_identity_requested,"
                   << "observation_exact_match,observation_family_fallback,"
                   << "code_bias_signal_id,phase_bias_signal_id,"
                   << "bias_exact_identity,code_bias_source_signal_id,"
                   << "phase_bias_source_signal_id,code_bias_present,"
                   << "phase_bias_present,code_bias_fallback,phase_bias_fallback,"
                   << "raw_p_m,corrected_p_m,applied_pr_corr_m,prc_m,"
                   << "prc_minus_trop_m,trop_correction_m,iono_l1_m,"
                   << "stec_tecu,iono_scaled_m,code_bias_m,network_compensation_m,"
                   << "receiver_ant_m,relativity_m,"
                   << "atmos_ref_week,atmos_ref_tow,clock_ref_week,clock_ref_tow,"
                   << "code_bias_ref_week,code_bias_ref_tow,"
                   << "atmos_clock_gap_s,atmos_network_id,atmos_grid_no,"
                   << "atmos_grid_distance_m,atmos_grid_count,"
                   << "atmos_grid1_no,atmos_grid1_weight,"
                   << "atmos_grid2_no,atmos_grid2_weight,"
                   << "atmos_grid3_no,atmos_grid3_weight,"
                   << "atmos_grid4_no,atmos_grid4_weight,"
                   << "atmos_lifecycle,atmos_lifecycle_tow,"
                   << "atmos_selected_satellite_count,"
                   << "atmos_valid_grid_count,atmos_stec_grid_value_count,"
                   << "atmos_selected_grid_stec_value_count,"
                   << "geo_m,sat_clk_m,receiver_clock_m,trop_model_m,"
                   << "iono_state_m,iono_scale,predicted_m,residual_m,"
                   << "variance_m2,los_e_m,los_n_m,los_u_m,az_rad,el_rad,"
                   << "rx_x_m,rx_y_m,rx_z_m"
                   << clasCodeDumpPhaseExtensionHeader() << '\n';
        }
    }
    return stream ? &stream : nullptr;
}

bool selectedClasGeometryDumpTow(double tow) {
    constexpr std::array<double, 3> kTargets{230572.0, 232034.0, 234018.0};
    for (double target : kTargets) {
        if (std::abs(tow - target) < 0.01) {
            return true;
        }
    }
    return false;
}

Vector3d clasGeometryDumpReceiverPosition(const Vector3d& fallback) {
    const auto& spec = pppEnvOverrides().clas_geom_dump_rx_xyz;
    if (spec.empty()) {
        return fallback;
    }

    std::string normalized = spec;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');
    std::istringstream input(normalized);
    Vector3d parsed = fallback;
    if (input >> parsed.x() >> parsed.y() >> parsed.z()) {
        return parsed;
    }
    return fallback;
}

bool usesClaslibCodePrcRows(const ppp_shared::PPPConfig& config) {
    return pppEnvOverrides().clas_code_row_full_prc &&
           config.clas_correction_application_policy ==
               ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
}

void dumpClasCodeRows(
    const char* stage,
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function) {
    auto* dump = clasCodeDumpStream();
    if (dump == nullptr) {
        return;
    }
    if (filter_state.total_states <= filter_state.trop_index ||
        filter_state.state.size() < filter_state.total_states) {
        return;
    }

    const Vector3d receiver_position =
        filter_state.state.segment(filter_state.pos_index, 3);
    const double trop_zenith = filter_state.state(filter_state.trop_index);
    double rx_lat = 0.0;
    double rx_lon = 0.0;
    double rx_h = 0.0;
    ecef2geodetic(receiver_position, rx_lat, rx_lon, rx_h);

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        const double geo = geodist(osr.satellite_position, receiver_position);
        const Vector3d range_vector = osr.satellite_position - receiver_position;
        if (!std::isfinite(geo) || range_vector.norm() <= 0.0) {
            continue;
        }
        const Vector3d los = range_vector.normalized();
        const Vector3d los_enu = ecef2enu(los, rx_lat, rx_lon);
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double receiver_clock_m =
            receiverClockBiasMeters(filter_state, osr.satellite);
        const double trop_mapping =
            trop_mapping_function(receiver_position, osr.elevation, obs.time);
        const double trop_model = trop_mapping * trop_zenith;
        const bool claslib_code_prc = usesClaslibCodePrcRows(config);
        const double code_trop_model = claslib_code_prc ? 0.0 : trop_model;
        const auto iono_state_it = filter_state.ionosphere_indices.find(osr.satellite);
        const bool have_iono_state =
            config.estimate_ionosphere &&
            iono_state_it != filter_state.ionosphere_indices.end() &&
            iono_state_it->second >= 0 &&
            iono_state_it->second < filter_state.total_states;
        const double iono_state_l1_m =
            have_iono_state ? filter_state.state(iono_state_it->second) : 0.0;

        for (int f = 0; f < osr.num_frequencies; ++f) {
            const auto lookup = findOsrFrequencyObservationWithProvenance(obs, osr, f);
            const Observation* raw = lookup.observation;
            if (raw == nullptr || !raw->valid || !raw->has_pseudorange ||
                !std::isfinite(raw->pseudorange)) {
                continue;
            }
            const auto applied = selectAppliedOsrCorrections(
                osr, f, config.clas_correction_application_policy);
            const double corrected_p =
                raw->pseudorange - applied.pseudorange_correction_m;
            const double iono_scale =
                (osr.frequencies[f] > 0.0 && osr.wavelengths[0] > 0.0)
                    ? std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2)
                    : 1.0;
            const double iono_scaled = iono_scale * osr.iono_l1_m;
            const double predicted =
                geo - sat_clk_m + receiver_clock_m + code_trop_model +
                iono_scale * iono_state_l1_m;
            const double residual = corrected_p - predicted;
            const double variance = clasCodeVariance(config, osr.elevation);
            const bool have_atmos_ref =
                osr.atmos_reference_time.week != 0 ||
                std::abs(osr.atmos_reference_time.tow) > 0.0;
            const bool have_clock_ref =
                osr.clock_reference_time.week != 0 ||
                std::abs(osr.clock_reference_time.tow) > 0.0;
            const bool have_code_bias_ref =
                osr.code_bias_reference_time.week != 0 ||
                std::abs(osr.code_bias_reference_time.tow) > 0.0;
            const double atmos_clock_gap_s =
                (have_atmos_ref && have_clock_ref)
                    ? osr.atmos_reference_time - osr.clock_reference_time
                    : 0.0;
            auto timeWeekField = [](const GNSSTime& time, bool have_time) {
                return have_time ? std::to_string(time.week) : std::string();
            };
            auto timeTowField = [](const GNSSTime& time, bool have_time) {
                if (!have_time) {
                    return std::string();
                }
                std::ostringstream stream;
                stream << std::setprecision(17) << time.tow;
                return stream.str();
            };
            auto secondsField = [](double value, bool have_value) {
                if (!have_value) {
                    return std::string();
                }
                std::ostringstream stream;
                stream << std::setprecision(17) << value;
                return stream.str();
            };
            auto doubleField = [](double value, bool have_value) {
                if (!have_value) {
                    return std::string();
                }
                std::ostringstream stream;
                stream << std::setprecision(17) << value;
                return stream.str();
            };

            *dump << std::setprecision(17)
                  << "CODE,"
                  << stage << ','
                  << obs.time.week << ','
                  << obs.time.tow << ','
                  << osr.satellite.toString() << ','
                  << f << ','
                  << static_cast<int>(raw->signal) << ','
                  << raw->pseudorange_observation_type << ','
                  << raw->carrier_phase_observation_type << ','
                  << algorithms::ppp_bias_identity::rtklibCodeForObservationType(
                         raw->pseudorange_observation_type) << ','
                  << algorithms::ppp_bias_identity::rtklibCodeForObservationType(
                         raw->carrier_phase_observation_type) << ','
                  << ppp_internal::signalFamilyName(raw->signal) << ','
                  << osr.pseudorange_rinex_codes[f] << ','
                  << osr.carrier_rinex_codes[f] << ','
                  << (lookup.exact_identity_requested ? 1 : 0) << ','
                  << (lookup.exact_identity_matched ? 1 : 0) << ','
                  << (lookup.family_fallback ? 1 : 0) << ','
                  << static_cast<int>(osr.code_bias_signal_ids[f]) << ','
                  << static_cast<int>(osr.phase_bias_signal_ids[f]) << ','
                  << (osr.bias_exact_identity[f] ? 1 : 0) << ','
                  << static_cast<int>(osr.code_bias_source_signal_ids[f]) << ','
                  << static_cast<int>(osr.phase_bias_source_signal_ids[f]) << ','
                  << (osr.code_bias_present[f] ? 1 : 0) << ','
                  << (osr.phase_bias_present[f] ? 1 : 0) << ','
                  << (osr.code_bias_fallback[f] ? 1 : 0) << ','
                  << (osr.phase_bias_fallback[f] ? 1 : 0) << ','
                  << raw->pseudorange << ','
                  << corrected_p << ','
                  << applied.pseudorange_correction_m << ','
                  << osr.PRC[f] << ','
                  << (osr.PRC[f] - osr.trop_correction_m) << ','
                  << osr.trop_correction_m << ','
                  << osr.iono_l1_m << ','
                  << osr.stec_tecu << ','
                  << iono_scaled << ','
                  << osr.code_bias_m[f] << ','
                  << osr.network_compensation_m << ','
                  << osr.receiver_antenna_m[f] << ','
                  << osr.relativity_correction_m << ','
                  << timeWeekField(osr.atmos_reference_time, have_atmos_ref) << ','
                  << timeTowField(osr.atmos_reference_time, have_atmos_ref) << ','
                  << timeWeekField(osr.clock_reference_time, have_clock_ref) << ','
                  << timeTowField(osr.clock_reference_time, have_clock_ref) << ','
                  << timeWeekField(osr.code_bias_reference_time, have_code_bias_ref) << ','
                  << timeTowField(osr.code_bias_reference_time, have_code_bias_ref) << ','
                  << secondsField(atmos_clock_gap_s, have_atmos_ref && have_clock_ref) << ','
                  << osr.atmos_network_id << ','
                  << osr.atmos_grid_no << ','
                  << osr.atmos_nearest_grid_distance_m << ','
                  << osr.atmos_interpolation_grid_count << ','
                  << osr.atmos_interpolation_grid_no[0] << ','
                  << osr.atmos_interpolation_weights[0] << ','
                  << osr.atmos_interpolation_grid_no[1] << ','
                  << osr.atmos_interpolation_weights[1] << ','
                  << osr.atmos_interpolation_grid_no[2] << ','
                  << osr.atmos_interpolation_weights[2] << ','
                  << osr.atmos_interpolation_grid_no[3] << ','
                  << osr.atmos_interpolation_weights[3] << ','
                  << (osr.atmos_lifecycle ? 1 : 0) << ','
                  << doubleField(
                         osr.atmos_lifecycle_tow,
                         osr.has_atmos_lifecycle_tow) << ','
                  << osr.atmos_selected_satellite_count << ','
                  << osr.atmos_valid_grid_count << ','
                  << osr.atmos_stec_grid_value_count << ','
                  << osr.atmos_selected_grid_stec_value_count << ','
                  << geo << ','
                  << sat_clk_m << ','
                  << receiver_clock_m << ','
                  << code_trop_model << ','
                  << iono_state_l1_m << ','
                  << iono_scale << ','
                  << predicted << ','
                  << residual << ','
                  << variance << ','
                  << los_enu.x() << ','
                  << los_enu.y() << ','
                  << los_enu.z() << ','
                  << osr.azimuth << ','
                  << osr.elevation << ','
                  << receiver_position.x() << ','
                  << receiver_position.y() << ','
                  << receiver_position.z();
            writeClasCodeDumpCodePhaseExtension(*dump);
            *dump << '\n';
        }
    }
}

void dumpClasPhaseRows(
    const char* stage,
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function) {
    if (!clasPhaseRowDumpEnabled()) {
        return;
    }
    auto* dump = clasCodeDumpStream();
    if (dump == nullptr) {
        return;
    }
    if (filter_state.total_states <= filter_state.trop_index ||
        filter_state.state.size() < filter_state.total_states) {
        return;
    }

    const Vector3d receiver_position =
        filter_state.state.segment(filter_state.pos_index, 3);
    const double trop_zenith = filter_state.state(filter_state.trop_index);
    double rx_lat = 0.0;
    double rx_lon = 0.0;
    double rx_h = 0.0;
    ecef2geodetic(receiver_position, rx_lat, rx_lon, rx_h);

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        const double geo = geodist(osr.satellite_position, receiver_position);
        const Vector3d range_vector = osr.satellite_position - receiver_position;
        if (!std::isfinite(geo) || range_vector.norm() <= 0.0) {
            continue;
        }
        const Vector3d los = range_vector.normalized();
        const Vector3d los_enu = ecef2enu(los, rx_lat, rx_lon);
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double receiver_clock_m =
            receiverClockBiasMeters(filter_state, osr.satellite);
        const double trop_mapping =
            trop_mapping_function(receiver_position, osr.elevation, obs.time);
        const double trop_model = trop_mapping * trop_zenith;
        const auto iono_state_it = filter_state.ionosphere_indices.find(osr.satellite);
        const bool have_iono_state =
            config.estimate_ionosphere &&
            iono_state_it != filter_state.ionosphere_indices.end() &&
            iono_state_it->second >= 0 &&
            iono_state_it->second < filter_state.total_states;
        const double iono_state_l1_m =
            have_iono_state ? filter_state.state(iono_state_it->second) : 0.0;

        for (int f = 0; f < osr.num_frequencies; ++f) {
            const auto lookup = findOsrFrequencyObservationWithProvenance(obs, osr, f);
            const Observation* raw = lookup.observation;
            if (raw == nullptr || !raw->valid || !raw->has_carrier_phase ||
                !std::isfinite(raw->carrier_phase)) {
                continue;
            }
            const auto applied = selectAppliedOsrCorrections(
                osr, f, config.clas_correction_application_policy);
            const double l_m = raw->carrier_phase * osr.wavelengths[f];
            const double l_corr = l_m - applied.carrier_phase_correction_m;
            const double iono_scale =
                (osr.frequencies[f] > 0.0 && osr.wavelengths[0] > 0.0)
                    ? std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2)
                    : 1.0;
            const double iono_scaled = iono_scale * osr.iono_l1_m;
            const auto& env = pppEnvOverrides();
            const bool claslib_amb_datum_phase =
                env.clas_amb_datum &&
                !env.clas_amb_datum_residual_phase_trop &&
                config.clas_correction_application_policy ==
                    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
            const bool residual_amb_datum_phase_trop =
                env.clas_amb_datum &&
                env.clas_amb_datum_residual_phase_trop &&
                config.clas_correction_application_policy ==
                    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
            const double phase_trop_model = claslib_amb_datum_phase
                ? 0.0
                : (residual_amb_datum_phase_trop
                      ? trop_model - osr.trop_correction_m
                      : trop_model);
            const uint8_t amb_prn = f == 0 ? osr.satellite.prn
                : static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
            const SatelliteId amb_sat(osr.satellite.system, amb_prn);
            const auto amb_it = filter_state.ambiguity_indices.find(amb_sat);
            if (amb_it == filter_state.ambiguity_indices.end()) {
                continue;
            }
            const int amb_idx = amb_it->second;
            const double predicted_no_amb =
                geo - sat_clk_m + receiver_clock_m + phase_trop_model
                - iono_scale * iono_state_l1_m;
            const double predicted =
                predicted_no_amb + filter_state.state(amb_idx);
            const double residual = l_corr - predicted;
            const double variance =
                clasPhaseVariance(config, osr.elevation, f, osr.frequencies[f]);
            const bool have_atmos_ref =
                osr.atmos_reference_time.week != 0 ||
                std::abs(osr.atmos_reference_time.tow) > 0.0;
            const bool have_clock_ref =
                osr.clock_reference_time.week != 0 ||
                std::abs(osr.clock_reference_time.tow) > 0.0;
            const bool have_code_bias_ref =
                osr.code_bias_reference_time.week != 0 ||
                std::abs(osr.code_bias_reference_time.tow) > 0.0;
            const double atmos_clock_gap_s =
                (have_atmos_ref && have_clock_ref)
                    ? osr.atmos_reference_time - osr.clock_reference_time
                    : 0.0;
            auto timeWeekField = [](const GNSSTime& time, bool have_time) {
                return have_time ? std::to_string(time.week) : std::string();
            };
            auto timeTowField = [](const GNSSTime& time, bool have_time) {
                if (!have_time) {
                    return std::string();
                }
                std::ostringstream stream;
                stream << std::setprecision(17) << time.tow;
                return stream.str();
            };
            auto secondsField = [](double value, bool have_value) {
                if (!have_value) {
                    return std::string();
                }
                std::ostringstream stream;
                stream << std::setprecision(17) << value;
                return stream.str();
            };
            auto doubleField = [](double value, bool have_value) {
                if (!have_value) {
                    return std::string();
                }
                std::ostringstream stream;
                stream << std::setprecision(17) << value;
                return stream.str();
            };

            *dump << std::setprecision(17)
                  << "PHASE,"
                  << stage << ','
                  << obs.time.week << ','
                  << obs.time.tow << ','
                  << osr.satellite.toString() << ','
                  << f << ','
                  << static_cast<int>(raw->signal) << ','
                  << raw->pseudorange_observation_type << ','
                  << raw->carrier_phase_observation_type << ','
                  << algorithms::ppp_bias_identity::rtklibCodeForObservationType(
                         raw->pseudorange_observation_type) << ','
                  << algorithms::ppp_bias_identity::rtklibCodeForObservationType(
                         raw->carrier_phase_observation_type) << ','
                  << ppp_internal::signalFamilyName(raw->signal) << ','
                  << osr.pseudorange_rinex_codes[f] << ','
                  << osr.carrier_rinex_codes[f] << ','
                  << (lookup.exact_identity_requested ? 1 : 0) << ','
                  << (lookup.exact_identity_matched ? 1 : 0) << ','
                  << (lookup.family_fallback ? 1 : 0) << ','
                  << static_cast<int>(osr.code_bias_signal_ids[f]) << ','
                  << static_cast<int>(osr.phase_bias_signal_ids[f]) << ','
                  << (osr.bias_exact_identity[f] ? 1 : 0) << ','
                  << static_cast<int>(osr.code_bias_source_signal_ids[f]) << ','
                  << static_cast<int>(osr.phase_bias_source_signal_ids[f]) << ','
                  << (osr.code_bias_present[f] ? 1 : 0) << ','
                  << (osr.phase_bias_present[f] ? 1 : 0) << ','
                  << (osr.code_bias_fallback[f] ? 1 : 0) << ','
                  << (osr.phase_bias_fallback[f] ? 1 : 0) << ','
                  << ','
                  << ','
                  << applied.carrier_phase_correction_m << ','
                  << ','
                  << ','
                  << osr.trop_correction_m << ','
                  << osr.iono_l1_m << ','
                  << osr.stec_tecu << ','
                  << iono_scaled << ','
                  << ','
                  << osr.network_compensation_m << ','
                  << osr.receiver_antenna_m[f] << ','
                  << osr.relativity_correction_m << ','
                  << timeWeekField(osr.atmos_reference_time, have_atmos_ref) << ','
                  << timeTowField(osr.atmos_reference_time, have_atmos_ref) << ','
                  << timeWeekField(osr.clock_reference_time, have_clock_ref) << ','
                  << timeTowField(osr.clock_reference_time, have_clock_ref) << ','
                  << timeWeekField(osr.code_bias_reference_time, have_code_bias_ref) << ','
                  << timeTowField(osr.code_bias_reference_time, have_code_bias_ref) << ','
                  << secondsField(atmos_clock_gap_s, have_atmos_ref && have_clock_ref) << ','
                  << osr.atmos_network_id << ','
                  << osr.atmos_grid_no << ','
                  << osr.atmos_nearest_grid_distance_m << ','
                  << osr.atmos_interpolation_grid_count << ','
                  << osr.atmos_interpolation_grid_no[0] << ','
                  << osr.atmos_interpolation_weights[0] << ','
                  << osr.atmos_interpolation_grid_no[1] << ','
                  << osr.atmos_interpolation_weights[1] << ','
                  << osr.atmos_interpolation_grid_no[2] << ','
                  << osr.atmos_interpolation_weights[2] << ','
                  << osr.atmos_interpolation_grid_no[3] << ','
                  << osr.atmos_interpolation_weights[3] << ','
                  << (osr.atmos_lifecycle ? 1 : 0) << ','
                  << doubleField(
                         osr.atmos_lifecycle_tow,
                         osr.has_atmos_lifecycle_tow) << ','
                  << osr.atmos_selected_satellite_count << ','
                  << osr.atmos_valid_grid_count << ','
                  << osr.atmos_stec_grid_value_count << ','
                  << osr.atmos_selected_grid_stec_value_count << ','
                  << geo << ','
                  << sat_clk_m << ','
                  << receiver_clock_m << ','
                  << phase_trop_model << ','
                  << iono_state_l1_m << ','
                  << iono_scale << ','
                  << predicted << ','
                  << residual << ','
                  << variance << ','
                  << los_enu.x() << ','
                  << los_enu.y() << ','
                  << los_enu.z() << ','
                  << osr.azimuth << ','
                  << osr.elevation << ','
                  << receiver_position.x() << ','
                  << receiver_position.y() << ','
                  << receiver_position.z();
            writeClasCodeDumpPhasePhaseExtension(
                *dump,
                osr.CPC[f],
                applied.carrier_phase_correction_m,
                osr.CPC[f] - osr.trop_correction_m,
                osr.phase_bias_m[f],
                osr.windup_m[f],
                osr.phase_compensation_m[f],
                l_m,
                l_corr);
            *dump << '\n';
        }
    }
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
            pppEnvOverrides().clas_code_row_full_prc ?
                osr.PRC[freq_index] :
                osr.PRC[freq_index] - osr.trop_correction_m;
        corrections.carrier_phase_correction_m =
            pppEnvOverrides().clas_amb_datum
                ? osr.CPC[freq_index]
                : osr.CPC[freq_index] - osr.trop_correction_m;
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

bool suppressesClasAmbDatumPhaseTrop(
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy) {
    const auto& env = pppEnvOverrides();
    return env.clas_amb_datum &&
           !env.clas_amb_datum_residual_phase_trop &&
           policy ==
               ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
}

bool usesResidualClasAmbDatumPhaseTrop(
    ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy policy) {
    const auto& env = pppEnvOverrides();
    return env.clas_amb_datum &&
           env.clas_amb_datum_residual_phase_trop &&
           policy ==
               ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
}

double effectiveClasTropPriorVariance(const ppp_shared::PPPConfig& config) {
    const double value = pppEnvOverrides().clas_trop_prior_variance;
    return value > 0.0 ? value : config.clas_trop_prior_variance;
}

double effectiveClasTropInitialVariance(const ppp_shared::PPPConfig& config) {
    const double value = pppEnvOverrides().clas_trop_initial_variance;
    return value > 0.0 ? value : config.clas_trop_initial_variance;
}

double effectiveClasTropProcessNoise(const ppp_shared::PPPConfig& config) {
    const double value = pppEnvOverrides().clas_trop_process_noise;
    return value > 0.0 ? value : config.clas_trop_process_noise;
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
        bool qzss_visible = false;
        for (const auto& satellite : obs.getSatellites()) {
            if (satellite.system == GNSSSystem::QZSS) {
                qzss_visible = true;
                break;
            }
        }
        initializeFilterState(
            filter_state,
            seed_solution,
            obs.time,
            iono_satellites,
            config,
            modeled_zenith_troposphere_delay_m,
            qzss_visible);
        filter_initialized = true;
        convergence_start_time = obs.time;
        if (!config.kinematic_mode || config.low_dynamics_mode) {
            static_anchor_position = seed_solution.position_ecef;
            has_static_anchor_position = true;
        }
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
        seed_solution.position_ecef,
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
    double modeled_zenith_troposphere_delay_m,
    bool qzss_visible) {
    filter_state.gal_clock_index = -1;
    filter_state.qzs_clock_index = -1;
    filter_state.bds_clock_index = -1;
    int isb_start = 9;
    if (pppEnvOverrides().clas_qzss_s_prn_fix && config.use_clas_osr_filter &&
        qzss_visible) {
        filter_state.qzs_clock_index = isb_start++;
    }
    const int n_isb = isb_start - 9;
    const int n_iono = static_cast<int>(iono_satellites.size());
    const int base = 9 + n_isb + n_iono;
    filter_state.ionosphere_indices.clear();
    filter_state.state = VectorXd::Zero(base);
    filter_state.covariance = MatrixXd::Identity(base, base);
    filter_state.state.segment(0, 3) = seed_solution.position_ecef;
    filter_state.state(filter_state.clock_index) = seed_solution.receiver_clock_bias;
    filter_state.state(filter_state.glo_clock_index) = seed_solution.receiver_clock_bias;
    if (filter_state.qzs_clock_index >= 0) {
        filter_state.state(filter_state.qzs_clock_index) =
            seed_solution.receiver_clock_bias;
    }
    filter_state.state(filter_state.trop_index) = modeled_zenith_troposphere_delay_m;
    filter_state.covariance.block(0, 0, 3, 3) *= config.clas_initial_position_variance;
    if (config.kinematic_mode && config.use_dynamics_model) {
        filter_state.covariance.block(
            filter_state.vel_index, filter_state.vel_index, 3, 3) *=
            config.initial_velocity_variance;
    }
    filter_state.covariance(6, 6) = config.clas_clock_variance;
    filter_state.covariance(7, 7) = config.clas_clock_variance;
    filter_state.covariance(8, 8) = effectiveClasTropInitialVariance(config);
    if (filter_state.qzs_clock_index >= 0) {
        filter_state.covariance(filter_state.qzs_clock_index,
                                filter_state.qzs_clock_index) =
            config.clas_clock_variance;
    }
    filter_state.iono_index = isb_start;
    // MRTKLIB literal-port track: clas.toml [kalman_filter.initial_std]
    // ionosphere = 0.01 m (the state is residual iono after the CLAS grid
    // STEC correction; mrtk_ppp_rtk.c:498 initx(1e-6, SQR(std[1]))).
    const double iono_initial_variance = clasMrtklibFloatParity(config)
        ? 0.01 * 0.01
        : std::min(config.initial_ionosphere_variance, 1.0);
    for (size_t index = 0; index < iono_satellites.size(); ++index) {
        const int state_index = filter_state.iono_index + static_cast<int>(index);
        filter_state.ionosphere_indices[iono_satellites[index]] = state_index;
        filter_state.state(state_index) = 0.0;
        filter_state.covariance(state_index, state_index) = iono_initial_variance;
    }
    filter_state.amb_index = base;
    filter_state.total_states = base;
}

namespace {

constexpr double kClasNominalEpochIntervalS = 0.2;
constexpr double kClasOutageGapResetS = 2.0;
// MRTKLIB clas.toml out_count = 1 (opt.maxout): reset a satellite's ambiguity
// once its observations were missing for MORE than maxout consecutive epochs
// (outc = 2 triggers the reset, mrtk_ppp_rtk.c:865). Expressed as a gap in
// units of the current epoch cadence: present = 1*dt, missed one epoch =
// 2*dt (kept), missed two = 3*dt (reset); threshold 2.5*dt separates them.
constexpr double kClasPerSatOutageEpochs = 2.0;
constexpr double kMinimumGeometryFreeSlipThresholdMeters = 0.05;
// MRTKLIB uses GF-only slip at 0.05 m; MW-mean is native-only. Raised from 0.5
// to reduce Galileo burst false slips (see gnss_mrtklib_diff_report.md Q4).
constexpr double kMinimumMwSlipThresholdCycles = 1.0;
constexpr double kMwSlipFallbackThresholdMeters = 10.0;

double clasSlipThresholdScale(double dt_seconds) {
    return std::max(1.0, dt_seconds / kClasNominalEpochIntervalS);
}

void clearClasWlnlMwState(ppp_shared::PPPAmbiguityInfo& ambiguity) {
    ambiguity.wl_is_fixed = false;
    ambiguity.wl_fixed_integer = 0;
    ambiguity.nl_is_fixed = false;
    ambiguity.nl_fixed_cycles = 0.0;
    ambiguity.mw_sum_cycles = 0.0;
    ambiguity.mw_count = 0;
    ambiguity.mw_mean_cycles = 0.0;
}

void resetClasPhaseBiasRepair(CLASPhaseBiasRepairInfo& repair) {
    repair.reference_time = GNSSTime();
    repair.last_continuity_m = {0.0, 0.0, 0.0};
    repair.offset_cycles = {0.0, 0.0, 0.0};
    repair.pending_state_shift_cycles = {0.0, 0.0, 0.0};
    repair.has_last = {false, false, false};
}

}  // namespace

ClasSlipDetectionStats detectClasCycleSlips(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPConfig& config,
    double dt_seconds,
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    const AmbiguityResetFunction& ambiguity_reset_function,
    double ambiguity_reset_variance,
    bool debug_enabled) {
    ClasSlipDetectionStats stats;
    if (!config.kinematic_mode || !config.enable_cycle_slip_detection) {
        return stats;
    }

    const double threshold_scale = clasSlipThresholdScale(dt_seconds);
    const double gf_threshold_m = std::max(
        config.cycle_slip_threshold, kMinimumGeometryFreeSlipThresholdMeters) *
        threshold_scale;
    const double mw_threshold_cycles = kMinimumMwSlipThresholdCycles * threshold_scale;
    const bool outage_gap = dt_seconds > kClasOutageGapResetS;
    // Dynamics mode: a receiver-wide data outage (bridge/tunnel) almost
    // certainly broke carrier lock on every satellite, and the GF/MW
    // detectors cannot see it (their history is cleared by the outage
    // branch below, so the first epoch back has nothing to difference
    // against). RTKLIB handles this via the per-satellite outage counter
    // (udbias: outage > maxout resets the bias); mirror that by treating
    // the gap itself as a slip on every satellite. White-noise mode keeps
    // its historical behavior (position/clock are re-anchored to SPP each
    // epoch there, which bounds the damage of a stale ambiguity until the
    // residual gates catch it).
    const bool outage_resets_ambiguity =
        outage_gap && config.use_dynamics_model && !config.low_dynamics_mode;

    for (const auto& osr : osr_corrections) {
        if (!osr.valid || osr.num_frequencies < 2) {
            continue;
        }
        const Observation* l1_raw = findOsrFrequencyObservation(obs, osr, 0);
        const Observation* l2_raw = findOsrFrequencyObservation(obs, osr, 1);
        if (!l1_raw || !l2_raw || !l1_raw->valid || !l2_raw->valid) {
            continue;
        }
        if (!l1_raw->has_carrier_phase || !l2_raw->has_carrier_phase) {
            continue;
        }
        if (!l1_raw->has_pseudorange || !l2_raw->has_pseudorange) {
            continue;
        }
        const double f1 = osr.frequencies[0];
        const double f2 = osr.frequencies[1];
        if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1e6) {
            continue;
        }

        auto& ambiguity = ambiguity_states[osr.satellite];
        const double l1_m = l1_raw->carrier_phase * osr.wavelengths[0] -
                              osr.phase_bias_m[0];
        const double l2_m = l2_raw->carrier_phase * osr.wavelengths[1] -
                              osr.phase_bias_m[1];
        const double gf_m = l1_m - l2_m;
        const double p1 = l1_raw->pseudorange - osr.code_bias_m[0];
        const double p2 = l2_raw->pseudorange - osr.code_bias_m[1];
        const double mw_m = (f1 * l1_m - f2 * l2_m) / (f1 - f2) -
                              (f1 * p1 + f2 * p2) / (f1 + f2);
        const double lambda_wl = constants::SPEED_OF_LIGHT / std::abs(f1 - f2);
        const double mw_cycles = mw_m / lambda_wl;

        bool lli_slip = l1_raw->loss_of_lock || l2_raw->loss_of_lock;
        bool gf_slip = false;
        bool mw_slip = false;

        // MRTKLIB per-satellite outage reset (mrtk_ppp_rtk.c:865-875,
        // clas.toml out_count = 1): a satellite whose observations were
        // missing (or whose measurement update failed) for more than maxout
        // consecutive epochs gets its ambiguity fully reset and lock
        // restarted. Without this a satellite blocked for a few seconds
        // returns with its stale ambiguity still AR-eligible -- the GF/MW
        // detectors compare against seconds-old history and can miss the
        // integer break, producing self-consistent wrong fixes. The global
        // outage_gap branch below only covers receiver-wide gaps.
        bool per_sat_outage = false;
        if (config.use_clas_osr_filter && !outage_gap &&
            ambiguity.last_time.week > 0 && dt_seconds > 0.0) {
            const double sat_gap_s = obs.time - ambiguity.last_time;
            if (sat_gap_s > kClasPerSatOutageEpochs * dt_seconds + 0.5 * dt_seconds) {
                per_sat_outage = true;
                ++stats.per_sat_outage_resets;
            }
        }

        if (outage_gap) {
            ++stats.outage_resets;
            ambiguity.has_last_geometry_free = false;
            ambiguity.has_last_melbourne_wubbena = false;
            clearClasWlnlMwState(ambiguity);
        } else {
            if (ambiguity.has_last_geometry_free &&
                std::isfinite(gf_m) &&
                std::abs(gf_m - ambiguity.last_geometry_free_m) > gf_threshold_m) {
                gf_slip = true;
            }
            // MW-mean slip: skip when WL is already fixed or the running mean has
            // converged to an integer (MRTKLIB has no MW-mean detector at all).
            const bool wl_mean_stable =
                ambiguity.mw_count >= config.wl_min_averaging_epochs &&
                std::isfinite(ambiguity.mw_mean_cycles) &&
                std::abs(ambiguity.mw_mean_cycles -
                         std::round(ambiguity.mw_mean_cycles)) < 0.25;
            if (!ambiguity.wl_is_fixed && !wl_mean_stable) {
                if (ambiguity.mw_count >= 3 &&
                    std::isfinite(mw_cycles) &&
                    std::abs(mw_cycles - ambiguity.mw_mean_cycles) >
                        mw_threshold_cycles) {
                    mw_slip = true;
                } else if (ambiguity.mw_count < 3 &&
                           ambiguity.has_last_melbourne_wubbena &&
                           std::isfinite(mw_cycles) &&
                           std::abs(mw_m - ambiguity.last_melbourne_wubbena_m) >
                               kMwSlipFallbackThresholdMeters * threshold_scale) {
                    mw_slip = true;
                }
            }
        }

        if (std::isfinite(gf_m)) {
            ambiguity.last_geometry_free_m = gf_m;
            ambiguity.has_last_geometry_free = true;
        }
        if (std::isfinite(mw_m)) {
            ambiguity.last_melbourne_wubbena_m = mw_m;
            ambiguity.has_last_melbourne_wubbena = true;
        }

        if (!lli_slip && !gf_slip && !mw_slip && !outage_resets_ambiguity &&
            !per_sat_outage) {
            continue;
        }

        if (lli_slip) {
            ++stats.lli_count;
        }
        if (gf_slip) {
            ++stats.gf_count;
        }
        if (mw_slip) {
            ++stats.mw_count;
        }
        ++stats.total_resets;

        ambiguity.needs_reinitialization = true;
        ambiguity.has_last_slip_time = true;
        ambiguity.last_slip_time = obs.time;
        clearClasWlnlMwState(ambiguity);
        ambiguity.has_last_geometry_free = false;
        ambiguity.has_last_melbourne_wubbena = false;

        dispersion_compensation[osr.satellite].slip = {true, true};
        auto repair_it = phase_bias_repair.find(osr.satellite);
        if (repair_it != phase_bias_repair.end()) {
            resetClasPhaseBiasRepair(repair_it->second);
        }

        if (ambiguity_reset_function) {
            const uint8_t l2_prn = static_cast<uint8_t>(
                std::min(255, static_cast<int>(osr.satellite.prn) + 100));
            const SatelliteId l2_satellite(osr.satellite.system, l2_prn);
            ambiguity_reset_function(osr.satellite, l1_raw->signal);
            ambiguity_reset_function(l2_satellite, l2_raw->signal);
        }

        if (debug_enabled) {
            std::string reason;
            if (lli_slip) {
                reason += "lli";
            }
            if (gf_slip) {
                if (!reason.empty()) {
                    reason += "+";
                }
                reason += "gf";
            }
            if (mw_slip) {
                if (!reason.empty()) {
                    reason += "+";
                }
                reason += "mw";
            }
            if (outage_resets_ambiguity) {
                if (!reason.empty()) {
                    reason += "+";
                }
                reason += "outage";
            }
            if (per_sat_outage) {
                if (!reason.empty()) {
                    reason += "+";
                }
                reason += "outage_sat";
            }
            std::cerr << "[CLAS-SLIP] " << osr.satellite.toString()
                      << " tow=" << obs.time.tow
                      << " reason=" << reason
                      << " dt=" << dt_seconds
                      << " gf_m=" << gf_m
                      << " mw_cyc=" << mw_cycles
                      << "\n";
        }
    }

    if (stats.total_resets > 0) {
        syncSlipState(
            obs,
            filter_state,
            ambiguity_states,
            dispersion_compensation,
            phase_bias_repair,
            ambiguity_reset_variance);
    }

    return stats;
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

namespace {

void reinitializePositionState(ppp_shared::PPPState& filter_state,
                               const Vector3d& position,
                               double variance) {
    for (int axis = 0; axis < 3; ++axis) {
        const int index = filter_state.pos_index + axis;
        filter_state.state(index) = position(axis);
        filter_state.covariance.row(index).setZero();
        filter_state.covariance.col(index).setZero();
        filter_state.covariance(index, index) = variance;
    }
}

}  // namespace

void predictFilterState(
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    double dt,
    const Vector3d& seed_position_ecef,
    double seed_receiver_clock_bias_m,
    bool seed_valid) {
    const int nx = filter_state.total_states;
    const bool kinematic_white_noise =
        config.kinematic_mode && !config.low_dynamics_mode;
    const bool use_dynamic_prediction =
        kinematic_white_noise && config.use_dynamics_model;

    MatrixXd F = MatrixXd::Identity(nx, nx);
    if (use_dynamic_prediction) {
        F.block(filter_state.pos_index, filter_state.vel_index, 3, 3) =
            MatrixXd::Identity(3, 3) * dt;
        filter_state.state = F * filter_state.state;
    } else if (kinematic_white_noise) {
        if (config.reset_kinematic_position_to_spp_each_epoch && seed_valid) {
            reinitializePositionState(
                filter_state,
                seed_position_ecef,
                config.initial_position_variance);
        }
    } else if (config.process_noise_position > 0.0) {
        const double position_q = config.process_noise_position * dt;
        for (int axis = 0; axis < 3; ++axis) {
            filter_state.covariance(filter_state.pos_index + axis,
                                    filter_state.pos_index + axis) += position_q;
        }
    }

    MatrixXd Q = MatrixXd::Zero(nx, nx);
    if (use_dynamic_prediction) {
        const double position_q =
            std::max(0.0, config.process_noise_position) * dt;
        const double velocity_q =
            std::max(0.0, config.process_noise_velocity) * dt;
        for (int axis = 0; axis < 3; ++axis) {
            Q(filter_state.pos_index + axis, filter_state.pos_index + axis) =
                position_q;
            Q(filter_state.vel_index + axis, filter_state.vel_index + axis) =
                velocity_q;
        }
    }
    // Receiver clock temporal update. Both modes treat the clock as WHITE
    // NOISE re-initialized from the SPP solution each epoch, mirroring
    // RTKLIB/MRTKLIB PPP udclk_ppp (mrtk_ppp.c:822: initx(CLIGHT*dtr,
    // VAR_CLK) every epoch; VAR_CLK = 60^2 m^2). A random-walk clock model
    // is NOT viable for consumer receivers: this class of hardware drifts
    // ~150 m/s (measured -30.5 m per 0.2 s epoch on the PPC tokyo_run2
    // rover), so any Q small enough to keep the ambiguity float covariance
    // well-conditioned lags the true clock, and a multi-second data outage
    // (bridge) accumulates hundreds of meters of clock error against a
    // few-meter sigma -- a guaranteed rejection spiral.
    //
    // When the SPP seed is unavailable (deep urban canyon, <4 usable
    // satellites), the dynamics filter coasts the clock state and must
    // inflate its variance by the drift accumulated over dt: the drift is
    // quasi-deterministic, so the coast variance grows with dt^2, not dt.
    const bool clock_coasting = use_dynamic_prediction && !seed_valid;
    const double clock_coast_drift_m = config.clas_dynamic_clock_coast_drift_mps * dt;
    const double clock_process_noise = clock_coasting
        ? clock_coast_drift_m * clock_coast_drift_m
        : config.clas_clock_variance;
    Q(filter_state.clock_index, filter_state.clock_index) = clock_process_noise;
    Q(filter_state.glo_clock_index, filter_state.glo_clock_index) = clock_process_noise;
    if (filter_state.qzs_clock_index >= 0) {
        Q(filter_state.qzs_clock_index, filter_state.qzs_clock_index) =
            pppEnvOverrides().isb_process_noise * dt;
    }
    Q(filter_state.trop_index, filter_state.trop_index) =
        effectiveClasTropProcessNoise(config) * dt;
    // MRTKLIB literal-port track: clas.toml [kalman_filter.process_noise]
    // ionosphere = 0.001 (std, m/sqrt(s)) and bias = 0.001 -> Q = 1e-6*dt
    // (mrtk_ppp_rtk.c:505-517 udion with adaptive filter disabled, and
    // :752 udbias). The historical path keeps its existing tuning.
    const bool mrtklib_parity = clasMrtklibFloatParity(config);
    constexpr double kMrtklibIonoProcessNoise = 0.001 * 0.001;   // (m^2/s)
    constexpr double kMrtklibBiasProcessNoise = 0.001 * 0.001;   // (m^2/s)
    if (config.estimate_ionosphere) {
        const double iono_process_noise = mrtklib_parity
            ? kMrtklibIonoProcessNoise
            : effectiveClasIonoProcessNoise(config);
        for (const auto& [_, state_index] : filter_state.ionosphere_indices) {
            if (state_index >= 0 && state_index < nx) {
                Q(state_index, state_index) = iono_process_noise * dt;
            }
        }
    }
    const double bias_process_noise = mrtklib_parity
        ? kMrtklibBiasProcessNoise
        : config.process_noise_ambiguity;
    for (const auto& [_, state_index] : filter_state.ambiguity_indices) {
        if (state_index >= 0 && state_index < nx) {
            Q(state_index, state_index) = bias_process_noise * dt;
        }
    }
    filter_state.covariance = F * filter_state.covariance * F.transpose() + Q;
    // White-noise clock: re-initialize the clock STATE from the SPP
    // solution every epoch in both position models (RTKLIB PPP udclk_ppp
    // semantics -- dynamics in udpos_ppp only ever applies to pos/vel/acc,
    // never to the clock). In dynamics mode the measurement update then
    // refines the clock within the reseed variance set below.
    if (seed_valid) {
        const double gps_clock_before = filter_state.state(filter_state.clock_index);
        filter_state.state(filter_state.clock_index) = seed_receiver_clock_bias_m;
        filter_state.state(filter_state.glo_clock_index) = seed_receiver_clock_bias_m;
        if (filter_state.qzs_clock_index >= 0) {
            const double isb_offset =
                filter_state.state(filter_state.qzs_clock_index) - gps_clock_before;
            filter_state.state(filter_state.qzs_clock_index) =
                seed_receiver_clock_bias_m + isb_offset;
        }
    }
    // Decouple clock from position/ambiguities: zero cross-covariance to
    // prevent code observation noise from leaking into position (and to keep
    // the ambiguity float covariance well-conditioned for AR/LAMBDA search)
    // via KF coupling. This cross-term zeroing is safe and desirable in both
    // modes: any correlation useful for a single measurement update is
    // re-derived within that same update from the current epoch's H/R, so
    // zeroing the *carried-forward* cross-covariance each predict step only
    // discards stale, epoch-old correlation.
    //
    // The diagonal (clock's own variance) differs by mode:
    //  - White-noise mode: the clock was just hard-reseeded from SPP, and
    //    position is also re-anchored to SPP each epoch, so the clock is
    //    treated as exactly known (variance 0; measurement update leaves it
    //    at the SPP value).
    //  - Dynamics mode with a valid seed: RTKLIB VAR_CLK semantics -- the
    //    reseeded SPP clock is a prior with ~60 m sigma and the measurement
    //    update refines it each epoch. The variance must NOT be zeroed
    //    (that would freeze the clock at the SPP value; worse, on epochs
    //    where the reseed is skipped it would freeze the clock entirely,
    //    which was the original divergence root cause) and must not be left
    //    at clas_clock_variance=1e8 either (destroys LAMBDA conditioning).
    //  - Dynamics mode coasting (no SPP seed): keep the propagated
    //    variance, which already includes the dt^2 drift inflation from Q.
    if (config.clas_decouple_clock_position) {
        const int ci = filter_state.clock_index;
        const int gi = filter_state.glo_clock_index;
        const int qi = filter_state.qzs_clock_index;
        for (int i = 0; i < nx; ++i) {
            if (i != ci) { filter_state.covariance(ci, i) = 0; filter_state.covariance(i, ci) = 0; }
            if (i != gi) { filter_state.covariance(gi, i) = 0; filter_state.covariance(i, gi) = 0; }
            if (qi >= 0 && i != qi) {
                filter_state.covariance(qi, i) = 0;
                filter_state.covariance(i, qi) = 0;
            }
        }
        if (!use_dynamic_prediction) {
            filter_state.covariance(ci, ci) = 0;
            filter_state.covariance(gi, gi) = 0;
        } else if (seed_valid) {
            filter_state.covariance(ci, ci) = config.clas_dynamic_clock_reseed_variance;
            filter_state.covariance(gi, gi) = config.clas_dynamic_clock_reseed_variance;
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
    (void)receiver_clock_m;
    MeasurementBuildResult result;
    const bool stec_constraint = usesClasStecConstraint(config);
    std::map<SatelliteId, double> iono_state_targets_m;
    std::map<SatelliteId, double> iono_state_variances_m2;

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }

        const double geo = geodist(osr.satellite_position, receiver_position);
        const Vector3d los =
            (osr.satellite_position - receiver_position).normalized();
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double receiver_clock_m =
            receiverClockBiasMeters(filter_state, osr.satellite);
        const int receiver_clock_index =
            receiverClockStateIndex(filter_state, osr.satellite);
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
                findOsrFrequencyObservation(obs, osr, f);
        }

        for (int f = 0; f < osr.num_frequencies; ++f) {
            const Observation* raw = raw_observations[static_cast<size_t>(f)];
            if (!raw || !raw->valid) {
                continue;
            }
            if (config.kinematic_mode &&
                config.use_clas_osr_filter &&
                raw->snr > 0.0 &&
                ppp_ar::clasKinematicSnrMasked(f, osr.elevation, raw->snr)) {
                continue;
            }
            const double iono_scale =
                (osr.frequencies[f] > 0.0 && osr.wavelengths[0] > 0.0)
                    ? std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2)
                    : 1.0;
            auto applied_corrections = selectAppliedOsrCorrections(
                osr, f, config.clas_correction_application_policy);
            if (stec_constraint && use_residual_iono_state &&
                osr.has_iono && std::isfinite(osr.iono_l1_m)) {
                const double iono_scaled = iono_scale * osr.iono_l1_m;
                applied_corrections.pseudorange_correction_m -= iono_scaled;
                applied_corrections.carrier_phase_correction_m += iono_scaled;
                iono_state_targets_m[osr.satellite] = osr.iono_l1_m;
                iono_state_variances_m2[osr.satellite] =
                    clasStecConstraintVariance(epoch_atmos, osr, config);
            }
            const double iono_state_m =
                use_residual_iono_state ? filter_state.state(iono_state_index) : 0.0;

            if (raw->has_pseudorange && std::isfinite(raw->pseudorange)) {
                const bool claslib_code_prc = usesClaslibCodePrcRows(config);
                const double code_trop_modeled =
                    claslib_code_prc ? 0.0 : trop_modeled;
                const double code_trop_mapping =
                    claslib_code_prc ? 0.0 : trop_mapping;
                const double p_corr =
                    raw->pseudorange - applied_corrections.pseudorange_correction_m;
                const double predicted =
                    geo - sat_clk_m + receiver_clock_m + code_trop_modeled +
                    iono_scale * iono_state_m;
                const double residual = p_corr - predicted;

                const double el_weight = elevationWeight(osr.elevation);

                MeasurementRow row;
                row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
                row.H.segment(0, 3) = -los.transpose();
                row.H(receiver_clock_index) = 1.0;
                row.H(filter_state.trop_index) = code_trop_mapping;
                if (use_residual_iono_state) {
                    row.H(iono_state_index) = iono_scale;
                    result.observed_iono_states.insert(osr.satellite);
                }
                row.residual = residual;
                row.variance = clasCodeVariance(config, osr.elevation);
                row.satellite = osr.satellite;
                row.is_phase = false;
                row.freq_index = f;
                result.measurements.push_back(row);
            }

            if (raw->has_carrier_phase && std::isfinite(raw->carrier_phase)) {
                const double l_m = raw->carrier_phase * osr.wavelengths[f];
                const double l_corr =
                    l_m - applied_corrections.carrier_phase_correction_m;
                if (auto* dump = clasGeometryDumpStream();
                    dump != nullptr && selectedClasGeometryDumpTow(obs.time.tow)) {
                    const Vector3d forced_receiver_position =
                        clasGeometryDumpReceiverPosition(receiver_position);
                    const double forced_rho =
                        geodist(osr.satellite_position, forced_receiver_position);
                    const double euclidean =
                        (osr.satellite_position - forced_receiver_position).norm();
                    const double forced_sagnac = forced_rho - euclidean;
                    const double forced_sat_clk_m =
                        constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
                    double forced_lat = 0.0;
                    double forced_lon = 0.0;
                    double forced_h = 0.0;
                    ecef2geodetic(
                        forced_receiver_position, forced_lat, forced_lon, forced_h);
                    const Vector3d forced_los_enu = ecef2enu(
                        osr.satellite_position - forced_receiver_position,
                        forced_lat,
                        forced_lon);
                    const double forced_el = std::atan2(
                        forced_los_enu.z(),
                        std::hypot(forced_los_enu.x(), forced_los_enu.y()));
                    const double forced_az =
                        std::atan2(forced_los_enu.x(), forced_los_enu.y());
                    *dump << std::setprecision(17)
                          << "GEOM,"
                          << obs.time.week << ','
                          << obs.time.tow << ','
                          << osr.signal_transmit_time.tow << ','
                          << osr.satellite.toString() << ','
                          << f << ','
                          << static_cast<int>(raw->signal) << ','
                          << receiver_position.x() << ','
                          << receiver_position.y() << ','
                          << receiver_position.z() << ','
                          << receiver_position.x() << ','
                          << receiver_position.y() << ','
                          << receiver_position.z() << ','
                          << forced_receiver_position.x() << ','
                          << forced_receiver_position.y() << ','
                          << forced_receiver_position.z() << ','
                          << 0.0 << ','
                          << 0.0 << ','
                          << 0.0 << ','
                          << osr.satellite_position.x() << ','
                          << osr.satellite_position.y() << ','
                          << osr.satellite_position.z() << ','
                          << osr.satellite_velocity.x() << ','
                          << osr.satellite_velocity.y() << ','
                          << osr.satellite_velocity.z() << ','
                          << forced_sat_clk_m << ','
                          << euclidean << ','
                          << forced_sagnac << ','
                          << forced_rho << ','
                          << forced_az << ','
                          << forced_el << ','
                          << osr.CPC[f] << ','
                          << forced_rho - forced_sat_clk_m + osr.CPC[f]
                          << '\n';
                }
                const bool claslib_amb_datum_phase =
                    suppressesClasAmbDatumPhaseTrop(
                        config.clas_correction_application_policy);
                const bool residual_amb_datum_phase_trop =
                    usesResidualClasAmbDatumPhaseTrop(
                        config.clas_correction_application_policy);
                const double phase_trop_modeled = claslib_amb_datum_phase
                    ? 0.0
                    : (residual_amb_datum_phase_trop
                          ? trop_modeled - osr.trop_correction_m
                          : trop_modeled);
                const double phase_trop_partial =
                    claslib_amb_datum_phase ? 0.0 : trop_mapping;

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

                if (filter_state.covariance(amb_idx, amb_idx) >= config.clas_ambiguity_reinit_threshold) {
                    filter_state.state(amb_idx) =
                        l_corr - (geo - sat_clk_m + receiver_clock_m + phase_trop_modeled
                                  - iono_scale * iono_state_m);
                }

                const double predicted_no_amb =
                    geo - sat_clk_m + receiver_clock_m + phase_trop_modeled
                    - iono_scale * iono_state_m;
                const double predicted =
                    predicted_no_amb + filter_state.state(amb_idx);
                const double residual = l_corr - predicted;

                const double el_weight = elevationWeight(osr.elevation);

                MeasurementRow row;
                row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
                row.H.segment(0, 3) = -los.transpose();
                row.H(receiver_clock_index) = 1.0;
                row.H(filter_state.trop_index) = phase_trop_partial;
                if (use_residual_iono_state) {
                    row.H(iono_state_index) = -iono_scale;
                    result.observed_iono_states.insert(osr.satellite);
                }
                row.H(amb_idx) = 1.0;
                row.residual = residual;
                row.variance =
                    clasPhaseVariance(config, osr.elevation, f, osr.frequencies[f]);
                row.satellite = osr.satellite;
                row.is_phase = true;
                row.freq_index = f;
                result.measurements.push_back(row);
                result.observed_ambiguities.push_back(
                    {amb_sat, raw->signal, osr.wavelengths[f], raw->carrier_phase, raw->snr});
                if (auto* dump = ambDatumDumpStream(); dump != nullptr) {
                    const auto lookup =
                        findOsrFrequencyObservationWithProvenance(obs, osr, f);
                    const double ambiguity_state_m = filter_state.state(amb_idx);
                    const double ambiguity_state_cycles =
                        osr.wavelengths[f] > 0.0
                            ? ambiguity_state_m / osr.wavelengths[f]
                            : 0.0;
                    const double iono_cpc_m = -iono_scale * osr.iono_l1_m;
                    *dump << std::setprecision(17)
                          << "PHASE,"
                          << obs.time.week << ','
                          << obs.time.tow << ','
                          << osr.satellite.toString() << ','
                          << f << ','
                          << static_cast<int>(raw->signal) << ','
                          << raw->carrier_phase_observation_type << ','
                          << algorithms::ppp_bias_identity::rtklibCodeForObservationType(
                                 raw->carrier_phase_observation_type) << ','
                          << ppp_internal::signalFamilyName(raw->signal) << ','
                          << osr.pseudorange_rinex_codes[f] << ','
                          << osr.carrier_rinex_codes[f] << ','
                          << (lookup.exact_identity_requested ? 1 : 0) << ','
                          << (lookup.exact_identity_matched ? 1 : 0) << ','
                          << (lookup.family_fallback ? 1 : 0) << ','
                          << static_cast<int>(osr.phase_bias_signal_ids[f]) << ','
                          << (osr.bias_exact_identity[f] ? 1 : 0) << ','
                          << static_cast<int>(osr.phase_bias_source_signal_ids[f]) << ','
                          << (osr.phase_bias_present[f] ? 1 : 0) << ','
                          << (osr.phase_bias_fallback[f] ? 1 : 0) << ','
                          << osr.wavelengths[f] << ','
                          << raw->carrier_phase << ','
                          << l_m << ','
                          << applied_corrections.carrier_phase_correction_m << ','
                          << osr.CPC[f] << ','
                          << (osr.CPC[f] - osr.trop_correction_m) << ','
                          << osr.trop_correction_m << ','
                          << osr.relativity_correction_m << ','
                          << osr.receiver_antenna_m[f] << ','
                          << iono_cpc_m << ','
                          << osr.phase_bias_m[f] << ','
                          << osr.windup_m[f] << ','
                          << osr.phase_compensation_m[f] << ','
                          << l_corr << ','
                          << predicted_no_amb << ','
                          << ambiguity_state_m << ','
                          << ambiguity_state_cycles << ','
                          << residual << '\n';
                }
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
            row.variance = effectiveClasTropPriorVariance(config);
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
            const auto target_it = iono_state_targets_m.find(satellite);
            const double target_m =
                target_it != iono_state_targets_m.end() ? target_it->second : 0.0;
            row.residual = target_m - filter_state.state(iono_state_index);
            const auto variance_it = iono_state_variances_m2.find(satellite);
            row.variance = variance_it != iono_state_variances_m2.end()
                ? variance_it->second
                : config.clas_iono_prior_variance;
            row.satellite = satellite;
            row.is_phase = false;
            row.freq_index = -1;
            result.measurements.push_back(row);
        }
    }

    // Single-difference formation for carrier phase observations.
    // SD cancels receiver-side fractional cycle bias, improving ambiguity
    // convergence.  Code observations stay undifferenced to provide clock
    // and troposphere constraints.
    if (config.use_clas_osr_filter) {
        // Build elevation map for reference satellite selection
        std::map<SatelliteId, double> elevation_map;
        for (const auto& osr : osr_corrections) {
            if (osr.valid) {
                elevation_map[osr.satellite] = osr.elevation;
            }
        }

        // Group same-system measurements by signal. CLASLIB's ddres()
        // single-differences both phase and code; keep native's historical
        // undifferenced code path unless the parity gate is explicitly set.
        struct SdGroupKey {
            GNSSSystem system;
            int freq_index;
            bool is_phase;
            bool operator<(const SdGroupKey& rhs) const {
                if (system != rhs.system) return system < rhs.system;
                if (freq_index != rhs.freq_index) return freq_index < rhs.freq_index;
                return is_phase < rhs.is_phase;
            }
        };
        const bool code_sd = pppEnvOverrides().clas_code_sd;
        std::map<SdGroupKey, std::vector<size_t>> sd_groups;
        for (size_t i = 0; i < result.measurements.size(); ++i) {
            const auto& m = result.measurements[i];
            if (m.freq_index < 0) continue;
            if (!m.is_phase && !code_sd) continue;
            sd_groups[{m.satellite.system, m.freq_index, m.is_phase}].push_back(i);
        }

        // Start with prior constraints, plus undifferenced code rows on the
        // default path. The gated path lets code rows be rebuilt as SD below.
        std::vector<MeasurementRow> sd_measurements;
        for (size_t i = 0; i < result.measurements.size(); ++i) {
            const auto& m = result.measurements[i];
            if (m.freq_index < 0 || (!m.is_phase && !code_sd)) {
                sd_measurements.push_back(m);
            }
        }

        // Form SD for each selected group.
        for (auto& [group_key, member_indices] : sd_groups) {
            if (member_indices.size() < 2) continue;

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
            for (size_t idx : member_indices) {
                if (idx == ref_index) continue;
                const auto& sat_row = result.measurements[idx];
                MeasurementRow sd_row;
                sd_row.H = ref_row.H - sat_row.H;
                sd_row.residual = ref_row.residual - sat_row.residual;
                sd_row.variance = ref_row.variance + sat_row.variance;
                sd_row.satellite = sat_row.satellite;
                sd_row.is_phase = group_key.is_phase;
                sd_row.freq_index = group_key.freq_index;
                sd_measurements.push_back(sd_row);
            }
        }
        result.measurements = std::move(sd_measurements);
    }

    return result;
}

KalmanUpdateStats applyMeasurementUpdate(
    ppp_shared::PPPState& filter_state,
    const std::vector<MeasurementRow>& measurements,
    const ppp_shared::PPPConfig& config,
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
        if (std::abs(z(i)) > config.clas_outlier_sigma_scale * sigma) {
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

    const bool apply_spp_position_anchor =
        seed_solution != nullptr && seed_solution->isValid() &&
        (!config.kinematic_mode || config.low_dynamics_mode);
    if (apply_spp_position_anchor) {
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
        filter_state, measurement_build_result.measurements, config, &seed_solution);
    if (!result.update_stats.updated) {
        return result;
    }
    dumpClasCodeRows(
        "post",
        obs,
        epoch_context.osr_corrections,
        filter_state,
        config,
        trop_mapping_function);
    dumpClasPhaseRows(
        "post",
        obs,
        epoch_context.osr_corrections,
        filter_state,
        config,
        trop_mapping_function);

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
    bool debug_enabled,
    const FixValidationOptions& options) {
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
    const double trop_zenith = filter_state.state(filter_state.trop_index);
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }

        const double geo = geodist(osr.satellite_position, receiver_position);
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double receiver_clock_m =
            receiverClockBiasMeters(filter_state, osr.satellite);
        const double trop_mapping =
            trop_mapping_function(receiver_position, osr.elevation, obs.time);
        const double trop_modeled = trop_mapping * trop_zenith;

        std::array<const Observation*, OSR_MAX_FREQ> raw_observations{};
        for (int f = 0; f < osr.num_frequencies; ++f) {
            raw_observations[static_cast<size_t>(f)] =
                findOsrFrequencyObservation(obs, osr, f);
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
            // Mirror the buildEpochMeasurements() phase model: the applied
            // troposphere term depends on the correction application policy.
            const bool claslib_amb_datum_phase =
                suppressesClasAmbDatumPhaseTrop(
                    config.clas_correction_application_policy);
            const bool residual_amb_datum_phase_trop =
                usesResidualClasAmbDatumPhaseTrop(
                    config.clas_correction_application_policy);
            const double phase_trop_modeled = claslib_amb_datum_phase
                ? 0.0
                : (residual_amb_datum_phase_trop
                      ? trop_modeled - osr.trop_correction_m
                      : trop_modeled);
            const double phase_predicted =
                geo - sat_clk_m + receiver_clock_m + phase_trop_modeled;

            if (raw->has_pseudorange && std::isfinite(raw->pseudorange)) {
                const bool claslib_code_prc = usesClaslibCodePrcRows(config);
                const double code_trop_modeled =
                    claslib_code_prc ? 0.0 : trop_modeled;
                const double code_predicted =
                    geo - sat_clk_m + receiver_clock_m + code_trop_modeled;
                const double residual =
                    (raw->pseudorange - applied_corrections.pseudorange_correction_m) -
                    (code_predicted + iono_scale * iono_state_m);
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
                (phase_predicted - iono_scale * iono_state_m +
                 filter_state.state(ambiguity_index));
            const double variance =
                clasPhaseVariance(config, osr.elevation, f, osr.frequencies[f]);
            PhaseResidualInfo info;
            info.ambiguity_satellite = ambiguity_satellite;
            info.real_satellite = osr.satellite;
            info.residual_m = residual;
            info.variance_m2 = variance;
            info.frequency_hz = osr.frequencies[f];
            info.wavelength_m = osr.wavelengths[f];
            const double range = std::max(geo, 1.0);
            info.unit_vector =
                (receiver_position - osr.satellite_position) / range;
            info.trop_mapping = trop_mapping;
            info.iono_index =
                iono_scale > 0.0 ? iono_it->second : -1;
            info.iono_scale = iono_scale;
            info.ambiguity_index = ambiguity_index;
            phase_residuals.push_back(std::move(info));
        }
    }

    // MRTKLIB innovation-variance basis: h_dd' P h_dd for one DD phase row
    // (reference r minus satellite s within the same system group; receiver
    // clock cancels). h entries: position (u_r - u_s), troposphere zenith
    // (m_r - m_s), per-satellite ionosphere (-mu_r at r, +mu_s at s; phase
    // sign), ambiguity states (+1 at r, -1 at s; states are in meters).
    const auto dd_state_variance =
        [&](const PhaseResidualInfo& r, const PhaseResidualInfo& s) -> double {
        const MatrixXd& P = *options.innovation_covariance;
        const int n = static_cast<int>(P.rows());
        std::array<std::pair<int, double>, 9> h{};
        size_t k = 0;
        for (int axis = 0; axis < 3; ++axis) {
            h[k++] = {filter_state.pos_index + axis,
                      r.unit_vector(axis) - s.unit_vector(axis)};
        }
        h[k++] = {filter_state.trop_index, r.trop_mapping - s.trop_mapping};
        if (r.iono_index >= 0 && r.iono_index < n) {
            h[k++] = {r.iono_index, -r.iono_scale};
        }
        if (s.iono_index >= 0 && s.iono_index < n) {
            h[k++] = {s.iono_index, s.iono_scale};
        }
        if (r.ambiguity_index >= 0 && r.ambiguity_index < n) {
            h[k++] = {r.ambiguity_index, 1.0};
        }
        if (s.ambiguity_index >= 0 && s.ambiguity_index < n) {
            h[k++] = {s.ambiguity_index, -1.0};
        }
        double quad = 0.0;
        for (size_t i = 0; i < k; ++i) {
            for (size_t j = 0; j < k; ++j) {
                quad += h[i].second * P(h[i].first, h[j].first) * h[j].second;
            }
        }
        return std::max(quad, 0.0);
    };

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
            double dd_variance = reference.variance_m2 + residual.variance_m2;
            if (options.innovation_covariance != nullptr &&
                options.innovation_covariance->rows() ==
                    filter_state.total_states) {
                // MRTKLIB filter2_ basis (mrtk_ppp_rtk.c:1125): the residual
                // gate and chi-square normalize by the innovation covariance
                // Q = H'*P*H + R, not by R alone.
                dd_variance += dd_state_variance(reference, residual);
            }
            const double sigma = std::sqrt(std::max(dd_variance, 1e-12));

            // MRTKLIB residual_test() (mrtk_ppp_rtk.c:1040-1058): individual
            // residuals beyond the rejionno1 sigma gate are excluded from the
            // chi-square sum instead of failing the whole validation. D2 rule
            // (mrtk_ppp_rtk.c:1046-1050): the gate (not the chi-square) is
            // inflated 10x for rows whose phase-bias state is still at its
            // initialization variance.
            double gate_scale = 1.0;
            if (options.innovation_covariance != nullptr &&
                options.innovation_covariance->rows() ==
                    filter_state.total_states) {
                const MatrixXd& P_gate = *options.innovation_covariance;
                const double init_var = 1e4;  // MRTKLIB SQR(std[0]=100)
                for (const int amb_idx :
                     {reference.ambiguity_index, residual.ambiguity_index}) {
                    if (amb_idx >= 0 && amb_idx < P_gate.rows() &&
                        std::abs(P_gate(amb_idx, amb_idx) - init_var) <
                            1e-3 * init_var) {
                        gate_scale = 10.0;
                    }
                }
            }
            if (options.outlier_sigma_gate > 0.0 &&
                dd_residual * dd_residual >
                    options.outlier_sigma_gate * options.outlier_sigma_gate *
                        gate_scale * gate_scale * dd_variance) {
                ++stats.phase_outlier_rows;
                continue;
            }

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
    } else if (options.mrtklib_chisq_fallback) {
        // MRTKLIB residual_test() fallback (mrtk_ppp_rtk.c:1080-1082): with
        // too few rows for a chi-square test, pass when at least half of the
        // carrier residuals survived the outlier gate.
        const int total_rows = stats.phase_rows + stats.phase_outlier_rows;
        stats.phase_chisq =
            (total_rows <= 0 ||
             static_cast<double>(stats.phase_rows) / total_rows < 0.5)
                ? 100.0
                : 0.0;
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
    const GNSSTime& /*time*/,
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
