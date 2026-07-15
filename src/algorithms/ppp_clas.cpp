#include <libgnss++/algorithms/ppp_clas.hpp>

#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_bias_identity.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include "ppp_internal.hpp"
#include "ppp_clas_diagnostics.hpp"

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
    double elevation_rad = 0.0;
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
    return ppp_clas_diagnostics::ambiguityDatumStream();
}

std::ofstream* clasGeometryDumpStream() {
    return ppp_clas_diagnostics::geometryDumpStream();
}

bool clasPhaseRowDumpEnabled() {
    return ppp_clas_diagnostics::phaseRowDumpEnabled();
}

void writeClasCodeDumpCodePhaseExtension(std::ostream& out) {
    ppp_clas_diagnostics::writeCodePhaseExtension(out);
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
    ppp_clas_diagnostics::writePhasePhaseExtension(
        out,
        cpc_m,
        carrier_correction_m,
        cpc_minus_trop_m,
        phase_bias_m,
        windup_m,
        phase_compensation_m,
        raw_l_m,
        corrected_l_m);
}

std::ofstream* clasCodeDumpStream() {
    return ppp_clas_diagnostics::codeDumpStream();
}

bool selectedClasGeometryDumpTow(double tow) {
    return ppp_clas_diagnostics::selectedGeometryDumpTow(tow);
}

Vector3d clasGeometryDumpReceiverPosition(const Vector3d& fallback) {
    return ppp_clas_diagnostics::geometryDumpReceiverPosition(fallback);
}

bool usesClaslibCodePrcRows(const ppp_shared::PPPConfig& config) {
    return pppEnvOverrides().clas_code_row_full_prc &&
           config.clas_correction_application_policy ==
               ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::FULL_OSR;
}

bool usesClaslibBiasValidityRows(const ppp_shared::PPPConfig& config) {
    const auto& env = pppEnvOverrides();
    return config.use_clas_osr_filter && env.clas_dd_filter &&
           env.clas_code_row_bias_identity;
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

        for (int f = 0; f < std::max(osr.num_frequencies,
                                     osr.num_output_frequencies); ++f) {
            if (usesClaslibBiasValidityRows(config) &&
                (!osr.code_bias_present[f] || !osr.phase_bias_present[f])) {
                continue;
            }
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
                  << osr.iode_geometry_compensation_m << ','
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

        for (int f = 0; f < std::max(osr.num_frequencies,
                                     osr.num_output_frequencies); ++f) {
            if (usesClaslibBiasValidityRows(config) &&
                (!osr.code_bias_present[f] || !osr.phase_bias_present[f])) {
                continue;
            }
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
                  << osr.iode_geometry_compensation_m << ','
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
    if (freq_index < 0 ||
        freq_index >= std::max(osr.num_frequencies,
                               osr.num_output_frequencies)) {
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
    bool initialized_this_epoch = false;
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
        initialized_this_epoch = true;
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
        ambiguity_reset_variance,
        !clasMrtklibFloatParity(config));
    // MRTKLIB udpos_ppp() returns immediately after initializing x from the
    // current SPP solution. Applying F(dt) in that same epoch advances the
    // freshly seeded position by one extra velocity interval (~1 m at urban
    // driving speed) after every floatcnt mass reset.
    if (!(initialized_this_epoch && clasMrtklibFloatParity(config))) {
        const auto observed_satellite_list = obs.getSatellites();
        const std::set<SatelliteId> observed_satellites(
            observed_satellite_list.begin(), observed_satellite_list.end());
        predictFilterState(
            filter_state,
            config,
            dt,
            seed_solution.position_ecef,
            seed_solution.receiver_clock_bias,
            seed_solution.isValid(),
            &observed_satellites);
    }
    if (!clasMrtklibFloatParity(config)) {
        markSlipCompensationFromAmbiguities(
            obs, ambiguity_states, dispersion_compensation);
    }
    result.ready = true;
    result.initialized_this_epoch = initialized_this_epoch;
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
    const bool mrtklib_pva = clasMrtklibFloatParity(config);
    filter_state.accel_index = mrtklib_pva ? 6 : -1;
    filter_state.clock_index = mrtklib_pva ? 9 : 6;
    filter_state.glo_clock_index = mrtklib_pva ? 10 : 7;
    filter_state.trop_index = mrtklib_pva ? 11 : 8;
    const int fixed_state_count = mrtklib_pva ? 12 : 9;
    filter_state.gal_clock_index = -1;
    filter_state.qzs_clock_index = -1;
    filter_state.bds_clock_index = -1;
    int isb_start = fixed_state_count;
    if (pppEnvOverrides().clas_qzss_s_prn_fix && config.use_clas_osr_filter &&
        qzss_visible) {
        filter_state.qzs_clock_index = isb_start++;
    }
    const int n_isb = isb_start - fixed_state_count;
    const int n_iono = static_cast<int>(iono_satellites.size());
    const int base = fixed_state_count + n_isb + n_iono;
    filter_state.ionosphere_indices.clear();
    filter_state.adaptive_ionosphere_process_noise.clear();
    filter_state.state = VectorXd::Zero(base);
    filter_state.covariance = MatrixXd::Identity(base, base);
    filter_state.state.segment(0, 3) = seed_solution.position_ecef;
    if (mrtklib_pva) {
        filter_state.state.segment(filter_state.vel_index, 3) =
            seed_solution.velocity_ecef;
        filter_state.state.segment(filter_state.accel_index, 3).setConstant(1e-6);
    }
    filter_state.state(filter_state.clock_index) = seed_solution.receiver_clock_bias;
    filter_state.state(filter_state.glo_clock_index) = seed_solution.receiver_clock_bias;
    if (filter_state.qzs_clock_index >= 0) {
        filter_state.state(filter_state.qzs_clock_index) =
            seed_solution.receiver_clock_bias;
    }
    // benchmark/clas.toml uses troposphere="off". Keep the architectural
    // placeholder zero so MRTKLIB filter2 state compaction excludes it.
    filter_state.state(filter_state.trop_index) = mrtklib_pva
        ? 0.0
        : modeled_zenith_troposphere_delay_m;
    // MRTKLIB udpos_ppp()/udtrop() literal initial covariance.  The legacy
    // native CLAS path uses its independently tuned 10 m position sigma and
    // configurable wet-trop prior; the parity path must use VAR_POS=30^2 and
    // clas.toml initial_std.troposphere=0.005 m.
    constexpr double kMrtklibInitialPositionVariance = 30.0 * 30.0;
    constexpr double kMrtklibInitialTropVariance = 0.005 * 0.005;
    filter_state.covariance.block(0, 0, 3, 3) *=
        mrtklib_pva ? kMrtklibInitialPositionVariance
                    : config.clas_initial_position_variance;
    if (config.kinematic_mode && config.use_dynamics_model) {
        filter_state.covariance.block(
            filter_state.vel_index, filter_state.vel_index, 3, 3) *=
            mrtklib_pva ? 1.0 : config.initial_velocity_variance;
        if (mrtklib_pva) {
            filter_state.covariance.block(
                filter_state.accel_index, filter_state.accel_index, 3, 3) *= 1.0;
        }
    }
    filter_state.covariance(filter_state.clock_index, filter_state.clock_index) =
        config.clas_clock_variance;
    filter_state.covariance(filter_state.glo_clock_index,
                            filter_state.glo_clock_index) = config.clas_clock_variance;
    filter_state.covariance(filter_state.trop_index, filter_state.trop_index) =
        mrtklib_pva ? kMrtklibInitialTropVariance
                    : effectiveClasTropInitialVariance(config);
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
        filter_state.adaptive_ionosphere_process_noise[iono_satellites[index]] = 0.0;
        filter_state.state(state_index) = mrtklib_pva ? 1e-6 : 0.0;
        filter_state.covariance(state_index, state_index) = iono_initial_variance;
    }
    filter_state.amb_index = base;
    filter_state.total_states = base;
}

namespace {

constexpr double kClasNominalEpochIntervalS = 0.2;
constexpr double kClasOutageGapResetS = 2.0;
// Historical native fallback for paths that do not use MRTKLIB's explicit
// per-frequency observation-outage counter.
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

const Observation* findMrtklibParityRawObservation(
    const ObservationData& obs,
    const OSRCorrection& osr,
    int frequency_index) {
    if (osr.satellite.system != GNSSSystem::GPS) {
        return findOsrFrequencyObservation(obs, osr, frequency_index);
    }
    static constexpr char kGpsL2Priority[] = "PYWCMNDLXS";

    // MRTKLIB v0.5.1 getcodepri(): GPS slot 0 is L1C/A and slot 1 uses
    // "PYWCMNDLXS" priority.  The RINEX reader stores all tracking codes so
    // this lookup does not silently substitute the native L2C selection for
    // the L2W slot used by the reference run.
    if (const std::string* slot = obs.getRinexFrequencySlot(
            osr.satellite.system, frequency_index)) {
        return obs.getRinexTrackingObservation(osr.satellite, *slot);
    }
    if (frequency_index == 0) {
        if (const Observation* exact =
                obs.getRinexTrackingObservation(osr.satellite, "1C")) {
            return exact;
        }
    } else if (frequency_index == 1) {
        for (const char* priority = kGpsL2Priority; *priority != '\0'; ++priority) {
            const char tracking = *priority;
            const std::string key = std::string("2") + tracking;
            const Observation* exact =
                obs.getRinexTrackingObservation(osr.satellite, key);
            if (exact != nullptr && exact->has_pseudorange &&
                exact->has_carrier_phase) {
                return exact;
            }
        }
    }

    // Hand-built ObservationData in callers predating the complete RINEX
    // store has no auxiliary entries.  Accept it only when its provenance is
    // already the code that MRTKLIB would place in the slot.
    const Observation* selected =
        findOsrFrequencyObservation(obs, osr, frequency_index);
    if (selected == nullptr) return nullptr;
    if (frequency_index == 0) {
        if (obs.rinex_tracking_observations.empty()) return selected;
        return selected->pseudorange_observation_type == "C1C" &&
                       selected->carrier_phase_observation_type == "L1C"
                   ? selected
                   : nullptr;
    }
    if (frequency_index == 1) {
        const std::string& code = selected->pseudorange_observation_type;
        const std::string& phase = selected->carrier_phase_observation_type;
        if (obs.rinex_tracking_observations.empty() && code.empty() && phase.empty()) {
            return selected;
        }
        for (const char* priority = kGpsL2Priority; *priority != '\0'; ++priority) {
            const char tracking = *priority;
            const std::string suffix = std::string("2") + tracking;
            if (code == "C" + suffix && phase == "L" + suffix) {
                return selected;
            }
        }
        return nullptr;
    }
    return selected;
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
    const bool mrtklib_float_parity =
        config.clas_mrtklib_float_parity && config.kinematic_mode &&
        !config.low_dynamics_mode && config.use_clas_osr_filter &&
        config.use_dynamics_model;
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

    // MRTKLIB udbias_ppp() increments ssat[].outc[f] for every extant
    // ambiguity before looking at the current observations. ppp_rtk_pos()
    // clears it only for phase rows that survive the post-fit residual test.
    // With benchmark/clas.toml maxout=1, one rejected/missing epoch therefore
    // makes the next epoch start at outc=2 and resets lock to -minlock.
    if (mrtklib_float_parity) {
        for (auto& [_, ambiguity] : ambiguity_states) {
            ambiguity.outage_count = std::min(ambiguity.outage_count + 1, 1000000);
        }
    }

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        const Observation* l1_raw = mrtklib_float_parity
            ? findMrtklibParityRawObservation(obs, osr, 0)
            : findOsrFrequencyObservation(obs, osr, 0);
        const Observation* l2_raw = mrtklib_float_parity
            ? findMrtklibParityRawObservation(obs, osr, 1)
            : findOsrFrequencyObservation(obs, osr, 1);
        const auto raw_frequency_usable = [](const Observation* raw) {
            return raw != nullptr && raw->valid && raw->has_carrier_phase &&
                   raw->has_pseudorange;
        };
        const bool l1_raw_usable = raw_frequency_usable(l1_raw);
        const bool l2_raw_usable = raw_frequency_usable(l2_raw);
        // MRTKLIB detslp_ll() runs before the per-frequency outage reset and
        // treats either of the two low LLI bits as a slip. Preserve that LLI
        // when the other frequency is missing: the outage branch below must
        // not return before resetting the still-observed slipping frequency.
        const bool l1_lli_slip =
            mrtklib_float_parity && l1_raw_usable && (l1_raw->lli & 0x03U) != 0;
        const bool l2_lli_slip =
            mrtklib_float_parity && l2_raw_usable && (l2_raw->lli & 0x03U) != 0;
        auto& ambiguity = ambiguity_states[osr.satellite];
        bool per_sat_outage = false;
        bool l1_outage_overflow = false;
        bool l2_outage_overflow = false;
        int l1_outage_count = ambiguity.outage_count;
        int l2_outage_count = 0;
        const SatelliteId l2_ambiguity_satellite(
            osr.satellite.system,
            static_cast<uint8_t>(std::min(
                255, static_cast<int>(osr.satellite.prn) + 100)));
        const auto l2_ambiguity_it = ambiguity_states.find(l2_ambiguity_satellite);
        if (l2_ambiguity_it != ambiguity_states.end()) {
            l2_outage_count = l2_ambiguity_it->second.outage_count;
        }
        if (mrtklib_float_parity && !outage_gap) {
            constexpr int kMrtklibMaxOut = 1;
            // clas_osr_zdres() does not create Galileo filter rows and uses
            // only QZSS frequency slot zero.  Their absent ambiguity states
            // therefore cannot overflow outc[] or set pbreset[] in CLASLIB.
            // Native retains the extra observations for OSR diagnostics and
            // dispersion compensation, so gate the outage lifecycle to the
            // subset that the literal filter actually tracks.
            const bool tracks_l1_filter_state =
                osr.satellite.system != GNSSSystem::Galileo;
            const bool tracks_l2_filter_state =
                tracks_l1_filter_state &&
                osr.satellite.system != GNSSSystem::QZSS;
            l1_outage_overflow =
                tracks_l1_filter_state &&
                l1_outage_count > kMrtklibMaxOut;
            l2_outage_overflow =
                tracks_l2_filter_state &&
                l2_outage_count > kMrtklibMaxOut;
            per_sat_outage = l1_outage_overflow || l2_outage_overflow;
            if (per_sat_outage) {
                ++stats.per_sat_outage_resets;
            }
        } else if (config.use_clas_osr_filter && !outage_gap &&
                   ambiguity.last_time.week > 0 && dt_seconds > 0.0) {
            const double sat_gap_s = obs.time - ambiguity.last_time;
            if (sat_gap_s >
                kClasPerSatOutageEpochs * dt_seconds + 0.5 * dt_seconds) {
                per_sat_outage = true;
                ++stats.per_sat_outage_resets;
            }
        }

        // detslp_ll() precedes udbias_ppp()'s dual-frequency checks.  A valid
        // LLI on the one frequency that remains available must therefore
        // reset that frequency even when neither outage counter has crossed
        // maxout yet (G04 at tokyo/run2 TOW 177084.4 is L1C-only with
        // LLI=1).  Keep the complete-pair case in the combined GF/LLI path
        // below, where both CLAS ambiguity states are reset together.
        const bool incomplete_pair =
            osr.num_frequencies < 2 || !l1_raw_usable || !l2_raw_usable;
        const bool incomplete_pair_lli =
            (l1_lli_slip || l2_lli_slip) && incomplete_pair;
        // udion() follows udbias_ppp() and resets the ionosphere state when
        // any currently selected frequency has exceeded maxout.  This is
        // independent of whether the complete pair continues into GF slip
        // detection below.
        const bool selected_frequency_outage =
            (l1_outage_overflow && l1_raw_usable) ||
            (l2_outage_overflow && osr.num_frequencies > 1 &&
             l2_raw_usable);
        const auto iono_it =
            filter_state.ionosphere_indices.find(osr.satellite);
        if (mrtklib_float_parity && selected_frequency_outage &&
            iono_it != filter_state.ionosphere_indices.end() &&
            iono_it->second >= 0 &&
            iono_it->second < filter_state.total_states) {
            constexpr double kMrtklibInitialIonoStateM = 1e-6;
            constexpr double kMrtklibInitialIonoVariance = 0.01 * 0.01;
            const int iono_index = iono_it->second;
            filter_state.state(iono_index) = kMrtklibInitialIonoStateM;
            filter_state.covariance.row(iono_index).setZero();
            filter_state.covariance.col(iono_index).setZero();
            filter_state.covariance(iono_index, iono_index) =
                kMrtklibInitialIonoVariance;
            filter_state.adaptive_ionosphere_process_noise[osr.satellite] =
                0.0;
        }
        if (mrtklib_float_parity &&
            (per_sat_outage || incomplete_pair_lli)) {
            // udbias_ppp() performs this frequency-specific outage reset
            // before it asks whether both L1 and L2 are currently usable.
            // Keeping the reset behind the dual-frequency slip detector
            // leaves a returning L1-only satellite (for example G11 at
            // tokyo/run2 TOW 177091.4) anchored to its stale ambiguity.
            const auto reset_frequency = [&](
                const SatelliteId& ambiguity_satellite,
                SignalType signal,
                int outage_count) {
                if (ambiguity_reset_function) {
                    ambiguity_reset_function(ambiguity_satellite, signal);
                }
                constexpr int kMrtklibMinLock = 5;
                auto& reset_ambiguity = ambiguity_states[ambiguity_satellite];
                reset_ambiguity.lock_count = -kMrtklibMinLock;
                reset_ambiguity.outage_count = outage_count;
            };
            // With a complete L1/L2 pair, MRTKLIB's geometry-free detector
            // runs immediately after detslp_ll().  On a returning frequency
            // the LLI epoch therefore marks both frequency slips (G09 at
            // tokyo/run2 TOW 177090.8 publishes slip=1/lock=-4 on L1 and L2).
            // Keep a genuinely incomplete pair frequency-specific.
            const bool complete_pair_lli =
                !incomplete_pair && (l1_lli_slip || l2_lli_slip);
            const bool reset_l1 =
                l1_outage_overflow || l1_lli_slip || complete_pair_lli;
            const bool reset_l2 =
                l2_outage_overflow || l2_lli_slip || complete_pair_lli;
            if (reset_l1) {
                reset_frequency(osr.satellite,
                                l1_lli_slip ? l1_raw->signal
                                            : SignalType::SIGNAL_TYPE_COUNT,
                                l1_outage_count);
            }
            if (reset_l2) {
                reset_frequency(l2_ambiguity_satellite,
                                l2_lli_slip ? l2_raw->signal
                                            : SignalType::SIGNAL_TYPE_COUNT,
                                l2_outage_count);
            }
            if (l1_lli_slip || l2_lli_slip) {
                ++stats.lli_count;
                dispersion_compensation[osr.satellite].slip = {true, true};
                auto repair_it = phase_bias_repair.find(osr.satellite);
                if (repair_it != phase_bias_repair.end()) {
                    resetClasPhaseBiasRepair(repair_it->second);
                }
            }
            ++stats.total_resets;
            stats.reset_satellites.insert(osr.satellite);
            if (debug_enabled) {
                std::cerr << "[CLAS-SLIP] " << osr.satellite.toString()
                          << " tow=" << obs.time.tow
                          << " reason="
                          << (per_sat_outage ? "outage_sat" : "")
                          << (per_sat_outage &&
                                      (l1_lli_slip || l2_lli_slip)
                                  ? "+"
                                  : "")
                          << ((l1_lli_slip || l2_lli_slip) ? "lli" : "")
                          << " freq="
                          << (reset_l1 && reset_l2
                                  ? "L1+L2"
                                  : (reset_l1 ? "L1" : "L2"))
                          << " dt=" << dt_seconds << "\n";
            }
            continue;
        }

        if (incomplete_pair) {
            continue;
        }

        const double f1 = osr.frequencies[0];
        const double f2 = osr.frequencies[1];
        if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1e6) {
            continue;
        }

        // MRTKLIB detslp_gf() / gfmeas_L1L2() operates on the raw carrier
        // phases.  Applying the broadcast phase biases here turns a CSSR
        // phase-bias update into an apparent geometry-free jump and resets
        // otherwise continuous ambiguities (G04 at tow 177067.4 in
        // tokyo/run2).  Biases belong to the corrected measurement model,
        // not to receiver cycle-slip detection.
        const double l1_m = l1_raw->carrier_phase * osr.wavelengths[0] -
            (mrtklib_float_parity ? 0.0 : osr.phase_bias_m[0]);
        const double l2_m = l2_raw->carrier_phase * osr.wavelengths[1] -
            (mrtklib_float_parity ? 0.0 : osr.phase_bias_m[1]);
        const double gf_m = l1_m - l2_m;
        // RTKLIB's GPS frequency-2 observation slot follows its RINEX code
        // priority (the tokyo receiver supplies L2W when usable).  If L2W is
        // absent, gfmeas_L1L2() sees obs->L[1] == 0 and leaves the saved GF
        // value untouched; it does not fall through to the simultaneously
        // recorded L2L signal.  Native's generic secondary-signal selector
        // does make that L2L fallback, so gate only the parity slip detector
        // to the literal RTKLIB slot identity.
        const bool mrtklib_gf_pair_usable = true;
        const double p1 = l1_raw->pseudorange - osr.code_bias_m[0];
        const double p2 = l2_raw->pseudorange - osr.code_bias_m[1];
        const double mw_m = (f1 * l1_m - f2 * l2_m) / (f1 - f2) -
                              (f1 * p1 + f2 * p2) / (f1 + f2);
        const double lambda_wl = constants::SPEED_OF_LIGHT / std::abs(f1 - f2);
        const double mw_cycles = mw_m / lambda_wl;

        bool lli_slip = l1_raw->loss_of_lock || l2_raw->loss_of_lock;
        bool gf_slip = false;
        bool mw_slip = false;
        bool code_change_slip = false;

        // MRTKLIB detslp_code(): changing the tracked observation code on a
        // frequency invalidates that frequency's phase-bias state.  CLAS uses
        // two frequencies and resets both together, matching the existing
        // real/pseudo-satellite ambiguity layout.  The first observed code
        // only seeds history and is not a slip.
        if (mrtklib_float_parity) {
            const std::array<SignalType, 2> current_signals{
                l1_raw->signal, l2_raw->signal};
            const std::array<std::string, 2> current_carrier_types{
                l1_raw->carrier_phase_observation_type,
                l2_raw->carrier_phase_observation_type};
            for (int frequency = 0; frequency < 2; ++frequency) {
                if (ambiguity.has_last_observation_signal[frequency] &&
                    (ambiguity.last_observation_signals[frequency] !=
                         current_signals[frequency] ||
                     ambiguity.last_carrier_observation_types[frequency] !=
                         current_carrier_types[frequency])) {
                    code_change_slip = true;
                }
                ambiguity.last_observation_signals[frequency] =
                    current_signals[frequency];
                ambiguity.last_carrier_observation_types[frequency] =
                    current_carrier_types[frequency];
                ambiguity.has_last_observation_signal[frequency] = true;
            }
        }

        // MRTKLIB per-satellite outage reset (mrtk_ppp_rtk.c:865-875,
        // clas.toml out_count = 1): a satellite whose observations were
        // missing (or whose measurement update failed) for more than maxout
        // consecutive epochs gets its ambiguity fully reset and lock
        // restarted. Without this a satellite blocked for a few seconds
        // returns with its stale ambiguity still AR-eligible -- the GF/MW
        // detectors compare against seconds-old history and can miss the
        // integer break, producing self-consistent wrong fixes. The global
        // outage_gap branch below only covers receiver-wide gaps.
        if (outage_gap) {
            ++stats.outage_resets;
            ambiguity.has_last_geometry_free = false;
            ambiguity.has_last_melbourne_wubbena = false;
            clearClasWlnlMwState(ambiguity);
        } else {
            if (mrtklib_gf_pair_usable &&
                ambiguity.has_last_geometry_free &&
                std::isfinite(gf_m) &&
                std::abs(gf_m - ambiguity.last_geometry_free_m) > gf_threshold_m) {
                gf_slip = true;
            }
            // MW-mean slip is a native-only detector. MRTKLIB detslp_gf uses
            // LLI + geometry-free phase here and does not reset ambiguities
            // from a Melbourne-Wubbena running-mean excursion.
            const bool wl_mean_stable =
                ambiguity.mw_count >= config.wl_min_averaging_epochs &&
                std::isfinite(ambiguity.mw_mean_cycles) &&
                std::abs(ambiguity.mw_mean_cycles -
                         std::round(ambiguity.mw_mean_cycles)) < 0.25;
            if (!mrtklib_float_parity &&
                !ambiguity.wl_is_fixed && !wl_mean_stable) {
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

        if (mrtklib_gf_pair_usable && std::isfinite(gf_m)) {
            ambiguity.last_geometry_free_m = gf_m;
            ambiguity.has_last_geometry_free = true;
        }
        if (std::isfinite(mw_m)) {
            ambiguity.last_melbourne_wubbena_m = mw_m;
            ambiguity.has_last_melbourne_wubbena = true;
        }

        const bool combined_phase_reset =
            lli_slip || gf_slip || mw_slip || code_change_slip ||
            outage_resets_ambiguity;
        if (!lli_slip && !gf_slip && !mw_slip && !code_change_slip &&
            !outage_resets_ambiguity && !per_sat_outage) {
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
        if (code_change_slip) {
            ++stats.code_change_count;
        }
        ++stats.total_resets;
        stats.reset_satellites.insert(osr.satellite);

        ambiguity.needs_reinitialization = true;
        ambiguity.has_last_slip_time = true;
        ambiguity.last_slip_time = obs.time;
        clearClasWlnlMwState(ambiguity);
        ambiguity.has_last_geometry_free = false;
        ambiguity.has_last_melbourne_wubbena = false;

        // MRTKLIB compensatedisp() latches ssat.slip, which is set only by
        // the receiver cycle-slip detectors.  An outage resets the ambiguity
        // and lock in udbias_ppp(), but it does not set comp_slip.  Keeping
        // those lifecycles separate is essential: treating an ordinary
        // post-fit rejection/outage as a carrier slip suppresses valid
        // measurement-based dispersion compensation until the next STEC
        // bank.
        if (lli_slip || gf_slip || mw_slip || code_change_slip) {
            dispersion_compensation[osr.satellite].slip = {true, true};
        }
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
            if (mrtklib_float_parity) {
                // udbias_ppp(): a slip/outage reset starts at -minlock;
                // subsequent accepted samples increment it and ddmat admits
                // the ambiguity only after lock becomes positive.
                constexpr int kMrtklibMinLock = 5;
                ambiguity_states[osr.satellite].lock_count = -kMrtklibMinLock;
                ambiguity_states[l2_satellite].lock_count = -kMrtklibMinLock;
                // resetAmbiguity() value-initializes the bookkeeping, but
                // MRTKLIB retains each frequency's current counter until a
                // valid post-fit phase row clears it.
                ambiguity_states[osr.satellite].outage_count = l1_outage_count;
                ambiguity_states[l2_satellite].outage_count = l2_outage_count;
            }
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
            if (code_change_slip) {
                if (!reason.empty()) {
                    reason += "+";
                }
                reason += "code";
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

    if (stats.total_resets > 0 && !mrtklib_float_parity) {
        syncSlipState(
            obs,
            filter_state,
            ambiguity_states,
            dispersion_compensation,
            phase_bias_repair,
            ambiguity_reset_variance,
            true);
    }

    return stats;
}

void syncSlipState(
    const ObservationData& obs,
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    double ambiguity_reset_variance,
    bool mark_dispersion_slip) {
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
            // detectClasCycleSlips() has already applied MRTKLIB's
            // lock=-minlock and retained its overflowed outc. This secondary
            // state/covariance synchronization must not turn a just-reset
            // ambiguity back into an immediately AR-eligible lock=0 state.
            const int reset_lock_count = ambiguity.lock_count;
            const int reset_outage_count = ambiguity.outage_count;
            ambiguity = ppp_shared::PPPAmbiguityInfo{};
            ambiguity.needs_reinitialization = true;
            if (reset_lock_count < 0) {
                ambiguity.lock_count = reset_lock_count;
            }
            ambiguity.outage_count = reset_outage_count;

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

        if (mark_dispersion_slip) {
            dispersion_compensation[satellite].slip = {true, true};
        }
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
    bool seed_valid,
    const std::set<SatelliteId>* observed_satellites) {
    const int nx = filter_state.total_states;
    const bool kinematic_white_noise =
        config.kinematic_mode && !config.low_dynamics_mode;
    const bool use_dynamic_prediction =
        kinematic_white_noise && config.use_dynamics_model;

    MatrixXd F = MatrixXd::Identity(nx, nx);
    MatrixXd covariance_F = MatrixXd::Identity(nx, nx);
    if (use_dynamic_prediction) {
        F.block(filter_state.pos_index, filter_state.vel_index, 3, 3) =
            MatrixXd::Identity(3, 3) * dt;
        covariance_F.block(
            filter_state.pos_index, filter_state.vel_index, 3, 3) =
            MatrixXd::Identity(3, 3) * dt;
        if (clasMrtklibFloatParity(config) && filter_state.accel_index >= 0) {
            F.block(filter_state.vel_index, filter_state.accel_index, 3, 3) =
                MatrixXd::Identity(3, 3) * dt;
            F.block(filter_state.pos_index, filter_state.accel_index, 3, 3) =
                MatrixXd::Identity(3, 3) * (dt * dt / 2.0);
            covariance_F.block(
                filter_state.vel_index, filter_state.accel_index, 3, 3) =
                MatrixXd::Identity(3, 3) * dt;
        }
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
    if (use_dynamic_prediction && clasMrtklibFloatParity(config) &&
        filter_state.accel_index >= 0) {
        // MRTKLIB v0.5.1 udpos_ppp(): acceleration random walk is defined
        // in local ENU (horizontal prn[3]=0.2, vertical prn[4]=0.1) and
        // rotated to ECEF before being added to P[6:9,6:9].
        double lat = 0.0, lon = 0.0, height = 0.0;
        ecef2geodetic(filter_state.state.segment(filter_state.pos_index, 3),
                      lat, lon, height);
        (void)height;
        Matrix3d enu_to_ecef;
        const Vector3d east = enu2ecef(Vector3d::UnitX(), lat, lon);
        const Vector3d north = enu2ecef(Vector3d::UnitY(), lat, lon);
        const Vector3d up = enu2ecef(Vector3d::UnitZ(), lat, lon);
        enu_to_ecef.col(0) = east;
        enu_to_ecef.col(1) = north;
        enu_to_ecef.col(2) = up;
        Matrix3d q_enu = Matrix3d::Zero();
        q_enu(0, 0) = q_enu(1, 1) = 0.2 * 0.2 * std::abs(dt);
        q_enu(2, 2) = 0.1 * 0.1 * std::abs(dt);
        Q.block(filter_state.accel_index, filter_state.accel_index, 3, 3) =
            enu_to_ecef * q_enu * enu_to_ecef.transpose();
    } else if (use_dynamic_prediction) {
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
    // MRTKLIB literal-port track: ionosphere="est-adaptive". udion() keeps a
    // persistent per-satellite Q rate, clamps it to [0.001^2, 0.05^2], and
    // adds Q*dt only for satellites observed in the current epoch. filter2()
    // refreshes that rate after the measurement update below. Bias remains a
    // fixed 0.001^2 cycle^2/s random walk.
    const bool mrtklib_parity = clasMrtklibFloatParity(config);
    constexpr double kMrtklibIonoProcessNoise = 0.001 * 0.001;   // (m^2/s)
    constexpr double kMrtklibBiasProcessNoise = 0.001 * 0.001;   // (m^2/s)
    if (config.estimate_ionosphere) {
        for (const auto& [satellite, state_index] :
             filter_state.ionosphere_indices) {
            if (state_index >= 0 && state_index < nx) {
                if (mrtklib_parity) {
                    if (observed_satellites != nullptr &&
                        observed_satellites->count(satellite) == 0) {
                        continue;
                    }
                    double& iono_process_noise =
                        filter_state.adaptive_ionosphere_process_noise[satellite];
                    if (iono_process_noise == 0.0) {
                        iono_process_noise = kMrtklibIonoProcessNoise;
                    } else {
                        iono_process_noise = std::clamp(
                            iono_process_noise,
                            kMrtklibIonoProcessNoise,
                            0.05 * 0.05);
                    }
                    Q(state_index, state_index) =
                        iono_process_noise * dt;
                } else {
                    Q(state_index, state_index) =
                        effectiveClasIonoProcessNoise(config) * dt;
                }
            }
        }
    }
    const double bias_process_noise = mrtklib_parity
        ? kMrtklibBiasProcessNoise
        : config.process_noise_ambiguity;
    for (const auto& [ambiguity_satellite, state_index] :
         filter_state.ambiguity_indices) {
        if (state_index >= 0 && state_index < nx) {
            double scale2 = 1.0;
            if (mrtklib_parity) {
                const auto wavelength_it =
                    filter_state.ambiguity_wavelengths_m.find(ambiguity_satellite);
                if (wavelength_it != filter_state.ambiguity_wavelengths_m.end() &&
                    wavelength_it->second > 0.0) {
                    scale2 = wavelength_it->second * wavelength_it->second;
                }
            }
            Q(state_index, state_index) = bias_process_noise * scale2 * dt;
        }
    }
    // MRTKLIB v0.5.1 udpos_ppp() deliberately does not use the F employed
    // above for its covariance propagation.  Its two in-place loops add
    // pos<-vel and vel<-acc on each side of P, but omit the dt^2/2
    // pos<-acc element that is present in the state prediction.  Preserve
    // that literal asymmetry on the parity path; using the mathematically
    // complete F here changes the carrier Kalman gains enough to alter the
    // postfit vsat admission and therefore the float-reset cadence.
    filter_state.covariance =
        covariance_F * filter_state.covariance * covariance_F.transpose() + Q;
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
        if (osr.wavelengths[0] > 0.0) {
            filter_state.ambiguity_wavelengths_m[osr.satellite] =
                osr.wavelengths[0];
        }
        if (osr.num_frequencies >= 2) {
            const uint8_t l2_prn =
                static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
            const SatelliteId l2_satellite(osr.satellite.system, l2_prn);
            allocate_ambiguity(l2_satellite);
            if (osr.wavelengths[1] > 0.0) {
                filter_state.ambiguity_wavelengths_m[l2_satellite] =
                    osr.wavelengths[1];
            }
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
    bool debug_enabled,
    int reference_rank) {
    (void)receiver_clock_m;
    MeasurementBuildResult result;
    const bool stec_constraint = usesClasStecConstraint(config);
    const bool mrtklib_parity = clasMrtklibFloatParity(config);
    std::map<SatelliteId, double> iono_state_targets_m;
    std::map<SatelliteId, double> iono_state_variances_m2;
    std::set<int> fresh_ambiguity_indices;
    // MRTKLIB v0.5.1 udbias_ppp() (mrtk_ppp_rtk.c:761-797): initialize
    // each phase-bias state from raw phase-minus-code, after removing the
    // common offset inferred from already-initialized satellites. Native
    // ambiguity states are stored in metres (MRTKLIB stores cycles), so the
    // literal equivalent is L*lambda-P-com_bias. initx() also clears every
    // cross-covariance of a newly initialized state.
    if (mrtklib_parity && ambiguity_reset_function) {
        struct BiasSeed {
            SatelliteId satellite;
            int state_index = -1;
            double phase_minus_code_m = 0.0;
            double wavelength_m = 0.0;
        };
        std::array<std::vector<BiasSeed>, OSR_MAX_FREQ> bias_seeds;
        for (const auto& osr : osr_corrections) {
            if (!osr.valid) continue;
            // The v0.5.1 CLAS residual path leaves Galileo unusable on this
            // dataset (ssat.vsat remains zero); retain those states for the
            // common layout but do not initialize them from absent rows.
            if (osr.satellite.system == GNSSSystem::Galileo) continue;
            for (int f = 0; f < osr.num_frequencies; ++f) {
                if (osr.satellite.system == GNSSSystem::QZSS && f > 0) {
                    continue;
                }
                const Observation* raw =
                    findMrtklibParityRawObservation(obs, osr, f);
                if (raw == nullptr || !raw->valid || !raw->has_carrier_phase ||
                    !raw->has_pseudorange || !std::isfinite(raw->carrier_phase) ||
                    !std::isfinite(raw->pseudorange) || osr.wavelengths[f] <= 0.0) {
                    continue;
                }
                const uint8_t amb_prn = f == 0 ? osr.satellite.prn
                    : static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
                const auto amb_it = filter_state.ambiguity_indices.find(
                    SatelliteId(osr.satellite.system, amb_prn));
                if (amb_it == filter_state.ambiguity_indices.end()) continue;
                bias_seeds[static_cast<size_t>(f)].push_back({
                    SatelliteId(osr.satellite.system, amb_prn),
                    amb_it->second,
                    raw->carrier_phase * osr.wavelengths[f] - raw->pseudorange,
                    osr.wavelengths[f]});
            }
        }
        for (auto& frequency_seeds : bias_seeds) {
            double offset_sum_m = 0.0;
            int offset_count = 0;
            for (const auto& seed : frequency_seeds) {
                if (filter_state.state(seed.state_index) != 0.0 &&
                    filter_state.covariance(seed.state_index, seed.state_index) <
                        config.clas_ambiguity_reinit_threshold) {
                    offset_sum_m += seed.phase_minus_code_m -
                                    filter_state.state(seed.state_index);
                    ++offset_count;
                }
            }
            const double common_bias_m = offset_count > 0
                ? offset_sum_m / static_cast<double>(offset_count)
                : 0.0;
            if (debug_enabled &&
                std::abs(obs.time.tow - 177091.4) < 0.01) {
                std::cerr << "[CLAS-BIAS-SEED] tow=" << obs.time.tow
                          << " common=" << common_bias_m
                          << " count=" << offset_count << "\n";
                for (const auto& seed : frequency_seeds) {
                    std::cerr << "[CLAS-BIAS-SEED] sat="
                              << seed.satellite.toString()
                              << " raw=" << seed.phase_minus_code_m
                              << " state=" << filter_state.state(seed.state_index)
                              << " var="
                              << filter_state.covariance(seed.state_index,
                                                         seed.state_index)
                              << "\n";
                }
            }
            for (const auto& seed : frequency_seeds) {
                if (filter_state.state(seed.state_index) != 0.0 &&
                    filter_state.covariance(seed.state_index, seed.state_index) <
                        config.clas_ambiguity_reinit_threshold) {
                    continue;
                }
                filter_state.state(seed.state_index) =
                    seed.phase_minus_code_m - common_bias_m;
                filter_state.covariance.row(seed.state_index).setZero();
                filter_state.covariance.col(seed.state_index).setZero();
                filter_state.covariance(seed.state_index, seed.state_index) =
                    1e4 * seed.wavelength_m * seed.wavelength_m;
                fresh_ambiguity_indices.insert(seed.state_index);
            }
        }
    }

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        if (mrtklib_parity &&
            osr.satellite.system == GNSSSystem::Galileo) {
            continue;
        }
        // MRTKLIB clas_osr_corrmeas() requires STEC data for this satellite
        // at the selected CLAS grid and returns 0 on search_data "no sat
        // data" (mrtk_clas_osr.c:575-584). Native historically falls back
        // across grids/means, which admitted extra DD rows (G03 at tow
        // 177000). Preserve that fallback outside the literal parity path.
        if (mrtklib_parity &&
            ((osr.atmos_stec_satellite_membership_known &&
              !osr.atmos_stec_has_satellite) ||
             (osr.atmos_grid_satellite_membership_known &&
              !osr.atmos_grid_has_satellite))) {
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
        // MRTKLIB ionmapf(): single-layer shell at HION=350 km. ddres()
        // multiplies every estimated vertical L1 ionosphere state by this
        // satellite-specific slant factor before applying the frequency
        // ratio. Native historically used only the frequency ratio.
        double receiver_lat = 0.0;
        double receiver_lon = 0.0;
        double receiver_height = 0.0;
        ecef2geodetic(receiver_position, receiver_lat, receiver_lon,
                     receiver_height);
        (void)receiver_lat;
        (void)receiver_lon;
        constexpr double kMrtklibIonosphereHeightM = 350000.0;
        const double ionosphere_mapping =
            receiver_height >= kMrtklibIonosphereHeightM
                ? 1.0
                : 1.0 / std::cos(std::asin(
                      (constants::WGS84_A + receiver_height) /
                      (constants::WGS84_A + kMrtklibIonosphereHeightM) *
                      std::cos(osr.elevation)));

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
            if (mrtklib_parity &&
                osr.satellite.system == GNSSSystem::QZSS && f > 0) {
                continue;
            }
            raw_observations[static_cast<size_t>(f)] =
                findOsrFrequencyObservation(obs, osr, f);
        }

        for (int f = 0; f < osr.num_frequencies; ++f) {
            const Observation* raw = raw_observations[static_cast<size_t>(f)];
            if (!raw || !raw->valid) {
                continue;
            }
            // clas_osr_zdres() skips the complete frequency cell when either
            // satellite code bias or phase bias is CSSRINVALID.  Applying a
            // sibling signal's collapsed RTCM-band value would admit rows
            // that do not exist in MRTKLIB (for example G04 C2L at 177067.2).
            if ((mrtklib_parity || usesClaslibBiasValidityRows(config)) &&
                (!osr.code_bias_present[f] || !osr.phase_bias_present[f])) {
                continue;
            }
            // MRTKLIB v0.5.1 clas_osr_zdres() multiplies its uint16 SNR
            // (stored in 0.001 dB-Hz units) by the legacy 0.25 factor before
            // testsnr().  The resulting thousands-of-dB-Hz value means the
            // configured mask never rejects a CLAS measurement.  Preserve
            // that observable behavior on the literal parity path.
            if (!mrtklib_parity &&
                config.kinematic_mode &&
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
            const double effective_iono_scale =
                iono_scale * (mrtklib_parity ? ionosphere_mapping : 1.0);
            if (mrtklib_parity) {
                applied_corrections.pseudorange_correction_m = osr.PRC[f];
                applied_corrections.carrier_phase_correction_m = osr.CPC[f];
            }
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
                    (mrtklib_parity || claslib_code_prc) ? 0.0 : trop_modeled;
                const double code_trop_mapping =
                    (mrtklib_parity || claslib_code_prc) ? 0.0 : trop_mapping;
                const double p_corr =
                    raw->pseudorange - applied_corrections.pseudorange_correction_m;
                const double predicted =
                    geo - sat_clk_m + receiver_clock_m + code_trop_modeled +
                    effective_iono_scale * iono_state_m;
                const double residual = p_corr - predicted;
                const double el_weight = elevationWeight(osr.elevation);

                MeasurementRow row;
                row.H = Eigen::RowVectorXd::Zero(filter_state.total_states);
                row.H.segment(0, 3) = -los.transpose();
                row.H(receiver_clock_index) = 1.0;
                row.H(filter_state.trop_index) = code_trop_mapping;
                if (use_residual_iono_state) {
                    row.H(iono_state_index) = effective_iono_scale;
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
                    !mrtklib_parity && suppressesClasAmbDatumPhaseTrop(
                        config.clas_correction_application_policy);
                const bool residual_amb_datum_phase_trop =
                    !mrtklib_parity && usesResidualClasAmbDatumPhaseTrop(
                        config.clas_correction_application_policy);
                const double phase_trop_modeled = mrtklib_parity
                    ? 0.0
                    : (claslib_amb_datum_phase
                    ? 0.0
                    : (residual_amb_datum_phase_trop
                          ? trop_modeled - osr.trop_correction_m
                          : trop_modeled));
                const double phase_trop_partial =
                    (mrtklib_parity || claslib_amb_datum_phase)
                        ? 0.0 : trop_mapping;

                const uint8_t amb_prn = f == 0 ? osr.satellite.prn
                    : static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
                const SatelliteId amb_sat(osr.satellite.system, amb_prn);
                const auto amb_it = filter_state.ambiguity_indices.find(amb_sat);
                if (amb_it == filter_state.ambiguity_indices.end()) {
                    continue;
                }
                const int amb_idx = amb_it->second;

                // On the MRTKLIB parity path slip/outage handling has already
                // reset the bias before the udbias_ppp-style seeding pass at
                // the top of this function.  Resetting it again here would
                // erase the freshly seeded L*lambda-P state, leave x[IB]==0
                // (therefore inactive in filter2()), and force the several-
                // metre phase innovation into position/ionosphere instead.
                // Legacy CLAS keeps its historical late-reset ordering.
                if (!mrtklib_parity && raw->loss_of_lock &&
                    ambiguity_reset_function) {
                    ambiguity_reset_function(amb_sat, raw->signal);
                }

                if (!mrtklib_parity &&
                    filter_state.covariance(amb_idx, amb_idx) >=
                        config.clas_ambiguity_reinit_threshold) {
                    filter_state.state(amb_idx) =
                        l_corr - (geo - sat_clk_m + receiver_clock_m + phase_trop_modeled
                                  - effective_iono_scale * iono_state_m);
                }

                const double predicted_no_amb =
                    geo - sat_clk_m + receiver_clock_m + phase_trop_modeled
                    - effective_iono_scale * iono_state_m;
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
                    row.H(iono_state_index) = -effective_iono_scale;
                    result.observed_iono_states.insert(osr.satellite);
                }
                row.H(amb_idx) = 1.0;
                row.residual = residual;
                row.variance =
                    clasPhaseVariance(config, osr.elevation, f, osr.frequencies[f]);
                row.satellite = osr.satellite;
                row.is_phase = true;
                row.ambiguity_fresh = fresh_ambiguity_indices.contains(amb_idx);
                row.freq_index = f;
                row.ambiguity_index = amb_idx;
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

    if (!mrtklib_parity &&
        config.estimate_troposphere && !epoch_atmos.empty() &&
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

    if (config.estimate_ionosphere && !mrtklib_parity) {
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

    // Same-system differencing of the zero-difference rows. MRTKLIB ddres()
    // differences both phase and code by system/frequency, cancelling the
    // receiver clock exactly (mrtk_ppp_rtk.c:1295-1451). The legacy native
    // path keeps code undifferenced; the literal-port parity path differences
    // both families and carries the shared-reference covariance into ddcov.
    if (config.use_clas_osr_filter) {
        // Build elevation map for reference satellite selection
        std::map<SatelliteId, double> elevation_map;
        for (const auto& osr : osr_corrections) {
            if (osr.valid) {
                elevation_map[osr.satellite] = osr.elevation;
            }
        }

        // Group same-system measurements by signal. MRTKLIB validobs()
        // requires the corresponding phase observation before admitting a
        // code row (mrtk_ppp_rtk.c:896-899).
        struct SdGroupKey {
            GNSSSystem system;
            int freq_index;
            bool is_phase;
            bool operator<(const SdGroupKey& rhs) const {
                if (system != rhs.system) return system < rhs.system;
                // MRTKLIB ddres() iterates f=0..2*nf: every carrier-phase
                // frequency first, followed by every code frequency.  This
                // order also fixes the innovation/LU pivot order used by the
                // literal filter2 update.
                if (is_phase != rhs.is_phase) return is_phase > rhs.is_phase;
                return freq_index < rhs.freq_index;
            }
        };
        const bool mrtklib_dd = clasMrtklibFloatParity(config);
        const bool code_sd = mrtklib_dd || pppEnvOverrides().clas_code_sd;
        std::set<std::pair<SatelliteId, int>> phase_observations;
        if (mrtklib_dd) {
            for (const auto& measurement : result.measurements) {
                if (measurement.is_phase && measurement.freq_index >= 0) {
                    phase_observations.insert(
                        {measurement.satellite, measurement.freq_index});
                }
            }
        }
        std::map<SdGroupKey, std::vector<size_t>> sd_groups;
        for (size_t i = 0; i < result.measurements.size(); ++i) {
            const auto& m = result.measurements[i];
            if (m.freq_index < 0) continue;
            if (!m.is_phase && !code_sd) continue;
            if (mrtklib_dd && !m.is_phase &&
                phase_observations.count({m.satellite, m.freq_index}) == 0) {
                continue;
            }
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

        // Form DD rows for each selected group. MRTKLIB searches with >=, so
        // equal-elevation ties select the later observation (lines 1303-1321).
        int dd_covariance_block = 0;
        for (auto& [group_key, member_indices] : sd_groups) {
            if (member_indices.size() < 2) continue;

            if (mrtklib_dd && ppp_internal::pppDebugEnabled()) {
                std::cerr << "[CLAS-DD-GROUP] sys="
                          << static_cast<int>(group_key.system)
                          << " f=" << group_key.freq_index
                          << " phase=" << group_key.is_phase
                          << " sats=";
                for (size_t idx : member_indices) {
                    std::cerr << result.measurements[idx].satellite.toString() << ' ';
                }
                std::cerr << "\n";
            }

            // MRTKLIB ddres(niter): first use the highest-elevation member;
            // on the second reference pass exclude that member and select the
            // next highest. Equal-elevation ties prefer the later observation.
            auto ranked_members = member_indices;
            std::stable_sort(
                ranked_members.begin(), ranked_members.end(),
                [&](size_t lhs, size_t rhs) {
                    const double lhs_el = elevation_map.count(
                                              result.measurements[lhs].satellite)
                                              ? elevation_map.at(
                                                    result.measurements[lhs].satellite)
                                              : -1.0;
                    const double rhs_el = elevation_map.count(
                                              result.measurements[rhs].satellite)
                                              ? elevation_map.at(
                                                    result.measurements[rhs].satellite)
                                              : -1.0;
                    if (lhs_el != rhs_el) {
                        return lhs_el > rhs_el;
                    }
                    return lhs > rhs;
                });
            const size_t ref_index = ranked_members[static_cast<size_t>(
                std::min(std::max(reference_rank, 0),
                         static_cast<int>(ranked_members.size()) - 1))];

            // Form SD: reference - satellite
            const auto& ref_row = result.measurements[ref_index];
            for (size_t idx : member_indices) {
                if (idx == ref_index) continue;
                const auto& sat_row = result.measurements[idx];
                MeasurementRow sd_row;
                sd_row.H = ref_row.H - sat_row.H;
                sd_row.residual = ref_row.residual - sat_row.residual;
                sd_row.variance = ref_row.variance + sat_row.variance;
                if (mrtklib_dd) {
                    sd_row.reference_variance = ref_row.variance;
                    sd_row.dd_covariance_block = dd_covariance_block;
                    sd_row.reference_satellite = ref_row.satellite;
                }
                sd_row.satellite = sat_row.satellite;
                sd_row.is_phase = group_key.is_phase;
                sd_row.ambiguity_fresh = sat_row.ambiguity_fresh;
                sd_row.freq_index = group_key.freq_index;
                sd_row.ambiguity_index = sat_row.ambiguity_index;
                if (mrtklib_dd && ppp_internal::pppDebugEnabled()) {
                    std::cerr << std::setprecision(15)
                              << "[CLAS-DD-ROW] ref="
                              << ref_row.satellite.toString()
                              << " sat=" << sat_row.satellite.toString()
                              << " phase=" << group_key.is_phase
                              << " f=" << group_key.freq_index
                              << " v=" << sd_row.residual
                              << " ref_amb=" << ref_row.ambiguity_index
                              << " sat_amb=" << sat_row.ambiguity_index
                              << " ref_p="
                              << (ref_row.ambiguity_index >= 0
                                      ? filter_state.covariance(
                                            ref_row.ambiguity_index,
                                            ref_row.ambiguity_index)
                                      : -1.0)
                              << " sat_p="
                              << (sat_row.ambiguity_index >= 0
                                      ? filter_state.covariance(
                                            sat_row.ambiguity_index,
                                            sat_row.ambiguity_index)
                                      : -1.0)
                              << " Ri=" << ref_row.variance
                              << " Rj=" << sat_row.variance << '\n';
                }
                sd_measurements.push_back(sd_row);
            }
            if (mrtklib_dd) {
                ++dd_covariance_block;
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

    // MRTKLIB ddcov() (mrtk_ppp_rtk.c:914-927): all DD rows in a
    // system/frequency/type block share the reference-satellite error Ri,
    // while only each diagonal adds its target error Rj. Keep this strictly
    // parity-gated so legacy CLAS SD rows retain their diagonal covariance.
    if (clasMrtklibFloatParity(config)) {
        for (int i = 0; i < stats.nobs; ++i) {
            const auto& row_i = measurements[static_cast<size_t>(i)];
            if (row_i.dd_covariance_block < 0) {
                continue;
            }
            for (int j = i + 1; j < stats.nobs; ++j) {
                const auto& row_j = measurements[static_cast<size_t>(j)];
                if (row_j.dd_covariance_block != row_i.dd_covariance_block) {
                    continue;
                }
                R(i, j) = row_i.reference_variance;
                R(j, i) = row_i.reference_variance;
            }
        }
    }

    const bool mrtklib_parity = clasMrtklibFloatParity(config);
    MatrixXd S;
    if (mrtklib_parity) {
        // MRTKLIB filter2() compacts the state first, then filter2_() forms
        // Q=H'*P*H+R and runs residual_test().  Inactive zero-valued states
        // can still have a positive covariance in the native full matrix;
        // allowing those states into S changes which DD observations survive
        // the prefit gate (and consequently which ambiguities reach PAR).
        std::vector<int> gate_active;
        gate_active.reserve(static_cast<size_t>(filter_state.total_states));
        for (int i = 0; i < filter_state.total_states; ++i) {
            if (filter_state.state(i) != 0.0 &&
                filter_state.covariance(i, i) > 0.0) {
                gate_active.push_back(i);
            }
        }
        const int gate_states = static_cast<int>(gate_active.size());
        MatrixXd H_gate(stats.nobs, gate_states);
        MatrixXd P_gate(gate_states, gate_states);
        for (int i = 0; i < gate_states; ++i) {
            const int source_i = gate_active[static_cast<size_t>(i)];
            H_gate.col(i) = H.col(source_i);
            for (int j = 0; j < gate_states; ++j) {
                P_gate(i, j) = filter_state.covariance(
                    source_i, gate_active[static_cast<size_t>(j)]);
            }
        }
        S = H_gate * P_gate * H_gate.transpose() + R;

        // MRTKLIB v0.5.1 filter2_()/residual_test() prefit stage
        // (mrtk_ppp_rtk.c:955-1185): reject against the full innovation
        // covariance H'PH+R, compact the surviving rows, and do not update
        // when every actual DD observation was rejected. clas.toml uses
        // l1_l2_residual=2 sigma; D2 permits 10x for a newly initialized
        // phase bias (P == 1e4 m^2).
        std::vector<int> accepted;
        int accepted_dd_observations = 0;
        std::set<int> pair_rejected_rows;
        std::map<SatelliteId, std::array<int, 2>> phase_pair_rows;
        for (int i = 0; i < stats.nobs; ++i) {
            const auto& row = measurements[static_cast<size_t>(i)];
            if (!row.is_phase || row.freq_index < 0 || row.freq_index > 1) {
                continue;
            }
            auto result = phase_pair_rows.try_emplace(
                row.satellite, std::array<int, 2>{-1, -1});
            result.first->second[static_cast<size_t>(row.freq_index)] = i;
        }
        constexpr double kF1F2 = 1575.42e6 / 1227.60e6;
        constexpr double kGamma = kF1F2 * kF1F2;
        for (const auto& [satellite, rows] : phase_pair_rows) {
            (void)satellite;
            if (rows[0] < 0 || rows[1] < 0) {
                continue;
            }
            const double v0 = z(rows[0]);
            const double v1 = z(rows[1]);
            const double dispersive =
                kF1F2 * (v0 - v1) / (1.0 - kGamma);
            const double nondispersive =
                (kGamma * v0 - v1) / (kGamma - 1.0);
            const double variance = std::max(S(rows[0], rows[0]),
                                             S(rows[1], rows[1]));
            if (variance > 0.0 &&
                (dispersive * dispersive > 9.0 * variance ||
                 nondispersive * nondispersive > 9.0 * variance)) {
                pair_rejected_rows.insert(rows[0]);
                pair_rejected_rows.insert(rows[1]);
            }
        }
        for (int i = 0; i < stats.nobs; ++i) {
            const auto& row = measurements[static_cast<size_t>(i)];
            if (row.freq_index < 0) {
                accepted.push_back(i);  // native-only atmosphere constraints
                continue;
            }
            double threshold = 2.0;
            if (row.is_phase && row.ambiguity_fresh) {
                threshold *= 10.0;
            }
            const bool row_accepted =
                pair_rejected_rows.count(i) == 0 &&
                S(i, i) > 0.0 &&
                z(i) * z(i) < S(i, i) * threshold * threshold;
            if (ppp_internal::pppDebugEnabled() && row.is_phase &&
                (!row_accepted ||
                 row.satellite.system == GNSSSystem::QZSS)) {
                std::cerr << "[CLAS-PHASE-GATE] tow="
                          << (seed_solution != nullptr
                                  ? seed_solution->time.tow
                                  : -1.0)
                          << " sat="
                          << row.satellite.toString()
                          << " f=" << row.freq_index
                          << " v=" << z(i)
                          << " S=" << S(i, i)
                          << " th=" << threshold
                          << " pair_rej=" << pair_rejected_rows.count(i)
                          << " accepted=" << row_accepted << '\n';
            }
            if (row_accepted) {
                accepted.push_back(i);
                ++accepted_dd_observations;
            }
            if (row.is_phase && !row_accepted) {
                const uint8_t ambiguity_prn =
                    row.freq_index == 0
                        ? row.satellite.prn
                        : static_cast<uint8_t>(std::min(
                              255, static_cast<int>(row.satellite.prn) + 100));
                stats.rejected_phase_ambiguities.insert(
                    SatelliteId(row.satellite.system, ambiguity_prn));
            }
        }
        if (accepted_dd_observations == 0) {
            if (ppp_internal::pppDebugEnabled()) {
                std::cerr << "[CLAS-FILTER2] all DD measurements rejected total="
                          << stats.nobs << "\n";
            }
            stats.nobs = 0;
            return stats;
        }
        if (ppp_internal::pppDebugEnabled()) {
            std::cerr << "[CLAS-FILTER2] accepted_dd="
                      << accepted_dd_observations
                      << " total=" << stats.nobs << "\n";
        }
        if (accepted.size() != static_cast<size_t>(stats.nobs)) {
            const int kept = static_cast<int>(accepted.size());
            MatrixXd kept_H(kept, filter_state.total_states);
            VectorXd kept_z(kept);
            MatrixXd kept_R(kept, kept);
            for (int i = 0; i < kept; ++i) {
                kept_H.row(i) = H.row(accepted[static_cast<size_t>(i)]);
                kept_z(i) = z(accepted[static_cast<size_t>(i)]);
                for (int j = 0; j < kept; ++j) {
                    kept_R(i, j) = R(accepted[static_cast<size_t>(i)],
                                     accepted[static_cast<size_t>(j)]);
                }
            }
            H = std::move(kept_H);
            z = std::move(kept_z);
            R = std::move(kept_R);
            stats.nobs = kept;
            S = H * filter_state.covariance * H.transpose() + R;
        }
    } else {
        for (int i = 0; i < stats.nobs; ++i) {
            const double sigma = std::sqrt(R(i, i));
            if (std::abs(z(i)) > config.clas_outlier_sigma_scale * sigma) {
                R(i, i) = 1e10;
            }
        }
        S = H * filter_state.covariance * H.transpose() + R;
    }
    if (mrtklib_parity) {
        // MRTKLIB stores phase-bias states in cycles and applies wavelength
        // coefficients in H. Native stores the same states in metres. Work in
        // MRTKLIB's cycle coordinates for the parity update, then transform
        // the posterior back to metres. Although this is an exact similarity
        // transform algebraically, doing the inversion in metre coordinates
        // changes Qb by several percent at fresh 1e4-cycle states.
        VectorXd state_scale = VectorXd::Ones(filter_state.total_states);
        for (const auto& [satellite, state_index] :
             filter_state.ambiguity_indices) {
            const auto wavelength_it =
                filter_state.ambiguity_wavelengths_m.find(satellite);
            if (state_index >= 0 && state_index < filter_state.total_states &&
                wavelength_it != filter_state.ambiguity_wavelengths_m.end() &&
                wavelength_it->second > 0.0) {
                state_scale(state_index) = wavelength_it->second;
            }
        }
        const VectorXd inverse_scale = state_scale.cwiseInverse();
        const VectorXd state_cycles =
            inverse_scale.array() * filter_state.state.array();

        // MRTKLIB filter2() (mrtk_ppp_rtk.c:1210-1244) compacts x/P/H to
        // states satisfying x[i] != 0 && P[i,i] > 0 before filter2_(), then
        // copies only that posterior submatrix back.  Preserve that exact
        // lifecycle here: zero-valued, not-yet-initialized states must not
        // enter H'PH or acquire cross-covariance through this update.
        std::vector<int> active;
        active.reserve(static_cast<size_t>(filter_state.total_states));
        for (int i = 0; i < filter_state.total_states; ++i) {
            if (state_cycles(i) != 0.0 &&
                filter_state.covariance(i, i) > 0.0) {
                active.push_back(i);
            }
        }
        const int na = static_cast<int>(active.size());
        if (ppp_internal::pppDebugEnabled()) {
            std::cerr << "[CLAS-FILTER2] active_states=" << na
                      << " total_states=" << filter_state.total_states << '\n';
        }
        if (na == 0) {
            stats.nobs = 0;
            return stats;
        }
        VectorXd x_active(na);
        MatrixXd P_active(na, na);
        MatrixXd H_active(stats.nobs, na);
        for (int i = 0; i < na; ++i) {
            const int source_i = active[static_cast<size_t>(i)];
            x_active(i) = state_cycles(source_i);
            H_active.col(i) = H.col(source_i) * state_scale(source_i);
            for (int j = 0; j < na; ++j) {
                const int source_j = active[static_cast<size_t>(j)];
                P_active(i, j) =
                    filter_state.covariance(source_i, source_j) *
                    inverse_scale(source_i) * inverse_scale(source_j);
            }
        }
        const MatrixXd innovation =
            H_active * P_active * H_active.transpose() + R;
        const MatrixXd K =
            P_active * H_active.transpose() * innovation.inverse();
        const VectorXd dx_active = K * z;
        const MatrixXd posterior =
            (MatrixXd::Identity(na, na) - K * H_active) * P_active;
        x_active += dx_active;

        VectorXd dx = VectorXd::Zero(filter_state.total_states);
        for (int i = 0; i < na; ++i) {
            const int target_i = active[static_cast<size_t>(i)];
            filter_state.state(target_i) =
                x_active(i) * state_scale(target_i);
            dx(target_i) = dx_active(i) * state_scale(target_i);
            for (int j = 0; j < na; ++j) {
                const int target_j = active[static_cast<size_t>(j)];
                filter_state.covariance(target_i, target_j) =
                    posterior(i, j) * state_scale(target_i) *
                    state_scale(target_j);
            }
        }
        stats.dx = dx;
    } else {
        const MatrixXd K =
            filter_state.covariance * H.transpose() * S.inverse();
        const VectorXd dx = K * z;
        filter_state.state += dx;
        const MatrixXd I_KH =
            MatrixXd::Identity(filter_state.total_states,
                               filter_state.total_states) - K * H;
        filter_state.covariance =
            I_KH * filter_state.covariance * I_KH.transpose() +
            K * R * K.transpose();
        stats.dx = dx;
    }

    stats.updated = true;
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

bool mrtklibPostfitRowsValid(
    const ppp_shared::PPPState& filter_state,
    const std::vector<MeasurementRow>& measurements,
    const std::vector<MeasurementRow>& prefit_linearization_measurements,
    std::set<SatelliteId>* accepted_l1_phase_satellites = nullptr,
    std::set<SatelliteId>* accepted_phase_ambiguities = nullptr,
    bool debug_enabled = false) {
    if (measurements.empty()) {
        return false;
    }
    const int nobs = static_cast<int>(measurements.size());
    MatrixXd H = MatrixXd::Zero(nobs, filter_state.total_states);
    VectorXd residuals = VectorXd::Zero(nobs);
    MatrixXd R = MatrixXd::Zero(nobs, nobs);
    const bool reuse_prefit_linearization =
        prefit_linearization_measurements.size() == measurements.size();
    for (int i = 0; i < nobs; ++i) {
        const auto& row = measurements[static_cast<size_t>(i)];
        // ppp_rtk_pos() recomputes postfit DD residuals with H=NULL, then
        // residual_test(stage=1) deliberately reuses the H produced by the
        // prefit ddres() call. Only v and R are postfit quantities.
        H.row(i) = reuse_prefit_linearization
            ? prefit_linearization_measurements[static_cast<size_t>(i)].H
            : row.H;
        residuals(i) = row.residual;
        R(i, i) = row.variance;
    }
    for (int i = 0; i < nobs; ++i) {
        const auto& row_i = measurements[static_cast<size_t>(i)];
        if (row_i.dd_covariance_block < 0) {
            continue;
        }
        for (int j = i + 1; j < nobs; ++j) {
            const auto& row_j = measurements[static_cast<size_t>(j)];
            if (row_j.dd_covariance_block == row_i.dd_covariance_block) {
                R(i, j) = row_i.reference_variance;
                R(j, i) = row_i.reference_variance;
            }
        }
    }
    const MatrixXd Q = H * filter_state.covariance * H.transpose() + R;
    int total_phase_rows = 0;
    int accepted_phase_rows = 0;
    double normalized_sum = 0.0;
    std::set<int> pair_rejected_rows;
    std::map<SatelliteId, std::array<int, 2>> phase_pair_rows;
    for (int i = 0; i < nobs; ++i) {
        const auto& row = measurements[static_cast<size_t>(i)];
        if (!row.is_phase || row.freq_index < 0 || row.freq_index > 1) {
            continue;
        }
        auto result = phase_pair_rows.try_emplace(
            row.satellite, std::array<int, 2>{-1, -1});
        result.first->second[static_cast<size_t>(row.freq_index)] = i;
    }
    constexpr double kF1F2 = 1575.42e6 / 1227.60e6;
    constexpr double kGamma = kF1F2 * kF1F2;
    for (const auto& [satellite, rows] : phase_pair_rows) {
        (void)satellite;
        if (rows[0] < 0 || rows[1] < 0) {
            continue;
        }
        const double v0 = residuals(rows[0]);
        const double v1 = residuals(rows[1]);
        const double dispersive = kF1F2 * (v0 - v1) / (1.0 - kGamma);
        const double nondispersive =
            (kGamma * v0 - v1) / (kGamma - 1.0);
        const double variance = std::max(Q(rows[0], rows[0]), Q(rows[1], rows[1]));
        if (variance > 0.0 &&
            (dispersive * dispersive > 9.0 * variance ||
             nondispersive * nondispersive > 9.0 * variance)) {
            if (debug_enabled) {
                std::cerr << "[CLAS-POSTFIT-PAIR] sat="
                          << satellite.toString() << " v0=" << v0
                          << " v1=" << v1 << " Smax=" << variance
                          << " dispersive=" << dispersive
                          << " nondispersive=" << nondispersive
                          << " rejected=1\n";
            }
            pair_rejected_rows.insert(rows[0]);
            pair_rejected_rows.insert(rows[1]);
        }
    }
    const auto ambiguity_satellite = [](const SatelliteId& satellite,
                                        int freq_index) {
        return SatelliteId(
            satellite.system,
            static_cast<uint8_t>(std::min(
                255, static_cast<int>(satellite.prn) +
                         (freq_index == 0 ? 0 : 100))));
    };
    // ddres() marks both ends of every phase DD as valid before residual_test().
    // residual_test() then clears only sat2 (the non-reference satellite) when
    // either the paired-frequency or individual innovation gate rejects a row.
    // Preserve that asymmetric vsat lifecycle: the reference satellite must not
    // acquire an outage merely because one of its DD targets was rejected.
    for (const auto& row : measurements) {
        if (!row.is_phase || row.freq_index < 0) {
            continue;
        }
        if (accepted_l1_phase_satellites != nullptr && row.freq_index == 0) {
            accepted_l1_phase_satellites->insert(row.reference_satellite);
            accepted_l1_phase_satellites->insert(row.satellite);
        }
        if (accepted_phase_ambiguities != nullptr) {
            accepted_phase_ambiguities->insert(
                ambiguity_satellite(row.reference_satellite, row.freq_index));
            accepted_phase_ambiguities->insert(
                ambiguity_satellite(row.satellite, row.freq_index));
        }
    }
    for (int i = 0; i < nobs; ++i) {
        const auto& row = measurements[static_cast<size_t>(i)];
        if (!row.is_phase || row.freq_index < 0) {
            continue;
        }
        ++total_phase_rows;
        double threshold = 2.0;
        if (row.ambiguity_fresh) {
            threshold *= 10.0;
        }
        if (pair_rejected_rows.count(i) == 0 && Q(i, i) > 0.0 &&
            residuals(i) * residuals(i) <
                threshold * threshold * Q(i, i)) {
            ++accepted_phase_rows;
            normalized_sum += residuals(i) * residuals(i) / Q(i, i);
        } else {
            if (accepted_l1_phase_satellites != nullptr &&
                row.freq_index == 0) {
                accepted_l1_phase_satellites->erase(row.satellite);
            }
            if (accepted_phase_ambiguities != nullptr) {
                accepted_phase_ambiguities->erase(
                    ambiguity_satellite(row.satellite, row.freq_index));
            }
            if (debug_enabled && row.freq_index == 0) {
                std::cerr << "[CLAS-POSTFIT-GATE] sat="
                          << row.satellite.toString() << " v=" << residuals(i)
                          << " S=" << Q(i, i) << " threshold=" << threshold
                          << " pair_rejected="
                          << (pair_rejected_rows.count(i) != 0) << "\n";
            }
        }
    }
    // MRTKLIB residual_test(): when phase rows do not exceed the nine PVA
    // states, postfit succeeds only if at least half survive the innovation
    // gate. With more rows it records a normalized chi-square; the outer
    // reference loop retries only on the sentinel value 100.
    if (accepted_phase_rows <= 9) {
        return total_phase_rows > 0 &&
               2 * accepted_phase_rows >= total_phase_rows;
    }
    const double limit = claslibChiSquare001ForDof(accepted_phase_rows - 9);
    return limit > 0.0 && normalized_sum / limit < 100.0;
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
    const Vector3d state_position_before_update =
        filter_state.state.segment(filter_state.pos_index, 3);
    const Vector3d receiver_geometry_offset =
        epoch_context.receiver_position - state_position_before_update;
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

    const ppp_shared::PPPState pre_update_state = filter_state;
    std::set<SatelliteId> postfit_accepted_phase_ambiguities;
    result.update_stats = applyMeasurementUpdate(
        filter_state, measurement_build_result.measurements, config, &seed_solution);
    if (!result.update_stats.updated) {
        return result;
    }
    if (clasMrtklibFloatParity(config)) {
        std::set<SatelliteId> accepted_l1_phase_satellites;
        ppp_shared::PPPState postfit_probe_state = filter_state;
        const Vector3d postfit_position =
            filter_state.state.segment(filter_state.pos_index, 3) +
            receiver_geometry_offset;
        const double postfit_trop =
            filter_state.trop_index >= 0 &&
                    filter_state.trop_index < filter_state.total_states
                ? filter_state.state(filter_state.trop_index)
                : epoch_context.trop_zenith_m;
        const auto postfit_build = buildEpochMeasurements(
            obs,
            epoch_context.osr_corrections,
            postfit_probe_state,
            config,
            postfit_position,
            epoch_context.receiver_clock_m,
            postfit_trop,
            epoch_context.epoch_atmos_tokens,
            trop_mapping_function,
            AmbiguityResetFunction{},
            debug_enabled,
            0);
        if (!mrtklibPostfitRowsValid(
                filter_state, postfit_build.measurements,
                measurement_build_result.measurements,
                &accepted_l1_phase_satellites,
                &postfit_accepted_phase_ambiguities,
                debug_enabled)) {
            filter_state = pre_update_state;
            auto retry_build = buildEpochMeasurements(
                obs,
                epoch_context.osr_corrections,
                filter_state,
                config,
                epoch_context.receiver_position,
                epoch_context.receiver_clock_m,
                epoch_context.trop_zenith_m,
                epoch_context.epoch_atmos_tokens,
                trop_mapping_function,
                AmbiguityResetFunction{},
                debug_enabled,
                1);
            result.update_stats = applyMeasurementUpdate(
                filter_state, retry_build.measurements, config, &seed_solution);
            bool retry_valid = result.update_stats.updated;
            if (retry_valid) {
                accepted_l1_phase_satellites.clear();
                postfit_accepted_phase_ambiguities.clear();
                ppp_shared::PPPState retry_probe_state = filter_state;
                const Vector3d retry_position =
                    filter_state.state.segment(filter_state.pos_index, 3) +
                    receiver_geometry_offset;
                const double retry_trop =
                    filter_state.trop_index >= 0 &&
                            filter_state.trop_index < filter_state.total_states
                        ? filter_state.state(filter_state.trop_index)
                        : epoch_context.trop_zenith_m;
                const auto retry_postfit = buildEpochMeasurements(
                    obs,
                    epoch_context.osr_corrections,
                    retry_probe_state,
                    config,
                    retry_position,
                    epoch_context.receiver_clock_m,
                    retry_trop,
                    epoch_context.epoch_atmos_tokens,
                    trop_mapping_function,
                    AmbiguityResetFunction{},
                    debug_enabled,
                    1);
                retry_valid = mrtklibPostfitRowsValid(
                    filter_state, retry_postfit.measurements,
                    retry_build.measurements,
                    &accepted_l1_phase_satellites,
                    &postfit_accepted_phase_ambiguities,
                    debug_enabled);
            }
            if (!retry_valid) {
                filter_state = pre_update_state;
                result.update_stats = {};
                if (debug_enabled) {
                    std::cerr << "[CLAS-FILTER2] both references rejected; rollback\n";
                }
                return result;
            }
            measurement_build_result = std::move(retry_build);
            if (debug_enabled) {
                std::cerr << "[CLAS-FILTER2] second reference accepted\n";
            }
        }
        if (accepted_l1_phase_satellites.size() < 4) {
            filter_state = pre_update_state;
            result.updated = false;
            result.insufficient_valid_satellites = true;
            if (debug_enabled) {
                std::cerr << "[CLAS-FILTER2] lack of valid satellites ns="
                          << accepted_l1_phase_satellites.size()
                          << " sats=";
                for (const auto& satellite : accepted_l1_phase_satellites) {
                    std::cerr << satellite.toString() << ' ';
                }
                std::cerr << "\n";
            }
            return result;
        }

        // MRTKLIB filter2_() records Qp=(K*v)(K*v)' and, after a successful
        // float update, adapts each observed ionosphere Q diagonal using the
        // CLASLIB paper/config coefficients: forget=0.3, gain=3.0. udion()
        // clamps the result on the next epoch, not here.
        constexpr double kIonoForgetting = 0.3;
        constexpr double kIonoAdaptiveGainSquared = 3.0 * 3.0;
        const auto observed_satellite_list = obs.getSatellites();
        const std::set<SatelliteId> observed_satellites(
            observed_satellite_list.begin(), observed_satellite_list.end());
        for (const auto& satellite : observed_satellites) {
            const auto iono_it = filter_state.ionosphere_indices.find(satellite);
            if (iono_it == filter_state.ionosphere_indices.end() ||
                iono_it->second < 0 ||
                iono_it->second >= result.update_stats.dx.size()) {
                continue;
            }
            double& process_noise =
                filter_state.adaptive_ionosphere_process_noise[satellite];
            const double iono_update = result.update_stats.dx(iono_it->second);
            process_noise =
                kIonoForgetting * process_noise +
                (1.0 - kIonoForgetting) * kIonoAdaptiveGainSquared *
                    iono_update * iono_update;
        }
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
        [&]() {
            if (!clasMrtklibFloatParity(config)) {
                return measurement_build_result.observed_ambiguities;
            }
            std::vector<AmbiguityObservation> accepted;
            accepted.reserve(measurement_build_result.observed_ambiguities.size());
            for (const auto& ambiguity_obs :
                 measurement_build_result.observed_ambiguities) {
                if (postfit_accepted_phase_ambiguities.count(
                        ambiguity_obs.ambiguity_satellite) != 0) {
                    accepted.push_back(ambiguity_obs);
                }
            }
            return accepted;
        }(),
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
        ambiguity.outage_count = 0;
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
    const bool mrtklib_parity = clasMrtklibFloatParity(config);

    const Vector3d receiver_position =
        filter_state.state.segment(filter_state.pos_index, 3);
    const double trop_zenith = filter_state.state(filter_state.trop_index);
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        // The float ddres path is the source of the stage-2 fixed residual
        // set as well.  On this v0.5.1 CLAS dataset Galileo never acquires a
        // valid phase row, and QZSS contributes L1 only.
        if (mrtklib_parity &&
            osr.satellite.system == GNSSSystem::Galileo) {
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
            if (mrtklib_parity &&
                osr.satellite.system == GNSSSystem::QZSS && f > 0) {
                continue;
            }
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
            const double phase_trop_modeled = mrtklib_parity
                ? 0.0
                : (claslib_amb_datum_phase
                      ? 0.0
                      : (residual_amb_datum_phase_trop
                            ? trop_modeled - osr.trop_correction_m
                            : trop_modeled));
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

            if (!raw->has_carrier_phase || !std::isfinite(raw->carrier_phase) ||
                (mrtklib_parity &&
                 (!raw->has_pseudorange ||
                  !std::isfinite(raw->pseudorange)))) {
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
            info.elevation_rad = osr.elevation;
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
                  [mrtklib_parity](const PhaseResidualInfo& lhs,
                                   const PhaseResidualInfo& rhs) {
                      if (mrtklib_parity &&
                          lhs.elevation_rad != rhs.elevation_rad) {
                          // ddres() selects the highest-elevation valid
                          // satellite as the reference independently for each
                          // system/frequency block.
                          return lhs.elevation_rad > rhs.elevation_rad;
                      }
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
        const double pair_validation_scale = mrtklib_parity ? 9.0 : 64.0;
        if (dispersive * dispersive > pair_validation_scale * max_variance ||
            nondispersive * nondispersive > pair_validation_scale * max_variance) {
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
